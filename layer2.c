/**********************************************************************************************
*
* Faller Car Controller - NRF52 Firmware
*
* (c) 2016
* Lukas Schrittwieser
*
* all rights reserved
*
* Layer 2: position estimation, reference speed generation
*
**********************************************************************************************/

#include <stdio.h>
#include "layer2.h"
#include "layer1.h"
#include "nuts_bolts.h"
#include "lococo.h"
#include "hw.h"
#include "main.h"
// nordic bsp/lib
#include "app_timer.h"


/**********************************************************************************************
*   D E F I N E S                                                                            */

// max number of scheduled speed ramps
#define N_RAMPS     8

// define max number of scheduled tasks
#define N_TASKS     16



/**********************************************************************************************
*   G L O B A L S                                                                            */

// set after execution of speed controller to signal we should update our speed ref value
volatile bool execute_l2 = false;

// scheduled speed ramps for acceleration / deceleration in order of application
static struct b_line ramps[N_RAMPS];

// number of scheduled speed ramps
static int n_ramps = 0;

// scheduled position dependent tasks
static struct p_task p_tasks[N_TASKS];

// number of scheduled tasks
static int n_tasks = 0;

// gain factor of position estimation in cm per accu LSB divided by 2^24
// max allowed value is 16 bits (32767)
static int cm_per_lsb = 1100;

// current reference speed in mm/s
static int ref_speed = 0;

// current vehicle position in cm
static int32_t pos = 0;

// low pass filter for battery voltage
static lpf1_t u_bat_flt;

APP_TIMER_DEF(l2_timer);



/**********************************************************************************************
*   L O C A L   P R O T O T Y P E S                                                          */

void set_speed(unsigned int s);

static void task_stop(struct p_task *t);

static void task_run(struct p_task *t);



/**********************************************************************************************
*   I M P L E M E N T A T I O N                                                              */

void l2_init()
{
    ref_speed = V_MIN;
    pos = 0;
    n_ramps = 0;
    n_tasks = 0;

    // schedule initial acceleration
    l2_sched_speed_ramp(60, 0, 30);    // 96mm/s is 30km/h, accelerate over next 30cm

    //printf("init ramp: dv:%d, sign:%d, dt:%d, v:%d\n", (int)ramps[0].delta_v, (int)ramps[0].sign, (int)ramps[0].delta_time, (int)ramps[0].v);
    lpf1Init(&u_bat_flt, F_EMF_CTRL, 2000000);  // 3rd param is time const of PT1 low pass filter in us
    u_bat_flt.y = 4000; // init high battery voltage to prevent battery empty shutdown during filter settling

    // create a timer which calls our periodic task
    uint32_t err = app_timer_create(&l2_timer, APP_TIMER_MODE_REPEATED, &l2_poll);
    APP_ERROR_CHECK(err);
    // start timer with 1ms timout
    err = app_timer_start(l2_timer, APP_TIMER_TICKS(1), NULL);
    APP_ERROR_CHECK(err);
}


// periodically called by main loop via an app timer
// (p_context is unused at the moment)
void l2_poll(void *p_context)
{
    static int log_cnt = 0;
    // update our position from the integrated back emf voltage
    uint64_t accu = l1_get_emf_pos();
    accu *= cm_per_lsb; // apply gain factor
    pos = accu >> 24;   // divide as the gain in cm per lsb is very low

    if (!execute_l2)
        return;     // nothing to do right now
    execute_l2 = false; // clear flag, we are working now, can be set again by L1 interrupt soon

    // change ref speed according to position
    if (n_ramps > 0) {
         // check if ramp has started
         if (ramps[0].start_pos <= pos) {
            // calc next point in the ramp
            if (get_next_ramp_pt(&ramps[0]) == 0) {
                // this ramp is done, remove it from the array
                n_ramps--;
                for (int i=0; i<n_ramps; i++)
                    ramps[i] = ramps[i+1];
                printf("ramp done, %d remaining\n",n_ramps);
            }
            log_cnt++;
            if (log_cnt == F_EMF_CTRL/2) {
                //printf("p:%d  v:%d  t:%d\n", (int)pos, (int)ramps[0].v, (int)ramps[0].time*1000/F_EMF_CTRL);
                log_cnt = 0;
            }

            // update ref speed, this will pass the correct emf ref voltage to layer 1
            set_speed(ramps[0].v);
        }
    }

    // check position dependent tasks
    for (int i=0; i<n_tasks; i++) {
        if (p_tasks[i].start > pos)
            break;  // this (and all following, because p_tasks is sorted) starts in the future
        // check the task's end
        if (p_tasks[i].stop <= pos) {
            task_stop(&p_tasks[i]);  // finish up
            // move other tasks down in the array (no gaps)
            n_tasks--;
            for (int j=i; j<n_tasks; j++)
                p_tasks[j] = p_tasks[j+1];
            continue;
        }
        //printf("running task %d\n", i);
        task_run(&p_tasks[i]);
    }

    lpf1Update(&u_bat_flt, l1_get_u_bat());
}


// returns current vehicle position in cm
int32_t l2_get_pos()
{
    return pos;
}


// sets a new absolute vehicle position in cm and updates the underlying emf accumulator in layer 1
// returns the accumulator delta (as calling function might want to update internal deltas)
int64_t l2_set_pos(int p)
{
    // convert position in cm to emf accu lsb, inverse operation to what we do during poll function above
    uint64_t accu = p;
    accu <<= 24;
    accu = accu / cm_per_lsb; // apply gain factor
    if (accu > 0xFFFFFFFF)
        accu = 0xFFFFFFFF;  // limit value to 32 bits
    uint64_t delta_emf = accu - l1_get_emf_pos();
    l1_set_emf_pos(accu);


    int delta = p - pos;    // change in absolute position

    // we have to adapt the positions of scheduled tasks and speed ramps
    for (int i=0; i<n_ramps; i++){
        ramps[i].start_pos += delta;
    }

    for (int i=0; i<n_tasks; i++){
        p_tasks[i].start += delta;
        p_tasks[i].stop += delta;
    }

    pos = p;

    return (delta_emf);
}


uint32_t l2_get_u_bat()
{
    return (uint32_t)u_bat_flt.y;
}


// update position estimator based on measured vehicle positions
// tag: meta info about received message
// cm_delta: traveled distance in cm since previous message, max 16 bit values
void l2_update_pos_est (const struct lococo_tag* tag, int cm_delta)
{
    if (cm_delta < 0)
        cm_delta = -cm_delta;
    if (cm_delta > ((1<<16)-2))
        return;

    // this is not done very often, we can do this with floats
    float gain = ((float)cm_delta) / tag->delta_pos;    // delta pos is in back emf accu LSBs
    int new_cm_per_lsb = (int)(gain * (1<<24));

    printf("new pos const: %d\n", new_cm_per_lsb);
    // TODO: update estimate with low pass filter and potentially save result
}


// schedule position dependent tasks (such as turn lights, etc)
// this is not thread safe, call it only from the main loop, not an interrupt handler
// returns 0 on success, error code otherwise
int l2_sched_pos_task (const struct p_task *t)
{
    if (n_tasks >= N_TASKS) {
        printf("can't schedule task: no struct left\n");
        return 1;
    }
    if (t->stop <= t->start) {
        printf("task stop before start\n");
        return 2;
    }
    // keep tasks ordered ascending by position
    int ind = 0;
    for (int i=n_tasks-1; i>=0; i--) {
        if (p_tasks[i].start < t->start) {
            // task i starts before the new one, so insert right after it
            ind = i+1;
            break;
        } else {
            // move task further down the list
            p_tasks[i+1] = p_tasks[i];
        }
    }
    //printf("adding task at index %d\n",ind);
    p_tasks[ind] = *t;
    n_tasks++;
    return 0;
}


// set a new reference speed and the time when it should be reached
// @param speed		Desired vehicle speed in mm/s
// @param start_pos position in cm at which this ramp should start
// @param dist      distance over which to accelerate in cm
// @return			0 on success, other values if an error is found
int l2_sched_speed_ramp (int speed, int start_pos, int dist)
{
    if (speed > V_MAX) {
        printf("speed to high\n");
		return 1;
    }

    // make sure we don't stop completely, otherwise we can't start again (should improve this)
    if (speed < V_MIN) {
        speed = V_MIN;
    }

    if (n_ramps >= N_RAMPS) {
        printf("no entry left to schedule speed ramp\n");
        return 2;
	}

    if (dist < 0) {
        return 3;
    }

	// find speed at which this speed ramp starts and at which position we go in the list of scheduled
	// ramps
	int s_speed = ref_speed;    // assume we start with our current vehicle speed
	int ind = n_ramps;    // index 0 is first in list (if there are none
	for (int i=n_ramps-1; i>=0; i--) {
        if (ramps[i].start_pos < start_pos) {
            ind = i+1;  // new ramp will be after this one
            // calculate final speed of this ramp
            s_speed = ramps[i].v;
            if (ramps[i].sign >= 1)
                s_speed += ramps[i].delta_v;
            else
                s_speed -= ramps[i].delta_v;
            break;  // found the entry, can stop searching
        } else {
            // move this entry further down the list, we start earlier than this
            ramps[i+1] = ramps[i];
        }
	}
    //printf("inserting ramp at %d, s_speed: %d\n", ind, (int)s_speed);
    ramps[ind].delta_v = speed - s_speed;
    //printf("delta_v: %d\n", (int)(ramps[ind].delta_v));

	if (ramps[ind].delta_v < 0)
	{
		// we have to deccelerate, however the algorithm needs a pos deltaRef (flip to first octant)
		ramps[ind].delta_v = -ramps[ind].delta_v;
		// remember that we actually go in the negative direction
		ramps[ind].sign = -1;
	}
	else
		ramps[ind].sign = 1;

    // calc. how long it will take to accelerate (t = 2*s/(2v0 + delta_v) )
    // in speed controller execution cycles (F_EMF_CTRL)
    int delta_t = 2 * 10 * dist * F_EMF_CTRL / (2*s_speed + ramps[ind].delta_v);

	ramps[ind].delta_time = delta_t;
	ramps[ind].cnt = delta_t;
	// init bresenham error at 0.5
	ramps[ind].err = dist/2;
    ramps[ind].v = s_speed; // start speed at beginning of ramp
    ramps[ind].start_pos = start_pos;
    ramps[ind].time = 0;    // starting time not yet known

	n_ramps++;  // we have one more ramp scheduled now

	//printf("ramp %d: dv: %d, pos: %d\n", ind, (int)ramps[ind].delta_v, (int)ramps[ind].start_pos);
	return 0;
}

// TODO: use floats instead of fixed point arithmetics for speed calculations

// set new speed ref in mm/s
void set_speed(unsigned int s)
{
    // we can max tolerate a 8 bit number for the speed (numerics)
    if (s > V_MAX)
        s = V_MAX;    // this is 80km/h in H0 scale
    ref_speed = s;  // remember value
    uint32_t emf = s;    // calc required back emf value
    emf <<= 24;
    // divide by mm_per_lsb ratio to get lsb, need to take accu update frq into account
    emf /= (10*F_EMF_CTRL*cm_per_lsb);
    l1_set_emf_ref(emf);
}


// returns the measured vehicle speed in mm/s
int32_t l2_get_speed()
{
    int32_t emf = l1_get_emf();
    int32_t s = (emf * 10*F_EMF_CTRL * cm_per_lsb) >> 24;
    if (s < 0)  // TODO: direction reversal
        s = 0;
    return s;
}


// perform final actions of a task once the stop position has been reached
static void task_stop(struct p_task *t)
{
    switch(t->id)
    {
        case T_TURN_LEFT:
            hw_set_led(2, 0);
            break;
        case T_TURN_RIGHT:
            hw_set_led(3, 0);
            break;
    }
}


static void task_run(struct p_task *t)
{
    uint32_t tick = hw_get_tick();

    switch(t->id)
    {
        case T_TURN_LEFT:
            /*if (t->data == 0)
                printf("starting turn left\n");*/
            // flash LED based on tick timer
            if (t->data <= tick) {
                t->data = tick + 200;   // flash with ~1Hz
                hw_toggle_led(2);
            }
            break;
        case T_TURN_RIGHT:
            /*if (t->data == 0)
                printf("starting turn right\n");*/
            // flash LED based on tick timer
            if (t->data <= tick) {
                t->data = tick + 200;   // flash
                hw_toggle_led(3);
            }
            break;
    }
}
