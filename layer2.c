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
#include "model_car_ble_service.h"
// nordic bsp/lib
#include "app_timer.h"
#include "nrf_log.h"
#include "vehicle_follower.h"
#include "advertising.h"



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

// gain factor of position estimation in cm per accu LSB 
static float cm_per_lsb = 65.5651e-6F; // empirical, needs to be updated

// current reference speed in mm/s
static volatile int ref_speed = 0;

// allowed top speed in mm/s
static uint16_t v_max = 255;

// current vehicle position in cm
static int32_t pos = 0;

// low pass filter for battery voltage
static lpf1_t u_bat_flt;

// last position change. This is updated whenever l2_set_position is called.
static uint32_t pos_delta = 0;

// 
static struct pi distance_ctrl;

static uint32_t dist = 0;

// Id of the track (road segment) on which this vehicle is currently driving
static uint8_t track_id = 0;

APP_TIMER_DEF(l2_timer);

// Gamma correction table from: http://tronicslab.com/gamma-correction-pulsing-leds/
const uint16_t l2_gamma_table[64] = 
{ 
  0,    0,    1,    2,    4,    6,    9,    13,  
  16,   21,   26,   31,   37,   44,   51,   58,  
  66,   74,   84,   93,   103,  114,  125,  136,  
  148,  161,  174,  188,  202,  217,  232,  248,  
  264,  281,  298,  316,  334,  353,  372,  392,  
  412,  433,  455,  477,  499,  522,  545,  569,  
  594,  619,  644,  670,  697,  724,  752,  780,  
  808,  837,  867,  897,  928,  959,  991,  1023  
};



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
    track_id = 0;   // 0 is the default track.

    // schedule initial acceleration
    l2_sched_speed_ramp(70, 0, 30);    // 96mm/s is 30km/h, accelerate over next 30cm

    NRF_LOG_INFO("init ramp: dv:%d, sign:%d, dt:%d, v:%d", (int)ramps[0].delta_v, (int)ramps[0].sign, (int)ramps[0].delta_time, (int)ramps[0].v);
    lpf1Init(&u_bat_flt, F_EMF_CTRL, 2000000);  // 3rd param is time const of PT1 low pass filter in us
    u_bat_flt.y = 4000; // init high battery voltage to prevent battery empty shutdown during filter settling

    // create a timer which calls our periodic task
    uint32_t err = app_timer_create(&l2_timer, APP_TIMER_MODE_REPEATED, &l2_poll);
    APP_ERROR_CHECK(err);
    // start timer with 1ms timout
    err = app_timer_start(l2_timer, APP_TIMER_TICKS(1), NULL);
    APP_ERROR_CHECK(err);

    float f_cross = 0.2;  // position controller crossover freq in Hz
    float k_p = f_cross * 2 * 3.1416F * 10; // *10 because pos is in cm but speed in mm/s
    set_pi_gains(&distance_ctrl, k_p, 0, F_EMF_CTRL);
}


// periodically called by main loop via an app timer
// (p_context is unused at the moment)
void l2_poll(void *p_context)
{
    static int log_cnt = 0;
    
    // update our position from the integrated back emf voltage
    float f_pos = l1_get_emf_pos() * cm_per_lsb;
    pos = (int32_t)f_pos;  

    if (!execute_l2) {
        return;     // nothing to do right now
    }
    execute_l2 = false; // clear flag, we are working now, can be set again by L1 interrupt soon

    // following code runs with a repetition rate of F_EMF_CTRL 

    
    int8_t requested_direction = mcs_get_direction();
       
    // change max speed according to position dependent speed ramps
    if (n_ramps > 0) {
         // check if ramp has started
         if (ramps[0].start_pos <= pos) {
            // calc next point (i.e. speed reference value) in the ramp
            if (get_next_ramp_pt(&ramps[0]) == 0) {
                // this ramp is done, remove it from the array
                n_ramps--;
                for (int i=0; i<n_ramps; i++) {
                    ramps[i] = ramps[i+1];
                }
                NRF_LOG_INFO("ramp done, %d remaining",n_ramps);
            }
            
            // update ref speed, this will pass the correct emf ref voltage to layer 1
            int32_t v = ramps[0].v;
            // check limit set by distance controller
            if (v > v_max) {
                v = v_max;
            }
        }
    }

    // distance control (to vehicle in front)
    dist = vf_get_dist_2_lead(pos);
    if (requested_direction == MCS_STOP) {
        // stop vehicle
        distance_ctrl.max = 0;
    } else {
        // keep going
        if (requested_direction == MCS_FWD) {
            l1_set_direction(true);
        } else {
            l1_set_direction(false);
        }
        distance_ctrl.max = ramps[0].v; // ramp gives our allowed max. speed
    }
    int32_t v_ref;
    const int32_t ref_distance = (int32_t)mcs_get_ref_distance(); // in cm 
    if (dist > 0) {
        // there is a vehicle in front of us, use a controller to keep a minimum distance
        v_ref = update_pi(&distance_ctrl, -(ref_distance - dist));
    } else {
        // no vehicle in front, keep updating controller with a positive distance to stay at the speed limit
        v_ref = update_pi(&distance_ctrl, ref_distance);
    }
    set_speed(v_ref);    

    log_cnt++;
    if (log_cnt == F_EMF_CTRL/2) {
        // NRF_LOG_INFO("p:%d  v:%d v_ref:%d  dist:%d", (int)pos, (int)ramps[0].v, (int)v_ref, (int)dist);
        log_cnt = 0;
    }

    // check position dependent tasks
    for (int i=0; i<n_tasks; i++) {
        if (p_tasks[i].start > pos) {
            break;  // this (and all following, because p_tasks is sorted) starts in the future
        }

        //printf("running task %d\n", i);
        task_run(&p_tasks[i]);

        // check the task's end
        if (p_tasks[i].stop <= pos) {
            task_stop(&p_tasks[i]);  // finish up
            // move other tasks down in the array (no gaps)
            n_tasks--;
            for (int j=i; j<n_tasks; j++) {
                p_tasks[j] = p_tasks[j+1];
            }
            continue;
        }
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
    float accu = p / cm_per_lsb; // apply gain factor
   
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
    pos_delta = delta;  // remeber this, can be used by other parts of the firmware

    return (delta_emf);
}


// returns how much the position (in cm) changed due to last call to l2_set_pos()
uint32_t l2_get_pos_delta()
{
    return pos_delta;
}


int32_t l2_get_dist()
{
    return dist;
}


// returns battery voltage in mV
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

    float gain = ((float)cm_delta) / tag->delta_pos;    // delta pos is in back emf accu LSBs
    int new_cm_per_lsb = (int)(gain * (1<<24));

    NRF_LOG_INFO("new pos const: %d", new_cm_per_lsb);
    // TODO: update estimate with low pass filter and potentially save result
}


// schedule position dependent tasks (such as turn lights, etc)
// this is not thread safe, call it only from the main loop, not an interrupt handler
// returns 0 on success, error code otherwise
int l2_sched_pos_task (const struct p_task *t)
{
    if (n_tasks >= N_TASKS) {
        NRF_LOG_INFO("can't schedule task: no struct left");
        return 1;
    }
    /* that's ok, task simply runns once if (t->stop <= t->start) {
        NRF_LOG_INFO("task stop before start");
        return 2;
    } */
    // keep tasks ordered, ascending by start position
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
    //NRF_LOG_INFO("adding task at index %d\n",ind);
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
    /* TODO: graceful handling
    if (speed > V_MAX) {
        NRF_LOG_ERROR("speed too high");
	return 1;
    } */

    // make sure we don't stop completely, otherwise we can't start again (should improve this)
    if (speed < V_MIN) {
        speed = V_MIN;
    }

    if (n_ramps >= N_RAMPS) {
        NRF_LOG_INFO("no entry left to schedule speed ramp");
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
    NRF_LOG_INFO("inserting ramp at %d, s_speed: %d", ind, (int)s_speed);
    ramps[ind].delta_v = speed - s_speed;
    //NRF_LOG_INFO("delta_v: %d\n", (int)(ramps[ind].delta_v));

    if (ramps[ind].delta_v < 0) {
        // we have to deccelerate, however the algorithm needs a pos deltaRef (flip to first octant)
        ramps[ind].delta_v = -ramps[ind].delta_v;
        // remember that we actually go in the negative direction
        ramps[ind].sign = -1;
    } else {
            ramps[ind].sign = 1;
    }

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
  
    NRF_LOG_INFO("ramp %d: dv: %d, pos: %d", ind, (int)ramps[ind].delta_v, (int)ramps[ind].start_pos);
    return 0;
}


// set new speed ref in mm/s
void set_speed(uint32_t s)
{
    // we can max tolerate a 8 bit number for the speed (numerics)
    if (s > v_max) {
        s = v_max;    // this is 80km/h in H0 scale
    }
    ref_speed = s;  // remember value

    // divide by mm_per_lsb ratio to get lsb, need to take accu update frq into account
    float emf = s / (10*F_EMF_CTRL*cm_per_lsb);

    l1_set_emf_ref((int32_t)emf);
}


// returns the measured vehicle speed in mm/s
int32_t l2_get_speed()
{
    int32_t emf = l1_get_emf();
    float f = emf * 10*F_EMF_CTRL * cm_per_lsb;
    int32_t s = (int32_t)f;
    if (s < 0) {  // TODO: direction reversal
        s = 0;
    }
    return s;
}


int32_t l2_get_ref_speed()
{
    return ref_speed;
}


// set allowed top speed in mm/s
void l2_set_max_speed(uint16_t v)
{
    if (v < V_MIN) {
        v = V_MIN;    // sanity checking...
    }
    v_max = v;
    // immediately reduce current speed if necessary (might be nicer to have a ramp...)
    if (ref_speed > v) {
        set_speed(v);
    }
}


// update the track id at which this vehicle is right now
void l2_set_track_id(int id)
{
    // map unkown / illegal track id values to value 0
    if (id < 0) {
        id = 0;
    }
    if (id > 255) {
        id = 0;
    }
    track_id = id;
    // tell advertising that the track id changes
    advertising_schedule_track_id_change(id);
}


uint8_t l2_get_track_id()
{
    return track_id;
}


// Set LED brightness value using gamma correction. 
// The 8 bit brightness value is mapped to 64 steps (gamma correction)
void l2_set_brigthness(uint8_t led, uint8_t value)
{
    uint32_t duty = l2_gamma_table[value>>2];
    hw_set_led_duty(led, duty);
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
                NRF_LOG_INFO("starting turn left\n");*/
            // flash LED based on tick timer
            if (t->data <= tick) {
                t->data = tick + 200;   // blink with ~1Hz
                hw_toggle_led(2);
            }
            break;
        case T_TURN_RIGHT:
            /*if (t->data == 0)
                NRF_LOG_INFO("starting turn right\n");*/
            // flash LED based on tick timer
            if (t->data <= tick) {
                t->data = tick + 200;   // flash
                hw_toggle_led(3);
            }
            break;
        case T_SET_TRACK_ID:
        {
            int id = t->data;
            NRF_LOG_INFO("l2: setting track id: %d", id);
            l2_set_track_id(id);
            break;
        }
    }
}
