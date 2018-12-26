/**********************************************************************************************
*
* Faller Car Controller - NRF52 Firmware
*
* (c) 2016
* Lukas Schrittwieser
*
* all rights reserved
*
* LoCo3: LoCo3 message reception and parsing
*
**********************************************************************************************/


/**********************************************************************************************
*   I N C L U D E S                                                                          */

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include "boards.h"
#include "nrf_nvic.h"
#include "app_timer.h"
#include "hw.h"
#include "lococo.h"
#include "layer1.h"
#include "layer2.h"
#include "main.h"



/**********************************************************************************************
*   D E F I N E S                                                                            */

#define MAX_LEN     50

// threshold to detect messages from a new station (this gives the min. time between to
// stations) in ms
#define NEW_MSG_THRESHOLD   200

// if time between two bytes is larger than this (in ms) we consider it to be the beginning of
// a new message
#define MSG_TIMEOUT     4


#define CMD_OP_POS      0
#define CMD_OP_DIST     1
#define CMD_OP_V_IN     2
#define CMD_OP_V_FROM   3
#define CMD_OP_TURN_LEFT    4
#define CMD_OP_TURN_RIGHT   5
#define CMD_OP_BAT_STOP     6



/**********************************************************************************************
*   S T R U C T S                                                                            */

union param {
    uint16_t    u16;
    uint8_t     u8[2];
};

// struct of commands inside a lococo message
struct cmd {
    uint8_t     op;     // opcode
    union param val;    // value, depends on opcode
} __attribute__((packed));



/**********************************************************************************************
*   G L O B A L S                                                                            */


uint8_t uart1_rx_buf[1];

volatile uint8_t msg[MAX_LEN];
volatile int wr_ind;
volatile bool msg_ready = false;    // set to true by receiver
static volatile int msg_len = 0;

enum state_e {ST_IDLE, ST_RX, ST_CRC_1, ST_CRC_2, ST_PROCESSING};

static volatile enum state_e state = ST_IDLE;

// meta information of last received message
static struct lococo_tag msg_tag = {0};

// if we should shutdown at next stop if battery is empty
static int bat_stop = 0;

APP_TIMER_DEF(lococo_timer);



/**********************************************************************************************
*   L O C A L   P R O T O T Y P E S                                                          */

void lococo_poll(void* p_context);

static uint16_t calcCRC(uint8_t c, uint16_t crc);



/**********************************************************************************************
*   I M P L E M E N T A T I O N                                                              */

void lococo_init()
{
    msg_ready = false;
    state = ST_IDLE;

    // create a timer which calls our periodic task
    uint32_t err = app_timer_create	(&lococo_timer, APP_TIMER_MODE_REPEATED, &lococo_poll);
    APP_ERROR_CHECK(err);
    // start timer with 1ms timout
    err = app_timer_start(lococo_timer, APP_TIMER_TICKS(1), NULL);
    APP_ERROR_CHECK(err);
}


// has to be called regularly by main loop (~ every 1ms)
void lococo_poll(void* p_context)
{
    static uint32_t last_msg_tick = 0;
    enum state_e s;
    int ind = 0;
    uint32_t tick = hw_get_tick();
    uint32_t emf_pos = l1_get_emf_pos();
    struct p_task task = {0};
    int64_t emf_update = 0;

    uint8_t cs_nested;
    sd_nvic_critical_region_enter(&cs_nested);
    s = state;
    sd_nvic_critical_region_exit(cs_nested);

    if (s != ST_PROCESSING)
        return; // nothing to do while reception is running (in interrupt handler below)

    /*printf("loco2 msg: ");
    for (int i=0; i<msg_len; i++)
        printf("%02x ",msg[i]);
    return;*/

    // check that the message is considerably newer than last one to make sure we don't process
    // two consecutive messages from the same station twice ( if we move slowly)
    if ((tick-last_msg_tick) < NEW_MSG_THRESHOLD) {
        //printf("message too new\n");
        last_msg_tick = tick;
        state = ST_IDLE;
        return;
    }
    last_msg_tick = tick;

    // update meta information
    msg_tag.delta_tick = tick - msg_tag.rx_tick;    // ticks since last message we tagged
    msg_tag.rx_tick = tick;
    msg_tag.delta_pos = emf_pos - msg_tag.rx_pos;
    msg_tag.rx_pos = emf_pos;

    while (ind < msg_len) {
        if ((msg_len-ind) < sizeof(struct cmd)) {
            printf("loco2: not enough data\n");
            state = ST_IDLE;    // we are ready for the next message
            break;
        }

        struct cmd* cmd_p = (void*)(msg+ind);
        ind += sizeof(struct cmd);

        // process opcode
        switch (cmd_p->op) {
        case CMD_OP_POS: // absolute position
            emf_update = l2_set_pos(cmd_p->val.u16);    // remember how much the emf integrator was changed
            printf("loco2 pos: %d  delta emf: %lld\n", cmd_p->val.u16, emf_update);
            break;
        case CMD_OP_DIST:    // distance since last station
            printf("loco2 d_pos: %d, d_emf %d, d_t %d\n", cmd_p->val.u16, (int)msg_tag.delta_pos, (int)msg_tag.delta_tick);
            l2_update_pos_est(&msg_tag, cmd_p->val.u16);
            break;
        case CMD_OP_V_IN:     // set ref speed to be reached at point after transmitter
            printf("loco2 set v=%d in %d\n", cmd_p->val.u8[0], cmd_p->val.u8[1]);
            int dist = 15;  // ramp over a distance of 10cm fixed (TODO)
            int start_pos = l2_get_pos()+cmd_p->val.u8[1]-dist;  // start ramp at: here + offset given by transponder, minus ramp len
            l2_sched_speed_ramp(cmd_p->val.u8[0], start_pos, dist);
            break;
        case CMD_OP_V_FROM:     // set ref speed, starting ramp at point after transmitter
            printf("loco2 set v=%d from %d\n", cmd_p->val.u8[0], cmd_p->val.u8[1]);
            dist = 15;  // ramp over a distance of 10cm fixed (TODO)
            start_pos = l2_get_pos()+cmd_p->val.u8[1];  // start ramp at: here + offset given by transponder, minus ramp len
            l2_sched_speed_ramp(cmd_p->val.u8[0], start_pos, dist);
            break;
        case CMD_OP_TURN_LEFT:     // turn left
            printf("loco2 left in %d for %d\n", cmd_p->val.u8[0], cmd_p->val.u8[1]);
            task.id = T_TURN_LEFT;
            task.start = l2_get_pos() + cmd_p->val.u8[0];
            task.stop = task.start + cmd_p->val.u8[1];
            l2_sched_pos_task(&task);
            break;
        case CMD_OP_TURN_RIGHT:     // turn right
            printf("loco2 right in %d for %d\n", cmd_p->val.u8[0], cmd_p->val.u8[1]);
            task.id = T_TURN_RIGHT;
            task.start = l2_get_pos() + cmd_p->val.u8[0];
            task.stop = task.start + cmd_p->val.u8[1];
            l2_sched_pos_task(&task);
            break;
        case CMD_OP_BAT_STOP:       // tells us whether we should shutdown at next stop if our battery is almost empty
            bat_stop = cmd_p->val.u16;
            printf("bat stop: %d\n", bat_stop);
            break;
        default:
            printf("uknown opcode %d\n",cmd_p->op);
            continue;
        }

    }
    sd_nvic_critical_region_enter(&cs_nested);
    // ready for next message
    state = ST_IDLE;
    sd_nvic_critical_region_exit(cs_nested);

    // update position at which this message was received in case the vehicle position was updated by the received message
    msg_tag.rx_pos += emf_update;
}


int lococo_get_bat_stop()
{
    return bat_stop;
}


// process received data, a timeout or error condition
void lococo_rx(uint8_t byte, bool err)
{
    static uint32_t last_rx = 0;
    static uint16_t crc = 0;
    uint32_t tick = hw_get_tick();


    if (err) {
        if ((state != ST_IDLE) && (state != ST_PROCESSING)) {
            //printf("loco2: rx error, aborting\n");
            state = ST_IDLE;
        }
        return;
    }

    if (((tick-last_rx) > MSG_TIMEOUT) && (state != ST_PROCESSING)) {
        // timeout elapsed between two bytes, this must be the beginning of a new message
        //if (state != ST_IDLE)
        //    printf("loco2: timeout\n");
        state = ST_IDLE;
    }

    // process received data
    switch (state) {
    case ST_IDLE:
        // if we had no data for long enough a new message starts, otherwise it is just gibberish from the last message
        if ((tick-last_rx) >= MSG_TIMEOUT) {
            // first byte is the message length
            msg_len = byte;
            if (msg_len >= MAX_LEN) {
                //printf("loco2: received message len is to long\n");
                break;
            }
            //printf("loco2: rx len %d\n", byte);
            state = ST_RX;
            crc = calcCRC(byte, 0); // start CRC calc with 0 initial value
            wr_ind = 0;
        }
        break;

    case ST_RX:
        crc = calcCRC(byte, crc);
        msg[wr_ind++] = byte;
        //printf("loco2: rx %02x\n", byte);
        if (wr_ind == msg_len) {
            // done, message is received
            state = ST_CRC_1;
        }
        break;

    case ST_CRC_1:
        //printf("calc crc: %04x  rx: %02x\n", crc, byte);
        crc = calcCRC(byte, crc);
        state = ST_CRC_2;
        break;

    case ST_CRC_2:\
        //printf(" 2nd crc: %02x\n", byte);
        crc = calcCRC(byte, crc);
        if (crc == 0) {
            //printf("msg received\n");
           state = ST_PROCESSING;   // signal main loop to process the message
        } else {
            //printf("loco2: crc err\n");
            state = ST_IDLE;    // crc error, ignore message
        }
        break;

    case ST_PROCESSING:
        // wait until processing function is done (called by main loop)
        break;

    default:
        //printf("loco2: unknown state\n");
        state = ST_IDLE;
        break; // should not happen
    }
    last_rx = tick;
}


// 16 bit CRC algorithm
// NOTE: could use cpu's CRC core to do this -> higher performance
static uint16_t calcCRC(uint8_t c, uint16_t crc)
{
	uint8_t i;
	for (i=0; i<8; i++)
	{
		if (crc & 0x8000)
		{
			crc <<= 1;
			crc |= ((c&0x80)?1:0);
			crc ^= 0x1021;
		}
		else
		{
			crc <<= 1;
			crc |= ((c&0x80)?1:0);
		}
		c<<=1;
	}
	return crc;
}
