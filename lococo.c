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
#include "nrf_drv_ppi.h"
#include "nrf_drv_timer.h"
#include "nrf_drv_gpiote.h"
#include "nrf_log.h"
#include "app_timer.h"
#include "app_uart.h"
#include "hw.h"
#include "lococo.h"
#include "layer1.h"
#include "layer2.h"
#include "main.h"



/**********************************************************************************************
*   D E F I N E S                                                                            */

#define UART_TX_BUF_SIZE                32      /**< UART TX buffer size. */
#define UART_RX_BUF_SIZE                256     /**< UART RX buffer size. */

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



enum state_e {ST_IDLE, ST_RX, ST_CRC_1, ST_CRC_2, ST_PROCESSING};


volatile bool m_loco3_rx_err;  // receiver error flag

// meta information of last received message
static struct lococo_tag msg_tag = {0};

// if we should shutdown at next stop if battery is empty
static int bat_stop = 0;

// LoCo3 Hardware
static const nrf_drv_timer_t m_loco3_timer = NRF_DRV_TIMER_INSTANCE(2);
static nrf_ppi_channel_t     m_loco3_ppi_channel;

APP_TIMER_DEF(lococo_timer);



/**********************************************************************************************
*   L O C A L   P R O T O T Y P E S                                                          */

void loco3_poll(void* p_context);

static void loco3_rx(uint8_t byte, bool err);

static void process_message(uint8_t* p_msg, uint32_t len, uint32_t tick);

static void loco3_uart_event_handle(app_uart_evt_t * p_event);

static uint16_t calcCRC(uint8_t c, uint16_t crc);

void loco3_timer_handler(nrf_timer_event_t event_type, void * p_context);



/**********************************************************************************************
*   I M P L E M E N T A T I O N                                                              */

void loco3_init()
{
    uint32_t ret;
  
    // create a timer which calls our periodic task
    uint32_t err = app_timer_create (&lococo_timer, APP_TIMER_MODE_REPEATED, &loco3_poll);
    APP_ERROR_CHECK(err);
    // start timer with 1ms timout
    err = app_timer_start(lococo_timer, APP_TIMER_TICKS(1), NULL);
    APP_ERROR_CHECK(err);
   
    // use a timer to create the LoCo3 carrier signal by toggling its GPIO pin on compare
    nrf_drv_timer_config_t timer_cfg = NRF_DRV_TIMER_DEFAULT_CONFIG;
    timer_cfg.bit_width = NRF_TIMER_BIT_WIDTH_32;
    ret = nrfx_timer_init(&m_loco3_timer, &timer_cfg, loco3_timer_handler);   // dummy timer handler 
    APP_ERROR_CHECK(ret);

    // setup timer to trigger compares
    uint32_t ticks = 16000000 / (2*F_LOCO3);
    nrf_drv_timer_extended_compare(&m_loco3_timer,
                                   NRF_TIMER_CC_CHANNEL0,
                                   ticks,
                                   NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK,
                                   false);
    nrf_drv_timer_enable(&m_loco3_timer);

    // setup output pin which will toggle on ppi event
    const nrf_drv_gpiote_out_config_t loco3_carrier_pin_config = GPIOTE_CONFIG_OUT_TASK_TOGGLE(true);

    ret = nrf_drv_gpiote_out_init(PIN_LOCO3_CARR, &loco3_carrier_pin_config);
    APP_ERROR_CHECK(ret);

    nrf_drv_gpiote_out_task_enable(PIN_LOCO3_CARR);


    // setup ppi channel so that timer compare event is triggered
    uint32_t timer_compare_event_addr = nrf_drv_timer_compare_event_address_get(&m_loco3_timer,
                                                                                NRF_TIMER_CC_CHANNEL0);

    uint32_t gpio_toggle_task_addr   = nrf_drv_gpiote_out_task_addr_get(PIN_LOCO3_CARR);

    ret = nrf_drv_ppi_channel_alloc(&m_loco3_ppi_channel);
    APP_ERROR_CHECK(ret);

    ret = nrf_drv_ppi_channel_assign(m_loco3_ppi_channel, timer_compare_event_addr, gpio_toggle_task_addr);
    APP_ERROR_CHECK(ret);

    // enable PPI to trigger adc on timer compare
    ret = nrf_drv_ppi_channel_enable(m_loco3_ppi_channel);
    APP_ERROR_CHECK(ret);

    // init uart
    const app_uart_comm_params_t comm_params = {
            PIN_LOCO3_RX,
            PIN_LOCO3_TX,
            UART_PIN_DISCONNECTED,  // rts
            UART_PIN_DISCONNECTED,  // cts
            APP_UART_FLOW_CONTROL_DISABLED,
            false,
            UART_BAUDRATE_BAUDRATE_Baud9600};

    // configure an uart connected to Tx and Rx FIFOs
    APP_UART_FIFO_INIT( &comm_params,
                       UART_RX_BUF_SIZE,
                       UART_TX_BUF_SIZE,
                       loco3_uart_event_handle,
                       APP_IRQ_PRIORITY_LOW,
                       ret);
    APP_ERROR_CHECK(ret);
}


// has to be called regularly by main loop (~ every 1ms)
void loco3_poll(void* p_context)
{
    uint32_t ret;
    uint8_t byte;

    // check global receiver error flag (set in uart event handler)
    if (m_loco3_rx_err) {
        m_loco3_rx_err = false;
        loco3_rx(0, true);
    }

    // process all data available in the Rx FIFO
    do {
        ret = app_uart_get(&byte);
        if (ret == NRF_SUCCESS) {
            loco3_rx(byte, false);
        }
    } while (ret == NRF_SUCCESS);
}


int loco3_get_bat_stop()
{
    return bat_stop;
}


// process received data, a timeout or error condition
static void loco3_rx(uint8_t byte, bool err)
{
    static volatile enum state_e state = ST_IDLE;
    static uint32_t last_rx = 0;
    static uint16_t crc = 0;
    static uint8_t msg[MAX_LEN];
    static uint32_t wr_ind;
    static int msg_len = 0;

    uint32_t tick = hw_get_tick();

    if (err) {
        if ((state != ST_IDLE) && (state != ST_PROCESSING)) {
            //NRF_LOG_INFO("loco3: rx error, aborting");
            state = ST_IDLE;
        }
        return;
    }

    if (((tick-last_rx) > MSG_TIMEOUT) && (state != ST_PROCESSING)) {
        // timeout elapsed between two bytes, this must be the beginning of a new message
        //if (state != ST_IDLE)
        //    NRF_LOG_INFO("loco3: timeout");
        state = ST_IDLE;
    }

    // process received data
    switch (state) {
    case ST_IDLE:
        // if we had no data for long enough, a new message starts, otherwise it is just gibberish from the last message
        if ((tick-last_rx) >= MSG_TIMEOUT) {
            // first byte is the message length
            msg_len = byte;
            if (msg_len >= MAX_LEN) {
                //NRF_LOG_INFO("loco3: received message len is to long");
                break;
            }
            //NRF_LOG_INFO("loco3: rx len %d", byte);
            state = ST_RX;
            crc = calcCRC(byte, 0); // start CRC calc with 0 initial value
            wr_ind = 0;
        }
        break;

    case ST_RX:
        crc = calcCRC(byte, crc);
        msg[wr_ind++] = byte;
        //NRF_LOG_INFO("loco3: rx %02x", byte);
        if (wr_ind == msg_len) {
            // done, message is received
            state = ST_CRC_1;
        }
        break;

    case ST_CRC_1:
        //NRF_LOG_INFO("loco3: calc crc: %04x  rx: %02x", crc, byte);
        crc = calcCRC(byte, crc);
        state = ST_CRC_2;
        break;

    case ST_CRC_2:
        //NRF_LOG_INFO("loco3: 2nd crc: %02x", byte);
        crc = calcCRC(byte, crc);
        if (crc == 0) {
            //NRF_LOG_INFO("loco3: msg received");
            process_message(msg, msg_len, tick);           
        } else { 
            //NRF_LOG_INFO("loco3: crc err");
        }
        state = ST_IDLE; 
        break;

    default:
        //NRF_LOG_ERROR("loco3: unknown state");
        state = ST_IDLE;
        break; // should not happen
    }
    last_rx = tick;
}


static void process_message(uint8_t* p_msg, uint32_t len, uint32_t tick)
{
    static uint32_t last_msg_tick = 0;
    

    int ind = 0;
    uint32_t emf_pos = l1_get_emf_pos();
    struct p_task task = {0};
    int64_t emf_update = 0;

    // check that the message is considerably newer than last one to make sure we don't process
    // two consecutive messages from the same station twice (e.g. if we move slowly)
    if ((tick-last_msg_tick) < NEW_MSG_THRESHOLD) {
        //NRF_LOG_INFO("message too new");
        last_msg_tick = tick; 
        return;
    }
    last_msg_tick = tick;

    // update meta information
    msg_tag.delta_tick = tick - msg_tag.rx_tick;    // ticks since last message we tagged
    msg_tag.rx_tick = tick;
    msg_tag.delta_pos = emf_pos - msg_tag.rx_pos;
    msg_tag.rx_pos = emf_pos;

    while (ind < len) {
        if ((len-ind) < sizeof(struct cmd)) {
            NRF_LOG_ERROR("loco3: not enough data");
            break;
        }

        struct cmd* cmd_p = (void*)(p_msg+ind);
        ind += sizeof(struct cmd);

        // process opcode
        switch (cmd_p->op) {
        case CMD_OP_POS: // absolute position
            emf_update = l2_set_pos(cmd_p->val.u16);    // remember how much the emf integrator was changed
            NRF_LOG_INFO("loco3 pos: %d  delta emf: %lld", cmd_p->val.u16, emf_update);
            break;
        case CMD_OP_DIST:    // distance since last station
            NRF_LOG_INFO("loco3 d_pos: %d, d_emf %d, d_t %d", cmd_p->val.u16, (int)msg_tag.delta_pos, (int)msg_tag.delta_tick);
            l2_update_pos_est(&msg_tag, cmd_p->val.u16);
            break;
        case CMD_OP_V_IN:     // set ref speed to be reached at point after transmitter
            NRF_LOG_INFO("loco3 set v=%d in %d", cmd_p->val.u8[0], cmd_p->val.u8[1]);
            int dist = 15;  // TODO: ramp over a distance (value in cm) 
            int start_pos = l2_get_pos()+cmd_p->val.u8[1]-dist;  // start ramp at: here + offset given by transponder, minus ramp len
            l2_sched_speed_ramp(cmd_p->val.u8[0], start_pos, dist);
            break;
        case CMD_OP_V_FROM:     // set ref speed, starting ramp at point after transmitter
            NRF_LOG_INFO("loco3 set v=%d from %d", cmd_p->val.u8[0], cmd_p->val.u8[1]);
            dist = 15;  // TODO: ramp over a distance (in cm)
            start_pos = l2_get_pos()+cmd_p->val.u8[1];  // start ramp at: here + offset given by transponder, minus ramp len
            l2_sched_speed_ramp(cmd_p->val.u8[0], start_pos, dist);
            break;
        case CMD_OP_TURN_LEFT:     // turn left
            NRF_LOG_INFO("loco3 left in %d for %d", cmd_p->val.u8[0], cmd_p->val.u8[1]);
            task.id = T_TURN_LEFT;
            task.start = l2_get_pos() + cmd_p->val.u8[0];
            task.stop = task.start + cmd_p->val.u8[1];
            l2_sched_pos_task(&task);
            break;
        case CMD_OP_TURN_RIGHT:     // turn right
            NRF_LOG_INFO("loco3 right in %d for %d", cmd_p->val.u8[0], cmd_p->val.u8[1]);
            task.id = T_TURN_RIGHT;
            task.start = l2_get_pos() + cmd_p->val.u8[0];
            task.stop = task.start + cmd_p->val.u8[1];
            l2_sched_pos_task(&task);
            break;
        case CMD_OP_BAT_STOP:       // tells us whether we should shutdown at next stop if our battery is almost empty
            bat_stop = cmd_p->val.u16;
            NRF_LOG_INFO("loco3 bat stop: %d", bat_stop);
            break;
        default:
            NRF_LOG_INFO("uknown opcode %d",cmd_p->op);
            continue;
        }

    }
  
    // update position at which this message was received in case the vehicle position was updated by the received message
    msg_tag.rx_pos += emf_update;
}




/**@brief   Function for handling app_uart events.
 *
 * @details This function will receive a single character from the app_uart module and send to
 *          processing by the loco3 module.
 */
static void loco3_uart_event_handle(app_uart_evt_t * p_event)
{
    uint8_t ch;
    ret_code_t ret;

    switch (p_event->evt_type)
    {
        case APP_UART_DATA_READY:
            // handled by polling, nothing to do here
            break;

        case APP_UART_COMMUNICATION_ERROR:
        case APP_UART_FIFO_ERROR:
            m_loco3_rx_err = true;
            break;

        default:
            break;
    }
}


// 16 bit CRC algorithm
static uint16_t calcCRC(uint8_t c, uint16_t crc)
{
    uint8_t i;
    for (i=0; i<8; i++) {
        if (crc & 0x8000) {
                crc <<= 1;
                crc |= ((c&0x80)?1:0);
                crc ^= 0x1021;
        } else {
                crc <<= 1;
                crc |= ((c&0x80)?1:0);
        }
        c<<=1;
    }
    return crc;
}


// unused timer interrupt handler (interrupt is never enabled)
void loco3_timer_handler(nrf_timer_event_t event_type, void * p_context)
{

}