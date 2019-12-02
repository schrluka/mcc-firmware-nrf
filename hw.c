/**********************************************************************************************
*
* Faller Car Controller - NRF52 Firmware
*
* (c) 2016
* Lukas Schrittwieser
*
* all rights reserved
*
* Hardware: Access to all hardware functions, such as PWM, GPIOs, ADCs, etc
*
**********************************************************************************************/

#include <stdint.h>
#include "nrf_drv_pwm.h"
#include "nrf_pwm.h"
#include "nrf_drv_saadc.h"
#include "nrf_drv_ppi.h"
#include "nrf_drv_timer.h"
#include "nrf_drv_gpiote.h"
#include "nrf_drv_saadc.h"
#include "boards.h"
#include "bsp.h"
#include "hw.h"
#include "layer1.h"
#include "lococo.h"



/**********************************************************************************************
*   M A C R O S                                                                              */

// Pin Assignments
#define PIN_PWM_HI_A    6
#define PIN_PWM_LO_A    7
#define PIN_PWM_HI_B    8
#define PIN_PWM_LO_B    11

#define PIN_PSU_EN      16
#define PIN_DBG_LED     1

#define PIN_PWR_BTN     22
#define PIN_REED        23

#define PIN_LED_HEAD        20
#define PIN_LED_BACK        15
#define PIN_LED_LEFT        14
#define PIN_LED_RIGHT       13

// adc channels (not pin numbers) of analog inputs
#define AIN_I_SENS_A    NRF_SAADC_INPUT_AIN0
#define AIN_U_SENS_A    NRF_SAADC_INPUT_AIN1
#define AIN_I_SENS_B    NRF_SAADC_INPUT_AIN3
#define AIN_U_SENS_B    NRF_SAADC_INPUT_AIN2
#define AIN_U_BAT       NRF_SAADC_INPUT_AIN7



/**********************************************************************************************
*   G L O B A L S                                                                            */

// driving direction of bus (motor output polarity)
static bool forward = true;

// PWM driver instance
static nrf_drv_pwm_t m_pwm0 = NRF_DRV_PWM_INSTANCE(0);
// PWM driver values which are
static volatile nrf_pwm_values_individual_t mot_pwm_values;  // memory for PWM compare values
static nrf_pwm_sequence_t const    mot_pwm_seq =
{
    .values.p_individual = (void*)&mot_pwm_values,
    .length              = NRF_PWM_VALUES_LENGTH(mot_pwm_values),
    .repeats             = 0,
    .end_delay           = 0
};

const int pwm_top     = PWM_PERIOD;    // main motor pwm freq in HZ, /2 because of up and down count

// ADC hardware

#define SAMPLES_IN_BUFFER 5
static const nrf_drv_timer_t m_adc_timer = NRF_DRV_TIMER_INSTANCE(1);
static nrf_saadc_value_t     m_adc_buffer_pool[2][SAMPLES_IN_BUFFER];   // two sample buffers for the adc
static nrf_ppi_channel_t     m_adc_ppi_channel;


// status of all leds (bit 1 means on)
static uint32_t leds = 0;
static nrf_drv_pwm_t m_pwm1 = NRF_DRV_PWM_INSTANCE(1);

// duty cycle compare values for all LEDs
static volatile nrf_pwm_values_individual_t  led_duty = {0};
static nrf_pwm_sequence_t const    led_pwm_seq =
{
    .values.p_individual = (void*)&led_duty,
    .length              = NRF_PWM_VALUES_LENGTH(led_duty),
    .repeats             = 0,
    .end_delay           = 0
};

static volatile uint32_t sys_tick = 0;



/**********************************************************************************************
*   P R O T O T Y P E S                                                                      */

static inline void init_adc();

static inline void init_loco3();

static void update_leds();

static void timer_handler(nrf_timer_event_t event_type, void * p_context);



/**********************************************************************************************
*   I M P L E M E N T A T I O N                                                              */


void hw_init()
{
    uint32_t err_code;


    ret_code_t ret = nrf_drv_ppi_init();
    APP_ERROR_CHECK(ret);

    //ret = nrf_drv_gpiote_init();
    //APP_ERROR_CHECK(ret);

    // make sure A bridge is off (both devices)
    nrf_gpio_cfg_output(PIN_PWM_HI_A);
    nrf_gpio_pin_set(PIN_PWM_HI_A);   // pwm sig is inverted as we use a PMOS
    nrf_gpio_pin_clear(PIN_PWM_LO_A);
    nrf_gpio_cfg_output(PIN_PWM_LO_A);

    hw_set_pwm_duty(0);

    // configure pwm module 0 for motor pwm
    nrf_drv_pwm_config_t const config0 =
    {
        // use forward mode as default config
        .output_pins =
        {
            PIN_PWM_HI_A,  // channel 0
            PIN_PWM_LO_A, // | NRF_DRV_PWM_PIN_INVERTED, // channel 1
            NRF_DRV_PWM_PIN_NOT_USED, // | NRF_DRV_PWM_PIN_INVERTED, // channel 2
            NRF_DRV_PWM_PIN_NOT_USED, // | NRF_DRV_PWM_PIN_INVERTED  // channel 3
        },
        .irq_priority = 7,  // 7 is lowest priority, we don't use interrupts anyway
        .base_clock   = NRF_PWM_CLK_16MHz,
        .count_mode   = NRF_PWM_MODE_UP_AND_DOWN,
        .top_value    = pwm_top,
        .load_mode    = NRF_PWM_LOAD_INDIVIDUAL,
        .step_mode    = NRF_PWM_STEP_AUTO
    };
    err_code = nrf_drv_pwm_init(&m_pwm0, &config0, NULL);
    APP_ERROR_CHECK(err_code);
    hw_set_pwm_duty(0);
    // start pwm signal generation
    nrf_drv_pwm_simple_playback(&m_pwm0, &mot_pwm_seq, 1, NRF_DRV_PWM_FLAG_LOOP);

    forward = true;
    nrf_gpio_cfg_output(PIN_PWM_HI_B);
    nrf_gpio_pin_set(PIN_PWM_HI_B);   // pwm sig is inverted as we use a PMOS
    nrf_gpio_cfg_output(PIN_PWM_LO_B);
    nrf_gpio_pin_set(PIN_PWM_LO_B);

    nrf_gpio_cfg_input(PIN_REED, GPIO_PIN_CNF_PULL_Pullup);
    nrf_gpio_cfg_input(PIN_PWR_BTN, GPIO_PIN_CNF_PULL_Disabled);

    init_adc(); // setup timer and adc to perform regular conversion

    for (int i=0; i<NUM_LEDS; i++) {
        leds |= (1<<i);
    }
    led_duty.channel_0 = 0;
    led_duty.channel_1 = 0;
    led_duty.channel_2 = 0;
    led_duty.channel_3 = 0;

    // configure pwm module 1 for leds
    nrf_drv_pwm_config_t const config1 =
    {
        // use forward mode as default config
        .output_pins =
        {
            PIN_LED_HEAD,
            PIN_LED_BACK,
            PIN_LED_LEFT,
            PIN_LED_RIGHT, // | NRF_DRV_PWM_PIN_INVERTED
        },
        .irq_priority = 7,  // 7 is lowest priority, we don't use interrupts anyway
        .base_clock   = NRF_PWM_CLK_16MHz,
        .count_mode   = NRF_PWM_MODE_UP,
        .top_value    = LED_PWM_TOP,
        .load_mode    = NRF_PWM_LOAD_INDIVIDUAL,
        .step_mode    = NRF_PWM_STEP_AUTO
    };
    err_code = nrf_drv_pwm_init(&m_pwm1, &config1, NULL);
    APP_ERROR_CHECK(err_code);
    nrf_drv_pwm_simple_playback(&m_pwm1, &led_pwm_seq, 1, NRF_DRV_PWM_FLAG_LOOP);

    hw_set_led(LED_HEAD_LIGHT, 1);   // head light on
    hw_set_led(LED_TURN_LEFT, 0);   // turn signal off
    hw_set_led(LED_TURN_RIGHT, 0);   // turn signal off
    hw_set_led_duty(LED_BACK_LIGHT, LED_PWM_TOP/3);   // lower back light
    hw_set_led_duty(LED_TURN_LEFT, LED_PWM_TOP-1);    // full power turn lights
    hw_set_led_duty(LED_TURN_RIGHT, LED_PWM_TOP-1);
}


uint32_t hw_get_tick()
{
    uint8_t nested;
    uint32_t tick;
    sd_nvic_critical_region_enter(&nested);
    tick = sys_tick;
    sd_nvic_critical_region_exit(nested);
    return tick;
}


void hw_dbg_led_on()
{
    nrf_gpio_cfg_output(PIN_DBG_LED);
    nrf_gpio_pin_clear(PIN_DBG_LED);
}


void hw_dbg_led_off()
{
    nrf_gpio_cfg_output(PIN_DBG_LED);
    nrf_gpio_pin_set(PIN_DBG_LED);
}


void hw_dbg_led_toggle()
{
    nrf_gpio_pin_toggle(PIN_DBG_LED);
}


// stop PWM signal of motor (to be able to measure EMF for example)
void hw_dis_pwm(void)
{
    uint32_t out_pins[NRF_PWM_CHANNEL_COUNT];

    for (int i=0; i<NRF_PWM_CHANNEL_COUNT; i++)
        out_pins[i] = NRF_PWM_PIN_NOT_CONNECTED;

    // make sure current return mosfet is still on
    if (forward) {
        // half bridge A is active in forward mode,
        // enable low side mosfet on half bridge B as current return path
        nrf_gpio_cfg_output(PIN_PWM_HI_B);
        nrf_gpio_pin_set(PIN_PWM_HI_B);   // pwm sig is inverted as we use a PMOS
        nrf_gpio_cfg_output(PIN_PWM_LO_B);
        nrf_gpio_pin_set(PIN_PWM_LO_B);
        // turn off both A side switches
        nrf_gpio_cfg_output(PIN_PWM_HI_A);
        nrf_gpio_pin_set(PIN_PWM_HI_A);   // pwm sig is inverted as we use a PMOS
        nrf_gpio_cfg_output(PIN_PWM_LO_A);
        nrf_gpio_pin_clear(PIN_PWM_LO_A);
    } else {
        // half bridge B is active in backward mode, disable both switches by
        nrf_gpio_cfg_output(PIN_PWM_HI_A);
        nrf_gpio_pin_set(PIN_PWM_HI_A);   // pwm sig is inverted as we use a PMOS
        nrf_gpio_cfg_output(PIN_PWM_LO_A);
        nrf_gpio_pin_set(PIN_PWM_LO_A);

        nrf_gpio_cfg_output(PIN_PWM_HI_B);
        nrf_gpio_pin_set(PIN_PWM_HI_B);   // pwm sig is inverted as we use a PMOS
        nrf_gpio_cfg_output(PIN_PWM_LO_B);
        nrf_gpio_pin_clear(PIN_PWM_LO_B);

    }
    // reconfigure pwm unit
    nrf_pwm_pins_set(m_pwm0.p_registers, out_pins);
}


// reenable motor PWM
void hw_en_pwm(bool fwd)
{
    // (re)configure pwm unit pins
    uint32_t out_pins[NRF_PWM_CHANNEL_COUNT] = {
            PIN_PWM_HI_A,  // channel 0
            PIN_PWM_LO_A, // | NRF_DRV_PWM_PIN_INVERTED, // channel 1
            PIN_PWM_HI_B, // | NRF_DRV_PWM_PIN_INVERTED, // channel 2
            PIN_PWM_LO_B}; // | NRF_DRV_PWM_PIN_INVERTED  // channel 3

    forward = fwd;  // TODO: protection against reversal at full speed

    // make sure only one half bridge is active
    if (forward) {
        // half bridge A is active in forward mode,
        // enable low side mosfet on half bridge B as current return path
        nrf_gpio_cfg_output(PIN_PWM_HI_B);
        nrf_gpio_pin_set(PIN_PWM_HI_B);   // pwm sig is inverted as we use a PMOS
        nrf_gpio_cfg_output(PIN_PWM_LO_B);
        nrf_gpio_pin_set(PIN_PWM_LO_B);
        out_pins[2]  = NRF_PWM_PIN_NOT_CONNECTED;
        out_pins[3]  = NRF_PWM_PIN_NOT_CONNECTED;

    } else {
        // half bridge B is active in backward mode, disable both switches by
        nrf_gpio_cfg_output(PIN_PWM_HI_A);
        nrf_gpio_pin_set(PIN_PWM_HI_A);   // pwm sig is inverted as we use a PMOS
        nrf_gpio_cfg_output(PIN_PWM_LO_A);
        nrf_gpio_pin_set(PIN_PWM_LO_A);
        out_pins[0]  = NRF_PWM_PIN_NOT_CONNECTED;
        out_pins[1]  = NRF_PWM_PIN_NOT_CONNECTED;
    }

    // reconfigure pwm unit
    nrf_pwm_pins_set(m_pwm0.p_registers, out_pins);
}


// set pwm duty cycle, d can be 0 to pwm_top-1
// larger d means higher voltage applied to motor
void hw_set_pwm_duty(uint32_t d)
{
    static uint32_t last_d = 0;  // used to check whether d increased or decreased since last call
    if (d > pwm_top-1) {
        d = pwm_top-1;
    }
    bool inc = true;    // assume d increased (need this to ensure proper dead time in half bridge
    if (d < last_d) {
        inc = false;
    }

    if (forward) {
        if (inc){
            // for increasing duty cycle we change the low side (NMOS) switch first
            mot_pwm_values.channel_1 = d;
            // now change hi side (PMOS) switch
            mot_pwm_values.channel_0 = d + 1; // +1 to ensure dead time
        } else {
            // for decreasing duty cycle we change the high side (PMOS) switch first
            mot_pwm_values.channel_0 = d + 1; // +1 to ensure dead time
            mot_pwm_values.channel_1 = d;
        }
        // turn off B side PMOS
        mot_pwm_values.channel_2 = 0;
        // turn on B side NMOS as current return path
        mot_pwm_values.channel_3 = pwm_top + 1;
    } else {
        // Neg voltage is achieved by switching half bridge B instead of A
        if (inc){
            // for increasing duty cycle we change the low side (NMOS) switch first
            mot_pwm_values.channel_3 = d;
            // now change hi side (PMOS) switch
            mot_pwm_values.channel_2 = d + 1; // +1 to ensure dead time
        } else {
            // for decreasing duty cycle we change the high side (PMOS) switch first
            mot_pwm_values.channel_2 = d + 1; // +1 to ensure dead time
            mot_pwm_values.channel_3 = d;
        }
        // turn off B side PMOS
        mot_pwm_values.channel_0 = 0;
        // turn on B side NMOS as current return path
        mot_pwm_values.channel_1 = pwm_top + 1;
    }

    last_d = d;
}


void hw_set_led(unsigned int ch, int on)
{
   if (ch >= NUM_LEDS)
        return;

    if (on) {
        leds |= (1<<ch);
    } else {
        leds &= ~(1<<ch);
    }
    update_leds();
}


void hw_toggle_led(unsigned int ch)
{
    if (leds & (1<<ch))
        hw_set_led(ch, 0);
    else
        hw_set_led(ch, 1);
    update_leds();
}


// set led duty cycle reference d for led number ch, allowed 0..LED_PWM_TOP
void hw_set_led_duty(unsigned int ch, uint32_t d)
{
    d = LED_PWM_TOP - d;    // invert duty cycle, invert flag did not work

    if (ch >= NUM_LEDS)
        return;
    switch (ch) {
        case 0: led_duty.channel_0 = d; break;
        case 1: led_duty.channel_1 = d; break;
        case 2: led_duty.channel_2 = d; break;
        case 3: led_duty.channel_3 = d; break;
    }
}


// enable / disable power supply (the buck boost converter which provides our supply voltage)
void hw_en_psu(bool en)
{
    if (en)
        nrf_gpio_pin_set(PIN_PSU_EN);
    else
        nrf_gpio_pin_clear(PIN_PSU_EN);

    nrf_gpio_cfg_output(PIN_PSU_EN);
}


bool hw_get_reed()
{
    if (!nrf_gpio_pin_read(PIN_REED))
        return true;
    else
        return false;
}


bool hw_get_pwr_btn()
{
    if (nrf_gpio_pin_read(PIN_PWR_BTN))
        return true;    // button is active high
    else
        return false;
}


// adc conversion complete callback
static void saadc_callback(nrf_drv_saadc_evt_t const * p_event)
{
    static int cnt=0;
    //hw_dbg_led_on();

    if (p_event->type == NRF_DRV_SAADC_EVT_DONE) {
        ret_code_t err_code;

        //printf("%d: %d %d %d %d %d\n",cnt++, p_event->data.done.p_buffer[0],
        //    p_event->data.done.p_buffer[1], p_event->data.done.p_buffer[2],
        //    p_event->data.done.p_buffer[3], p_event->data.done.p_buffer[4]);

        //printf("ua: %d\n",p_event->data.done.p_buffer[1]);
        //printf("bat: %d\n",p_event->data.done.p_buffer[4]);

        //printf("%05d %05d\n",p_event->data.done.p_buffer[2],p_event->data.done.p_buffer[1]);

        float i_mot,emf;
        if (forward) {
            // half bridge A is switching
            i_mot = p_event->data.done.p_buffer[2] * 0.0355F;
            emf = p_event->data.done.p_buffer[1] * 0.1956F;
        } else {
            i_mot = p_event->data.done.p_buffer[0] * 0.0355F;
            emf = p_event->data.done.p_buffer[3] * 0.1956F;
        }
        float u_bat =  p_event->data.done.p_buffer[4] * 0.2957F;

        // execute current controller
        l1_periodic (i_mot, emf, u_bat);

        // requeue this buffer for a new conversion
        err_code = nrf_drv_saadc_buffer_convert(p_event->data.done.p_buffer, SAMPLES_IN_BUFFER);
        APP_ERROR_CHECK(err_code);

    }

    // create a 1kHz sys tick
    cnt++;
    if (cnt >= 3) {
        cnt = 0;
        sys_tick++;
    }

    //hw_dbg_led_off();
}



static inline void init_adc()
{
    // init SAADC peripheral
    ret_code_t ret;

    //nrf_drv_saadc_config_t saadc_config;
    //saadc_config.

    ret = nrf_drv_saadc_init(NULL, &saadc_callback);
    APP_ERROR_CHECK(ret);

    // setup conversion channels channels
    nrf_saadc_channel_config_t channel_config[] = {
        NRF_DRV_SAADC_DEFAULT_CHANNEL_CONFIG_SE(AIN_I_SENS_A),
        NRF_DRV_SAADC_DEFAULT_CHANNEL_CONFIG_SE(AIN_U_SENS_A),
        NRF_DRV_SAADC_DEFAULT_CHANNEL_CONFIG_SE(AIN_I_SENS_B),
        NRF_DRV_SAADC_DEFAULT_CHANNEL_CONFIG_SE(AIN_U_SENS_B),
        NRF_DRV_SAADC_DEFAULT_CHANNEL_CONFIG_SE(AIN_U_BAT),
        };

    channel_config[0].gain = NRF_SAADC_GAIN1;
    channel_config[2].gain = NRF_SAADC_GAIN1;
    channel_config[4].gain = NRF_SAADC_GAIN1;

    for (int i=0; i<sizeof(channel_config)/sizeof(channel_config[0]); i++) {
        ret = nrf_drv_saadc_channel_init(i, &(channel_config[i]));
        APP_ERROR_CHECK(ret);
    }

    // register two conversion buffers with the adc driver
    ret = nrf_drv_saadc_buffer_convert(m_adc_buffer_pool[0], SAMPLES_IN_BUFFER);
    APP_ERROR_CHECK(ret);

    ret = nrf_drv_saadc_buffer_convert(m_adc_buffer_pool[1], SAMPLES_IN_BUFFER);
    APP_ERROR_CHECK(ret);

    // configure timer

    nrf_drv_timer_config_t timer_cfg = NRF_DRV_TIMER_DEFAULT_CONFIG;
    timer_cfg.bit_width = NRF_TIMER_BIT_WIDTH_32;
    ret = nrfx_timer_init(&m_adc_timer, &timer_cfg, timer_handler);   // register a dummy timer interrupt
    APP_ERROR_CHECK(ret);

    // setup timer to trigger compares
    uint32_t ticks = nrf_drv_timer_us_to_ticks(&m_adc_timer, 1000000/F_FAST_CTRL); // time in us
    nrf_drv_timer_extended_compare(&m_adc_timer,
                                   NRF_TIMER_CC_CHANNEL0,
                                   ticks,
                                   NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK,
                                   false);
    nrf_drv_timer_enable(&m_adc_timer);

    // setup ppi channel so that timer compare event is triggering sample task in SAADC
    uint32_t timer_compare_event_addr = nrf_drv_timer_compare_event_address_get(&m_adc_timer,
                                                                                NRF_TIMER_CC_CHANNEL0);
    uint32_t saadc_sample_task_addr   = nrf_drv_saadc_sample_task_get();

    ret = nrf_drv_ppi_channel_alloc(&m_adc_ppi_channel);
    APP_ERROR_CHECK(ret);

    ret = nrf_drv_ppi_channel_assign(m_adc_ppi_channel,
                                          timer_compare_event_addr,
                                          saadc_sample_task_addr);
    APP_ERROR_CHECK(ret);

    // enable PPI to trigger adc on timer compare
    ret = nrf_drv_ppi_channel_enable(m_adc_ppi_channel);
    APP_ERROR_CHECK(ret);

}


// unused timer interrupt handler (interrupt is never enabled)
static void timer_handler(nrf_timer_event_t event_type, void * p_context)
{

}


// en-/disable leds according to global var
static void update_leds()
{

    uint32_t out_pins[NRF_PWM_CHANNEL_COUNT] = {
            PIN_LED_HEAD,
            PIN_LED_BACK,
            PIN_LED_LEFT,
            PIN_LED_RIGHT};

    for (int i=0; i<NRF_PWM_CHANNEL_COUNT; i++) {
        if (!(leds&(1<<i))) {
            nrf_gpio_pin_clear(out_pins[i]);    // turn led of by gpio
            out_pins[i] = NRF_PWM_PIN_NOT_CONNECTED;    // tell pwm unit to not use this pin
        }
    }

    // reconfigure pwm unit
    nrf_pwm_pins_set(m_pwm1.p_registers, out_pins);
}






