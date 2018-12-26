/**********************************************************************************************
* Blockstreckensteuerung
*
* (c) 2011
* Lukas Schrittwieser
*
* all rights reserved
*
**********************************************************************************************/
#ifndef __HW_H__
#define __HW_H__


#include <stdbool.h>



// pwm counter top value
#define PWM_PERIOD  16000000/2/50000



// set LoCo3 carrier frequency in Hz
#define F_LOCO3     180000

// current controller execution and ADC sampling frequency
// note: needs to match divider for tick counter
#define F_FAST_CTRL 3000



#define NUM_LEDS    4

// max duty cycle value for LEDs
#define LED_PWM_TOP 1023


/**********************************************************************************************
*   P R O T Y P E S                                                                          */

void hw_init();

uint32_t hw_get_tick();

void hw_dbg_led_on();

void hw_dbg_led_off();

void hw_dbg_led_toggle();

void hw_set_led(unsigned int ch, int on);

void hw_toggle_led(unsigned int ch);

void hw_set_led_duty(unsigned int ch, uint32_t d);

void hw_led_poll();

void hw_en_psu(bool en);

bool hw_get_reed();

bool hw_get_pwr_btn();

void sample_ADC();

void hw_set_pwm_duty(uint32_t d);

void hw_dis_pwm();

void hw_en_pwm();

#endif
