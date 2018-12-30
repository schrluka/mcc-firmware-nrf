/**********************************************************************************************
*
* Faller Car Controller - NRF52 Firmware
*
* (c) 2016
* Lukas Schrittwieser
*
* all rights reserved
*
* Layer1: Torque (current) and speed (voltage) controllers
*
**********************************************************************************************/

#include <stdio.h>
#include "nuts_bolts.h"
#include "layer1.h"
#include "layer2.h"
#include "hw.h"

#include "nrf_nvic.h"
#include "nrf_log.h"



/*******************************************************************************************************************************************
*   G L O B A L S
*/

// measured motor current in mA
static volatile int32_t	i_m	= 0;

// current reference value in mA
static volatile int32_t iRef = 100;

// measured back-emf in mV
static volatile int32_t	u_emf = 0;

// measured position as integral of back-emf with 5mVs per LSB
static volatile uint32_t emf_pos = 0;

// reference for speed controller (in mV per LSB)
static volatile int32_t	uRef = 600;

// measured battery voltage in mV
static volatile uint32_t u_bat_sample = 4000;    // assume a full battery

// PI controller for motor current, input is in mA, output in duty cycle
static struct pi current_ctrl;

// speed controller, output is the motor current in mA
static struct pi speed_ctrl;



/*******************************************************************************************************************************************
*   I M P L E M E N T A T I O N
*/

void l1_init()
{
    //printf("current controller sampling frequency: %ld\n", (F_FAST_CTRL));

    // motor (+ filter inductor) has a pole frequency of 7.4kHz, compensate this with the PI zero
    // i.e. we get a nice 20dB/dec slope
    // with a DC resistance of 7 Ohm (and 150uH inductance) a cross over frq of about 1kHz results for
    // controller p gain of 1.
    // Note: input (current errror) and output (requested output voltage) of the controller are in
    // mA and mV
    current_ctrl.max = 3300;    // max bridge output voltage
    current_ctrl.min = 0;
    current_ctrl.int_state = 0;
    float kp = 0.9F * PWM_PERIOD / 3300;    // rescale from voltage to duty cycle by /3300mV and multiply with PWM period to get compare value

    set_pi_gains(&current_ctrl, kp, 500, F_FAST_CTRL);
    //NRF_LOG_INFO("current ctrl: kI: %d   kP: %d   denom: %d \n", (int)current_ctrl.k_i, (int)current_ctrl.k_p, (int)current_ctrl.denom);

    speed_ctrl.max = I_MAX;
    speed_ctrl.min = 0;
    speed_ctrl.int_state = 0;
    //set_pi_gains(&speed_ctrl, 0.6, 15, F_EMF_CTRL);
    set_pi_gains(&speed_ctrl, 0.4, 10, F_EMF_CTRL);
    //NRF_LOG_INFO("speed ctrl: kI: %d   kP: %d   denom: %d \n", (int)speed_ctrl.k_i, (int)speed_ctrl.k_p, (int)speed_ctrl.denom);
}


// called with F_FAST_CTRL Hz by hw.c (adc timer)
// i_mot: measured motor current in mA
// emf: measured motor voltage in mV
// u_bat: measured battery voltage in mV
void l1_periodic (float i_mot, float emf, float u_bat)
{
    static int u_ctrl_cnt = 0;  // counter for slower speed controller

    int32_t I;
    int32_t d;

    I = (int32_t)i_mot;    // motor current in mA

    if (u_ctrl_cnt == 0) {
        // disable PWM output, pin will float once motor current has died out
        hw_dis_pwm();
    }
    if (u_ctrl_cnt == 1) {
        u_emf = (int32_t)emf;    // sample motor emf voltage
        u_bat_sample = (int32_t)u_bat;     // bat voltage in mV, meas when motor is off, this still needs lowpass filtering

        if ((uRef > 0) && (hw_get_reed())) {
            // execute speed controller
            iRef = update_pi(&speed_ctrl, uRef - u_emf);
        } else {
            iRef = 0;
            u_emf = 0;  // prevent integration of offset errors
        }

        // integrate back emf, we use this as an estimation of the vehicle position
        emf_pos += u_emf;

        hw_en_pwm();
        execute_l2 = true;  // signal that we need an update of the speed ref during ramps
    }

    if (u_ctrl_cnt != 0) {
        // update current controller, the result is the requested output voltage
        d = update_pi(&current_ctrl, iRef - I);

        hw_set_pwm_duty(d);

        i_m = I;
    }

    if (u_ctrl_cnt == (F_FAST_CTRL/F_EMF_CTRL-1)) {   // periodic voltage measurement and speed control counter
        u_ctrl_cnt = 0;
    } else
        u_ctrl_cnt++;
}


// return sampled motor current
int32_t L1getCurrent()
{
    int32_t s;
    s = i_m;
    return s;
}


// returns the EMF (induced motor voltage) if the modulator is in SPEED mode or
// measured track voltage if the modulator is in MEAS mode
int32_t l1_get_emf()
{
    int32_t s;
    //uint8_t nested;
    //sd_nvic_critical_region_enter(&nested);
	s = u_emf;
	//sd_nvic_critical_region_exit(nested);
    return s;
}


int32_t L1getIRef()
{
	int32_t s;
    //uint8_t nested;
    //sd_nvic_critical_region_enter(&nested);
	s = iRef;
	//sd_nvic_critical_region_exit(nested);
    return s;
}


uint32_t l1_get_emf_pos()
{
    int32_t s;
    //uint8_t nested;
    //sd_nvic_critical_region_enter(&nested);
    s = emf_pos;
    //sd_nvic_critical_region_exit(nested);
    return s;
}


void l1_set_emf_pos(uint32_t e)
{
    emf_pos = e;
}


uint32_t l1_get_u_bat()
{
    uint32_t s;
    //uint8_t nested;
    //sd_nvic_critical_region_enter(&nested);
    s = u_bat_sample;
    //sd_nvic_critical_region_exit(nested);
    return s;
}


// set back emf voltage reference in mV
void l1_set_emf_ref(int32_t emf)
{
    // safety limit
    if (emf > 2000)
        emf = 2000;
    // we can only drive in one direction
    if (emf < 0)
        emf = 0;
    uRef = emf;
}


int32_t l1_get_emf_ref()
{
    return uRef;
}





