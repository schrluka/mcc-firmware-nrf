/**********************************************************************************************
*
* Faller Car Controller - NRF52 Firmware
*
* (c) 2016
* Lukas Schrittwieser
*
* all rights reserved
*
* nuts_bolts.c: Various functions for filtering, control and reference signal generation
*
**********************************************************************************************/


#include <stdint.h>
#include <limits.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "nuts_bolts.h"



// update pi controller ctrl with error signal in, returns the new controller output
int32_t update_pi(struct pi* ctrl, int32_t in)
{
    in *= ctrl->k_p;
    int32_t denom = ctrl->denom;

    int32_t s = ctrl->int_state; // abbrevation
    s += (in * ctrl->k_i / 100);
    if (s > (ctrl->max*denom))
        s = ctrl->max * denom;
    if (s < (ctrl->min*denom))
        s = ctrl->min * denom;
    ctrl->int_state = s;
    // this could overflow if in is big!
    int32_t ret = in/100 + s/ctrl->denom;

    if (ret > ctrl->max)
        ret = ctrl->max;
    if (ret < ctrl->min)
        ret = ctrl->min;

    return ret;
}


// setups controller gains and denominator (fix point arithmetic of PI controller) with specified
// P gain and PI zero frequency for given sampling frq f_samp
void set_pi_gains(struct pi* ctrl, float k_p, float f_zero, float f_samp)
{
    float k_i = f_zero * 2 * 3.1415927F / f_samp; // desired accumulator gain

    // find limit (max or min) which has the higher absolute value
    int abs_limit = abs(ctrl->max);
    if (abs(ctrl->min) > abs_limit)
        abs_limit = abs(ctrl->min);

    // calc the max. denominator which we can have such that the accumulator will not overflow
    int32_t denom_max = INT32_MAX / abs_limit;

    // use a smaller denominator to be on the safe side (headroom)
    ctrl->denom = denom_max / 256;
    ctrl->k_i = (int32_t)(k_i * ctrl->denom);
    ctrl->k_p = (int32_t)(k_p * 100); // could implement a denom here as well
    // check achieved values
    float k_is = ctrl->k_i / ((float)ctrl->denom);
    if (fabs(k_is-k_i) > 0.1)
        printf("PI controller resolution problem, more than 10%% I gain error.\n");
    k_is = ctrl->k_p / (100.0F);
    if (fabs(k_is-k_p) > 0.1)
        printf("PI controller resolution problem, more than 10%% P gain error.\n");
}



// update first order low pass filter (IIR)
int32_t lpf1Update (lpf1_t* filter, int32_t x)
{
	if (filter == NULL)
		return 0;
	filter->y = ((filter->alphaDen - filter->alphaNum)*x + (filter->alphaNum * filter->y)) / filter->alphaDen;
	return filter->y;
}


// initialize the first order low pass filter and caluclate the filter coefficient from a given sampling rate and time const
// filter: the filter to be initialized
// F: filter sampling frequency in Hz
// tau: filter time constant in micro seconds
// return: 1 on success, 0 otherwise
int lpf1Init (lpf1_t* filter, uint32_t F, uint32_t tau)
{
	if (filter == NULL)
		return 0;

	filter->y = 0;

	// the formula for a follows from an analysis of the filter equation given in the header file
	float a = expf(-1e6F/(tau*F));	// 1e6 because tau is in micro seconds
	//float a = sinf(F);

	// find a denominator
	float b = 1-a;		// the factor we have to express as (alphaDen-alphaNum)/alphaDen
	filter->alphaDen = 1;	// init denom to default
	for (int i=0; i<16; i++)	// do not use more than 16 bits (otherwise there is not much left for the signal itself)
	{
		if (b*(filter->alphaDen) > 1.0F)
		{
			// ok we have found a denominator which causes a useful	result
			// to ensure better accuracy we add 3 bits
			filter->alphaDen *= 8;
			// now calculate the numerator
			filter->alphaNum = (uint32_t)(filter->alphaDen * a);
			return 1;	// done
		}
		filter->alphaDen *= 2;	// use one more bit
	}
	// we came here -> could not find a suitable denominator
	printf("lpf1Init: could not find a denominator\n");
	return 0;
}


// update time and velocity values in speed trajectory according to scheduled ramp
// returns 0 if this ramp is finished
int get_next_ramp_pt(struct b_line* ramp)
{
    if (ramp->cnt <= 0)
        return 0;

    // take one step on position axis
    ramp->cnt--;
    ramp->time++;    // move one step
    int32_t e = ramp->err; // abbrevation
    e -= ramp->delta_v; // update error signal according to bresenham

    while (e < 0) { // take as many steps in speed direction as needed
        ramp->v += ramp->sign;  // accelerate or decelerate as needed
        e += ramp->delta_time;
    }
    ramp->err = e;  // write back value
    return 1;
}

