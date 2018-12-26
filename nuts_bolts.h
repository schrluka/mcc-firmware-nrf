
#ifndef __NUTS_BOLTS_H__
#define __NUTS_BOLTS_H__

#include <stdint.h>
#include <stdbool.h>


struct pi {
    int32_t k_p;        // proportional gain
    int32_t k_i;        // integrator gain
    int32_t denom;      // common denominator for gains
    int32_t max;        // max allowed output
    int32_t min;        // min allowed output
    int32_t int_state;  // integrator state variable
};

// first order low pass filter information & state struct type
// note that alpha = alphaNum / alphaDen must be < 1, otherwise the filter is unstable!
typedef struct
{
	int32_t 	alphaNum;		// filter constant numerator
	uint32_t 	alphaDen;		// filter constant denominator
	int32_t		y;				// state variable
} lpf1_t;

// meta data for bresenham line algorithm
struct b_line {
    // config
    int32_t start_pos;  // starting position of ramp
    int32_t delta_time; // time delta over which to ramp in controller execution period
    int32_t delta_v;    // speed delta over which to ramp
    int32_t sign;       // +1: accelerate, -1: deccelerate
    // state, has to be set to correct start values
    int32_t err;        // bresenham error
    int32_t v;          // current speed
    int32_t time;       // current position
    int32_t cnt;        // counter, number of remaining position steps
};

int32_t update_pi(struct pi* ctrl, int32_t in);

void set_pi_gains(struct pi* ctrl, float k_p, float f_zero, float f_samp);

int32_t lpf1Update (lpf1_t* filter, int32_t x);

int lpf1Init (lpf1_t* filter, uint32_t F, uint32_t tau);

int get_next_ramp_pt(struct b_line* ramp);


#endif
