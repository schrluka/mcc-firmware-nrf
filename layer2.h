
#ifndef __LAYER2_H__
#define __LAYER2_H__

#include <stdint.h>
#include "lococo.h"

// max vehicle speed in mm/s
// for numerical reasons this is limited to 8 bits unsigned (i.e. 255) which is 80km/h
#define V_MAX   255

// min safe speed at which can go
#define V_MIN   20

extern volatile bool execute_l2;

// position depedent tasks which can be scheduled with layer 2
enum task_id {T_TURN_LEFT, T_TURN_RIGHT, T_SET_TRACK_ID};

struct p_task {
    enum task_id    id;     // what do we have to do
    int start;              // pos (in cm) where should it start
    int stop;               // pos (in cm) where should it end, it this is < start, it will run once
    int data;               // task depedent
};


void l2_init();

void l2_poll(void *p_context);

int32_t l2_get_pos();

int64_t l2_set_pos(int p);

uint32_t l2_get_pos_delta();

int32_t l2_get_dist();

uint32_t l2_get_u_bat();

int32_t l2_get_speed();

int32_t l2_get_ref_speed();

void l2_set_max_speed(uint16_t v);

void l2_update_pos_est (const struct lococo_tag* tag, int cm_delta);

int l2_sched_speed_ramp (int speed, int start_pos, int dist);

int l2_sched_pos_task (const struct p_task *t);

void l2_set_track_id(int id);

uint8_t l2_get_track_id();

void l2_set_brigthness(uint8_t led, uint8_t value);


#endif

