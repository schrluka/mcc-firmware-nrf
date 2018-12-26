#ifndef __LOCOCOCO_H__
#define __LOCOCOCO_H__

#include <stdbool.h>


// meta data of a lococo message
struct lococo_tag {
    uint32_t rx_tick;       // tick at which message was received
    uint32_t delta_tick;    // ticks since last message was received
    int rx_pos;             // local position estimation at which this messages was received
    int delta_pos;          // difference of local position estimation between this and the
                            // previous messages
};

/**********************************************************************************************
*   P R O T O T Y P E S                                                                      */

void lococo_init(void);

int lococo_get_bat_stop();

void lococo_rx(uint8_t byte, bool err);

#endif // __LOCOCOCO_H__
