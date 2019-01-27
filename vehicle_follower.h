/**********************************************************************************************
*
* Faller Car Controller - NRF52 Firmware
*
* (c) 2019
* Lukas Schrittwieser
*
* all rights reserved
*
* Vehicle Follower: Track posisition of other vehicles broadcasted via adverstising and make
*    make sure we don't crash into another vehicle ahead of us.
*
**********************************************************************************************/
#ifndef __VEHICLE_FOLLOWER__
#define __VEHICLE_FOLLOWER__

void vf_init();

uint32_t vf_get_dist_2_lead(int32_t current_pos);


#endif
