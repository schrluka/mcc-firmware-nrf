#ifndef __LAYER1_H__
#define __LAYER1_H__

#include <stdint.h>

/**********************************************************************************************
*   C O N F I G U R A T I O N                                                                */


// maximum current (in mA) requested by the speed controller
#define I_MAX			200

// speed controller execution freq in Hz (must be an integer of current ctrl frq)
#define F_EMF_CTRL      200



/**********************************************************************************************
*   P R O T Y P E S                                                                          */


void l1_init();

void l1_periodic (float i_mot, float emf, float u_bat);

int32_t l1_get_emf();

int32_t L1getIRef();

uint32_t l1_get_emf_pos();

void l1_set_emf_pos(uint32_t e);

uint32_t l1_get_u_bat();

void l1_set_emf_ref(int32_t emf);

int32_t l1_get_emf_ref();

#endif
