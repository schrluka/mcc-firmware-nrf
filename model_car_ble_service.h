

#ifndef __MODEL_CAR_BLE_SERVICE__
#define __MODEL_CAR_BLE_SERVICE__


// for simplicity we use short (2 byte) UUID which recycles all other bytes from some other device UUID
#define BLE_UUID_MODEL_CAR_SERVICE      0xCAFE

// values for mcs_direction
#define MCS_FWD     0
#define MCS_STOP    1
#define MCS_BWD     2


uint32_t mcs_init();

uint32_t mcs_update();

uint32_t mcs_get_dist_coil2tail();

uint32_t mcs_get_ref_distance();

int8_t mcs_get_direction();

#endif
