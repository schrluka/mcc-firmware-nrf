

#ifndef __MODEL_CAR_BLE_SERVICE__
#define __MODEL_CAR_BLE_SERVICE__


// for simplicity we use short (2 byte) UUID which recycles all other bytes from some other device UUID
#define BLE_UUID_MODEL_CAR_SERVICE      0xCAFE



uint32_t mcs_init();

uint32_t mcs_update();



#endif
