

void bat_init();

void bat_event (ble_evt_t * p_ble_evt);

// update battery status according to measured voltage in mV
void bat_update(uint32_t v_bat);


