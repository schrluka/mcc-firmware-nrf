
#ifndef __ADV_H__
#define __ADV_H__


// struct which is added to the advertisement broadcast to send status info.
// this must match the app and has to be quite short (not many bytes available in the ble adv bc)
// TODO: should we add a version number here?
struct adv_status_data {
    int16_t speed;  // measured vehicle speed in mm/sec
    int16_t u_bat;  // battery voltage in mV
    int32_t pos;    // vehicle position in cm
    int32_t delta_pos; 
    int8_t  track_id; // to identify tracks
    int8_t  dist;
} __attribute__((packed));


void advertising_init(void);

void advertising_start(void);

void advertising_update(void);

void advertising_schedule_track_id_change(uint8_t new_track_id);


#endif