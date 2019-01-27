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
#include <stdbool.h>
#include "nordic_common.h"
#include "nrf.h"
#include "nrf_log.h"
#include "main.h"



/**********************************************************************************************
*   D E F I N E S                                                                            */

// number of vehicles which we track as candidates for the one defining our max position
#define VF_NUM_TRACKED 3


struct vehicle_status {
    uint32_t                rx_time;  // time stamp of last status update, 0 means entry invalid
    int8_t                  rssi;
    ble_gap_addr_t          addr;
    struct adv_status_data  data;
};



/**********************************************************************************************
*   L O C A L   P R O T O T Y P E S                                                          */

static void vf_ble_evt_handler(ble_evt_t const * p_ble_evt, void * p_context);

static process_scan_resp(ble_gap_evt_adv_report_t const * p_report);



/**********************************************************************************************
*   G L O B A L S                                                                            */

static struct vehicle_status detected_vehicles[VF_NUM_TRACKED] = {};


void vf_init()
{
    // Register a handler for BLE events.
    NRF_SDH_BLE_OBSERVER(m_ble_observer, APP_BLE_OBSERVER_PRIO, vf_ble_evt_handler, NULL);

}


static void vf_ble_evt_handler(ble_evt_t const * p_ble_evt, void * p_context)
{
    uint32_t err_code;

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_ADV_REPORT:
        {
            ble_gap_evt_adv_report_t const * p_adv_report = &p_ble_evt->evt.gap_evt.params.adv_report;
            if (p_adv_report->type.scan_response) {
                if (p_adv_report->data.len > 0) {
                    send_log(p_adv_report);                        
                } else {
                    NRF_LOG_INFO("Empty scan response received.");
                }
            } else {
                //NRF_LOG_INFO("Advertising packet received:");
                //NRF_LOG_RAW_HEXDUMP_INFO(p_adv_report->data.p_data, p_adv_report->data.len);
            }
            break;
        }
    }
}


// parse and process a scan response received from another vehicle (or any other BLE device)
static process_scan_resp(ble_gap_evt_adv_report_t const * p_report)
{
    uint32_t now = hw_get_tick(); // remember when this message was received (time stamp in ms)

    if (p_report == NULL) {
        return;
    }

    // lets see if we have a manufacturer specific data structure
    uint16_t offset = 0;  // this is important
    uint16_t len    = ble_advdata_search(p_report->data.p_data, p_report->data.len, &offset, 
                                          BLE_GAP_AD_TYPE_MANUFACTURER_SPECIFIC_DATA);

    if (len == 0) {
        return;
    }

    //int8_t rssi = p_report->rssi;
    uint8_t * p_data = p_report->data.p_data + offset;

    uint16_t company_identifier = uint16_decode(p_data);
    p_data += 2;
    len -= 2;

    if (company_identifier != 0xFFFF) {  // we use 'invalid company' as ID
        return;
    }

    if (len != sizeof(struct adv_status_data)) {
        return;   // 
    }
    // cast pointer, this gives us the current status of the trailing vehicle
    struct adv_status_data * p_status = (void*) p_data;

    // TODO: identify which vehicle are really ahead of us and 

    // is this vehicle in our list?
    for (int i=0; i<VF_NUM_TRACKED; i++) {

    }

}