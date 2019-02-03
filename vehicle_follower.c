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
#include "ble_gap.h"
#include "nrf_sdh.h"
#include "nrf_sdh_ble.h"
#include "ble.h"
#include "ble_advdata.h"
#include "nrf_ble_scan.h"
#include "app_error.h"

#include "main.h"
#include "hw.h"



/**********************************************************************************************
*   D E F I N E S                                                                            */

// number of vehicles that we track as candidates for the one in front of us
#define VF_NUM_TRACKED 5


struct vehicle_status {
    uint32_t                rx_time;  // time stamp of last status update, 0 means entry invalid
    int32_t                  rssi;
    ble_gap_addr_t          addr;
    struct adv_status_data  data;
};



/**********************************************************************************************
*   L O C A L   P R O T O T Y P E S                                                          */

static void scan_evt_handler(scan_evt_t const * p_scan_evt);

static void vf_ble_evt_handler(ble_evt_t const * p_ble_evt, void * p_context);

static void process_scan_resp(ble_gap_evt_adv_report_t const * p_report);

static bool compare_gap_addrs(const ble_gap_addr_t * a1, const ble_gap_addr_t * a2);

static int32_t get_est_vehicle_pos(const struct vehicle_status * v_p);



/**********************************************************************************************
*   G L O B A L S                                                                            */

static struct vehicle_status detected_vehicles[VF_NUM_TRACKED] = {0};

NRF_BLE_SCAN_DEF(m_scan);   



/**********************************************************************************************
*   I M P L E M E N T A T I O N                                                              */


void vf_init()
{
    ret_code_t ret;
    nrf_ble_scan_init_t init_scan;

    memset(&init_scan, 0, sizeof(init_scan));
    init_scan.connect_if_match = false;  // we don't want to connect, we just scan
    init_scan.conn_cfg_tag     = APP_BLE_CONN_CFG_TAG;
    
    // Register a handler for BLE events.
    NRF_SDH_BLE_OBSERVER(m_ble_observer, APP_BLE_OBSERVER_PRIO, vf_ble_evt_handler, NULL);

    ret = nrf_ble_scan_init(&m_scan, &init_scan, scan_evt_handler);
    APP_ERROR_CHECK(ret);

    ret = nrf_ble_scan_start(&m_scan);
    APP_ERROR_CHECK(ret);
}


// returns the distance (in cm) to the leading vehicle or 0 if none is detected and/or 
// within range
uint32_t vf_get_dist_2_lead(int32_t current_pos)
{
    uint32_t now = hw_get_tick(); // remember when this message was received (time stamp in ms)

    uint32_t ind;
    uint32_t min_dist = UINT32_MAX; // for searching the minimum
    uint32_t ind_min = VF_NUM_TRACKED; // index of the vehicle with min. dist.
    for (ind=0; ind<VF_NUM_TRACKED; ind++) {
        struct vehicle_status * v_p = &detected_vehicles[ind];  // abbrevation

        if ((v_p->rx_time == 0) || ((now-(v_p->rx_time)) > 50000)) {
            //NRF_LOG_INFO("too old");
            continue;   // entry is empty or too old.
        }        

        int32_t dist = get_est_vehicle_pos(v_p) - current_pos; // correct if other vehicle position did not jump recently
        int32_t dist_ch = dist - v_p->data.delta_pos;   // assuming the other did change position
        // TODO: check track IDs
        if ((dist < 0) && (dist_ch < 0)) {
            NRF_LOG_DEBUG("VF: %02x:%02x:%02x is behind.", v_p->addr.addr[2], v_p->addr.addr[1], v_p->addr.addr[0]);
            continue;
        }

        // if dist_ch is valid, but not dist, its likely the other vehicle updated its position
        if ((dist < 0) && (dist_ch > 0)) {
            dist = dist_ch;
        }
        if (dist_ch < 0) {
            // dist_ch is not valid, set same value as dist
            dist_ch = dist;
        }
    
        // if both are positive, the smaller one counts
        if (dist_ch < dist) {
            dist = dist_ch;
        }

        // find vehicle in the list that has the smallest (positive) distance
        if (dist <= min_dist) {
            min_dist = dist;
            ind_min = ind;
        }       
    }

    if (ind_min >= VF_NUM_TRACKED){
        // none found
        return 0;
    }
    struct vehicle_status * v_p = &detected_vehicles[ind];  // abbrevation
    //NRF_LOG_INFO("VF: %02x:%02x:%02x leads by %l", v_p->addr.addr[2], v_p->addr.addr[1], v_p->addr.addr[0], min_dist);
    return min_dist;
}


// we are actually not interested in these scan events, but we need to implement a handler, so we keep this 
// copy paste handler from a nordic example
static void scan_evt_handler(scan_evt_t const * p_scan_evt)
{
    ret_code_t err_code;

    switch(p_scan_evt->scan_evt_id)
    {
         case NRF_BLE_SCAN_EVT_CONNECTING_ERROR:
         {
              
         } break;

         case NRF_BLE_SCAN_EVT_CONNECTED:
         {
              ble_gap_evt_connected_t const * p_connected =
                               p_scan_evt->params.connected.p_connected;
             // Scan is automatically stopped by the connection.
             NRF_LOG_INFO("Connecting to target %02x%02x%02x%02x%02x%02x",
                      p_connected->peer_addr.addr[0],
                      p_connected->peer_addr.addr[1],
                      p_connected->peer_addr.addr[2],
                      p_connected->peer_addr.addr[3],
                      p_connected->peer_addr.addr[4],
                      p_connected->peer_addr.addr[5]
                      );
         } break;

         case NRF_BLE_SCAN_EVT_SCAN_TIMEOUT:
         {
             NRF_LOG_INFO("Scan timed out.");
             
         } break;

         default:
             break;
    }
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
                    process_scan_resp(p_adv_report);                        
                } else {
                    //NRF_LOG_INFO("VF: empty scan response");
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
static void process_scan_resp(ble_gap_evt_adv_report_t const * p_report)
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

    int32_t rssi = p_report->rssi;
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
    
    // is this vehicle in our list?
    uint32_t ind;
    for (ind=0; ind<VF_NUM_TRACKED; ind++) {
        if (compare_gap_addrs(&detected_vehicles[ind].addr, &p_report->peer_addr)) {
            //NRF_LOG_INFO("vf: found %02x %02x %02x", p_report->peer_addr.addr[0], p_report->peer_addr.addr[1], p_report->peer_addr.addr[2]);
            break;  // found it!
        } 
    }

    if (ind >= VF_NUM_TRACKED) {
        // this vehicle is not in our list, if its rssi is strong enough we put it in the list
        if (rssi < -75) {
            NRF_LOG_INFO("vf: not accepting vehicle, rssi too low");
            return;
        }
        // TODO: does this replacement strategy make sense?
        // search for the oldest entry
        uint32_t t_max = 0;
        for (int i=0; i<VF_NUM_TRACKED; i++) {
            if (detected_vehicles[i].rx_time >= t_max) {  // >= instead of > to make sure it works with an empty list where all rx_time values are 0
                ind = i;
                t_max = detected_vehicles[i].rx_time;
            }
        }
        // copy address of this vehicle
        detected_vehicles[ind].addr = p_report->peer_addr;
    }

    // TODO: mutual exclusion
    // update status data
    detected_vehicles[ind].rssi = rssi;
    detected_vehicles[ind].rx_time = now;
    // cast pointer, this gives us the current status of the trailing vehicle
    struct adv_status_data * p_status = (void*) p_data;
    detected_vehicles[ind].data = *p_status;
    //NRF_LOG_INFO("pos: %d", p_status->pos);
}


// compare to gap (i.e. 48 bit MAC) addresses
static bool compare_gap_addrs(const ble_gap_addr_t * a1, const ble_gap_addr_t * a2)
{
    for (int i=0; i<BLE_GAP_ADDR_LEN; i++) {
        if (a1->addr[i] != a2->addr[i]) {
            return false;
        }
    }
    return true;
}


// returns an estimate of the current position of vehicle v_p
static int32_t get_est_vehicle_pos(const struct vehicle_status * v_p)
{
    ASSERT(v_p != NULL);
    uint32_t delta_t = hw_get_tick() - v_p->rx_time;    // time delta since last report
    // calculate how far the vehicle travelled since we received its last status report
    uint32_t delta_pos = delta_t * v_p->data.speed / 10000; // div by to to scale ms -> s and mm -> cm
    // this yields our guess where it is now.
    return v_p->data.pos + delta_pos;
}
