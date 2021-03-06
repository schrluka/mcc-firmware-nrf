/*******************************************************************************************************************************************
*
* Faller Car Controller - NRF52 Firmware
*
* (c) 2019
* Lukas Schrittwieser
*
* all rights reserved
*
* BLE Advertising: broadcasts live status data (pos, speed, etc.) as customer specific advertising data.
*
*******************************************************************************************************************************************/

#include "nordic_common.h"
#include "nrf.h"
#include "nrf_sdh.h"
#include "nrf_sdh_soc.h"
#include "nrf_sdh_ble.h"
#include "nrf_log.h"
#include "ble_hci.h"
#include "ble_advdata.h"
#include "ble_advertising.h"
#include "app_error.h"
#include "advertising.h"
#include "model_car_ble_service.h"
#include "main.h"
#include "layer2.h"
#include "layer1.h"


/*******************************************************************************************************************************************
*   S T A T I C
*/

// BLE advertisement data (we need this for periodic updates of our custom status shipped along with the scan)
static struct adv_status_data   adv_manuf_data_data;
static ble_advdata_manuf_data_t adv_manuf_data;

// advertised UUIDs
static ble_uuid_t m_adv_uuids[] = {             /**< Universally unique service identifier we advertise */
//  {BLE_UUID_NUS_SERVICE, NUS_SERVICE_UUID_TYPE},  // announcing two UUIDs raises an exception (too long for packet?)
  {BLE_UUID_MODEL_CAR_SERVICE, BLE_UUID_TYPE_VENDOR_BEGIN}};  

#define VALUE_TO_STRING(x) #x
#define VALUE(x) VALUE_TO_STRING(x)
#define VAR_NAME_VALUE(var) #var "="  VALUE(var)

/* Some example here */
#pragma message(VAR_NAME_VALUE(BLE_ADV_BLE_OBSERVER_PRIO))

BLE_ADVERTISING_DEF(m_advertising);                                                 /**< Advertising module instance. */

static int32_t track_change_pos = 0;
static uint8_t next_track_id = 0;
static bool track_change_pending = false;



/*******************************************************************************************************************************************
*   L O C A L   P R O T O T Y P E S
*/

static void advertising_fill_manufacturer_data();

static void on_adv_evt(ble_adv_evt_t ble_adv_evt);



/*******************************************************************************************************************************************
*   I M P L E M E N T A T I O N 
*/

/**@brief Function for initializing the Advertising functionality.
 */
 
void advertising_init(void)
{
    uint32_t               err_code;
    ble_advertising_init_t init;

    memset(&init, 0, sizeof(init));

    init.advdata.name_type          = BLE_ADVDATA_SHORT_NAME;
    init.advdata.short_name_len     = 6;  // how many bytes of the name to send (at most)
    init.advdata.include_appearance = false;
    init.advdata.flags              = BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE;
    
    init.advdata.uuids_complete.uuid_cnt = sizeof(m_adv_uuids) / sizeof(m_adv_uuids[0]);
    init.advdata.uuids_complete.p_uuids  = m_adv_uuids;

    init.srdata.name_type = BLE_ADVDATA_NO_NAME;
    init.srdata.include_ble_device_addr = true;
    init.srdata.p_manuf_specific_data = &adv_manuf_data;  // our manufacturer specific data goes to the scan response, so we can utilize the full 31 bytes
   
    // manufacturer specific data (used to transmit status infos)
    advertising_fill_manufacturer_data();

    // capsulate our manufacturer specific data
    adv_manuf_data.data.p_data = (void*)(&adv_manuf_data_data);
    adv_manuf_data.data.size = sizeof(adv_manuf_data_data);
    adv_manuf_data.company_identifier = 0xFFFF; // value indicates 'no valid company' and can be used for testing
     
    //init.config.ble_adv_extended_enabled = true;   //this allows much longer adv packets -> we can send more manufacturer specific data?!
    init.config.ble_adv_fast_enabled  = true;
    init.config.ble_adv_fast_interval = APP_ADV_INTERVAL;
    init.config.ble_adv_fast_timeout  = APP_ADV_DURATION;
    init.evt_handler = on_adv_evt;
    
    err_code = ble_advertising_init(&m_advertising, &init);
    APP_ERROR_CHECK(err_code);

    ble_advertising_conn_cfg_tag_set(&m_advertising, APP_BLE_CONN_CFG_TAG);
} 


/**@brief Function for starting advertising.
 */
void advertising_start(void)
{
    uint32_t err_code = ble_advertising_start(&m_advertising, BLE_ADV_MODE_FAST);
    APP_ERROR_CHECK(err_code);
}


/**@brief Update manufacturer specific data (status info) with recent values from layer 2 (motion control)
 */
void advertising_update(void)
{
    //static ble_advdata_t advdata;  
    static ble_advdata_t srdata = {0}; // used to collect data for the encoder 
    uint32_t err_code;

    advertising_fill_manufacturer_data();

    srdata.include_ble_device_addr = true;
    srdata.p_manuf_specific_data = &adv_manuf_data;

    // need to pass the max length to the encoder in the length field. Thank's nordic for making it so plain obvious, took me only a couple of hours...
    m_advertising.adv_data.scan_rsp_data.len = BLE_GAP_ADV_SET_DATA_SIZE_MAX;
    // store the updated information in the same buffer where it was placed upon initialization
    err_code = ble_advdata_encode(&srdata, m_advertising.enc_scan_rsp_data, &m_advertising.adv_data.scan_rsp_data.len);
    APP_ERROR_CHECK(err_code);
}


void advertising_schedule_track_id_change(uint8_t new_track_id)
{
    // remember that the track id that we broadcast has to change once our tail reaches the position at which we received the function call
    track_change_pos = l2_get_pos() + mcs_get_dist_coil2tail();
    NRF_LOG_INFO("changing to id %d at %d", (int)new_track_id, (int)track_change_pos);
    next_track_id = new_track_id;
    track_change_pending = true;
}


static void advertising_fill_manufacturer_data()
{
    int32_t pos = l2_get_pos();
    // update the status message sent with broadcast messages when advertising
    adv_manuf_data_data.pos = pos - mcs_get_dist_coil2tail();  // report the position of our tail
    adv_manuf_data_data.speed = (int16_t)l2_get_speed();
    adv_manuf_data_data.u_bat = (int16_t)l2_get_u_bat();
    adv_manuf_data_data.delta_pos = l2_get_pos_delta();
    int32_t d = l2_get_dist();
    if (d > 127) {
        d = 127;
    }
    if (d < -128) {
        d = -128;
    }
    adv_manuf_data_data.dist = d;
    d = l1_get_i_ref();
    if (d < 0) {
        d = 0;
    }
    if (d > 255) {
        d = 255;
    }
    adv_manuf_data_data.i_ref = (uint8_t)d;

    if (track_change_pending && (pos >= track_change_pos)) {
        adv_manuf_data_data.track_id = next_track_id;
        track_change_pending = false;
    }

    //adv_manuf_data_data.ref_speed = l2_get_ref_speed();
}


/**@brief Function for handling advertising events.
 *
 * @details This function will be called for advertising events which are passed to the application.
 *
 * @param[in] ble_adv_evt  Advertising event.
 */
static void on_adv_evt(ble_adv_evt_t ble_adv_evt)
{
    uint32_t err_code;

    switch (ble_adv_evt)
    {
        case BLE_ADV_EVT_FAST:
            
            break;
        case BLE_ADV_EVT_IDLE:
            // Note: the app keeps on scanning without connecting using the broadcasts to gain info about
            // available devices
            break;
        default:
            break;
    }
}
