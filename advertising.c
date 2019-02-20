/*******************************************************************************************************************************************
*
* Faller Car Controller - NRF52 Firmware
*
* (c) 2019
* Lukas Schrittwieser
*
* all rights reserved
*
* BLE Advertising: broadcasts live status data (pos, speed, etc) as customer specific advertising data.
*
*******************************************************************************************************************************************/

#include "nordic_common.h"
#include "nrf.h"
#include "ble_hci.h"
#include "ble_advdata.h"
#include "ble_advertising.h"
#include "advertising.h"


/*******************************************************************************************************************************************
*   S T A T I C
*/

// BLE advertisement data (we need this for periodic updates of our custom status shipped along with the scan)
static struct adv_status_data   adv_manuf_data_data;
static ble_advdata_manuf_data_t adv_manuf_data;


/*******************************************************************************************************************************************
*   L O C A L   P R O T O T Y P E S
*/

static void advertising_fill_manufacturer_data();



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

    //memset(&srdata, 0, sizeof(srdata));
    srdata.include_ble_device_addr = true;
    srdata.p_manuf_specific_data = &adv_manuf_data;

    // need to pass the max length to the encoder in the length field. Thank's nordic for making it so plain obvious, took me only a couple of hours...
    m_advertising.adv_data.scan_rsp_data.len = BLE_GAP_ADV_SET_DATA_SIZE_MAX;
    // store the updated information in the same buffer where it was placed upon initialization
    err_code = ble_advdata_encode(&srdata, m_advertising.enc_scan_rsp_data, &m_advertising.adv_data.scan_rsp_data.len);
    APP_ERROR_CHECK(err_code);
}


static void advertising_fill_manufacturer_data()
{
     // update the status message sent with broad cast messages when advertising
    adv_manuf_data_data.pos = l2_get_pos();
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
    //adv_manuf_data_data.ref_speed = l2_get_ref_speed();

}
