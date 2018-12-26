/**********************************************************************************************
*
* Faller Car Controller - NRF52 Firmware
*
* (c) 2016
* Lukas Schrittwieser
*
* all rights reserved
*
* battery_service.c: interfacing with nordic battery service to report our battery status
*
**********************************************************************************************/

#include <string.h>
#include "ble_bas.h"
#include "app_util_platform.h"
#include "hw.h"



/**********************************************************************************************
*   G L O B A L S                                                                            */

// internal data storage for the service
BLE_BAS_DEF(m_bas);



/**********************************************************************************************
*   L O C A L   P R O T O T Y P E S                                                          */




/**********************************************************************************************
*   I M P L E M E N T A T I O N                                                              */



void bat_init()
{
    ble_bas_init_t init;
    memset((void*)&init,0,sizeof(init));

    //init.evt_handler = &bat_event_handler,  /**< Event handler to be called for handling events in the Battery Service. */
    init.support_notification = true,      /**< TRUE if notification of Battery Level measurement is supported. */
    init.p_report_ref = NULL;               /**< If not NULL, a Report Reference descriptor with the specified value will be added to the Battery Level characteristic */
    init.initial_batt_level = 100;          /**< Initial battery level */

     // Here the sec level for the Battery Service can be changed/increased.
    init.bl_cccd_wr_sec = SEC_OPEN;
    init.bl_rd_sec = SEC_OPEN;
    init.bl_report_rd_sec = SEC_OPEN;

    uint32_t ret = ble_bas_init(&m_bas, &init);
    APP_ERROR_CHECK(ret);


}

void bat_event (ble_evt_t * p_ble_evt)
{
    ble_bas_on_ble_evt(&m_bas, p_ble_evt);
}


// update battery status according to measured voltage in mV
void bat_update(uint32_t v_bat)
{
    uint8_t battery_level = (uint8_t)(100*(v_bat-2800)/1400);   // TODO: replace linear interpolation

    uint32_t err_code = ble_bas_battery_level_update(&m_bas, battery_level, BLE_CONN_HANDLE_ALL);
    if ((err_code != NRF_SUCCESS) && (err_code != NRF_ERROR_BUSY) && (err_code != NRF_ERROR_RESOURCES) &&
        (err_code != NRF_ERROR_FORBIDDEN) && (err_code != NRF_ERROR_INVALID_STATE) &&
        (err_code != BLE_ERROR_GATTS_SYS_ATTR_MISSING)) {
        APP_ERROR_HANDLER(err_code);
    }

}



