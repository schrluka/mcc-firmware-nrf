
#include <string.h>
#include <stdio.h>

#include <ble_gatts.h>
#include "nordic_common.h"
#include "ble_srv_common.h"
#include "nrf_sdh_ble.h"
#include "app_util.h"

#include "layer2.h"
#include "model_car_ble_service.h"



/*******************************************************************************************************************************************
*   D E F I N E S
*/

// Units specified in the BLE assigned numbers list
// https://www.bluetooth.com/specifications/assigned-numbers/units
#define UNIT_1          0x2700      // unit less
#define UNIT_VOLT       0x2728
#define UNIT_M_PER_S    0x2712
#define UNIT_M          0x2701
#define UNIT_PERCENT    0x27AD


// total number of characteristics we have
#define N_CHARS      (sizeof(chars)/sizeof(chars[0]))

// values for mcs_direction
#define MCS_FWD     0
#define MCS_STOP    1
#define MCS_BWD     2


// Additional BLE UUIDs not defined by nordic, see https://www.bluetooth.com/specifications/gatt
#define BLE_UUID_VALID_RANGE         0x2906



struct char_config_s {
    uint16_t    uuid;       // we use the short 16 bit UUIDs for the chars
    bool        we;         // can the phone or computer change this value
    void*       p_value;    // pointer to storage of the chars value
    size_t      len;        // length in bytes, limited by ble to ~500
    void        (*wr_cb)(); // callback on writes by BLE client (e.g. phone)
    uint16_t    unit;       // ble assigned number indicating unit
    int8_t      exp;        // base 10 exponent for representation
    bool        is_signed;  // true: indicate a signed value
    char*       user_desc;  // human readable string
    void*       p_minmax_val;  // pointer to allowed min. & max value (reported via descriptor to phone) same data type as value
};



/*******************************************************************************************************************************************
*   P R O T O T Y P E S
*/


void mcs_on_ble_evt(ble_evt_t const * p_ble_evt, void * p_context);

void mcs_direction_written();



/*******************************************************************************************************************************************
*   G L O B A L S
*/

// storage space for all elements in the GATT data base (providing it here makes it easy to write them)
static volatile int16_t mcs_speed   = 0;    // measured vehicle speed in mm/sec
static volatile uint16_t mcs_u_bat  = 0;    // battery voltage in mV
static volatile int32_t mcs_pos    = 0;     // vehicle position in cm
static volatile int8_t mcs_direction = MCS_FWD; // which direction we drive

static volatile int8_t minmax_direction[2] = {0,2};

// information needed to register all characteristics with the softdevice
const struct char_config_s chars[] = {
    {.uuid=0xBE00, .we=false, .p_value=(void*)&mcs_speed, .len=sizeof(mcs_speed),
        .unit=UNIT_M_PER_S, .exp=-3, .is_signed=true, .user_desc="speed"},
    {.uuid=0xBE01, .we=false, .p_value=(void*)&mcs_u_bat, .len=sizeof(mcs_u_bat),
        .unit=UNIT_VOLT, .exp=-3, .user_desc="battery voltage"},
    {.uuid=0xBE02, .we=false, .p_value=(void*)&mcs_pos, .len=sizeof(mcs_pos), .unit=UNIT_M, .exp=-2, .is_signed=true},
    {.uuid=0xBE03, .we=true, .p_value=(void*)&mcs_direction, .len=sizeof(mcs_direction),
        .wr_cb=&mcs_direction_written, .p_minmax_val=(void*)&minmax_direction},
};

// storage for the handles used (and also assigned) by the softdevice which identify everything related to this service
// service handle used by the softdevice to identify our service
static volatile uint16_t mcs_service_handle = BLE_GATT_HANDLE_INVALID;
// connection identification
static volatile uint16_t mcs_conn_handle = 0;
// handles of almost everything related to a characteristic
ble_gatts_char_handles_t mcs_char_handles[N_CHARS];
// handles of valid_range descriptors, each char can have one, but does not have to
uint16_t mcs_valid_range_descriptors[N_CHARS];





/*******************************************************************************************************************************************
*   I M P L E M E N T A T I O N
*/


/**@brief Function for handling the Connect event.
 *
 * @param[in]   p_ble_evt   Event received from the BLE stack.
 */
static void on_connect(ble_evt_t const * p_ble_evt)
{
    mcs_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
}


/**@brief Function for handling the Disconnect event.
 *
 * @param[in]   p_ble_evt   Event received from the BLE stack.
 */
static void on_disconnect(ble_evt_t const * p_ble_evt)
{
    UNUSED_PARAMETER(p_ble_evt);
    mcs_conn_handle = BLE_CONN_HANDLE_INVALID;
}


/**@brief Function for handling the Write event.
 *
 * @param[in]   p_ble_evt   Event received from the BLE stack.
 */
static void on_write(ble_evt_t const * p_ble_evt)
{
    ble_gatts_evt_write_t * p_evt_write = &p_ble_evt->evt.gatts_evt.params.write;

    /* Need to support notifications on client-to-server data transfer ?!
    if ((p_evt_write->handle == p_nus->rx_handles.cccd_handle)
        &&
        (p_evt_write->len == 2)
       )
    {
        if (ble_srv_is_notification_enabled(p_evt_write->data))
        {
            p_nus->is_notification_enabled = true;
        }
        else
        {
            p_nus->is_notification_enabled = false;
        }
    }
    else */
    // scan which mcs was written
    for (size_t i=0; i<N_CHARS; i++) {
        if ((p_evt_write->handle == mcs_char_handles[i].value_handle)) {
            // do we have a callback registered?
            if (chars[i].wr_cb != NULL) {
                chars[i].wr_cb();   // call it
            }
        }
    }
}


void mcs_on_ble_evt(ble_evt_t const * p_ble_evt, void * p_context)
{
    if (p_ble_evt == NULL)
    {
        return;
    }

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            on_connect(p_ble_evt);
            break;

        case BLE_GAP_EVT_DISCONNECTED:
            on_disconnect(p_ble_evt);
            break;
        case BLE_GATTS_EVT_WRITE:
            on_write(p_ble_evt);
            break;
        default:
            break;
    }
}


// add our characteristics, can only be called after the service has been added
static uint32_t char_add()
{
    uint32_t            err_code = NRF_SUCCESS;
    ble_gatts_char_md_t char_md;
    ble_gatts_attr_t    attr_char_value;
    ble_uuid_t          ble_uuid;
    ble_gatts_char_pf_t pf;
    //ble_gatts_attr_md_t cccd_md;
    ble_gatts_attr_md_t attr_md;

    memset(mcs_valid_range_descriptors, 0, sizeof(mcs_valid_range_descriptors));

    // Add all characteristics
    for (int i=0; i<N_CHARS; i++) {

        /* not sure we need this
        if (chars[i].we) {
            // create Client Characteristic Configuration meta data
            memset(&cccd_md, 0, sizeof(cccd_md));
            cccd_md.vloc = BLE_GATTS_VLOC_STACK;    // tell soft device to store the CCCD
            cccd_md
            BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.read_perm);
            BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.write_perm);
        } */

        memset(&char_md, 0, sizeof(char_md));
        // configure the characteristic's meta data
        char_md.char_props.read   = 1;
        char_md.char_props.notify = 0;  // we don't support notifications
        char_md.p_char_user_desc  = NULL;
        char_md.p_char_pf         = NULL;
        char_md.p_user_desc_md    = NULL;
        char_md.p_cccd_md         = NULL; //&cccd_md;
        char_md.p_sccd_md         = NULL;

        BLE_UUID_BLE_ASSIGN(ble_uuid, chars[i].uuid);

        if (chars[i].unit != 0) {
            // add a presentation format descriptor
            memset(&pf, 0, sizeof(pf));
            switch(chars[i].len) {  // specify data format according to https://www.bluetooth.com/specifications/gatt/viewer?attributeXmlFile=org.bluetooth.descriptor.gatt.characteristic_presentation_format.xml
            case 1: pf.format = 4; break;   // 8 bits unsigned
            case 2: pf.format = 6; break;   // 16 bits unsigned
            case 3: pf.format = 7; break;   // 24 bits unsigned
            case 4: pf.format = 8; break;   // 32 bits unsigned
            default: pf.format = 27;         // opaque struct
            }
            if (chars[i].is_signed) {
                pf.format += 8; // signed values have an offset of 8 in the code
            }
            pf.exponent = chars[i].exp;
            pf.name_space = 1;
            pf.unit = chars[i].unit;
            char_md.p_char_pf = &pf;    // hook it up with the characteristic
        }

        if (chars[i].user_desc != NULL) {
            char_md.p_char_user_desc = (void*)chars[i].user_desc;
            char_md.char_user_desc_size = strlen((void*)chars[i].user_desc);
            char_md.char_user_desc_max_size = char_md.char_user_desc_size;
        }

        memset(&attr_md, 0, sizeof(attr_md));

        //BLE_GAP_CONN_SEC_MODE_SET_OPEN(&init.battery_level_char_attr_md.cccd_write_perm);
        BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
        if (chars[i].we) {
            BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.write_perm);
            char_md.char_props.write = 1;           // enable indications
            //char_md.char_props.write_wo_resp = 1; // could enabled notifications
        } else {
            BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&attr_md.write_perm);
            }
        attr_md.vloc       = BLE_GATTS_VLOC_USER;   // tell the softdevice that we keep the variable in our global memory
        attr_md.rd_auth    = 0;
        attr_md.wr_auth    = 0;
        attr_md.vlen       = 0;

        memset(&attr_char_value, 0, sizeof(attr_char_value));

        attr_char_value.p_uuid    = &ble_uuid;
        attr_char_value.p_attr_md = &attr_md;
        attr_char_value.init_len  = chars[i].len;
        attr_char_value.init_offs = 0;
        attr_char_value.max_len   = chars[i].len;
        attr_char_value.p_value   = chars[i].p_value;

        // register this char with the soft device, this will allocate handles, etc
        err_code = sd_ble_gatts_characteristic_add(mcs_service_handle, &char_md,
                                                   &attr_char_value,
                                                   &(mcs_char_handles[i]));
        if (err_code != NRF_SUCCESS)
            return err_code;

        // now that the char is added we can add further descriptors to it
        if ((chars[i].p_minmax_val != NULL)) {
            // add a valid_range descriptor
            BLE_UUID_BLE_ASSIGN(ble_uuid, BLE_UUID_VALID_RANGE);

            memset(&attr_md, 0, sizeof(attr_md));
            BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
            BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&attr_md.write_perm);
            attr_md.vloc    = BLE_GATTS_VLOC_USER; // we keep the entry in our memory
            attr_md.rd_auth = 0;
            attr_md.wr_auth = 0;
            attr_md.vlen    = 0;

            memset(&attr_char_value, 0, sizeof(attr_char_value));
            attr_char_value.p_uuid    = &ble_uuid;
            attr_char_value.p_attr_md = &attr_md;
            attr_char_value.init_len  = 2*chars[i].len; // 2* because we have a min. and a max. value
            attr_char_value.init_offs = 0;
            attr_char_value.max_len   = attr_char_value.init_len;
            attr_char_value.p_value   = chars[i].p_minmax_val;

            err_code = sd_ble_gatts_descriptor_add(mcs_char_handles[i].value_handle,
                                                   &attr_char_value,
                                                   &(mcs_valid_range_descriptors[i]));
            if (err_code != NRF_SUCCESS)
                return err_code;
        }


    }
    return err_code;
}


uint32_t mcs_init()
{
   uint32_t   err_code;
    ble_uuid_t ble_uuid;

    // Add our service as a primary one
    BLE_UUID_BLE_ASSIGN(ble_uuid, BLE_UUID_MODEL_CAR_SERVICE);
    err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY, &ble_uuid, (void*)&mcs_service_handle);
    if (err_code != NRF_SUCCESS) {
        return err_code;
    }

    // Add our characteristics
    return char_add();

    // register our event listener with the soft device
    NRF_SDH_BLE_OBSERVER(m_mcs_observer, 3, mcs_on_ble_evt, NULL);  // sencond parameter is the priority level, 3 is lowest

}


// update the GATT database with status information
uint32_t mcs_update()
{
    // gather information
    int32_t speed = l2_get_speed();

    // limit
    if (speed > 0x7FFF)
        speed = 0x7FFF;
    if (speed < -0x8000)
        speed = -0x8000;
    mcs_speed = speed;

    mcs_u_bat = (uint16_t)l2_get_u_bat();

    mcs_pos = l2_get_pos();

    return NRF_SUCCESS;
}

// called when the client (e.g. phone) has written mcs_direction
void mcs_direction_written()
{
    printf("mcs_direction: %d\n", (int)mcs_direction);
}
