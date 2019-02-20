/*******************************************************************************************************************************************
*
* Faller Car Controller - NRF52 Firmware
*
* (c) 2016
* Lukas Schrittwieser
*
* all rights reserved
*
* Model Car Service: BLE Service, nonvolatile storage of characteristics (config data)
*
*******************************************************************************************************************************************/


#include <string.h>
#include <stdio.h>

#include <ble_gatts.h>
#include "nordic_common.h"
#include "ble_srv_common.h"
#include "nrf_sdh_ble.h"
#include "app_util.h"
#include "fds.h"
#include "nrf_log.h"

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
#define UNIT_M_PER_S2   0x2713      // meters per second squared
#define UNIT_M          0x2701
#define UNIT_WB         0x272C      // magnetic flux: weber = volt seconds
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
    void*       p_value;    // pointer to storage of the char's value
    size_t      len;        // length in bytes, limited by ble to ~500
    void        (*wr_cb)(); // callback on writes by BLE client (e.g. phone)
    uint16_t    unit;       // ble assigned number indicating unit
    int8_t      exp;        // base 10 exponent for representation
    bool        is_signed;  // true: indicate a signed value
    char*       user_desc;  // human readable string
    void*       p_minmax_val;  // pointer to allowed min. & max value (reported via descriptor to phone) same data type as value
    bool        fds_store;  // if true, value will be stored in flash using FDS driver
};


/* Array to map FDS return values to strings. */
char const * fds_err_str[] =
{
    "FDS_SUCCESS",
    "FDS_ERR_OPERATION_TIMEOUT",
    "FDS_ERR_NOT_INITIALIZED",
    "FDS_ERR_UNALIGNED_ADDR",
    "FDS_ERR_INVALID_ARG",
    "FDS_ERR_NULL_ARG",
    "FDS_ERR_NO_OPEN_RECORDS",
    "FDS_ERR_NO_SPACE_IN_FLASH",
    "FDS_ERR_NO_SPACE_IN_QUEUES",
    "FDS_ERR_RECORD_TOO_LARGE",
    "FDS_ERR_NOT_FOUND",
    "FDS_ERR_NO_PAGES",
    "FDS_ERR_USER_LIMIT_REACHED",
    "FDS_ERR_CRC_CHECK_FAILED",
    "FDS_ERR_BUSY",
    "FDS_ERR_INTERNAL",
};

/* Array to map FDS events to strings. */
static char const * fds_evt_str[] =
{
    "FDS_EVT_INIT",
    "FDS_EVT_WRITE",
    "FDS_EVT_UPDATE",
    "FDS_EVT_DEL_RECORD",
    "FDS_EVT_DEL_FILE",
    "FDS_EVT_GC",
};

// we use just a single fds file identified by this magic constant
#define CONFIG_FILE   0x1337



/*******************************************************************************************************************************************
*   P R O T O T Y P E S
*/


static void mcs_on_ble_evt(ble_evt_t const * p_ble_evt, void * p_context);

static void fds_evt_handler(fds_evt_t const * p_evt);

void mcs_direction_written();

static void mcs_v_max_written();

static void fds_poll();

static void store_char(const struct char_config_s* char_p);



/*******************************************************************************************************************************************
*   G L O B A L S
*/

// storage space for all elements in the GATT data base (providing it here makes it easy to write them)
static volatile int16_t mcs_speed   = 0;    // measured vehicle speed in mm/sec
static volatile uint16_t mcs_u_bat  = 0;    // battery voltage in mV
static volatile int32_t mcs_pos    = 0;     // vehicle position in cm
static volatile int8_t mcs_direction = MCS_FWD; // which direction we drive
static volatile uint16_t mcs_v_max = 255;     // max. speed in mm/s
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
    {.uuid=0xBE04, .we=true, .p_value=(void*)&mcs_v_max, .len=sizeof(mcs_v_max),
        .wr_cb=&mcs_v_max_written, .unit=UNIT_M_PER_S, .exp=-3, .fds_store=true},
};

// storage for the handles used (and also assigned) by the softdevice which identify everything related to this service
static volatile uint16_t mcs_service_handle = BLE_GATT_HANDLE_INVALID;
// connection identification
static volatile uint16_t mcs_conn_handle = 0;
// handles of almost everything related to a characteristic
ble_gatts_char_handles_t mcs_char_handles[N_CHARS];
// handles of valid_range descriptors, each char can have one, but does not have to
uint16_t mcs_valid_range_descriptors[N_CHARS];


// Flag to check flash data system initialization status
static volatile bool m_fds_initialized = false;

// set when initialization is finished to load all values from flash to memory
static volatile bool m_fds_read_values = false; 


/*******************************************************************************************************************************************
*   I M P L E M E N T A T I O N
*/


/**@brief Function for handling the Write event.
 *
 * @param[in]   p_ble_evt   Event received from the BLE stack.
 */
static void on_write(ble_evt_t const * p_ble_evt)
{
    ble_gatts_evt_write_t const * p_evt_write = &p_ble_evt->evt.gatts_evt.params.write;

    /* Needed to support notifications on client-to-server data transfer ?!
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
            // store value in flash?
            if (chars[i].fds_store) {
                NRF_LOG_INFO("storing updated value in flash");
                store_char(&chars[i]);
            }
        }
    }
}


static void mcs_on_ble_evt(ble_evt_t const * p_ble_evt, void * p_context)
{
    if (p_ble_evt == NULL)
    {
        return;
    }

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            mcs_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
            break;

        case BLE_GAP_EVT_DISCONNECTED:
            mcs_conn_handle = BLE_CONN_HANDLE_INVALID;
            // TODO: potentially start a flash garbage collection here?
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

    m_fds_initialized = false;

    // Add our service as a primary one
    BLE_UUID_BLE_ASSIGN(ble_uuid, BLE_UUID_MODEL_CAR_SERVICE);
    err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY, &ble_uuid, (void*)&mcs_service_handle);
    if (err_code != NRF_SUCCESS) {
        return err_code;
    }

    // Add our characteristics
    err_code = char_add();
    if (err_code != NRF_SUCCESS) {
        return err_code;
    }

    // register our event listener with the soft device
    NRF_SDH_BLE_OBSERVER(m_mcs_observer, 3, mcs_on_ble_evt, NULL);  // second parameter is the priority level, 3 is lowest

    // start flash storage init, this might take a while, we'll receive a callback
    APP_ERROR_CHECK(fds_register(fds_evt_handler));
    err_code = fds_init();
    
    return err_code;   
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

    // TODO: implement notifications

    // check for background tasks of the flash system
    fds_poll();

    return NRF_SUCCESS;
}


// called periodically from main loop to perform background & houskeeping tasks
static void fds_poll()
{
    ret_code_t rc;
    fds_record_desc_t desc;
    fds_find_token_t  tok;
    fds_flash_record_t config;

    if (m_fds_read_values) {
        m_fds_read_values = false;
        // go through all characteristics and load their stored value from flash
        for (int i=0; i<N_CHARS; i++) {
            // check if this characteristic is stored in flash
            if (!chars[i].fds_store) {
                continue;
            }
            // try to find the corresponding record, using the UUID as key
            memset(&desc, sizeof(desc), 0);
            memset(&tok, sizeof(tok), 0);
            rc = fds_record_find(CONFIG_FILE, chars[i].uuid, &desc, &tok);
            if (rc != FDS_SUCCESS) {
                NRF_LOG_ERROR("no flash entry for char 0x%04x", (unsigned long)chars[i].uuid);
                // TODO: create an entry?
                continue;
            }
            // get data from flash
            memset(&config, sizeof(config), 0);
            rc = fds_record_open(&desc, &config);
            if (rc != FDS_SUCCESS) {
                NRF_LOG_ERROR("char 0x%04x: fds error %s", (unsigned long)chars[i].uuid, fds_err_str[rc]);
                // TODO: create an entry?
                continue;
            }
            // TODO: validate length of entry in flash and expected value
            memcpy(chars[i].p_value, config.p_data, chars[i].len);
            // done reading, close the entry
            rc = fds_record_close(&desc);
            APP_ERROR_CHECK(rc);
            // trigger the characteristic's write callback because its value might have changed
            if (chars[i].wr_cb != NULL) {
                chars[i].wr_cb();
            }
        }
    }
}


// store a characteristic's value in flash 
static void store_char(const struct char_config_s* char_p)
{
    static fds_record_t rec = {0}; // must be static for th FDS lib
    fds_record_desc_t desc = {0};
    fds_find_token_t  tok = {0};
    ret_code_t rc;    

    rec.file_id           = CONFIG_FILE;
    rec.key               = char_p->uuid;
    rec.data.p_data       = char_p->p_value;
    // The length of a record is always expressed in 4-byte units (words). 
    rec.data.length_words = (char_p->len + 3) / 4;
    
    // try to find the record
    rc = fds_record_find(CONFIG_FILE, rec.key, &desc, &tok);

    if (rc == FDS_SUCCESS)
    {
        // entry exists, update it
        rc = fds_record_update(&desc, &rec);
        APP_ERROR_CHECK(rc);
    }
    else
    {
        // entry not found, creating a new one
        NRF_LOG_INFO("creating flash entry for 0x%04x", (unsigned long)(char_p->uuid));

        rc = fds_record_write(&desc, &rec);
        APP_ERROR_CHECK(rc);
    }

    // FIXME: this is dangerours: another write could be started before the first finished, causing data gargabe because rec needs to be static.
}


// called when the client (e.g. phone) has written mcs_direction
void mcs_direction_written()
{
    NRF_LOG_INFO("mcs_direction: %d", (int)mcs_direction);
    // TODO: interface with layer 2 to set the new direction
}


void mcs_v_max_written()
{
    NRF_LOG_INFO("mcs_v_max: %d\n", (int)mcs_v_max);
    l2_set_max_speed(mcs_v_max);
}


// callback from the flash data storage system
static void fds_evt_handler(fds_evt_t const * p_evt)
{
    NRF_LOG_INFO("Event: %s received (%s)",
                  fds_evt_str[p_evt->id],
                  fds_err_str[p_evt->result]);

    switch (p_evt->id)
    {
        case FDS_EVT_INIT:
        {
            if (p_evt->result == FDS_SUCCESS) {
                m_fds_initialized = true;
                m_fds_read_values = true;
            }
            break;
        }

        case FDS_EVT_WRITE:
        {
            if (p_evt->result == FDS_SUCCESS) {
                NRF_LOG_INFO("Record ID:\t0x%04x",  p_evt->write.record_id);
                NRF_LOG_INFO("File ID:\t0x%04x",    p_evt->write.file_id);
                NRF_LOG_INFO("Record key:\t0x%04x", p_evt->write.record_key);
            }
        } break;

        case FDS_EVT_DEL_RECORD:
        {
            if (p_evt->result == FDS_SUCCESS) {
                NRF_LOG_INFO("Record ID:\t0x%04x",  p_evt->del.record_id);
                NRF_LOG_INFO("File ID:\t0x%04x",    p_evt->del.file_id);
                NRF_LOG_INFO("Record key:\t0x%04x", p_evt->del.record_key);
            }
            
        } break;

        default:
            break;
    }
}