/**********************************************************************************************
*
* Faller Car Controller - NRF52 Firmware
*
* (c) 2016
* Lukas Schrittwieser
*
* all rights reserved
*
* Redirect printf messages to nordic uart service which uses BLE to communicate with a host
*
**********************************************************************************************/

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include "app_uart.h"
#include "nordic_common.h"
#include "nrf_error.h"
#include "ble_nus.h"
#include "nrf_log_backend_serial.h"
#include "nrf_log_backend_interface.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_internal.h"


static void nrf_log_backend_nus_put(nrf_log_backend_t const * p_backend, nrf_log_entry_t * p_msg);
static void nrf_log_backend_nus_flush(nrf_log_backend_t const * p_backend);
static void nrf_log_backend_nus_panic_set(nrf_log_backend_t const * p_backend);



static ble_nus_t* nus_p = NULL;
static uint16_t   m_printf_conn_handle = BLE_CONN_HANDLE_INVALID;   // connection id where we should send the data to


const nrf_log_backend_api_t nrf_log_backend_nus_api = {
        .put       = nrf_log_backend_nus_put,
        .flush     = nrf_log_backend_nus_flush,
        .panic_set = nrf_log_backend_nus_panic_set,
};


NRF_LOG_BACKEND_DEF(nus_log_backend, nrf_log_backend_nus_api, NULL);


void retarget_init (ble_nus_t* nus)
{
    int32_t backend_id = -1;
    nus_p = nus;
    m_printf_conn_handle = BLE_CONN_HANDLE_INVALID;
    // register and enable this file as NRF LOG backend
    backend_id = nrf_log_backend_add(&nus_log_backend, NRF_LOG_SEVERITY_DEBUG);
    ASSERT(backend_id >= 0);
    nrf_log_backend_enable(&nus_log_backend);
}


void retarget_set_conn_handle (uint16_t h)
{
    m_printf_conn_handle = h;
}


/*
Note: This is legacy code. Nordic redirects printf itself (to RTT), so we redirect NRF_LOG to the Nordic uart service insted 
// overload according library stubs
int _write (int file, const char * p_char, int len)
{
    static uint8_t buf[BLE_NUS_MAX_DATA_LEN];
    static uint16_t buf_len = 0;
    UNUSED_PARAMETER(file);

    if (m_printf_conn_handle == BLE_CONN_HANDLE_INVALID) {
        // no connection, clear buffer and ignore data
        buf_len = 0;
        return len;
    }

    // copy data to buffer
    int i;
    for (i=0; i<len; i++) {
        buf[buf_len++] = p_char[i];
        // transmit once buffer is full or on a \n
        if ((buf_len >= BLE_NUS_MAX_DATA_LEN) || (p_char[i]=='\n')) {
            if (nus_p) {
               UNUSED_PARAMETER(ble_nus_data_send(nus_p, buf, &buf_len, m_printf_conn_handle));
            }
            buf_len = 0;
        }
    }

    return len;
}


int _read (int file, char * p_char, int len)
{
    UNUSED_PARAMETER(file);
    UNUSED_PARAMETER(p_char);
    // TODO
    return 0;
}
*/

#define NUS_LOG_BUF_LEN     BLE_NUS_MAX_DATA_LEN

static uint8_t nus_log_buf[NUS_LOG_BUF_LEN];
//static uint16_t buf_len = 0;


// assemble bytes in an a TX buffer
static void log_nus_serial_tx(void const * p_context, char const * buffer, size_t len)
{
    uint16_t length = len;
    if ((m_printf_conn_handle == BLE_CONN_HANDLE_INVALID) || (nus_p == NULL)) {
        // no connection, ignore data
         return;
    }

    // the log backend has filled the buffer with data, send it via nordic uart service
    UNUSED_PARAMETER(ble_nus_data_send(nus_p, buffer, &length, m_printf_conn_handle));
}


static void nrf_log_backend_nus_put(nrf_log_backend_t const * p_backend, nrf_log_entry_t * p_msg)
{
    nrf_log_backend_serial_put(p_backend, p_msg, nus_log_buf, NUS_LOG_BUF_LEN, log_nus_serial_tx);
}

static void nrf_log_backend_nus_flush(nrf_log_backend_t const * p_backend)
{
    // TODO: should trigger a TX of the buffer used in log_nus_serial_tx
}

static void nrf_log_backend_nus_panic_set(nrf_log_backend_t const * p_backend)
{

}

