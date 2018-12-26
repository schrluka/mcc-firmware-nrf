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


static ble_nus_t* nus_p = NULL;
static uint16_t   m_printf_conn_handle = BLE_CONN_HANDLE_INVALID;   // connection id where we should send the data to

void retarget_init (ble_nus_t* nus)
{
    nus_p = nus;
    m_printf_conn_handle = BLE_CONN_HANDLE_INVALID;
}


void retarget_set_conn_handle (uint16_t h)
{
    m_printf_conn_handle = h;
}


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
