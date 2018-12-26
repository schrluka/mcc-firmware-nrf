/* Faller Car Ctrl - NRF52 Bootloader for firmware update over BLE
 *
 * (c) 2016
 * Lukas Schrittwieser
 */

/* Copyright (c) 2014 Nordic Semiconductor. All Rights Reserved.
 *
 * The information contained herein is property of Nordic Semiconductor ASA.
 * Terms and conditions of usage are described in detail in NORDIC
 * SEMICONDUCTOR STANDARD SOFTWARE LICENSE AGREEMENT.
 *
 * Licensees are granted free, non-transferable use of the information. NO
 * WARRANTY of ANY KIND is provided. This heading must NOT be removed from
 * the file.
 *
 */
 #ifndef CUSTOM_BOARD_H
#define CUSTOM_BOARD_H

#ifdef __cplusplus
extern "C" {
#endif

// LED definitions
#define LEDS_NUMBER    1


//#define LED_DBG          1
#define LED_FAKE        28

//#define LEDS_LIST { LED_FAKE, LED_BACK, LED_LEFT, LED_RIGHT }
#define LEDS_LIST { LED_FAKE }

#define BSP_LED_0      LED_FAKE
//#define BSP_LED_1      LED_FAKE
//#define BSP_LED_2      LED_FAKE
//#define BSP_LED_3      LED_FAKE

#define LEDS_ACTIVE_STATE 0

//#define BSP_LED_0_MASK (1<<BSP_LED_0)
//#define BSP_LED_1_MASK (1<<BSP_LED_1)
//#define BSP_LED_2_MASK (1<<BSP_LED_2)
//#define BSP_LED_3_MASK (1<<BSP_LED_3)

//#define LEDS_MASK      (BSP_LED_0_MASK | BSP_LED_1_MASK | BSP_LED_2_MASK | BSP_LED_3_MASK)
#define LEDS_INV_MASK  (0)



#define BUTTONS_NUMBER 1

#define BUTTON_START   9
#define BUTTON_DFU     9
#define BUTTON_STOP    9
#define BUTTON_PULL    NRF_GPIO_PIN_PULLUP

#define BUTTONS_LIST { BUTTON_DFU }

//#define BSP_BUTTON_0   BUTTON_1
//#define BSP_BUTTON_1   BUTTON_2
//#define BSP_BUTTON_2   BUTTON_3
#define BSP_BUTTON_3   BUTTON_DFU

//#define BSP_BUTTON_0_MASK (1<<BSP_BUTTON_0)
//#define BSP_BUTTON_1_MASK (1<<BSP_BUTTON_1)
//#define BSP_BUTTON_2_MASK (1<<BSP_BUTTON_2)
#define BSP_BUTTON_3_MASK (1<<BSP_BUTTON_3)

//#define BUTTONS_MASK   BSP_BUTTON_3_MASK


#define BUTTONS_ACTIVE_STATE 0


// Low frequency clock source to be used by the SoftDevice
// As we don't have a dedicated LF crystal we use the internal RC osc and calibrate it every 16/4 = 4sec
/*
#define NRF_CLOCK_LFCLKSRC      {.source        = NRF_CLOCK_LF_SRC_RC,            \
                                 .rc_ctiv       = 16,                                \
                                 .rc_temp_ctiv  = 0,                                \
                                 .xtal_accuracy = NRF_CLOCK_LF_XTAL_ACCURACY_20_PPM}

*/

// UART pins (we don't use those, left over from demo code)
//#define RX_PIN_NUMBER   24
//#define TX_PIN_NUMBER   25
//#define CTS_PIN_NUMBER  26
//#define RTS_PIN_NUMBER  27
// disable hardware flow control
#define HWFC           false

#ifdef __cplusplus
}
#endif

#endif // PCA10040_H
