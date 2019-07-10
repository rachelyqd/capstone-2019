/*
 * Copyright 2018, Cypress Semiconductor Corporation or a subsidiary of Cypress Semiconductor 
 *  Corporation. All rights reserved. This software, including source code, documentation and  related 
 * materials ("Software"), is owned by Cypress Semiconductor  Corporation or one of its 
 *  subsidiaries ("Cypress") and is protected by and subject to worldwide patent protection  
 * (United States and foreign), United States copyright laws and international treaty provisions. 
 * Therefore, you may use this Software only as provided in the license agreement accompanying the 
 * software package from which you obtained this Software ("EULA"). If no EULA applies, Cypress 
 * hereby grants you a personal, nonexclusive, non-transferable license to  copy, modify, and 
 * compile the Software source code solely for use in connection with Cypress's  integrated circuit 
 * products. Any reproduction, modification, translation, compilation,  or representation of this 
 * Software except as specified above is prohibited without the express written permission of 
 * Cypress. Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO  WARRANTY OF ANY KIND, EXPRESS 
 * OR IMPLIED, INCLUDING,  BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED WARRANTIES OF MERCHANTABILITY 
 * AND FITNESS FOR A PARTICULAR PURPOSE. Cypress reserves the right to make changes to 
 * the Software without notice. Cypress does not assume any liability arising out of the application 
 * or use of the Software or any product or circuit  described in the Software. Cypress does 
 * not authorize its products for use in any products where a malfunction or failure of the 
 * Cypress product may reasonably be expected to result  in significant property damage, injury 
 * or death ("High Risk Product"). By including Cypress's product in a High Risk Product, the 
 *  manufacturer of such system or application assumes  all risk of such use and in doing so agrees 
 * to indemnify Cypress against all liability.
 */

/** @file
 *
 * Watch Sample Application for 2070X devices.
 *
 * This file implements 2070x embedded application controlled over UART or SPI.
 * This application act as a SPI slave device.
 * To use SPI transport, append TRANSPORT=SPI to the make target,
 * for example "demo.watch-PLATFORM_NAME TRANSPORT=SPI download"
 * Current version of the application exposes Audio Source and GATT/GAP
 * functionality.
 * MCU connected over UART or SPI can send commands to execute certain
 * functionality while configuration is local in the application including SDP
 * and GATT databases, as well as configuration of different activities like
 * inquiry advertisements or scanning.
 *
 * To demonstrate the app via SPI, work through the following steps.
 * 1. Plug the WICED Bluetooth ( 2070x ) evaluation board into your computer
 * 2. Build and download the application ( to the 2070x board )
 * 3. Build and download the hci_uart_spi_bridge application to another
 *    2070x board(SPI Master).
 * 4. Connect the SPI signals between master and slave.
 *
 *    |---------------------------------------------------|----------------------------|-------------------|
 *    |    2070x  Master          |       2070x Slave     |  CYW920719Q40EVB_01 Slave  | CYW9207x9WCDEVAL  |
 *    |---------------------------------------------------|----------------------------|-------------------|
 *    | SPI CLK    P36(J19.6)     |       P3(J22.8)       |          P38 (D13)         |     P10(J11.7)    |
 *    |---------------------------------------------------|----------------------------|-------------------|
 *    | SPI MOSI   P0(J22.6)      |       P0(J22.6)       |          P28 (D11)         |     P28(J12.3)    |
 *    |---------------------------------------------------|----------------------------|-------------------|
 *    | SPI MISO   P25(J19.4)     |       P25(J19.4)      |          P01 (D12)         |     P29(J12.4)    |
 *    |---------------------------------------------------|----------------------------|-------------------|
 *    | SPI CS     P26(J22.4)     |       P2(J22.5)       |          P07 (D10)         |     P01(J11.2)    |
 *    |---------------------------------------------------|----------------------------|-------------------|
 *    | SLAVE READY   P12(J22.7)  |       P15(J22.3)      |          P06 (D8)          |     P6(J11.5)     |
 *    |---------------------------------------------------|----------------------------|-------------------|
 *    | GND         J19.8         |       J19.8           |           J3.4             |      J11.10       |
 *    |---------------------------------------------------|----------------------------|-------------------|
 *
 * 5. Open the Client Control application. Select  the PUART port on the 2070x
 *    running the hci_uart_spi_bridge application at 115200 baud rate
 *    without flow control
 * 6. Reset the 2070x that runs the hci_uart_spi_bridge application.
 * 7. Reset the 2070x that runs the application.
 * 6. Use ClientControl application to send various commands
 *
 * The sample app performs as a Bluetooth A2DP source, AVRCP Controller/Target
 *
 * The sample Windows ClientControl application is provided to show sample
 * MCU implementation on Windows platform.
 *
 * Features demonstrated
 *  - WICED BT A2DP Source APIs
 *  - WICED BT AVRCP (Controller/Target) APIs
 *  - WICED BT GATT APIs
 *  - Handling of the UART WICED protocol
 *  - SDP and GATT descriptor/attribute configuration
 *
 * On startup this demo:
 *  - Initializes the Bluetooth sub system
 *  - Receive NVRAM information from the host
 *
 * BLE
 *  - To find Gatt devices: Click on the "Start Ble Discovery" button
 *  - To start adverterisements: Click on the "Start Adverts" button
 *  - To connect Ble device: Choose device from the drop down combo box and
 *    click "Connect" button
 *  - To discover services: Click on the "Discover Services" button
 *  - To discover characteristics: Enter the handles in the edit box and click
 *    on "Discover Characteristics"
 *  - To discover descriptors: Enter the handles in the edit box and click on
 *    "Discover Characteristics"
 *  - Enter the Handle and Hex Value to write to the remote device using buttons
 *     "Write" : Write hex value to remote handle
 *     "Write no rsp" : Write hex value without response to remote handle
 *     "Value Notify" : Write notification value to the remote handle
 *     "Value Indicate" : Write indication value to the remote handle
 *
 * iOS ANCS and AMS services
 *   - Select Pairable
 *   - Click on the Start Adverts button
 *   - To initiate pairing from peer, try reading firmware/software revision from GATT characteristics.
 *   - From the Light Blue application find and connect to Wiced device
 *   - Click on Listen for Notifications
 *   - On the trace dialog the Passkey will be displayed that needs to be
 *     entered on the iOS device for pairing.
 *   - Use AVRC buttons to control the AMS music
 *   - Watch incoming notifications, try to send positive and negative acks.
 *
 * BR/EDR
 * - To find BR/EDR devices: Click on "Start BR/EDR Discovery"
 *
 * Audio Connection
 * - Set the Audio Freq to desired value (48kHz, 44.1kHz, 32kHz, 16kHz)
 * - Set the route of the audio data transfer ( Sine Wav(testing),  File )
 * - In case of file, stored sine data in the hci bridge application is sent
 *   over the SPI transport.
 * - To create audio connection to remote A2DP sink device, choose the bluetooth
 *   address of the remote device from the BR/EDR combo box
 * - Click "Connect" button under Audio
 *
 * AVRCP Target
 * - The audio player uses AVRCP Target role. Once connected, the AVRCP Target 
 * - can send notifications for play status change (Play, Pause, Stop) and 
 * - setting change (Repeat, Shuffle) to peer AVRCP controller.
 *
 * AVRCP Controller
 * - In Controller mode, pass-thru commands are executed via Play, Pause, Stop, etc. buttons.
 * - Absolute volume change can be done via the drop down combo box "Audio"
 */
#include "wiced_bt_gatt.h"
#include "wiced_bt_ble.h"
#include "wiced_bt_cfg.h"
#include "wiced_memory.h"
#include "wiced_bt_sdp.h"
#include "wiced_bt_avrc_defs.h"
#include "wiced_bt_avrc.h"
#include "wiced_timer.h"
#include "hci_control.h"
#include "hci_control_test.h"
#if ( !defined(CYW20719B1) && !defined(CYW20721B1) )
#include "wiced_hal_platform.h"
#else
#include "wiced_sleep.h"
#include "wiced_platform.h"
#endif

#ifdef CYW20706A2
#include "wiced_power_save.h"
#if (WICED_HCI_TRANSPORT == WICED_HCI_TRANSPORT_SPI)
#include "wiced_hal_pspi.h"
#endif
#endif

#include "wiced_hal_puart.h"
#include "wiced_transport.h"
#include "wiced_app.h"
#include "wiced_app_cfg.h"
#include "wiced_bt_avrc_tg.h"
#include "string.h"
#include "wiced_bt_stack.h"
#include "wiced_bt_app_common.h"
#include "wiced_hal_wdog.h"
#ifdef CYW20719B1
#include "wiced_bt_event.h"
#endif

/*****************************************************************************
**  Constants
*****************************************************************************/
#define WICED_HS_EIR_BUF_MAX_SIZE               264
#define KEY_INFO_POOL_BUFFER_SIZE   145 //Size of the buffer used for holding the peer device key info
#define KEY_INFO_POOL_BUFFER_COUNT  5  //Correspond's to the number of peer devices

/*****************************************************************************
**  Structures
*****************************************************************************/
typedef struct
{
    void    *p_next;
    uint16_t nvram_id;
    uint8_t  chunk_len;
    uint8_t  data[1];
} hci_control_nvram_chunk_t;

/******************************************************
 *               Variables Definitions
 ******************************************************/
wiced_bool_t avrcp_profile_role = AVRCP_TARGET_ROLE;

hci_control_nvram_chunk_t *p_nvram_first = NULL;

/* HS control block */
#if BTA_DYNAMIC_MEMORY == WICED_FALSE
hci_control_cb_t  hci_control_cb;
#endif

wiced_transport_buffer_pool_t* transport_pool;   // Trans pool for sending the RFCOMM data to host
wiced_bt_buffer_pool_t*        p_key_info_pool;  //Pool for storing the  key info


static void      hci_control_transport_status( wiced_transport_type_t type );
static uint32_t  hci_control_proc_rx_cmd( uint8_t *p_data, uint32_t length );
static void      hci_control_transport_tx_cplt_cback(wiced_transport_buffer_pool_t* p_pool);

#if ( !defined(CYW43012C0) && (WICED_HCI_TRANSPORT == WICED_HCI_TRANSPORT_SPI))

#ifdef CYW20706A2
#define SLAVE1_P01_CS_P10_CLK_P28_MOSI_P29_MISO     0x010A1C1D
#endif

const wiced_transport_cfg_t transport_cfg =
{
    WICED_TRANSPORT_SPI,
    {
#ifdef CYW20706A2
        SPI_SLAVE_ROLE,
        SLAVE1_P02_CS_P03_CLK_P00_MOSI_P25_MISO, /**< Pins to use for the data and clk lines. Refer  spiffdriver.h for details */
        INPUT_PIN_FLOATING,                       /**< Pin pull-up or pull-down */
#else
        SPI_SLAVE,
        SLAVE1_P01_CS_P10_CLK_P28_MOSI_P29_MISO, /**< Pins to use for the data and clk lines. Refer  spiffdriver.h for details */
        INPUT_PIN_PULL_DOWN,                      /**< Floating (MOSI in Slave mode) */
#endif
        0,
        SPI_MSB_FIRST,                           /**< Direction of bit data flow (MSB or LSB first) */
        SPI_SS_ACTIVE_LOW,                       /**< Active high or active low for chip select line */
        SPI_MODE_0,                              /**< SPI mode (0-3). */
        0,
#ifdef CYW20706A2
        WICED_P15                                /**< GPIO pin to be used as slave ready */
#else
        WICED_P06                                /**< GPIO pin to be used as slave ready */
#endif
    },
    {  TRANS_SPI_BUFFER_SIZE, 2 },
    hci_control_transport_status,
    hci_control_proc_rx_cmd,
    hci_control_transport_tx_cplt_cback
};

#else

const wiced_transport_cfg_t transport_cfg =
{
    WICED_TRANSPORT_UART,
    {  WICED_TRANSPORT_UART_HCI_MODE, HCI_UART_DEFAULT_BAUD },
    {  TRANS_UART_BUFFER_SIZE, 2 },
    hci_control_transport_status,
    hci_control_proc_rx_cmd,
    hci_control_transport_tx_cplt_cback
};

#endif

/******************************************************
 *               Function Declarations
 ******************************************************/
static void     hci_control_handle_reset_cmd( void );
static void     hci_control_handle_trace_enable( uint8_t *p_data );
static void     hci_control_device_handle_command( uint16_t cmd_opcode, uint8_t* p_data, uint32_t data_len );
static void     hci_control_handle_set_local_bda( uint8_t *p_bda );
static void     hci_control_inquiry( uint8_t enable );
static void     hci_control_handle_set_visibility( uint8_t discoverability, uint8_t connectability );
static void     hci_control_handle_set_pairability ( uint8_t pairing_allowed );
#if (defined(COEX_SUPPORTED) && COEX_SUPPORTED == WICED_TRUE)
static void     hci_control_handle_enable_disable_coex ( wiced_bool_t enable );
#endif
static void     hci_control_handle_read_local_bda( void );
static void     hci_control_handle_user_confirmation( uint8_t *p_bda, uint8_t accept_pairing );
static void     hci_control_handle_read_buffer_stats( void );
static void     hci_control_send_device_started_evt( void );
static void     hci_control_send_device_error_evt( uint8_t fw_error_code, uint8_t app_error_code );
static void     hci_control_send_pairing_completed_evt( uint8_t status, wiced_bt_device_address_t bdaddr );
static void     hci_control_send_user_confirmation_request_evt( BD_ADDR bda, uint32_t numeric_value );
static void     hci_control_send_encryption_changed_evt( uint8_t encrypted, wiced_bt_device_address_t bdaddr );
static wiced_result_t hci_control_management_callback( wiced_bt_management_evt_t event, wiced_bt_management_evt_data_t *p_event_data );

extern void wiced_bt_rc_target_initiate_close( void );
extern wiced_result_t wiced_bt_remote_control_cleanup( void );
extern void av_app_init( void );
extern void hci_control_rc_controller_init( void );
extern void hci_control_rc_target_init( void );
extern void wiced_bt_rc_target_register(void);
extern wiced_result_t wiced_bt_avrc_ct_cleanup( void );

/******************************************************
 *               Function Definitions
 ******************************************************/

/*
 *  hci_control_init
 */
void hci_control_init( void )
{
    memset( &hci_control_cb, 0, sizeof( hci_control_cb ) );

#ifdef WICED_BT_TRACE_ENABLE
#ifdef CYW43012C0
    wiced_set_debug_uart(WICED_ROUTE_DEBUG_TO_WICED_UART);
#else
    wiced_set_debug_uart( WICED_ROUTE_DEBUG_TO_PUART );
#if ( defined(CYW20706A2) )
    wiced_hal_puart_select_uart_pads( WICED_PUART_RXD, WICED_PUART_TXD, 0, 0);
#endif // CYW20706A2
#endif // CYW43012C0
#endif // WICED_BT_TRACE_ENABLE

    wiced_transport_init( &transport_cfg );

    wiced_bt_stack_init( hci_control_management_callback,
                         &wiced_bt_cfg_settings,
                         wiced_app_cfg_buf_pools);

    wiced_audio_buffer_initialize(wiced_bt_audio_buf_config);
}

#if (defined(SLEEP_SUPPORTED) && (SLEEP_SUPPORTED == WICED_TRUE))

#ifndef CYW20706A2
wiced_timer_t hci_control_app_timer;

void hci_control_timeout( uint32_t count )
{
    WICED_BT_TRACE("Idle Timeout.\n");

    hci_control_cb.application_state = HCI_CONTROL_STATE_IDLE;
}

/*
* The function invoked on timeout of app seconds timer.
*/
void hci_control_init_timer(void)
{
    /* Start idle timer to enter to sleep */
    if ( wiced_init_timer( &hci_control_app_timer, hci_control_timeout, 0, WICED_SECONDS_TIMER ) == WICED_SUCCESS )
    {
        if ( wiced_start_timer( &hci_control_app_timer, 10 ) !=  WICED_SUCCESS )
        {
            WICED_BT_TRACE("idle timer start failure\n");
            return;
        }
    }
    else
    {
        WICED_BT_TRACE("idle timer init fail \n");
    }
}

/*
* The function handles sleep.
*/
static uint32_t hci_control_sleep_hanlder(wiced_sleep_poll_type_t type )
{
    uint32_t ret = WICED_SLEEP_NOT_ALLOWED;

    WICED_BT_TRACE(".");
    switch(type)
    {
        case WICED_SLEEP_POLL_SLEEP_PERMISSION:
            if( hci_control_cb.application_state == HCI_CONTROL_STATE_IDLE )
            {
#ifdef ENABLE_SDS_SLEEP
                ret = WICED_SLEEP_ALLOWED_WITH_SHUTDOWN;
                WICED_BT_TRACE("$");
#else
                WICED_BT_TRACE("&");
                ret = WICED_SLEEP_ALLOWED_WITHOUT_SHUTDOWN;
#endif
            }
            break;

        case WICED_SLEEP_POLL_TIME_TO_SLEEP:
            if( hci_control_cb.application_state == HCI_CONTROL_STATE_NOT_IDLE )
            {
                ret = 0;
            }
            else
            {
                ret = WICED_SLEEP_MAX_TIME_TO_SLEEP;
            }
            break;
    }
    return ret;
}
#endif // !CYW20706A2

/*
* The function does sleep configuration based on platform.
*/
void hci_control_sleep_configure()
{
    WICED_BT_TRACE("[%s] \n",__FUNCTION__);
#ifdef CYW20706A2
    {
        wiced_ptu_clock_bits_t ptu_clock_bits;
        wiced_ptu_aux_clock_bits_t ptu_aux_clock_bits;
        ptu_clock_bits     = (  GATE_ON_SDIO_CLK_BIT |
                                PCM_CLK_EN_BIT  |
                                GATE_ON_SDIO_HOST_CLK_BIT
                             );
        ptu_aux_clock_bits = (  PCM_DEV_CLK_EN_BIT |
                                ABURST_CLK_EN_BIT  |
                                SPIFFY2_TPORT_CLK_EN_BIT |
                                SPIFFY2_HCLK_CLK_EN_BIT
                             );
#if (WICED_HCI_TRANSPORT == WICED_HCI_TRANSPORT_SPI)
        //SPI Sleep Init
        //P2 is the SPI CS and the device should wake when CS is asserted.
        //I2S_CLK/PCM_CLK/P2/P37/P28 available on the same pad B7 on the 20706-P49 package.
        //So configuring I2S_CLK(BT_GPIO_6) as the DEV_WAKE.
        //SPI CS is configured as active low, so DEV_WAKE should be active low
        wiced_trans_spi_sleep_config( WICED_GPIO_06, WICED_WAKE_GPIO_ACTIVE_LOW );
#else
        wiced_sleep_config( WICED_TRUE, WICED_WAKE_GPIO_ACTIVE_LOW, WICED_WAKE_GPIO_ACTIVE_HIGH );
#endif
        wiced_ptu_clock_disable(ptu_clock_bits, ptu_aux_clock_bits);
    }
#else // !20703

    {

        /*sleep configuration*/
        static wiced_sleep_config_t    hci_control_sleep_config;
        //Todo: sleep configuration in case of UART transport
#if (WICED_HCI_TRANSPORT == WICED_HCI_TRANSPORT_SPI)
        // For UART mode MCU should control wake gpio and configure the sleep
        hci_control_sleep_config.sleep_mode             = WICED_SLEEP_MODE_NO_TRANSPORT;
        hci_control_sleep_config.device_wake_gpio_num   = WICED_P06;

        hci_control_sleep_config.device_wake_mode       = WICED_SLEEP_WAKE_ACTIVE_LOW;
        hci_control_sleep_config.device_wake_source     = WICED_SLEEP_WAKE_SOURCE_GPIO;
        hci_control_sleep_config.host_wake_mode         = WICED_SLEEP_WAKE_ACTIVE_HIGH;
        hci_control_sleep_config.sleep_permit_handler   = hci_control_sleep_hanlder;

        // Init Idle timer
        hci_control_init_timer();

        if( wiced_sleep_get_boot_mode() == WICED_SLEEP_COLD_BOOT )
        {
            WICED_BT_TRACE( "Watch Cold Start \n" );
        }
        else
        {
            // If warm start, retrieve data from retention ram if needed
            WICED_BT_TRACE( "Watch Warm Start \n" );
        }
        hci_control_cb.application_state = HCI_CONTROL_STATE_NOT_IDLE;

        wiced_sleep_configure( &hci_control_sleep_config );
#endif
    }
#endif // CYW20706A2

}
#endif // end (defined(SLEEP_SUPPORTED) && (SLEEP_SUPPORTED == WICED_TRUE))
/*
 *  Prepare extended inquiry response data.  Current version publishes headset,
 *  handsfree and generic audio services.
 */
void hci_control_write_eir( void )
{
    uint8_t *pBuf;
    uint8_t *p, *p_tmp;
    uint8_t nb_uuid = 0;
    uint8_t length;

    //Allocating a buffer from the public pool
#ifdef CYW43012C0
    pBuf = (uint8_t *)watch_get_buffer( WICED_HS_EIR_BUF_MAX_SIZE );
#else
    pBuf = (uint8_t *)wiced_bt_get_buffer( WICED_HS_EIR_BUF_MAX_SIZE );
#endif
    WICED_BT_TRACE( "hci_control_write_eir %x\n", pBuf );

    if ( !pBuf )
    {
        return;
    }

    p = pBuf;

    length = strlen( (char *)wiced_bt_cfg_settings.device_name );
    UINT8_TO_STREAM(p, length + 1);
    UINT8_TO_STREAM(p, BT_EIR_COMPLETE_LOCAL_NAME_TYPE);
    memcpy( p, wiced_bt_cfg_settings.device_name, length );
    p += length;

    // Add other BR/EDR UUIDs
    p_tmp = p;      // We don't now the number of UUIDs for the moment
    p++;
    UINT8_TO_STREAM(p, BT_EIR_COMPLETE_16BITS_UUID_TYPE);
#if (WICED_APP_AUDIO_SRC_INCLUDED == WICED_TRUE)
    UINT16_TO_STREAM(p, UUID_SERVCLASS_AUDIO_SOURCE);       nb_uuid++;
#endif
#if (WICED_APP_AUDIO_RC_TG_INCLUDED == WICED_TRUE)
    UINT16_TO_STREAM(p, UUID_SERVCLASS_AV_REM_CTRL_TARGET); nb_uuid++;
#endif
#if (WICED_APP_AUDIO_RC_CT_INCLUDED == WICED_TRUE)
    UINT16_TO_STREAM(p, UUID_SERVCLASS_AV_REMOTE_CONTROL);  nb_uuid++;
    UINT16_TO_STREAM(p, UUID_SERVCLASS_AUDIO_SINK);         nb_uuid++;
#endif
    /* Now, we can update the UUID Tag's length */
    UINT8_TO_STREAM(p_tmp, (nb_uuid * LEN_UUID_16) + 1);

    // Last Tag
    UINT8_TO_STREAM(p, 0x00);

    // print EIR data
    wiced_bt_trace_array( "EIR :", ( uint8_t* )( pBuf+1 ), MIN( p - ( uint8_t* )pBuf, 100 ) );
    wiced_bt_dev_write_eir( pBuf, (uint16_t)(p - pBuf) );

    /* Allocated buffer not anymore needed. Free it */
    wiced_bt_free_buffer( pBuf );

    return;
}

/*
 *  Process all HCI packet from
 */
#define COD_MAJOR_COMPUTER  1
#define COD_MAJOR_PHONE     2

void hci_control_hci_packet_cback( wiced_bt_hci_trace_type_t type, uint16_t length, uint8_t* p_data )
{
#if (WICED_HCI_TRANSPORT == WICED_HCI_TRANSPORT_UART)
    // send the trace
    wiced_transport_send_hci_trace( NULL, type, length, p_data  );
#endif

    if ( !test_command.test_executing )
        return;

    // If executing test command, need to send Command Complete event back to host app
    if( ( type == HCI_TRACE_EVENT ) && ( length >= 6 ) )
    {
        hci_control_handle_hci_test_event( p_data, length );
    }
}

/*
 *  Management callback receives various notifications from the stack
 */
wiced_result_t hci_control_management_callback( wiced_bt_management_evt_t event, wiced_bt_management_evt_data_t *p_event_data )
{
    wiced_result_t                     result = WICED_BT_SUCCESS;
    wiced_bt_dev_encryption_status_t  *p_encryption_status;
    int                                bytes_written, bytes_read;
    int                                nvram_id;
    wiced_bt_power_mgmt_notification_t *p_power_mgmt_notification;
    wiced_bt_dev_pairing_cplt_t        *p_pairing_cmpl;
    uint8_t                             pairing_result;
    wiced_bool_t                       sdp_rv;

    WICED_BT_TRACE( "hci_control_management_callback 0x%02x\n", event );

    switch( event )
    {
        /* Bluetooth  stack enabled */
        case BTM_ENABLED_EVT:
#ifdef CYW20706A2
            wiced_bt_app_init();
#endif
            hci_control_write_eir( );

            /* initialize everything */
            memset( &hci_control_cb, 0, sizeof( hci_control_cb ) );

            /* create SDP records */
            sdp_rv = wiced_bt_sdp_db_init((uint8_t *)wiced_app_cfg_sdp_record,
                    wiced_app_cfg_sdp_record_get_size());
            if (sdp_rv != WICED_TRUE)
                WICED_BT_TRACE("Err: wiced_bt_sdp_db_init Failed\n");

#if (WICED_APP_LE_INCLUDED == WICED_TRUE)
            hci_control_le_enable( );
#endif

#if (WICED_APP_AUDIO_SRC_INCLUDED == WICED_TRUE)
            av_app_init();
#endif
#if (WICED_TRUE == WICED_APP_AUDIO_RC_CT_INCLUDED)
            /* Initialize AVRC Target and Controler */
            hci_control_rc_controller_init();
#endif
#if (WICED_TRUE == WICED_APP_AUDIO_RC_TG_INCLUDED)
            hci_control_rc_target_init();
#endif
            // Disable while streaming audio over the uart.
            wiced_bt_dev_register_hci_trace( hci_control_hci_packet_cback );

            // Creating a buffer pool for holding the peer devices's key info
            p_key_info_pool = wiced_bt_create_pool( KEY_INFO_POOL_BUFFER_SIZE, KEY_INFO_POOL_BUFFER_COUNT );
            if (p_key_info_pool == NULL)
                WICED_BT_TRACE("Err: wiced_bt_create_pool failed\n");
#ifdef CYW20706A2
            // Tell Host that App is started
            hci_control_send_device_started_evt();
#endif
#if (defined(SLEEP_SUPPORTED) && (SLEEP_SUPPORTED == WICED_TRUE))
            hci_control_sleep_configure();
#endif

#if (defined(COEX_SUPPORTED) && COEX_SUPPORTED == WICED_TRUE)
#if defined(CYW20719B1) || defined(CYW20721B1)
        wiced_bt_coex_enable(0x1E8480);
#else
        wiced_bt_coex_enable();
#endif
#endif
            break;

        case BTM_DISABLED_EVT:
            hci_control_send_device_error_evt( p_event_data->disabled.reason, 0 );
            break;

        case BTM_PIN_REQUEST_EVT:
            WICED_BT_TRACE("remote address= %B\n", p_event_data->pin_request.bd_addr);
            wiced_bt_dev_pin_code_reply(*p_event_data->pin_request.bd_addr, WICED_BT_SUCCESS, WICED_PIN_CODE_LEN, (uint8_t *)&pincode[0]);
            break;

        case BTM_USER_CONFIRMATION_REQUEST_EVT:
            // If this is just works pairing, accept. Otherwise send event to the MCU to confirm the same value.
            if (p_event_data->user_confirmation_request.just_works)
            {
                wiced_bt_dev_confirm_req_reply( WICED_BT_SUCCESS, p_event_data->user_confirmation_request.bd_addr );
            }
            else
            {
                hci_control_send_user_confirmation_request_evt(p_event_data->user_confirmation_request.bd_addr, p_event_data->user_confirmation_request.numeric_value );
            }
            break;

        case BTM_PASSKEY_NOTIFICATION_EVT:
            WICED_BT_TRACE("PassKey Notification. BDA %B, Key %d \n", p_event_data->user_passkey_notification.bd_addr, p_event_data->user_passkey_notification.passkey );
            hci_control_send_user_confirmation_request_evt(p_event_data->user_passkey_notification.bd_addr, p_event_data->user_passkey_notification.passkey );
            break;

        case BTM_PAIRING_IO_CAPABILITIES_BR_EDR_REQUEST_EVT:
            /* Use the default security for BR/EDR*/
            WICED_BT_TRACE("BTM_PAIRING_IO_CAPABILITIES_BR_EDR_REQUEST_EVT bda %B\n", p_event_data->pairing_io_capabilities_br_edr_request.bd_addr);
            p_event_data->pairing_io_capabilities_br_edr_request.local_io_cap = BTM_IO_CAPABILITIES_DISPLAY_AND_YES_NO_INPUT;
            p_event_data->pairing_io_capabilities_br_edr_request.auth_req     = BTM_AUTH_SINGLE_PROFILE_GENERAL_BONDING_NO;
            p_event_data->pairing_io_capabilities_br_edr_request.oob_data     = WICED_FALSE;
            p_event_data->pairing_io_capabilities_br_edr_request.auth_req     = BTM_AUTH_ALL_PROFILES_NO;
            break;

        case BTM_PAIRING_IO_CAPABILITIES_BLE_REQUEST_EVT:
            /* Use the default security for BLE */
            WICED_BT_TRACE("BTM_PAIRING_IO_CAPABILITIES_BLE_REQUEST_EVT bda %B\n",
                    p_event_data->pairing_io_capabilities_ble_request.bd_addr);
            p_event_data->pairing_io_capabilities_ble_request.local_io_cap  = BTM_IO_CAPABILITIES_DISPLAY_AND_YES_NO_INPUT;
            p_event_data->pairing_io_capabilities_ble_request.oob_data      = BTM_OOB_NONE;
            p_event_data->pairing_io_capabilities_ble_request.auth_req      = BTM_LE_AUTH_REQ_SC_MITM_BOND;
            p_event_data->pairing_io_capabilities_ble_request.max_key_size  = 16;
            p_event_data->pairing_io_capabilities_ble_request.init_keys     = BTM_LE_KEY_PENC|BTM_LE_KEY_PID|BTM_LE_KEY_PCSRK|BTM_LE_KEY_LENC;
            p_event_data->pairing_io_capabilities_ble_request.resp_keys     = BTM_LE_KEY_PENC|BTM_LE_KEY_PID|BTM_LE_KEY_PCSRK|BTM_LE_KEY_LENC;
            break;

        case BTM_PAIRING_COMPLETE_EVT:
            p_pairing_cmpl = &p_event_data->pairing_complete;
            if(p_pairing_cmpl->transport == BT_TRANSPORT_BR_EDR)
            {
                pairing_result = p_pairing_cmpl->pairing_complete_info.br_edr.status;
            }
            else
            {
                pairing_result = p_pairing_cmpl->pairing_complete_info.ble.reason;
            }
            WICED_BT_TRACE( "Pairing Result: %d\n", pairing_result );
            hci_control_send_pairing_completed_evt( pairing_result, p_event_data->pairing_complete.bd_addr );
            break;

        case BTM_ENCRYPTION_STATUS_EVT:
            p_encryption_status = &p_event_data->encryption_status;

            WICED_BT_TRACE( "Encryption Status:(%B) res:%d\n", p_encryption_status->bd_addr, p_encryption_status->result );

#if (WICED_APP_LE_SLAVE_CLIENT_INCLUDED == WICED_TRUE)
            if (p_encryption_status->transport == BT_TRANSPORT_LE)
                le_slave_encryption_status_changed(p_encryption_status);
#endif
            hci_control_send_encryption_changed_evt( p_encryption_status->result, p_encryption_status->bd_addr );
            break;

        case BTM_SECURITY_REQUEST_EVT:
            WICED_BT_TRACE( "Security Request Event, Pairing allowed %d\n", hci_control_cb.pairing_allowed );
            if ( hci_control_cb.pairing_allowed )
            {
                wiced_bt_ble_security_grant( p_event_data->security_request.bd_addr, WICED_BT_SUCCESS );
            }
            else
            {
                // Pairing not allowed, return error
                result = WICED_BT_ERROR;
            }
            break;

        case BTM_PAIRED_DEVICE_LINK_KEYS_UPDATE_EVT:
            /* Check if we already have information saved for this bd_addr */
            if ( ( nvram_id = hci_control_find_nvram_id( p_event_data->paired_device_link_keys_update.bd_addr, BD_ADDR_LEN ) ) == 0)
            {
                // This is the first time, allocate id for the new memory chunk
                nvram_id = hci_control_alloc_nvram_id( );
                WICED_BT_TRACE( "Allocated NVRAM ID:%d\n", nvram_id );
            }
            bytes_written = hci_control_write_nvram( nvram_id, sizeof( wiced_bt_device_link_keys_t ), &p_event_data->paired_device_link_keys_update, WICED_FALSE );

            WICED_BT_TRACE("NVRAM write:id:%d bytes:%d dev: [%B]\n", nvram_id, bytes_written, p_event_data->paired_device_link_keys_update.bd_addr);
            break;

        case  BTM_PAIRED_DEVICE_LINK_KEYS_REQUEST_EVT:
            /* read existing key from the NVRAM  */

            WICED_BT_TRACE("\t\tfind device %B\n", p_event_data->paired_device_link_keys_request.bd_addr);

            if ( ( nvram_id = hci_control_find_nvram_id( p_event_data->paired_device_link_keys_request.bd_addr, BD_ADDR_LEN ) ) != 0)
            {
                 bytes_read = hci_control_read_nvram( nvram_id, &p_event_data->paired_device_link_keys_request, sizeof( wiced_bt_device_link_keys_t ) );

                 result = WICED_BT_SUCCESS;
                 WICED_BT_TRACE("Read:nvram_id:%d bytes:%d\n", nvram_id, bytes_read);
            }
            else
            {
                result = WICED_BT_ERROR;
                WICED_BT_TRACE("Key retrieval failure\n");
            }
            break;

        case BTM_LOCAL_IDENTITY_KEYS_UPDATE_EVT:
            /* Request to store newly generated local identity keys to NVRAM */
            /* (sample app does not store keys to NVRAM) */

            break;


        case BTM_LOCAL_IDENTITY_KEYS_REQUEST_EVT:
            /*
             * Request to restore local identity keys from NVRAM
             * (requested during Bluetooth start up)
             * */
            /* (sample app does not store keys to NVRAM)
             * New local identity keys will be generated
             * */
            result = WICED_BT_NO_RESOURCES;
            break;
#if (WICED_APP_LE_INCLUDED == WICED_TRUE)

        case BTM_BLE_SCAN_STATE_CHANGED_EVT:
            hci_control_le_scan_state_changed( p_event_data->ble_scan_state_changed );
            break;

        case BTM_BLE_ADVERT_STATE_CHANGED_EVT:
            hci_control_le_advert_state_changed( p_event_data->ble_advert_state_changed );
            break;

        case BTM_BLE_CONNECTION_PARAM_UPDATE:
            WICED_BT_TRACE ("BTM BLE Connection Update event status %d BDA [%B] interval %d latency %d supervision timeout %d \n",
                                p_event_data->ble_connection_param_update.status, p_event_data->ble_connection_param_update.bd_addr,
                                p_event_data->ble_connection_param_update.conn_interval, p_event_data->ble_connection_param_update.conn_latency,
                                p_event_data->ble_connection_param_update.supervision_timeout);
            break;
#endif
        case BTM_POWER_MANAGEMENT_STATUS_EVT:
            p_power_mgmt_notification = &p_event_data->power_mgmt_notification;
            WICED_BT_TRACE( "Power mgmt status event: bd ( %B ) status:%d hci_status:%d\n", p_power_mgmt_notification->bd_addr, \
                    p_power_mgmt_notification->status, p_power_mgmt_notification->hci_status);
            break;

        default:
            result = WICED_BT_USE_DEFAULT_SECURITY;
            break;
    }
    return result;
}

/*
 * Handle received command over UART. Please refer to the WICED Smart Ready
 * Software User Manual (WICED-Smart-Ready-SWUM100-R) for details on the
 * HCI UART control protocol.
*/
static uint32_t hci_control_proc_rx_cmd( uint8_t *p_buffer, uint32_t length )
{
    uint16_t opcode;
    uint16_t payload_len;
    uint8_t *p_data = p_buffer;
    uint8_t  buffer_processed = WICED_TRUE;

    if ( !p_buffer )
    {
        return HCI_CONTROL_STATUS_INVALID_ARGS;
    }

    //Expected minimum 4 byte as the wiced header
    if(( length < 4 ) || (p_data == NULL))
    {
        WICED_BT_TRACE("invalid params\n");
        wiced_transport_free_buffer( p_buffer );
        return HCI_CONTROL_STATUS_INVALID_ARGS;
    }

    STREAM_TO_UINT16(opcode, p_data);       // Get OpCode
    STREAM_TO_UINT16(payload_len, p_data);  // Gen Payload Length

    WICED_BT_TRACE("cmd_opcode 0x%02x\n", opcode);

    switch((opcode >> 8) & 0xff)
    {
    case HCI_CONTROL_GROUP_DEVICE:
        hci_control_device_handle_command( opcode, p_data, payload_len );
        break;

#if (WICED_APP_LE_INCLUDED == WICED_TRUE)
    case HCI_CONTROL_GROUP_LE:
    case HCI_CONTROL_GROUP_GATT:
        hci_control_le_handle_command( opcode, p_data, payload_len );
        break;
#endif

#if (WICED_APP_AUDIO_SRC_INCLUDED == WICED_TRUE)
    case HCI_CONTROL_GROUP_AUDIO:
        hci_control_audio_handle_command( opcode, p_data, payload_len );
        break;
#endif
#if ( WICED_APP_AUDIO_RC_TG_INCLUDED == WICED_TRUE )
    case HCI_CONTROL_GROUP_AVRC_TARGET:
        hci_control_avrc_handle_command( opcode, p_data, payload_len );
        break;
#endif

    case HCI_CONTROL_GROUP_AVRC_CONTROLLER:
#if ( WICED_APP_AUDIO_RC_TG_INCLUDED == WICED_TRUE )
        if ((avrcp_profile_role == AVRCP_TARGET_ROLE) &&
            (hci_control_rc_target_is_connected()))
        {
            hci_control_avrc_handle_command( opcode, p_data, payload_len );
        }
        else
        {
#endif
#if (WICED_APP_LE_SLAVE_CLIENT_INCLUDED == WICED_TRUE)
        if ( hci_control_is_ams_connection_up() )
        {
            hci_control_ams_handle_command( opcode, p_data, payload_len );
        }
        else
#endif
#if (WICED_APP_AUDIO_RC_CT_INCLUDED == WICED_TRUE)
        {
            hci_control_avrc_handle_ctrlr_command(opcode, p_data, payload_len);
            }
        }
#endif
        break;

#if (WICED_APP_TEST_INCLUDED == WICED_TRUE)
    case HCI_CONTROL_GROUP_TEST:
        hci_control_test_handle_command( opcode, p_data, payload_len );
        break;
#endif

#if (WICED_APP_ANCS_INCLUDED == WICED_TRUE)
    case HCI_CONTROL_GROUP_ANCS:
        hci_control_ancs_handle_command( opcode, p_data, payload_len );
        break;
#endif

    case HCI_CONTROL_GROUP_MISC:
        hci_control_misc_handle_command(opcode, p_data, payload_len);
        break;

    default:
        WICED_BT_TRACE( "unknown class code (opcode:%x)\n", opcode);
        break;
    }
    if (buffer_processed)
    {
        // Freeing the buffer in which data is received
        wiced_transport_free_buffer( p_buffer );
    }

    return HCI_CONTROL_STATUS_SUCCESS;
}

void hci_control_device_handle_command( uint16_t cmd_opcode, uint8_t* p_data, uint32_t data_len )
{
    uint8_t bytes_written;
    switch( cmd_opcode )
    {
    case HCI_CONTROL_COMMAND_RESET:
        hci_control_handle_reset_cmd( );
        break;

    case HCI_CONTROL_COMMAND_TRACE_ENABLE:
        hci_control_handle_trace_enable( p_data );
        break;

    case HCI_CONTROL_COMMAND_SET_LOCAL_BDA:
        hci_control_handle_set_local_bda( p_data );
        break;

    case HCI_CONTROL_COMMAND_PUSH_NVRAM_DATA:
        bytes_written = hci_control_write_nvram( p_data[0] | ( p_data[1] << 8 ), data_len - 2, &p_data[2], WICED_TRUE );
        WICED_BT_TRACE( "NVRAM write: %d dev: [%B]\n", bytes_written , &p_data[2]);
        break;

    case HCI_CONTROL_COMMAND_DELETE_NVRAM_DATA:
        hci_control_delete_nvram( p_data[0] | ( p_data[1] << 8 ), WICED_TRUE );
        WICED_BT_TRACE( "NVRAM delete: %d\n", p_data[0] | ( p_data[1] << 8 ) );
        break;

    case HCI_CONTROL_COMMAND_INQUIRY:
        hci_control_inquiry( *p_data );
        break;
            
    case HCI_CONTROL_COMMAND_SET_VISIBILITY:
        hci_control_handle_set_visibility( p_data[0], p_data[1] );
        break;

    case  HCI_CONTROL_COMMAND_SET_PAIRING_MODE:
        hci_control_handle_set_pairability( p_data[0] );
        break;
#if (defined(COEX_SUPPORTED) && COEX_SUPPORTED == WICED_TRUE)
    case HCI_CONTROL_COMMAND_ENABLE_COEX:
        hci_control_handle_enable_disable_coex( WICED_TRUE );
        break;

    case HCI_CONTROL_COMMAND_DISABLE_COEX:
        hci_control_handle_enable_disable_coex( WICED_FALSE );
        break;
#endif
    case HCI_CONTROL_COMMAND_READ_LOCAL_BDA:
        hci_control_handle_read_local_bda();
        break;

    case HCI_CONTROL_COMMAND_USER_CONFIRMATION:
        hci_control_handle_user_confirmation( p_data, p_data[6] );
        break;

    case HCI_CONTROL_COMMAND_READ_BUFF_STATS:
        hci_control_handle_read_buffer_stats ();
        break;

    default:
        WICED_BT_TRACE( "??? Unknown command code\n" );
        break;
    }
}

/*
 * handle reset command from UART
 */
void hci_control_handle_reset_cmd( void )
{
    hci_control_send_command_status_evt( HCI_CONTROL_EVENT_COMMAND_STATUS, HCI_CONTROL_STATUS_SUCCESS );

    // trip watch dog now.
    wiced_hal_wdog_reset_system( );
}

/*
 * handle command from UART to configure traces
 */
void hci_control_handle_trace_enable( uint8_t *p_data )
{
    uint8_t hci_trace_enable = *p_data++;
    wiced_debug_uart_types_t route_debug = (wiced_debug_uart_types_t)*p_data;

    WICED_BT_TRACE("HCI Traces:%d DebugRoute:%d\n", hci_trace_enable, route_debug);

    if ( hci_trace_enable )
    {
        /* Register callback for receiving hci traces */
        // Disable while streaming audio over the uart.
        wiced_bt_dev_register_hci_trace( hci_control_hci_packet_cback );
    }
    else
    {
        wiced_bt_dev_register_hci_trace( NULL);
    }
// In SPI transport case, PUART is recommended for debug traces and is set to PUART by default.
#if (WICED_HCI_TRANSPORT != WICED_HCI_TRANSPORT_SPI)
    wiced_set_debug_uart( route_debug );
#endif

    hci_control_send_command_status_evt( HCI_CONTROL_EVENT_COMMAND_STATUS, HCI_CONTROL_STATUS_SUCCESS );
}

/*
 * handle command to set local Bluetooth device address
 */
void hci_control_handle_set_local_bda( uint8_t *p_bda)
{
    BD_ADDR bd_addr;

    STREAM_TO_BDADDR(bd_addr,p_bda);
    wiced_bt_set_local_bdaddr( bd_addr, BLE_ADDR_PUBLIC );

    hci_control_send_command_status_evt( HCI_CONTROL_EVENT_COMMAND_STATUS, HCI_CONTROL_STATUS_SUCCESS );
}

/*
 *  Handle read buffer statistics
 */
void hci_control_handle_read_buffer_stats( void )
{
    wiced_bt_buffer_statistics_t buff_stats[ wiced_bt_get_number_of_buffer_pools() ];
    wiced_result_t result;
    uint8_t i;

    result = wiced_bt_get_buffer_usage( buff_stats, sizeof( buff_stats ) );

    if( result == WICED_BT_SUCCESS )
    {
        // Print out the stats to trace
        WICED_BT_TRACE( "Buffer usage statistics:\n");

        for( i=0; i < wiced_bt_get_number_of_buffer_pools(); i++) {
            WICED_BT_TRACE("pool_id:%d size:%d curr_cnt:%d max_cnt:%d total:%d\n",
                           buff_stats[i].pool_id, buff_stats[i].pool_size,
                           buff_stats[i].current_allocated_count, buff_stats[i].max_allocated_count,
                           buff_stats[i].total_count);
        }

        // Return the stats via WICED-HCI
        wiced_transport_send_data( HCI_CONTROL_EVENT_READ_BUFFER_STATS, (uint8_t*)&buff_stats, sizeof( buff_stats ) );
    }
    else {
        hci_control_send_command_status_evt( HCI_CONTROL_EVENT_COMMAND_STATUS, HCI_CONTROL_STATUS_FAILED );
    }
}

/*
 *  Handle Inquiry result callback from teh stack, format and send event over UART
 */
void hci_control_inquiry_result_cback( wiced_bt_dev_inquiry_scan_result_t *p_inquiry_result, uint8_t *p_eir_data )
{
    int       i;
    uint8_t   len;
    uint8_t   tx_buf[300];
    uint16_t  code;
    uint8_t   *p = tx_buf;

    if ( p_inquiry_result == NULL )
    {
        code = HCI_CONTROL_EVENT_INQUIRY_COMPLETE;
        WICED_BT_TRACE( "inquiry complete \n");
    }
    else
    {
        code = HCI_CONTROL_EVENT_INQUIRY_RESULT;
        WICED_BT_TRACE( "inquiry result %B\n", p_inquiry_result->remote_bd_addr );
        for ( i = 0; i < 6; i++ )
            *p++ = p_inquiry_result->remote_bd_addr[5 - i];
        for ( i = 0; i < 3; i++ )
            *p++ = p_inquiry_result->dev_class[2 - i];
        *p++ = p_inquiry_result->rssi;

        // currently callback does not pass the data of the adv data, need to go through the data
        // zero len in the LTV means that there is no more data
        while ( ( p_eir_data != NULL ) && ( len = *p_eir_data ) != 0 )
        {
            // In the HCI event all parameters should fit into 255 bytes
            if ( p + len + 1 > tx_buf + 255 )
            {
                WICED_BT_TRACE( "Bad data\n" );
                break;
            }
            for ( i = 0; i < len + 1; i++ )
                *p++ = *p_eir_data++;
        }
    }
    wiced_transport_send_data( code, tx_buf, ( int )( p - tx_buf ) );
}

/*
 *  Handle Inquiry command received over UART
 */
void hci_control_inquiry( uint8_t enable )
{
    wiced_result_t           result;
    wiced_bt_dev_inq_parms_t params;

    if ( enable )
    {

        memset( &params, 0, sizeof( params ) );

        params.mode             = BTM_GENERAL_INQUIRY;
        params.duration         = 5;
        params.filter_cond_type = BTM_CLR_INQUIRY_FILTER;

        result = wiced_bt_start_inquiry( &params, &hci_control_inquiry_result_cback );
        WICED_BT_TRACE( "inquiry started:%d\n", result );
    }
    else
    {
        result = wiced_bt_cancel_inquiry( );
        WICED_BT_TRACE( "cancel inquiry:%d\n", result );
    }
    hci_control_send_command_status_evt( HCI_CONTROL_EVENT_COMMAND_STATUS, HCI_CONTROL_STATUS_SUCCESS );
}

/*
 *  Handle Set Visibility command received over UART
 */
void hci_control_handle_set_visibility( uint8_t discoverability, uint8_t connectability )
{
    // we cannot be discoverable and not connectable
    if ( ( ( discoverability != 0 ) && ( connectability == 0 ) ) ||
           ( discoverability > 1 ) ||
           ( connectability > 1 ) )
    {
        hci_control_send_command_status_evt( HCI_CONTROL_EVENT_COMMAND_STATUS, HCI_CONTROL_STATUS_INVALID_ARGS );
    }
    else
    {
        wiced_bt_dev_set_discoverability( ( discoverability != 0 ) ? BTM_GENERAL_DISCOVERABLE : BTM_NON_DISCOVERABLE ,
                                            BTM_DEFAULT_DISC_WINDOW,
                                            BTM_DEFAULT_DISC_INTERVAL);

        wiced_bt_dev_set_connectability( ( connectability != 0 ) ? WICED_TRUE : WICED_FALSE ,
                                            BTM_DEFAULT_CONN_WINDOW,
                                            BTM_DEFAULT_CONN_INTERVAL);

        hci_control_send_command_status_evt( HCI_CONTROL_EVENT_COMMAND_STATUS, HCI_CONTROL_STATUS_SUCCESS );
    }
}

/*
 *  Handle Set Pairability command received over UART
 */
void hci_control_handle_set_pairability ( uint8_t pairing_allowed )
{
    uint8_t                   status = HCI_CONTROL_STATUS_SUCCESS;
    hci_control_nvram_chunk_t *p1 = NULL;

    if ( hci_control_cb.pairing_allowed != pairing_allowed )
    {
        if ( pairing_allowed )
        {
            // Check if key buffer pool has buffer available. If not, cannot enable pairing until nvram entries are deleted
            if (wiced_bt_get_buffer_count(p_key_info_pool) <= 0)
            {
                WICED_BT_TRACE( "Err: No more memory for Pairing\n" );
                pairing_allowed = 0; //The key buffer pool is full therefore we cannot allow pairing to be enabled
                status = HCI_CONTROL_STATUS_OUT_OF_MEMORY;
            }
        }

        hci_control_cb.pairing_allowed = pairing_allowed;
        wiced_bt_set_pairable_mode( hci_control_cb.pairing_allowed, 0 );
        WICED_BT_TRACE( " Set the pairing allowed to %d \n", hci_control_cb.pairing_allowed );
    }

    hci_control_send_command_status_evt( HCI_CONTROL_EVENT_COMMAND_STATUS, status );
}

#if (defined(COEX_SUPPORTED) && COEX_SUPPORTED == WICED_TRUE)
/*
 *  Handle Enable/Disable Coex command received over UART
 */
static void hci_control_handle_enable_disable_coex ( wiced_bool_t enable )
{
    if ( enable )
    {
        WICED_BT_TRACE( "Enabling Coex\n" );
#if defined(CYW20719B1) || defined(CYW20721B1)
        wiced_bt_coex_enable(0x1E8480);
#else
        wiced_bt_coex_enable();
#endif
    }
    else
    {
        WICED_BT_TRACE( "Disabling Coex\n" );
        wiced_bt_coex_disable();
    }

    hci_control_send_command_status_evt( HCI_CONTROL_EVENT_COMMAND_STATUS, HCI_CONTROL_STATUS_SUCCESS );
}
#endif

/*
 *  Handle Get Local BDA command received over UART
 */
void hci_control_handle_read_local_bda( void )
{
    wiced_bt_device_address_t bda = { 0 };

    wiced_bt_dev_read_local_addr(bda);
    WICED_BT_TRACE("Local Bluetooth Address: [%B]\n", bda);

    wiced_transport_send_data( HCI_CONTROL_EVENT_READ_LOCAL_BDA, (uint8_t*)bda , 6 );
}

/*
 *  Handle User Confirmation received over UART
 */
void hci_control_handle_user_confirmation( uint8_t *p_bda, uint8_t accept_pairing )
{
    wiced_bt_device_address_t bd_addr;

    STREAM_TO_BDADDR(bd_addr,p_bda);
    wiced_bt_dev_confirm_req_reply( accept_pairing == WICED_TRUE ? WICED_BT_SUCCESS : WICED_BT_ERROR, bd_addr);

    hci_control_send_command_status_evt( HCI_CONTROL_EVENT_COMMAND_STATUS, HCI_CONTROL_STATUS_SUCCESS );
}

/*
 *  Send Device Started event through UART
 */
void hci_control_send_device_started_evt( void )
{
    wiced_transport_send_data( HCI_CONTROL_EVENT_DEVICE_STARTED, NULL, 0 );

    WICED_BT_TRACE( "maxLinks:%d maxChannels:%d maxpsm:%d rfcom max links%d, rfcom max ports:%d\n",
            wiced_bt_cfg_settings.l2cap_application.max_links,
            wiced_bt_cfg_settings.l2cap_application.max_channels,
            wiced_bt_cfg_settings.l2cap_application.max_psm,
            wiced_bt_cfg_settings.rfcomm_cfg.max_links,
            wiced_bt_cfg_settings.rfcomm_cfg.max_ports );
}

/*
 *  Send Device Error event through UART
 */
void hci_control_send_device_error_evt( uint8_t fw_error_code, uint8_t app_error_code )
{
    uint8_t event_data[] = { 0, 0 };

    event_data[0] = app_error_code;
    event_data[1] = fw_error_code;

    WICED_BT_TRACE( "[hci_control_send_device_error_evt]  app_error_code=0x%02x  fw_error_code=0x%02x\n", event_data[0], event_data[1] );

    wiced_transport_send_data( HCI_CONTROL_EVENT_DEVICE_ERROR, event_data, 2 );
}

/*
* transfer command status event to UART
*/
void hci_control_send_command_status_evt( uint16_t code, uint8_t status )
{
    wiced_transport_send_data( code, &status, 1 );
}

/*
 *  Send Pairing Completed event through UART
 */
void hci_control_send_pairing_completed_evt( uint8_t status , wiced_bt_device_address_t bdaddr )
{
    int i;
    const int cmd_size = BD_ADDR_LEN + sizeof(uint8_t);
    uint8_t event_data[cmd_size];
    int     cmd_bytes = 0;

    event_data[cmd_bytes++] = status;

    for ( i = 0 ; i < BD_ADDR_LEN; i++ )                     // bd address
        event_data[cmd_bytes++] = bdaddr[BD_ADDR_LEN - 1 - i];

    WICED_BT_TRACE( "pairing complete evt: %B as %B status %d\n", bdaddr, &event_data[1], status );

    wiced_transport_send_data( HCI_CONTROL_EVENT_PAIRING_COMPLETE, event_data, cmd_bytes );
}

/*
 *  Send User Confirmation Request event through UART
 */
void hci_control_send_user_confirmation_request_evt( BD_ADDR bda, uint32_t numeric_value )
{
    uint8_t buf[10];
    uint8_t *p = &buf[6];
    memcpy( buf, bda, BD_ADDR_LEN );
    *p++ = numeric_value & 0xff;
    *p++ = (numeric_value >> 8) & 0xff;
    *p++ = (numeric_value >> 16) & 0xff;
    *p++ = (numeric_value >> 24) & 0xff;
    wiced_transport_send_data( HCI_CONTROL_EVENT_USER_CONFIRMATION, buf, 10 );
}


/*
 *  Send Encryption Changed event through UART
 */
void hci_control_send_encryption_changed_evt( uint8_t encrypted ,  wiced_bt_device_address_t bdaddr )
{
    int i;
    const int cmd_size = BD_ADDR_LEN + sizeof(uint8_t);
    uint8_t event_data[cmd_size];
    int     cmd_bytes = 0;

    event_data[cmd_bytes++] = encrypted;

    for ( i = 0 ; i < BD_ADDR_LEN; i++ )                     // bd address
        event_data[cmd_bytes++] = bdaddr[BD_ADDR_LEN - 1 - i];

    wiced_transport_send_data( HCI_CONTROL_EVENT_ENCRYPTION_CHANGED, event_data, cmd_bytes );
}

/*
 *  send audio connect complete event to UART
 */
wiced_result_t hci_control_audio_send_connect_complete( wiced_bt_device_address_t bd_addr, uint8_t status, uint16_t handle )
{
    int i;
    const int cmd_size = BD_ADDR_LEN + sizeof(handle) + sizeof(uint8_t);
    uint8_t event_data[cmd_size];

    WICED_BT_TRACE( "[%s] %B status %x handle %x\n", __FUNCTION__, bd_addr, status, handle );

    //Build event payload
    if ( status == WICED_SUCCESS )
    {
        for ( i = 0; i < BD_ADDR_LEN; i++ )                     // bd address
            event_data[i] = bd_addr[BD_ADDR_LEN - 1 - i];

        event_data[i++] = handle & 0xff;                        //handle
        event_data[i++]   = ( handle >> 8 ) & 0xff;

#if (WICED_TRUE == WICED_APP_AUDIO_RC_TG_INCLUDED)
        event_data[i] = wiced_bt_avrc_tg_is_peer_absolute_volume_capable( );
#endif
        return wiced_transport_send_data( HCI_CONTROL_AUDIO_EVENT_CONNECTED, event_data, cmd_size );
    }
    else
    {
        return wiced_transport_send_data( HCI_CONTROL_AUDIO_EVENT_CONNECTION_FAILED, NULL, 0 );
    }
}

/*
 *  send audio disconnect complete event to UART
 */
wiced_result_t hci_control_audio_send_disconnect_complete( uint16_t handle, uint8_t status, uint8_t reason )
{
    uint8_t event_data[4];

    WICED_BT_TRACE( "[%s] %04x status %d reason %d\n", __FUNCTION__, handle, status, reason );

    //Build event payload
    event_data[0] = handle & 0xff;                          //handle
    event_data[1] = ( handle >> 8 ) & 0xff;
    event_data[2] = status;                                 // status
    event_data[3] = reason;                                 // reason(1 byte)

    return wiced_transport_send_data( HCI_CONTROL_AUDIO_EVENT_DISCONNECTED, event_data, 4 );
}

/*
 *  send audio connect complete event to UART
 */
wiced_result_t hci_control_audio_send_started_stopped( uint16_t handle, wiced_bool_t started )
{
    uint8_t event_data[2];

    WICED_BT_TRACE( "[%s] handle %04x started:%d", __FUNCTION__, handle, started );

    //Build event payload
    event_data[0] = handle & 0xff;                          //handle
    event_data[1] = ( handle >> 8 ) & 0xff;

    return wiced_transport_send_data(started ? HCI_CONTROL_AUDIO_EVENT_STARTED : HCI_CONTROL_AUDIO_EVENT_STOPPED, event_data, 2);
}

/*
 *  send AVRC event to UART
 */
wiced_result_t hci_control_send_avrc_target_event( int type, uint16_t handle )
{
    uint8_t event_data[2];

    WICED_BT_TRACE( "[%s] handle %04x\n", __FUNCTION__, handle );

    //Build event payload
    event_data[0] = handle & 0xff;                          //handle
    event_data[1] = ( handle >> 8 ) & 0xff;

    return wiced_transport_send_data( type, event_data, 2 );
}

/*********************************************************************************************
 * AVRCP controller event handlers
 *********************************************************************************************/
/*
 *  send avrcp controller complete event to UART
 */
wiced_result_t hci_control_avrc_send_connect_complete( wiced_bt_device_address_t bd_addr, uint8_t status, uint16_t handle )
{
    int i = 0;
    const int cmd_size = BD_ADDR_LEN + sizeof(handle) + sizeof(uint8_t);
    uint8_t event_data[cmd_size];

    WICED_BT_TRACE( "[%s] %B status %x handle %x\n", __FUNCTION__, bd_addr, status, handle );

    //Build event payload
    if ( status == WICED_SUCCESS )
    {
        for ( ; i < BD_ADDR_LEN; i++ )                     // bd address
            event_data[i] = bd_addr[BD_ADDR_LEN - 1 - i];

        event_data[i++] = status;

        event_data[i++] = handle & 0xff;                        //handle
        event_data[i++] = ( handle >> 8 ) & 0xff;

    }
    else
    {
        event_data[i++] = status;
    }

    return wiced_transport_send_data( HCI_CONTROL_AVRC_CONTROLLER_EVENT_CONNECTED, event_data, i );
}

/*
 *  send avrcp controller disconnect complete event to UART
 */
wiced_result_t hci_control_avrc_send_disconnect_complete( uint16_t handle )
{
    uint8_t event_data[4];

    WICED_BT_TRACE( "[%s] handle: %04x\n", __FUNCTION__, handle );

    //Build event payload
    event_data[0] = handle & 0xff;                          //handle
    event_data[1] = ( handle >> 8 ) & 0xff;

    return wiced_transport_send_data( HCI_CONTROL_AVRC_CONTROLLER_EVENT_DISCONNECTED, event_data, 2 );
}

/*
 *  send avrcp controller connect complete event to UART
 */
wiced_result_t hci_control_avrc_send_play_status_change( uint16_t handle, uint8_t play_status )
{
    uint8_t event_data[3];

    WICED_BT_TRACE( "[%s] handle %04x\n", __FUNCTION__, handle );

    //Build event payload
    event_data[0] = handle & 0xff;                          //handle
    event_data[1] = ( handle >> 8 ) & 0xff;

    event_data[2] = play_status & 0xff;                      // play status

    return wiced_transport_send_data(HCI_CONTROL_AVRC_CONTROLLER_EVENT_PLAY_STATUS, event_data, 3);
}

/*
 *  send AVRC event to UART
 */

wiced_result_t hci_control_send_avrc_event( int type, uint8_t *p_data, uint16_t data_size )
{
    return wiced_transport_send_data( type, p_data, data_size );
}

/*
 * Write NVRAM function is called to store information in the RAM.  This can be called when
 * stack requires persistent storage, for example to save link keys.  In this case
 * data is also formatted and send to the host for real NVRAM storage.  The same function is
 * called when host pushes NVRAM chunks during the startup.  Parameter from_host in this
 * case is set to WICED_FALSE indicating that data does not need to be forwarded.
 */
int hci_control_write_nvram( int nvram_id, int data_len, void *p_data, wiced_bool_t from_host )
{
    uint8_t                    tx_buf[257];
    uint8_t                   *p = tx_buf;
    hci_control_nvram_chunk_t *p1;
    wiced_result_t            result;

    /* first check if this ID is being reused and release the memory chunk */
    hci_control_delete_nvram( nvram_id, WICED_FALSE );

    /* Allocating a buffer from the pool created for storing the peer info */
    if ( ( p1 = ( hci_control_nvram_chunk_t * )wiced_bt_get_buffer_from_pool( p_key_info_pool ) ) == NULL)
    {
        WICED_BT_TRACE( "Failed to alloc:%d\n", data_len );
        return ( 0 );
    }

    if ( wiced_bt_get_buffer_size( p1 ) < ( sizeof( hci_control_nvram_chunk_t ) + data_len - 1 ) )
    {
        WICED_BT_TRACE( "Insufficient buffer size, Buff Size %d, Len %d  \n",
                        wiced_bt_get_buffer_size( p1 ),
                        ( sizeof( hci_control_nvram_chunk_t ) + data_len - 1 ) );
        wiced_bt_free_buffer( p1 );
        return ( 0 );
    }

    p1->p_next    = p_nvram_first;
    p1->nvram_id  = nvram_id;
    p1->chunk_len = data_len;
    memcpy( p1->data, p_data, data_len );

    p_nvram_first = p1;

    wiced_bt_device_link_keys_t * p_keys = ( wiced_bt_device_link_keys_t *) p_data;
#ifdef CYW20706A2
    result = wiced_bt_dev_add_device_to_address_resolution_db( p_keys, p_keys->key_data.ble_addr_type );
#else
    result = wiced_bt_dev_add_device_to_address_resolution_db( p_keys );
#endif
    WICED_BT_TRACE("Updated Addr Resolution DB:%d\n", result );

    // If NVRAM chunk arrived from host, no need to send it back, otherwise send over transport
    if (!from_host)
    {
        *p++ = nvram_id & 0xff;
        *p++ = (nvram_id >> 8) & 0xff;
        memcpy(p, p_data, data_len);

        wiced_transport_send_data( HCI_CONTROL_EVENT_NVRAM_DATA, tx_buf, ( int )( data_len + 2 ) );
    }
    else
    {
        hci_control_send_command_status_evt( HCI_CONTROL_EVENT_COMMAND_STATUS, HCI_CONTROL_STATUS_SUCCESS );
    }
    return (data_len);
}

/*
 * Find nvram_id of the NVRAM chunk with first bytes matching specified byte array
 */
int hci_control_find_nvram_id(uint8_t *p_data, int len)
{
    hci_control_nvram_chunk_t *p1;

    /* Go through the linked list of chunks */
    for (p1 = p_nvram_first; p1 != NULL; p1 = (hci_control_nvram_chunk_t *)p1->p_next)
    {
        WICED_BT_TRACE( "find %B %B len:%d\n", p1->data, p_data, len );
        if ( memcmp( p1->data, p_data, len ) == 0 )
        {
            return ( p1->nvram_id );
        }
    }
    return HCI_CONTROL_INVALID_NVRAM_ID;
}

/*
 * Delete NVRAM function is called when host deletes NVRAM chunk from the persistent storage.
 */
void hci_control_delete_nvram( int nvram_id, wiced_bool_t from_host )
{
    hci_control_nvram_chunk_t *p1, *p2;

    /* If Delete NVRAM data command arrived from host, send a Command Status response to ack command */
    if (from_host)
    {
        hci_control_send_command_status_evt( HCI_CONTROL_EVENT_COMMAND_STATUS, HCI_CONTROL_STATUS_SUCCESS );
    }

    if ( p_nvram_first == NULL )
    	return;

    /* Special case when need to remove the first chunk */
    if ( ( p_nvram_first != NULL ) && ( p_nvram_first->nvram_id == nvram_id ) )
    {
        p1 = p_nvram_first;

        if ( from_host && ( wiced_bt_dev_delete_bonded_device (p1->data) == WICED_ERROR ) )
        {
            WICED_BT_TRACE("ERROR: while Unbonding device \n");
        }
        else
        {
            p_nvram_first = (hci_control_nvram_chunk_t *)p_nvram_first->p_next;
            wiced_bt_free_buffer( p1 );
        }
        return;
    }

    /* Go through the linked list of chunks */
    for ( p1 = p_nvram_first; p1 != NULL; p1 = (hci_control_nvram_chunk_t *)p1->p_next )
    {
        p2 = (hci_control_nvram_chunk_t *)p1->p_next;

        if ( ( p2 != NULL ) && ( p2->nvram_id == nvram_id ) )
        {
            if ( from_host && ( wiced_bt_dev_delete_bonded_device (p2->data) == WICED_ERROR ) )
            {
                WICED_BT_TRACE("ERROR: while Unbonding device \n");
            }
            else
            {
                p1->p_next = p2->p_next;
                wiced_bt_free_buffer( p2 );
            }
            return;
        }
    }
}

/*
 * Read NVRAM actually finds the memory chunk in the RAM
 */
int hci_control_read_nvram( int nvram_id, void *p_data, int data_len )
{
    hci_control_nvram_chunk_t *p1;
    int                        data_read = 0;

    /* Go through the linked list of chunks */
    for ( p1 = p_nvram_first; p1 != NULL; p1 = p1->p_next )
    {
        if ( p1->nvram_id == nvram_id )
        {
            data_read = ( data_len < p1->chunk_len ) ? data_len : p1->chunk_len;
            memcpy( p_data, p1->data, data_read );
            break;
        }
    }
    return ( data_read );
}

/*
 * Allocate nvram_id to save new NVRAM chunk
 */
int hci_control_alloc_nvram_id( void )
{
    hci_control_nvram_chunk_t *p1 = p_nvram_first;
    int                        nvram_id;
    uint8_t                    allocated_key_pool_count;

    /* Go through the linked list of chunks */
    WICED_BT_TRACE ( "hci_control_alloc_nvram_id\n" );
    for ( nvram_id = HCI_CONTROL_FIRST_VALID_NVRAM_ID; p1 != NULL; nvram_id++ )
    {
        allocated_key_pool_count = 1;

        for ( p1 = p_nvram_first; p1 != NULL; p1 = (hci_control_nvram_chunk_t *)p1->p_next )
        {
            /* If the key buffer pool is becoming full, we need to notify the mcu and disable Pairing.
             * The mcu will need to delete some nvram entries and enable pairing in order to
             * pair with more devices */
            allocated_key_pool_count++;
            if ( ( allocated_key_pool_count == KEY_INFO_POOL_BUFFER_COUNT ) && ( hci_control_cb.pairing_allowed ) )
            {
                // Send Max Number of Paired Devices Reached event message
                wiced_transport_send_data( HCI_CONTROL_EVENT_MAX_NUM_OF_PAIRED_DEVICES_REACHED, NULL, 0 );

                hci_control_cb.pairing_allowed = WICED_FALSE;
                wiced_bt_set_pairable_mode( hci_control_cb.pairing_allowed, 0 );
            }

            if ( p1->nvram_id == nvram_id )
            {
                /* this nvram_id is already used */
                break;
            }
        }
        if ( p1 == NULL )
        {
            break;
        }
    }
    WICED_BT_TRACE ( "hci_control_alloc_nvram_id:%d\n", nvram_id );
    return ( nvram_id );
}

/*
 * Remote Control can work a target or a controller.  This function sets up the appropriate role.
 */
void hci_control_switch_avrcp_role(uint8_t new_role)
{
#if ( (WICED_APP_AUDIO_RC_TG_INCLUDED == WICED_TRUE) && (WICED_APP_AUDIO_RC_CT_INCLUDED == WICED_TRUE) )
    WICED_BT_TRACE ( "[%s] New Role: %d \n", __FUNCTION__, new_role);

    if (new_role != avrcp_profile_role)
    {
        switch (avrcp_profile_role)
        {
        case AVRCP_TARGET_ROLE:
            /* Shutdown the avrcp target */
            wiced_bt_avrc_tg_initiate_close();

            /* Initialize the avrcp controller */
            hci_control_rc_controller_init();

            avrcp_profile_role = new_role;
            break;

        case AVRCP_CONTROLLER_ROLE:
            /* Shutdown the avrcp controller */
            wiced_bt_avrc_ct_cleanup();

            /* Initialize the avrcp target */
            hci_control_rc_target_init();
            wiced_bt_avrc_tg_register();

            avrcp_profile_role = new_role;
            break;

        default:
            break;
        }
    }
#endif
}

/*
 * hci_control_transport_tx_cplt_cback.
 * This function is called when a Transport Buffer has been sent to the MCU
 */
static void hci_control_transport_tx_cplt_cback( wiced_transport_buffer_pool_t* p_pool )
{
    WICED_BT_TRACE( " hci_control_transport_tx_cplt_cback %x \n", p_pool );
}

static void hci_control_transport_status( wiced_transport_type_t type )
{
    WICED_BT_TRACE( " hci_control_transport_status %x \n", type );
    hci_control_send_device_started_evt();
}

