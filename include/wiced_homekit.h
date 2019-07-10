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

#ifndef WICED_HOMEKIT_H
#define WICED_HOMEKIT_H

#include "wiced.h"
#include "wiced_bt_gatt.h"
#include "wiced_homekit_defs.h"

#ifdef __cplusplus
extern "C" {
#endif


/******************************************************
 *                    Constants
 ******************************************************/
// Homekit library uses all NVRAM IDs between APPLE_HOMEKIT_NVRAM_VSID_START and APPLE_HOMEKIT_NVRAM_VSID_END
// Application may not use them
#ifdef CYW20706A2
#define APPLE_HOMEKIT_NVRAM_VSID_START  (WICED_NVRAM_VSID_END - 30)
#define APPLE_HOMEKIT_NVRAM_VSID_END    (WICED_NVRAM_VSID_END - 1)
#else
#define APPLE_HOMEKIT_NVRAM_VSID_START  (WICED_NVRAM_VSID_END - 200)
#define APPLE_HOMEKIT_NVRAM_VSID_END    (WICED_NVRAM_VSID_END - 171)
#endif


// HomeKit firmware update service UUIDs

#define UUID_HAP_FW_UPDATE_SERVICE                          0xef, 0xa0, 0x82, 0xc9, 0xd1, 0x25, 0x44, 0xa3, 0x00, 0x4c, 0x34, 0x84, 0x83, 0x3e, 0x73, 0xb0
#define UUID_HAP_FW_UPDATE_CHARACTERISTIC_CONTROL_POINT     0x83, 0x61, 0xe9, 0x96, 0x07, 0x9d, 0xc6, 0xa6, 0xbd, 0x47, 0x48, 0x41, 0x7f, 0xbd, 0x76, 0xb1
#define UUID_HAP_FW_UPDATE_CHARACTERISTIC_DATA              0x97, 0xc6, 0x58, 0xc7, 0x2e, 0x20, 0x6c, 0xb1, 0x17, 0x4f, 0xd3, 0xea, 0x2d, 0x7f, 0xfd, 0xb2
#define UUID_HAP_FW_UPDATE_SERVICE_CHARACTERISTIC_APP_INFO  0x96, 0x51, 0x82, 0x57, 0x6f, 0x95, 0x8a, 0x83, 0xc2, 0x45, 0xcc, 0x9a, 0xa5, 0x59, 0x12, 0xb3


/*
 *  HomeKit GATT database handles
 */
enum
{
    HDLS_PAIRING                                  = 0xf0,
    HDLC_PAIRING_SERVICE_INSTANCE                 = 0xf1,
    HDLC_PAIRING_SERVICE_INSTANCE_VALUE           = 0xf2,
    HDLC_PAIRING_PAIR_SETUP                       = 0xf3,
    HDLC_PAIRING_PAIR_SETUP_VALUE                 = 0xf4,
    HDLD_PAIRING_PAIR_SETUP_INSTANCE_ID           = 0xf5,
    HDLC_PAIRING_PAIR_VERIFY                      = 0xf6,
    HDLC_PAIRING_PAIR_VERIFY_VALUE                = 0xf7,
    HDLD_PAIRING_PAIR_VERIFY_INSTANCE_ID          = 0xf8,
    HDLC_PAIRING_FEATURES                         = 0xf9,
    HDLC_PAIRING_FEATURES_VALUE                   = 0xfa,
    HDLD_PAIRING_FEATURES_INSTANCE_ID             = 0xfb,
    HDLC_PAIRING_MANAGE                           = 0xfc,
    HDLC_PAIRING_MANAGE_VALUE                     = 0xfd,
    HDLD_PAIRING_MANAGE_INSTANCE_ID               = 0xfe,

    HDLS_OTA_FW_UPGRADE_SERVICE                         = 0xff11,
    HDLC_OTA_FW_UPGRADE_SERVICE_INSTANCE                = 0xff12,
    HDLC_OTA_FW_UPGRADE_SERVICE_INSTANCE_VALUE          = 0xff13,
    HDLC_OTA_FW_UPGRADE_SERVICE_SIGNATURE               = 0xff14,
    HDLC_OTA_FW_UPGRADE_SERVICE_SIGNATURE_VALUE         = 0xff15,
    HDLD_OTA_FW_UPGRADE_SERVICE_SIGNATURE_INSTANCE_ID   = 0xff16,
    HDLC_OTA_FW_UPGRADE_CONTROL_POINT                   = 0xff17,
    HDLC_OTA_FW_UPGRADE_CONTROL_POINT_VALUE             = 0xff18,
    HDLD_OTA_FW_UPGRADE_CONTROL_POINT_INSTANCE_ID       = 0xff19,
    HDLD_OTA_FW_UPGRADE_CONTROL_POINT_CLNT_CHAR_CFG     = 0xff1a,
    HDLC_OTA_FW_UPGRADE_DATA                            = 0xff1b,
    HDLC_OTA_FW_UPGRADE_DATA_VALUE                      = 0xff1c,
    HDLD_OTA_FW_UPGRADE_DATA_INSTANCE_ID                = 0xff1d,
    HDLC_OTA_FW_UPGRADE_APP_INFO                        = 0xff1e,
    HDLC_OTA_FW_UPGRADE_APP_INFO_VALUE                  = 0xff1f,
    HDLD_OTA_FW_UPGRADE_APP_INFO_INSTANCE_ID            = 0xff20,
};

typedef enum
{
    WICED_HOMEKIT_EVENT_PAIRED                      = 1,
    WICED_HOMEKIT_EVENT_UNPAIRED,
    WICED_HOMEKIT_EVENT_PAIR_VERIFIED,
    WICED_HOMEKIT_EVENT_PAIR_UNVERIFIED,
} wiced_homekit_event_t;

/******************************************************
 *                   Enumerations
 ******************************************************/

/******************************************************
 *                    Structures
 ******************************************************/
/* HAP Info */
typedef struct
{
    uint16_t        category;
    uint8_t         pairing_feature_flags;
    uint8_t *       model;
    uint8_t         model_length;
    uint8_t *       protocol_version;
    uint8_t         protocol_version_length;
} wiced_btle_hap_info_t;

/* HAP characteristic parameters */
typedef struct
{
    /* app provided parameters */
    uint16_t        handle;
    uint16_t        inst_id_handle;
    uint16_t        properties;
    uint8_t         format;
    uint16_t        unit;
    uint32_t        min;
    uint32_t        max;
    uint32_t        step;

    /* internal variables */
    uint8_t         request_tid;
    uint16_t        config_properties;
    uint8_t         broadcast_interval;
} wiced_btle_hap_char_params_t;

/* HAP instance ID table */
typedef struct
{
    uint16_t        handle;
    uint16_t        id;
} wiced_btle_instance_id_t;

/**
 * HAP request callback function to application
 */
typedef void wiced_btle_hap_request_cback_t(uint16_t conn_id, uint16_t handle, uint8_t* p_data, uint16_t len);

/**
 * HomeKit event callback function
 */
typedef void wiced_btle_homekit_event_cback_t(wiced_homekit_event_t event, uint8_t* p_param, uint32_t length);

typedef wiced_bool_t(*wiced_btle_homekit_is_pair_allowed_t)();
typedef wiced_result_t (*wiced_homekit_display_password_callback_t)( uint8_t* accessory_password );

/******************************************************
 *               Function Declarations
 ******************************************************/

/**
 * Use this API to initialize homekit library.  This should be called once from APPLICATION_INIT function.
 */
void wiced_btle_homekit_init(wiced_btle_hap_info_t* p_hap_info);

/**
 * Set SCL/SDA pins for MFi auth chip
 */
void wiced_btle_homekit_auth_chip_setup(uint8_t scl_pin, uint8_t sda_pin);

/**
 * Initialize firmware upgrade library
 */
void wiced_btle_homekit_fw_upgrade_init();

/**
 * Use this API to start HomeKit library
 */
wiced_result_t wiced_btle_homekit_start();

/**
* Scan GATT database and initialize HomeKit related parameters
*/
wiced_result_t wiced_btle_homekit_gatt_db_init(void);

/**
* Use this API to call homekit library every second
*/
void wiced_btle_homekit_timer();

/**
* Use this API to detect if accessory is in the pair-verified state
*/
wiced_bool_t wiced_btle_homekit_is_accessory_pair_verified(void);

/**
* Use this API to set homekit advertisement data
*/
void wiced_btle_homekit_set_advertisement_data(void);

/**
* Use this API to register HAP characteristics and a callback for HAP request.
*/
wiced_result_t wiced_btle_homekit_register_hap_characteristics(wiced_btle_hap_char_params_t* p_hap_chars,
                                                               int num_of_chars,
                                                               wiced_btle_hap_request_cback_t* p_callback);

/**
* Use this API to register HAP instance ID table
*/
wiced_result_t wiced_btle_homekit_register_hap_instance_ids(wiced_btle_instance_id_t* p_hap_instance_ids,
                                                            int num_of_ids);

/**
* Register HomeKit event callback function
*/
wiced_result_t wiced_btle_homekit_register_event_callback(wiced_btle_homekit_event_cback_t* p_callback);

/**
* Use this API to send back HAP response after a HAP request had been processed.
*/
wiced_result_t wiced_btle_homekit_send_hap_response(uint16_t conn_id, uint16_t handle, uint8_t* p_data, uint16_t len);

/**
* Use this API to tell HomeKit library to increase global state number
*/
wiced_bool_t wiced_btle_homekit_change_global_state_number(wiced_bool_t reset);

/**
* Use this API to tell HomeKit library to increment configuration number
*/
void wiced_btle_homekit_change_configuration_number();

/**
* Use this API to tell HomeKit library the value of an indication enabled characteristic had changed
*/
void wiced_btle_homekit_char_value_changed(uint16_t handle, uint8_t *p_val, uint32_t val_len);

/**
* Use this API to find specific parameter in HAP PDU, returns pointer to the TLV8
*/
hap_pdu_tlv8_t* wiced_btle_homekit_find_hap_param(uint8_t type, uint8_t* p_data, uint32_t data_len);
hap_pdu_tlv8_t* wiced_btle_homekit_find_next_hap_param(uint8_t type, uint8_t* p_data, uint32_t data_len);

/**
* Use this API to get specific parameter data from HAP PDU.  This is to retrieve large data that might be
* separated in multiple parameters.  Caller needs to provide buffer to store retrieved data.
*/
wiced_result_t wiced_btle_homekit_get_hap_param(uint8_t type, uint8_t* p_data, uint32_t data_len, uint8_t* p_out, uint32_t* p_out_len);

/**
* Use this API to set properties of a service (primary service/hidden service)
*/
wiced_result_t wiced_btle_homekit_set_service_properties(uint16_t handle, uint16_t properties);

/**
* Use this API to add linked service to a service
*/
wiced_result_t wiced_btle_homekit_add_linked_service(uint16_t handle, uint16_t linked_service_id);

/**
 *  Read MFi chip information
 */
wiced_result_t wiced_btle_homekit_get_auth_chip_info(uint8_t* buffer, uint32_t buffer_size, uint8_t scl_pin, uint8_t sda_pin);

/**
 *  Check if firmware update is going on (app should not idle timeout during firmware update)
 */
wiced_bool_t wiced_btle_homekit_is_fw_updating();

/**
 *  Check NVRAM to see if we are paired
 */
wiced_bool_t wiced_btle_homekit_is_accessory_paired();

/**
 *  Get software authentication token
 */
wiced_result_t wiced_btle_homekit_get_software_token(uint8_t* p_buffer, uint32_t* p_len);

/**
 *  GATT server HomeKit event handler
 */
wiced_bt_gatt_status_t wiced_btle_homekit_gatt_event_handler(wiced_bt_gatt_evt_t event, wiced_bt_gatt_event_data_t *p_event_data);

/**
 *  Factory reset
 */
void wiced_homekit_reset();

/**
* If application wants to disable pairing of then it should set the callback function.
* Callback function is_pair_allowed_callback should return 0 to disable pairing attempt
* or return non-0 to enable it
*/
void wiced_btle_homekit_set_is_pair_allowed_callback(wiced_btle_homekit_is_pair_allowed_t is_pair_allowed_callback);

/**
 * If accessory has a display, register this callback to retrieve password as soon as it is available
 *
 * @param[in] callback  The callback to be display or hand over the password once it is generated
 *
 */
wiced_result_t wiced_configure_accessory_password_for_device_with_display(wiced_homekit_display_password_callback_t callback);

/**
 * If accessory has no display, create password in format XXX-XX-XXX where X is a digit between 0-9
 */
wiced_result_t wiced_configure_accessory_password_for_device_with_no_display(char* password);

/**
 * Configure accessory setup ID in HomeKit library
 */
wiced_result_t wiced_configure_accessory_setup_id(char* setup_id);

#ifdef __cplusplus
}
#endif

#endif // WICED_HOMEKIT_H
