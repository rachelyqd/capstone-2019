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

#ifndef BATTERY_CLIENT__H
#define BATTERY_CLIENT__H

#ifdef __cplusplus
extern "C" {
#endif

typedef void (*wiced_bt_bas_operation_complete_callback_t)(uint16_t conn_id, wiced_bool_t res_status);

typedef void (*wiced_bt_battery_status_callback_t)(uint16_t conn_id, uint8_t battery_level);

typedef void (*wiced_bt_battery_notify_callback_t)(uint8_t notify_val);

typedef struct
{
    wiced_bt_bas_operation_complete_callback_t p_op_complete_callback; /**< callback to be executed when BAS library completes application requested operation */
    wiced_bt_battery_status_callback_t         p_battery_level_callback; /**< callback to be executed when BAS library receives Battery Level from the server */
    wiced_bt_battery_notify_callback_t         p_battery_notify_callback;/**< callback to be executed when notifications is enabled */
}wiced_bt_bas_app_reg_t;


/*****************************************************************************
 *                         Function Prototypes
 *****************************************************************************/

/**
 * Battery Service Client profile gets initialized (ex: Allocates control block).
 * Application registered call backs gets called when requested operations completed.
 * Once connection success, Application does BAS service discovery.If BAS service is discovered successfully, calls below for Characteristic discovery
 *
 * @param          p_reg  : Registration control block that includes BAS application call backs.
 * @return          WICED_TRUE if BAS initialized successfully, WICED_FALSE otherwise.
 */
wiced_result_t wiced_bt_bas_initialize( wiced_bt_bas_app_reg_t *app_reg_cb);

/**
 * Performs Battery Service characteristics discovery and characteristic descriptor discovery.
 * Once discovery complete, registered application p_op_complete_callback is called with result of operation.
 *
 * @param           conn_id  : GATT connection ID.
 * @param           start_handle : Start GATT handle of the ANP service.
 * @param           end_handle : End GATT handle of the ANP service.
 * @return           WICED_TRUE if GATT discovery started successfully, WICED_FALSE otherwise.
 */
wiced_result_t wiced_bt_bas_discover( uint16_t conn_id, uint16_t start_handle, uint16_t end_handle);

/**
 * While application performs GATT discovery it shall pass discovery results of
 * the Battery service to the Battery Service CLient Library.
 * The library needs to find Battery service characteristics
 * and associated characteristic client configuration descriptors.
 *
 * @param           p_data   : Discovery result data as passed from the stack.
 * @return          none
 */
void wiced_bt_bas_discovery_result(wiced_bt_gatt_discovery_result_t *p_data);

/**
 * While application performs GATT discovery it shall pass discovery complete callbacks
 * for the Battery service to the Battery Service Client Library. As the GATT discovery is performed in multiple steps
 * this function initiates the next discovery request.
 *
 * @param           p_data   : Discovery complete data as passed from the stack.
 * @return          none
 */
void wiced_bt_bas_client_discovery_complete(wiced_bt_gatt_discovery_complete_t *p_data);

/**
 * On Success of discovery complete, the Application should read the battery level provided by the server.
 *
 * @param          conn_id: GATT connection id.
 * @return         WICED_TRUE if successfully read battery level, else WICED_FALSE.
 */
wiced_result_t wiced_bt_bas_read_battery_level( uint16_t conn_id );

/**
 * Write client characteristic configuration descriptor to receive battery level notifications from server.
 *
 * @param          conn_id: GATT connection id.
 * @return          returns GATT status codes.
 */
wiced_bt_gatt_status_t wiced_bt_bas_enable_notification( uint16_t conn_id );

/**
 * Write client characteristic configuration descriptor to stop receiving battery level notifications from server.
 *
 * @param          conn_id: GATT connection id.
 * @return          eturns GATT status codes.
 */
wiced_bt_gatt_status_t wiced_bt_bas_disable_notification( uint16_t conn_id );

/**
 * The application should call this function when it receives GATT Read Response
 * for the attribute handle which belongs to the BAS service.
 *
 * @param           p_data  : pointer to a GATT operation complete data structure.
 * @return          none
 */
void wiced_bt_bas_read_rsp(wiced_bt_gatt_operation_complete_t *p_data);

/**
 * The application calls this when it receives notification from the server.
 * The Profile processes the notification and sends it as a callback to the application.
 *
 * @param          p_data  : pointer to a GATT operation complete data structure.
 * @return           none
 */
void wiced_bt_bas_process_notification(wiced_bt_gatt_operation_complete_t *p_data);

/**
 * The application should call this function when BLE connection with a peer
 * device has been established.
 *
 * @param           p_conn_status  : pointer to a wiced_bt_gatt_connection_status_t which includes the address and connection ID.
 * @return          none
 */
void wiced_bt_bas_client_connection_up(wiced_bt_gatt_connection_status_t *p_conn_status);

/**
 * The application should call this function when BLE connection with a peer
 * device has been disconnected.
 *
 * @param           p_conn_status  : pointer to a wiced_bt_gatt_connection_status_t which includes the address and connection ID.
 * @return          none
 */
void wiced_bt_bas_client_connection_down(wiced_bt_gatt_connection_status_t *p_conn_status);

#ifdef __cplusplus
}
#endif

/**@} wiced_bt_bas_api_functions */

#endif

