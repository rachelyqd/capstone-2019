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

#ifndef ANC_CLIENT__H
#define ANC_CLIENT__H

#ifdef __cplusplus
extern "C" {
#endif

#include "wiced_bt_anp.h"

enum
{
    ANC_DISCOVER_RESULT,
    ANC_READ_SUPPORTED_NEW_ALERTS_RESULT,
    ANC_READ_SUPPORTED_UNREAD_ALERTS_RESULT,
    ANC_CONTROL_ALERTS_RESULT,
    ANC_ENABLE_NEW_ALERTS_RESULT,
    ANC_DISABLE_NEW_ALERTS_RESULT,
    ANC_ENABLE_UNREAD_ALERTS_RESULT,
    ANC_DISABLE_UNREAD_ALERTS_RESULT,
};
typedef uint8_t anc_operation_result_type_t;

typedef struct
{
    uint16_t      conn_id;
    wiced_bool_t  result;
}wiced_bt_anc_discovery_result_t;

typedef struct
{
    uint16_t                                conn_id;
    wiced_bt_anp_alert_category_enable_t    supported_alerts;
    wiced_bt_gatt_status_t                  result;
}wiced_bt_anc_supported_new_alerts_result_t;

typedef struct
{
    uint16_t                                conn_id;
    wiced_bt_anp_alert_category_enable_t    supported_alerts;
    wiced_bt_gatt_status_t                  result;
}wiced_bt_anc_supported_unread_alerts_result_t;

typedef struct
{
    uint16_t                             conn_id;
    wiced_bt_anp_alert_control_cmd_id_t  control_point_cmd_id;
    wiced_bt_anp_alert_category_id_t     category_id;
    wiced_bt_gatt_status_t               result;
}wiced_bt_anc_control_alerts_result_t;

typedef struct
{
    uint16_t                conn_id;
    wiced_bt_gatt_status_t  result;
}wiced_bt_anc_enable_disable_alerts_result_t;


typedef union
{
    wiced_bt_anc_discovery_result_t                 discovery_result;
    wiced_bt_anc_supported_new_alerts_result_t      supported_new_alerts_result;
    wiced_bt_anc_supported_unread_alerts_result_t   supported_unread_alerts_result;
    wiced_bt_anc_control_alerts_result_t            control_alerts_result;
    wiced_bt_anc_enable_disable_alerts_result_t     enable_disable_alerts_result;
}wiced_bt_anc_op_result_data_t;

typedef struct
{
    anc_operation_result_type_t            result_type;
    wiced_bt_anc_op_result_data_t          result_data;
}wiced_bt_anc_op_result_t;

typedef void (*wiced_bt_anc_operation_complete_callback_t)(wiced_bt_anc_op_result_t *result_data);

typedef struct
{
    uint16_t                           conn_id;
    wiced_bt_anp_alert_category_id_t   new_alert_type;
    uint8_t                            new_alert_count;
    char                               *p_last_alert_data; /* Null terminated string */
}anc_new_alert_event_t;

typedef struct
{
    uint16_t                           conn_id;
    wiced_bt_anp_alert_category_id_t   unread_alert_type;
    uint8_t                            unread_count;
}anc_unread_alert_event_t;

typedef void (*wiced_bt_anc_new_alert_callback_t)(anc_new_alert_event_t *p_new_alert);
typedef void (*wiced_bt_anc_unread_alert_callback_t)(anc_unread_alert_event_t *p_unread_alert);

typedef struct
{
    wiced_bt_anc_operation_complete_callback_t p_op_complete_callback; /**< callback to be executed when ANC library completes application requested operation */
    wiced_bt_anc_new_alert_callback_t          p_new_alert_callback; /**< callback to be executed when ANC library receives Alerts from the server */
    wiced_bt_anc_unread_alert_callback_t       p_unread_alert_callback;
}wiced_bt_anc_app_reg_t;

/*****************************************************************************
 *                         Function Prototypes
 *****************************************************************************/

/**
 * Alert notification profile gets initialized (ex: Allocates control block).
 * Application registered call backs gets called when requested operations completed.
 * Once connection success, Application does ANS service discovery.If ANS service is discovered successfully, calls below for Characteristic discovery
 *
 * @param           p_reg  : Registration control block that includes ANC application call backs.
 * @return          WICED_TRUE if ANC initialized successfully, WICED_FALSE otherwise.
 */
wiced_result_t wiced_bt_anc_initialize( wiced_bt_anc_app_reg_t *app_reg_cb );

/**
 * Performs ANC characteristics discovery and characteristic descriptor discovery.
 * Once discovery complete, registered application p_op_complete_callback is called with result of operation.
 *
 * @param           conn_id  : GATT connection ID.
 * @param           start_handle : Start GATT handle of the ANC service.
 * @param           end_handle : End GATT handle of the ANC service.
 * @return          WICED_TRUE if GATT discovery started successfully, WICED_FALSE otherwise.
 */
wiced_result_t wiced_bt_anc_discover( uint16_t conn_id, uint16_t start_handle, uint16_t end_handle );

/**
 * While application performs GATT discovery it shall pass discovery results for
 * the ANC service to the ANC Library. The library needs to find ANC service characteristics
 * and associated characteristic client configuration descriptors.
 *
 * @param           p_data   : Discovery result data as passed from the stack.
 * @return          none
 */
void wiced_bt_anc_discovery_result( wiced_bt_gatt_discovery_result_t *p_data );

/**
 * While application performs GATT discovery it shall pass discovery complete callbacks
 * for the ANC service to the ANC Library. As the GATT discovery is performed in multiple steps
 * this function initiates the next discovery request.
 *
 * @param           p_data   : Discovery complete data as passed from the stack.
 * @return          none
 */
void wiced_bt_anc_client_discovery_complete( wiced_bt_gatt_discovery_complete_t *p_data );

/**
 * On Success of discovery complete, the Application should read the supported new alerts.
 *
 * @param          conn_id: GATT connection id.
 * @return         WICED_TRUE if new supported alerts found, else WICED_FALSE.
 */
wiced_bt_gatt_status_t wiced_bt_anc_read_server_supported_new_alerts( uint16_t conn_id );

/**
 * Read the Value of Supported Unread Alert Categories.
 * Application provides GATT operation result through wiced_bt_anc_read_response (wiced_bt_gatt_operation_complete_t *p_data) to profile.
 * Profile shall parse the data and provide the result through p_op_complete_callback.
 *
 * @param          conn_id: GATT connection id.
 * @return         WICED_TRUE if new supported alerts found, else WICED_FALSE.
 */
wiced_bt_gatt_status_t wiced_bt_anc_read_server_supported_unread_alerts( uint16_t conn_id );

/**
 * Control notifications using Alert notification control point characteristic.
 * Application provides GATT operation result through wiced_bt_anc_write_response to profile.
 * Profile shall parse the data and provide the result through p_op_complete_callback.
 *
 * @param          conn_id: GATT connection id.
 * @param          cmd_id:  ANC alert command id.
 * @param          category: ANC alert category id.
 * @return         WICED_TRUE if new supported alerts found, else WICED_FALSE.
 */
wiced_bt_gatt_status_t wiced_bt_anc_control_required_alerts( uint16_t conn_id , wiced_bt_anp_alert_control_cmd_id_t cmd_id, wiced_bt_anp_alert_category_id_t category);

/**
 * Write client characteristic configuration descriptor to receive new alerts.
 * Application provides GATT operation result through wiced_bt_anc_write_response to profile.
 * Profile shall parse the result and provide enable success result through p_op_complete_callback.
 *
 * @param          conn_id: GATT connection id.
 * @return         WICED_TRUE if enable new alerts is success, else WICED_FALSE.
 */
wiced_bt_gatt_status_t wiced_bt_anc_enable_new_alerts( uint16_t conn_id );

/**
 * Write client characteristic configuration descriptor to stop receiving new alerts.
 * Application provides GATT operation result through wiced_bt_anc_write_response to profile.
 * Profile shall parse the result and provide disable success result through p_op_complete_callback.
 *
 * @param          conn_id: GATT connection id.
 * @return         WICED_TRUE if disable new alerts is success, else WICED_FALSE.
 */
wiced_bt_gatt_status_t wiced_bt_anc_disable_new_alerts( uint16_t conn_id );

/**
 * Write client characteristic configuration descriptor to receive new unread alerts.
 * Application provides GATT operation result through wiced_bt_anc_write_response to profile.
 * Profile shall parse the result and provide enable success result through p_op_complete_callback.
 *
 * @param          conn_id: GATT connection id.
 * @return         WICED_TRUE if enable new unread alerts is success, else WICED_FALSE.
 */
wiced_bt_gatt_status_t wiced_bt_anc_enable_unread_alerts( uint16_t conn_id );

/**
 * Write client characteristic configuration descriptor to stop receiving new unread alerts.
 * Application provides GATT operation result through wiced_bt_anc_write_response to profile.
 * Profile shall parse the result and provide enable success result through p_op_complete_callback.
 *
 * @param          conn_id: GATT connection id.
 * @return         WICED_TRUE if disable new unread alerts is success, else WICED_FALSE.
 */
wiced_bt_gatt_status_t wiced_bt_anc_disable_unread_alerts( uint16_t conn_id );

/**
 * This function shall sequentially call wiced_bt_anc_enable_new_alerts and wiced_bt_anc_control_required_alerts API inside the library.
 *
 * @param          conn_id: GATT connection id.
 * @param          cmd_id:  ANC alert command id.
 * @param          category: ANC alert category id.
 * @return         WICED_TRUE if new supported alerts found, else WICED_FALSE.
 */
wiced_result_t wiced_bt_anc_recover_new_alerts_from_conn_loss(uint16_t conn_id, wiced_bt_anp_alert_control_cmd_id_t cmd_id, wiced_bt_anp_alert_category_id_t category);

/**
 * This function shall sequentially call wiced_bt_anc_enable_unread_alerts and wiced_bt_anc_control_required_alerts API inside the library.
 *
 * @param          conn_id: GATT connection id.
 * @param          cmd_id:  ANC alert command id.
 * @param          category: ANC alert category id.
 * @return         WICED_TRUE if new supported alerts found, else WICED_FALSE.
 */
wiced_result_t wiced_bt_anc_recover_new_unread_alerts_from_conn_loss(uint16_t conn_id, wiced_bt_anp_alert_control_cmd_id_t cmd_id, wiced_bt_anp_alert_category_id_t category);

/**
 * The application should call this function when it receives GATT Write Response
 * for the attribute handle which belongs to the ANC service.
 *
 * @param           p_data  : pointer to a GATT operation complete data structure.
 * @return          none
 */
void wiced_bt_anc_write_rsp(wiced_bt_gatt_operation_complete_t *p_data);

/**
 * The application should call this function when it receives GATT Read Response
 * for the attribute handle which belongs to the ANC service.
 *
 * @param           p_data  : pointer to a GATT operation complete data structure.
 * @return          none
 */
void wiced_bt_anc_read_rsp(wiced_bt_gatt_operation_complete_t *p_data);

/**
 * The application should call this function when BLE connection with a peer
 * device has been established.
 *
 * @param           p_conn_status  : pointer to a wiced_bt_gatt_connection_status_t which includes the address and connection ID.
 * @return          none
 */
void wiced_bt_anc_client_connection_up(wiced_bt_gatt_connection_status_t *p_conn_status);

/**
 * The application should call this function when BLE connection with a peer
 * device has been disconnected.
 *
 * @param           p_conn_status  : pointer to a wiced_bt_gatt_connection_status_t which includes the address and connection ID.
 * @return          none
 */
void wiced_bt_anc_client_connection_down(wiced_bt_gatt_connection_status_t *p_conn_status);

/**
 * This function processes the ANC process Notification
 *
 * @param           p_data  : pointer to a GATT operation complete data structure.
 * @return          none
 */
void wiced_bt_anc_client_process_notification(wiced_bt_gatt_operation_complete_t *p_data);

#ifdef __cplusplus
}
#endif

/**@} wiced_bt_anc_api_functions */

#endif
