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
 * This file provides definitions of the Alert Notification Server (ANS) library interface
 */

#ifndef WICED_BT_ANS__H
#define WICED_BT_ANS__H

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @addtogroup  wiced_bt_ans_api_functions        ANS Library API
 * @ingroup     wicedbt
 *
 * ANS library of the WICED SDK provide a simple method for an application to integrate ANS
 * service functionality. Application calls the library APIs to configure supported alerts,
 * to process alert notification client gatt read, write requests for enable/disable/control the alerts.
 * @{
 */
#include "wiced_bt_anp.h"
#include "wiced_bt_gatt.h"
#include "wiced_bt_types.h"

/* GATT handles used by ANP server application and library */
// ***** Primary Service 'Alert Notification'
#define HDLS_ALERT_NOTIFICATION                                     0x0028
// ----- Characteristic 'Supported New Alert Category'
#define HDLC_SUPPORTED_NEW_ALERT_CATEGORY                           0x0029
#define HDLC_SUPPORTED_NEW_ALERT_CATEGORY_VALUE                     0x0030
// ----- Characteristic 'Supported New Alert'
#define HDLC_NEW_ALERT                                              0x0031
#define HDLC_NEW_ALERT_VALUE                                        0x0032
#define HDLC_NEW_ALERT_CLIENT_CONFIG_DESCRIPTOR                     0x0033
// ----- Characteristic 'Supported Unread Alert Category'
#define HDLC_SUPPORTED_UNREAD_ALERT_CATEGORY                        0x0034
#define HDLC_SUPPORTED_UNREAD_ALERT_CATEGORY_VALUE                  0x0035
// ----- Characteristic 'Supported Unread status'
#define HDLC_UNREAD_ALERT_STATUS                                    0x0036
#define HDLC_UNREAD_ALERT_STATUS_VALUE                              0x0037
#define HDLC_UNREAD_ALERT_STATUS_CLIENT_CONFIG_DESCRIPTOR           0x0038
// ----- Characteristic 'Alert Notification Control Point'
#define HDLC_ALERT_NOTIFICATION_CONTROL_POINT                       0x0039
#define HDLC_ALERT_NOTIFICATION_CONTROL_POINT_VALUE                 0x003A

/*****************************************************************************
 *          Function Prototypes
 *****************************************************************************/

/**
 * Application call this API on application start to initialize WICED BT ANS server library.
 *
 * @param           none
 *
 * @return          none
 */
void wiced_bt_ans_initialize ( void );

/**
 * Application call this API when application connected with alert notification client
 *
 * @param           conn_id : GATT connection ID
 *
 * @return          none
 */
void wiced_bt_ans_connection_up(uint16_t conn_id);

/**
 * Application call this API when application disconnected from client
 *
 * @param           conn_id : GATT connection ID
 *
 * @return          none
 */
void wiced_bt_ans_connection_down(uint16_t conn_id);

/**
 * Application call this API to set Server supported new alert categories. This API should be 
 * called before application connect to alert notification client. Supported new alerts static
 * during the connection.
 *
 * @param           conn_id : GATT connection ID
 * @param           supported_new_alert_cat  : Server supported new alert categories.
 *                      Each category represented by bit. Bit positioned by category id value. see @ref ANP_ALERT_CATEGORY_ENABLE."Alert category enable bit mask".
 *
 * @return          none
 */
void wiced_bt_ans_set_supported_new_alert_categories (uint16_t conn_id, wiced_bt_anp_alert_category_enable_t supported_new_alert_cat);

/**
 * Application call this API to set Server supported unread alert categories. This API should be 
 * called before application connect to alert notification client. Supported unread alerts static
 * during the connection.
 *
 * @param           conn_id : GATT connection ID
 * @param           supported_unread_alert_cat  : Server supported unread alert categories.
 *                      Each category represented by bit. Bit positioned by category id value. see @ref ANP_ALERT_CATEGORY_ENABLE."Alert category enable bit mask".
 *
 * @return          none
 */
void wiced_bt_ans_set_supported_unread_alert_categories (uint16_t conn_id, wiced_bt_anp_alert_category_enable_t supported_unread_alert_cat);

/**
 * Application call this API to process alert notification client GATT read requets
.* Alert notification client uses GATT read procedure to know server supported new alerts,
 * unread alerts and to check GATT notification configuration enabled/disabled for new alerts and/or unread alerts 
 *
 * @param           conn_id : GATT connection ID
 * @param           p_read  : GATT read request.
 *
 * @return          Status of the GATT read.operation
 */
wiced_bt_gatt_status_t wiced_bt_ans_process_gatt_read_req(uint16_t conn_id, wiced_bt_gatt_read_t *p_read);

/**
 * Application call this API to process alert notification client GATT write requets
.* Alert notification client uses GATT write procedure to configure, enable/disable and to control the alerts.
 *
 * @param           conn_id : GATT connection ID
 * @param           p_write  : GATT write request.
 *
 * @return          Status of the GATT write.operation
 */
wiced_bt_gatt_status_t wiced_bt_ans_process_gatt_write_req(uint16_t conn_id, wiced_bt_gatt_write_t *p_write);

/**
 * Application call this API to process and send the new alert.on given alert category id.
.* Library increments the new alert count of the specified category.
 * Library sends new alert to alert notification client if client already configured to receive
 * notification of the given category, otherwise server send the new alert wheneven client enables.
 * Count will be cleared when the alert sent to client or when application asks to clear the alerts
 * 
 * @param           conn_id : GATT connection ID
 * @param           category_id  : New Alert category id. see @ref ANP_ALERT_CATEGORY_ID."Alert category id".
 *
 * @return          Status of the GATT notification
 */
wiced_bt_gatt_status_t wiced_bt_ans_process_and_send_new_alert(uint16_t conn_id, wiced_bt_anp_alert_category_id_t category_id);

/**
 * Application call this API to process and send the unread alert.on given alert category id.
.* Library increments the unread alert count of the specified category.
 * Library sends unread alert to alert notification client if client already configured to receive
 * notification of the given category, otherwise server send the unread alert wheneven client enables.
 * Count will be cleared when the alert sent to client or when application asks to clear the alerts
 * 
 * @param           conn_id : GATT connection ID
 * @param           category_id  : Unread Alert category id. see @ref ANP_ALERT_CATEGORY_ID."Alert category id".
 *
 * @return          Status of the GATT notification
 */
wiced_bt_gatt_status_t wiced_bt_ans_process_and_send_unread_alert(uint16_t conn_id, wiced_bt_anp_alert_category_id_t category_id);

/**
 * Application call this API to clear the new alert and unread alert count.of the specified category
.* Library clears new alert count and unread alert count of given category.
 *
 * @param           conn_id : GATT connection ID
 * @param           category_id  : Unread Alert category id. see @ref ANP_ALERT_CATEGORY_ID."Alert category id".
 *
 * @return          WICED_TRUE on success
                        WICED_FALSE on invalid category id
 */
wiced_bool_t wiced_bt_ans_clear_alerts(uint16_t conn_id, wiced_bt_anp_alert_category_id_t category_id);

#ifdef __cplusplus
}
#endif


/**@} wiced_bt_ans_api_functions */

#endif
