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
 * This file provides definitions of the Heart rate profile Server(HRS) library interface
 */

#ifndef WICED_BT_HRS_H
#define WICED_BT_HRS_H
#include "wiced_bt_hrp.h"
#include "wiced_bt_gatt.h"
#include "wiced_bt_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @addtogroup  wiced_bt_hrs_api_functions        HRS Library API
 * @ingroup     wicedbt
 *
 * HRS library of the WICED SDK provide a simple method for an application to integrate Heart Rate
 * server functionality. Application calls library APIs to process Heart Rate Client(HRC) 
 * GATT read, write requests to configure Heart Rate services. Application notified with 
 * simple events to start or stop Heart rate notification based on client configuration.
 * Formats Heart rate notification as per spec and sends to client when application sends 
 * Heart rate data data. see @ref wiced_bt_heart_rate_data_t
 * @{
 */

/* GATT handles used by ANP server application and library */
// ***** Primary Service 'Heart Rate'
#define HDLS_HEART_RATE                                         0x0028
// ----- Characteristic 'Heart Rate Measurement'
#define HDLC_HEART_RATE_MEASUREMENT                             0x0029
#define HDLC_HEART_RATE_MEASUREMENT_VALUE                       0x0030
#define HDLC_HEART_MEASUREMENT_CLIENT_CONFIG_DESCRIPTOR         0x0031

// ----- Characteristic 'Sensor location'
#define HDLC_HEART_RATE_SENSOR_LOCATION                         0x0032
#define HDLC_HEART_RATE_SENSOR_LOCATION_VALUE                   0x0033

// ----- Characteristic 'Heart Rate Control Point'
#define HDLC_HEART_RATE_CONTROL_POINT                           0x0034
#define HDLC_HEART_RATE_CONTROL_POINT_VALUE                     0x0035

/**
 * @anchor WICED_BT_HRS_EVENT
 * @name heart rate server library events
 * @{ */
#define     WICED_BT_HRS_EVENT_HEART_RATE_NOTIFICATIONS_ENABLED     1   /**< Client Registered for for Heart rate notifications*/
#define     WICED_BT_HRS_EVENT_HEART_RATE_NOTIFICATIONS_DISABLED    2   /**< Client Un-registered Heart rate notifications*/
#define     WICED_BT_HRS_RESET_ENERGY_EXPENDED_VALUE                3   /**< Reset Energy expended during heart rate measurement since last reset */
typedef uint8_t wiced_bt_hrs_event_type_t;
/** @} WICED_BT_HRS_EVENT */

/*****************************************************************************
 *          Function Prototypes
 *****************************************************************************/

/**
 * This callback is executed when HRS library completes HRC requested configuration (Client requests
 * to start/stop/reset energy expended in heart rate using GATT write operation)
 *
 * @param           conn_id : GATT connection ID
 * @param           event : Event to application to act accordingly
 * @return          none
 */
typedef void (*wiced_bt_hrs_event_cback_t)(uint16_t conn_id, wiced_bt_hrs_event_type_t event);

/**
 * The application should call this function on start up to initialize HRS library. Through callback application notified
 * with different events (see @wiced_bt_hrs_event_cback_t) based on heart rate client actions
 *
 * @param           p_app_event_cb  : Application callback pointer to receive events from library.See @ref wiced_bt_hrs_event_cback_t
 * @return          none
 */
void                    wiced_bt_hrs_initialize ( wiced_bt_hrs_event_cback_t p_app_event_cb );

/**
 * The application should call this function when BLE connection with a peer
 * device has been established.
 *
 * @param           conn_id  : GATT connection ID.
 * @return          none
 */
void                    wiced_bt_hrs_connection_up(uint16_t conn_id);

/**
 * The application should call this function when BLE connection with a peer
 * device has been disconnected.
 *
 * @param           conn_id  : GATT connection ID.
 * @return          none
 */
 void                    wiced_bt_hrs_connection_down(uint16_t conn_id);

/**
 * The application should call this function when it receives GATT read request on Heart rate 
 * service characteristc and descriptors.
 *
 * @param           conn_id  : GATT connection ID.
 * @param           p_read :  GATT read request.
 * @return          Status of GATT Read operation
 */
wiced_bt_gatt_status_t  wiced_bt_hrs_process_client_read_req(uint16_t conn_id, wiced_bt_gatt_read_t *p_read);

/**
 * The application should call this function when it receives GATT write request on Heart rate 
 * service characteristc and descriptors.
 *
 * @param           conn_id  : GATT connection ID.
 * @param           p_write : GATT write request.
 * @return          Status of GATT Write operation (Note: WICED_BT_HRP_CONTROL_POINT_WRITE_UNSUPPORTED_VALUE
                                value retuns when client provides invalid value in reset energy expended request.)
 */
wiced_bt_gatt_status_t  wiced_bt_hrs_process_client_write_req(uint16_t conn_id, wiced_bt_gatt_write_t *p_write);

/**
 * The application should call this function to send heart rate notification.
 *
 * @param           heart_rate_data  : Heart rate data.See @ref wiced_bt_heart_rate_data_t
 * @return          Status of GATT notification operation
 */
wiced_bt_gatt_status_t wiced_bt_hrs_send_heart_rate( wiced_bt_heart_rate_data_t *heart_rate_data);

/**
 * The application should call this API on successfull encryption with Heart Rate client.
 * Library is required to know whether the client enabled notifications.
 *
 * @param           conn_id  : GATT connection ID.
 * @param           notifications_enabled  : Notifications status. WICED_TRUE:Enabled, WICED_FALSE:disabled
 * @return          none
 */
void wiced_bt_hrs_set_previous_connection_client_notification_configuration( uint16_t conn_id, wiced_bool_t notifications_enabled );

#ifdef __cplusplus
}
#endif


/**@} wiced_bt_hrs_api_functions */

#endif
