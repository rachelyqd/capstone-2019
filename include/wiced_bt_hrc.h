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
 * This file provides definitions of the Heart rate profile Client(HRC) library interface
 */
#ifndef WICED_BT_HRC_H
#define WICED_BT_HRC_H
#include "wiced_bt_hrp.h"
#include "wiced_bt_gatt.h"
#include "wiced_bt_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @addtogroup  wiced_bt_hrc_api_functions        HRC Library API
 * @ingroup     wicedbt
 *
 * HRC library of the WICED SDK provides a simple method for an application to integrate Heart Rate
 * Client functionality. Application Calls the Library APIs to:
 *    Discover Heart Rate Service characteristics and descriptors whenever it finds Heart Rate Service in the connected device.
 *    Start or Stop Heart Rate Notifications.
 *    To Reset Energy expended during Heart Rate notification based on application request
 * Application registers callback on application start to receive the result of above 3 operations.
  * @{
 */

/*****************************************************************************
 *          Function Prototypes
 *****************************************************************************/

/**
 * This callback is executed when HRC library completes discovery of Heart Rate service
 * characteristics and descriptors and Device information Service Discovery
 *
 * @param           conn_id : GATT connection ID
 * @param           status  : WICED_TRUE if initialization completed successfully.
 * @return          none
 */
typedef void (*wiced_bt_hrc_discovery_complete_callback_t)(uint16_t conn_id, wiced_bool_t success);

/**
 * This callback is executed when HRC library completes startup operation of the HRC.
 *
 * @param           conn_id : GATT connection ID
 * @param           status  : Status of the operation.
 * @return          none
 */
typedef void (*wiced_bt_hrc_start_complete_callback_t)(uint16_t conn_id, wiced_bt_gatt_status_t success);

/**
 * This callback is executed when HRC library completes stop operation of the HRC.
 *
 * @param           conn_id : GATT connection ID
 * @param           status  : Status of the operation.
 * @return          none
 */
typedef void (*wiced_bt_hrc_stop_complete_callback_t)(uint16_t conn_id, wiced_bt_gatt_status_t success);

/**
 * This callback is executed when HRC library completes the request to reset Heart Rate Server energy expended value.
 *
 * @param           conn_id : GATT connection ID
 * @param           status  : Status of the operation.
 * @return          none
 */
typedef void (*wiced_bt_hrc_reset_energy_expended_callback_t)(uint16_t conn_id, wiced_bt_gatt_status_t success);

/**
 * WICED BT HRC library executes this callback when it receives complete notification
 * from the Heart Rate Server device with new Heart Rate Measurement.
 *
 * @param           conn_id                             : Connection ID.
 * @param           p_heart_rate_notification       : Received Heart Rate Measurement.
 * @return          none
 */
typedef void (*wiced_bt_hrc_notification_callback_t)( wiced_bt_heart_rate_data_t *p_heart_rate_notification );

/**
 * Following structure is used to register application with wiced_bt_ancs library
 */
typedef struct
{
    wiced_bt_hrc_discovery_complete_callback_t      p_discovery_complete_callback; /**< callback to be executed when HRC library completes GATT discovery */
    wiced_bt_hrc_start_complete_callback_t          p_start_complete_callback;     /**< callback to be executed when HRC library completes Heart Rate Server configuration */
    wiced_bt_hrc_stop_complete_callback_t           p_stop_complete_callback;      /**< callback to be executed when HRC library completes Heart Rate Server deconfiguration */
    wiced_bt_hrc_reset_energy_expended_callback_t   p_reset_energy_expended_callback; /**< callback to be executed when HRC library completes the request to Heart Rate Server to reset energy expended value */
    wiced_bt_hrc_notification_callback_t            p_notification_callback;       /**< callback to be executed when HRC library receives Heart Measurement from the Server */
} wiced_bt_hrc_reg_t;


/**
 * The application should call this function to register application callbacks
 *
 * @param           p_reg  : Registration control block that includes HRC application callbacks.
 * @return          none
 */
void wiced_bt_hrc_initialize ( wiced_bt_hrc_reg_t *p_reg );

/**
 * The application should call this function when it discovers that connected central device
 * contains the heart rate service.  The function starts the GATT discovery 
 * of heart rate service characteristics.
 *
 * @param           conn_id  : GATT connection ID.
 * @param           s_handle : Start GATT handle of the heart rate service.
 * @param           e_handle : End GATT handle of the heart rate service.
 * @return          WICED_TRUE if GATT discovery started successfully, WICED_FALSE otherwise.
 */
wiced_bool_t wiced_bt_hrc_discover(uint16_t conn_id, uint16_t s_handle, uint16_t e_handle);

/**
 * While application performs GATT discovery it shall pass discovery results for
 * the heart Rate service to the HRC Library. The library needs to find heart rate service characteristics
 * and associated characteristic client configuration descriptors.
 *
 * @param           p_data   : Discovery result data as passed from the stack.
 * @return          none
 */
void wiced_bt_hrc_discovery_result(wiced_bt_gatt_discovery_result_t *p_data);

/**
 * While application performs GATT discovery it shall pass discovery complete callbacks
 * for the heart rate service to the HRC Library. As the GATT discovery is perfformed in multiple steps
 * this function initiates the next discovery request.
 *
 * @param           p_data   : Discovery complete data as passed from the stack.
 * @return          none
 */
void wiced_bt_hrc_discovery_complete(wiced_bt_gatt_discovery_complete_t *p_data);

/**
 * The application should call this function when BLE connection with a peer
 * device has been established.
 *
 * @param           conn_id  : connection identifier.
 * @return          none
 */
void wiced_bt_hrc_connection_up(uint16_t conn_id);

/**
 * The application should call this function when BLE connection with a peer
 * device has been disconnected.
 *
 * @param           conn_id  : connection identifier..
 * @return          none
 */
void wiced_bt_hrc_connection_down(uint16_t conn_id);

/**
 * The application should call this function whenever it is needed to collect Heart Rate.
 * Discovery should be completed before this function is executed.
 *
 * The start function enables the server to send heart Rate notifications.
 *
 * @param           conn_id  : GATT connection ID.
 * @return          Result of GATT operation.
 */
wiced_bt_gatt_status_t  wiced_bt_hrc_start(uint16_t conn_id);

/**
 * The application calls this function whenever it need to stop to collect Heart Rate.
 *
 * The stop function disables the server to send heart Rate notifications
 
 * @param           conn_id  : GATT connection ID.
 * @return          Result of GATT operation.
 */
wiced_bt_gatt_status_t  wiced_bt_hrc_stop(uint16_t conn_id);


/**
 * The application calls this function to reset accumulated Energy expended value during
 * heart rate measure process. Client can reset when it finds max energy expended value (i.e oxffff)
 * in heart rate notification.
 
 * @param           conn_id  : GATT connection ID.
 * @return          Result of GATT operation.
 */
wiced_bt_gatt_status_t  wiced_bt_hrc_reset_energy_expended(uint16_t conn_id);


/**
 * The application calls this function when it receive GATT operation complete event on 
 * heart rate service characteristics. Library parse the result and calls application registered call
 * backs.
 
 * @param           conn_id  : GATT connection ID.
 * @return          none
 */
void wiced_bt_hrc_gatt_op_complete(wiced_bt_gatt_operation_complete_t *p_data);

#ifdef __cplusplus
}
#endif


/**@} wiced_bt_hrc_api_functions */

#endif

