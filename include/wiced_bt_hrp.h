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
 * This file provides common definitions used by the Heart Rate Server(HRS) and Heart Rate Client.(HRC)
 */

#ifndef WICED_BT_HRP_H
#define WICED_BT_HRP_H
#include "wiced_bt_types.h"

/**
 * @anchor HEART_RATE_SENSOR_LOCATION
 * @name Heart rate sendor location
 * @{ */
#define WICED_BT_HEART_RATE_SENSOR_LOCATION_OTHER       0
#define WICED_BT_HEART_RATE_SENSOR_LOCATION_CHEST       1
#define WICED_BT_HEART_RATE_SENSOR_LOCATION_WRIST       2
#define WICED_BT_HEART_RATE_SENSOR_LOCATION_FINGER      3
#define WICED_BT_HEART_RATE_SENSOR_LOCATION_HANDLE      4
#define WICED_BT_HEART_RATE_SENSOR_LOCATION_EARLOBE     5
#define WICED_BT_HEART_RATE_SENSOR_LOCATION_FOOT        6 
#define WICED_BT_HEART_RATE_SENSOR_LOCATION_MAX         WICED_BT_HEART_RATE_SENSOR_LOCATION_FOOT
typedef uint8_t wiced_bt_heart_rate_senor_location_t;
/** @} HEART_RATE_SENSOR_LOCATION */

/** Heart rate data.Data get pass from application to HRS library on Heart rate notification sending and 
    From HRC library to application.on receiving Heart rate notification.*/
typedef struct
{
    uint16_t    conn_id;                    /**< connection identifier */
    uint16_t    heart_rate;                 /**< Value in bpm, beats per minutes. */
    uint8_t     energy_expended_present;    /**< 1 = present, 0= not present */
    uint16_t    energy_expended;            /**< Value in kilo Jouls Range is 0x0 to 0xffff*/

#ifdef SUPPORT_RR_INTERVALS
    uint8_t     num_of_rr_intervals; /* Number of RR intervals present */
    uint16_t    *rr_intervals_data; /* pointer to RR intervals. Each RR interval is in terms of seconds */
#endif
} wiced_bt_heart_rate_data_t;

#define WICED_BT_HRP_CONTROL_POINT_WRITE_UNSUPPORTED_VALUE      0x80

#endif
