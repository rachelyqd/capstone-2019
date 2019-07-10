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

/**< @file
 *
 * Definitions for interface between Bluetooth Mesh Models and Mesh Core
 *
 */

#ifndef __WICED_BT_MESH_MODELS_H__
#define __WICED_BT_MESH_MODELS_H__

#include "wiced_bt_mesh_event.h"
#include "wiced_bt_mesh_core.h"
#include "wiced_bt_mesh_model_defs.h"
extern uint64_t wiced_bt_mesh_core_get_tick_count(void);

/**
 * NVRAM sections used by different models.  Each model uses IDs from START_ID to (START_ID + number of elements on the device)
 * The values should be initialized by the application.
 */
extern uint16_t wiced_bt_mesh_scene_register_nvram_id;
extern uint16_t wiced_bt_mesh_scene_nvram_id_start;
extern uint16_t wiced_bt_mesh_scene_nvram_id_end;
extern uint16_t wiced_bt_mesh_scheduler_nvram_id_start;
extern uint16_t wiced_bt_mesh_default_trans_time_nvram_id_start;
extern uint16_t wiced_bt_mesh_power_level_nvram_id_start;
extern uint16_t wiced_bt_mesh_power_onoff_nvram_id_start;
extern uint16_t wiced_bt_mesh_light_lightness_nvram_id_start;
extern uint16_t wiced_bt_mesh_light_xyl_nvram_id_start;
extern uint16_t wiced_bt_mesh_light_ctl_nvram_id_start;
extern uint16_t wiced_bt_mesh_light_hsl_nvram_id_start;
extern uint16_t wiced_bt_mesh_light_lc_nvram_id_start;

extern uint8_t  wiced_bt_mesh_scene_max_num;
extern uint8_t  wiced_bt_mesh_sheduler_events_max_num;

 /**
 * @anchor WICED_BT_MESH_CORE_COMPANY_ID
 * @name Company identifiers
 * \details The following is the list of mesh company id
 * @{
 */
#define MESH_COMPANY_ID_BT_SIG                          0x0000
#define MESH_COMPANY_ID_CYPRESS                         0x0131
 /** @} WICED_BT_MESH_CORE_COMPANY_ID */

/**
 * @anchor BATTERY_EVENT
 * @name Definition for messages exchanged between an app and Generic Battery Model
 * @{ */
#define WICED_BT_MESH_BATTERY_GET                           0   ///< Get battery state 
#define WICED_BT_MESH_BATTERY_STATUS                        1   ///< Reply to get battery or battery changed notification
/** @} BATTERY_EVENT */

/**
 * @anchor LOCATION_EVENT
 * @name Definition for messages exchanged between an app and Generic Location Model
 * @{ */
#define WICED_BT_MESH_LOCATION_GLOBAL_GET                   2   ///< Get global location
#define WICED_BT_MESH_LOCATION_GLOBAL_SET                   3   ///< Set global location
#define WICED_BT_MESH_LOCATION_GLOBAL_STATUS                4   ///< Reply to get/set global location or global location changed notification
#define WICED_BT_MESH_LOCATION_LOCAL_GET                    5   ///< Get local location
#define WICED_BT_MESH_LOCATION_LOCAL_SET                    6   ///< Set local location   
#define WICED_BT_MESH_LOCATION_LOCAL_STATUS                 7   ///< Reply to get/set local location or local location changed notification
/** @} LOCATION_EVENT */

/**
 * @anchor ONOFF_EVENT
 * @name Definition for messages exchanged between an app and Generic OnOff Model
 * @{ */
#define WICED_BT_MESH_ONOFF_GET                             8   ///< Get on off state
#define WICED_BT_MESH_ONOFF_SET                             9   ///< Set on off state
#define WICED_BT_MESH_ONOFF_STATUS                          10  ///< Reply to get/set on off message or on off state changed notification
/** @} ONOFF_EVENT */

/**
 * @anchor LEVEL_EVENT
 * @name Definition for messages exchanged between an app and Generic Level Model
 * @{ */
#define WICED_BT_MESH_LEVEL_GET                             12  ///< Get level state
#define WICED_BT_MESH_LEVEL_SET                             13  ///< Set level state
#define WICED_BT_MESH_LEVEL_STATUS                          16  ///< Reply to  level message or level state changed notification
/** @} LEVEL_EVENT */

/**
 * @anchor DEFAULT_TRANSITION_TIME_EVENT
 * @name Definition for messages exchanged between an app and Generic OnOff Model
 * @{ */
#define WICED_BT_MESH_DEFAULT_TRANSITION_TIME_GET           18  ///< Get default transition time
#define WICED_BT_MESH_DEFAULT_TRANSITION_TIME_SET           19  ///< Set default transition time
#define WICED_BT_MESH_DEFAULT_TRANSITION_TIME_STATUS        20  ///< Reply to get/set default transition time message or on off status changed notification
/** @} DEFAULT_TRANSITION_TIME_EVENT */

/**
 * @anchor POWER_ONOFF_EVENT
 * @name Definition for messages exchanged between an app and Generic Power OnOff Model and Generic Power OnOff Setup Model
 * @{ */
#define WICED_BT_MESH_POWER_ONOFF_ONPOWERUP_STATUS           23  ///< Reply to get power on off message or power on off state changed notification
/** @} POWER_ONOFF_EVENT */

/**
 * @anchor POWER_LEVEL_EVENT
 * @name Definition for messages exchanged between an app and Generic Power Level Model and Generic Power Level Setup Models
 * @{ */
#define WICED_BT_MESH_POWER_LEVEL_GET                        24  ///< Get power level state
#define WICED_BT_MESH_POWER_LEVEL_SET                        25  ///< Set power level state
#define WICED_BT_MESH_POWER_LEVEL_STATUS                     28  ///< Reply to power level message or level state changed notification
#define WICED_BT_MESH_POWER_LEVEL_GET_DEFAULT                29  ///< Get power level default value for the element
#define WICED_BT_MESH_POWER_LEVEL_GET_RANGE                  30  ///< Get power level min/max values for the element
#define WICED_BT_MESH_POWER_LEVEL_SET_DEFAULT                31  ///< Set power level default value for the element
#define WICED_BT_MESH_POWER_LEVEL_SET_RANGE                  32  ///< Set power level min/max values for the element
#define WICED_BT_MESH_POWER_LEVEL_LAST_STATUS                33  ///< Reply to Get Last Power Level message
#define WICED_BT_MESH_POWER_LEVEL_DEFAULT_STATUS             34  ///< Reply to Get default Power :evel message or default changed notification
#define WICED_BT_MESH_POWER_LEVEL_RANGE_STATUS               35  ///< Reply to Get range power level message or range changed notification
/** @} POWER_LEVEL_EVENT */

/**
 * @anchor LIGHT_LIGHTNESS_EVENT
 * @name Definition for messages exchanged between an app and Light Lightness Model and Light Lightness Setup Models
 * @{ */
#define WICED_BT_MESH_LIGHT_LIGHTNESS_GET                   36  ///< Get Light Lightness Actual state
#define WICED_BT_MESH_LIGHT_LIGHTNESS_SET                   37  ///< Set Light Lightness Actual state
#define WICED_BT_MESH_LIGHT_LIGHTNESS_STATUS                40  ///< Reply to Light Lightness Get/Set message or Light Lightness Actual changed notification
#define WICED_BT_MESH_LIGHT_LIGHTNESS_LINEAR_GET            41  ///< Get Light Lightness Linear state
#define WICED_BT_MESH_LIGHT_LIGHTNESS_LINEAR_SET            42  ///< Set Light Lightness Linear state
#define WICED_BT_MESH_LIGHT_LIGHTNESS_LINEAR_STATUS         43  ///< Reply to Light Lightness Get/Set message or Light Lightness Status changed notification
#define WICED_BT_MESH_LIGHT_LIGHTNESS_SET_RANGE             44  ///< Set Light Lightness Actual min/max values for the element
#define WICED_BT_MESH_LIGHT_LIGHTNESS_LAST_STATUS           45  ///< Reply to Get last Light Lightness Actual message
#define WICED_BT_MESH_LIGHT_LIGHTNESS_DEFAULT_STATUS        46  ///< Reply to Get Light Lightness Actual default message or default level state changed notification
#define WICED_BT_MESH_LIGHT_LIGHTNESS_RANGE_STATUS          47  ///< Reply to Get Light Lightness Actual range message or range changed notification
/** @} LIGHT_LIGHTNESS_EVENT */

/**
 * @anchor LIGHT_CTL_EVENT
 * @name Definition for messages exchanged between an app and Light CTL Model and Light CTL Setup Models
 * @{ */
#define WICED_BT_MESH_LIGHT_CTL_GET                         50  ///< Get Light CTL state
#define WICED_BT_MESH_LIGHT_CTL_SET                         51  ///< Set Light CTL state
#define WICED_BT_MESH_LIGHT_CTL_STATUS                      54  ///< Reply to Light CTL Get/Set message or Light CTL changed notification
#define WICED_BT_MESH_LIGHT_CTL_TEMPERATURE_STATUS          59  ///< Reply to Get last Light CTL message
#define WICED_BT_MESH_LIGHT_CTL_SET_TEMPERATURE_RANGE       60  ///< Set Light CTL min/max temperature range values for the element
#define WICED_BT_MESH_LIGHT_CTL_DEFAULT_STATUS              61  ///< Reply to Get Light CTL default message or default level state changed notification
#define WICED_BT_MESH_LIGHT_CTL_SET_DEFAULT                 62  ///< Set Light CTL default temperature value for the element
#define WICED_BT_MESH_LIGHT_CTL_TEMPERATURE_RANGE_STATUS    63  ///< Reply to Get Light CTL range message or range changed notification
/** @} LIGHT_CTL_EVENT */

/**
 * @anchor LIGHT_HSL_EVENT
 * @name Definition for messages exchanged between an app and Light CTL Model and Light CTL Setup Models
 * @{ */
#define WICED_BT_MESH_LIGHT_HSL_GET                         64  ///< Get Light HSL state
#define WICED_BT_MESH_LIGHT_HSL_SET                         65  ///< Set Light HSL state
#define WICED_BT_MESH_LIGHT_HSL_STATUS                      66  ///< Reply to Light HSL Get/Set message or Light HSL changed notification
#define WICED_BT_MESH_LIGHT_HSL_TARGET_GET                  67  ///< Get Light HSL Target state
#define WICED_BT_MESH_LIGHT_HSL_TARGET_STATUS               68  ///< Light HSL Target status
#define WICED_BT_MESH_LIGHT_HSL_DEFAULT_STATUS              69  ///< Reply to Get Light HSL default message or default level state changed notification
#define WICED_BT_MESH_LIGHT_HSL_RANGE_SET                   70  ///< Set Light HSL min/max hue/saturation range values for the element
#define WICED_BT_MESH_LIGHT_HSL_RANGE_STATUS                71  ///< Reply to Get Light HSL range message or range changed notification
#define WICED_BT_MESH_LIGHT_HSL_HUE_SET                     72  ///< Set Light HSL Temperature state
#define WICED_BT_MESH_LIGHT_HSL_HUE_STATUS                  73  ///< Reply to Get/Set Light HSL Hue message
#define WICED_BT_MESH_LIGHT_HSL_SATURATION_SET              74  ///< Set Light HSL Saturation state
#define WICED_BT_MESH_LIGHT_HSL_SATURATION_STATUS           75  ///< Light HSL Saturation changed or reply message
/** @} LIGHT_HSL_EVENT */

/**
* @anchor LIGHT_XYL_EVENT
* @name Definition for messages exchanged between an app and Light CTL Model and Light CTL Setup Models
* @{ */
#define WICED_BT_MESH_LIGHT_XYL_GET                         76  ///< Get Light xyL state
#define WICED_BT_MESH_LIGHT_XYL_SET                         77  ///< Set Light xyL state
#define WICED_BT_MESH_LIGHT_XYL_STATUS                      78  ///< Reply to Light xyL Get/Set message or Light xyL changed notification
#define WICED_BT_MESH_LIGHT_XYL_TARGET_GET                  79  ///< Get Light xyL Target state
#define WICED_BT_MESH_LIGHT_XYL_TARGET_STATUS               80  ///< Light xyL Target values
#define WICED_BT_MESH_LIGHT_XYL_DEFAULT_STATUS              81  ///< Reply to Get Light xyL default message or default level state changed notification
#define WICED_BT_MESH_LIGHT_XYL_RANGE_SET                   82  ///< Set Light xyL min/max x/y range values for the element
#define WICED_BT_MESH_LIGHT_XYL_RANGE_STATUS                83  ///< Reply to Get Light xyL range message or range changed notification
#define WICED_BT_MESH_LIGHT_XYL_X_SET                       84  ///< Set Light xyL Temperature state
#define WICED_BT_MESH_LIGHT_XYL_X_STATUS                    85  ///< Reply to Get last Light xyL message
#define WICED_BT_MESH_LIGHT_XYL_Y_SET                       86  ///< Set Light xyL Temperature state
#define WICED_BT_MESH_LIGHT_XYL_Y_STATUS                    87  ///< Reply to Get last Light xyL message
/** @} LIGHT_XYL_EVENT */

/**
 * @anchor LIGHT_LC_EVENT
 * @name Definition for messages exchanged between an app and Light CTL Model and Light CTL Setup Models
 * @{ */
#define WICED_BT_MESH_LIGHT_LC_MODE_GET                     88  ///< Get Light LC Mode Get state
#define WICED_BT_MESH_LIGHT_LC_MODE_SET                     89  ///< Set Light LC Mode Set state
#define WICED_BT_MESH_LIGHT_LC_MODE_STATUS                  90  ///< Reply to Light LC Mode Get/Set
#define WICED_BT_MESH_LIGHT_LC_OCCUPANCY_MODE_GET           91  ///< Get Light LC Occupancy Mode Get state
#define WICED_BT_MESH_LIGHT_LC_OCCUPANCY_MODE_SET           92  ///< Set Light LC Occupancy Mode Set state
#define WICED_BT_MESH_LIGHT_LC_OCCUPANCY_MODE_STATUS        93  ///< Reply to Light LC Occupancy Mode Get/Set
#define WICED_BT_MESH_LIGHT_LC_LIGHT_ONOFF_GET              94  ///< Get Light LC Light OnOff Get state
#define WICED_BT_MESH_LIGHT_LC_LIGHT_ONOFF_SET              95  ///< Set Light LC Light OnOff Set state
#define WICED_BT_MESH_LIGHT_LC_LIGHT_ONOFF_STATUS           96  ///< Reply to Light LC Light OnOff Get/Set
#define WICED_BT_MESH_LIGHT_LC_PROPERTY_GET                 97  ///< Get Light LC Property Get state
#define WICED_BT_MESH_LIGHT_LC_PROPERTY_SET                 98  ///< Set Light LC Property Set state
#define WICED_BT_MESH_LIGHT_LC_PROPERTY_STATUS              99  ///< Reply to Light LC Property Get/Set
 /** @} LIGHT_LC_EVENT */

/**
 * @anchor PROPERTY_EVENT
 * @name Definition for messages exchanged between an app and Generic User Property Model
 * @{ */
#define WICED_BT_MESH_USER_PROPERTY_GET                     100 ///< Get Property state
#define WICED_BT_MESH_USER_PROPERTY_SET                     101 ///< Set Property state
#define WICED_BT_MESH_ADMIN_PROPERTIES_STATUS               102 ///< A list of the Admin Properties reported by the peer
#define WICED_BT_MESH_ADMIN_PROPERTY_STATUS                 103 ///< Reply to get/set Property message or Property state changed notification
#define WICED_BT_MESH_MANUF_PROPERTIES_STATUS               104 ///< A list of the Manufacturer Properties reported by the peer
#define WICED_BT_MESH_MANUF_PROPERTY_STATUS                 105 ///< Reply to get/set Property message or Property state changed notification
#define WICED_BT_MESH_USER_PROPERTIES_STATUS                106 ///< A list of the User Properties reported by the peer
#define WICED_BT_MESH_USER_PROPERTY_STATUS                  107 ///< Reply to get/set Property message or Property state changed notification
#define WICED_BT_MESH_CLIENT_PROPERTIES_STATUS              108 ///< A list of the Client Properties reported by the peer
/** @} PROPERTY_EVENT */

/**
 * @anchor SENSOR_EVENT
 * @name Definition for messages exchanged between an app and Sensor Model
 * @{ */
#define WICED_BT_MESH_SENSOR_DESCRIPTOR_GET                  110 ///< Get Sensor Descriptor state
#define WICED_BT_MESH_SENSOR_DESCRIPTOR_STATUS               111 ///< A list of the Descriptors reported by the peer
#define WICED_BT_MESH_SENSOR_GET                             112 ///< Get Sensor values from the peer
#define WICED_BT_MESH_SENSOR_STATUS                          113 ///< A list of the sensor values reported by the peer
#define WICED_BT_MESH_SENSOR_COLUMN_GET                      114 ///< Get Sensor Column values from the peer
#define WICED_BT_MESH_SENSOR_COLUMN_STATUS                   115 ///< A list of the sensor Column values from the peer
#define WICED_BT_MESH_SENSOR_SERIES_GET                      116 ///< Get sensor Series values from the peer
#define WICED_BT_MESH_SENSOR_SERIES_STATUS                   117 ///< A list of the Series values reported by the peer

// Sensor Setup
#define WICED_BT_MESH_SENSOR_CADENCE_GET                     120 ///< Get Sensor Cadence state
#define WICED_BT_MESH_SENSOR_CADENCE_SET                     121 ///< Set Sensor Cadence state
#define WICED_BT_MESH_SENSOR_CADENCE_SET_UNACKED             122 ///< Set Sensor Cadence state UnAcknowledged
#define WICED_BT_MESH_SENSOR_CADENCE_STATUS                  123 ///< Sensor Cadence state values reported by peer
#define WICED_BT_MESH_SENSOR_SETTINGS_GET                    124 ///< Get Sensor Settings list from the peer
#define WICED_BT_MESH_SENSOR_SETTINGS_STATUS                 125 ///< A list of the sensor setting values reported by the peer 
#define WICED_BT_MESH_SENSOR_SETTING_GET                     126 ///< Get Sensor Setting state
#define WICED_BT_MESH_SENSOR_SETTING_SET                     127 ///< Set Sensor Setting state
#define WICED_BT_MESH_SENSOR_SETTING_SET_UNACKED             128 ///< Set Sensor Setting state UnAcknowledged
#define WICED_BT_MESH_SENSOR_SETTING_STATUS                  129 ///< Sensor Setting state values reported by peer
/** @} SENSOR_EVENT */

/**
 * @anchor SCENE_EVENT
 * @name Definition for messages exchanged between an app and Scene Model
 * @{ */
#define WICED_BT_MESH_SCENE_GET                              130 ///< Get the scene
#define WICED_BT_MESH_SCENE_RECALL                           131 ///< Recall the scene
#define WICED_BT_MESH_SCENE_STORE                            132 ///< Store the scene
#define WICED_BT_MESH_SCENE_DELETE                           133 ///< Delete the scene
#define WICED_BT_MESH_SCENE_STATUS                           134 ///< Scene Status reply
#define WICED_BT_MESH_SCENE_REGISTER_STATUS                  135 ///< Scene Register Status reply
/** @} SCENE_EVENT */

/**
 * @anchor SCHEDULER_EVENT
 * @name Definition for messages exchanged between an app and Scheduler Model
 * @{ */
#define WICED_BT_MESH_SCHEDULER_GET                          135 ///< Get the scheduler register
#define WICED_BT_MESH_SCHEDULER_STATUS                       136 ///< Scheduler register status reported by the peer
#define WICED_BT_MESH_SCHEDULER_ACTION_GET                   137 ///< Get the scheduler entry
#define WICED_BT_MESH_SCHEDULER_ACTION_SET                   138 ///< Set the scheduler entry
#define WICED_BT_MESH_SCHEDULER_ACTION_STATUS                139 ///< Scheduler entry status reply
/** @} SCHEDULER_EVENT */

/*
 * @anchor TIME_EVENT
 * @name Definition for messages exchanged between an app and Time Model
 * @{ */
#define WICED_BT_MESH_TIME_GET                               140 ///< Get Time state
#define WICED_BT_MESH_TIME_SET                               141 ///< Set Time state
#define WICED_BT_MESH_TIME_STATUS                            142 ///< Get Time status
#define WICED_BT_MESH_TIME_ROLE_GET                          143 ///< Get Time role state
#define WICED_BT_MESH_TIME_ROLE_SET                          144 ///< Set Time role state
#define WICED_BT_MESH_TIME_ROLE_STATUS                       145 ///< Time role state value reported by time setup model
#define WICED_BT_MESH_TIME_ZONE_GET                          146 ///< Get Time zone state
#define WICED_BT_MESH_TIME_ZONE_SET                          147 ///< Set Time zone state
#define WICED_BT_MESH_TIME_ZONE_STATUS                       148 ///< Time zone status values reported by time model
#define WICED_BT_MESH_TAI_UTC_DELTA_GET                      149 ///< Get Time delta state
#define WICED_BT_MESH_TAI_UTC_DELTA_SET                      150 ///< Set Time delta state
#define WICED_BT_MESH_TAI_UTC_DELTA_STATUS                   151 ///< Time Delta state values reported by time model
#define WICED_BT_MESH_TIME_CHANGED                           152 ///< Time changed event state
/** @} TIME_EVENT */

#define WICED_BT_MESH_TX_COMPLETE                           255  ///< Transmission completed, or timeout waiting for peer ack.

// TO DO : Move this defines into sensor structures
#define WICED_BT_MESH_MAX_PROP_VAL_LEN         12
#define WICED_BT_MESH_SENSOR_MAX_LIST_LEN      5
#define WICED_BT_MESH_SENSOR_MAX_SERIES_COLUMN 5
#define WICED_BT_MESH_SENSOR_MAX_SETTINGS      5
#define WICED_BT_MESH_MAX_SENSOR_PAYLOAD_LEN   380

/**
 * Sensor property id structure exchanged between the application and the Sensor Model
 */
typedef struct
{
    uint16_t property_id;                    /**<  Property ID identifying a sensor */
} wiced_bt_mesh_sensor_get_t;


/**
 * Sensor Descriptor structure exchanged between the application and the Sensor Model
 */
typedef struct
{
    uint16_t property_id;                    /**< property that describes the meaning and format of data reported by sensor*/
    uint16_t positive_tolerance; //12 bits   /**< 12-bit value representing the magnitude of a possible positive tolerance with sensor measurement*/
    uint16_t negative_tolerance; //12 bits   /**< 12-bit value representing the magnitude of a possible negative tolerance with sensor measurement*/
    uint8_t  sampling_function;              /**< averaging operation or type of sampling function applied to the measured value*/
    uint8_t  measurement_period;             /**< uint8 value n that represents the averaging time span, accumulation time, or measurement period in seconds*/
    uint8_t  update_interval;                /**< measurement reported by a sensor is internally refreshed at the frequency indicated in the Sensor Update Interval field*/
} wiced_bt_mesh_sensor_descriptor_data_t;

/**
 * Sensor Descriptor status structure exchanged between the application and the Sensor Model
 */
typedef struct
{
    uint8_t                                 num_descriptors;                                      /**< Total number of descriptors present in status structure*/
    wiced_bt_mesh_sensor_descriptor_data_t  descriptor_list[WICED_BT_MESH_SENSOR_MAX_LIST_LEN];   /**< Descriptor data list */
} wiced_bt_mesh_sensor_descriptor_status_data_t;


/**
 * Sensor data structure exchanged between the application and the Sensor Model
 */
typedef struct
{
    uint16_t property_id;                                   /**< Property identifying a sensor */
    uint8_t  prop_value_len;                                /**< Length of the raw_value corresponding to property_id*/
    uint8_t  raw_value[WICED_BT_MESH_MAX_PROP_VAL_LEN];     /**< Value of the Property */
} wiced_bt_mesh_sensor_data_t;

/**
 * Sensor status data structure exchanged between the application and the Sensor Model
 */
typedef struct
{
   uint8_t                          num_status;                                           /**< Total number of status received */
   wiced_bt_mesh_sensor_data_t      status_list[WICED_BT_MESH_SENSOR_MAX_LIST_LEN];       /**< Status data list */
} wiced_bt_mesh_sensor_status_data_t;

/**
 * Sensor Column get structure exchanged between the application and the Sensor Model
 */
typedef struct
{
    uint16_t property_id;                                   /**< Property identifying a sensor*/
    uint8_t  prop_value_len;                                /**< Length of the raw_value corresponding to property_id*/
    uint8_t  raw_valuex[WICED_BT_MESH_MAX_PROP_VAL_LEN];    /**< Raw value identifying a column*/
} wiced_bt_mesh_sensor_column_get_data_t;

/**
 * Sensor column data structure exchanged between the application and the Sensor Model
 */
typedef struct
{
    uint8_t         raw_valuex[WICED_BT_MESH_MAX_PROP_VAL_LEN];    /**< Raw value representing the left corner of the column on the X axis*/
    uint8_t         column_width[WICED_BT_MESH_MAX_PROP_VAL_LEN];  /**< Raw value representing the width of the column*/
    uint8_t         raw_valuey[WICED_BT_MESH_MAX_PROP_VAL_LEN];    /**< Raw value representing the height of the column on the Y axis*/
} wiced_bt_mesh_sensor_column_data_t;
/**
 * Sensor Column status structure exchanged between the application and the Sensor Model
 */
typedef struct
{
    uint16_t                            property_id;        /**< Property identifying a sensor and the Y axis*/
    uint8_t                             prop_value_len;     /**< Length of the raw_value corresponding to property_id*/
    wiced_bool_t                        is_column_present;  /**< boolean value indicating if the column width is present in the message*/
    wiced_bt_mesh_sensor_column_data_t  column_data;        /**< Data field of column status */
} wiced_bt_mesh_sensor_column_status_data_t;


/**
 * Sensor Series get structure exchanged between the application and the Sensor Model
 */
typedef struct
{
    uint16_t property_id;                                   /**< Property identifying a sensor*/
    uint8_t  prop_value_len;                                /**< Length of the raw_value corresponding to property_id*/
 
    uint8_t  start_index;                                   /**< Start index of the series to be copied */
    uint8_t  end_index;                                     /**< End index of the series to be copied */
    uint8_t  raw_valuex1[WICED_BT_MESH_MAX_PROP_VAL_LEN];   /**< Raw value identifying a starting column*/
    uint8_t  raw_valuex2[WICED_BT_MESH_MAX_PROP_VAL_LEN];   /**< Raw value identifying an ending column*/
} wiced_bt_mesh_sensor_series_get_data_t;

/**
 * Sensor series status data structure exchanged between the application and the Sensor Model
 */
typedef struct
{
    uint16_t                             property_id;                                           /**< Property identifying a sensor*/
    uint8_t                              prop_value_len;                                        /**< Length of the raw_value corresponding to property_id*/
    uint8_t                              no_of_columns;                                         /**< Total number of columns*/
    wiced_bt_mesh_sensor_column_data_t   column_list[WICED_BT_MESH_SENSOR_MAX_SERIES_COLUMN];   /**< Series of column states*/
} wiced_bt_mesh_sensor_series_status_data_t;

/**
 * Sensor setting exchanged between the application and the Sensor Model
 */
typedef struct
{
#define WICED_BT_MESH_SENSOR_SETTING_READABLE                   0x01
#define WICED_BT_MESH_SENSOR_SETTING_READABLE_AND_WRITABLE      0x03
    uint16_t setting_property_id;                 /**<  Setting ID identifying a setting within a sensor */
    uint8_t  access;                              /**<  Read / Write access rights for the setting */
    uint8_t  value_len;                           /**< Length of the raw_value corresponding to property_id*/
    uint8_t  val[WICED_BT_MESH_MAX_PROP_VAL_LEN]; /**<  Raw value for the setting */
} wiced_bt_mesh_sensor_setting_t;

/**
 * Sensor Setting status data structure exchanged between the application and the Sensor Model
 */
typedef struct
{
    uint16_t                       property_id;      /**< Property ID identifying a sensor */
    wiced_bt_mesh_sensor_setting_t setting;          /**< Sensor setting data */
} wiced_bt_mesh_sensor_setting_status_data_t;

/**
 * Sensor Setting get structure exchanged between the application and the Sensor Model
 */
typedef struct
{
    uint16_t property_id;                       /**<  Property ID identifying a sensor */
    uint16_t setting_property_id;               /**<  Setting ID identifying a setting within a sensor */
} wiced_bt_mesh_sensor_setting_get_data_t;

/**
 * Sensor Setting set structure exchanged between the application and the Sensor Model
 */
typedef struct
{
    uint16_t property_id;                                      /**<  Property ID identifying a sensor */
    uint16_t setting_property_id;                              /**<  Setting ID identifying a setting within a sensor */
    uint8_t  prop_value_len;                                   /**< Length of the raw_value corresponding to property_id*/
    uint8_t  setting_raw_val[WICED_BT_MESH_MAX_PROP_VAL_LEN];  /**<  Raw value for the setting */
} wiced_bt_mesh_sensor_setting_set_data_t;


/**
 * Sensor Settings status structure exchanged between the application and the Sensor Model
 */
typedef struct
{
    uint16_t property_id;                                                 /**<  Property ID identifying a sensor */
    uint16_t num_setting_property_id;                                     /**<  Total Setting IDs identifying a setting within a sensor */
    uint16_t  setting_property_id_list[WICED_BT_MESH_SENSOR_MAX_SETTINGS]; /**<  List of setting ID identifying a setting within a sensor */
} wiced_bt_mesh_sensor_settings_status_data_t;

/**
 * Sensor Cadence structure exchanged between the application and the Sensor Model
 */
typedef struct
{
    uint8_t       fast_cadence_period_divisor;                               /**< Divisor for the Publish Period */
    wiced_bool_t  trigger_type;                                              /**< Defines the unit and format of the Status Trigger Delta fields*/
    uint8_t       trigger_delta_down[WICED_BT_MESH_MAX_PROP_VAL_LEN];        /**< Delta down value that triggers a status message */
    uint8_t       trigger_delta_up[WICED_BT_MESH_MAX_PROP_VAL_LEN];          /**< Delta up value that triggers a status message */
    uint8_t       min_interval;                                              /**< Minimum interval between two consecutive Status messages */
    uint8_t       fast_cadence_low[WICED_BT_MESH_MAX_PROP_VAL_LEN];          /**< Low value for the fast cadence range */
    uint8_t       fast_cadence_high[WICED_BT_MESH_MAX_PROP_VAL_LEN];         /**< High value for the fast cadence range */
} wiced_bt_mesh_sensor_cadence_t;


/**
 * Sensor Cadence set structure exchanged between the application and the Sensor Model
 */
typedef struct
{
    uint16_t                             property_id;    /**<  Property ID identifying a sensor */
    uint8_t                              prop_value_len; /**< Length of the raw_value corresponding to property_id*/
    wiced_bt_mesh_sensor_cadence_t       cadence_data;   /**<  Cadence data field of set message */
} wiced_bt_mesh_sensor_cadence_set_data_t;

/**
 * Sensor Cadence status structure exchanged between the application and the Sensor Model
 */
typedef struct
{
    uint16_t                             property_id;    /**<  Property ID identifying a sensor */
    wiced_bool_t                         is_data_present;/**<  Cadence data is received from peer */
    uint8_t                              prop_value_len; /**< Length of the raw_value corresponding to property_id*/
    wiced_bt_mesh_sensor_cadence_t       cadence_data;   /**<  Optional cadence data field of status message */
} wiced_bt_mesh_sensor_cadence_status_data_t;


/**
 * Sensor status data structure exchanged between the sensor model and Application
 */
typedef union
{
    wiced_bt_mesh_sensor_descriptor_status_data_t      desc_status;      /**< Descriptor Status data received from Server*/
    wiced_bt_mesh_sensor_status_data_t                 sensor_status;    /**< Sensor Status data received from Server*/
    wiced_bt_mesh_sensor_column_status_data_t          column_status;    /**< Column Status data received from Server*/
    wiced_bt_mesh_sensor_series_status_data_t          series_status;    /**< Series Status data received from Server*/
    wiced_bt_mesh_sensor_setting_status_data_t         setting_status;   /**< Setting Status data received from Server*/
    wiced_bt_mesh_sensor_settings_status_data_t        settings_status;  /**< Settings Status data received from Server*/
    wiced_bt_mesh_sensor_cadence_status_data_t         cadence_status;   /**< Cadence Status data received from Server*/
} wiced_bt_mesh_sensor_status_message_data_t;

/**
 * Scene Data structure exchanged between the application and the Scene Models
 */
typedef struct
{
#define WICED_BT_MESH_SCENE_REQUEST_TYPE_STORE  0
#define WICED_BT_MESH_SCENE_REQUEST_TYPE_DELETE 1
#define WICED_BT_MESH_SCENE_REQUEST_TYPE_GET    2
    uint8_t  type;                                  /**< Scene Request Type */
    uint16_t scene_number;                          /**< The number of the scene to be recalled */
} wiced_bt_mesh_scene_request_t;

/**
 * Scene Recall Data structure exchanged between the application and the Scene Models
 */
typedef struct
{
    uint16_t scene_number;                          /**< The number of the scene to be recalled */
    uint32_t transition_time;                       /**< Transition time to the target level */
    uint16_t delay;                                 /**< Delay before starting the transition */
} wiced_bt_mesh_scene_recall_t;

/**
 * Scene Status Data structure exchanged between the application Scene Models
 */
typedef struct
{
    uint8_t  status_code;
    uint16_t current_scene;                         /**< Scene Number of a current scene */
    uint16_t target_scene;                          /**< Scene Number of a target scene */
    uint32_t remaining_time;                        /**< Remaining time for transaction */
} wiced_bt_mesh_scene_status_data_t;

#define WICED_BT_MESH_MODELS_MAX_SCENES                 100

/**
 * Scene Register Status Data structure exchanged between the application Scene Models
 */
typedef struct
{
    uint8_t  status_code;                           /**< Scene operation status code (see @ref WICED_BT_MESH_SCENE_STATUS_CODE */
    uint16_t current_scene;                         /**< Scene Number of a current scene */
    uint8_t  scene_num;                             /**< Number of scenese reported */
    uint32_t scene[WICED_BT_MESH_MODELS_MAX_SCENES];/**< Remaining time for transaction */
} wiced_bt_mesh_scene_register_status_data_t;

/**
 * Time State structure exchanged between the Time Client and Time Server
 */
typedef struct
{
    uint64_t tai_seconds;                           /**< Current TAI time in seconds since the epoch */
    uint8_t  subsecond;                             /**< The sub-second time in units of 1/256s */
    uint8_t  uncertainty;                           /**< Estimated uncertainty in 10-millisecond steps */
    uint8_t  time_authority;                        /**< Element has a reliable source of TAI or not */
    uint16_t tai_utc_delta_current;                 /**< Current difference between TAI and UTC in seconds */
    uint8_t  time_zone_offset_current;              /**< Current zone offset in 15-minute increments */
} wiced_bt_mesh_time_state_msg_t;

/**
 * Time Zone State structure exchanged between the Application and Time model
 */
typedef struct
{
    uint8_t  time_zone_offset_new;                  /**< Upcoming local time zone offset */
    uint64_t tai_of_zone_change;                    /**< Absolute TAI time when the Time Zone Offset will change from Current to New */
} wiced_bt_mesh_time_zone_set_t;

/**
 * Time Zone Status structure exchanged between the Application and Time model
 */
typedef struct
{
    uint8_t  time_zone_offset_current;              /**< Current local time zone offset */
    uint8_t  time_zone_offset_new;                  /**< Upcoming local time zone offset */
    uint64_t tai_of_zone_change;                    /**< Absolute TAI time when the Time Zone Offset will change from Current to New */
} wiced_bt_mesh_time_zone_status_t;

/**
 * Time TAI UTC delta set structure exchanged between the Application and Time model
 */
typedef struct
{
    uint16_t tai_utc_delta_new;                     /**< Upcoming difference between TAI and UTC in seconds */
    uint64_t tai_of_delta_change;                   /**< Absolute TAI time when the TAI-UTC Delta will change from Current to New */
} wiced_bt_mesh_time_tai_utc_delta_set_t;

/**
 * Time TAI UTC delta status structure exchanged between the Application and Time model
 */
typedef struct
{
    uint16_t tai_utc_delta_current;                 /**< Current difference between TAI and UTC in seconds */
    uint16_t tai_utc_delta_new;                     /**< Upcoming difference between TAI and UTC in seconds */
    uint64_t tai_of_delta_change;                   /**< Absolute TAI time when the TAI-UTC Delta will change from Current to New */
} wiced_bt_mesh_time_tai_utc_delta_status_t;

/**
 * Time role structure exchanged between the Application and Time model
 */
typedef struct
{
   uint8_t role;                                    /**< The Time Role for the element */
} wiced_bt_mesh_time_role_msg_t;

/**
 * Scheduler Status Data structure exchanged between the application Scheduler Models
 */
typedef struct
{
    uint16_t actions;                               /**< Bit field indicating defined Actions in the Schedule Register */
} wiced_bt_mesh_scheduler_status_t;

/**
 * Scheduler Status Data structure exchanged between the application Scheduler Models
 */
typedef struct
{
    uint16_t action_number;                         /**< Bit field indicating defined Actions in the Schedule Register */
} wiced_bt_mesh_scheduler_action_get_t;

/**
 * @anchor SCHEDULER_ACTIONS
 * @name Scheduler Actions
 * @{ */
#define WICED_BT_MESH_SCHEDULER_ACTION_TURN_OFF     0       /**< Scheduler action Turn Off */
#define WICED_BT_MESH_SCHEDULER_ACTION_TURN_ON      1       /**< Scheduler action Turn On */
#define WICED_BT_MESH_SCHEDULER_ACTION_SCENE_RECALL 2       /**< Scheduler action Scene Recall */
#define WICED_BT_MESH_SCHEDULER_ACTION_NONE         0x0f    /**< Scheduler action None */
 /* @} */

#define WICED_BT_MESH_SCHEDULER_EVERY_YEAR          0x00    /**< Scheduled event should happen every year */
#define WICED_BT_MESH_SCHEDULER_EVERY_DAY           0x00    /**< Scheduled event should happen every day */

 /**
 * @anchor SCHEDULER_HOUR
 * @name Scheduler Hours
 * @{ */
#define WICED_BT_MESH_SCHEDULER_EVERY_HOUR          0x18    /**< Scheduled event should happen every hour */
#define WICED_BT_MESH_SCHEDULER_RANDOM_HOUR         0x19    /**< Scheduled event should happen on a random hour */
 /* @} */

/**
 * @anchor SCHEDULER_MINUTE
 * @name Scheduler Minutes
 * @{ */
#define WICED_BT_MESH_SCHEDULER_EVERY_MINUTE        0x3C    /**< Scheduled event should happen every minute */
#define WICED_BT_MESH_SCHEDULER_EVERY_15_MINUTES    0x3D    /**< Scheduled event should happen every 15 minutes */
#define WICED_BT_MESH_SCHEDULER_EVERY_20_MINUTES    0x3E    /**< Scheduled event should happen every 20 minutes */
#define WICED_BT_MESH_SCHEDULER_RANDOM_MINUTE       0x3F    /**< Scheduled event should happen on a random minute */
/* @} */

/**
 * @anchor SCHEDULER_SECONDS
 * @name Scheduler Seconds
 * @{ */
#define WICED_BT_MESH_SCHEDULER_EVERY_SECOND        0x3C    /**< Scheduled event should happen every second */
#define WICED_BT_MESH_SCHEDULER_EVERY_15_SECONDS    0x3D    /**< Scheduled event should happen every 15 seconds */
#define WICED_BT_MESH_SCHEDULER_EVERY_20_SECONDS    0x3E    /**< Scheduled event should happen every 20 seconds */
#define WICED_BT_MESH_SCHEDULER_RANDOM_SECOND       0x3F    /**< Scheduled event should happen on a random second */
/* @} */

/**
 * Scheduler Actions Data structure exchanged between the application Scheduler Models
 */
typedef struct
{
    uint8_t action_number;                                  /**< zero based entry number */
    uint8_t year;                                           /**< scheduled year for the action, or 0 if action needs to happen every year */
    uint16_t month;                                         /**< Bit field of the months for the action */
    uint8_t  day;                                           /**< Scheduled day of the month, or 0 to repeat every day */
    uint8_t  hour;                                          /**< Scheduled hour for the action, values 0-23 indicate the hour, or use one of the (@ref SCHEDULER_HOUR "special values") */  
    uint8_t  minute;                                        /**< Scheduled hour for the action, values 0-59 indicate the minute, or use one of the (@ref SCHEDULER_MINUTE "special values")  */
    uint8_t  second;                                        /**< Scheduled second for the action, values 0-59 indicate the second, or use one of the (@ref SCHEDULER_SECONDS "special values")  */
    uint8_t  day_of_week;                                   /**< Bit field of the days of week when the action should happen */
    uint8_t  action;                                        /**< Action to be executed (see @ref SCHEDULER_ACTIONS "possible scheduler actions") */
    uint32_t transition_time;                               /**< Transition time to turn on/off or to transition to a scene */
    uint16_t scene_number;                                  /**< Scene number to transition to */
} wiced_bt_mesh_scheduler_action_data_t;


#ifdef __cplusplus
extern "C"
{
#endif

#define WICED_BT_MESH_MODELS_MAX_UNIQUE_MODELS_ON_NODE    20

/**
 * Prototype for the bound state value change callback function residing in the second Model.
 * This  will be called by the first state update function in a first model.
 */
typedef void (*wiced_update_bound_state)(void *);


/**
 * @addtogroup  wiced_bt_mesh_models        Mesh Models Library API
 * @ingroup     wiced_bt_mesh
 *
 * Mesh Models library of the WICED SDK provide a simple method for an application to use various Mesh models
 * defined in the Bluetooth Mesh Model specification.
 *
 * @{
 */



/**************************************************************************************************/
/**
* @anchor WICED_BT_MESH_MODEL_INSTANTIATION
* @name Use following macros to instantiate the Mesh models
* \details Following macros instantiate the correct mesh models on a given component
*
* @{
*/
#define WICED_BT_MESH_DEVICE \
    { MESH_COMPANY_ID_BT_SIG, WICED_BT_MESH_CORE_MODEL_ID_CONFIG_SRV, NULL, NULL, NULL }, \
    { MESH_COMPANY_ID_BT_SIG, WICED_BT_MESH_CORE_MODEL_ID_HEALTH_SRV, NULL, NULL, NULL }

#define WICED_BT_MESH_MODEL_CONFIG_CLIENT \
    { MESH_COMPANY_ID_BT_SIG, WICED_BT_MESH_CORE_MODEL_ID_CONFIG_CLNT, wiced_bt_mesh_config_client_message_handler, NULL, NULL }

#define WICED_BT_MESH_MODEL_HEALTH_CLIENT \
    { MESH_COMPANY_ID_BT_SIG, WICED_BT_MESH_CORE_MODEL_ID_HEALTH_CLNT, wiced_bt_mesh_model_health_client_message_handler, NULL, NULL }

// special case for the filter messages between Proxy Client and Proxy Server
#define WICED_BT_MESH_MODEL_PROXY_CLIENT \
    { 0xFFFF, 0xFFFF, wiced_bt_mesh_proxy_client_message_handler, NULL, NULL }

#define WICED_BT_MESH_MODEL_ONOFF_SERVER \
    { MESH_COMPANY_ID_BT_SIG, WICED_BT_MESH_CORE_MODEL_ID_SCENE_SRV, wiced_bt_mesh_model_scene_server_message_handler, NULL, NULL }, \
    { MESH_COMPANY_ID_BT_SIG, WICED_BT_MESH_CORE_MODEL_ID_SCHEDULER_SRV, wiced_bt_mesh_model_scheduler_server_message_handler, NULL, NULL }, \
    { MESH_COMPANY_ID_BT_SIG, WICED_BT_MESH_CORE_MODEL_ID_TIME_SRV, wiced_bt_mesh_model_time_server_message_handler, NULL, NULL }, \
    { MESH_COMPANY_ID_BT_SIG, WICED_BT_MESH_CORE_MODEL_ID_GENERIC_ONOFF_SRV, wiced_bt_mesh_model_onoff_server_message_handler, NULL, wiced_bt_mesh_model_onoff_server_scene_recall_handler }

#define WICED_BT_MESH_MODEL_ONOFF_SETUP_SERVER \
    { MESH_COMPANY_ID_BT_SIG, WICED_BT_MESH_CORE_MODEL_ID_SCENE_SRV, wiced_bt_mesh_model_scene_server_message_handler, NULL, NULL }, \
    { MESH_COMPANY_ID_BT_SIG, WICED_BT_MESH_CORE_MODEL_ID_SCENE_SETUP_SRV, wiced_bt_mesh_model_scene_setup_server_message_handler, NULL, NULL }, \
    { MESH_COMPANY_ID_BT_SIG, WICED_BT_MESH_CORE_MODEL_ID_SCHEDULER_SRV, wiced_bt_mesh_model_scheduler_server_message_handler, NULL, NULL }, \
    { MESH_COMPANY_ID_BT_SIG, WICED_BT_MESH_CORE_MODEL_ID_SCHEDULER_SETUP_SRV, wiced_bt_mesh_model_scheduler_setup_server_message_handler, NULL, NULL }, \
    { MESH_COMPANY_ID_BT_SIG, WICED_BT_MESH_CORE_MODEL_ID_TIME_SRV, wiced_bt_mesh_model_time_server_message_handler, NULL, NULL }, \
    { MESH_COMPANY_ID_BT_SIG, WICED_BT_MESH_CORE_MODEL_ID_GENERIC_ONOFF_SRV, wiced_bt_mesh_model_onoff_server_message_handler, wiced_bt_mesh_model_onoff_server_scene_store_handler, wiced_bt_mesh_model_onoff_server_scene_recall_handler }, \
    { MESH_COMPANY_ID_BT_SIG, WICED_BT_MESH_CORE_MODEL_ID_GENERIC_POWER_ONOFF_SETUP_SRV, NULL, NULL, NULL }

#define WICED_BT_MESH_MODEL_LEVEL_SERVER \
    { MESH_COMPANY_ID_BT_SIG, WICED_BT_MESH_CORE_MODEL_ID_SCENE_SRV, wiced_bt_mesh_model_scene_server_message_handler, NULL, NULL }, \
    { MESH_COMPANY_ID_BT_SIG, WICED_BT_MESH_CORE_MODEL_ID_SCHEDULER_SRV, wiced_bt_mesh_model_scheduler_server_message_handler, NULL, NULL }, \
    { MESH_COMPANY_ID_BT_SIG, WICED_BT_MESH_CORE_MODEL_ID_TIME_SRV, wiced_bt_mesh_model_time_server_message_handler, NULL, NULL }, \
    { MESH_COMPANY_ID_BT_SIG, WICED_BT_MESH_CORE_MODEL_ID_GENERIC_LEVEL_SRV, wiced_bt_mesh_model_level_server_message_handler, NULL, wiced_bt_mesh_model_level_server_scene_recall_handler }

#define WICED_BT_MESH_MODEL_LEVEL_SETUP_SERVER \
    { MESH_COMPANY_ID_BT_SIG, WICED_BT_MESH_CORE_MODEL_ID_SCENE_SRV, wiced_bt_mesh_model_scene_server_message_handler, NULL, NULL }, \
    { MESH_COMPANY_ID_BT_SIG, WICED_BT_MESH_CORE_MODEL_ID_SCENE_SETUP_SRV, wiced_bt_mesh_model_scene_setup_server_message_handler, NULL, NULL }, \
    { MESH_COMPANY_ID_BT_SIG, WICED_BT_MESH_CORE_MODEL_ID_SCHEDULER_SRV, wiced_bt_mesh_model_scheduler_server_message_handler, NULL, NULL }, \
    { MESH_COMPANY_ID_BT_SIG, WICED_BT_MESH_CORE_MODEL_ID_SCHEDULER_SETUP_SRV, wiced_bt_mesh_model_scheduler_setup_server_message_handler, NULL, NULL }, \
    { MESH_COMPANY_ID_BT_SIG, WICED_BT_MESH_CORE_MODEL_ID_TIME_SRV, wiced_bt_mesh_model_time_server_message_handler, NULL, NULL }, \
    { MESH_COMPANY_ID_BT_SIG, WICED_BT_MESH_CORE_MODEL_ID_GENERIC_LEVEL_SRV, wiced_bt_mesh_model_level_server_message_handler, wiced_bt_mesh_model_level_server_scene_store_handler, wiced_bt_mesh_model_level_server_scene_recall_handler }

#define WICED_BT_MESH_MODEL_DEFAULT_TRANSITION_TIME_SERVER \
    { MESH_COMPANY_ID_BT_SIG, WICED_BT_MESH_CORE_MODEL_ID_GENERIC_DEFTT_SRV, wiced_bt_mesh_model_default_transition_time_server_message_handler, NULL, NULL }

#define WICED_BT_MESH_MODEL_POWER_ONOFF_SERVER \
    { MESH_COMPANY_ID_BT_SIG, WICED_BT_MESH_CORE_MODEL_ID_SCENE_SRV, wiced_bt_mesh_model_scene_server_message_handler, NULL, NULL }, \
    { MESH_COMPANY_ID_BT_SIG, WICED_BT_MESH_CORE_MODEL_ID_SCHEDULER_SRV, wiced_bt_mesh_model_scheduler_server_message_handler, NULL, NULL }, \
    { MESH_COMPANY_ID_BT_SIG, WICED_BT_MESH_CORE_MODEL_ID_TIME_SRV, wiced_bt_mesh_model_time_server_message_handler, NULL, NULL }, \
    { MESH_COMPANY_ID_BT_SIG, WICED_BT_MESH_CORE_MODEL_ID_GENERIC_ONOFF_SRV, wiced_bt_mesh_model_onoff_server_message_handler, NULL, wiced_bt_mesh_model_onoff_server_scene_recall_handler }, \
    { MESH_COMPANY_ID_BT_SIG, WICED_BT_MESH_CORE_MODEL_ID_GENERIC_POWER_ONOFF_SRV, wiced_bt_mesh_model_power_onoff_server_message_handler, NULL, NULL }

#define WICED_BT_MESH_MODEL_POWER_ONOFF_SETUP_SERVER \
    { MESH_COMPANY_ID_BT_SIG, WICED_BT_MESH_CORE_MODEL_ID_SCENE_SRV, wiced_bt_mesh_model_scene_server_message_handler, NULL, NULL }, \
    { MESH_COMPANY_ID_BT_SIG, WICED_BT_MESH_CORE_MODEL_ID_SCENE_SETUP_SRV, wiced_bt_mesh_model_scene_setup_server_message_handler, NULL, NULL }, \
    { MESH_COMPANY_ID_BT_SIG, WICED_BT_MESH_CORE_MODEL_ID_SCHEDULER_SRV, wiced_bt_mesh_model_scheduler_server_message_handler, NULL, NULL }, \
    { MESH_COMPANY_ID_BT_SIG, WICED_BT_MESH_CORE_MODEL_ID_SCHEDULER_SETUP_SRV, wiced_bt_mesh_model_scheduler_setup_server_message_handler, NULL, NULL }, \
    { MESH_COMPANY_ID_BT_SIG, WICED_BT_MESH_CORE_MODEL_ID_TIME_SRV, wiced_bt_mesh_model_time_server_message_handler, NULL, NULL }, \
    { MESH_COMPANY_ID_BT_SIG, WICED_BT_MESH_CORE_MODEL_ID_GENERIC_ONOFF_SRV, wiced_bt_mesh_model_onoff_server_message_handler, wiced_bt_mesh_model_onoff_server_scene_store_handler, wiced_bt_mesh_model_onoff_server_scene_recall_handler }, \
    { MESH_COMPANY_ID_BT_SIG, WICED_BT_MESH_CORE_MODEL_ID_GENERIC_POWER_ONOFF_SRV, wiced_bt_mesh_model_power_onoff_server_message_handler, NULL, NULL }, \
    { MESH_COMPANY_ID_BT_SIG, WICED_BT_MESH_CORE_MODEL_ID_GENERIC_DEFTT_SRV, wiced_bt_mesh_model_default_transition_time_server_message_handler, NULL, NULL }, \
    { MESH_COMPANY_ID_BT_SIG, WICED_BT_MESH_CORE_MODEL_ID_GENERIC_POWER_ONOFF_SETUP_SRV, wiced_bt_mesh_model_power_onoff_setup_server_message_handler, NULL, NULL }

#define WICED_BT_MESH_MODEL_POWER_LEVEL_SERVER \
    { MESH_COMPANY_ID_BT_SIG, WICED_BT_MESH_CORE_MODEL_ID_SCENE_SRV, wiced_bt_mesh_model_scene_server_message_handler, NULL, NULL }, \
    { MESH_COMPANY_ID_BT_SIG, WICED_BT_MESH_CORE_MODEL_ID_SCHEDULER_SRV, wiced_bt_mesh_model_scheduler_server_message_handler, NULL, NULL }, \
    { MESH_COMPANY_ID_BT_SIG, WICED_BT_MESH_CORE_MODEL_ID_TIME_SRV, wiced_bt_mesh_model_time_server_message_handler, NULL, NULL }, \
    { MESH_COMPANY_ID_BT_SIG, WICED_BT_MESH_CORE_MODEL_ID_GENERIC_ONOFF_SRV, wiced_bt_mesh_model_onoff_server_message_handler, NULL, NULL }, \
    { MESH_COMPANY_ID_BT_SIG, WICED_BT_MESH_CORE_MODEL_ID_GENERIC_DEFTT_SRV, wiced_bt_mesh_model_default_transition_time_server_message_handler, NULL, NULL }, \
    { MESH_COMPANY_ID_BT_SIG, WICED_BT_MESH_CORE_MODEL_ID_GENERIC_POWER_ONOFF_SRV, wiced_bt_mesh_model_power_onoff_server_message_handler, NULL, NULL }, \
    { MESH_COMPANY_ID_BT_SIG, WICED_BT_MESH_CORE_MODEL_ID_GENERIC_LEVEL_SRV, wiced_bt_mesh_model_level_server_message_handler, NULL, NULL }, \
    { MESH_COMPANY_ID_BT_SIG, WICED_BT_MESH_CORE_MODEL_ID_GENERIC_POWER_LEVEL_SRV, wiced_bt_mesh_model_power_level_server_message_handler, wiced_bt_mesh_model_power_level_server_scene_store_handler, wiced_bt_mesh_model_power_level_server_scene_recall_handler }

#define WICED_BT_MESH_MODEL_POWER_LEVEL_SETUP_SERVER \
    { MESH_COMPANY_ID_BT_SIG, WICED_BT_MESH_CORE_MODEL_ID_SCENE_SRV, wiced_bt_mesh_model_scene_server_message_handler, NULL, NULL }, \
    { MESH_COMPANY_ID_BT_SIG, WICED_BT_MESH_CORE_MODEL_ID_SCENE_SETUP_SRV, wiced_bt_mesh_model_scene_setup_server_message_handler, NULL, NULL }, \
    { MESH_COMPANY_ID_BT_SIG, WICED_BT_MESH_CORE_MODEL_ID_SCHEDULER_SRV, wiced_bt_mesh_model_scheduler_server_message_handler, NULL, NULL }, \
    { MESH_COMPANY_ID_BT_SIG, WICED_BT_MESH_CORE_MODEL_ID_SCHEDULER_SETUP_SRV, wiced_bt_mesh_model_scheduler_setup_server_message_handler, NULL, NULL }, \
    { MESH_COMPANY_ID_BT_SIG, WICED_BT_MESH_CORE_MODEL_ID_TIME_SRV, wiced_bt_mesh_model_time_server_message_handler, NULL, NULL }, \
    { MESH_COMPANY_ID_BT_SIG, WICED_BT_MESH_CORE_MODEL_ID_GENERIC_ONOFF_SRV, wiced_bt_mesh_model_onoff_server_message_handler, NULL, NULL }, \
    { MESH_COMPANY_ID_BT_SIG, WICED_BT_MESH_CORE_MODEL_ID_GENERIC_DEFTT_SRV, wiced_bt_mesh_model_default_transition_time_server_message_handler, NULL, NULL }, \
    { MESH_COMPANY_ID_BT_SIG, WICED_BT_MESH_CORE_MODEL_ID_GENERIC_POWER_ONOFF_SRV, wiced_bt_mesh_model_power_onoff_server_message_handler, NULL, NULL }, \
    { MESH_COMPANY_ID_BT_SIG, WICED_BT_MESH_CORE_MODEL_ID_GENERIC_POWER_ONOFF_SETUP_SRV, wiced_bt_mesh_model_power_onoff_setup_server_message_handler, NULL, NULL }, \
    { MESH_COMPANY_ID_BT_SIG, WICED_BT_MESH_CORE_MODEL_ID_GENERIC_LEVEL_SRV, wiced_bt_mesh_model_level_server_message_handler, NULL, NULL }, \
    { MESH_COMPANY_ID_BT_SIG, WICED_BT_MESH_CORE_MODEL_ID_GENERIC_POWER_LEVEL_SRV, wiced_bt_mesh_model_power_level_server_message_handler, wiced_bt_mesh_model_power_level_server_scene_store_handler, wiced_bt_mesh_model_power_level_server_scene_recall_handler }, \
    { MESH_COMPANY_ID_BT_SIG, WICED_BT_MESH_CORE_MODEL_ID_GENERIC_POWER_LEVEL_SETUP_SRV, wiced_bt_mesh_model_power_level_setup_server_message_handler, NULL, NULL }

#define WICED_BT_MESH_MODEL_BATTERY_SERVER \
    { MESH_COMPANY_ID_BT_SIG, WICED_BT_MESH_CORE_MODEL_ID_GENERIC_BATTERY_SRV, wiced_bt_mesh_model_battery_server_message_handler, NULL, NULL }

#define WICED_BT_MESH_MODEL_LOCATION_SERVER \
    { MESH_COMPANY_ID_BT_SIG, WICED_BT_MESH_CORE_MODEL_ID_GENERIC_LOCATION_SRV, wiced_bt_mesh_model_location_server_message_handler, NULL, NULL }

#define WICED_BT_MESH_MODEL_LOCATION_SETUP_SERVER \
    WICED_BT_MESH_MODEL_LOCATION_SERVER, \
    { MESH_COMPANY_ID_BT_SIG, WICED_BT_MESH_CORE_MODEL_ID_GENERIC_LOCATION_SETUP_SRV, wiced_bt_mesh_model_location_setup_server_message_handler, NULL, NULL }

#define WICED_BT_MESH_MODEL_USER_PROPERTY_SERVER \
    { MESH_COMPANY_ID_BT_SIG, WICED_BT_MESH_CORE_MODEL_ID_GENERIC_USER_PROPERTY_SRV, wiced_bt_mesh_model_user_property_server_message_handler, NULL, NULL }

#define WICED_BT_MESH_MODEL_ADMIN_PROPERTY_SERVER \
    { MESH_COMPANY_ID_BT_SIG, WICED_BT_MESH_CORE_MODEL_ID_GENERIC_ADMIN_PROPERTY_SRV, wiced_bt_mesh_model_admin_property_server_message_handler, NULL, NULL }

#define WICED_BT_MESH_MODEL_MANUFACTURER_PROPERTY_SERVER \
    { MESH_COMPANY_ID_BT_SIG, WICED_BT_MESH_CORE_MODEL_ID_GENERIC_MANUFACT_PROPERTY_SRV, wiced_bt_mesh_model_manufacturer_property_server_message_handler, NULL, NULL }

#define WICED_BT_MESH_MODEL_CLIENT_PROPERTY_SERVER \
    { MESH_COMPANY_ID_BT_SIG, WICED_BT_MESH_CORE_MODEL_ID_GENERIC_CLIENT_PROPERTY_SRV, wiced_bt_mesh_model_client_property_server_message_handler, NULL, NULL }

#define WICED_BT_MESH_MODEL_LIGHT_LIGHTNESS_SERVER \
    { MESH_COMPANY_ID_BT_SIG, WICED_BT_MESH_CORE_MODEL_ID_SCENE_SRV, wiced_bt_mesh_model_scene_server_message_handler, NULL, NULL }, \
    { MESH_COMPANY_ID_BT_SIG, WICED_BT_MESH_CORE_MODEL_ID_SCHEDULER_SRV, wiced_bt_mesh_model_scheduler_server_message_handler, NULL, NULL }, \
    { MESH_COMPANY_ID_BT_SIG, WICED_BT_MESH_CORE_MODEL_ID_TIME_SRV, wiced_bt_mesh_model_time_server_message_handler, NULL, NULL }, \
    { MESH_COMPANY_ID_BT_SIG, WICED_BT_MESH_CORE_MODEL_ID_GENERIC_ONOFF_SRV, wiced_bt_mesh_model_onoff_server_message_handler, NULL, NULL }, \
    { MESH_COMPANY_ID_BT_SIG, WICED_BT_MESH_CORE_MODEL_ID_GENERIC_DEFTT_SRV, wiced_bt_mesh_model_default_transition_time_server_message_handler, NULL, NULL }, \
    { MESH_COMPANY_ID_BT_SIG, WICED_BT_MESH_CORE_MODEL_ID_GENERIC_POWER_ONOFF_SRV, wiced_bt_mesh_model_power_onoff_server_message_handler, NULL, NULL }, \
    { MESH_COMPANY_ID_BT_SIG, WICED_BT_MESH_CORE_MODEL_ID_GENERIC_LEVEL_SRV, wiced_bt_mesh_model_level_server_message_handler, NULL, NULL }, \
    { MESH_COMPANY_ID_BT_SIG, WICED_BT_MESH_CORE_MODEL_ID_LIGHT_LIGHTNESS_SRV, wiced_bt_mesh_model_light_lightness_server_message_handler, wiced_bt_mesh_model_light_lightness_server_scene_store_handler, wiced_bt_mesh_model_light_lightness_server_scene_recall_handler }

#define WICED_BT_MESH_MODEL_LIGHT_LIGHTNESS_SETUP_SERVER \
    { MESH_COMPANY_ID_BT_SIG, WICED_BT_MESH_CORE_MODEL_ID_SCENE_SRV, wiced_bt_mesh_model_scene_server_message_handler, NULL, NULL }, \
    { MESH_COMPANY_ID_BT_SIG, WICED_BT_MESH_CORE_MODEL_ID_SCENE_SETUP_SRV, wiced_bt_mesh_model_scene_setup_server_message_handler, NULL, NULL }, \
    { MESH_COMPANY_ID_BT_SIG, WICED_BT_MESH_CORE_MODEL_ID_SCHEDULER_SRV, wiced_bt_mesh_model_scheduler_server_message_handler, NULL, NULL }, \
    { MESH_COMPANY_ID_BT_SIG, WICED_BT_MESH_CORE_MODEL_ID_SCHEDULER_SETUP_SRV, wiced_bt_mesh_model_scheduler_setup_server_message_handler, NULL, NULL }, \
    { MESH_COMPANY_ID_BT_SIG, WICED_BT_MESH_CORE_MODEL_ID_TIME_SRV, wiced_bt_mesh_model_time_server_message_handler, NULL, NULL }, \
    { MESH_COMPANY_ID_BT_SIG, WICED_BT_MESH_CORE_MODEL_ID_GENERIC_ONOFF_SRV, wiced_bt_mesh_model_onoff_server_message_handler, NULL, NULL }, \
    { MESH_COMPANY_ID_BT_SIG, WICED_BT_MESH_CORE_MODEL_ID_GENERIC_DEFTT_SRV, wiced_bt_mesh_model_default_transition_time_server_message_handler, NULL, NULL }, \
    { MESH_COMPANY_ID_BT_SIG, WICED_BT_MESH_CORE_MODEL_ID_GENERIC_POWER_ONOFF_SRV, wiced_bt_mesh_model_power_onoff_server_message_handler, NULL, NULL }, \
    { MESH_COMPANY_ID_BT_SIG, WICED_BT_MESH_CORE_MODEL_ID_GENERIC_POWER_ONOFF_SETUP_SRV, wiced_bt_mesh_model_power_onoff_setup_server_message_handler, NULL, NULL }, \
    { MESH_COMPANY_ID_BT_SIG, WICED_BT_MESH_CORE_MODEL_ID_GENERIC_LEVEL_SRV, wiced_bt_mesh_model_level_server_message_handler, NULL, NULL }, \
    { MESH_COMPANY_ID_BT_SIG, WICED_BT_MESH_CORE_MODEL_ID_LIGHT_LIGHTNESS_SRV, wiced_bt_mesh_model_light_lightness_server_message_handler, wiced_bt_mesh_model_light_lightness_server_scene_store_handler, wiced_bt_mesh_model_light_lightness_server_scene_recall_handler }, \
    { MESH_COMPANY_ID_BT_SIG, WICED_BT_MESH_CORE_MODEL_ID_LIGHT_LIGHTNESS_SETUP_SRV, wiced_bt_mesh_model_light_lightness_setup_server_message_handler, NULL, NULL }

#define WICED_BT_MESH_MODEL_LIGHT_CTL_SERVER \
    { MESH_COMPANY_ID_BT_SIG, WICED_BT_MESH_CORE_MODEL_ID_SCENE_SRV, wiced_bt_mesh_model_scene_server_message_handler, NULL, NULL }, \
    { MESH_COMPANY_ID_BT_SIG, WICED_BT_MESH_CORE_MODEL_ID_SCHEDULER_SRV, wiced_bt_mesh_model_scheduler_server_message_handler, NULL, NULL }, \
    { MESH_COMPANY_ID_BT_SIG, WICED_BT_MESH_CORE_MODEL_ID_TIME_SRV, wiced_bt_mesh_model_time_server_message_handler, NULL, NULL }, \
    { MESH_COMPANY_ID_BT_SIG, WICED_BT_MESH_CORE_MODEL_ID_GENERIC_ONOFF_SRV, wiced_bt_mesh_model_onoff_server_message_handler, NULL, NULL }, \
    { MESH_COMPANY_ID_BT_SIG, WICED_BT_MESH_CORE_MODEL_ID_GENERIC_DEFTT_SRV, wiced_bt_mesh_model_default_transition_time_server_message_handler, NULL, NULL }, \
    { MESH_COMPANY_ID_BT_SIG, WICED_BT_MESH_CORE_MODEL_ID_GENERIC_POWER_ONOFF_SRV, wiced_bt_mesh_model_power_onoff_server_message_handler, NULL, NULL }, \
    { MESH_COMPANY_ID_BT_SIG, WICED_BT_MESH_CORE_MODEL_ID_GENERIC_LEVEL_SRV, wiced_bt_mesh_model_level_server_message_handler, NULL, NULL }, \
    { MESH_COMPANY_ID_BT_SIG, WICED_BT_MESH_CORE_MODEL_ID_LIGHT_LIGHTNESS_SRV, wiced_bt_mesh_model_light_lightness_server_message_handler, NULL, NULL }, \
    { MESH_COMPANY_ID_BT_SIG, WICED_BT_MESH_CORE_MODEL_ID_LIGHT_CTL_SRV, wiced_bt_mesh_model_light_ctl_server_message_handler, wiced_bt_mesh_model_light_ctl_server_scene_store_handler, wiced_bt_mesh_model_light_ctl_server_scene_recall_handler },

#define WICED_BT_MESH_MODEL_LIGHT_CTL_SETUP_SERVER \
    { MESH_COMPANY_ID_BT_SIG, WICED_BT_MESH_CORE_MODEL_ID_SCENE_SRV, wiced_bt_mesh_model_scene_server_message_handler, NULL, NULL }, \
    { MESH_COMPANY_ID_BT_SIG, WICED_BT_MESH_CORE_MODEL_ID_SCENE_SETUP_SRV, wiced_bt_mesh_model_scene_setup_server_message_handler, NULL, NULL }, \
    { MESH_COMPANY_ID_BT_SIG, WICED_BT_MESH_CORE_MODEL_ID_SCHEDULER_SRV, wiced_bt_mesh_model_scheduler_server_message_handler, NULL, NULL }, \
    { MESH_COMPANY_ID_BT_SIG, WICED_BT_MESH_CORE_MODEL_ID_SCHEDULER_SETUP_SRV, wiced_bt_mesh_model_scheduler_setup_server_message_handler, NULL, NULL }, \
    { MESH_COMPANY_ID_BT_SIG, WICED_BT_MESH_CORE_MODEL_ID_TIME_SRV, wiced_bt_mesh_model_time_server_message_handler, NULL, NULL }, \
    { MESH_COMPANY_ID_BT_SIG, WICED_BT_MESH_CORE_MODEL_ID_GENERIC_ONOFF_SRV, wiced_bt_mesh_model_onoff_server_message_handler, NULL, NULL }, \
    { MESH_COMPANY_ID_BT_SIG, WICED_BT_MESH_CORE_MODEL_ID_GENERIC_DEFTT_SRV, wiced_bt_mesh_model_default_transition_time_server_message_handler, NULL, NULL }, \
    { MESH_COMPANY_ID_BT_SIG, WICED_BT_MESH_CORE_MODEL_ID_GENERIC_POWER_ONOFF_SRV, wiced_bt_mesh_model_power_onoff_server_message_handler, NULL, NULL }, \
    { MESH_COMPANY_ID_BT_SIG, WICED_BT_MESH_CORE_MODEL_ID_GENERIC_POWER_ONOFF_SETUP_SRV, wiced_bt_mesh_model_power_onoff_setup_server_message_handler, NULL, NULL }, \
    { MESH_COMPANY_ID_BT_SIG, WICED_BT_MESH_CORE_MODEL_ID_GENERIC_LEVEL_SRV, wiced_bt_mesh_model_level_server_message_handler, NULL, NULL }, \
    { MESH_COMPANY_ID_BT_SIG, WICED_BT_MESH_CORE_MODEL_ID_LIGHT_LIGHTNESS_SRV, wiced_bt_mesh_model_light_lightness_server_message_handler, NULL, NULL }, \
    { MESH_COMPANY_ID_BT_SIG, WICED_BT_MESH_CORE_MODEL_ID_LIGHT_LIGHTNESS_SETUP_SRV,  wiced_bt_mesh_model_light_lightness_setup_server_message_handler, NULL, NULL }, \
    { MESH_COMPANY_ID_BT_SIG, WICED_BT_MESH_CORE_MODEL_ID_LIGHT_CTL_SRV, wiced_bt_mesh_model_light_ctl_server_message_handler, wiced_bt_mesh_model_light_ctl_server_scene_store_handler, wiced_bt_mesh_model_light_ctl_server_scene_recall_handler }, \
    { MESH_COMPANY_ID_BT_SIG, WICED_BT_MESH_CORE_MODEL_ID_LIGHT_CTL_SETUP_SRV, wiced_bt_mesh_model_light_ctl_setup_server_message_handler, NULL, NULL  }

#define WICED_BT_MESH_MODEL_LIGHT_CTL_TEMPERATURE_SERVER \
    { MESH_COMPANY_ID_BT_SIG, WICED_BT_MESH_CORE_MODEL_ID_GENERIC_LEVEL_SRV, wiced_bt_mesh_model_level_server_message_handler, NULL, NULL }, \
    { MESH_COMPANY_ID_BT_SIG, WICED_BT_MESH_CORE_MODEL_ID_LIGHT_CTL_TEMPERATURE_SRV, wiced_bt_mesh_model_light_ctl_temperature_server_message_handler, NULL, NULL }

#define WICED_BT_MESH_MODEL_LIGHT_HSL_SERVER \
    { MESH_COMPANY_ID_BT_SIG, WICED_BT_MESH_CORE_MODEL_ID_SCENE_SRV, wiced_bt_mesh_model_scene_server_message_handler, NULL, NULL }, \
    { MESH_COMPANY_ID_BT_SIG, WICED_BT_MESH_CORE_MODEL_ID_SCHEDULER_SRV, wiced_bt_mesh_model_scheduler_server_message_handler, NULL, NULL }, \
    { MESH_COMPANY_ID_BT_SIG, WICED_BT_MESH_CORE_MODEL_ID_TIME_SRV, wiced_bt_mesh_model_time_server_message_handler, NULL, NULL }, \
    { MESH_COMPANY_ID_BT_SIG, WICED_BT_MESH_CORE_MODEL_ID_GENERIC_ONOFF_SRV, wiced_bt_mesh_model_onoff_server_message_handler, NULL, NULL }, \
    { MESH_COMPANY_ID_BT_SIG, WICED_BT_MESH_CORE_MODEL_ID_GENERIC_DEFTT_SRV, wiced_bt_mesh_model_default_transition_time_server_message_handler, NULL, NULL }, \
    { MESH_COMPANY_ID_BT_SIG, WICED_BT_MESH_CORE_MODEL_ID_GENERIC_POWER_ONOFF_SRV, wiced_bt_mesh_model_power_onoff_server_message_handler, NULL, NULL }, \
    { MESH_COMPANY_ID_BT_SIG, WICED_BT_MESH_CORE_MODEL_ID_GENERIC_LEVEL_SRV, wiced_bt_mesh_model_level_server_message_handler, NULL, NULL }, \
    { MESH_COMPANY_ID_BT_SIG, WICED_BT_MESH_CORE_MODEL_ID_LIGHT_LIGHTNESS_SRV, wiced_bt_mesh_model_light_lightness_server_message_handler, NULL, NULL }, \
    { MESH_COMPANY_ID_BT_SIG, WICED_BT_MESH_CORE_MODEL_ID_LIGHT_HSL_SRV,  wiced_bt_mesh_model_light_hsl_server_message_handler, wiced_bt_mesh_model_light_hsl_server_scene_store_handler, wiced_bt_mesh_model_light_hsl_server_scene_recall_handler }

#define WICED_BT_MESH_MODEL_LIGHT_HSL_SETUP_SERVER \
    { MESH_COMPANY_ID_BT_SIG, WICED_BT_MESH_CORE_MODEL_ID_SCENE_SRV, wiced_bt_mesh_model_scene_server_message_handler, NULL, NULL }, \
    { MESH_COMPANY_ID_BT_SIG, WICED_BT_MESH_CORE_MODEL_ID_SCENE_SETUP_SRV, wiced_bt_mesh_model_scene_setup_server_message_handler, NULL, NULL }, \
    { MESH_COMPANY_ID_BT_SIG, WICED_BT_MESH_CORE_MODEL_ID_SCHEDULER_SRV, wiced_bt_mesh_model_scheduler_server_message_handler, NULL, NULL }, \
    { MESH_COMPANY_ID_BT_SIG, WICED_BT_MESH_CORE_MODEL_ID_SCHEDULER_SETUP_SRV, wiced_bt_mesh_model_scheduler_setup_server_message_handler, NULL, NULL }, \
    { MESH_COMPANY_ID_BT_SIG, WICED_BT_MESH_CORE_MODEL_ID_TIME_SRV, wiced_bt_mesh_model_time_server_message_handler, NULL, NULL }, \
    { MESH_COMPANY_ID_BT_SIG, WICED_BT_MESH_CORE_MODEL_ID_GENERIC_ONOFF_SRV, wiced_bt_mesh_model_onoff_server_message_handler, NULL, NULL }, \
    { MESH_COMPANY_ID_BT_SIG, WICED_BT_MESH_CORE_MODEL_ID_GENERIC_DEFTT_SRV, wiced_bt_mesh_model_default_transition_time_server_message_handler, NULL, NULL }, \
    { MESH_COMPANY_ID_BT_SIG, WICED_BT_MESH_CORE_MODEL_ID_GENERIC_POWER_ONOFF_SRV, wiced_bt_mesh_model_power_onoff_server_message_handler, NULL, NULL }, \
    { MESH_COMPANY_ID_BT_SIG, WICED_BT_MESH_CORE_MODEL_ID_GENERIC_POWER_ONOFF_SETUP_SRV, wiced_bt_mesh_model_power_onoff_setup_server_message_handler, NULL, NULL }, \
    { MESH_COMPANY_ID_BT_SIG, WICED_BT_MESH_CORE_MODEL_ID_GENERIC_LEVEL_SRV, wiced_bt_mesh_model_level_server_message_handler, NULL, NULL }, \
    { MESH_COMPANY_ID_BT_SIG, WICED_BT_MESH_CORE_MODEL_ID_LIGHT_LIGHTNESS_SRV, wiced_bt_mesh_model_light_lightness_server_message_handler, NULL, NULL }, \
    { MESH_COMPANY_ID_BT_SIG, WICED_BT_MESH_CORE_MODEL_ID_LIGHT_LIGHTNESS_SETUP_SRV,  wiced_bt_mesh_model_light_lightness_setup_server_message_handler, NULL, NULL }, \
    { MESH_COMPANY_ID_BT_SIG, WICED_BT_MESH_CORE_MODEL_ID_LIGHT_HSL_SRV, wiced_bt_mesh_model_light_hsl_server_message_handler, wiced_bt_mesh_model_light_hsl_server_scene_store_handler, wiced_bt_mesh_model_light_hsl_server_scene_recall_handler }, \
    { MESH_COMPANY_ID_BT_SIG, WICED_BT_MESH_CORE_MODEL_ID_LIGHT_HSL_SETUP_SRV, wiced_bt_mesh_model_light_hsl_setup_server_message_handler, NULL, NULL }

#define WICED_BT_MESH_MODEL_LIGHT_HSL_CTL_SETUP_SERVER \
    { MESH_COMPANY_ID_BT_SIG, WICED_BT_MESH_CORE_MODEL_ID_SCENE_SRV, wiced_bt_mesh_model_scene_server_message_handler, NULL, NULL }, \
    { MESH_COMPANY_ID_BT_SIG, WICED_BT_MESH_CORE_MODEL_ID_SCENE_SETUP_SRV, wiced_bt_mesh_model_scene_setup_server_message_handler, NULL, NULL }, \
    { MESH_COMPANY_ID_BT_SIG, WICED_BT_MESH_CORE_MODEL_ID_SCHEDULER_SRV, wiced_bt_mesh_model_scheduler_server_message_handler, NULL, NULL }, \
    { MESH_COMPANY_ID_BT_SIG, WICED_BT_MESH_CORE_MODEL_ID_SCHEDULER_SETUP_SRV, wiced_bt_mesh_model_scheduler_setup_server_message_handler, NULL, NULL }, \
    { MESH_COMPANY_ID_BT_SIG, WICED_BT_MESH_CORE_MODEL_ID_TIME_SRV, wiced_bt_mesh_model_time_server_message_handler, NULL, NULL }, \
    { MESH_COMPANY_ID_BT_SIG, WICED_BT_MESH_CORE_MODEL_ID_GENERIC_ONOFF_SRV, wiced_bt_mesh_model_onoff_server_message_handler, NULL, NULL }, \
    { MESH_COMPANY_ID_BT_SIG, WICED_BT_MESH_CORE_MODEL_ID_GENERIC_DEFTT_SRV, wiced_bt_mesh_model_default_transition_time_server_message_handler, NULL, NULL }, \
    { MESH_COMPANY_ID_BT_SIG, WICED_BT_MESH_CORE_MODEL_ID_GENERIC_POWER_ONOFF_SRV, wiced_bt_mesh_model_power_onoff_server_message_handler, NULL, NULL }, \
    { MESH_COMPANY_ID_BT_SIG, WICED_BT_MESH_CORE_MODEL_ID_GENERIC_POWER_ONOFF_SETUP_SRV, wiced_bt_mesh_model_power_onoff_setup_server_message_handler, NULL, NULL }, \
    { MESH_COMPANY_ID_BT_SIG, WICED_BT_MESH_CORE_MODEL_ID_GENERIC_LEVEL_SRV, wiced_bt_mesh_model_level_server_message_handler, NULL, NULL }, \
    { MESH_COMPANY_ID_BT_SIG, WICED_BT_MESH_CORE_MODEL_ID_LIGHT_LIGHTNESS_SRV, wiced_bt_mesh_model_light_lightness_server_message_handler, NULL, NULL }, \
    { MESH_COMPANY_ID_BT_SIG, WICED_BT_MESH_CORE_MODEL_ID_LIGHT_LIGHTNESS_SETUP_SRV,  wiced_bt_mesh_model_light_lightness_setup_server_message_handler, NULL, NULL }, \
    { MESH_COMPANY_ID_BT_SIG, WICED_BT_MESH_CORE_MODEL_ID_LIGHT_HSL_SRV, wiced_bt_mesh_model_light_hsl_server_message_handler, wiced_bt_mesh_model_light_hsl_server_scene_store_handler, wiced_bt_mesh_model_light_hsl_server_scene_recall_handler }, \
    { MESH_COMPANY_ID_BT_SIG, WICED_BT_MESH_CORE_MODEL_ID_LIGHT_HSL_SETUP_SRV, wiced_bt_mesh_model_light_hsl_setup_server_message_handler, NULL, NULL }, \
    { MESH_COMPANY_ID_BT_SIG, WICED_BT_MESH_CORE_MODEL_ID_LIGHT_CTL_SRV, wiced_bt_mesh_model_light_ctl_server_message_handler, wiced_bt_mesh_model_light_ctl_server_scene_store_handler, wiced_bt_mesh_model_light_ctl_server_scene_recall_handler }, \
    { MESH_COMPANY_ID_BT_SIG, WICED_BT_MESH_CORE_MODEL_ID_LIGHT_CTL_SETUP_SRV, wiced_bt_mesh_model_light_ctl_setup_server_message_handler, NULL, NULL  }

#define WICED_BT_MESH_MODEL_LIGHT_HSL_HUE_SERVER \
    { MESH_COMPANY_ID_BT_SIG, WICED_BT_MESH_CORE_MODEL_ID_GENERIC_LEVEL_SRV, wiced_bt_mesh_model_level_server_message_handler, NULL, NULL }, \
    { MESH_COMPANY_ID_BT_SIG, WICED_BT_MESH_CORE_MODEL_ID_LIGHT_HSL_HUE_SRV, wiced_bt_mesh_model_light_hsl_hue_server_message_handler, NULL, NULL }

#define WICED_BT_MESH_MODEL_LIGHT_HSL_SATURATION_SERVER \
    { MESH_COMPANY_ID_BT_SIG, WICED_BT_MESH_CORE_MODEL_ID_GENERIC_LEVEL_SRV, wiced_bt_mesh_model_level_server_message_handler, NULL, NULL }, \
    { MESH_COMPANY_ID_BT_SIG, WICED_BT_MESH_CORE_MODEL_ID_LIGHT_HSL_SATURATION_SRV, wiced_bt_mesh_model_light_hsl_saturation_server_message_handler, NULL, NULL }

#define WICED_BT_MESH_MODEL_LIGHT_XYL_SERVER \
    { MESH_COMPANY_ID_BT_SIG, WICED_BT_MESH_CORE_MODEL_ID_SCENE_SRV, wiced_bt_mesh_model_scene_server_message_handler, NULL, NULL }, \
    { MESH_COMPANY_ID_BT_SIG, WICED_BT_MESH_CORE_MODEL_ID_SCHEDULER_SRV, wiced_bt_mesh_model_scheduler_server_message_handler, NULL, NULL }, \
    { MESH_COMPANY_ID_BT_SIG, WICED_BT_MESH_CORE_MODEL_ID_TIME_SRV, wiced_bt_mesh_model_time_server_message_handler, NULL, NULL }, \
    { MESH_COMPANY_ID_BT_SIG, WICED_BT_MESH_CORE_MODEL_ID_GENERIC_ONOFF_SRV, wiced_bt_mesh_model_onoff_server_message_handler, NULL, NULL }, \
    { MESH_COMPANY_ID_BT_SIG, WICED_BT_MESH_CORE_MODEL_ID_GENERIC_DEFTT_SRV, wiced_bt_mesh_model_default_transition_time_server_message_handler, NULL, NULL }, \
    { MESH_COMPANY_ID_BT_SIG, WICED_BT_MESH_CORE_MODEL_ID_GENERIC_POWER_ONOFF_SRV, wiced_bt_mesh_model_power_onoff_server_message_handler, NULL, NULL }, \
    { MESH_COMPANY_ID_BT_SIG, WICED_BT_MESH_CORE_MODEL_ID_GENERIC_LEVEL_SRV, wiced_bt_mesh_model_level_server_message_handler, NULL, NULL }, \
    { MESH_COMPANY_ID_BT_SIG, WICED_BT_MESH_CORE_MODEL_ID_LIGHT_LIGHTNESS_SRV, wiced_bt_mesh_model_light_lightness_server_message_handler, NULL, NULL }, \
    { MESH_COMPANY_ID_BT_SIG, WICED_BT_MESH_CORE_MODEL_ID_LIGHT_XYL_SRV, wiced_bt_mesh_model_light_xyl_server_message_handler, wiced_bt_mesh_model_light_xyl_server_scene_store_handler, wiced_bt_mesh_model_light_xyl_server_scene_recall_handler }

#define WICED_BT_MESH_MODEL_LIGHT_XYL_SETUP_SERVER \
    { MESH_COMPANY_ID_BT_SIG, WICED_BT_MESH_CORE_MODEL_ID_SCENE_SRV, wiced_bt_mesh_model_scene_server_message_handler, NULL, NULL }, \
    { MESH_COMPANY_ID_BT_SIG, WICED_BT_MESH_CORE_MODEL_ID_SCENE_SETUP_SRV, wiced_bt_mesh_model_scene_setup_server_message_handler, NULL, NULL }, \
    { MESH_COMPANY_ID_BT_SIG, WICED_BT_MESH_CORE_MODEL_ID_SCHEDULER_SRV, wiced_bt_mesh_model_scheduler_server_message_handler, NULL, NULL }, \
    { MESH_COMPANY_ID_BT_SIG, WICED_BT_MESH_CORE_MODEL_ID_SCHEDULER_SETUP_SRV, wiced_bt_mesh_model_scheduler_setup_server_message_handler, NULL, NULL }, \
    { MESH_COMPANY_ID_BT_SIG, WICED_BT_MESH_CORE_MODEL_ID_TIME_SRV, wiced_bt_mesh_model_time_server_message_handler, NULL, NULL }, \
    { MESH_COMPANY_ID_BT_SIG, WICED_BT_MESH_CORE_MODEL_ID_GENERIC_ONOFF_SRV, wiced_bt_mesh_model_onoff_server_message_handler, NULL, NULL }, \
    { MESH_COMPANY_ID_BT_SIG, WICED_BT_MESH_CORE_MODEL_ID_GENERIC_DEFTT_SRV, wiced_bt_mesh_model_default_transition_time_server_message_handler, NULL, NULL }, \
    { MESH_COMPANY_ID_BT_SIG, WICED_BT_MESH_CORE_MODEL_ID_GENERIC_POWER_ONOFF_SRV, wiced_bt_mesh_model_power_onoff_server_message_handler, NULL, NULL }, \
    { MESH_COMPANY_ID_BT_SIG, WICED_BT_MESH_CORE_MODEL_ID_GENERIC_POWER_ONOFF_SETUP_SRV, wiced_bt_mesh_model_power_onoff_setup_server_message_handler, NULL, NULL }, \
    { MESH_COMPANY_ID_BT_SIG, WICED_BT_MESH_CORE_MODEL_ID_GENERIC_LEVEL_SRV, wiced_bt_mesh_model_level_server_message_handler, NULL, NULL }, \
    { MESH_COMPANY_ID_BT_SIG, WICED_BT_MESH_CORE_MODEL_ID_LIGHT_LIGHTNESS_SRV, wiced_bt_mesh_model_light_lightness_server_message_handler, NULL, NULL }, \
    { MESH_COMPANY_ID_BT_SIG, WICED_BT_MESH_CORE_MODEL_ID_LIGHT_LIGHTNESS_SETUP_SRV,  wiced_bt_mesh_model_light_lightness_setup_server_message_handler, NULL, NULL }, \
    { MESH_COMPANY_ID_BT_SIG, WICED_BT_MESH_CORE_MODEL_ID_LIGHT_XYL_SRV, wiced_bt_mesh_model_light_xyl_server_message_handler, wiced_bt_mesh_model_light_xyl_server_scene_store_handler, wiced_bt_mesh_model_light_xyl_server_scene_recall_handler }, \
    { MESH_COMPANY_ID_BT_SIG, WICED_BT_MESH_CORE_MODEL_ID_LIGHT_XYL_SETUP_SRV, wiced_bt_mesh_model_light_xyl_setup_server_message_handler, NULL, NULL }

#define WICED_BT_MESH_MODEL_LIGHT_LC_MAIN_SERVER \
    { MESH_COMPANY_ID_BT_SIG, WICED_BT_MESH_CORE_MODEL_ID_SCENE_SRV, wiced_bt_mesh_model_scene_server_message_handler, NULL, NULL }, \
    { MESH_COMPANY_ID_BT_SIG, WICED_BT_MESH_CORE_MODEL_ID_SCHEDULER_SRV, wiced_bt_mesh_model_scheduler_server_message_handler, NULL, NULL }, \
    { MESH_COMPANY_ID_BT_SIG, WICED_BT_MESH_CORE_MODEL_ID_TIME_SRV, wiced_bt_mesh_model_time_server_message_handler, NULL, NULL }, \
    { MESH_COMPANY_ID_BT_SIG, WICED_BT_MESH_CORE_MODEL_ID_GENERIC_ONOFF_SRV, wiced_bt_mesh_model_onoff_server_message_handler, NULL, NULL }, \
    { MESH_COMPANY_ID_BT_SIG, WICED_BT_MESH_CORE_MODEL_ID_GENERIC_DEFTT_SRV, wiced_bt_mesh_model_default_transition_time_server_message_handler, NULL, NULL }, \
    { MESH_COMPANY_ID_BT_SIG, WICED_BT_MESH_CORE_MODEL_ID_GENERIC_POWER_ONOFF_SRV, wiced_bt_mesh_model_power_onoff_server_message_handler, NULL, NULL }, \
    { MESH_COMPANY_ID_BT_SIG, WICED_BT_MESH_CORE_MODEL_ID_GENERIC_LEVEL_SRV, wiced_bt_mesh_model_level_server_message_handler, NULL, NULL }, \
    { MESH_COMPANY_ID_BT_SIG, WICED_BT_MESH_CORE_MODEL_ID_LIGHT_LIGHTNESS_SRV, wiced_bt_mesh_model_light_lightness_server_message_handler, NULL, NULL }

#define WICED_BT_MESH_MODEL_LIGHT_LC_SERVER \
    { MESH_COMPANY_ID_BT_SIG, WICED_BT_MESH_CORE_MODEL_ID_GENERIC_ONOFF_SRV, wiced_bt_mesh_model_onoff_server_message_handler, NULL, NULL }, \
    { MESH_COMPANY_ID_BT_SIG, WICED_BT_MESH_CORE_MODEL_ID_LIGHT_LC_SRV, wiced_bt_mesh_model_light_lc_server_message_handler, wiced_bt_mesh_model_light_lc_server_scene_store_handler, wiced_bt_mesh_model_light_lc_server_scene_recall_handler }, \
    { MESH_COMPANY_ID_BT_SIG, WICED_BT_MESH_CORE_MODEL_ID_SENSOR_CLNT }

#define WICED_BT_MESH_MODEL_LIGHT_LC_SETUP_SERVER \
    { MESH_COMPANY_ID_BT_SIG, WICED_BT_MESH_CORE_MODEL_ID_SCENE_SRV, wiced_bt_mesh_model_scene_server_message_handler, NULL, NULL }, \
    { MESH_COMPANY_ID_BT_SIG, WICED_BT_MESH_CORE_MODEL_ID_SCENE_SETUP_SRV, wiced_bt_mesh_model_scene_setup_server_message_handler, NULL, NULL }, \
    { MESH_COMPANY_ID_BT_SIG, WICED_BT_MESH_CORE_MODEL_ID_SCHEDULER_SRV, wiced_bt_mesh_model_scheduler_server_message_handler, NULL, NULL }, \
    { MESH_COMPANY_ID_BT_SIG, WICED_BT_MESH_CORE_MODEL_ID_SCHEDULER_SETUP_SRV, wiced_bt_mesh_model_scheduler_setup_server_message_handler, NULL, NULL }, \
    { MESH_COMPANY_ID_BT_SIG, WICED_BT_MESH_CORE_MODEL_ID_TIME_SRV, wiced_bt_mesh_model_time_server_message_handler, NULL, NULL }, \
    { MESH_COMPANY_ID_BT_SIG, WICED_BT_MESH_CORE_MODEL_ID_GENERIC_ONOFF_SRV, wiced_bt_mesh_model_onoff_server_message_handler, NULL, NULL }, \
    { MESH_COMPANY_ID_BT_SIG, WICED_BT_MESH_CORE_MODEL_ID_LIGHT_LC_SRV, wiced_bt_mesh_model_light_lc_server_message_handler, wiced_bt_mesh_model_light_lc_server_scene_store_handler, wiced_bt_mesh_model_light_lc_server_scene_recall_handler }, \
    { MESH_COMPANY_ID_BT_SIG, WICED_BT_MESH_CORE_MODEL_ID_LIGHT_LC_SETUP_SRV, wiced_bt_mesh_model_light_lc_setup_server_message_handler, NULL, NULL }, \
    { MESH_COMPANY_ID_BT_SIG, WICED_BT_MESH_CORE_MODEL_ID_SENSOR_CLNT, NULL, NULL }, \
    { MESH_COMPANY_ID_BT_SIG, WICED_BT_MESH_CORE_MODEL_ID_GENERIC_USER_PROPERTY_SRV, wiced_bt_mesh_model_user_property_server_message_handler, NULL, NULL }

#define WICED_BT_MESH_MODEL_SENSOR_SERVER \
    { MESH_COMPANY_ID_BT_SIG, WICED_BT_MESH_CORE_MODEL_ID_SENSOR_SRV, wiced_bt_mesh_model_sensor_server_message_handler, NULL, NULL }

#define WICED_BT_MESH_MODEL_SENSOR_SETUP_SERVER \
    WICED_BT_MESH_MODEL_SENSOR_SERVER, \
    { MESH_COMPANY_ID_BT_SIG, WICED_BT_MESH_CORE_MODEL_ID_SENSOR_SETUP_SRV, wiced_bt_mesh_model_sensor_setup_server_message_handler, NULL, NULL }

#define WICED_BT_MESH_MODEL_SCENE_SERVER \
    { MESH_COMPANY_ID_BT_SIG, WICED_BT_MESH_CORE_MODEL_ID_GENERIC_SCENE_SRV, wiced_bt_mesh_model_scene_server_message_handler, NULL, NULL }

#define WICED_BT_MESH_MODEL_SCENE_SETUP_SERVER \
    { MESH_COMPANY_ID_BT_SIG, WICED_BT_MESH_CORE_MODEL_ID_GENERIC_SCENE_SRV, wiced_bt_mesh_model_scene_server_message_handler, NULL, NULL }, \
    { MESH_COMPANY_ID_BT_SIG, WICED_BT_MESH_CORE_MODEL_ID_GENERIC_SCENE_SETUP_SRV, wiced_bt_mesh_model_scene_setup_server_message_handler, NULL, NULL }

#define WICED_BT_MESH_MODEL_TIME_SERVER \
    { MESH_COMPANY_ID_BT_SIG, WICED_BT_MESH_CORE_MODEL_ID_TIME_SRV, wiced_bt_mesh_model_time_server_message_handler, NULL, NULL }

#define WICED_BT_MESH_MODEL_TIME_SETUP_SERVER \
    { MESH_COMPANY_ID_BT_SIG, WICED_BT_MESH_CORE_MODEL_ID_TIME_SRV, wiced_bt_mesh_model_time_server_message_handler, NULL, NULL }, \
    { MESH_COMPANY_ID_BT_SIG, WICED_BT_MESH_CORE_MODEL_ID_TIME_SETUP_SRV, wiced_bt_mesh_model_time_setup_server_message_handler, NULL, NULL }

#define WICED_BT_MESH_MODEL_ONOFF_CLIENT \
    { MESH_COMPANY_ID_BT_SIG, WICED_BT_MESH_CORE_MODEL_ID_GENERIC_ONOFF_CLNT, wiced_bt_mesh_model_onoff_client_message_handler, NULL, NULL }

#define WICED_BT_MESH_MODEL_LEVEL_CLIENT \
    { MESH_COMPANY_ID_BT_SIG, WICED_BT_MESH_CORE_MODEL_ID_GENERIC_LEVEL_CLNT, wiced_bt_mesh_model_level_client_message_handler, NULL, NULL }

#define WICED_BT_MESH_MODEL_DEFAULT_TRANSITION_TIME_CLIENT \
    { MESH_COMPANY_ID_BT_SIG, WICED_BT_MESH_CORE_MODEL_ID_GENERIC_DEFTT_CLNT, wiced_bt_mesh_model_default_transition_time_client_message_handler, NULL, NULL }

#define WICED_BT_MESH_MODEL_POWER_ONOFF_CLIENT \
    { MESH_COMPANY_ID_BT_SIG, WICED_BT_MESH_CORE_MODEL_ID_GENERIC_POWER_ONOFF_CLNT, wiced_bt_mesh_model_power_onoff_client_message_handler, NULL, NULL }

#define WICED_BT_MESH_MODEL_POWER_LEVEL_CLIENT \
    { MESH_COMPANY_ID_BT_SIG, WICED_BT_MESH_CORE_MODEL_ID_GENERIC_POWER_LEVEL_CLNT, wiced_bt_mesh_model_power_level_client_message_handler, NULL, NULL }

#define WICED_BT_MESH_MODEL_BATTERY_CLIENT \
    { MESH_COMPANY_ID_BT_SIG, WICED_BT_MESH_CORE_MODEL_ID_GENERIC_BATTERY_CLNT, wiced_bt_mesh_model_battery_client_message_handler, NULL, NULL }

#define WICED_BT_MESH_MODEL_LOCATION_CLIENT \
    { MESH_COMPANY_ID_BT_SIG, WICED_BT_MESH_CORE_MODEL_ID_GENERIC_LOCATION_CLNT, wiced_bt_mesh_model_location_client_message_handler, NULL, NULL }

#define WICED_BT_MESH_MODEL_PROPERTY_CLIENT \
    { MESH_COMPANY_ID_BT_SIG, WICED_BT_MESH_CORE_MODEL_ID_GENERIC_PROPERTY_CLNT, wiced_bt_mesh_model_property_client_message_handler, NULL, NULL }

#define WICED_BT_MESH_MODEL_SENSOR_CLIENT \
    { MESH_COMPANY_ID_BT_SIG,  WICED_BT_MESH_CORE_MODEL_ID_SENSOR_CLNT, wiced_bt_mesh_model_sensor_client_message_handler, NULL, NULL }

#define WICED_BT_MESH_MODEL_SCENE_CLIENT \
    { MESH_COMPANY_ID_BT_SIG, WICED_BT_MESH_CORE_MODEL_ID_SCENE_CLNT, wiced_bt_mesh_model_scene_client_message_handler, NULL, NULL }

#define WICED_BT_MESH_MODEL_SCHEDULER_CLIENT \
    { MESH_COMPANY_ID_BT_SIG, WICED_BT_MESH_CORE_MODEL_ID_SCHEDULER_CLNT, wiced_bt_mesh_model_scheduler_client_message_handler, NULL, NULL }

#define WICED_BT_MESH_MODEL_TIME_CLIENT \
    { MESH_COMPANY_ID_BT_SIG, WICED_BT_MESH_CORE_MODEL_ID_TIME_CLNT, wiced_bt_mesh_model_time_client_message_handler, NULL, NULL }

#define WICED_BT_MESH_MODEL_LIGHT_LIGHTNESS_CLIENT \
    { MESH_COMPANY_ID_BT_SIG, WICED_BT_MESH_CORE_MODEL_ID_LIGHT_LIGHTNESS_CLNT, wiced_bt_mesh_model_light_lightness_client_message_handler, NULL, NULL }

#define WICED_BT_MESH_MODEL_LIGHT_CTL_CLIENT \
    { MESH_COMPANY_ID_BT_SIG, WICED_BT_MESH_CORE_MODEL_ID_LIGHT_CTL_CLNT, wiced_bt_mesh_model_light_ctl_client_message_handler, NULL, NULL }

#define WICED_BT_MESH_MODEL_LIGHT_HSL_CLIENT \
    { MESH_COMPANY_ID_BT_SIG, WICED_BT_MESH_CORE_MODEL_ID_LIGHT_HSL_CLNT, wiced_bt_mesh_model_light_hsl_client_message_handler, NULL, NULL }

#define WICED_BT_MESH_MODEL_LIGHT_XYL_CLIENT \
    { MESH_COMPANY_ID_BT_SIG, WICED_BT_MESH_CORE_MODEL_ID_LIGHT_XYL_CLNT, wiced_bt_mesh_model_light_xyl_client_message_handler, NULL, NULL }

#define WICED_BT_MESH_MODEL_LIGHT_LC_CLIENT \
    { MESH_COMPANY_ID_BT_SIG, WICED_BT_MESH_CORE_MODEL_ID_LIGHT_LC_CLNT, wiced_bt_mesh_model_light_lc_client_message_handler, NULL, NULL }

#define WICED_BT_MESH_MODEL_TIME_CLIENT \
    { MESH_COMPANY_ID_BT_SIG, WICED_BT_MESH_CORE_MODEL_ID_TIME_CLNT, wiced_bt_mesh_model_time_client_message_handler, NULL, NULL }

/** @} WICED_BT_MESH_MODEL_INSTANTIATION */

/**
 * @anchor BATTERY_PRESENCE_FLAGS
 * @name Battery Presence Flags
 * @{ */
#define WICED_BT_MESH_BATTERY_FLAG_NOT_PRESENT              0x00    /**< The battery is not present */
#define WICED_BT_MESH_BATTERY_FLAG_PRESENT_REMOVABLE        0x01    /**< The battery is present and is removable */
#define WICED_BT_MESH_BATTERY_FLAG_PRESENT_NON_REMOVABLE    0x02    /**< The battery is present and is non-removable */
#define WICED_BT_MESH_BATTERY_FLAG_PRESENCE_UNKNOWN         0x03    /**< The battery presence is unknown */
/* @} */

typedef uint8_t wiced_bt_mesh_battery_presence_t;                   /**< Generic Battery Flags Presence State (see @ref BATTERY_PRESENCE_FLAGS "Battery Presence Flags") */

/**
 * @anchor BATTERY_CHARGE_FLAGS
 * @name Battery Charge Flags
 * @{ */
#define WICED_BT_MESH_BATTERY_FLAG_LEVEL_CRITICALLY_LOW     0x00    /**< The battery charge is Critically Low Level. */
#define WICED_BT_MESH_BATTERY_FLAG_LEVEL_LOW                0x01    /**< The battery charge is Low Level. */
#define WICED_BT_MESH_BATTERY_FLAG_LEVEL_GOOD               0x02    /**< The battery charge is Good Level. */
#define WICED_BT_MESH_BATTERY_FLAG_LEVEL_UNKNOWN            0x03    /**< The battery charge is unknown. */
/* @} */

typedef uint8_t wiced_bt_mesh_battery_indicator_t;                  /**< Generic Battery Flags Indicator States (see @ref BATTERY_CHARGE_FLAGS "Battery Indicator Flags") */

/**
 * @anchor BATTERY_CHARGING_FLAGS
 * @name Battery Charging State Flags
 * @{ */
#define WICED_BT_MESH_BATTERY_FLAG_NOT_CHARGABLE            0x00    /**< The battery is not chargeable. */
#define WICED_BT_MESH_BATTERY_FLAG_NOT_CHARGING             0x01    /**< The battery is chargeable and is not charging. */
#define WICED_BT_MESH_BATTERY_FLAG_CHARGING                 0x02    /**< The battery is chargeable and is charging. */
#define WICED_BT_MESH_BATTERY_FLAG_CHARGING_UNKNOWN         0x03    /**< The battery charging state is unknown. */
/* @} */

typedef uint8_t wiced_bt_mesh_battery_charging_t;                   /**< Generic Battery Flags Charging States (see @ref BATTERY_CHARGING_FLAGS "Battery Charging Flags") */

/**
 * @anchor BATTERY_SERVICABLITY_FLAGS
 * @name Battery Service Required State Flags
 * @{ */
#define WICED_BT_MESH_BATTERY_FLAG_SERVICE_NOT_REQUIRED     0x01    /**< The battery does not require service. */
#define WICED_BT_MESH_BATTERY_FLAG_SERVICE_REQUIRED         0x02    /**< The battery requires service. */
#define WICED_BT_MESH_BATTERY_FLAG_SERVICABILITY_UNKNOWN    0x03    /**< The battery serviceability is unknown. */
/* @} */

typedef uint8_t wiced_bt_mesh_battery_servicibility_t;              /**< Generic Battery Flags Serviceability States (see @ref BATTERY_SERVICABLITY_FLAGS "Battery Service Required Flags") */

#define WICED_BT_BATTERY_LEVEL_UNKNOWN                      0xff        /**< Current battery level unknown */
#define WICED_BT_BATTERY_TIME_TO_DISCHARGE_UNKNOWN          0xffffff    /**< Time to full discharge unknown */
#define WICED_BT_BATTERY_TIME_TO_CHARGE_UNKNOWN             0xffffff    /**< Time to full charge unknown */


/**
 * Mesh Battery Status Event used to report battery state to the client device
 */
typedef struct
{
    uint8_t                                 battery_level;          /**< The Generic Battery Level state is a value ranging from 0 percent through 100 percent */
    uint32_t                                time_to_discharge;      /**< The remaining time (in minutes) of the discharging process */
    uint32_t                                time_to_charge;         /**< The remaining time of the charging process is not known. */
    wiced_bt_mesh_battery_presence_t        presence;               /**< Generic Battery Flags Presence */
    wiced_bt_mesh_battery_indicator_t       level_inidicator;       /**< Generic Battery Flags Indicator */
    wiced_bt_mesh_battery_charging_t        charging;               /**< Generic Battery Flags Charging */
    wiced_bt_mesh_battery_servicibility_t   servicability;          /**< Generic Battery Flags Serviceability */
} mesh_battery_event_t;

/**
 * Global Location Data structure exchanged between the application Generic Location Model
 */
typedef struct
{
    uint32_t global_latitude;                       /**< Global latitude */
    uint32_t global_longitude;                      /**< Global longitude */
    uint16_t global_altitude;                       /**< Global altitude */
} wiced_bt_mesh_location_global_data_t;

/**
 * Local Location Data structure exchanged between the application Generic Location Model
 */
typedef struct
{
    uint16_t local_north;                           /**< North coordinate of the device using a local coordinate system */
    uint16_t local_east;                            /**< East coordinate of the device using a local coordinate system */
    uint16_t local_altitude;                        /**< Altitude of the device relative to the Generic Location Global Altitude */
    uint8_t  floor_number;                          /**< Floor number where the element is installed */
    uint8_t  is_mobile;                             /**< 0 if device is stationary, 1 if mobile */
    uint8_t  update_time;                           /**< time in seconds elapsed since the last update of the device's position (t = 2 ^ (update_time - 3)) */
    uint8_t  precision;                             /**< location precision in meters position (m = 2 ^ (precision - 3)) */
} wiced_bt_mesh_location_local_data_t;

#define WICED_BT_MESH_ONOFF_MAX_RETRANSMIT_TIME 6   /**< Max retransmit duration for on/off set command in seconds */

#define WICED_BT_MESH_TRANSITION_TIME_DEFAULT   0xFFFFFFFF  /**< Application or parent model should use default setting for transition time */

/**
 * OnOff Set Data structure exchanged between the application Generic OnOff Model
 */
typedef struct
{
    uint8_t  onoff;                                 /**< The target value of the Generic OnOff state */
    uint32_t transition_time;                       /**< Transition time to the target state */
    uint16_t delay;                                 /**< Delay before starting the transition */
} wiced_bt_mesh_onoff_set_data_t;

/**
 * OnOff Status Data structure exchanged between the application and Generic OnOff Model
 */
typedef struct
{
    uint8_t  present_onoff;                         /**< The present value of the Generic OnOff state */
    uint8_t  target_onoff;                          /**< The target value of the Generic OnOff state */
    uint32_t remaining_time;                        /**< Remaining time for transaction */
} wiced_bt_mesh_onoff_status_data_t;

#define WICED_BT_MESH_LEVEL_MAX_RETRANSMIT_TIME         6       /**< Max retransmit duration for level set command in seconds. This time is also indicates max duration of the delta command.  */
#define WICED_BT_MESH_LEVEL_MOVE_MAX_TRANSITION_TIME    0x3E    /**< Max transition time for Move command */

/**
 * Level Set Data structure exchanged between the application and the Generic Level Model
 */
typedef struct
{
    int16_t level;                                  /**< The target value of the Generic Level state */
    uint32_t transition_time;                       /**< Transition time to the target level */
    uint16_t delay;                                 /**< Delay before starting the transition */
} wiced_bt_mesh_level_set_level_t;

/**
 * Delta Set Data structure exchanged between the application and the Generic Level Model
 */
typedef struct
{
    int32_t     delta;                             /**< The Delta change of the Generic Level state */
    uint32_t    transition_time;                   /**< Transition time to the target level */
    uint16_t    delay;                             /**< Delay before starting the transition */
#define WICED_BT_MESH_LEVEL_DELTA_START             0   /**< Delta transaction start */
#define WICED_BT_MESH_LEVEL_DELTA_CONTINUATION      1   /**< Delta transaction continuation */
    uint8_t     continuation;                      /**< If false, a new delta transaction is started */
} wiced_bt_mesh_level_set_delta_t;

/**
 * Move Set Data structure exchanged between the application and the Generic Level Model
 */
typedef struct
{
    int16_t  delta;                                 /**< The Delta Level step to calculate Move speed for the Generic Level state. */
    uint32_t transition_time;                       /**< Transition time to the target level */
    uint16_t delay;                                 /**< Delay before starting the transition */
#define WICED_BT_MESH_LEVEL_MOVE_START              0   /**< Move transaction start */
#define WICED_BT_MESH_LEVEL_MOVE_STOP               1   /**< Move transaction stop */
    uint8_t  continuation;                      /**< If false, a new delta transaction is started */
} wiced_bt_mesh_level_set_move_t;

/**
 * Level Status Data structure exchanged between the application and Generic Level Model
 */
typedef struct
{
    uint16_t present_level;                         /**< The present value of the Generic Level state */
    uint16_t target_level;                          /**< The target value of the Generic Level state */
    uint32_t remaining_time;                        /**< Remaining time for transaction */
} wiced_bt_mesh_level_status_data_t;

/**
 * Default Transition Time Data structure exchanged between an application and the Generic Default Transition Time Model
 */
typedef struct
{
    uint32_t time;                                  /**< Transition time in milliseconds */
} wiced_bt_mesh_default_transition_time_data_t;


/**
 * Power OnOff Data structure exchanged between an application and the Power OnOff Model
 */
typedef struct
{
    uint8_t on_power_up;                                /**< Value on power up */
} wiced_bt_mesh_power_onoff_data_t;

/**
 * Power Level Status Data structure exchanged between the application and Generic Power Level Model
 */
typedef struct
{
    uint16_t present_power;                         /**< The present value of the Generic Power Actual state */
    uint16_t target_power;                          /**< The target value of the Generic Power Actual state  */
    uint32_t remaining_time;                        /**< Remaining time for transaction */
} wiced_bt_mesh_power_level_status_data_t;

/**
 * Power Level Last Status Data structure exchanged between the application and Generic Power Level Model
 */
typedef struct
{
    uint16_t power;                                 /**< The value of the Generic Power Last state. */
} wiced_bt_mesh_power_level_last_data_t;

/**
 * Power Level Default Status Data structure exchanged between the application and Generic Power Level Model
 */
typedef struct
{
    uint16_t power;                                 /**< The value of the Generic Power Default state. */
} wiced_bt_mesh_power_default_data_t;

/**
 * Power Level Set Range Data structure exchanged between the application and Generic Power Level Model
 */
typedef struct
{
    uint16_t power_min;                             /**< The value of the Generic Power Min field of the Generic Power Range state. */
    uint16_t power_max;                             /**< The value of the Generic Power Range Max field of the Generic Power Range state. */
} wiced_bt_mesh_power_level_range_set_data_t;

/**
 * Power Range Status Data structure exchanged between the application and Generic Power Level Model
 */
typedef struct
{
    uint8_t  status;                                /**< status of set range */
    uint16_t power_min;                             /**< The value of the Generic Power Min field of the Generic Power Range state. */
    uint16_t power_max;                             /**< The value of the Generic Power Range Max field of the Generic Power Range state. */
} wiced_bt_mesh_power_range_status_data_t;

/**
 * Level Set Data structure exchanged between the application and the Generic Level Model
 */
typedef struct
{
    uint16_t level;                                 /**< The target value of the Generic Power Actual state */
    uint32_t transition_time;                       /**< Transition time to the target level */
    uint16_t delay;                                 /**< Delay before starting the transition */
} wiced_bt_mesh_power_level_set_level_t;

/**
 * Light Lightness Status Data structure exchanged between the application and Light Lightness Level Model
 */
typedef struct
{
    uint16_t present;                               /**< The present value of the Light Lightness Actual state */
    uint16_t target;                                /**< The target value of the Light Lightness Actual state  */
    uint32_t remaining_time;                        /**< Remaining time for transaction */
} wiced_bt_mesh_light_lightness_status_data_t;

/**
 * Light Lightness Set Data structure exchanged between the application and the models Light Lightness Server library
 */
typedef struct
{
    uint16_t lightness_actual;                      /**< The target value of the Light Lightness Actual */
    uint16_t lightness_linear;                      /**< The target value of the Light Lightness Linear */
    uint32_t transition_time;                       /**< Transition time to the target level */
    uint16_t delay;                                 /**< Delay before starting the transition */
} wiced_bt_mesh_light_lightness_set_t;

/**
 * Light Lightness Actual Set Data structure exchanged between the application and the models Light Lightness Client library
 */
typedef struct
{
    uint16_t lightness_actual;                      /**< The target value of the Light Lightness Actual */
    uint32_t transition_time;                       /**< Transition time to the target level */
    uint16_t delay;                                 /**< Delay before starting the transition */
} wiced_bt_mesh_light_lightness_actual_set_t;

/**
 * Light Lightness Linear Set Data structure exchanged between the application and the models Light Lightness Client library
 */
typedef struct
{
    uint16_t lightness_linear;                      /**< The target value of the Light Lightness Linear */
    uint32_t transition_time;                       /**< Transition time to the target level */
    uint16_t delay;                                 /**< Delay before starting the transition */
} wiced_bt_mesh_light_lightness_linear_set_t;

/**
 * Light Lightness Last Status Data structure exchanged between the application and Light Lightness Model
 */
typedef struct
{
    uint16_t last_level;                           /**< The value of the Light Lightness Last state. */
} wiced_bt_mesh_light_lightness_last_data_t;

/**
 * Lightness Default Status Data structure exchanged between the application and Light Lightness Model
 */
typedef struct
{
    uint16_t default_level;                        /**< The default lightness level for the Light Lightness Actual state. */
} wiced_bt_mesh_light_lightness_default_data_t;

/**
* Lightness Range Status Data structure exchanged between the application and Generic Power Level Model
*/
typedef struct
{
    uint16_t min_level;                             /**< The value of the Light Lightness Min field of the Light Lightness Range state. */
    uint16_t max_level;                             /**< The value of the Light Lightness Max field of the Light Lightness Range state. */
} wiced_bt_mesh_light_lightness_range_set_data_t;

/**
 * Lightness Range Status Data structure exchanged between the application and Light Lightness Model
 */
typedef struct
{
    uint8_t  status;                                /**< status of set range */
    uint16_t min_level;                             /**< The value of the Light Lightness Min field of the Light Lightness Range state. */
    uint16_t max_level;                             /**< The value of the Light Lightness Max field of the Light Lightness Range state. */
} wiced_bt_mesh_light_lightness_range_status_data_t;

/**
 * Property types defined in the Mesh Models specifications
 */
#define WICED_BT_MESH_PROPERTY_TYPE_CLIENT          0           /**< Property type client */
#define WICED_BT_MESH_PROPERTY_TYPE_ADMIN           1           /**< Property type admin */
#define WICED_BT_MESH_PROPERTY_TYPE_MANUFACTURER    2           /**< Property type manufacturer */
#define WICED_BT_MESH_PROPERTY_TYPE_USER            3           /**< Property type user */

/**
 * Properties Get Data for Generic Property Models
 */
typedef struct
{
    uint8_t  type;                                  /**< One of the property types defined in the wiced_bt_mesh_property_type enumeration */
    uint16_t starting_id;                           /**< The property ID to be reported in the property status message */
} wiced_bt_mesh_properties_get_data_t;

/**
 * Property Get Data for Generic Property Models
 */
typedef struct
{
    uint8_t  type;                                  /**< One of the property types defined in the wiced_bt_mesh_property_type enumeration */
    uint16_t id;                                    /**< The property ID to be reported in the property status message */
} wiced_bt_mesh_property_get_data_t;

/**
 * Properties Status Data for Generic Property Models
 */
typedef struct
{
    uint8_t  type;                                  /**< One of the property types defined in the wiced_bt_mesh_property_type enumeration */
    uint8_t  properties_num;                        /**< Number of property IDs */
    uint16_t id[1];                                 /**< Start of the list of Property IDs. */
} wiced_bt_mesh_properties_status_data_t;

/**
 * Property Set Data structure exchanged between the application Generic User Property Models
 */
typedef struct
{
    uint8_t  type;                                  /**< One of the property types defined in the wiced_bt_mesh_property_type enumeration */
    uint16_t id;                                    /**< The Property ID received in the set message */
    uint8_t  access;                                /**< Access level to be set for the property.  This is a bit map when 0x01 indicates readable and 0x02 writable */
    uint16_t len;                                   /**< Size of the Property Value */
#define MESH_MAX_PROPERTY_VALUE_LEN 32  // ToDo
    uint8_t  value[MESH_MAX_PROPERTY_VALUE_LEN];    /**< Property value */
} wiced_bt_mesh_property_set_data_t;

/**
 * Property Status Data structure exchanged between the application Generic User Property Models
 */
typedef struct
{
    uint8_t  type;                                  /**< One of the property types defined in the wiced_bt_mesh_property_type enumeration */
    uint16_t id;                                    /**< The Property ID received in the set message */
    uint8_t  access;                                /**< Access level allowed for the property.  This is a bit map when 0x01 indicates readable and 0x02 writable */
    uint16_t len;                                   /**< Size of the Property Value */
    uint8_t  value[1];                              /**< First byte of property value */
} wiced_bt_mesh_property_status_data_t;

/**
 * Data structure representing the state of Light Color Temperature device
 */
typedef struct
{
    uint16_t lightness;                             /**< Value of the Light CTL Lightness state */
    uint16_t temperature;                           /**< Value of the Light CTL Temperature state */
    uint16_t delta_uv;                              /**< Value of the Light CTL Delta UV state*/
} wiced_bt_mesh_light_ctl_data_t;

/**
* Light CTL Status Data structure exchanged between the application and the models library
*/
typedef struct
{
    wiced_bt_mesh_light_ctl_data_t present;         /**< Present value of the Light CTL Lightness, Temperature and Delta UV */
    wiced_bt_mesh_light_ctl_data_t target;          /**< Target value of the Light CTL Lightness, Temperature and Delta UV */
    uint32_t remaining_time;                        /**< Remaining time for transaction */
} wiced_bt_mesh_light_ctl_status_data_t;

/**
 * Light CTL Set Data structure exchanged between the application and the models library
 */
typedef struct
{
    wiced_bt_mesh_light_ctl_data_t target;          /**< Target value of the Light CTL state */
    uint32_t transition_time;                       /**< Transition time to the target level */
    uint16_t delay;                                 /**< Delay before starting the transition */
} wiced_bt_mesh_light_ctl_set_t;

/**
 * Light Lightness Last Status Data structure exchanged between the application and Light Lightness Model
 */
typedef struct
{
    uint16_t target_temperature;                    /**< Target value of the Light CTL Temperature state */
    uint16_t target_delta_uv;                       /**< Target value of the Light CTL Delta UV state */
    uint32_t transition_time;                       /**< Transition time to the target level */
    uint16_t delay;                                 /**< Delay before starting the transition */
} wiced_bt_mesh_light_ctl_temperature_set_t;

/**
 * Lightness Default Status Data structure exchanged between the application and Light Lightness Model
 */
typedef struct
{
    wiced_bt_mesh_light_ctl_data_t default_status;          /**< The default lightness, temperature, delta uv for the Light CTL state. */
} wiced_bt_mesh_light_ctl_default_data_t;

/**
 * Temperature Range Status Data structure exchanged between the application and Light CTL Temperature Model
 */
typedef struct
{
    uint8_t  status;                                /**< Status of the Set Range operation */
    uint16_t min_level;                             /**< The value of the Light Lightness Min field of the Light Lightness Range state. */
    uint16_t max_level;                             /**< The value of the Light Lightness Max field of the Light Lightness Range state. */
} wiced_bt_mesh_light_ctl_temperature_range_status_data_t;

/**
 * Temperature Range Set Data structure exchanged between the application and Light CTL Temperature Model
 */
typedef struct
{
    uint16_t min_level;                             /**< The value of the Light Temperature Min field of the Light CTL Temperature Range state. */
    uint16_t max_level;                             /**< The value of the Light Lightness Max field of the Light Lightness Range state. */
} wiced_bt_mesh_light_ctl_temperature_range_data_t;

/**
 * Light Lightness Set Data structure exchanged between the application and the models library
 */
typedef struct
{
    uint16_t level;                                 /**< The target value of the Light Lightness Actual state */
    uint32_t transition_time;                       /**< Transition time to the target level */
    uint16_t delay;                                 /**< Delay before starting the transition */
} wiced_bt_mesh_light_delta_uv_set_level_t;


/**
 * Light Lightness Last Status Data structure exchanged between the application and Light Lightness Model
 */
typedef struct
{
    uint16_t last_level;                           /**< The value of the Light Lightness Last state. */
} wiced_bt_mesh_light_delta_uv_last_data_t;

/**
 * Lightness Default Status Data structure exchanged between the application and Light Lightness Model
 */
typedef struct
{
    uint16_t default_level;                        /**< The default lightness level for the Light Lightness Actual state. */
} wiced_bt_mesh_light_delta_uv_default_data_t;

/**
 * Lightness Range Status Data structure exchanged between the application and Generic Power Level Model
 */
typedef struct
{
    uint16_t min_level;                             /**< The value of the Light Lightness Min field of the Light Lightness Range state. */
    uint16_t max_level;                             /**< The value of the Light Lightness Max field of the Light Lightness Range state. */
} wiced_bt_mesh_light_delta_uv_range_data_t;

/**
 * Data structure representing the state of Light HSL device
 */
typedef struct
{
    uint16_t lightness;                             /**< Value of the Light HSL Lightness state */
    uint16_t hue;                                   /**< Value of the Light HSL Hue state */
    uint16_t saturation;                            /**< Value of the Light HSL Saturation UV state*/
} wiced_bt_mesh_light_hsl_data_t;

/**
* Light HSL Status Data structure exchanged between the application and the models library
*/
typedef struct
{
    wiced_bt_mesh_light_hsl_data_t present;         /**< Present value of HSL state */
    uint32_t remaining_time;                        /**< Remaining time for transaction */
} wiced_bt_mesh_light_hsl_status_data_t;

/**
* Light HSL Target Status Data structure exchanged between the application and the models library
*/
typedef struct
{
    wiced_bt_mesh_light_hsl_data_t target;          /**< Target value of HSL state */
    uint32_t remaining_time;                        /**< Remaining time for transaction */
} wiced_bt_mesh_light_hsl_target_status_data_t;

/**
 * Light HSL Set Data structure exchanged between the application and the models library
 */
typedef struct
{
    wiced_bt_mesh_light_hsl_data_t target;          /**< Target value of the Light HSL state */
    uint32_t transition_time;                       /**< Transition time to the target level */
    uint16_t delay;                                 /**< Delay before starting the transition */
} wiced_bt_mesh_light_hsl_set_t;

/**
* Light HSL Hue Set Data structure exchanged between the application and the models library
*/
typedef struct
{
    uint16_t level;                                 /**< The target value of the Light HSL Hue state */
    uint32_t transition_time;                       /**< Transition time to the target level */
    uint16_t delay;                                 /**< Delay before starting the transition */
} wiced_bt_mesh_light_hsl_hue_set_t;

/**
* Light HSL Saturation Set Data structure exchanged between the application and the models library
*/
typedef struct
{
    uint16_t level;                                 /**< The target value of the Light HSL Saturation state */
    uint32_t transition_time;                       /**< Transition time to the target level */
    uint16_t delay;                                 /**< Delay before starting the transition */
} wiced_bt_mesh_light_hsl_saturation_set_t;

/**
* Light HSL Set Default Data structure exchanged between the application and the models library
*/
typedef struct
{
    wiced_bt_mesh_light_hsl_data_t default_status;   /**< Target value of the Light HSL state */
} wiced_bt_mesh_light_hsl_default_data_t;

/**
 * Light HSL Hue status Data structure exchanged between the application and Light HSL Model
 */
typedef struct
{
    uint16_t present_hue;                           /**< The present value of the Light HSL Hue state */
    uint16_t target_hue;                            /**< The target value of the Light HSL Hue state */
    uint32_t remaining_time;                        /**< Remaining time for transaction */
} wiced_bt_mesh_light_hsl_hue_status_data_t;

/**
 * Light HSL Saturation Status Data structure exchanged between the application and Light HSL Model
 */
typedef struct
{
    uint16_t present_saturation;                    /**< The present value of the Light HSL Saturation state */
    uint16_t target_saturation;                     /**< The target value of the Light HSL Saturation state */
    uint32_t remaining_time;                        /**< Remaining time for transaction */
} wiced_bt_mesh_light_hsl_saturation_status_data_t;

/**
 * Light HSL Range Set Data structure exchanged between the application and Light HSL Level Model
 */
typedef struct
{
    uint16_t hue_min;                               /**< The value of the Hue Min  */
    uint16_t hue_max;                               /**< The value of the Hue Max  */
    uint16_t saturation_min;                        /**< The value of the Saturation Min  */
    uint16_t saturation_max;                        /**< The value of the Saturation Max  */
} wiced_bt_mesh_light_hsl_range_set_data_t;

/**
 * Light HSL Range Set Data structure exchanged between the application and Light HSL Level Model
 */
typedef struct
{
    uint8_t  status;                                /**< Status Code for the requesting message */
    uint16_t hue_min;                               /**< The value of the Hue Min  */
    uint16_t hue_max;                               /**< The value of the Hue Max  */
    uint16_t saturation_min;                        /**< The value of the Saturation Min  */
    uint16_t saturation_max;                        /**< The value of the Saturation Max  */
} wiced_bt_mesh_light_hsl_range_status_data_t;

/**
 * Light HSL Range Default Status structure exchanged between the application and Light HSL Level Model
 */
typedef struct
{
    wiced_bt_mesh_light_hsl_data_t default_status;  /**< HSL status data */
} wiced_bt_mesh_light_hsl_default_status_data_t;

/**
 * Data structure reppresenting the state of the Light xyL device
 */
typedef struct
{
    uint16_t lightness;                             /**< Value of the Light xyL Lightness */
    uint16_t x;                                     /**< x coordinate on the CIE1931 color space chart of a color light emitted by an element */
    uint16_t y;                                     /**< y coordinate on the CIE1931 color space chart of a color light emitted by an element */
} wiced_bt_mesh_light_xyl_data_t;

/**
 * Light xyL x_y_default and range settings
 */
typedef struct
{
    uint16_t x_default;                             /**< default value of x */
    uint16_t x_min;                                 /**< minimum value of x */
    uint16_t x_max;                                 /**< maximum value of x */
    uint16_t y_default;                             /**< default value of x */
    uint16_t y_min;                                 /**< minimum value of y */
    uint16_t y_max;                                 /**< maximum value of y */
} wiced_bt_mesh_light_xyl_xy_settings_t;


/**
* Light xyL Status Data structure exchanged between the application and the models library
*/
typedef struct
{
    wiced_bt_mesh_light_xyl_data_t present;         /**< Present value of xyL state */
    uint32_t remaining_time;                        /**< Remaining time for transaction */
} wiced_bt_mesh_light_xyl_status_data_t;

/**
* Light xyL Target Value Data structure exchanged between the application and the models library
*/
typedef struct
{
    wiced_bt_mesh_light_xyl_data_t target;          /**< Target value of xyL state */
    uint32_t remaining_time;                        /**< Remaining time for transaction */
} wiced_bt_mesh_light_xyl_target_status_data_t;

/**
* Light xyL Set Data structure exchanged between the application and the models library
*/
typedef struct
{
    wiced_bt_mesh_light_xyl_data_t target;          /**< Target value of the Light xyL state */
    uint32_t transition_time;                       /**< Transition time to the target levels */
    uint16_t delay;                                 /**< Delay before starting the transition */
} wiced_bt_mesh_light_xyl_set_t;

/**
* Light xyL Set Default Data structure exchanged between the application and the models library
*/
typedef struct
{
    wiced_bt_mesh_light_xyl_data_t default_status;   /**< Default value of the Light xyL state */
} wiced_bt_mesh_light_xyl_default_data_t;

/**
* Light xyL Range Set Data structure exchanged between the application and Light xyL Level Model
*/
typedef struct
{
    uint16_t x_min;                                 /**< Light xyL x Range Min field of the Light xyL x Range state  */
    uint16_t x_max;                                 /**< Light xyL x Range Max field of the Light xyL y Range state  */
    uint16_t y_min;                                 /**< Light xyL y Range Min field of the Light xyL y Range state  */
    uint16_t y_max;                                 /**< Light xyL y Range Max field of the Light xyL y Range state  */
} wiced_bt_mesh_light_xyl_range_set_data_t;

/**
* Light xyL Range Set Data structure exchanged between the application and Light xyL Level Model
*/
typedef struct
{
    uint8_t  status;                                /**< Status Code for the requesting message */
    uint16_t x_min;                                 /**< Light xyL x Range Min field of the Light xyL x Range state  */
    uint16_t x_max;                                 /**< Light xyL x Range Max field of the Light xyL y Range state  */
    uint16_t y_min;                                 /**< Light xyL y Range Min field of the Light xyL y Range state  */
    uint16_t y_max;                                 /**< Light xyL y Range Max field of the Light xyL y Range state  */
} wiced_bt_mesh_light_xyl_range_status_data_t;

/**
* Light xyL Range Default Status structure exchanged between the application and Light xyL Level Model
*/
typedef struct
{
    wiced_bt_mesh_light_xyl_data_t default_status;  /**< Light xyL x Default value */
} wiced_bt_mesh_light_xyl_default_status_data_t;

/**
 * Light LC Mode Set data structure exchanged between the application and Light LC library
 */
typedef struct
{
    uint8_t mode;                                   /**< The target value of the Light LC Mode state */
} wiced_bt_mesh_light_lc_mode_set_data_t;

/**
 * Light LC Occupancy Mode Set data structure exchanged between the application and Light LC library
 */
typedef struct
{
    uint8_t mode;                                   /**< The target value of the Light LC Mode state */
} wiced_bt_mesh_light_lc_occupancy_mode_set_data_t;

/**
 * Light LC Light OnOff Set data structure exchanged between the application and Light LC library
 */
typedef struct
{
    uint8_t light_onoff;                            /**< The target value of the Light LC Light OnOff state */
    uint32_t transition_time;                       /**< Transition time to the target state */
    uint16_t delay;                                 /**< Delay before starting the transition */
} wiced_bt_mesh_light_lc_light_onoff_set_data_t;

/**
 * Light LC Light OnOff Status data structure exchanged between the application and Light LC library
 */
typedef struct
{
    uint8_t present_onoff;                          /**< Present value of the Light LC Light OnOff state */
    uint8_t target_onoff;                           /**< Target value of the Light LC Light OnOff state */
    uint32_t remaining_time;                        /**< Remaining time */
} wiced_bt_mesh_light_lc_light_onoff_status_data_t;

/**
 * Light LC Light OnOff Set data structure exchanged between the application and Light LC library
 */
typedef struct
{
    uint16_t linear_out;                            /**< The target value of the Light LC Linear Output state */
    uint32_t transition_time;                       /**< Transition time to the target state */
    uint16_t delay;                                 /**< Delay before starting the transition */
} wiced_bt_mesh_light_lc_linear_out_set_data_t;

/**
 * Light LC Property Get Data structure exchanged between the application and Light LC library
 */
typedef struct
{
    uint16_t id;                                   /**< The Property ID identifying a Light LC Property */
} wiced_bt_mesh_light_lc_property_get_data_t;

/**
 * Light LC Property Set Data structure exchanged between the application and Light LC library
 */
typedef struct
{
    uint16_t id;                                   /**< The Property ID identifying a Light LC Property */
    uint16_t len;                                  /**< Size of the Property Value */
#define MESH_MAX_LIGHT_LC_PROPERTY_VALUE_LEN    4  /**< Light LC maximum property value len */
    uint8_t  value[MESH_MAX_LIGHT_LC_PROPERTY_VALUE_LEN];   /**< Property value */
} wiced_bt_mesh_light_lc_property_set_data_t;

/**
 * Light LC Property Status Data structure exchanged between the application and Light LC library
 */
typedef struct
{
    uint16_t id;                                   /**< The Property ID received in the set message */
    uint16_t len;                                  /**< Size of the Property Value */
    uint8_t  value[MESH_MAX_LIGHT_LC_PROPERTY_VALUE_LEN];   /**< Property value */
} wiced_bt_mesh_light_lc_property_status_data_t;



/**
 * \brief Scene store message handler
 * \details Each model may define a message handler if it needs to store data per scene in the NVRAM.  When Scene Server
 * receives Scene Store message it calls each model's handler.
 *
 * @param       element_idx Element index for which scene information is being stored
 * @param       p_buffer Pointer to a buffer where data should be stored
 * @param       buf_size Maximum amount of data a model can store
 *
 * @return      Number of bytes the model wrote into the provided buffer
 */
typedef uint16_t (*wiced_model_scene_store_handler_t)(uint8_t element_idx, uint8_t *p_buffer, uint16_t buf_size);

/**
 * \brief Scene retrieve message handler
 * \details Each model may define a message handler if it stores data per scene in the NVRAM.  When Scene Server
 * receives Scene Recall message it calls each model's handler.
 *
 * @param       element_idx Element index for which scene information is being recalled
 * @param       p_buffer Pointer to a buffer where model's data is located
 * @param       buf_size Maximum amount of data a model can get from the buffer
 *
 * @return      Number of bytes the model read from the provided buffer
 */
typedef uint16_t (*wiced_model_scene_recall_handler_t)(uint8_t element_idx, uint8_t *p_buffer, uint16_t buf_size, uint32_t transition_time, uint32_t delay);

/**
 * @addtogroup  wiced_bt_mesh_battery_server        Mesh Battery Server
 * @ingroup     wiced_bt_mesh_models
 *
 * The WICED Mesh Battery Server module provides a way for a client to retrieve the Battery status of this device. The Server Model
 * can also be configured by a Provisioner to allow sending the Status message to a specific node or group of nodes. 
 *
 * On startup the application should call wiced_bt_mesh_model_battery_server_init function providing the callback that
 * will be called when a request for the battery status is received. The application should call the wiced_bt_mesh_battery_server_send_status
 * function in response to a get request, or when application logic requires battery status to be sent out.  In the latter
 * case the message will be sent out only if device is configured remotely with the destination address where the battery status
 * should be sent to and the application key to be used to secure the message.
 *
 * @{
 */

/**
 * \brief Battery Server callback 
 * \details The Battery Server callback is called by the Mesh Models library on receiving a get message from the peer
 *
 * @param       event that application should process (see @ref BATTERY_EVENT "Mesh Battery Events")
 * @param       p_event information about the message received.  The same pointer should be used in the reply if required.
 *
 * @return      None
 */
typedef void(wiced_bt_mesh_battery_server_callback_t)(uint16_t event, wiced_bt_mesh_event_t *p_event);

/**
 * \brief Battery Server Model initialization
 * \details Application should call this function during startup to register a callback which will be executed when received message requires application processing.
 *
 * @param       p_callback The application callback function that will be executed by the mesh models library when application action is required, or when a reply for the application request has been received.
 * @param       is_provisioned If TRUE, the application is being restarted after being provisioned or after a power loss. If FALSE the model cleans up NVRAM on startup.
 *
 * @return      WICED_TRUE if initialization was successful
 */
void wiced_bt_mesh_model_battery_server_init(wiced_bt_mesh_battery_server_callback_t *p_callback, wiced_bool_t is_provisioned);

/**
 * \brief Battery Server Message Handler
 * \details Application can call this models library function to process a message received from the Battery Client.
 * The function parses the message and if appropriate calls the application back to perform functionality.
 *
 * @param       p_event Mesh event with information about received message.
 * @param       p_data Pointer to the data portion of the message
 * @param       data_len Length of the data in the message
 *
 * @return      WICED_TRUE if the message is for this company ID/Model combination, WICED_FALSE otherwise.
*/
wiced_bool_t wiced_bt_mesh_model_battery_server_message_handler(wiced_bt_mesh_event_t *p_event, uint8_t *p_data, uint16_t data_len);

/**
 * \brief The application can call this function to send status message to the Battery Client. The mesh event should
 * contain information if this is a reply or an unsolicited message.
 *
 * @param       p_event Mesh event with the information about the message that has been received in the request, or created by the app for unsolicited message.
 * @param       p_data Pointer to the Battery data
 *
 * @return      None
*/
void wiced_bt_mesh_battery_server_send_status(wiced_bt_mesh_event_t *p_event, mesh_battery_event_t *p_data);

/* @} wiced_bt_mesh_battery_server */

/**
 * @addtogroup  wiced_bt_mesh_battery_client        Mesh Battery Client
 * @ingroup     wiced_bt_mesh_models
 *
 * The WICED Mesh Battery Client module provides a way for an application to retrieve the battery status of a mesh node. The module
 * can also process unsolicited status messages from mesh nodes. 
 *
 * On startup the application should call wiced_bt_mesh_model_battery_client_init function providing the callback that
 * will be called when battery status message is received. The application can call the wiced_bt_mesh_battery_client_send_get
 * function to retrieve the battery status of a mesh device.
 *
 * @{
 */

/**
 * \brief Battery Client callback
 * \details The Battery Client callback is called by the Mesh Models library on receiving a Battery Status message from the peer
 *
 * @param       event Event that the application should process (see @ref BATTERY_EVENT "Mesh Battery Events")
 * @param       p_event The information about the message received.  The same pointer should be used in the reply if required.
 *
 * @return      None
*/
typedef void(wiced_bt_mesh_battery_client_callback_t)(uint16_t event, wiced_bt_mesh_event_t *p_event, mesh_battery_event_t *p_data);

/**
 * \brief Battery Client Model initialization
 * \details Application should call this function during startup to register a callback which will be executed when received message requires application processing.
 *
 * @param       p_callback The application callback function that will be executed by the mesh models library when application action is required, or when a reply for the application request has been received.
 * @param       is_provisioned If TRUE, the application is being restarted after being provisioned or after a power loss. If FALSE the model cleans up NVRAM on startup.
 *
 * @return      None
 */
void wiced_bt_mesh_model_battery_client_init(wiced_bt_mesh_battery_client_callback_t *p_callback, wiced_bool_t is_provisioned);

/**
 * \brief Battery Client Message Handler
 * \details The application should call this models library function to process a message received from the Battery Server.
 *  The function parses and validates the message and if appropriate calls the application to perform functionality.
 *
 * @param       p_event Mesh event with information about received message.
 * @param       p_data Pointer to the data portion of the message
 * @param       data_len Length of the data in the message
 *
 * @return      WICED_TRUE if the message is for this company ID/Model combination, WICED_FALSE otherwise.
 */
wiced_bool_t wiced_bt_mesh_model_battery_client_message_handler(wiced_bt_mesh_event_t *p_event, uint8_t *p_data, uint16_t data_len);

/**
 * \brief Battery Client Send
 * \details The application can call this function to send get request to the Battery Server. The mesh event should
 * contain the destination address and the application key index to be used to secure the message. In some cases the 
 * model can be configured by the provisioner to send the request to a specific address and use specific key. In that
 * case application can use zeroes in the destination address and app key index fields.
 *
 * @param       p_event Mesh event created by the app for unsolicited message.
 *
 * @return      None
 */
void wiced_bt_mesh_battery_client_send_get(wiced_bt_mesh_event_t *p_event);

/* @} wiced_bt_mesh_battery_client */

/**
 * @addtogroup  wiced_bt_mesh_location_server        Mesh Location Server
 * @ingroup     wiced_bt_mesh_models
 *
 * The WICED Mesh Location Server module provides a way for a client to set the Global and Local location for a mesh device. 
 * Setup location can be retrieved by the same or different clients. The Location Server may also be configured to 
 * publish unsolicited messages with the location information to a specific node or to a group of nodes. 
 *
 * Two types of location information can be exchanged between the client and the server.  The Global Location Data (see
 * @ref wiced_bt_mesh_location_global_data_t) and the Local Location Data (see @ref wiced_bt_mesh_location_local_data_t).
 *
 * On startup the application should call the wiced_bt_mesh_model_location_server_init function providing the callback
 * to be executed when a Get or Set request is received from a mesh device. The application should call the 
 * wiced_bt_mesh_model_location_server_send function in response to a get request, a set request when reply is required
 * or when application logic requires location information to be sent out.  In the latter case the message will be sent 
 * out only if device is configured remotely with the destination address where the location status should be set to and 
 * and the application key to be used to secure the message.
 *
 * @{
 */

/**
 * \brief Location Server callback.
 * \details The Location Server callback is executed by the Mesh Models library on receiving a Get or a Set message from 
 * the peer. In case of the Set the application should update the location information. In case of the Get or if the 
 * reply variable is set to TRUE in the p_event structure, the application should call the wiced_bt_mesh_model_location_server_send 
 * function returning p_event back to the library. Otherwise p_event should be released.
 *
 * @param       event that application should process (see @ref LOCATION_EVENT "Mesh Location Events")
 * @param       p_event information about the message received. The same pointer should be used in the reply if required.
 * @param       p_data The pointer to the data received.
 *
 * @return      None
 */
typedef void(wiced_bt_mesh_location_server_callback_t)(uint16_t event, wiced_bt_mesh_event_t *p_event, void *p_data);

/**
 * \brief Location Server initialization
 *
 * @param       p_callback The application callback function that will be executed by the mesh models library when application action is required, or when a reply for the application request has been received.
 * @param       is_provisioned If TRUE, the application is being restarted after being provisioned or after a power loss. If FALSE the model cleans up NVRAM on startup.
 *
 * @return      None
 */
void wiced_bt_mesh_model_location_server_init(wiced_bt_mesh_location_server_callback_t *p_callback, wiced_bool_t is_provisioned);

/**
 * \brief Location Server Message Handler
 * \details Application can call this models library function to process a message received from the Location Client.
 * The function parses the message and if appropriate calls the application back to perform functionality.
 *
 * @param       p_event Mesh event with information about received message. 
 * @param       p_data Pointer to the data portion of the message
 * @param       data_len Length of the data in the message
 *
 * @return      WICED_TRUE if the message is for this company ID/Model combination, WICED_FALSE otherwise.
*/
wiced_bool_t wiced_bt_mesh_model_location_server_message_handler(wiced_bt_mesh_event_t *p_event, uint8_t *p_data, uint16_t data_len);

/**
 * \brief The application can call this function to send status message to the Location Client. The mesh event should
 * contain information if this is a reply or an unsolicited message.
 *
 * @param       type Type of the message to be send to the Client (see @ref LOCATION_EVENT "Location Events")
 * @param       p_event Mesh event with the information about the message that has been received in the request, or created by the app for unsolicited message.
 * @param       p_data Depending on the type of the message this should point to the Global Location Data (@ref wiced_bt_mesh_location_global_data_t) or the Local Location Data (see @ref wiced_bt_mesh_location_local_data_t)
 *
 * @return      None
*/
void wiced_bt_mesh_model_location_server_send(uint16_t type, wiced_bt_mesh_event_t *p_event, void *p_data);

/**
* \brief Location Setup Server Message Handler
* \details Application can call this models library function to process a message received from the Location Setup Client.
* The function parses the message and if appropriate calls the application back to perform functionality.
*
* @param       p_event Mesh event with information about received message.
* @param       p_data Pointer to the data portion of the message
* @param       data_len Length of the data in the message
*
 * @return      WICED_TRUE if the message is for this company ID/Model combination, WICED_FALSE otherwise.
*/
wiced_bool_t wiced_bt_mesh_model_location_setup_server_message_handler(wiced_bt_mesh_event_t *p_event, uint8_t *p_data, uint16_t data_len);

/* @} wiced_bt_mesh_location_server */

/**
 * @addtogroup  wiced_bt_mesh_location_client        Mesh Location Client
 * @ingroup     wiced_bt_mesh_models
 *
 * The WICED Mesh Location Client module provides a way for an application to configure the device with a Global or a Local Location
 * data or to retrieve this data. The module can also process unsolicited status messages from mesh nodes. 
 *
 * On startup the application should call wiced_bt_mesh_model_location_client_init function providing the callback that
 * will be called when location status message is received. The application can call the appropriate Get or Set 
 * function to retrieve or setup the Global or Local Location data on a specific mesh node.
 *
 * @{
 */

/**
 * \brief Location Client callback 
 * \details The Location Client callback is called by the Mesh Models library on receiving a message from the peer.
 * Depending on the event, the p_data can contain he Global Location Data (see @ref wiced_bt_mesh_location_global_data_t) or
 * the Local Location Data (see @ref wiced_bt_mesh_location_local_data_t). The application is responsible for releasing the 
 * p_event when it completes processing of the data.
 *
 * @param       event that application should process (see @ref LOCATION_EVENT "Mesh Location Events")
 * @param       p_event information about the message received including the message source address.
 * @param       p_data pointer to the data to send
 *
 * @return      None
 */
typedef void(wiced_bt_mesh_location_client_callback_t)(uint16_t event, wiced_bt_mesh_event_t *p_event, void *p_data);

/**
 * \brief Location Client Model initialization
 *
 * @param       p_callback The application callback function that will be executed by the mesh models library when application action is required, or when a reply for the application request has been received.
 * @param       is_provisioned If TRUE, the application is being restarted after being provisioned or after a power loss. If FALSE the model cleans up NVRAM on startup.
 *
 * @return      None
 */
void wiced_bt_mesh_model_location_client_init(wiced_bt_mesh_location_client_callback_t *p_callback, wiced_bool_t is_provisioned);

/**
 * \brief Location Client Message Handler
 * \details The mesh core library calls this function for each message received.  The function returns WICED_TRUE if the message is destined for this specific model and successfully processed, and returns WICED_FALSE otherwise. 
 * The function parses the message and if appropriate calls the parent back to perform functionality.
 *
 * @param       p_event Mesh event with information about received message.
 * @param       p_data Pointer to the data portion of the message
 * @param       data_len Length of the data in the message
 *
 * @return      WICED_TRUE if the message is for this company ID/Model/Element Index combination, WICED_FALSE otherwise.
*/
wiced_bool_t wiced_bt_mesh_model_location_client_message_handler(wiced_bt_mesh_event_t *p_event, uint8_t *p_data, uint16_t data_len);


/**
 * \brief The application can call this function to send Global Location Get client message to the server.
 *
 * @param       p_event Mesh event created by the app for unsolicited message.
 *
 * @return      None
 */
void wiced_bt_mesh_model_location_client_send_global_get(wiced_bt_mesh_event_t *p_event);

/**
 * \brief The application can call this function to send Set Global Location client message to the server.
 *
 * @param       p_event Mesh event with the information about the message that has been created by the app for unsolicited message.
 * @param       p_data Pointer to the data to send
 *
 * @return      None
 */
void wiced_bt_mesh_model_location_client_send_global_set(wiced_bt_mesh_event_t *p_event, wiced_bt_mesh_location_global_data_t* p_location_data);

/**
 * \brief The application can call this function to send Local Location Get client message to the server.
 *
 * @param       p_event Mesh event created by the app for unsolicited message.
 *
 * @return      None
 */
void wiced_bt_mesh_model_location_client_send_global_get(wiced_bt_mesh_event_t *p_event);

/**
 * \brief The application can call this function to send Get Location client message to the server.
 *
 * @param       p_event Mesh event created by the app for unsolicited message.
 *
 * @return      None
 */
void wiced_bt_mesh_model_location_client_send_local_get(wiced_bt_mesh_event_t *p_event);

/**
 * \brief The application can call this function to send Set Location client message to the server.
 *
 * @param       p_event Mesh event created by the app for unsolicited message.
 * @param       p_data Pointer to the data to send
 *
 * @return      None
 */
void wiced_bt_mesh_model_location_client_send_local_set(wiced_bt_mesh_event_t *p_event, wiced_bt_mesh_location_local_data_t* p_location_data);

/* @} wiced_bt_mesh_location_client */

/**
 * @addtogroup  wiced_bt_mesh_onoff_server        Mesh OnOff Server
 * @ingroup     wiced_bt_mesh_models
 *
 * The WICED Mesh OnOff Server module provides a way for a client to control a device which provides On/Off functionality. 
 *
 * On startup the application should call wiced_bt_mesh_model_onoff_server_init function providing the callback that
 * will be called when a request is received to get the current status or to turn the device on or off. The application should call the 
 * wiced_bt_mesh_onoff_server_send_status function in response to a get request or to a set request if the reply variable of the p_event
 * structure is set. It is also recommended that the application calls the wiced_bt_mesh_onoff_server_send_status function when the
 * state of the device is changed locally.  In this ase the message will be sent out only if device is configured remotely with the 
 * destination address where the OnOff status should be sent to and the application key to be used to secure the message.
 *
 * @{
*/

/**
 * \brief Generic On/Off Server callback is called by the Mesh Models library on receiving a message from the peer
 *
 * @param       event that application should process (see @ref ONOFF_EVENT "On/Off Events")
 * @param       p_event information about the message received.  The same pointer should be used in the reply if required.
 * @param       p_data pointer to the on/off state data
 *
 * @return      None
 */
typedef void(wiced_bt_mesh_onoff_server_callback_t)(uint16_t event, wiced_bt_mesh_event_t *p_event, void *p_data);

/**
 * \brief On/Off Models initialization
 *
 * @param       element_idx Device element to where model is used
 * @param       p_callback Application or Parent model callback that will be executed on receiving a message
 * @param       is_provisioned If TRUE, the application is being restarted after being provisioned or after a power loss. If FALSE the model cleans up NVRAM on startup.
 *
 * @return      None
 */
void wiced_bt_mesh_model_onoff_server_init(uint8_t element_idx, wiced_bt_mesh_onoff_server_callback_t *p_callback, wiced_bool_t is_provisioned);

/**
 * \brief On/Off Server Message Handler
 * \details The mesh core library calls this function for each message received.  The function returns WICED_TRUE if the message is destined for this specific model and successfully processed, and returns WICED_FALSE otherwise. On/Off Client.
 * The function parses the message and if appropriate calls the parent back to perform functionality.
 *
 * @param       p_event Mesh event with information about received message.
 * @param       p_data Pointer to the data portion of the message
 * @param       data_len Length of the data in the message
 *
 * @return      WICED_TRUE if the message is for this company ID/Model/Element Index combination, WICED_FALSE otherwise.
*/
wiced_bool_t wiced_bt_mesh_model_onoff_server_message_handler(wiced_bt_mesh_event_t *p_event, uint8_t *p_data, uint16_t data_len);

/**
 * \brief On/Off Server Scene Store Handler
 * \details The mesh Scene Server calls this function so that the module can store required data for the scene.
 *
 * @param       element_idx Element index for which scene information is being stored
 * @param       p_buffer Pointer to a buffer where data should be stored
 * @param       buf_size Maximum amount of data a model can store
 *
 * @return      Number of bytes the model wrote into the provided buffer
 */
uint16_t wiced_bt_mesh_model_onoff_server_scene_store_handler(uint8_t element_idx, uint8_t *p_buffer, uint16_t buffer_len);

/**
 * \brief On/Off Server Scene Store Handler
 * \details When Scene Server receives Scene Recall message it calls this function with data previously stored in the NVRAM.
 *
 * @param       element_idx Element index for which scene information is being recalled
 * @param       p_buffer Pointer to a buffer where model's data is located
 * @param       buf_size Maximum amount of data a model can get from the buffer
 *
 * @return      Number of bytes the model read from the provided buffer
 */
uint16_t wiced_bt_mesh_model_onoff_server_scene_recall_handler(uint8_t element_idx, uint8_t *p_buffer, uint16_t buffer_len, uint32_t transition_time, uint32_t delay);

/**
 * \brief An application or a a parent model can call this function to send status message to the On/Off Client. The mesh event should
 * contain information if this is a reply or an unsolicited message.
 *
 * @param       p_event Mesh event with the information about the message that has been received in the request, or created by the app for unsolicited message.
 * @param       p_data Pointer to the status data
 *
 * @return      None
*/
void wiced_bt_mesh_model_onoff_server_send_status(wiced_bt_mesh_event_t *p_event, wiced_bt_mesh_onoff_status_data_t *p_data);

/* @} wiced_bt_mesh_onoff_server */

/**
 * @addtogroup  wiced_bt_mesh_onoff_client        Mesh OnOff Client
 * @ingroup     wiced_bt_mesh_models
 *
 * The WICED Mesh OnOff Client module provides a way for an application to send OnOff 
 * commands to the server and receive status information.
 *
 * On startup the application should call wiced_bt_mesh_model_onoff_client_init 
 * function providing the callback that will be called when OnOff status message 
 * is received. The application can call the appropriate Get or Set
 * function to retrieve or set the OnOff state for a specific mesh node.
 */

/**
 * \brief On/Off Client callback is called by the Mesh Models library on receiving a message from the peer
 *
 * @param       event that application should process (see @ref ONOFF_EVENT "Mesh On/Off Events")
 * @param       p_event information about the message received.  The same pointer should be used in the reply if required.
 * @param       p_data pointer to the data to send
 *
 * @return      None
 */
typedef void(wiced_bt_mesh_onoff_client_callback_t)(uint16_t event, wiced_bt_mesh_event_t *p_event, void *p_data);

/**
 * \brief On/Off Client Model initialization
 *
 * @param       element_idx Device element to where model is used
 * @param       p_callback The application callback function that will be executed by the mesh models library when application action is required, or when a reply for the application request has been received.
 * @param       is_provisioned If TRUE, the application is being restarted after being provisioned or after a power loss. If FALSE the model cleans up NVRAM on startup.
 *
 * @return      None
 */
void wiced_bt_mesh_model_onoff_client_init(uint8_t element_idx, wiced_bt_mesh_onoff_client_callback_t *p_callback, wiced_bool_t is_provisioned);

/**
 * \brief On/Off Client Message Handler
 * \details The mesh core library calls this function for each message received.  The function returns WICED_TRUE if the message is destined for this specific model and successfully processed, and returns WICED_FALSE otherwise. 
 *
 * @param       p_event Mesh event with information about received message.
 * @param       p_data Pointer to the data portion of the message
 * @param       data_len Length of the data in the message
 *
 * @return      WICED_TRUE if the message is for this company ID/Model/Element Index combination, WICED_FALSE otherwise.
*/
wiced_bool_t wiced_bt_mesh_model_onoff_client_message_handler(wiced_bt_mesh_event_t *p_event, uint8_t *p_data, uint16_t data_len);


/**
 * \brief The application can call this function to send get On/Off state client message to the server.
 *
 * @param       p_event Mesh event with the information about the message that has been created by the app for unsolicited message.
 *
 * @return      None
 */
void wiced_bt_mesh_model_onoff_client_send_get(wiced_bt_mesh_event_t *p_event);

/**
 * \brief The application can call this function to send Set OnOff client message to the server.
 *
 * @param       p_event Mesh event with the information about the message that has been created by the app for unsolicited message.
 * @param       p_data Pointer to the data to send
 *
 * @return      None
 */
void wiced_bt_mesh_model_onoff_client_send_set(wiced_bt_mesh_event_t *p_event, wiced_bt_mesh_onoff_set_data_t* p_data);

/* @} wiced_bt_mesh_onoff_client */

/**
 * \brief Generic Level Server callback is called by the Mesh Models library on receiving a message from the peer
 *
 * @param       event that application should process (see @ref LEVEL_EVENT "Level Events")
 * @param       p_event information about the message received.  The same pointer should be used in the reply if required.
 * @param       p_data pointer to the on/off state data
 *
 * @return      None
 */
typedef void(wiced_bt_mesh_level_server_callback_t)(uint16_t event, wiced_bt_mesh_event_t *p_event, void *p_data);

/**
 * \brief Level Model initialization
 *
 * @param       element_idx Device element to where model is used
 * @param       p_callback Application or Parent model callback that will be executed on receiving a message
 * @param       is_provisioned If TRUE, the application is being restarted after being provisioned or after a power loss. If FALSE the model cleans up NVRAM on startup.
 *
 * @return      None
 */
void wiced_bt_mesh_model_level_server_init(uint8_t element_idx, wiced_bt_mesh_level_server_callback_t *p_callback, wiced_bool_t is_provisioned);

/**
 * \brief Level Server Message Handler
 * \details The mesh core library calls this function for each message received.  The function returns WICED_TRUE if the message is destined for this specific model and successfully processed, and returns WICED_FALSE otherwise.
 *
 * @param       p_event Mesh event with information about received message.
 * @param       p_data Pointer to the data portion of the message
 * @param       data_len Length of the data in the message
 *
 * @return      WICED_TRUE if the message is for this company ID/Model/Element Index combination, WICED_FALSE otherwise.
*/
wiced_bool_t wiced_bt_mesh_model_level_server_message_handler(wiced_bt_mesh_event_t *p_event, uint8_t *p_data, uint16_t data_len);

/**
 * \brief Level Server Scene Store Handler
 * \details The mesh Scene Server calls this function so that the module can store required data for the scene.
 *
 * @param       element_idx Element index for which scene information is being stored
 * @param       p_buffer Pointer to a buffer where data should be stored
 * @param       buf_size Maximum amount of data a model can store
 *
 * @return      Number of bytes the model wrote into the provided buffer
 */
uint16_t wiced_bt_mesh_model_level_server_scene_store_handler(uint8_t element_idx, uint8_t *p_buffer, uint16_t buffer_len);

/**
 * \brief Level Server Scene Store Handler
 * \details When Scene Server receives Scene Recall message it calls this function with data previously stored in the NVRAM.
 *
 * @param       element_idx Element index for which scene information is being recalled
 * @param       p_buffer Pointer to a buffer where model's data is located
 * @param       buf_size Maximum amount of data a model can get from the buffer
 *
 * @return      Number of bytes the model read from the provided buffer
 */
uint16_t wiced_bt_mesh_model_level_server_scene_recall_handler(uint8_t element_idx, uint8_t *p_buffer, uint16_t buffer_len, uint32_t transition_time, uint32_t delay);

/**
 * \brief An application or a a parent model can call this function to send status message to the Level Client. The mesh event should
 * contain information if this is a reply or an unsolicited message.
 *
 * @param       p_event Mesh event with the information about the message that has been received in the request, or created by the app for unsolicited message.
 * @param       p_data Pointer to the status data
 *
 * @return      None
*/
void wiced_bt_mesh_model_level_server_send_status(wiced_bt_mesh_event_t *p_event, wiced_bt_mesh_level_status_data_t *p_data);

/**
 * \brief Level Client callback is called by the Mesh Models library on receiving a message from the peer
 *
 * @param       event that application should process (see @ref LEVEL_EVENT "Mesh Level Events")
 * @param       p_event information about the message received.  The same pointer should be used in the reply if required.
 * @param       p_data pointer to the level data
 *
 * @return      None
 */
typedef void(wiced_bt_mesh_level_client_callback_t)(uint16_t event, wiced_bt_mesh_event_t *p_event, void *p_data);

/**
 * \brief Generic Level Client Message Handler
 * \details The mesh core library calls this function for each message received.  The function returns WICED_TRUE if the message is destined for this specific model and successfully processed, and returns WICED_FALSE otherwise. Generic Level Server device.
 * The function parses the message and if appropriate calls the parent back to perform functionality.
 *
 * @param       p_event Mesh event with information about received message.
 * @param       p_data Pointer to the data portion of the message
 * @param       data_len Length of the data in the message
 *
 * @return      WICED_TRUE if the message is for this company ID/Model/Element Index combination, WICED_FALSE otherwise.
 */
wiced_bool_t wiced_bt_mesh_model_level_client_message_handler(wiced_bt_mesh_event_t *p_event, uint8_t *p_data, uint16_t data_len);

/**
 * \brief Level Client Model initialization
 *
 * @param       element_idx Device element to where model is used
 * @param       p_callback The application callback function that will be executed by the mesh models library when application action is required, or when a reply for the application request has been received.
 * @param       is_provisioned If TRUE, the application is being restarted after being provisioned or after a power loss. If FALSE the model cleans up NVRAM on startup.
 *
 * @return      None
 */
void wiced_bt_mesh_model_level_client_init(uint8_t element_idx, void *p_callback, wiced_bool_t is_provisioned);

/**
 * \brief The application can call this function to send Generic Level Get message to the server.
 *
 * @param       p_event Mesh event with the information about the message that has been created by the app for unsolicited message.
 *
 * @return      None
 */
void wiced_bt_mesh_model_level_client_send_get(wiced_bt_mesh_event_t *p_event);

/**
 * \brief The application can call this function to send Set Level client message to the server.
 *
 * @param       p_event Mesh event with the information about the message that has been created by the app for unsolicited message.
 * @param       p_data Pointer to the data to send
 *
 * @return      None
 */
void wiced_bt_mesh_model_level_client_send_set(wiced_bt_mesh_event_t *p_event, wiced_bt_mesh_level_set_level_t* p_data);

/**
 * \brief The application can call this function to send Set Delta Level client message to the server.
 *
 * @param       p_event Mesh event with the information about the message that has been created by the app for unsolicited message.
 * @param       p_data Pointer to the data to send
 *
 * @return      None
 */
void wiced_bt_mesh_model_level_client_send_delta_set(wiced_bt_mesh_event_t *p_event, wiced_bt_mesh_level_set_delta_t* p_data);

/**
 * \brief The application can call this function to send Set Move client message to the server.
 *
 * @param       p_event Mesh event with the information about the message that has been created by the app for unsolicited message.
 * @param       p_data Pointer to the data to send
 *
 * @return      None
 */
void wiced_bt_mesh_model_level_client_send_move_set(wiced_bt_mesh_event_t *p_event, wiced_bt_mesh_level_set_move_t* p_data);

/**
 * \brief Generic Default Transition Time Server callback is called by the Mesh Models library on receiving a message from the peer
 *
 * @param       event that application should process (see @ref DEFAULT_TRANSITION_TIME_EVENT "Default Transition Time Events")
 * @param       p_event information about the message received.  The same pointer should be used in the reply if required.
 * @param       p_data pointer to the on/off state data
 *
 * @return      None
 */
typedef void(wiced_bt_mesh_default_transition_time_server_callback_t)(uint16_t event, wiced_bt_mesh_event_t *p_event, void *wiced_bt_mesh_default_transition_time_data_t);

/**
 * \brief Generic Default Transition Time Model initialization
 * \details An application should initialize default transition time model for each element which supports any of the server models that have transitions.
 * The callback that is passed as a parameter will be executed if the provisioner changes the value of the default transition time.
 *
 * @param       element_idx Device element to where model is used
 * @param       p_callback The application callback function that will be executed by the mesh models library when application action is required, or when a reply for the application request has been received.
 * @param       is_provisioned If TRUE, the application is being restarted after being provisioned or after a power loss. If FALSE the model cleans up NVRAM on startup.
 *
 * @return      None
 */
void wiced_bt_mesh_model_default_transition_time_server_init(uint8_t element_idx, wiced_bt_mesh_default_transition_time_server_callback_t *p_callback, wiced_bool_t is_provisioned);

/**
 * \brief Default Transition Time Server Message Handler
 * \details The mesh core library calls this function for each message received.  
 * The function returns WICED_TRUE if the message is destined for this specific 
 * model and successfully processed, and returns WICED_FALSE otherwise.
 *
 * @param       p_event Mesh event with information about received message.
 * @param       p_data Pointer to the data portion of the message
 * @param       data_len Length of the data in the message
 *
 * @return      WICED_TRUE if the message is for this company ID/Model/Element Index combination, WICED_FALSE otherwise.
*/
wiced_bool_t wiced_bt_mesh_model_default_transition_time_server_message_handler(wiced_bt_mesh_event_t *p_event, uint8_t *p_data, uint16_t data_len);

/**
 * \brief An application or a a parent model can call this function to send status message to the Default Transition Time Client. The mesh event should
 * contain information if this is a reply or an unsolicited message.
 *
 * @param       p_event Mesh event with the information about the message that has been received in the request, or created by the app for unsolicited message.
 * @param       p_data Pointer to the status data
 *
 * @return      None
*/
void wiced_bt_mesh_model_default_transition_time_server_send_status(wiced_bt_mesh_event_t *p_event, wiced_bt_mesh_default_transition_time_data_t *p_data);


/**
 * \brief Default Transition Time Client callback is called by the Mesh Models library on receiving a message from the peer
 *
 * @param       event that application should process (see @ref DEFAULT_TRANSITION_TIME_EVENT "Mesh Default Transition Time Events")
 * @param       p_event information about the message received.  The same pointer should be used in the reply if required.
 * @param       p_data pointer to the data to send
 *
 * @return      None
 */
typedef void(wiced_bt_mesh_default_transition_time_client_callback_t)(uint16_t event, wiced_bt_mesh_event_t *p_event, void *p_data);

/**
 * \brief Default Transition Time Client Model initialization
 *
 * @param       element_idx Device element to where model is used
 * @param       p_callback The application callback function that will be executed by the mesh models library when application action is required, or when a reply for the application request has been received.
 * @param       is_provisioned If TRUE, the application is being restarted after being provisioned or after a power loss. If FALSE the model cleans up NVRAM on startup.
 *
 * @return      None
 */
void wiced_bt_mesh_model_default_transition_time_client_init(uint8_t element_idx, wiced_bt_mesh_default_transition_time_client_callback_t *p_callback, wiced_bool_t is_provisioned);

/**
* \brief Default Transition Time Client Message Handler
* \details The mesh core library calls this function for each message received.  The function returns WICED_TRUE if the message is destined for this specific model and successfully processed, and returns WICED_FALSE otherwise. Default Transition Time Server device.
* The function parses the message and if appropriate calls the parent back to perform functionality.
*
* @param       p_event Mesh event with information about received message.
* @param       p_data Pointer to the data portion of the message
* @param       data_len Length of the data in the message
*
* @return      WICED_TRUE if the message is for this company ID/Model/Element Index combination, WICED_FALSE otherwise.
*/
wiced_bool_t wiced_bt_mesh_model_default_transition_time_client_message_handler(wiced_bt_mesh_event_t *p_event, uint8_t *p_data, uint16_t data_len);

/**
 * \brief The application can call this function to send get Default Transition Time client message to the server.
 *
 * @param       p_event Mesh event with the information about the message that has been created by the app for unsolicited message.
 *
 * @return      None
 */
void wiced_bt_mesh_model_default_transition_time_client_send_get(wiced_bt_mesh_event_t *p_event);

/**
 * \brief The application can call this function to send Set Default Transition Time client message to the server.
 *
 * @param       p_event Mesh event with the information about the message that has been created by the application for unsolicited message.
 * @param       p_data Pointer to the data to send
 *
 * @return      None
 */
void wiced_bt_mesh_model_default_transition_time_client_send_set(wiced_bt_mesh_event_t *p_event, wiced_bt_mesh_default_transition_time_data_t* p_data);

/**
 * \brief Generic Power OnOff Server callback is executed by the Mesh Models library on receiving a message from the peer
 *
 * @param       event that application should process (see @ref POWER_ONOFF_EVENT "Power OnOff Events")
 * @param       p_event information about the message received.  The same pointer should be used in the reply if required.
 * @param       p_data pointer to the on/off state data
 *
 * @return      None
 */
typedef void(wiced_bt_mesh_power_onoff_server_callback_t)(uint16_t event, wiced_bt_mesh_event_t *p_event, void *p_data);

/**
 * \brief Generic Power OnOff Model initialization
 * \details An application or the model extending Power OnOff model should initialize an instance of the Power OnOff model for each element.
 *
 * @param       element_idx Device element to where model is used
 * @param       p_callback The application callback function that will be executed by the mesh models library when application action is required, or when a reply for the application request has been received.
 * @param       is_provisioned If TRUE, the application is being restarted after being provisioned or after a power loss. If FALSE the model cleans up NVRAM on startup.
 *
 * @return      None
 */
void wiced_bt_mesh_model_power_onoff_server_init(uint8_t element_idx, wiced_bt_mesh_power_onoff_server_callback_t *p_callback, wiced_bool_t is_provisioned);

/**
 * \brief Generic Power OnOff Server Message Handler
 * \details The mesh core library calls this function for each message received.  The function returns WICED_TRUE if the message is destined for this specific model and successfully processed, and returns WICED_FALSE otherwise.
 *
 * @param       p_event Mesh event with information about received message.
 * @param       p_data Pointer to the data portion of the message
 * @param       data_len Length of the data in the message
 *
 * @return      WICED_TRUE if the message is for this company ID/Model/Element Index combination, WICED_FALSE otherwise.
*/
wiced_bool_t wiced_bt_mesh_model_power_onoff_server_message_handler(wiced_bt_mesh_event_t *p_event, uint8_t *p_data, uint16_t data_len);

/**
 * \brief An application or a a parent model can call this function to send status message to the Default Transition Time Client. The mesh event should
 * contain information if this is a reply or an unsolicited message.
 *
 * @param       p_event Mesh event with the information about the message that has been received in the request, or created by the app for unsolicited message.
 * @param       p_data Pointer to the status data
 *
 * @return      None
*/
void wiced_bt_mesh_model_power_onoff_server_send_onpowerup_status(wiced_bt_mesh_event_t *p_event, wiced_bt_mesh_power_onoff_data_t *p_data);

/**
 * \brief An application or a a parent model can call this function to send On/Off status message to the Power On/Off Client. The mesh event should
 * contain information if this is a reply or an unsolicited message.
 *
 * @param       p_event Mesh event with the information about the message that has been received in the request, or created by the app for unsolicited message.
 * @param       p_data Pointer to the status data
 *
 * @return      None
 */
void wiced_bt_mesh_model_power_onoff_server_send_onoff_status(wiced_bt_mesh_event_t *p_event, wiced_bt_mesh_onoff_status_data_t *p_data);


/**
 * \brief Power On/Off Client callback is called by the Mesh Models library on receiving a message from the peer
 *
 * @param       event that application should process (see @ref POWER_ONOFF_EVENT "Mesh Power On/Off Events")
 * @param       p_event information about the message received.  The same pointer should be used in the reply if required.
 * @param       p_data pointer to the data to send
 *
 * @return      None
 */
typedef void(wiced_bt_mesh_power_onoff_client_callback_t)(uint16_t event, wiced_bt_mesh_event_t *p_event, void *p_data);

/**
 * \brief Power On/Off Client Model initialization
 *
 * @param       element_idx Device element to where model is used
 * @param       p_callback The application callback function that will be executed by the mesh models library when application action is required, or when a reply for the application request has been received.
 * @param       is_provisioned If TRUE, the application is being restarted after being provisioned or after a power loss. If FALSE the model cleans up NVRAM on startup.
 *
 * @return      None
 */
void wiced_bt_mesh_model_power_onoff_client_init(uint8_t element_idx, wiced_bt_mesh_power_onoff_client_callback_t *p_callback, wiced_bool_t is_provisioned);

/**
 * \brief Generic Power OnOff Client Message Handler
 * \details The mesh core library calls this function for each message received.  The function returns WICED_TRUE if the message is destined for this specific model and successfully processed, and returns WICED_FALSE otherwise.
 *
 * @param       p_event Mesh event with information about received message.
 * @param       p_data Pointer to the data portion of the message
 * @param       data_len Length of the data in the message
 *
 * @return      WICED_TRUE if the message is for this company ID/Model/Element Index combination, WICED_FALSE otherwise.
*/
wiced_bool_t wiced_bt_mesh_model_power_onoff_client_message_handler(wiced_bt_mesh_event_t *p_event, uint8_t *p_data, uint16_t data_len);


/**
 * \brief The application can call this function to send the Get Power On/Off state client message to the server.
 *
 * @param       p_event Mesh event with the information about the message that has been created by the app for unsolicited message.
 *
 * @return      None
 */
void wiced_bt_mesh_model_power_onoff_client_send_onpowerup_get(wiced_bt_mesh_event_t *p_event);

/**
 * \brief The application can call this function to send Set OnOff client message to the server.
 *
 * @param       p_event Mesh event with the information about the message that has been created by the app for unsolicited message.
 * @param       p_data Pointer to the data to send
 *
 * @return      None
 */
void wiced_bt_mesh_model_power_onoff_client_send_onpowerup_set(wiced_bt_mesh_event_t *p_event, wiced_bt_mesh_power_onoff_data_t* p_data);

/**
 * \brief Generic Power OnOff Setup Server callback is executed by the Mesh Models library on receiving a message from the peer
 *
 * @param       event that application should process (see @ref POWER_ONOFF_EVENT "Power OnOff Events")
 * @param       p_event information about the message received.  The same pointer should be used in the reply if required.
 * @param       p_data pointer to the on/off state data
 *
 * @return      None
 */
typedef void(wiced_bt_mesh_power_onoff_setup_server_callback_t)(uint16_t event, wiced_bt_mesh_event_t *p_event, void *p_data);

/**
 * \brief Generic Power OnOff Setup Model initialization
 * \details An application or the model extending Power OnOff model should initialize an instance of the Power OnOff model for each element.
 *
 * @param       element_idx Device element to where model is used
 * @param       p_callback The application callback function that will be executed by the mesh models library when application action is required, or when a reply for the application request has been received.
 * @param       is_provisioned If TRUE, the application is being restarted after being provisioned or after a power loss. If FALSE the model cleans up NVRAM on startup.
 *
 * @return      None
 */
void wiced_bt_mesh_model_power_onoff_setup_server_init(uint8_t element_idx, wiced_bt_mesh_power_onoff_server_callback_t *p_callback, wiced_bool_t is_provisioned);

/**
 * \brief Generic Power OnOff Setup Server Message Handler
 * \details The mesh core library calls this function for each message received.  The function returns WICED_TRUE if the message is destined for this specific model and successfully processed, and returns WICED_FALSE otherwise.
 *
 * @param       p_event Mesh event with information about received message.
 * @param       p_data Pointer to the data portion of the message
 * @param       data_len Length of the data in the message
 *
 * @return      WICED_TRUE if the message is for this company ID/Model/Element Index combination, WICED_FALSE otherwise.
*/
wiced_bool_t wiced_bt_mesh_model_power_onoff_setup_server_message_handler(wiced_bt_mesh_event_t *p_event, uint8_t *p_data, uint16_t data_len);


/**
 * \brief Generic Power Level Server callback is called by the Mesh Models library on receiving a message from the peer
 * Application or higher level model should be able process messages for the power level server model as well as
 * bound models including generic on/off and generic level.
 *
 * @param       event that the application should process (see @ref POWER_LEVEL_EVENT "Power Level Events")
 * @param       p_event information about the message received.  The same pointer should be used in the reply if required.
 * @param       p_data pointer to the on/off state data
 *
 * @return      None
 */
typedef void(wiced_bt_mesh_power_level_server_callback_t)(uint16_t event, wiced_bt_mesh_event_t *p_event, void *p_data);

/**
 * \brief Power Level Model initialization
 * \details A Mesh application which contains Power Level Server should call this function for each element where the Power Level Server is present
 *
 * @param       element_idx Device element to where model is used
 * @param       p_callback Application or Parent model callback that will be executed on receiving a message
 * @param       is_provisioned If TRUE, the application is being restarted after being provisioned or after a power loss. If FALSE the model cleans up NVRAM on startup.
 *
 * @return      WICED_TRUE if initialization was successful
 */
void wiced_bt_mesh_model_power_level_server_init(uint8_t element_idx, wiced_bt_mesh_power_level_server_callback_t *p_callback, wiced_bool_t is_provisioned);

/**
 * \brief Power Level Server Message Handler
 * \details The mesh core library calls this function for each message received.  The function returns WICED_TRUE if the message is destined for this specific model and successfully processed, and returns WICED_FALSE otherwise.
 *
 * @param       p_event Mesh event with information about received message.
 * @param       p_data Pointer to the data portion of the message
 * @param       data_len Length of the data in the message
 *
 * @return      WICED_TRUE if the message is for this company ID/Model/Element Index combination, WICED_FALSE otherwise.
*/
wiced_bool_t wiced_bt_mesh_model_power_level_server_message_handler(wiced_bt_mesh_event_t *p_event, uint8_t *p_data, uint16_t data_len);

/**
 * \brief An application or a a parent model can call this function to send status message to the Power Level Client. The mesh event should
 * contain information if this is a reply or an unsolicited message.
 *
 * @param       p_event Mesh event with the information about the message that has been received in the request, or created by the app for unsolicited message.
 * @param       p_data Pointer to the status data
 *
 * @return      None
*/
void wiced_bt_mesh_model_power_level_server_send_status(wiced_bt_mesh_event_t *p_event, wiced_bt_mesh_power_level_status_data_t *p_data);

/**
 * \brief An application or a a parent model can call this function to send default power status message to the Power Level Client. The mesh event should
 * contain information if this is a reply or an unsolicited message.
 *
 * @param       p_event Mesh event with the information about the message that has been received in the request, or created by the app for unsolicited message.
 * @param       p_data Pointer to the status data
 *
 * @return      None
*/
void wiced_bt_mesh_model_power_level_server_send_default_status(wiced_bt_mesh_event_t *p_event, wiced_bt_mesh_power_default_data_t *p_data);

/**
 * \brief An application or a a parent model can call this function to send range power status message to the Power Level Client. The mesh event should
 * contain information if this is a reply or an unsolicited message.
 *
 * @param       p_event Mesh event with the information about the message that has been received in the request, or created by the app for unsolicited message.
 * @param       p_data Pointer to the status data
 *
 * @return      None
*/
void wiced_bt_mesh_model_power_level_server_send_range_status(wiced_bt_mesh_event_t *p_event, wiced_bt_mesh_power_range_status_data_t *p_data);

/**
 * \brief Generic Power Level Setup Server callback is called by the Mesh Models library on receiving a message from the peer
 * Application or higher level model should be able process messages for the Power Level Setup Server model as well as
 * bound models including Power Level Server which is in turn need to process messages for Generic On/Off and Generic Level models.
 *
 * @param       event that application should process (see @ref POWER_LEVEL_EVENT "Power Level Events, @ref ONOFF_EVENT "On/Off Events, @ref LEVEL_EVENT "Level Events")
 * @param       p_event information about the message received.  The same pointer should be used in the reply if required.
 * @param       p_data pointer to the on/off state data
 *
 * @return      None
 */
typedef void(wiced_bt_mesh_power_level_setup_server_callback_t)(uint16_t event, wiced_bt_mesh_event_t *p_event, void *p_data);

/**
 * \brief Power Level Setup Model initialization
 * \details A Mesh application which contains Power Level Setup Server should call this function for each element where the Power Level Setup Server is present
 *
 * @param       element_idx Device element to where model is used
 * @param       p_callback Application or Parent model callback that will be executed on receiving a message
 * @param       is_provisioned If TRUE, the application is being restarted after being provisioned or after a power loss. If FALSE the model cleans up NVRAM on startup.
 *
 * @return      WICED_TRUE if initialization was successful
 */
void wiced_bt_mesh_model_power_level_setup_server_init(uint8_t element_idx, wiced_bt_mesh_power_level_setup_server_callback_t *p_callback, wiced_bool_t is_provisioned);

/**
 * \brief Power Level Setup Server Message Handler
 * \details The mesh core library calls this function for each message received.  The function returns WICED_TRUE if the message is destined for this specific model and successfully processed, and returns WICED_FALSE otherwise.
 *
 * @param       p_event Mesh event with information about received message.
 * @param       p_data Pointer to the data portion of the message
 * @param       data_len Length of the data in the message
 *
 * @return      WICED_TRUE if the message is for this company ID/Model/Element Index combination, WICED_FALSE otherwise.
*/
wiced_bool_t wiced_bt_mesh_model_power_level_setup_server_message_handler(wiced_bt_mesh_event_t *p_event, uint8_t *p_data, uint16_t data_len);

/**
 * \brief Power Level Server Scene Store Handler
 * \details The mesh Scene Server calls this function so that the module can store required data for the scene.
 *
 * @param       element_idx Element index for which scene information is being stored
 * @param       p_buffer Pointer to a buffer where data should be stored
 * @param       buf_size Maximum amount of data a model can store
 *
 * @return      Number of bytes the model wrote into the provided buffer
 */
uint16_t wiced_bt_mesh_model_power_level_server_scene_store_handler(uint8_t element_idx, uint8_t *p_buffer, uint16_t buffer_len);

/**
 * \brief Power Level Server Scene Store Handler
 * \details When Scene Server receives Scene Recall message it calls this function with data previously stored in the NVRAM.
 *
 * @param       element_idx Element index for which scene information is being recalled
 * @param       p_buffer Pointer to a buffer where model's data is located
 * @param       buf_size Maximum amount of data a model can get from the buffer
 *
 * @return      Number of bytes the model read from the provided buffer
 */
uint16_t wiced_bt_mesh_model_power_level_server_scene_recall_handler(uint8_t element_idx, uint8_t *p_buffer, uint16_t buffer_len, uint32_t transition_time, uint32_t delay);

/**
 * \brief Power Level Client callback is called by the Mesh Models library on receiving a message from the peer
 *
 * @param       event that application should process (see @ref POWER_LEVEL_EVENT "Power Level Events")
 * @param       p_event information about the message received.  The same pointer should be used in the reply if required.
 * @param       p_data pointer to the level data
 *
 * @return      None
 */
typedef void(wiced_bt_mesh_power_level_client_callback_t)(uint16_t event, wiced_bt_mesh_event_t *p_event, void *p_data);

/**
* \brief Generic Power Level Client Message Handler
* \details The mesh core library calls this function for each message received.  The function returns WICED_TRUE if the message is destined for this specific model and successfully processed, and returns WICED_FALSE otherwise. Generic Power Level Server device.
* The function parses the message and if appropriate calls the parent back to perform functionality.
*
* @param       p_event Mesh event with information about received message.
* @param       p_data Pointer to the data portion of the message
* @param       data_len Length of the data in the message
*
* @return      WICED_TRUE if the message is for this company ID/Model/Element Index combination, WICED_FALSE otherwise.
*/
wiced_bool_t wiced_bt_mesh_model_power_level_client_message_handler(wiced_bt_mesh_event_t *p_event, uint8_t *p_data, uint16_t data_len);

/**
 * \brief Power Level Client Model initialization
 *
 * @param       element_idx Device element to where model is used
 * @param       p_callback The application callback function that will be executed by the mesh models library when application action is required, or when a reply for the application request has been received.
 * @param       is_provisioned If TRUE, the application is being restarted after being provisioned or after a power loss. If FALSE the model cleans up NVRAM on startup.
 *
 * @return      None
 */
void wiced_bt_mesh_model_power_level_client_init(uint8_t element_idx, wiced_bt_mesh_power_level_client_callback_t *p_callback, wiced_bool_t is_provisioned);

/**
 * \brief The application can call this function to send Generic Power Level Get message to the server.
 *
 * @param       p_event Mesh event with the information about the message that has been created by the app for unsolicited message.
 *
 * @return      None
 */
void wiced_bt_mesh_model_power_level_client_send_get(wiced_bt_mesh_event_t *p_event);

/**
 * \brief The application can call this function to send Set Level client message to the server.
 *
 * @param       p_event Mesh event with the information about the message that has been created by the app for unsolicited message.
 * @param       p_data Pointer to the data to send
 *
 * @return      None
 */
void wiced_bt_mesh_model_power_level_client_send_set(wiced_bt_mesh_event_t *p_event, wiced_bt_mesh_power_level_set_level_t* p_data);

/**
 * \brief The application can call this function to send Get Last Level client message to the server.
 *
 * @param       p_event Mesh event with the information about the message that has been created by the app for unsolicited message.
 *
 * @return      None
 */
void wiced_bt_mesh_model_power_level_client_send_last_get(wiced_bt_mesh_event_t *p_event);

/**
 * \brief The application can call this function to send Get Default Level client message to the server.
 *
 * @param       p_event Mesh event with the information about the message that has been created by the app for unsolicited message.
 *
 * @return      None
 */
void wiced_bt_mesh_model_power_level_client_send_default_get(wiced_bt_mesh_event_t *p_event);

/**
 * \brief The application can call this function to send Set Default Level client message to the server.
 *
 * @param       p_event Mesh event with the information about the message that has been created by the app for unsolicited message.
 * @param       p_data Pointer to the data to send
 *
 * @return      None
 */
void wiced_bt_mesh_model_power_level_client_send_default_set(wiced_bt_mesh_event_t *p_event, wiced_bt_mesh_power_default_data_t* p_data);

/**
 * \brief The application can call this function to send Get Range Level client message to the server.
 *
 * @param       p_event Mesh event with the information about the message that has been created by the app for unsolicited message.
 *
 * @return      None
 */
void wiced_bt_mesh_model_power_level_client_send_range_get(wiced_bt_mesh_event_t *p_event);

/**
 * \brief The application can call this function to send Set Range Level client message to the server.
 *
 * @param       p_event Mesh event with the information about the message that has been created by the app for unsolicited message.
 * @param       p_data Pointer to the data to send
 *
 * @return      None
 */
void wiced_bt_mesh_model_power_level_client_send_range_set(wiced_bt_mesh_event_t *p_event, wiced_bt_mesh_power_level_range_set_data_t* p_data);

/**
 * \brief Generic User Property Server callback is called by the Mesh Models library on receiving a message from the peer
 *
 * @param       event that application should process (see @ref USER_PROPERTY_EVENT "On/Off Events")
 * @param       p_event information about the message received.  The same pointer should be used in the reply if required.
 * @param       p_data pointer to the on/off state data
 *
 * @return      None
 */
typedef void(wiced_bt_mesh_property_server_callback_t)(uint16_t event, wiced_bt_mesh_event_t *p_event, void *p_data);

/**
 * \brief Property Model initialization
 *
 * @param       element_idx Device element to where model is used
 * @param       p_callback Application or Parent model callback that will be executed on receiving a message
 * @param       is_provisioned If TRUE, the application is being restarted after being provisioned or after a power loss. If FALSE the model cleans up NVRAM on startup.
 *
 * @return      None
 */
void wiced_bt_mesh_model_property_server_init(uint8_t element_idx, wiced_bt_mesh_property_server_callback_t *p_callback, wiced_bool_t is_provisioned);

/**
 * \brief User Property Server Message Handler
 * \details The mesh core library calls this function for each message received.  The function returns WICED_TRUE if the message is destined for this specific model and successfully processed, and returns WICED_FALSE otherwise.
 *
 * @param       p_event Mesh event with information about received message.
 * @param       p_data Pointer to the data portion of the message
 * @param       data_len Length of the data in the message
 *
 * @return      WICED_TRUE if the message is for this company ID/Model/Element Index combination, WICED_FALSE otherwise.
*/
wiced_bool_t wiced_bt_mesh_model_user_property_server_message_handler(wiced_bt_mesh_event_t *p_event, uint8_t *p_data, uint16_t data_len);

/**
 * \brief Admin Property Server Message Handler
 * \details The mesh core library calls this function for each message received.  The function returns WICED_TRUE if the message is destined for this specific model and successfully processed, and returns WICED_FALSE otherwise.
 *
 * @param       p_event Mesh event with information about received message.
 * @param       p_data Pointer to the data portion of the message
 * @param       data_len Length of the data in the message
 *
 * @return      WICED_TRUE if the message is for this company ID/Model/Element Index combination, WICED_FALSE otherwise.
*/
wiced_bool_t wiced_bt_mesh_model_admin_property_server_message_handler(wiced_bt_mesh_event_t *p_event, uint8_t *p_data, uint16_t data_len);

/**
 * \brief Manufacturer Property Server Message Handler
 * \details The mesh core library calls this function for each message received.  The function returns WICED_TRUE if the message is destined for this specific model and successfully processed, and returns WICED_FALSE otherwise.
 *
 * @param       p_event Mesh event with information about received message.
 * @param       p_data Pointer to the data portion of the message
 * @param       data_len Length of the data in the message
 *
 * @return      WICED_TRUE if the message is for this company ID/Model/Element Index combination, WICED_FALSE otherwise.
*/
wiced_bool_t wiced_bt_mesh_model_manufacturer_property_server_message_handler(wiced_bt_mesh_event_t *p_event, uint8_t *p_data, uint16_t data_len);

/**
 * \brief Client Property Server Message Handler
 * \details The mesh core library calls this function for each message received.  The function returns WICED_TRUE if the message is destined for this specific model and successfully processed, and returns WICED_FALSE otherwise.
 *
 * @param       p_event Mesh event with information about received message.
 * @param       p_data Pointer to the data portion of the message
 * @param       data_len Length of the data in the message
 *
 * @return      WICED_TRUE if the message is for this company ID/Model/Element Index combination, WICED_FALSE otherwise.
*/
wiced_bool_t wiced_bt_mesh_model_client_property_server_message_handler(wiced_bt_mesh_event_t *p_event, uint8_t *p_data, uint16_t data_len);

/*
 * Application can call this function to send the Properties Status message to the Client. The mesh event should
 * contain information if this is a reply or an unsolicited message.
 *
 * @param       p_event information about the message to be set.
 * @param       p_data pointer to the data to send
 *
 * @return      None
 */
void wiced_bt_mesh_model_property_server_send_properties_status(wiced_bt_mesh_event_t *p_event, wiced_bt_mesh_properties_status_data_t *p_data);

/*
 * Application can call this function to send the Property Status message to the Client. The mesh event should
 * contain information if this is a reply or an unsolicited message.  The wiced_bt_mesh_property_status_data_t contains
 * the type, ID and the value of the property
 *
 * @param       p_event information about the message to be set.
 * @param       type of the property requested
 * @param       p_data pointer to the property configuration
 *
 * @return      None
 */
void wiced_bt_mesh_model_property_server_send_property_status(wiced_bt_mesh_event_t *p_event, uint8_t type, wiced_bt_mesh_core_config_property_t *p_property);

/**
 * \brief Property Client callback is called by the Mesh Models library on receiving a message from the peer
 *
 * @param       event that application should process (see @ref PROPERTY_EVENT "Mesh Property Events")
 * @param       p_event information about the message received.  The same pointer should be used in the reply if required.
 * @param       p_data pointer to the data to send
 *
 * @return      None
 */
typedef void(wiced_bt_mesh_property_client_callback_t)(uint16_t event, wiced_bt_mesh_event_t *p_event, void *p_data);

/**
 * \brief Property Client Model initialization
 *
 * @param       element_idx Device element to where model is used
 * @param       p_callback The application callback function that will be executed by the mesh models library when application action is required, or when a reply for the application request has been received.
 * @param       is_provisioned If TRUE, the application is being restarted after being provisioned or after a power loss. If FALSE the model cleans up NVRAM on startup.
 *
 * @return      None
 */
void wiced_bt_mesh_model_property_client_init(uint8_t element_idx, wiced_bt_mesh_property_client_callback_t *p_callback, wiced_bool_t is_provisioned);

/**
 * \brief Property Client Message Handler
 * \details The mesh core library calls this function for each message received.  The function returns WICED_TRUE if the message is destined for this specific model and successfully processed, and returns WICED_FALSE otherwise.
 *
 * @param       p_event Mesh event with information about received message.
 * @param       p_data Pointer to the data portion of the message
 * @param       data_len Length of the data in the message
 *
 * @return      WICED_TRUE if the message is for this company ID/Model/Element Index combination, WICED_FALSE otherwise.
*/
wiced_bool_t wiced_bt_mesh_model_property_client_message_handler(wiced_bt_mesh_event_t *p_event, uint8_t *p_data, uint16_t data_len);

/**
 * \brief The application can call this function to send Properties Get client message to the server.
 *
 * @param       p_event Mesh event with the information about the message that has been created by the app for unsolicited message.
 *
 * @return      None
 */
void wiced_bt_mesh_model_property_client_send_properties_get(wiced_bt_mesh_event_t *p_event, wiced_bt_mesh_properties_get_data_t *p_data);

/**
 * \brief The application can call this function to send Property Get client message to the server.
 *
 * @param       p_event Mesh event with the information about the message that has been created by the app for unsolicited message.
 *
 * @return      None
 */
void wiced_bt_mesh_model_property_client_send_property_get(wiced_bt_mesh_event_t *p_event, wiced_bt_mesh_property_get_data_t *p_get);

/**
 * \brief The application can call this function to send Property Set client message to the server.
 *
 * @param       p_event Mesh event with the information about the message that has been created by the app for unsolicited message.
 * @param       p_data Pointer to the data to send
 *
 * @return      None
 */
void wiced_bt_mesh_model_property_client_send_property_set(wiced_bt_mesh_event_t *p_event, wiced_bt_mesh_property_set_data_t* p_sata);

/**
 * \brief Light Lightness Server callback is called by the Mesh Models library on receiving a message from the peer
 * Application or higher level model should be able process messages for the Light CTL server model as well as
 * bound models including generic Light Lightness, Power On/Off, Generic Level and Generic On/Off.
 *
 * @param       event that application should process (see @ref LIGHT_LIGHTNESS_EVENT Light Lightness Events, @ref ONOFF_EVENT "On/Off Events, @ref LEVEL_EVENT "Level Events")
 * @param       p_event information about the message received.  The same pointer should be used in the reply if required.
 * @param       p_data pointer to the on/off state data
 *
 * @return      None
 */
typedef void(wiced_bt_mesh_light_lightness_server_callback_t)(uint16_t event, wiced_bt_mesh_event_t *p_event, void *p_data);

/**
 * \brief Light Lightness Model initialization
 * \details A Mesh application which contains Light Lightness Server (for example a dimmable bulb) should call this function for each element where the Light Lightness Server is present (for example for each bulb).
 *
 * @param       element_idx Device element to where model is used
 * @param       p_callback Application or Parent model callback that will be executed on receiving a message
 * @param       is_provisioned If TRUE, the application is being restarted after being provisioned or after a power loss. If FALSE the model cleans up NVRAM on startup.
 *
 * @return      WICED_TRUE if initialization was successful
 */
void wiced_bt_mesh_model_light_lightness_server_init(uint8_t element_idx, wiced_bt_mesh_light_lightness_server_callback_t *p_callback, wiced_bool_t is_provisioned);

/**
 * \brief Light Lightness Server Message Handler
 * \details The mesh core library calls this function for each message received.  The function returns WICED_TRUE if the message is destined for this specific model and successfully processed, and returns WICED_FALSE otherwise.
 *
 * @param       p_event Mesh event with information about received message.
 * @param       p_data Pointer to the data portion of the message
 * @param       data_len Length of the data in the message
 *
 * @return      WICED_TRUE if the message is for this company ID/Model/Element Index combination, WICED_FALSE otherwise.
*/
wiced_bool_t wiced_bt_mesh_model_light_lightness_server_message_handler(wiced_bt_mesh_event_t *p_event, uint8_t *p_data, uint16_t data_len);

/**
 * \brief An application or a a parent model can call this function to send Light Lightness Status message to the Client. The mesh event should
 * contain information if this is a reply or an unsolicited message.
 *
 * @param       p_event Mesh event with the information about the message that has been received in the request, or created by the app for unsolicited message.
 * @param       p_data Pointer to the status data
 *
 * @return      None
*/
void wiced_bt_mesh_model_light_lightness_server_send_status(wiced_bt_mesh_event_t *p_event, wiced_bt_mesh_light_lightness_status_data_t *p_data);

/**
 * \brief An application or a a parent model can call this function to send Light Lightness Linear Status message to the Client. The mesh event should
 * contain information if this is a reply or an unsolicited message.
 *
 * @param       p_event Mesh event with the information about the message that has been received in the request, or created by the app for unsolicited message.
 * @param       p_data Pointer to the status data
 *
 * @return      None
*/
void wiced_bt_mesh_model_light_lightness_server_send_linear_status(wiced_bt_mesh_event_t *p_event, wiced_bt_mesh_light_lightness_status_data_t *p_data);

/**
 * \brief An application or a a parent model can call this function to send Light Lightness Last Status message to the Client. The mesh event should
 * contain information if this is a reply or an unsolicited message.
 *
 * @param       p_event Mesh event with the information about the message that has been received in the request, or created by the app for unsolicited message.
 * @param       p_data Pointer to the status data
 *
 * @return      None
*/
void wiced_bt_mesh_model_light_lightness_server_send_last_status(wiced_bt_mesh_event_t *p_event, wiced_bt_mesh_power_level_last_data_t *p_data);

/**
 * \brief An application or a a parent model can call this function to send Ligth Lightness Default Status message to the Client. The mesh event should
 * contain information if this is a reply or an unsolicited message.
 *
 * @param       p_event Mesh event with the information about the message that has been received in the request, or created by the app for unsolicited message.
 * @param       p_data Pointer to the status data
 *
 * @return      None
*/
void wiced_bt_mesh_model_light_lightness_server_send_default_status(wiced_bt_mesh_event_t *p_event, wiced_bt_mesh_light_lightness_default_data_t *p_data);

/**
 * \brief An application or a a parent model can call this function to send Light Lightness Range Status message to the Client. The mesh event should
 * contain information if this is a reply or an unsolicited message.
 *
 * @param       p_event Mesh event with the information about the message that has been received in the request, or created by the app for unsolicited message.
 * @param       p_data Pointer to the status data
 *
 * @return      None
*/
void wiced_bt_mesh_model_light_lightness_server_send_range_status(wiced_bt_mesh_event_t *p_event, wiced_bt_mesh_light_lightness_range_status_data_t *p_data);

/**
 * \brief Light Lightness Setup Server callback is called by the Mesh Models library on receiving a message from the peer
 * Application or higher level model should be able process messages for the Light Lightness Setup server model as well as
 * bound models including generic Light Lightness, Power On/Off, Generic Level and Generic On/Off.
 *
 * @param       event that application should process (see @ref LIGHT_LIGHTNESS_EVENT Light Lightness Events, @ref ONOFF_EVENT "On/Off Events, @ref LEVEL_EVENT "Level Events")
 * @param       p_event information about the message received.  The same pointer should be used in the reply if required.
 * @param       p_data pointer to the data structure specific to the event
 *
 * @return      None
 */
typedef void (wiced_bt_mesh_light_lightness_setup_server_callback_t)(uint16_t event, wiced_bt_mesh_event_t *p_event, void *p_data);

/**
 * \brief Light Lightness Setup Server Model initialization
 * \details A Mesh application which contains Light Lightness Setup Server (for example a dimmable bulb) should call this function for each element where the Light Lightness Server is present (for example for each bulb).
 *
 * @param       element_idx Device element to where model is used
 * @param       p_callback Application or Parent model callback that will be executed on receiving a message
 * @param       is_provisioned If TRUE, the application is being restarted after being provisioned or after a power loss. If FALSE the model cleans up NVRAM on startup.
 *
 * @return      WICED_TRUE if initialization was successful
 */
void wiced_bt_mesh_model_light_lightness_setup_server_init(uint8_t element_idx, wiced_bt_mesh_light_lightness_setup_server_callback_t *p_callback, wiced_bool_t is_provisioned);

/**
 * \brief Light Lightness Setup Server Set Paremt
 * \details In multi server applications, for example HSL/CTL, only one model should receive Get/Set for lightness/level/ononff.  This function
 * should be called when the master changes.  
 *
 * @param       element_idx Device element to where model is used
 * @param       p_callback Application or Parent model callback that will be executed on receiving a message
 *
 * @return      WICED_TRUE if initialization was successful
 */
void wiced_bt_mesh_model_light_lightness_set_parent(uint8_t element_idx, wiced_bt_mesh_light_lightness_setup_server_callback_t *p_callback);

/**
 * \brief Light Lightness Setup Server Message Handler
 * \details The mesh core library calls this function for each message received.  The function returns WICED_TRUE if the message is destined for this specific model and successfully processed, and returns WICED_FALSE otherwise. 
 *
 * @param       p_event Mesh event with information about received message.
 * @param       p_data Pointer to the data portion of the message
 * @param       data_len Length of the data in the message
 *
 * @return      WICED_TRUE if the message is for this company ID/Model/Element Index combination, WICED_FALSE otherwise.
*/
wiced_bool_t wiced_bt_mesh_model_light_lightness_setup_server_message_handler(wiced_bt_mesh_event_t *p_event, uint8_t *p_data, uint16_t data_len);

/**
 * \brief Light Lightness Server Scene Store Handler
 * \details The mesh Scene Server calls this function so that the module can store required data for the scene.
 *
 * @param       element_idx Element index for which scene information is being stored
 * @param       p_buffer Pointer to a buffer where data should be stored
 * @param       buf_size Maximum amount of data a model can store
 *
 * @return      Number of bytes the model wrote into the provided buffer
 */
uint16_t wiced_bt_mesh_model_light_lightness_server_scene_store_handler(uint8_t element_idx, uint8_t *p_buffer, uint16_t buffer_len);

/**
 * \brief Light Lightness Server Scene Store Handler
 * \details When Scene Server receives Scene Recall message it calls this function with data previously stored in the NVRAM.
 *
 * @param       element_idx Element index for which scene information is being recalled
 * @param       p_buffer Pointer to a buffer where model's data is located
 * @param       buf_size Maximum amount of data a model can get from the buffer
 *
 * @return      Number of bytes the model read from the provided buffer
 */
uint16_t wiced_bt_mesh_model_light_lightness_server_scene_recall_handler(uint8_t element_idx, uint8_t *p_buffer, uint16_t buffer_len, uint32_t transition_time, uint32_t delay);

/**
 * \brief Light Lightness Client callback is called by the Mesh Models library on receiving a message from the peer
 *
 * @param       event that application should process (see @ref LIGHT_LIGHTNESS_EVENT "Mesh Light Lightness Events")
 * @param       p_event information about the message received.  The same pointer should be used in the reply if required.
 * @param       p_data pointer to the data to send
 *
 * @return      None
 */
typedef void(wiced_bt_mesh_light_lightness_client_callback_t)(uint16_t event, wiced_bt_mesh_event_t *p_event, void *p_data);

/**
 * \brief Light Lightness Client Model initialization
 *
 * @param       element_idx Device element to where model is used
 * @param       p_callback The application callback function that will be executed by the mesh models library when application action is required, or when a reply for the application request has been received.
 * @param       is_provisioned If TRUE, the application is being restarted after being provisioned or after a power loss. If FALSE the model cleans up NVRAM on startup.
 *
 * @return      None
 */
void wiced_bt_mesh_model_light_lightness_client_init(uint8_t element_idx, wiced_bt_mesh_power_onoff_client_callback_t *p_callback, wiced_bool_t is_provisioned);

/**
 * \brief Light Lightness Client Message Handler
 * \details The mesh core library calls this function for each message received.  The function returns WICED_TRUE if the message is destined for this specific model and successfully processed, and returns WICED_FALSE otherwise.
 *
 * @param       p_event Mesh event with information about received message.
 * @param       p_data Pointer to the data portion of the message
 * @param       data_len Length of the data in the message
 *
 * @return      WICED_TRUE if the message is for this company ID/Model/Element Index combination, WICED_FALSE otherwise.
*/
wiced_bool_t wiced_bt_mesh_model_light_lightness_client_message_handler(wiced_bt_mesh_event_t *p_event, uint8_t *p_data, uint16_t data_len);

/**
 * \brief The application can call this function to send the Light Lightness Get client message to the server.
 *
 * @param       p_event Mesh event with the information about the message that has been created by the app for unsolicited message.
 *
 * @return      None
 */
void wiced_bt_mesh_model_light_lightness_client_send_get(wiced_bt_mesh_event_t *p_event);

/**
 * \brief The application can call this function to send Light Lightness Set client message to the server.
 *
 * @param       p_event Mesh event with the information about the message that has been created by the app for unsolicited message.
 * @param       p_data Pointer to the data to send
 *
 * @return      None
 */
void wiced_bt_mesh_model_light_lightness_client_send_set(wiced_bt_mesh_event_t *p_event, wiced_bt_mesh_light_lightness_actual_set_t* p_data);

/**
 * \brief The application can call this function to send the Light Lightness Linear Get client message to the server.
 *
 * @param       p_event Mesh event with the information about the message that has been created by the app for unsolicited message.
 *
 * @return      None
 */
void wiced_bt_mesh_model_light_lightness_client_send_linear_get(wiced_bt_mesh_event_t *p_event);

/**
 * \brief The application can call this function to send Light Lightness Linear Set client message to the server.
 *
 * @param       p_event Mesh event with the information about the message that has been created by the app for unsolicited message.
 * @param       p_data Pointer to the data to send
 *
 * @return      None
 */
void wiced_bt_mesh_model_light_lightness_client_send_linear_set(wiced_bt_mesh_event_t *p_event, wiced_bt_mesh_light_lightness_linear_set_t* p_data);

/**
 * \brief The application can call this function to send the Light Lightness Last Get client message to the server.
 *
 * @param       p_event Mesh event with the information about the message that has been created by the app for unsolicited message.
 *
 * @return      None
 */
void wiced_bt_mesh_model_light_lightness_client_send_last_get(wiced_bt_mesh_event_t *p_event);

/**
 * \brief The application can call this function to send the Light Lightness Default Get client message to the server.
 *
 * @param       p_event Mesh event with the information about the message that has been created by the app for unsolicited message.
 *
 * @return      None
 */
void wiced_bt_mesh_model_light_lightness_client_send_default_get(wiced_bt_mesh_event_t *p_event);

/**
 * \brief The application can call this function to send Light Lightness Default Set client message to the server.
 *
 * @param       p_event Mesh event with the information about the message that has been created by the app for unsolicited message.
 * @param       p_data Pointer to the data to send
 *
 * @return      None
 */
void wiced_bt_mesh_model_light_lightness_client_send_default_set(wiced_bt_mesh_event_t *p_event, wiced_bt_mesh_light_lightness_default_data_t* p_data);

/**
 * \brief The application can call this function to send the Light Lightness Range Get client message to the server.
 *
 * @param       p_event Mesh event with the information about the message that has been created by the app for unsolicited message.
 *
 * @return      None
 */
void wiced_bt_mesh_model_light_lightness_client_send_range_get(wiced_bt_mesh_event_t *p_event);

/**
 * \brief The application can call this function to send Light Lightness Default Set client message to the server.
 *
 * @param       p_event Mesh event with the information about the message that has been created by the app for unsolicited message.
 * @param       p_data Pointer to the data to send
 *
 * @return      None
 */
void wiced_bt_mesh_model_light_lightness_client_send_range_set(wiced_bt_mesh_event_t *p_event, wiced_bt_mesh_light_lightness_range_set_data_t* p_data);

/**
 * \brief Light CTL Temperature Server callback is called by the Mesh Models library on receiving a message from the peer
 * Application or higher level model (typically Light CTL server) should be able process messages for the this server model as well as
 * bound generic level model.
 *
 * @param       event that application should process (see @ref LIGHT_CTL_TEMPERATURE_EVENT "Light CTL Temperature Level Events")
 * @param       p_event information about the message received.  The same pointer should be used in the reply if required.
 * @param       p_data pointer to the on/off state data
 *
 * @return      None
 */
typedef void(wiced_bt_mesh_light_ctl_temperature_server_callback_t)(uint16_t event, wiced_bt_mesh_event_t *p_event, void *p_data);

/**
 * \brief Light CTL Temperature Server Message Handler
 * \details The mesh core library calls this function for each message received.  The function returns WICED_TRUE if the message is destined for this specific model and successfully processed, and returns WICED_FALSE otherwise.
 *
 * @param       p_event Mesh event with information about received message.
 * @param       p_data Pointer to the data portion of the message
 * @param       data_len Length of the data in the message
 *
 * @return      WICED_TRUE if the message is for this company ID/Model/Element Index combination, WICED_FALSE otherwise.
*/
wiced_bool_t wiced_bt_mesh_model_light_ctl_temperature_server_message_handler(wiced_bt_mesh_event_t *p_event, uint8_t *p_data, uint16_t data_len);

/**
 * \brief Light CTL Server callback is called by the Mesh Models library on receiving a message from the peer
 * Application or higher level model should be able process messages for the Light CTL server model as well as
 * bound models including generic Power On/Off, Generic Level and Generic On/Off.
 *
 * @param       event that application should process (see @ref LIGHT_CTL_EVENT Light CTL Events, @ref ONOFF_EVENT "On/Off Events, @ref LEVEL_EVENT "Level Events")
 * @param       p_event information about the message received.  The same pointer should be used in the reply if required.
 * @param       p_data pointer to the on/off state data
 *
 * @return      None
 */
typedef void(wiced_bt_mesh_light_ctl_server_callback_t)(uint16_t event, wiced_bt_mesh_event_t *p_event, void *p_data);

/**
 * \brief Light CTL Model initialization
 * \details A Mesh application which contains Light CTL Server (for example a dimmable bulb with temperature control) should call this function for each element where the Light CTL Server is present (for example for each bulb).
 *
 * @param       element_idx Device element to where model is used
 * @param       p_callback Application or Parent model callback that will be executed on receiving a message
 * @param       is_provisioned If TRUE, the application is being restarted after being provisioned or after a power loss. If FALSE the model cleans up NVRAM on startup.
 *
 * @return      WICED_TRUE if initialization was successful
 */
void wiced_bt_mesh_model_light_ctl_server_init(uint8_t element_idx, wiced_bt_mesh_light_ctl_server_callback_t *p_callback, wiced_bool_t is_provisioned);

/**
 * \brief Light CTL Server Message Handler
 * \details The mesh core library calls this function for each message received.  The function returns WICED_TRUE if the message is destined for this specific model and successfully processed, and returns WICED_FALSE otherwise. 
 *
 * @param       p_event Mesh event with information about received message.
 * @param       p_data Pointer to the data portion of the message
 * @param       data_len Length of the data in the message
 *
 * @return      WICED_TRUE if the message is for this company ID/Model/Element Index combination, WICED_FALSE otherwise.
*/
wiced_bool_t wiced_bt_mesh_model_light_ctl_server_message_handler(wiced_bt_mesh_event_t *p_event, uint8_t *p_data, uint16_t data_len);

/**
 * \brief Light CTL Server Scene Store Handler
 * \details The mesh Scene Server calls this function so that the module can store required data for the scene.
 *
 * @param       element_idx Element index for which scene information is being stored
 * @param       p_buffer Pointer to a buffer where data should be stored
 * @param       buf_size Maximum amount of data a model can store
 *
 * @return      Number of bytes the model wrote into the provided buffer
 */
uint16_t wiced_bt_mesh_model_light_ctl_server_scene_store_handler(uint8_t element_idx, uint8_t *p_buffer, uint16_t buffer_len);

/**
 * \brief Light CTL Server Scene Store Handler
 * \details When Scene Server receives Scene Recall message it calls this function with data previously stored in the NVRAM.
 *
 * @param       element_idx Element index for which scene information is being recalled
 * @param       p_buffer Pointer to a buffer where model's data is located
 * @param       buf_size Maximum amount of data a model can get from the buffer
 *
 * @return      Number of bytes the model read from the provided buffer
 */
uint16_t wiced_bt_mesh_model_light_ctl_server_scene_recall_handler(uint8_t element_idx, uint8_t *p_buffer, uint16_t buffer_len, uint32_t transition_time, uint32_t delay);

/**
 * \brief An application or a a parent model can call this function to send Light CTL Status message to the Client. The mesh event should
 * contain information if this is a reply or an unsolicited message.
 *
 * @param       p_event Mesh event with the information about the message that has been received in the request, or created by the app for unsolicited message.
 * @param       p_data Pointer to the status data
 *
 * @return      None
*/
void wiced_bt_mesh_model_light_ctl_server_send_status(wiced_bt_mesh_event_t *p_event, wiced_bt_mesh_light_ctl_status_data_t *p_data);

/**
 * \brief An application or a a parent model can call this function to send Light CTL Temperature Status message to the Client. The mesh event should
 * contain information if this is a reply or an unsolicited message.
 *
 * @param       p_event Mesh event with the information about the message that has been received in the request, or created by the app for unsolicited message.
 * @param       p_data Pointer to the status data
 *
 * @return      None
 */
void wiced_bt_mesh_model_light_ctl_server_send_temperature_status(wiced_bt_mesh_event_t *p_event, wiced_bt_mesh_light_ctl_status_data_t *p_data);

 /**
 * \brief An application or a a parent model can call this function to send Light CTL Temperature Range Status message to the Client. The mesh event should
 * contain information if this is a reply or an unsolicited message.
 *
 * @param       p_event Mesh event with the information about the message that has been received in the request, or created by the app for unsolicited message.
 * @param       p_data Pointer to the status data
 *
 * @return      None
 */
void wiced_bt_mesh_model_light_ctl_server_send_temperature_range_status(wiced_bt_mesh_event_t *p_event, wiced_bt_mesh_light_ctl_temperature_range_status_data_t *p_status);

/**
 * \brief An application or a a parent model can call this function to send Light CTL Default Status message to the Client. The mesh event should
 * contain information if this is a reply or an unsolicited message.
 *
 * @param       p_event Mesh event with the information about the message that has been received in the request, or created by the app for unsolicited message.
 * @param       p_data Pointer to the status data
 *
 * @return      None
 */
void wiced_bt_mesh_model_light_ctl_server_send_default_status(wiced_bt_mesh_event_t *p_event, wiced_bt_mesh_light_ctl_default_data_t *p_data);

/**
 * \brief Set Light CTL Model as a startup master
 * \details For multi-model applications, for example the ones that support both HSL and CTL controls, only one can be setup as a startup master.
 * If the master mode is set, the library will execute the power on requirements, for example it can set the temperature, lightness
 * based on the last set state.  If the model is not set as a master, it will not execute on power up state.
 *
 * @param       elemnt_idx The element index of the top level model
 * @param       startup_master If set to WICED_TRUE, the model will execute on power up operation
 *
 * @return      None
 */
void wiced_bt_mesh_model_light_ctl_server_set_startup_master(uint8_t element_idx, wiced_bool_t startup_master);

/**
 * \brief Light CTL Setup Server callback is called by the Mesh Models library on receiving a message from the peer
 * Application or higher level model should be able process messages for the Light CTL Setup server model.
 *
 * @param       event that application should process (see @ref LIGHT_CTL_EVENT "Light CTL Events")
 * @param       p_event information about the message received.  The same pointer should be used in the reply if required.
 * @param       p_data pointer to the data structure specific to the event
 *
 * @return      None
 */
typedef void (wiced_bt_mesh_light_ctl_setup_server_callback_t)(uint16_t event, wiced_bt_mesh_event_t *p_event, void *p_data);

/**
 * \brief Light CTL Setup Server Model initialization
 * \details A Mesh application which contains Light CTL Setup Server (for example a dimmable bulb with color temperature control)
 * should call this function. In the mesh model specifications the functionality is split into two elements, the lightness and
 * the color temperature.  During initialization the application should pass element indexes of two elements.
 *
 * @param       lightness_element_idx Device element where lightness part of the CTL is located
 * @param       p_callback Application or Parent model callback that will be executed on receiving a message
 * @param       is_provisioned If TRUE, the application is being restarted after being provisioned or after a power loss. If FALSE the model cleans up NVRAM on startup.
 *
 * @return      WICED_TRUE if initialization was successful
 */
void wiced_bt_mesh_model_light_ctl_setup_server_init(uint8_t lightness_element_idx, wiced_bt_mesh_light_ctl_server_callback_t *p_callback, wiced_bool_t is_provisioned);

/**
 * \brief Light CTL Setup Server Message Handler
 * \details The mesh core library calls this function for each message received.  The function returns WICED_TRUE if the message is destined for this specific model and successfully processed, and returns WICED_FALSE otherwise. 
 *
 * @param       p_event Mesh event with information about received message.
 * @param       p_data Pointer to the data portion of the message
 * @param       data_len Length of the data in the message
 *
 * @return      WICED_TRUE if the message is for this company ID/Model/Element Index combination, WICED_FALSE otherwise.
*/
wiced_bool_t wiced_bt_mesh_model_light_ctl_setup_server_message_handler(wiced_bt_mesh_event_t *p_event, uint8_t *p_data, uint16_t data_len);

/**
 * \brief Light CTL Client callback is called by the Mesh Models library on receiving a message from the peer
 *
 * @param       event that application should process (see @ref LIGHT_CTL_EVENT "Light CTL Events")
 * @param       p_event information about the message received.  The same pointer should be used in the reply if required.
 * @param       p_data pointer to the level data
 *
 * @return      None
 */
typedef void(wiced_bt_mesh_light_ctl_client_callback_t)(uint16_t event, wiced_bt_mesh_event_t *p_event, void *p_data);

/**
 * \brief Light CTL Client Message Handler
 * \details The mesh core library calls this function for each message received.  The function returns WICED_TRUE if the message is destined for this specific model and successfully processed, and returns WICED_FALSE otherwise.
 * The function parses the message and if appropriate calls the parent back to perform functionality.
 *
 * @param       p_event Mesh event with information about received message.
 * @param       p_data Pointer to the data portion of the message
 * @param       data_len Length of the data in the message
 *
 * @return      WICED_TRUE if the message is for this company ID/Model/Element Index combination, WICED_FALSE otherwise.
 */
wiced_bool_t wiced_bt_mesh_model_light_ctl_client_message_handler(wiced_bt_mesh_event_t *p_event, uint8_t *p_data, uint16_t data_len);

/**
 * \brief Light CTL Client Model initialization
 *
 * @param       element_idx Device element to where model is used
 * @param       p_callback The application callback function that will be executed by the mesh models library when application action is required, or when a reply for the application request has been received.
 * @param       is_provisioned If TRUE, the application is being restarted after being provisioned or after a power loss. If FALSE the model cleans up NVRAM on startup.
 *
 * @return      None
 */
void wiced_bt_mesh_model_light_ctl_client_init(uint8_t element_idx, wiced_bt_mesh_light_ctl_client_callback_t *p_callback, wiced_bool_t is_provisioned);

/**
 * \brief The application can call this function to send Light CTL Get message to the server.
 *
 * @param       p_event Mesh event with the information about the message that has been created by the app for unsolicited message.
 *
 * @return      None
 */
void wiced_bt_mesh_model_light_ctl_client_send_get(wiced_bt_mesh_event_t *p_event);

/**
 * \brief The application can call this function to send Light CTL Set client message to the server.
 *
 * @param       p_event Mesh event with the information about the message that has been created by the app for unsolicited message.
 * @param       p_data Pointer to the data to send
 *
 * @return      None
 */
void wiced_bt_mesh_model_light_ctl_client_send_set(wiced_bt_mesh_event_t *p_event, wiced_bt_mesh_light_ctl_set_t* p_data);

/**
 * \brief The application can call this function to send Light CTL Temperature Get message to the server.
 *
 * @param       p_event Mesh event with the information about the message that has been created by the app for unsolicited message.
 *
 * @return      None
 */
void wiced_bt_mesh_model_light_ctl_client_send_temperature_get(wiced_bt_mesh_event_t *p_event);

/**
 * \brief The application can call this function to send Light CTL Temperature Set client message to the server.
 *
 * @param       p_event Mesh event with the information about the message that has been created by the app for unsolicited message.
 * @param       p_data Pointer to the data to send
 *
 * @return      None
 */
void wiced_bt_mesh_model_light_ctl_client_send_temperature_set(wiced_bt_mesh_event_t *p_event, wiced_bt_mesh_light_ctl_temperature_set_t* p_data);

/**
 * \brief The application can call this function to send Light CTL Get Default client message to the server.
 *
 * @param       p_event Mesh event with the information about the message that has been created by the app for unsolicited message.
 * @param       p_data Pointer to the data to send
 *
 * @return      None
 */
void wiced_bt_mesh_model_light_ctl_client_send_default_get(wiced_bt_mesh_event_t *p_event);

/**
 * \brief The application can call this function to send Light CTL Set Default  client message to the server.
 *
 * @param       p_event Mesh event with the information about the message that has been created by the app for unsolicited message.
 * @param       p_data Pointer to the data to send
 *
 * @return      None
 */
void wiced_bt_mesh_model_light_ctl_client_send_default_set(wiced_bt_mesh_event_t *p_event, wiced_bt_mesh_light_ctl_default_data_t* p_data);

/**
 * \brief The application can call this function to send Light CTL Temperature Range Get client message to the server.
 *
 * @param       p_event Mesh event with the information about the message that has been created by the app for unsolicited message.
 * @param       p_data Pointer to the data to send
 *
 * @return      None
 */
void wiced_bt_mesh_model_light_ctl_client_send_temperature_range_get(wiced_bt_mesh_event_t *p_event);

/**
 * \brief The application can call this function to send Light CTL Temperature Range Set client message to the server.
 *
 * @param       p_event Mesh event with the information about the message that has been created by the app for unsolicited message.
 * @param       p_data Pointer to the data to send
 *
 * @return      None
 */
void wiced_bt_mesh_model_light_ctl_client_send_temperature_range_set(wiced_bt_mesh_event_t *p_event, wiced_bt_mesh_light_ctl_temperature_range_data_t* p_data);

/**
 * \brief Light HSL Server callback is called by the Mesh Models library on receiving a message from the peer
 * Application or higher level model should be able process messages for the Light HSL server model as well as
 * bound models.
 *
 * @param       event that application should process (see @ref LIGHT_HSL_EVENT Light HSL Events)
 * @param       p_event information about the message received.  The same pointer should be used in the reply if required.
 * @param       p_data pointer to the on/off state data
 *
 * @return      None
 */
typedef void(wiced_bt_mesh_light_hsl_server_callback_t)(uint16_t event, wiced_bt_mesh_event_t *p_event, void *p_data);

/**
 * \brief Light HSL Model initialization
 * \details A Mesh application which contains Light HSL Server (for example a colored dimmable bulb) should call this function for each element
 * where the Light HSL Server is present (for example for each bulb).
 *
 * @param       element_idx Device element to where model is used
 * @param       p_callback Application or Parent model callback that will be executed on receiving a message
 * @param       is_provisioned If TRUE, the application is being restarted after being provisioned or after a power loss. If FALSE the model cleans up NVRAM on startup.
 *
 * @return      WICED_TRUE if initialization was successful
 */
void wiced_bt_mesh_model_light_hsl_server_init(uint8_t element_idx, wiced_bt_mesh_light_hsl_server_callback_t *p_callback, wiced_bool_t is_provisioned);

/**
 * \brief Light HSL Server Message Handler
 * \details The mesh core library calls this function for each message received.  The function shall return WICED_TRUE if the message is 
 * destined for this specific model and successfully processed, and returns WICED_FALSE otherwise.
 *
 * @param       p_event Mesh event with information about received message.
 * @param       p_data Pointer to the data portion of the message
 * @param       data_len Length of the data in the message
 *
 * @return      WICED_TRUE if the message is for this company ID/Model/Element Index combination, WICED_FALSE otherwise.
 */
wiced_bool_t wiced_bt_mesh_model_light_hsl_server_message_handler(wiced_bt_mesh_event_t *p_event, uint8_t *p_data, uint16_t data_len);

/**
 * \brief Light HSL Server Scene Store Handler
 * \details The mesh Scene Server calls this function so that the module can store required data for the scene.
 *
 * @param       element_idx Element index for which scene information is being stored
 * @param       p_buffer Pointer to a buffer where data should be stored
 * @param       buf_size Maximum amount of data a model can store
 *
 * @return      Number of bytes the model wrote into the provided buffer
 */
uint16_t wiced_bt_mesh_model_light_hsl_server_scene_store_handler(uint8_t element_idx, uint8_t *p_buffer, uint16_t buffer_len);

/**
 * \brief Light HSL Server Scene Store Handler
 * \details When Scene Server receives Scene Recall message it calls this function with data previously stored in the NVRAM.
 *
 * @param       element_idx Element index for which scene information is being recalled
 * @param       p_buffer Pointer to a buffer where model's data is located
 * @param       buf_size Maximum amount of data a model can get from the buffer
 *
 * @return      Number of bytes the model read from the provided buffer
 */
uint16_t wiced_bt_mesh_model_light_hsl_server_scene_recall_handler(uint8_t element_idx, uint8_t *p_buffer, uint16_t buffer_len, uint32_t transition_time, uint32_t delay);

/**
 * \brief An application or a a parent model can call this function to send Light HSL Status message to the Client. The mesh event should
 * contain information if this is a reply or an unsolicited message.
 *
 * @param       p_event Mesh event with the information about the message that has been received in the request, or created by the app for unsolicited message.
 * @param       p_data Pointer to the status data
 *
 * @return      None
 */
void wiced_bt_mesh_model_light_hsl_server_send_status(wiced_bt_mesh_event_t *p_event, wiced_bt_mesh_light_hsl_status_data_t *p_data);

/**
 * \brief An application or a a parent model can call this function to send Light HSL Hue Status message to the Client. The mesh event should
 * contain information if this is a reply or an unsolicited message.
 *
 * @param       p_event Mesh event with the information about the message that has been received in the request, or created by the app for unsolicited message.
 * @param       p_data Pointer to the status data
 *
 * @return      None
 */
void wiced_bt_mesh_model_light_hsl_server_send_hue_status(wiced_bt_mesh_event_t *p_event, wiced_bt_mesh_light_hsl_hue_status_data_t *p_data);

/**
 * \brief An application or a a parent model can call this function to send Light HSL Saturation Status message to the Client. The mesh event should
 * contain information if this is a reply or an unsolicited message.
 *
 * @param       p_event Mesh event with the information about the message that has been received in the request, or created by the app for unsolicited message.
 * @param       p_data Pointer to the status data
 *
 * @return      None
 */
void wiced_bt_mesh_model_light_hsl_server_send_saturation_status(wiced_bt_mesh_event_t *p_event, wiced_bt_mesh_light_hsl_saturation_status_data_t *p_data);

/**
 * \brief An application or a a parent model can call this function to send Light HSL Range Status message to the Client. The mesh event should
 * contain information if this is a reply or an unsolicited message.
 *
 * @param       p_event Mesh event with the information about the message that has been received in the request, or created by the app for unsolicited message.
 * @param       p_data Pointer to the status data
 *
 * @return      None
 */
void wiced_bt_mesh_model_light_hsl_server_send_range_status(wiced_bt_mesh_event_t *p_event, wiced_bt_mesh_light_hsl_range_status_data_t *p_status);

/**
 * \brief An application or a a parent model can call this function to send Light HSL Default Status message to the Client. The mesh event should
 * contain information if this is a reply or an unsolicited message.
 *
 * @param       p_event Mesh event with the information about the message that has been received in the request, or created by the app for unsolicited message.
 * @param       p_data Pointer to the status data
 *
 * @return      None
 */
void wiced_bt_mesh_model_light_hsl_server_send_default_status(wiced_bt_mesh_event_t *p_event, wiced_bt_mesh_light_hsl_default_status_data_t *p_data);

/**
 * \brief Set Light HSL Model as a startup master
 * \details For multi-model applications, for example the ones that support both HSL and CTL controls, only one can be setup as a startup master.
 * If the master mode is set, the library will execute the power on requirements, for example it can set the lightness/hue/saturation
 * based on the last state before the power off.  If the model is not set as a master, it will not execute on power up state.
 *
 * @param       elemnt_idx The element index of the top level model
 * @param       startup_master If set to WICED_TRUE, the model will execute on power up operation
 *
 * @return      None
 */
void wiced_bt_mesh_model_light_hsl_server_set_startup_master(uint8_t element_idx, wiced_bool_t startup_master);

/**
 * \brief Light HSL Setup Server callback is called by the Mesh Models library on receiving a message from the peer
 * Application or higher level model should be able process messages for the Light HSL Setup server model.
 *
 * @param       event that application should process (see @ref LIGHT_HSL_EVENT "Light HSL Events")
 * @param       p_event information about the message received.  The same pointer should be used in the reply if required.
 * @param       p_data pointer to the data structure specific to the event
 *
 * @return      None
 */
typedef void (wiced_bt_mesh_light_hsl_setup_server_callback_t)(uint16_t event, wiced_bt_mesh_event_t *p_event, void *p_data);

/**
 * \brief Light HSL Setup Server Model initialization
 * \details A Mesh application which contains Light HSL Setup Server (for example a dimmable bulb color bulb)
 * should call this function. In the mesh model specifications the functionality is split into 3 elements, the
 * hue and saturation.  During initialization the application passes the indes of the first lightness element
 * while the hue and saturation elements should follow consecutively.
 *
 * @param       lightness_element_idx Device element where lightness part of the HSL is located
 * @param       p_callback Application or Parent model callback that will be executed on receiving a message
 * @param       is_provisioned If TRUE, the application is being restarted after being provisioned or after a power loss. If FALSE the model cleans up NVRAM on startup.
 *
 * @return      WICED_TRUE if initialization was successful
 */
void wiced_bt_mesh_model_light_hsl_setup_server_init(uint8_t lightness_element_idx, wiced_bt_mesh_light_hsl_server_callback_t *p_callback, wiced_bool_t is_provisioned);

/**
 * \brief Light HSL Setup Server Message Handler
 * \details The mesh core library calls this function for each message received.  The function returns WICED_TRUE if the message is destined for this specific model and successfully processed, and returns WICED_FALSE otherwise.
 *
 * @param       p_event Mesh event with information about received message.
 * @param       p_data Pointer to the data portion of the message
 * @param       data_len Length of the data in the message
 *
 * @return      WICED_TRUE if the message is for this company ID/Model/Element Index combination, WICED_FALSE otherwise.
 */
wiced_bool_t wiced_bt_mesh_model_light_hsl_setup_server_message_handler(wiced_bt_mesh_event_t *p_event, uint8_t *p_data, uint16_t data_len);

/**
 * \brief Light HSL Hue Server Message Handler
 * \details The mesh core library calls this function for each message received.  The function returns WICED_TRUE if the message is destined for this specific model and successfully processed, and returns WICED_FALSE otherwise.
 *
 * @param       p_event Mesh event with information about received message.
 * @param       p_data Pointer to the data portion of the message
 * @param       data_len Length of the data in the message
 *
 * @return      WICED_TRUE if the message is for this company ID/Model/Element Index combination, WICED_FALSE otherwise.
 */
wiced_bool_t wiced_bt_mesh_model_light_hsl_hue_server_message_handler(wiced_bt_mesh_event_t *p_event, uint8_t *p_data, uint16_t data_len);

/**
 * \brief Light HSL Saturation Server Message Handler
 * \details The mesh core library calls this function for each message received.  The function returns WICED_TRUE if the message is destined for this specific model and successfully processed, and returns WICED_FALSE otherwise.
 *
 * @param       p_event Mesh event with information about received message.
 * @param       p_data Pointer to the data portion of the message
 * @param       data_len Length of the data in the message
 *
 * @return      WICED_TRUE if the message is for this company ID/Model/Element Index combination, WICED_FALSE otherwise.
 */
wiced_bool_t wiced_bt_mesh_model_light_hsl_saturation_server_message_handler(wiced_bt_mesh_event_t *p_event, uint8_t *p_data, uint16_t data_len);


/**
 * \brief Light HSL Client callback is called by the Mesh Models library on receiving a message from the peer
 *
 * @param       event that application should process (see @ref LIGHT_HSL_EVENT "Light HSL Events")
 * @param       p_event information about the message received.  The same pointer should be used in the reply if required.
 * @param       p_data pointer to the level data
 *
 * @return      None
 */
typedef void(wiced_bt_mesh_light_hsl_client_callback_t)(uint16_t event, wiced_bt_mesh_event_t *p_event, void *p_data);

 /**
 * \brief Light HSL Client Message Handler
 * \details The mesh core library calls this function for each message received.  The function returns WICED_TRUE if the message is destined for this specific model and successfully processed, and returns WICED_FALSE otherwise.
 * The function parses the message and if appropriate calls the parent back to perform functionality.
 *
 * @param       p_event Mesh event with information about received message.
 * @param       p_data Pointer to the data portion of the message
 * @param       data_len Length of the data in the message
 *
 * @return      WICED_TRUE if the message is for this company ID/Model/Element Index combination, WICED_FALSE otherwise.
 */
wiced_bool_t wiced_bt_mesh_model_light_hsl_client_message_handler(wiced_bt_mesh_event_t *p_event, uint8_t *p_data, uint16_t data_len);

 /**
 * \brief Light HSL Client Model initialization
 *
 * @param       element_idx Device element to where model is used
 * @param       p_callback The application callback function that will be executed by the mesh models library when application action is required, or when a reply for the application request has been received.
 * @param       is_provisioned If TRUE, the application is being restarted after being provisioned or after a power loss. If FALSE the model cleans up NVRAM on startup.
 *
 * @return      None
 */
void wiced_bt_mesh_model_light_hsl_client_init(uint8_t element_idx, wiced_bt_mesh_light_hsl_client_callback_t *p_callback, wiced_bool_t is_provisioned);

/**
 * \brief The application can call this function to send Light HSL Get message to the server.
 *
 * @param       p_event Mesh event with the information about the message that has been created by the app for unsolicited message.
 *
 * @return      None
 */
void wiced_bt_mesh_model_light_hsl_client_send_get(wiced_bt_mesh_event_t *p_event);

/**
 * \brief The application can call this function to send Light HSL Set client message to the server.
 *
 * @param       p_event Mesh event with the information about the message that has been created by the app for unsolicited message.
 * @param       p_data Pointer to the data to send
 *
 * @return      None
 */
void wiced_bt_mesh_model_light_hsl_client_send_set(wiced_bt_mesh_event_t *p_event, wiced_bt_mesh_light_hsl_set_t* p_data);

/**
 * \brief The application can call this function to send Light HSL Hue Get message to the server.
 *
 * @param       p_event Mesh event with the information about the message that has been created by the app for unsolicited message.
 *
 * @return      None
 */
void wiced_bt_mesh_model_light_hsl_client_send_hue_get(wiced_bt_mesh_event_t *p_event);

/**
 * \brief The application can call this function to send Light HSL Hue Set client message to the server.
 *
 * @param       p_event Mesh event with the information about the message that has been created by the app for unsolicited message.
 * @param       p_data Pointer to the data to send
 *
 * @return      None
 */
void wiced_bt_mesh_model_light_hsl_client_send_hue_set(wiced_bt_mesh_event_t *p_event, wiced_bt_mesh_light_hsl_hue_set_t* p_data);

/**
 * \brief The application can call this function to send Light HSL Saturation Get message to the server.
 *
 * @param       p_event Mesh event with the information about the message that has been created by the app for unsolicited message.
 *
 * @return      None
 */
void wiced_bt_mesh_model_light_hsl_client_send_saturation_get(wiced_bt_mesh_event_t *p_event);

/**
 * \brief The application can call this function to send Light HSL Saturation Set client message to the server.
 *
 * @param       p_event Mesh event with the information about the message that has been created by the app for unsolicited message.
 * @param       p_data Pointer to the data to send
 *
 * @return      None
 */
void wiced_bt_mesh_model_light_hsl_client_send_saturation_set(wiced_bt_mesh_event_t *p_event, wiced_bt_mesh_light_hsl_saturation_set_t* p_data);

/**
 * \brief The application can call this function to send Light HSL Target Get message to the server.
 *
 * @param       p_event Mesh event with the information about the message that has been created by the app for unsolicited message.
 *
 * @return      None
 */
void wiced_bt_mesh_model_light_hsl_client_send_target_get(wiced_bt_mesh_event_t *p_event);

/**
 * \brief The application can call this function to send Light HSL Get Default client message to the server.
 *
 * @param       p_event Mesh event with the information about the message that has been created by the app for unsolicited message.
 * @param       p_data Pointer to the data to send
 *
 * @return      None
 */
void wiced_bt_mesh_model_light_hsl_client_send_default_get(wiced_bt_mesh_event_t *p_event);

/**
 * \brief The application can call this function to send Light HSL Set Default client message to the server.
 *
 * @param       p_event Mesh event with the information about the message that has been created by the app for unsolicited message.
 * @param       p_data Pointer to the data to send
 *
 * @return      None
 */
void wiced_bt_mesh_model_light_hsl_client_send_default_set(wiced_bt_mesh_event_t *p_event, wiced_bt_mesh_light_hsl_default_data_t* p_data);

 /**
 * \brief The application can call this function to send Light HSL Range Get client message to the server.
 *
 * @param       p_event Mesh event with the information about the message that has been created by the app for unsolicited message.
 * @param       p_data Pointer to the data to send
 *
 * @return      None
 */
 void wiced_bt_mesh_model_light_hsl_client_send_range_get(wiced_bt_mesh_event_t *p_event);

 /**
 * \brief The application can call this function to send Light HSL Range Set client message to the server.
 *
 * @param       p_event Mesh event with the information about the message that has been created by the app for unsolicited message.
 * @param       p_data Pointer to the data to send
 *
 * @return      None
 */
void wiced_bt_mesh_model_light_hsl_client_send_range_set(wiced_bt_mesh_event_t *p_event, wiced_bt_mesh_light_hsl_range_set_data_t* p_data);

/**
 * \brief Light xyL Server callback is called by the Mesh Models library on receiving a message from the peer
 * Application or higher level model should be able process messages for the Light xyL server model as well as
 * bound models.
 *
 * @param       event that application should process (see @ref LIGHT_XYL_EVENT Light xyL Events)
 * @param       p_event information about the message received.  The same pointer should be used in the reply if required.
 * @param       p_data pointer to the on/off state data
 *
 * @return      None
 */
typedef void(wiced_bt_mesh_light_xyl_server_callback_t)(uint16_t event, wiced_bt_mesh_event_t *p_event, void *p_data);

/**
 * \brief Light xyL Model initialization
 * \details A Mesh application which contains Light xyL Server (for example a colored dimmable bulb) should call this function for each element
 * where the Light xyL Server is present (for example for each bulb).
 *
 * @param       element_idx Device element to where model is used
 * @param       p_callback Application or Parent model callback that will be executed on receiving a message
 * @param       is_provisioned If TRUE, the application is being restarted after being provisioned or after a power loss. If FALSE the model cleans up NVRAM on startup.
 *
 * @return      WICED_TRUE if initialization was successful
 */
void wiced_bt_mesh_model_light_xyl_server_init(uint8_t element_idx, wiced_bt_mesh_light_xyl_server_callback_t *p_callback, wiced_bt_mesh_light_xyl_xy_settings_t *p_xy_settings, wiced_bool_t is_provisioned);

/**
 * \brief Light xyL Server Message Handler
 * \details The mesh core library calls this function for each message received.  The function returns WICED_TRUE if the message is destined for this specific model and successfully processed, and returns WICED_FALSE otherwise.
 *
 * @param       p_event Mesh event with information about received message.
 * @param       p_data Pointer to the data portion of the message
 * @param       data_len Length of the data in the message
 *
 * @return      WICED_TRUE if the message is for this company ID/Model/Element Index combination, WICED_FALSE otherwise.
 */
wiced_bool_t wiced_bt_mesh_model_light_xyl_server_message_handler(wiced_bt_mesh_event_t *p_event, uint8_t *p_data, uint16_t data_len);

/**
 * \brief Light xyL Server Scene Store Handler
 * \details The mesh Scene Server calls this function so that the module can store required data for the scene.
 *
 * @param       element_idx Element index for which scene information is being stored
 * @param       p_buffer Pointer to a buffer where data should be stored
 * @param       buf_size Maximum amount of data a model can store
 *
 * @return      Number of bytes the model wrote into the provided buffer
 */
uint16_t wiced_bt_mesh_model_light_xyl_server_scene_store_handler(uint8_t element_idx, uint8_t *p_buffer, uint16_t buffer_len);

/**
 * \brief Light xyL Server Scene Store Handler
 * \details When Scene Server receives Scene Recall message it calls this function with data previously stored in the NVRAM.
 *
 * @param       element_idx Element index for which scene information is being recalled
 * @param       p_buffer Pointer to a buffer where model's data is located
 * @param       buf_size Maximum amount of data a model can get from the buffer
 *
 * @return      Number of bytes the model read from the provided buffer
 */
uint16_t wiced_bt_mesh_model_light_xyl_server_scene_recall_handler(uint8_t element_idx, uint8_t *p_buffer, uint16_t buffer_len, uint32_t transition_time, uint32_t delay);

/**
 * \brief An application or a a parent model can call this function to send Light xyL Status message to the Client. The mesh event should
 * contain information if this is a reply or an unsolicited message.
 *
 * @param       p_event Mesh event with the information about the message that has been received in the request, or created by the app for unsolicited message.
 * @param       p_data Pointer to the status data
 *
 * @return      None
 */
void wiced_bt_mesh_model_light_xyl_server_send_status(wiced_bt_mesh_event_t *p_event, wiced_bt_mesh_light_xyl_status_data_t *p_data);

/**
 * \brief An application or a a parent model can call this function to send Light xyL Range Status message to the Client. The mesh event should
 * contain information if this is a reply or an unsolicited message.
 *
 * @param       p_event Mesh event with the information about the message that has been received in the request, or created by the app for unsolicited message.
 * @param       p_data Pointer to the status data
 *
 * @return      None
 */
void wiced_bt_mesh_model_light_xyl_server_send_range_status(wiced_bt_mesh_event_t *p_event, wiced_bt_mesh_light_xyl_range_status_data_t *p_status);

/**
 * \brief An application or a a parent model can call this function to send Light xyL Default Status message to the Client. The mesh event should
 * contain information if this is a reply or an unsolicited message.
 *
 * @param       p_event Mesh event with the information about the message that has been received in the request, or created by the app for unsolicited message.
 * @param       p_data Pointer to the status data
 *
 * @return      None
 */
void wiced_bt_mesh_model_light_xyl_server_send_default_status(wiced_bt_mesh_event_t *p_event, wiced_bt_mesh_light_xyl_default_status_data_t *p_data);

/**
 * \brief Light xyL Setup Server callback is called by the Mesh Models library on receiving a message from the peer
 * Application or higher level model should be able process messages for the Light xyL Setup server model.
 *
 * @param       event that application should process (see @ref LIGHT_XYL_EVENT "Light xyL Events")
 * @param       p_event information about the message received.  The same pointer should be used in the reply if required.
 * @param       p_data pointer to the data structure specific to the event
 *
 * @return      None
 */
typedef void (wiced_bt_mesh_light_xyl_setup_server_callback_t)(uint16_t event, wiced_bt_mesh_event_t *p_event, void *p_data);

/**
 * \brief Light xyL Setup Server Model initialization
 * \details A Mesh application which contains Light xyL Setup Server (for example a dimmable bulb color bulb)
 * should call this function. 
 *
 * @param       lightness_element_idx Device element where lightness part of the xyL is located
 * @param       p_callback Application or Parent model callback that will be executed on receiving a message
 * @param       is_provisioned If TRUE, the application is being restarted after being provisioned or after a power loss. If FALSE the model cleans up NVRAM on startup.
 *
 * @return      WICED_TRUE if initialization was successful
 */
void wiced_bt_mesh_model_light_xyl_setup_server_init(uint8_t element_idx, wiced_bt_mesh_light_xyl_server_callback_t *p_callback, wiced_bt_mesh_light_xyl_xy_settings_t *p_xy_settings, wiced_bool_t is_provisioned);

/**
 * \brief Light xyL Setup Server Message Handler
 * \details The mesh core library calls this function for each message received.  The function returns WICED_TRUE if the message is destined for this specific model and successfully processed, and returns WICED_FALSE otherwise. 
 *
 * @param       p_event Mesh event with information about received message.
 * @param       p_data Pointer to the data portion of the message
 * @param       data_len Length of the data in the message
 *
 * @return      WICED_TRUE if the message is for this company ID/Model/Element Index combination, WICED_FALSE otherwise.
 */
wiced_bool_t wiced_bt_mesh_model_light_xyl_setup_server_message_handler(wiced_bt_mesh_event_t *p_event, uint8_t *p_data, uint16_t data_len);

/**
 * \brief Light xyL Client callback is called by the Mesh Models library on receiving a message from the peer
 *
 * @param       event that application should process (see @ref LIGHT_XYL_EVENT "Light xyL Events")
 * @param       p_event information about the message received.  The same pointer should be used in the reply if required.
 * @param       p_data pointer to the level data
 *
 * @return      None
 */
typedef void(wiced_bt_mesh_light_xyl_client_callback_t)(uint16_t event, wiced_bt_mesh_event_t *p_event, void *p_data);
 
/**
 * \brief Light xyL Client Message Handler
 * \details The mesh core library calls this function for each message received.  The function returns WICED_TRUE if the message is destined for this specific model and successfully processed, and returns WICED_FALSE otherwise.
 * The function parses the message and if appropriate calls the parent back to perform functionality.
 *
 * @param       p_event Mesh event with information about received message.
 * @param       p_data Pointer to the data portion of the message
 * @param       data_len Length of the data in the message
 *
 * @return      WICED_TRUE if the message is for this company ID/Model/Element Index combination, WICED_FALSE otherwise.
 */
wiced_bool_t wiced_bt_mesh_model_light_xyl_client_message_handler(wiced_bt_mesh_event_t *p_event, uint8_t *p_data, uint16_t data_len);
 
 /**
 * \brief Light xyL Client Model initialization
 *
 * @param       element_idx Device element to where model is used
 * @param       p_callback The application callback function that will be executed by the mesh models library when application action is required, or when a reply for the application request has been received.
 * @param       is_provisioned If TRUE, the application is being restarted after being provisioned or after a power loss. If FALSE the model cleans up NVRAM on startup.
 *
 * @return      None
 */
void wiced_bt_mesh_model_light_xyl_client_init(uint8_t element_idx, wiced_bt_mesh_light_xyl_client_callback_t *p_callback, wiced_bool_t is_provisioned);
 
/**
 * \brief The application can call this function to send Light xyL Get message to the server.
 *
 * @param       p_event Mesh event with the information about the message that has been created by the app for unsolicited message.
 *
 * @return      None
 */
void wiced_bt_mesh_model_light_xyl_client_send_get(wiced_bt_mesh_event_t *p_event);

/**
 * \brief The application can call this function to send Light xyL Set message to the server.
 *
 * @param       p_event Mesh event with the information about the message that has been created by the app for unsolicited message.
 *
 * @return      None
 */
void wiced_bt_mesh_model_light_xyl_client_send_set(wiced_bt_mesh_event_t *p_event, wiced_bt_mesh_light_xyl_set_t *p_data);

/**
 * \brief The application can call this function to send Light xyL Target Get message to the server.
 *
 * @param       p_event Mesh event with the information about the message that has been created by the app for unsolicited message.
 *
 * @return      None
 */
void wiced_bt_mesh_model_light_xyl_client_send_target_get(wiced_bt_mesh_event_t *p_event);
 
/**
 * \brief The application can call this function to send Light xyL Get Default client message to the server.
 *
 * @param       p_event Mesh event with the information about the message that has been created by the app for unsolicited message.
 * @param       p_data Pointer to the data to send
 *
 * @return      None
 */
void wiced_bt_mesh_model_light_xyl_client_send_default_get(wiced_bt_mesh_event_t *p_event);
 
/**
 * \brief The application can call this function to send Light xyL Set Default client message to the server.
 *
 * @param       p_event Mesh event with the information about the message that has been created by the app for unsolicited message.
 * @param       p_data Pointer to the data to send
 *
 * @return      None
 */
void wiced_bt_mesh_model_light_xyl_client_send_default_set(wiced_bt_mesh_event_t *p_event, wiced_bt_mesh_light_xyl_default_data_t* p_data);
 
/**
 * \brief The application can call this function to send Light xyL Range Get client message to the server.
 *
 * @param       p_event Mesh event with the information about the message that has been created by the app for unsolicited message.
 * @param       p_data Pointer to the data to send
 *
 * @return      None
 */
 void wiced_bt_mesh_model_light_xyl_client_send_range_get(wiced_bt_mesh_event_t *p_event);
 
/**
 * \brief The application can call this function to send Light xyL Range Set client message to the server.
 *
 * @param       p_event Mesh event with the information about the message that has been created by the app for unsolicited message.
 * @param       p_data Pointer to the data to send
 *
 * @return      None
 */
void wiced_bt_mesh_model_light_xyl_client_send_range_set(wiced_bt_mesh_event_t *p_event, wiced_bt_mesh_light_xyl_range_set_data_t* p_data);

/**
 * \brief Light Control (Light LC) Server callback is called by the Mesh Models library on receiving a message from the peer
 * Application or higher level model should be able process messages for the Light CC server model as well as
 * bound models including generic Light Lightness, Power On/Off, Generic Level and Generic On/Off.
 *
 * @param       event that application should process (see @ref LIGHT_LIGHTNESS_EVENT Light Lightness Events, @ref ONOFF_EVENT "On/Off Events, @ref LEVEL_EVENT "Level Events")
 * @param       p_event information about the message received.  The same pointer should be used in the reply if required.
 * @param       p_data pointer to the on/off state data
 *
 * @return      None
 */
typedef void(wiced_bt_mesh_light_lc_server_callback_t)(uint16_t event, wiced_bt_mesh_event_t *p_event, void *p_data);

/**
 * \brief Light Control Model initialization
 * \details A Mesh application which contains Light Control Server (for example a dimmable bulb) should call this function for each element where the Light Control Server is present (for example for each bulb).
 *
 * @param       element_idx Device element to where model is used
 * @param       p_callback Application or Parent model callback that will be executed on receiving a message
 * @param       is_provisioned If TRUE, the application is being restarted after being provisioned or after a power loss. If FALSE the model cleans up NVRAM on startup.
 *
 * @return      WICED_TRUE if initialization was successful
 */
void wiced_bt_mesh_model_light_lc_server_init(uint8_t element_idx, wiced_bt_mesh_light_lc_server_callback_t *p_callback, wiced_bool_t is_provisioned);

/**
 * \brief Light LC Server Message Handler
 * \details The mesh core library calls this function for each message received.  The function returns WICED_TRUE if the message is destined for this specific model and successfully processed, and returns WICED_FALSE otherwise.
 *
 * @param       p_event Mesh event with information about received message.
 * @param       p_data Pointer to the data portion of the message
 * @param       data_len Length of the data in the message
 *
 * @return      WICED_TRUE if the message is for this company ID/Model/Element Index combination, WICED_FALSE otherwise.
*/
wiced_bool_t wiced_bt_mesh_model_light_lc_server_message_handler(wiced_bt_mesh_event_t *p_event, uint8_t *p_data, uint16_t data_len);

/**
 * \brief An application or a a parent model can call this function to send Light Lightness Status message to the Client. The mesh event should
 * contain information if this is a reply or an unsolicited message.
 *
 * @param       p_event Mesh event with the information about the message that has been received in the request, or created by the app for unsolicited message.
 * @param       p_data Pointer to the status data
 *
 * @return      None
*/
void wiced_bt_mesh_model_light_lc_server_send_status(wiced_bt_mesh_event_t *p_event, wiced_bt_mesh_light_lightness_status_data_t *p_data);

/**
 * \brief Light LC Server Mode Changed 
 * \details Application should call this function when LC Mode is changed locally
 *
 * @param       element_idx Element index where the LC Model is located
 * @param       p_status Pointer to the structure with the new mode
 *
 * @return      None
*/
void wiced_bt_mesh_model_light_lc_mode_changed(wiced_bt_mesh_event_t *p_event, wiced_bt_mesh_light_lc_mode_set_data_t *p_status);

/**
 * \brief Light LC Server Occupancy Mode Changed 
 * \details Application should call this function when LC Occupancy Mode is changed locally
 *
 * @param       element_idx Element index where the LC Model is located
 * @param       p_status Pointer to the structure with the new mode
 *
 * @return      None
*/
void wiced_bt_mesh_model_light_lc_occupancy_mode_changed(wiced_bt_mesh_event_t *p_event, wiced_bt_mesh_light_lc_occupancy_mode_set_data_t *p_status);

/**
 * \brief Light LC Server Occupancy Detected 
 * \details Application should call this function when LC Occupancy is detected locally
 *
 * @param       element_idx Element index where the LC Model is located
 *
 * @return      None
*/
void wiced_bt_mesh_model_light_lc_occupancy_detected(wiced_bt_mesh_event_t *p_event);

/**
 * \brief Light LC Server OnOff Changed 
 * \details Application should call this function when LC Light OnOff is changed locally
 *
 * @param       element_idx Element index where the LC Model is located
 * @param       p_status Pointer to the structure with the new Light OnOff State
 *
 * @return      None
*/
void wiced_bt_mesh_model_light_lc_onoff_changed(wiced_bt_mesh_event_t *p_event, wiced_bt_mesh_light_lc_light_onoff_status_data_t *p_status);

/**
 * \brief Light LC Property Changed 
 * \details Application should call this function when an LC LC property is changed locally
 *
 * @param       element_idx Element index where the LC Model is located
 * @param       p_status Pointer to the structure with the new property id and value
 *
 * @return      None
*/
void wiced_bt_mesh_model_light_lc_property_changed(wiced_bt_mesh_event_t *p_event, wiced_bt_mesh_light_lc_property_set_data_t *p_status);

/**
 * \brief Light LC Server Scene Store Handler
 * \details The mesh Scene Server calls this function so that the module can store required data for the scene.
 *
 * @param       element_idx Element index for which scene information is being stored
 * @param       p_buffer Pointer to a buffer where data should be stored
 * @param       buf_size Maximum amount of data a model can store
 *
 * @return      Number of bytes the model wrote into the provided buffer
 */
uint16_t wiced_bt_mesh_model_light_lc_server_scene_store_handler(uint8_t element_idx, uint8_t *p_buffer, uint16_t buffer_len);

/**
 * \brief Light LC Server Scene Store Handler
 * \details When Scene Server receives Scene Recall message it calls this function with data previously stored in the NVRAM.
 *
 * @param       element_idx Element index for which scene information is being recalled
 * @param       p_buffer Pointer to a buffer where model's data is located
 * @param       buf_size Maximum amount of data a model can get from the buffer
 *
 * @return      Number of bytes the model read from the provided buffer
 */
uint16_t wiced_bt_mesh_model_light_lc_server_scene_recall_handler(uint8_t element_idx, uint8_t *p_buffer, uint16_t buffer_len, uint32_t transition_time, uint32_t delay);

/**
 * \brief Light Control Setup Server callback is called by the Mesh Models library on receiving a message from the peer
 * Application or higher level model should be able process messages for the Light LC Setup server model as well as
 * bound models including generic Light Lightness, Power On/Off, Generic Level and Generic On/Off.
 *
 * @param       event that application should process (see @ref LIGHT_LIGHTNESS_EVENT Light Lightness Events, @ref ONOFF_EVENT "On/Off Events, @ref LEVEL_EVENT "Level Events")
 * @param       p_event information about the message received.  The same pointer should be used in the reply if required.
 * @param       p_data pointer to the data structure specific to the event
 *
 * @return      None
 */
typedef void (wiced_bt_mesh_light_lc_setup_server_callback_t)(uint16_t event, wiced_bt_mesh_event_t *p_event, void *p_data);

/**
 * \brief Light LC Setup Server Model initialization
 * \details A Mesh application which contains Light LC Setup Server (for example a dimmable bulb) should call this function for each element where the Light LC Server is present (for example for each bulb).
 *
 * @param       element_idx Device element to where model is used
 * @param       p_callback Application or Parent model callback that will be executed on receiving a message
 * @param       is_provisioned If TRUE, the application is being restarted after being provisioned or after a power loss. If FALSE the model cleans up NVRAM on startup.
 *
 * @return      WICED_TRUE if initialization was successful
 */
void wiced_bt_mesh_model_light_lc_setup_server_init(uint8_t element_idx, wiced_bt_mesh_light_lc_setup_server_callback_t *p_callback, wiced_bool_t is_provisioned);

/**
 * \brief Light LC Setup Server Message Handler
 * \details The mesh core library calls this function for each message received.  The function returns WICED_TRUE if the message is destined for this specific model and successfully processed, and returns WICED_FALSE otherwise. 
 *
 * @param       p_event Mesh event with information about received message.
 * @param       p_data Pointer to the data portion of the message
 * @param       data_len Length of the data in the message
 *
 * @return      WICED_TRUE if the message is for this company ID/Model/Element Index combination, WICED_FALSE otherwise.
*/
wiced_bool_t wiced_bt_mesh_model_light_lc_setup_server_message_handler(wiced_bt_mesh_event_t *p_event, uint8_t *p_data, uint16_t data_len);

/**
 * \brief Light Control Client callback is called by the Mesh Models library on receiving a message from the peer
 *
 * @param       event that application should process (see @ref LIGHT_LC_EVENT "Mesh Light LC Events")
 * @param       p_event information about the message received.  The same pointer should be used in the reply if required.
 * @param       p_data pointer to the data to send
 *
 * @return      None
 */
typedef void(wiced_bt_mesh_light_lc_client_callback_t)(uint16_t event, wiced_bt_mesh_event_t *p_event, void *p_data);

/**
 * \brief Light Control Client Model initialization
 *
 * @param       element_idx Device element to where model is used
 * @param       p_callback The application callback function that will be executed by the mesh models library when application action is required, or when a reply for the application request has been received.
 * @param       is_provisioned If TRUE, the application is being restarted after being provisioned or after a power loss. If FALSE the model cleans up NVRAM on startup.
 *
 * @return      None
 */
void wiced_bt_mesh_model_light_lc_client_init(uint8_t element_idx, wiced_bt_mesh_power_onoff_client_callback_t *p_callback, wiced_bool_t is_provisioned);

/**
 * \brief Light Control Client Message Handler
 * \details The mesh core library calls this function for each message received.  The function returns WICED_TRUE if the message is destined for this specific model and successfully processed, and returns WICED_FALSE otherwise.
 *
 * @param       p_event Mesh event with information about received message.
 * @param       p_data Pointer to the data portion of the message
 * @param       data_len Length of the data in the message
 *
 * @return      WICED_TRUE if the message is for this company ID/Model/Element Index combination, WICED_FALSE otherwise.
*/
wiced_bool_t wiced_bt_mesh_model_light_lc_client_message_handler(wiced_bt_mesh_event_t *p_event, uint8_t *p_data, uint16_t data_len);

/**
 * \brief The application can call this function to send the Light LC Mode Get client message to the server.
 *
 * @param       p_event Mesh event with the information about the message that has been created by the app for unsolicited message.
 *
 * @return      None
 */
void wiced_bt_mesh_model_light_lc_client_send_mode_get(wiced_bt_mesh_event_t *p_event);

/**
 * \brief The application can call this function to send Light LC Mode Set client message to the server.
 *
 * @param       p_event Mesh event with the information about the message that has been created by the app for unsolicited message.
 * @param       p_data Pointer to the data to send
 *
 * @return      None
 */
void wiced_bt_mesh_model_light_lc_client_send_mode_set(wiced_bt_mesh_event_t *p_event, wiced_bt_mesh_light_lc_mode_set_data_t* p_data);

/**
 * \brief The application can call this function to send the Light LC Occupancy Mode Get client message to the server.
 *
 * @param       p_event Mesh event with the information about the message that has been created by the app for unsolicited message.
 *
 * @return      None
 */
void wiced_bt_mesh_model_light_lc_client_send_occupancy_mode_get(wiced_bt_mesh_event_t *p_event);

/**
 * \brief The application can call this function to send Light LC Occupancy Mode Set client message to the server.
 *
 * @param       p_event Mesh event with the information about the message that has been created by the app for unsolicited message.
 * @param       p_data Pointer to the data to send
 *
 * @return      None
 */
void wiced_bt_mesh_model_light_lc_client_send_occupancy_mode_set(wiced_bt_mesh_event_t *p_event, wiced_bt_mesh_light_lc_occupancy_mode_set_data_t* p_data);

/**
 * \brief The application can call this function to send the Light LC OnOff Get client message to the server.
 *
 * @param       p_event Mesh event with the information about the message that has been created by the app for unsolicited message.
 *
 * @return      None
 */
void wiced_bt_mesh_model_light_lc_client_send_light_onoff_get(wiced_bt_mesh_event_t *p_event);

/**
 * \brief The application can call this function to send Light LC Occupancy Mode Set client message to the server.
 *
 * @param       p_event Mesh event with the information about the message that has been created by the app for unsolicited message.
 * @param       p_data Pointer to the data to send
 *
 * @return      None
 */
void wiced_bt_mesh_model_light_lc_client_send_light_onoff_set(wiced_bt_mesh_event_t *p_event, wiced_bt_mesh_light_lc_light_onoff_set_data_t* p_data);

/**
* \brief The application can call this function to send Light LC Property Get client message to the server.
*
* @param       p_event Mesh event with the information about the message that has been created by the app for unsolicited message.
* @param       p_data Pointer to the data to send containing property ID to retrieve
*
* @return      None
*/
void wiced_bt_mesh_model_light_lc_client_send_property_get(wiced_bt_mesh_event_t *p_event, wiced_bt_mesh_light_lc_property_get_data_t *p_data);

/**
* \brief The application can call this function to send Light LC Property Set client message to the server.
*
* @param       p_event Mesh event with the information about the message that has been created by the app for unsolicited message.
* @param       p_data Pointer to the data to send containing property ID and the value.
*
* @return      None
*/
void wiced_bt_mesh_model_light_lc_client_send_property_set(wiced_bt_mesh_event_t *p_event, wiced_bt_mesh_light_lc_property_set_data_t *p_data);

/*
 * \brief Sensor server callback is called by the Mesh Models library on receiving a message from the peer
 *
 * @param       event that application should process (see @ref SENSOR_EVENT "Sensor Events")
 * @param       p_event information about the message received.  The same pointer should be used in the reply if required.
 * @param       p_data pointer to the level data
 *
 * @return      None
 */
typedef void(wiced_bt_mesh_sensor_server_callback_t)(uint16_t event, wiced_bt_mesh_event_t *p_event, void *p_data);

/**
 * \brief Sensor Senosr Model initialization
 *
 * @param       element_idx Device element to where model is used
 * @param       p_callback The application callback function that will be executed by the mesh models library when application action is required, or when a reply for the application request has been received.
 * @param       is_provisioned If TRUE, the application is being restarted after being provisioned or after a power loss. If FALSE the model cleans up NVRAM on startup.
 *
 * @return      None
 */
void wiced_bt_mesh_model_sensor_server_init(uint8_t element_idx, wiced_bt_mesh_sensor_server_callback_t *p_callback, wiced_bool_t is_provisioned);

/**
 * \brief Sensor server Message Handler
 * \details An application or a parent model can call this models library function to process a message received
 * from the Sensor Client device. The function parses the message and if appropriate calls the parent back
 * to perform functionality.
 *
 * @param       p_event Mesh event with information about received message.
 * @param       p_data Pointer to the data portion of the message
 * @param       data_len Length of the data in the message
 *
 * @return      WICED_TRUE if the message is for this company ID/Model/Element Index combination, WICED_FALSE otherwise.
*/
wiced_bool_t wiced_bt_mesh_model_sensor_server_message_handler(wiced_bt_mesh_event_t *p_event, uint8_t *p_data, uint16_t data_len);

/**
 * \brief Sensor setup server Message Handler
 * \details An application or a parent model can call this models library function to process a message received
 * from the Sensor Client device. The function parses the message and if appropriate calls the parent back
 * to perform functionality.
 *
 * @param       p_event Mesh event with information about received message.
 * @param       p_data Pointer to the data portion of the message
 * @param       data_len Length of the data in the message
 *
 * @return      WICED_TRUE if the message is for this company ID/Model/Element Index combination, WICED_FALSE otherwise.
*/
wiced_bool_t wiced_bt_mesh_model_sensor_setup_server_message_handler(wiced_bt_mesh_event_t *p_event, uint8_t *p_data, uint16_t data_len);

/**
 * \brief Sensor Series Status send status
 *
 * @param       p_event Mesh event with the information about the message that has been received in the request, or created by the app for unsolicited message.
 * @param       series_status_data pointer to the series data
 *
 * @return      None
 */
void wiced_bt_mesh_model_sensor_server_series_status_send(wiced_bt_mesh_event_t *p_event, wiced_bt_mesh_sensor_series_status_data_t *series_status_data);

/**
 * \brief Sensor Column Status send status
 *
 * @param       p_event Mesh event with the information about the message that has been received in the request, or created by the app for unsolicited message.
 * @param       column_status_data pointer to the column data
 * @return      None
 */
void wiced_bt_mesh_model_sensor_server_column_status_send(wiced_bt_mesh_event_t *p_event, wiced_bt_mesh_sensor_column_status_data_t *column_status_data);

/**
 * \brief Sensor Status send
 *
 * @param       p_event Mesh event with the information about the message that has been received in the request, or created by the app for unsolicited message.
 * @param       status_data pointer to the data from sensor server
 *
 * @return      None
 */
void wiced_bt_mesh_model_sensor_server_status_send(wiced_bt_mesh_event_t *p_event, wiced_bt_mesh_sensor_status_data_t *status_data);


/**
 * \brief Sensor Setting status
 *
 * @param       p_event Mesh event with the information about the message that has been received in the request, or created by the app for unsolicited message.
 * @param       setting_status pointer to setting status values
 *
 * @return      None
 */
void wiced_bt_mesh_model_sensor_server_setting_status_send(wiced_bt_mesh_event_t *p_event, wiced_bt_mesh_sensor_setting_status_data_t *setting_status );

/**
 * \brief Sensor Cadence status
 *
 * @param       p_event Mesh event with the information about the message that has been received in the request, or created by the app for unsolicited message.
 * @param       cadence_status pointer to cadence status values
 *
 * @return      None
 */
void wiced_bt_mesh_model_sensor_server_cadence_status_send(wiced_bt_mesh_event_t *p_event, wiced_bt_mesh_sensor_cadence_status_data_t *cadence_status );

/**
 * \brief Sensor Client callback is called by the Mesh Models library on receiving a message from the peer
 *
 * @param       event that application should process (see @ref SENSOR_EVENT "Sensor Events")
 * @param       p_event information about the message received.  The same pointer should be used in the reply if required.
 * @param       p_data pointer to the level data
 *
 * @return      None
 */
typedef void(wiced_bt_mesh_sensor_client_callback_t)(uint16_t event, wiced_bt_mesh_event_t *p_event, void *p_data);

/**
 * \brief Sensor Client Model initialization
 *
 * @param       element_idx Device element to where model is used
 * @param       p_callback The application callback function that will be executed by the mesh models library when application action is required, or when a reply for the application request has been received.
 * @param       is_provisioned If TRUE, the application is being restarted after being provisioned or after a power loss. If FALSE the model cleans up NVRAM on startup.
 *
 * @return      None
 */
void wiced_bt_mesh_model_sensor_client_init(uint8_t element_idx, wiced_bt_mesh_sensor_client_callback_t *p_callback, wiced_bool_t is_provisioned);

/**
 * \brief Sensor client Message Handler
 * \details An application or a parent model can call this models library function to process a message received
 * from the Sensor Server device. The function parses the message and if appropriate calls the parent back
 * to perform functionality.
 *
 * @param       p_event Mesh event with information about received message.
 * @param       p_data Pointer to the data portion of the message
 * @param       data_len Length of the data in the message
 *
 * @return      WICED_TRUE if the message is for this company ID/Model/Element Index combination, WICED_FALSE otherwise.
*/
wiced_bool_t wiced_bt_mesh_model_sensor_client_message_handler(wiced_bt_mesh_event_t *p_event, uint8_t *p_data, uint16_t data_len);

/**
 * \brief Sensor Descriptor get
 *
 * @param       p_event Mesh event with the information about the message that has been received in the request, or created by the app for unsolicited message.
 * @param       desc_get_data pointer to descriptor get values
 *
 * @return      None
 */
void wiced_bt_mesh_model_sensor_client_descriptor_send_get(wiced_bt_mesh_event_t *p_event, wiced_bt_mesh_sensor_get_t *desc_get_data);

/**
 * \brief Sensor Data get
 *
 * @param       p_event Mesh event with the information about the message that has been received in the request, or created by the app for unsolicited message.
 * @param       sensor_get pointer to the sensor data to be received
 *
 * @return      None
 */
void wiced_bt_mesh_model_sensor_client_sensor_send_get(wiced_bt_mesh_event_t *p_event, wiced_bt_mesh_sensor_get_t *sensor_get);

/**
 * \brief Sensor Column get
 *
 * @param       p_event Mesh event with the information about the message that has been received in the request, or created by the app for unsolicited message.
 * @param       column_data pointer to data portion of the sensor column get message
 *
 * @return      None
 */
void wiced_bt_mesh_model_sensor_client_sensor_column_send_get(wiced_bt_mesh_event_t *p_event, wiced_bt_mesh_sensor_column_get_data_t *column_data);

/**
 * \brief Sensor Series get
 *
 * @param       p_event Mesh event with the information about the message that has been received in the request, or created by the app for unsolicited message.
 * @param       series_data pointer to data portion of the sensor series get message
 *
 * @return      None
 */
void wiced_bt_mesh_model_sensor_client_sensor_series_send_get(wiced_bt_mesh_event_t *p_event, wiced_bt_mesh_sensor_series_get_data_t *series_data);

/**
 * \brief Sensor Setting get
 *
 * @param       p_event Mesh event with the information about the message that has been received in the request, or created by the app for unsolicited message.
 * @param       setting_data pointer to data portion of the sensor setting get message
 *
 * @return      None
 */
void wiced_bt_mesh_model_sensor_client_sensor_setting_send_get(wiced_bt_mesh_event_t *p_event, wiced_bt_mesh_sensor_setting_get_data_t *setting_data);

/**
 * \brief Sensor Settings get
 *
 * @param       p_event Mesh event with the information about the message that has been received in the request, or created by the app for unsolicited message.
 * @param       settings_data pointer to data portion of the sensor settings get message
 *
 * @return      None
 */
void wiced_bt_mesh_model_sensor_client_sensor_settings_send_get(wiced_bt_mesh_event_t *p_event, wiced_bt_mesh_sensor_get_t *settings_data);

/**
 * \brief Sensor Setting set
 *
 * @param       p_event Mesh event with the information about the message that has been received in the request, or created by the app for unsolicited message.
 * @param       setting_data pointer to data portion of the sensor settings get message
 *
 * @return      None
 */
void wiced_bt_mesh_model_sensor_client_sensor_setting_send_set(wiced_bt_mesh_event_t *p_event, wiced_bt_mesh_sensor_setting_set_data_t *setting_data);

/**
 * \brief Sensor Cadence get
 *
 * @param       p_event Mesh event with the information about the message that has been received in the request, or created by the app for unsolicited message.
 * @param       cadence_data pointer to data portion of the cadence get message
 *
 * @return      None
 */
void wiced_bt_mesh_model_sensor_client_sensor_cadence_send_get(wiced_bt_mesh_event_t *p_event, wiced_bt_mesh_sensor_get_t *cadence_data);

/**
 * \brief Sensor Cadence set
 *
 * @param       p_event Mesh event with the information about the message that has been received in the request, or created by the app for unsolicited message.
 * @param       cadence_set pointer to data portion of the cadence set message
 *
 * @return      None
 */
void wiced_bt_mesh_model_sensor_client_sensor_cadence_send_set(wiced_bt_mesh_event_t *p_event, wiced_bt_mesh_sensor_cadence_set_data_t *cadence_set);

/**
 * \brief Scene Server callback is called by the Mesh Models library on receiving a message from the peer
 *
 * @param       event that application should process (see @ref SCENE_EVENT "Scene Events")
 * @param       p_event information about the message received.  The same pointer should be used in the reply if required.
 * @param       p_data pointer to the data structure
 *
 * @return      None
 */
typedef void(wiced_bt_mesh_scene_server_callback_t)(uint16_t event, wiced_bt_mesh_event_t *p_event, void *p_data);

/**
 * \brief Scene Server Model initialization
 *
 * @param       p_callback Application or Parent model callback that will be executed on receiving a message
 * @param       is_provisioned If TRUE, the application is being restarted after being provisioned or after a power loss. If FALSE the model cleans up NVRAM on startup.
 *
 * @return      None
 */
void wiced_bt_mesh_model_scene_server_init(wiced_bt_mesh_scene_server_callback_t *p_callback, wiced_bool_t is_provisioned);

/**
 * \brief Scene Server Message Handler
 * \details The mesh core library calls this function for each message received.  The function returns WICED_TRUE if the message is destined for this specific model and successfully processed, and returns WICED_FALSE otherwise.
 * The function parses the message and if appropriate calls the parent back to perform functionality.
 *
 * @param       p_event Mesh event with information about received message.
 * @param       p_data Pointer to the data portion of the message
 * @param       data_len Length of the data in the message
 *
 * @return      WICED_TRUE if the message is for this company ID/Model/Element Index combination, WICED_FALSE otherwise.
*/
wiced_bool_t wiced_bt_mesh_model_scene_server_message_handler(wiced_bt_mesh_event_t *p_event, uint8_t *p_data, uint16_t data_len);

/**
 * \brief Scene Setup Server Message Handler
 * \details The mesh core library calls this function for each message received.  The function returns WICED_TRUE if the message is destined for this specific model and successfully processed, and returns WICED_FALSE otherwise.
 * The function parses the message and if appropriate calls the parent back to perform functionality.
 *
 * @param       p_event Mesh event with information about received message.
 * @param       p_data Pointer to the data portion of the message
 * @param       data_len Length of the data in the message
 *
 * @return      WICED_TRUE if the message is for this company ID/Model/Element Index combination, WICED_FALSE otherwise.
*/
wiced_bool_t wiced_bt_mesh_model_scene_setup_server_message_handler(wiced_bt_mesh_event_t *p_event, uint8_t *p_data, uint16_t data_len);

/**
 * \brief Scene Data Changed
 * The Scene Data Changed function should be called when any data stored with a scene has changed
 */
void wiced_bt_mesh_model_scene_data_changed(uint8_t element_idx);

/**
 * \brief Scene Client callback is called by the Mesh Models library on receiving a message from the peer
 *
 * @param       event that application should process (see @ref SCENE_EVENT "Scene Events")
 * @param       p_event information about the message received.  The same pointer should be used in the reply if required.
 * @param       p_data pointer to the data structure
 *
 * @return      None
 */
typedef void(wiced_bt_mesh_scene_client_callback_t)(uint16_t event, wiced_bt_mesh_event_t *p_event, void *p_data);

/**
 * \brief Scene Client Model initialization
 *
 * @param       element_idx Device element to where model is used
 * @param       p_callback Application or Parent model callback that will be executed on receiving a message
 * @param       is_provisioned If TRUE, the application is being restarted after being provisioned or after a power loss. If FALSE the model cleans up NVRAM on startup.
 *
 * @return      None
 */
void wiced_bt_mesh_model_scene_client_init(uint8_t element_idx, wiced_bt_mesh_scene_client_callback_t *p_callback, wiced_bool_t is_provisioned);

/**
 * \brief Scene client Message Handler
 * \details An application or a parent model can call this models library function to process a message received
 * from the Scene Server device. The function parses the message and if appropriate calls the parent back
 * to perform functionality.
 *
 * @param       p_event Mesh event with information about received message.
 * @param       p_data Pointer to the data portion of the message
 * @param       data_len Length of the data in the message
 *
 * @return      WICED_TRUE if the message is for this company ID/Model/Element Index combination, WICED_FALSE otherwise.
*/
wiced_bool_t wiced_bt_mesh_model_scene_client_message_handler(wiced_bt_mesh_event_t *p_event, uint8_t *p_data, uint16_t data_len);

/**
 * \brief Scene Client send request
 * \details Application can call Scene Client send request function to send Scene Store/Recall/Get/Delete client message to the server.
 *
 * @param       p_event Mesh event with the information about the message that has been received in the request, or created by the app for unsolicited message.
 * @param       p_request pointer to a data structure containing request type and scene number
 *
 * @return      None
 */
void wiced_bt_mesh_model_scene_client_send_request(wiced_bt_mesh_event_t *p_event, wiced_bt_mesh_scene_request_t *p_request);

/**
 * \brief Scene Client Send Recall
 * \details Application can call Scene Client Send Recall function to send Scene Recall client message to the server.
 *
 * @param       p_event Mesh event with the information about the message that has been received in the request, or created by the app for unsolicited message.
 * @param       p_request pointer to a data structure containing request type and scene number
 *
 * @return      None
 */
void wiced_bt_mesh_model_scene_client_send_recall(wiced_bt_mesh_event_t *p_event, wiced_bt_mesh_scene_recall_t *p_request);

 /**
 * \brief Scene Client Send Register Get
 * \details Application can call Scene Client Send Register Get function to send Scene Register Get client message to the server.
 *
 * @param       p_event Mesh event with the information about the message that has been received in the request, or created by the app for unsolicited message.
 */
void wiced_bt_mesh_model_scene_client_send_register_get(wiced_bt_mesh_event_t *p_event);
/**
 * \brief Time Server callback is called by the Mesh Models library on receiving a message from the peer
 *
 * @param       event that application should process (see @ref TIME_EVENT "Time Events")
 * @param       p_data pointer to the RTC Time data structure
 *
 * @return      None
 */
typedef void(wiced_bt_mesh_time_server_callback_t)(uint16_t event, wiced_bt_mesh_event_t *p_event, void *p_data);

/**
 * \brief Time Server Model initialization
 *
 * @param       p_callback Application or Parent model callback that will be executed on receiving a message
 *
 * @return      None
 */
void wiced_bt_mesh_model_time_server_init(wiced_bt_mesh_time_server_callback_t *p_callback, wiced_bool_t is_provisioned);

/**
 * \brief Time Server Message Handler
 * \details The mesh core library calls this function for each message received.  The function returns WICED_TRUE if the message is destined for this specific model and successfully processed, and returns WICED_FALSE otherwise.
 * The function parses the message and if appropriate calls the parent back to perform functionality.
 *
 * @param       p_event Mesh event with information about received message.
 * @param       p_data Pointer to the data portion of the message
 * @param       data_len Length of the data in the message
 *
 * @return      WICED_TRUE if the message is for this company ID/Model/Element Index combination, WICED_FALSE otherwise.
*/
wiced_bool_t wiced_bt_mesh_model_time_server_message_handler(wiced_bt_mesh_event_t *p_event, uint8_t *p_data, uint16_t data_len);

/**
 * \brief time Setup Server Message Handler
 * \details The mesh core library calls this function for each message received.  The function returns WICED_TRUE if the message is destined for this specific model and successfully processed, and returns WICED_FALSE otherwise.
 * The function parses the message and if appropriate calls the parent back to perform functionality.
 *
 * @param       p_event Mesh event with information about received message.
 * @param       p_data Pointer to the data portion of the message
 * @param       data_len Length of the data in the message
 *
 * @return      WICED_TRUE if the message is for this company ID/Model/Element Index combination, WICED_FALSE otherwise.
*/
wiced_bool_t wiced_bt_mesh_model_time_setup_server_message_handler(wiced_bt_mesh_event_t *p_event, uint8_t *p_data, uint16_t data_len);

/**
 * \brief Time server status send
 *
 * @param       p_event               Mesh event with the information about the message that has been received in the request, or created by the app for unsolicited message.
 * @param       p_time_zone_status_data pointer to data portion of the time zone status message
 *
 * @return      None
 */
void wiced_bt_mesh_model_time_server_status_send(wiced_bt_mesh_event_t *p_event, wiced_bt_mesh_time_state_msg_t *p_time_status_data);

/**
 * \brief Time zone status send
 *
 * @param       p_event          Mesh event with the information about the message that has been received in the request, or created by the app for unsolicited message.
 * @param       p_time_status_data pointer to data portion of the time status message
 *
 * @return      None
 */
void wiced_bt_mesh_model_time_server_zone_status_send(wiced_bt_mesh_event_t *p_event, wiced_bt_mesh_time_zone_status_t *p_time_zone_status_data);

/**
 * \brief Time TAI_UTC delta status send
 *
 * @param       p_event          Mesh event with the information about the message that has been received in the request, or created by the app for unsolicited message.
 * @param       p_time_delta_status_data pointer to data portion of the Time TAI_UTC delta status message
 *
 * @return      None
 */
void wiced_bt_mesh_model_time_server_tai_utc_delta_status_send(wiced_bt_mesh_event_t *p_event, wiced_bt_mesh_time_tai_utc_delta_status_t *p_time_delta_status_data);

/**
 * \brief Time Client callback is called by the Mesh Models library on receiving a message from the peer
 *
 * @param       event that application should process (see @ref TIME_EVENT "Time Events")
 * @param       p_event information about the message received.  The same pointer should be used in the reply if required.
 * @param       p_data pointer to the data structure
 *
 * @return      None
 */
typedef void(wiced_bt_mesh_time_client_callback_t)(uint16_t event, wiced_bt_mesh_event_t *p_event, void *p_data);

/**
 * \brief Time Client Model initialization
 *
 * @param       element_idx Device element to where model is used
 * @param       p_callback Application or Parent model callback that will be executed on receiving a message
 *
 * @return      None
 */
void wiced_bt_mesh_model_time_client_init(wiced_bt_mesh_time_client_callback_t *p_callback, wiced_bool_t is_provisioned);

/**
 * \brief Time Client Message Handler
 * \details The mesh core library calls this function for each message received.  The function returns WICED_TRUE if the message is destined for this specific model and successfully processed, and returns WICED_FALSE otherwise.
 * The function parses the message and if appropriate calls the parent back to perform functionality.
 *
 * @param       p_event Mesh event with information about received message.
 * @param       p_data Pointer to the data portion of the message
 * @param       data_len Length of the data in the message
 *
 * @return      WICED_TRUE if the message is for this company ID/Model/Element Index combination, WICED_FALSE otherwise.
*/
wiced_bool_t wiced_bt_mesh_model_time_client_message_handler(wiced_bt_mesh_event_t *p_event, uint8_t *p_data, uint16_t data_len);

/**
 * \brief Time Get send
 *
 * @param       p_event          Mesh event with the information about the message that has been received in the request, or created by the app for unsolicited message.
 *
 * @return      None
 */
void wiced_bt_mesh_model_time_client_time_get_send(wiced_bt_mesh_event_t *p_event);

/**
 * \brief Time Set send
 *
 * @param       p_event          Mesh event with the information about the message that has been received in the request, or created by the app for unsolicited message.
 * @param       p_time_set_data    pointer to data portion of the Time set message
 *
 * @return      None
 */
void wiced_bt_mesh_model_time_client_time_set_send(wiced_bt_mesh_event_t *p_event, wiced_bt_mesh_time_state_msg_t *p_time_set_data);

/**
 * \brief Time Zone Get send
 *
 * @param       p_event  Mesh event with the information about the message that has been received in the request, or created by the app for unsolicited message.
 *
 * @return      None
 */
void wiced_bt_mesh_model_time_client_time_zone_get_send(wiced_bt_mesh_event_t *p_event);

/**
 * \brief Time Zone Set send
 *
 * @param       p_event          Mesh event with the information about the message that has been received in the request, or created by the app for unsolicited message.
 * @param       p_set_data         pointer to data portion of the Time zone set message
 *
 * @return      None
 */
void wiced_bt_mesh_model_time_client_time_zone_set_send(wiced_bt_mesh_event_t *p_event, wiced_bt_mesh_time_zone_set_t *p_set_data);

/**
 * \brief Time TAI UTC delta Get send
 *
 * @param       p_event  Mesh event with the information about the message that has been received in the request, or created by the app for unsolicited message.
 *
 * @return      None
 */
void wiced_bt_mesh_model_time_client_tai_utc_delta_get_send(wiced_bt_mesh_event_t *p_event);

/**
 * \brief Time TAI UTC delta Set send
 *
 * @param       p_event          Mesh event with the information about the message that has been received in the request, or created by the app for unsolicited message.
 * @param       p_time_delta_set         pointer to data portion of the Time TAI UTC delta Set message
 *
 * @return      None
 */
void wiced_bt_mesh_model_time_client_tai_utc_delta_set_send(wiced_bt_mesh_event_t *p_event, wiced_bt_mesh_time_tai_utc_delta_set_t *p_time_delta_set);

/**
 * \brief Time Role get send
 *
 * @param       p_event  Mesh event with the information about the message that has been received in the request, or created by the app for unsolicited message.
 *
 * @return      None
 */
void wiced_bt_mesh_model_time_client_time_role_get_send(wiced_bt_mesh_event_t *p_event);

/**
 * \brief Time role Set send
 *
 * @param       p_event          Mesh event with the information about the message that has been received in the request, or created by the app for unsolicited message.
 * @param       p_set_data         pointer to data portion of the Time role Set message
 *
 * @return      None
 */
void wiced_bt_mesh_model_time_client_time_role_set_send(wiced_bt_mesh_event_t *p_event, wiced_bt_mesh_time_role_msg_t *p_set_data);


/**
 * \brief Scheduler Server callback is called by the Mesh Models library on receiving a message from the peer
 *
 * @param       event that application should process (see @ref SCHEDULER_EVENT "Scheduler Events")
 * @param       p_event information about the message received.  The same pointer should be used in the reply if required.
 * @param       p_data pointer to the data structure
 *
 * @return      None
 */
typedef void(wiced_bt_mesh_scheduler_server_callback_t)(uint16_t event, wiced_bt_mesh_event_t *p_event, void *p_data);

/**
 * \brief Scheduler Server Model initialization
 *
 * @param       p_callback Application or Parent model callback that will be executed on receiving a message
 * @param       is_provisioned If TRUE, the application is being restarted after being provisioned or after a power loss. If FALSE the model cleans up NVRAM on startup.
 *
 * @return      None
 */
void wiced_bt_mesh_model_scheduler_server_init(wiced_bt_mesh_scheduler_server_callback_t *p_callback, wiced_bool_t is_provisioned);

/**
 * \brief Scheduler Server Message Handler
 * \details The mesh core library calls this function for each message received.  The function returns WICED_TRUE if the message is destined for this specific model and successfully processed, and returns WICED_FALSE otherwise.
 * The function parses the message and if appropriate calls the parent back to perform functionality.
 *
 * @param       p_event Mesh event with information about received message.
 * @param       p_data Pointer to the data portion of the message
 * @param       data_len Length of the data in the message
 *
 * @return      WICED_TRUE if the message is for this company ID/Model/Element Index combination, WICED_FALSE otherwise.
*/
wiced_bool_t wiced_bt_mesh_model_scheduler_server_message_handler(wiced_bt_mesh_event_t *p_event, uint8_t *p_data, uint16_t data_len);

/**
 * \brief Scheduler Setup Server Message Handler
 * \details The mesh core library calls this function for each message received.  The function returns WICED_TRUE if the message is destined for this specific model and successfully processed, and returns WICED_FALSE otherwise.
 * The function parses the message and if appropriate calls the parent back to perform functionality.
 *
 * @param       p_event Mesh event with information about received message.
 * @param       p_data Pointer to the data portion of the message
 * @param       data_len Length of the data in the message
 *
 * @return      WICED_TRUE if the message is for this company ID/Model/Element Index combination, WICED_FALSE otherwise.
*/
wiced_bool_t wiced_bt_mesh_model_scheduler_setup_server_message_handler(wiced_bt_mesh_event_t *p_event, uint8_t *p_data, uint16_t data_len);

/**
 * \brief Scheduler Client callback is called by the Mesh Models library on receiving a message from the peer
 *
 * @param       event that application should process (see @ref SCHEDULER_EVENT "Scheduler Events")
 * @param       p_event information about the message received.  The same pointer should be used in the reply if required.
 * @param       p_data pointer to the data structure
 *
 * @return      None
 */
typedef void(wiced_bt_mesh_scheduler_client_callback_t)(uint16_t event, wiced_bt_mesh_event_t *p_event, void *p_data);

/**
 * \brief Scheduler Client Model initialization
 *
 * @param       element_idx Device element to where model is used
 * @param       p_callback Application or Parent model callback that will be executed on receiving a message
 * @param       is_provisioned If TRUE, the application is being restarted after being provisioned or after a power loss. If FALSE the model cleans up NVRAM on startup.
 *
 * @return      None
 */
void wiced_bt_mesh_model_scheduler_client_init(wiced_bt_mesh_scheduler_client_callback_t *p_callback, wiced_bool_t is_provisioned);

/**
 * \brief Scheduler client Message Handler
 * \details An application or a parent model can call this models library function to process a message received
 * from the Scheduler Server device. The function parses the message and if appropriate calls the parent back
 * to perform functionality.
 *
 * @param       p_event Mesh event with information about received message.
 * @param       p_data Pointer to the data portion of the message
 * @param       data_len Length of the data in the message
 *
 * @return      WICED_TRUE if the message is for this company ID/Model/Element Index combination, WICED_FALSE otherwise.
*/
wiced_bool_t wiced_bt_mesh_model_scheduler_client_message_handler(wiced_bt_mesh_event_t *p_event, uint8_t *p_data, uint16_t data_len);

/**
 * \brief Scheduler Client send Get request
 * \details Application can call Scheduler Client Get function to send Scheduler Get client message to the server.
 *
 * @param       p_event Mesh event with the information about the message that has been received in the request, or created by the app for unsolicited message.
 *
 * @return      None
 */
void wiced_bt_mesh_model_scheduler_client_send_get(wiced_bt_mesh_event_t *p_event);

/**
 * \brief Scheduler Client Send Action Get request
 * \details Application can call Scheduler Client Action Get function to send Scheduler Action Get client message to the server.
 *
 * @param       p_event Mesh event with the information about the message that has been received in the request, or created by the app for unsolicited message.
 * @param       p_data Pointer to the data structure containing the action number of interest
 *
 * @return      None
 */
void wiced_bt_mesh_model_scheduler_client_send_action_get(wiced_bt_mesh_event_t *p_event, wiced_bt_mesh_scheduler_action_get_t *p_data);

/**
 * \brief Scheduler Client Send Action Set request
 * \details Application can call Scheduler Client Action Set function to send Scheduler Action Set client message to the server.
 *
 * @param       p_event Mesh event with the information about the message that has been received in the request, or created by the app for unsolicited message.
 * @param       p_data Pointer to the data structure containing the action number of interest, time stamp when action should accure and the scene number for the device to move to.
 *
 * @return      None
 */
void wiced_bt_mesh_model_scheduler_client_send_action_set(wiced_bt_mesh_event_t *p_event, wiced_bt_mesh_scheduler_action_data_t *p_data);


/* @} wiced_bt_mesh_models */

#define MESH_NODE_ID_INVALID          0x0000

#ifdef __cplusplus
}
#endif

#endif /* __WICED_BT_MESH_MODELS_H__ */
