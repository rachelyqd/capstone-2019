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
 * Definitions for interface for mesh application interface library
 *
 */

#ifndef __WICED_BT_MESH_APPS_H__
#define __WICED_BT_MESH_APPS_H__

#include "wiced_bt_mesh_event.h"
#include "wiced_bt_mesh_model_defs.h"

#ifdef HCI_CONTROL
#pragma pack(1)

typedef PACKED struct
{
    uint16_t src;
    uint16_t app_key_idx;
    uint8_t  element_idx;
    uint8_t  data[1];
} wiced_bt_mesh_hci_event_t;
#endif

#pragma pack()

extern char   *mesh_dev_name;
extern uint8_t mesh_appearance[WICED_BT_MESH_PROPERTY_LEN_DEVICE_APPEARANCE];
extern uint8_t mesh_mfr_name[WICED_BT_MESH_PROPERTY_LEN_DEVICE_MANUFACTURER_NAME];
extern uint8_t mesh_model_num[WICED_BT_MESH_PROPERTY_LEN_DEVICE_MODEL_NUMBER];
extern uint8_t mesh_system_id[8];

#ifdef __cplusplus
extern "C"
{
#endif

extern void mesh_application_factory_reset(void);

#ifdef HCI_CONTROL
extern wiced_bt_mesh_hci_event_t *wiced_bt_mesh_create_hci_event(wiced_bt_mesh_event_t *p_event);

extern wiced_bt_mesh_event_t *wiced_bt_mesh_create_event_from_wiced_hci(uint16_t opcode, uint16_t company_id, uint16_t model_id, uint8_t **p_data, uint32_t *len);

extern void mesh_transport_send_data(uint16_t opcode, uint8_t *p_trans_buf, uint16_t data_len);

extern void wiced_bt_mesh_send_hci_tx_complete(wiced_bt_mesh_hci_event_t *p_hci_event, wiced_bt_mesh_event_t *p_event);
#endif

#ifdef __cplusplus
}
#endif

#endif /* __WICED_BT_MESH_APP_H__ */
