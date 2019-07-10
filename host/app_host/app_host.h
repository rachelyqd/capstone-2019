
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


#ifndef WICED_APP_H
#define WICED_APP_H

#include "wiced_hci.h"
#include "hci_control_api.h"

#define WICED_CONNECTION_TYPE_NONE    0x0000
#define WICED_CONNECTION_TYPE_AG      0x0001
#define WICED_CONNECTION_TYPE_SPP     0x0002
#define WICED_CONNECTION_TYPE_AUDIO   0x0004
#define WICED_CONNECTION_TYPE_HF      0x0008
#define WICED_CONNECTION_TYPE_HIDH    0x0010
#define WICED_CONNECTION_TYPE_IAP2    0x0020
#define WICED_CONNECTION_TYPE_LE      0x0040
#define WICED_CONNECTION_TYPE_AVRC    0x0080
#define WICED_CONNECTION_TYPE_AVK     0x0100
#define WICED_CONNECTION_TYPE_PBC     0x0200
#define WICED_CONNECTION_TYPE_BATTC   0x0400
#define WICED_CONNECTION_TYPE_FINDMEL 0x0800
#define WICED_CONNECTION_TYPE_OPS     0x1000

#define WICED_NULL_HANDLE             0xFFFF

typedef struct
{
    bool ag_audio_opened;
} wiced_ag_state_t;

typedef struct
{
    bool audio_started;
} wiced_audio_state_t;

typedef struct
{
    wiced_ag_state_t ag_state;
    wiced_audio_state_t audio_src_state;
} wiced_app_t;

typedef struct
{
    uint8_t m_address[6];

    bool m_paired;
    char m_name[100];

    bool m_ledevice;
    uint8_t m_le_address_type;
    uint8_t  m_le_role;

    uint16_t m_nvram_id;
    char m_nvram[100];

    uint16_t m_conn_type;

    uint16_t m_audio_handle;
    uint16_t m_hf_handle;
    uint16_t m_ag_handle;
    uint16_t m_spp_handle;
    uint16_t m_hidh_handle;
    uint16_t m_iap2_handle;
    uint16_t m_avrc_handle;
    uint16_t m_le_handle;
    uint16_t m_bsg_handle;
    uint16_t m_pbc_handle;
    uint16_t m_avk_handle;
    uint16_t m_battc_handle;
    uint16_t m_findmel_handle;

    wiced_app_t m_app_state;

    void *p_next;

} wiced_bt_device_t;

typedef struct
{
     // app state

     // ANS
     uint16_t m_ans_supported_alerts;
     uint16_t m_ans_supported_new_alerts;
     uint16_t m_ans_supported_unread_alerts;
     bool m_ans_connected;

     // ANC
     bool m_b_new_alerts_enabled;
     bool m_b_unread_alerts_enabled;
     bool m_anc_connected;
     uint16_t m_anc_server_supported_new_alerts;
     uint16_t m_anc_server_supported_unread_alerts;

     // list of peer devices
     wiced_bt_device_t *p_devices;

} wiced_host_app_t;



extern wiced_host_app_t g_app;

// device management API
wiced_bt_device_t*  app_host_add_device(uint8_t bda[6]);
void app_host_remove_device(uint8_t bda[6]);
void app_host_remove_all_devices();
wiced_bt_device_t* app_host_find_device(uint8_t bda[6]);
wiced_bt_device_t *app_host_find_device_by_connection(uint16_t conn_type);

// Audio Gateway commands
bool app_host_ag_connect(uint8_t bda[6]);
bool app_host_ag_disconnect(uint8_t bda[6]);
bool app_host_ag_audio_open(uint8_t bda[6]);
bool app_host_ag_audio_close(uint8_t bda[6]);

// Audio Gateway events
void app_host_ag_event(uint16_t opcode, uint8_t * p_data, uint16_t len);

// Audio Source
bool app_host_audio_src_connect(wiced_bt_audio_source_connect_data_t *p_data);
bool app_host_audio_src_disconnect(uint8_t bda[6]);
bool app_host_audio_src_start(uint8_t bda[6], uint8_t sample_freq, uint8_t audio_mode);
bool app_host_audio_src_stop(uint8_t bda[6]);
bool app_host_audio_src_audio_data(uint8_t *data, uint16_t len);

// Audio Src event
void app_host_audio_src_event(uint16_t opcode, uint8_t * p_data, uint16_t len);

// ANS
void app_host_ans_event(uint16_t opcode, uint8_t * p_data, uint16_t len);
// ANC
void app_host_anc_event(uint16_t opcode, uint8_t * p_data, uint16_t len);

//LE COC
bool app_host_le_coc_set_phy(bool le2m_enable);
bool app_host_le_coc_send_psm(uint16_t psm);
bool app_host_le_coc_send_mtu(uint16_t mtu);
bool app_host_le_coc_connect(uint8_t bda[6]);
bool app_host_le_coc_disconnect(uint8_t bda[6]);
bool app_host_le_coc_start_adv(bool start);
bool app_host_le_coc_send_data(uint8_t *p_data, uint16_t len);
void app_host_le_coc_event(uint16_t opcode, uint8_t * p_data, uint16_t len);

// Customer app APIs
void app_host_init();
void app_host_deinit();
void app_host_handle_event(uint16_t opcode, uint8_t * p_data, uint16_t len);

// extern API, Implemented by application
extern int app_host_port_write(uint16_t command, uint8_t *data, uint16_t len);
extern void app_host_log(const char * fmt, ...);

#endif // WICED_APP_H


