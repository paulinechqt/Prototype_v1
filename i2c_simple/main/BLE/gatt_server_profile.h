/*
 * gatt_profile.h
 *
 *  Created on: 13 mars 2022
 *      Author: herve
 */

#ifndef MAIN_GATT_SERVER_PROFILE_H_
#define MAIN_GATT_SERVER_PROFILE_H_

#include "esp_bt.h"

#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_bt_defs.h"
#include "esp_bt_main.h"

void gap_init(void);
void gap_start(void);
void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param);

void gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);


//typedef struct blufi_beacon_s blufi_beacon_t;

#define PROFILE_NUM 1
#define PROFILE_APP_ID 0

#define GATT_SERVER_TAG "GATT_SERVER"
char prof_shared_buf[9];
char buffer_to_send[1000];

// char buffer_ax[10];
// char buffer_ay[10];
// char buffer_az[10];
// char buffer_gx[10];
// char buffer_gy[10];
// char buffer_gz[10];
// char buffer_mx[10];
// char buffer_my[10];
// char buffer_mz[10];

#endif /* MAIN_GATT_SERVER_PROFILE_H_ */
