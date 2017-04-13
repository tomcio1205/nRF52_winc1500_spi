/*
 * wifi_settings.h
 *
 *  Created on: 13 kwi 2017
 *      Author: TOMEK
 */

#ifndef WIFI_SETTINGS_H_
#define WIFI_SETTINGS_H_

#include "driver/include/m2m_types.h"
#include <string.h>
#include "driver/include/m2m_wifi.h"

/**< Define wifi connection settings. */
// #define MAIN_WLAN_SSID                  "NETIASPOT-60A5F0" /**< Destination SSID */
// #define MAIN_WLAN_AUTH                  M2M_WIFI_SEC_WPA_PSK /**< Security manner */
// #define MAIN_WLAN_PSK                   "7ov9fxy7zb4w" /**< Password for Destination SSID */
#define MAIN_WLAN_SSID                  "NETIASPOT-52CC50" /**< Destination SSID */
#define MAIN_WLAN_AUTH                  M2M_WIFI_SEC_WPA_PSK /**< Security manner */
#define MAIN_WLAN_PSK                   "c2svzibeu6i5" /**< Password for Destination SSID */

uint32_t _localip;
uint32_t _submask;
uint32_t _gateway;
int _dhcp;
uint32_t _resolve;
char _scan_ssid[M2M_MAX_SSID_LEN];
uint8_t _scan_auth;
uint8_t _scan_channel;
char _ssid[M2M_MAX_SSID_LEN];

typedef enum {
	WL_NO_SHIELD = 255,
	WL_IDLE_STATUS = 0,
	WL_NO_SSID_AVAIL,
	WL_SCAN_COMPLETED,
	WL_CONNECTED,
	WL_CONNECT_FAILED,
	WL_CONNECTION_LOST,
	WL_DISCONNECTED,
	WL_AP_LISTENING,
	WL_AP_CONNECTED,
	WL_AP_FAILED,
	WL_PROVISIONING,
	WL_PROVISIONING_FAILED
} wl_status_t;

typedef enum {
	WL_RESET_MODE = 0,
	WL_STA_MODE,
	WL_PROV_MODE,
	WL_AP_MODE
} wl_mode_t;

wl_status_t _status;
wl_mode_t _mode;


void wifi_cb(uint8_t u8MsgType, void *pvMsg);


uint8_t wifi_connect(const char *ssid, uint8_t u8SecType, const void *pvAuthInfo);


int8_t scanNetworks();


int32_t rssi(uint8_t pos);


void ssid(uint8_t pos);


void listNetworks();

#endif /* WIFI_SETTINGS_H_ */
