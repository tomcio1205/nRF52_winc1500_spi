/*
 * wifi_settings.c
 *
 *  Created on: 13 kwi 2017
 *      Author: TOMEK
 */

#include <stdint.h>
#include <wifi_settings.h>
#include "nrf_log.h"

void wifi_cb(uint8_t u8MsgType, void *pvMsg)
{
	switch (u8MsgType) {
	case M2M_WIFI_RESP_SCAN_DONE:
	{
		tstrM2mScanDone *pstrInfo = (tstrM2mScanDone *)pvMsg;
		if (pstrInfo->u8NumofCh >= 1) {
			_status = WL_SCAN_COMPLETED;
		}

		break;
	}

	case M2M_WIFI_RESP_SCAN_RESULT:
	{
		tstrM2mWifiscanResult *pstrScanResult = (tstrM2mWifiscanResult *)pvMsg;
		uint16_t scan_ssid_len = strlen((const char *)pstrScanResult->au8SSID);
		memset(_scan_ssid, 0, M2M_MAX_SSID_LEN);
		if (scan_ssid_len) {
			memcpy(_scan_ssid, (const char *)pstrScanResult->au8SSID, scan_ssid_len);
		}
		_resolve = pstrScanResult->s8rssi;
		_scan_auth = pstrScanResult->u8AuthType;
		_scan_channel = pstrScanResult->u8ch;
		_status = WL_SCAN_COMPLETED;

		break;
	}

	case M2M_WIFI_RESP_DEFAULT_CONNECT:
	{
		tstrM2MDefaultConnResp *pstrDefaultConnResp = (tstrM2MDefaultConnResp *)pvMsg;
		if (pstrDefaultConnResp->s8ErrorCode) {
			_status = WL_DISCONNECTED;
		}
	}
	break;

	case M2M_WIFI_RESP_CON_STATE_CHANGED:
	{
		tstrM2mWifiStateChanged *pstrWifiState = (tstrM2mWifiStateChanged *)pvMsg;
		if (pstrWifiState->u8CurrState == M2M_WIFI_CONNECTED) {

			if (_mode == WL_STA_MODE && !_dhcp) {
				_status = WL_CONNECTED;

			} else if (_mode == WL_AP_MODE) {
				_status = WL_AP_CONNECTED;
			}
		} else if (pstrWifiState->u8CurrState == M2M_WIFI_DISCONNECTED) {

			if (_mode == WL_STA_MODE) {
				_status = WL_DISCONNECTED;
				if (_dhcp) {
					_localip = 0;
					_submask = 0;
					_gateway = 0;
				}
			} else if (_mode == WL_AP_MODE) {
				_status = WL_AP_LISTENING;
			}
		}
	}
	break;

	case M2M_WIFI_REQ_DHCP_CONF:
	{
		uint8_t *pu8IPAddress = (uint8_t *)pvMsg;
		NRF_LOG_PRINTF("Wi-Fi connected\r\n");
		NRF_LOG_PRINTF("Wi-Fi IP is %u.%u.%u.%u\r\n",
				pu8IPAddress[0], pu8IPAddress[1], pu8IPAddress[2], pu8IPAddress[3]);
		_status = WL_CONNECTED;
		break;
	}

	case M2M_WIFI_RESP_CURRENT_RSSI:
	{
		/* This message type is triggered by "m2m_wifi_req_curr_rssi()" function. */
		_resolve = *((int8_t *)pvMsg);
		break;
	}

	default:
	{
		break;
	}
	}
}

uint8_t wifi_connect(const char *ssid, uint8_t u8SecType, const void *pvAuthInfo)
{
//	if (!_init) {
//		init();
//	}

	// Connect to router:
	if (_dhcp) {
		_localip = 0;
		_submask = 0;
		_gateway = 0;
	}
	if (m2m_wifi_connect((char*)ssid, strlen(ssid), u8SecType, (void*)pvAuthInfo, M2M_WIFI_CH_ALL) < 0) {
		_status = WL_CONNECT_FAILED;
		return _status;
	}
	_status = WL_IDLE_STATUS;
	_mode = WL_STA_MODE;

	// Wait for connection or timeout:
	while (!(_status & WL_CONNECTED) &&	!(_status & WL_DISCONNECTED))
	{
		m2m_wifi_handle_events(NULL);
	}
	if (!(_status & WL_CONNECTED))
	{
		_mode = WL_RESET_MODE;
	}

	memset(_ssid, 0, M2M_MAX_SSID_LEN);
	memcpy(_ssid, ssid, strlen(ssid));
	return _status;
}

int8_t scanNetworks()
{
	wl_status_t tmp = _status;
	// Start scan:
	if (m2m_wifi_request_scan(M2M_WIFI_CH_ALL) < 0) {
		return 0;
	}
	// Wait for scan result or timeout:
	_status = WL_IDLE_STATUS;
	while (!(_status & WL_SCAN_COMPLETED)) {
		m2m_wifi_handle_events(NULL);
	}
	_status = tmp;
	return m2m_wifi_get_num_ap_found();
}

int32_t rssi(uint8_t pos)
{
	wl_status_t tmp = _status;

	// Get scan RSSI result:
	if (m2m_wifi_req_scan_result(pos) < 0) {
		return 0;
	}

	// Wait for connection or timeout:
	_status = WL_IDLE_STATUS;
	while (!(_status & WL_SCAN_COMPLETED)) {
		m2m_wifi_handle_events(NULL);
	}

	_status = tmp;
	int32_t rssi = _resolve;
	_resolve = 0;
	return rssi;
}

void ssid(uint8_t pos)
{
	wl_status_t tmp = _status;

	// Get scan SSID result:
	memset(_scan_ssid, 0, M2M_MAX_SSID_LEN);
	if (m2m_wifi_req_scan_result(pos) < 0) {
		return 0;
	}

	// Wait for connection or timeout:
	_status = WL_IDLE_STATUS;
	while (!(_status & WL_SCAN_COMPLETED)) {
		m2m_wifi_handle_events(NULL);
	}

	_status = tmp;
	_resolve = 0;
}

void listNetworks() {
	// scan for nearby networks:
	NRF_LOG_PRINTF("** Scan Networks **\r\n");
	int numSsid = scanNetworks();
	if (numSsid == -1) {
		NRF_LOG_PRINTF("Couldn't get a wifi connection\r\n");
		while (true);
	}

    // print the list of networks seen:
	NRF_LOG_PRINTF("number of available networks:");
	NRF_LOG_PRINTF(" %d\r\n",numSsid);

	// print the network number and name for each network found:
	for (int thisNet = 0; thisNet < numSsid; thisNet++) {
		NRF_LOG_PRINTF("%d",thisNet);
		NRF_LOG_PRINTF(") ");
		ssid(thisNet);
		NRF_LOG_PRINTF("%s",_scan_ssid);
		NRF_LOG_PRINTF("\tSignal: ");
		NRF_LOG_PRINTF("%d ", rssi(thisNet));
		NRF_LOG_PRINTF(" dBm\r\n");
    }
}