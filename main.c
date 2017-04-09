/* Copyright (c) 2015 Nordic Semiconductor. All Rights Reserved.
 *
 * The information contained herein is property of Nordic Semiconductor ASA.
 * Terms and conditions of usage are described in detail in NORDIC
 * SEMICONDUCTOR STANDARD SOFTWARE LICENSE AGREEMENT.
 *
 * Licensees are granted free, non-transferable use of the information. NO
 * WARRANTY of ANY KIND is provided. This heading must NOT be removed from
 * the file.
 *
 */

#include "nrf_drv_spi.h"
#include "app_util_platform.h"
#include "nrf_gpio.h"
#include "nrf_delay.h"
#include "nrf_log.h"
#include "boards.h"
#include "app_error.h"
#include <string.h>
#include "driver/include/m2m_wifi.h"
#include "driver/source/nmasic.h"
#include "needed.h"
#include "iot/http/http_client.h"
#include "iot/json.h"
#include <errno.h>

#if defined(BOARD_PCA10036) || defined(BOARD_PCA10040)
#define SPI_CS_PIN   7 /**< SPI CS Pin.*/
#elif defined(BOARD_PCA10028)
#define SPI_CS_PIN   4  /**< SPI CS Pin.*/
#else
#error "Example is not supported on that board."
#endif

#define STRING_EOL    "\r\n"
#define STRING_HEADER "-- WINC1500 chip information example --"STRING_EOL \
	"-- "BOARD_NAME " --"STRING_EOL	\
	"-- Compiled: "__DATE__ " "__TIME__ " --"STRING_EOL
#define MAIN_WLAN_SSID                  "PolskiBus-545" /**< Destination SSID */
#define MAIN_WLAN_AUTH                  M2M_WIFI_SEC_WPA_PSK /**< Security manner */
#define MAIN_WLAN_PSK                   "" /**< Password for Destination SSID */

/** server URL which will be requested */
#define MAIN_HTTP_SERVER_TEST_URL        "http://ipinfo.io"
/** Method of server TEST request. */
#define MAIN_HTTP_SERVER_TEST_METHOD     HTTP_METHOD_GET
/** Instance of HTTP client module. */
struct http_client_module http_client_module_inst;

#define SPI_INSTANCE  0 /**< SPI instance index. */
static const nrf_drv_spi_t spi = NRF_DRV_SPI_INSTANCE(SPI_INSTANCE);  /**< SPI instance. */

#define TEST_STRING "Nordic"
static uint8_t       m_tx_buf[] = TEST_STRING;           /**< TX buffer. */
static uint8_t       m_rx_buf[sizeof(TEST_STRING)+1];    /**< RX buffer. */
static const uint8_t m_length = sizeof(m_tx_buf);        /**< Transfer length. */
/** Index of scan list to request scan result. */
static uint8_t scan_request_index = 0;
/** Number of APs found. */
static uint8_t num_founded_ap = 0;

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

static int status = WL_IDLE_STATUS;
wl_status_t _status;
wl_mode_t _mode;
static uint32_t _localip;
static uint32_t _submask;
static uint32_t _gateway;
static int _dhcp;
static uint32_t _resolve;
static char _scan_ssid[M2M_MAX_SSID_LEN];
static uint8_t _scan_auth;
static uint8_t _scan_channel;
static char _ssid[M2M_MAX_SSID_LEN];

static void wifi_cb(uint8_t u8MsgType, void *pvMsg)
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

	case M2M_WIFI_RESP_CON_STATE_CHANGED:
	{
		tstrM2mWifiStateChanged *pstrWifiState = (tstrM2mWifiStateChanged *)pvMsg;
		if (pstrWifiState->u8CurrState == M2M_WIFI_CONNECTED) {
			m2m_wifi_request_dhcp_client();
		} else if (pstrWifiState->u8CurrState == M2M_WIFI_DISCONNECTED) {
			NRF_LOG_PRINTF("Wi-Fi disconnected\r\n");

			/* Request scan. */
			m2m_wifi_request_scan(M2M_WIFI_CH_ALL);
		}

		break;
	}

	case M2M_WIFI_REQ_DHCP_CONF:
	{
		uint8_t *pu8IPAddress = (uint8_t *)pvMsg;
		NRF_LOG_PRINTF("Wi-Fi connected\r\n");
		NRF_LOG_PRINTF("Wi-Fi IP is %u.%u.%u.%u\r\n",
				pu8IPAddress[0], pu8IPAddress[1], pu8IPAddress[2], pu8IPAddress[3]);
		http_client_send_request(&http_client_module_inst, MAIN_HTTP_SERVER_TEST_URL, MAIN_HTTP_SERVER_TEST_METHOD, NULL, NULL);
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

/**
 * \brief Callback of the HTTP client.
 *
 * \param[in]  module_inst     Module instance of HTTP client module.
 * \param[in]  type            Type of event.
 * \param[in]  data            Data structure of the event. \refer http_client_data
 */
static void http_client_callback(struct http_client_module *module_inst, int type, union http_client_data *data)
{
	struct json_obj json, loc;
	switch (type) {
	case HTTP_CLIENT_CALLBACK_SOCK_CONNECTED:
		NRF_LOG_PRINTF("Connected\r\n");
		break;

	case HTTP_CLIENT_CALLBACK_REQUESTED:
		NRF_LOG_PRINTF("Request complete\r\n");
		break;

	case HTTP_CLIENT_CALLBACK_RECV_RESPONSE:
		NRF_LOG_PRINTF("Received response %u data size %u\r\n",
				(unsigned int)data->recv_response.response_code,
				(unsigned int)data->recv_response.content_length);
		if (data->recv_response.content != NULL) {
			if (json_create(&json, data->recv_response.content, data->recv_response.content_length) == 0 &&
					json_find(&json, "loc", &loc) == 0) {
				NRF_LOG_PRINTF("Location : %s\r\n", loc.value.s);
			}
		}

		break;

	case HTTP_CLIENT_CALLBACK_DISCONNECTED:
		NRF_LOG_PRINTF("Disconnected reason:%d\r\n", data->disconnected.reason);

		/* If disconnect reason is equals to -ECONNRESET(-104),
		 * It means Server was disconnected your connection by the keep alive timeout.
		 * This is normal operation.
		 */
		if (data->disconnected.reason == -EAGAIN) {
			/* Server has not responded. retry it immediately. */
			http_client_send_request(&http_client_module_inst, MAIN_HTTP_SERVER_TEST_URL, MAIN_HTTP_SERVER_TEST_METHOD, NULL, NULL);
		}

		break;
	}
}

/**
 * \brief Callback to get the Socket event.
 *
 * \param[in] Socket descriptor.
 * \param[in] msg_type type of Socket notification. Possible types are:
 *  - [SOCKET_MSG_CONNECT](@ref SOCKET_MSG_CONNECT)
 *  - [SOCKET_MSG_BIND](@ref SOCKET_MSG_BIND)
 *  - [SOCKET_MSG_LISTEN](@ref SOCKET_MSG_LISTEN)
 *  - [SOCKET_MSG_ACCEPT](@ref SOCKET_MSG_ACCEPT)
 *  - [SOCKET_MSG_RECV](@ref SOCKET_MSG_RECV)
 *  - [SOCKET_MSG_SEND](@ref SOCKET_MSG_SEND)
 *  - [SOCKET_MSG_SENDTO](@ref SOCKET_MSG_SENDTO)
 *  - [SOCKET_MSG_RECVFROM](@ref SOCKET_MSG_RECVFROM)
 * \param[in] msg_data A structure contains notification informations.
 */
static void socket_event_handler(SOCKET sock, uint8_t msg_type, void *msg_data)
{
	http_client_socket_event_handler(sock, msg_type, msg_data);
}

/**
 * \brief Callback of gethostbyname function.
 *
 * \param[in] doamin_name Domain name.
 * \param[in] server_ip IP of server.
 */
static void socket_resolve_handler(uint8_t *domain_name, uint32_t server_ip)
{
	http_client_socket_resolve_handler(domain_name, server_ip);
}

/**
 * \brief Configure HTTP client module.
 */
static void configure_http_client(void)
{
	struct http_client_config httpc_conf;
	int ret;

	http_client_get_config_defaults(&httpc_conf);

	httpc_conf.recv_buffer_size = 256;
//	httpc_conf.timer_inst = &swt_module_inst;
	/* ipinfo.io send json format data if only client is a curl. */
	httpc_conf.user_agent = "curl/7.10.6";

	ret = http_client_init(&http_client_module_inst, &httpc_conf);
	if (ret < 0) {
		NRF_LOG_PRINTF("HTTP client initialization has failed(%s)\r\n", strerror(ret));
		while (1) {
		} /* Loop forever. */
	}

	http_client_register_callback(&http_client_module_inst, http_client_callback);
}

/**
 * @brief SPI user event handler.
 * @param event
 */
void spi_event_handler(nrf_drv_spi_evt_t const * p_event)
{
    spi_xfer_done = true;
//    NRF_LOG_PRINTF(" Transfer completed.\r\n");
//    if (m_rx_buf[0] != 0)
//    {
//        NRF_LOG_PRINTF(" Received: %s\r\n",m_rx_buf);
//    }
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

int main(void)
{
	NRF_LOG_INIT();
	spi_xfer_done = false;
	nrf_gpio_cfg_output(12);
	nrf_gpio_cfg_output(13);
	nrf_gpio_pin_set(12);
	nrf_gpio_pin_set(13);
    LEDS_CONFIGURE(BSP_LED_0_MASK);
    LEDS_OFF(BSP_LED_0_MASK);

    APP_ERROR_CHECK(NRF_LOG_INIT());
    NRF_LOG_PRINTF("SPI example\r\n");

    nrf_drv_spi_config_t spi_config = NRF_DRV_SPI_DEFAULT_CONFIG(SPI_INSTANCE);
    spi_config.ss_pin = SPI_CS_PIN;
    spi_config.frequency = NRF_DRV_SPI_FREQ_1M;
    spi_config.mode = NRF_DRV_SPI_MODE_0;
    spi_config.bit_order = NRF_DRV_SPI_BIT_ORDER_MSB_FIRST;
    APP_ERROR_CHECK(nrf_drv_spi_init(&spi, &spi_config, spi_event_handler));

    configure_http_client();

    tstrWifiInitParam param;
    int8_t ret;
	/* Initialize the BSP. */
	nm_bsp_init();

	/* Initialize Wi-Fi parameters structure. */
	memset((uint8_t *)&param, 0, sizeof(tstrWifiInitParam));
	param.pfAppWifiCb = wifi_cb;
	ret = m2m_wifi_init(&param);
	if (M2M_SUCCESS != ret) {
		NRF_LOG_PRINTF("main: m2m_wifi_init call error!(%d)\r\n", ret);
		while (1) {
		}
	}
	/* Initialize Socket module */
	socketInit();
	registerSocketCallback(socket_event_handler, socket_resolve_handler);

//    	listNetworks();
	while ( status != WL_CONNECTED) {
		NRF_LOG_PRINTF("Attempting to connect to WPA SSID: ");
		NRF_LOG_PRINTF("%s\r\n", MAIN_WLAN_SSID);
		// Connect to WPA/WPA2 network:
//	   		status = wifi_connect(MAIN_WLAN_SSID, MAIN_WLAN_AUTH, MAIN_WLAN_PSK);
		status = wifi_connect(MAIN_WLAN_SSID, M2M_WIFI_SEC_OPEN, (void *)0);
		// wait 5 seconds for connection:
		nrf_delay_ms(5000);
	}
	while(1)
	{
		while (m2m_wifi_handle_events(NULL) != M2M_SUCCESS) {
		}
	}

//	nrf_delay_ms(5000);
}
