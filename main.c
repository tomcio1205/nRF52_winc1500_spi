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
//#include <string.h>
//#include "driver/include/m2m_wifi.h"
#include "driver/source/nmasic.h"
#include "needed.h"
//#include "iot/http/http_client.h"
//#include "iot/json.h"
#include <errno.h>
#include "wifi_settings.h"
#include "http_settings.h"


#if defined(BOARD_PCA10036) || defined(BOARD_PCA10040)
#define SPI_CS_PIN   7 /**< SPI CS Pin.*/
#elif defined(BOARD_PCA10028)
#define SPI_CS_PIN   4  /**< SPI CS Pin.*/
#else
#error "Example is not supported on that board."
#endif

#define SPI_INSTANCE  0 /**< SPI instance index. */
static const nrf_drv_spi_t spi = NRF_DRV_SPI_INSTANCE(SPI_INSTANCE);  /**< SPI instance. */


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

	while ( _status != WL_CONNECTED) {
		NRF_LOG_PRINTF("Attempting to connect to WPA SSID: ");
		NRF_LOG_PRINTF("%s\r\n", MAIN_WLAN_SSID);
		// Connect to WPA/WPA2 network:
	   	wifi_connect(MAIN_WLAN_SSID, MAIN_WLAN_AUTH, MAIN_WLAN_PSK);
//		status = wifi_connect(MAIN_WLAN_SSID, M2M_WIFI_SEC_OPEN, (void *)0);
		// wait 5 seconds for connection:
		nrf_delay_ms(5000);
	}
	while(1)
	{
   	// listNetworks();
		sprintf(activate_data,"{\n \"powerConsumption\": 23,\n \"deviceId\": \"F8F135BA-1426-4CD6-836C-7CD23C95FBA5\"\n}");
		struct http_entity * entity = http_set_default_entity();
		entity->priv_data = (void*)activate_data;
		http_client_send_request(&http_client_module_inst, MAIN_HTTP_SERVER_TEST_URL, MAIN_HTTP_SERVER_TEST_METHOD, entity, NULL);
		while (m2m_wifi_handle_events(NULL) != M2M_SUCCESS) {
		}
		nrf_delay_ms(2000);
	}
}
