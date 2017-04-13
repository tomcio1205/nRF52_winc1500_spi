/*
 * http_settings.c
 *
 *  Created on: 13 kwi 2017
 *      Author: TOMEK
 */

#include <stdint.h>
#include "http_settings.h"
#include "nrf_log.h"
#include <errno.h>

struct http_entity g_http_entity = {0,};
char activate_data[100] = {0,};
/**
 * \brief Callback of the HTTP client.
 *
 * \param[in]  module_inst     Module instance of HTTP client module.
 * \param[in]  type            Type of event.
 * \param[in]  data            Data structure of the event. \refer http_client_data
 */
void http_client_callback(struct http_client_module *module_inst, int type, union http_client_data *data)
{
	struct json_obj json, iO;
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
				json_find(&json, "isOn", &iO) == 0) {
					NRF_LOG_PRINTF("isOn : %s\r\n", iO.value.s);
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
void socket_event_handler(SOCKET sock, uint8_t msg_type, void *msg_data)
{
	http_client_socket_event_handler(sock, msg_type, msg_data);
}

/**
 * \brief Callback of gethostbyname function.
 *
 * \param[in] doamin_name Domain name.
 * \param[in] server_ip IP of server.
 */
void socket_resolve_handler(uint8_t *domain_name, uint32_t server_ip)
{
	http_client_socket_resolve_handler(domain_name, server_ip);
}

/**
 * \brief Configure HTTP client module.
 */
 void configure_http_client(void)
{
	struct http_client_config httpc_conf;
	int ret;

	http_client_get_config_defaults(&httpc_conf);

	httpc_conf.recv_buffer_size = 256;
	httpc_conf.send_buffer_size = 1024;
//	httpc_conf.timer_inst = &swt_module_inst;

	ret = http_client_init(&http_client_module_inst, &httpc_conf);
	if (ret < 0) {
		NRF_LOG_PRINTF("HTTP client initialization has failed(%s)\r\n", strerror(ret));
		while (1) {
		} /* Loop forever. */
	}

	http_client_register_callback(&http_client_module_inst, http_client_callback);
}


const char* http_get_contents_type(void *priv_data)
{
	return (const char*)EXOSITE_EXAMPLE_HTTP_CONTENT_TYPE;
}

int http_get_contents_length(void *priv_data)
{
	return strlen( (char*)priv_data);
}

int http_read(void *priv_data, char *buffer, uint32_t size, uint32_t written)
{
	int32_t length = 0;

	if(priv_data)
	{
		length = strlen( (char*)priv_data);
		memcpy(buffer,(char*)priv_data, length);
	}

	return length;
}

void http_close(void *priv_data)
{
}

struct http_entity * http_set_default_entity()
{
	memset(&g_http_entity, 0x00, sizeof(struct http_entity));
	g_http_entity.close = http_close;
	g_http_entity.is_chunked = 0;
	g_http_entity.priv_data = NULL;
	g_http_entity.read = http_read;
	g_http_entity.get_contents_length = http_get_contents_length;
	g_http_entity.get_contents_type = http_get_contents_type;

	return &g_http_entity;
}