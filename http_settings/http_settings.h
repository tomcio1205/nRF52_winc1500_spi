/*
 * http_settings.h
 *
 *  Created on: 13 kwi 2017
 *      Author: TOMEK
 */

#ifndef HTTP_SETTINGS_H_
#define HTTP_SETTINGS_H_

#include "iot/http/http_client.h"
#include "iot/json.h"
#include <string.h>

/** server URL which will be requested */
#define MAIN_HTTP_SERVER_TEST_URL        "192.168.1.5/api/DeviceMeasurement"
//#define MAIN_HTTP_SERVER_TEST_URL        "192.168.1.5"
/** Method of server TEST request. */
#define MAIN_HTTP_SERVER_TEST_METHOD     HTTP_METHOD_POST
#define EXOSITE_EXAMPLE_HTTP_CONTENT_TYPE		"application/json"

extern struct http_entity g_http_entity;
extern char activate_data[100];
/** Instance of HTTP client module. */
struct http_client_module http_client_module_inst;


struct http_entity * http_set_default_entity();


void http_client_callback(struct http_client_module *module_inst, int type, union http_client_data *data);


void socket_event_handler(SOCKET sock, uint8_t msg_type, void *msg_data);


void socket_resolve_handler(uint8_t *domain_name, uint32_t server_ip);


void configure_http_client(void);


const char* http_get_contents_type(void *priv_data);


int http_read(void *priv_data, char *buffer, uint32_t size, uint32_t written);


void http_close(void *priv_data);


int http_get_contents_length(void *priv_data);

#endif /* WIFI_SETTINGS_H_ */
