#ifndef __ESP_ETHERNET___H
#define __ESP_ETHERNET___H

//lib c
#include <stdio.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>

//freeRTOS
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"

//logs
#include "esp_system.h"
#include "esp_log.h"

//events
#include "esp_event.h"

//ethernet
#include "esp_eth.h"
#include "esp_netif.h"

//lwip 
#include "lwip/dns.h"
#include "lwip/inet.h"

//sntp
#include "esp_sntp.h"

//error checking
#define ESP_ETH_ERR_CHECK(func, err, tag, message)                                   \
    err = func;                                                                      \
    if (err)                                                                         \
    {                                                                                \
        ESP_LOGE(tag, "Funcão %s linha %u --> %s", __FUNCTION__, __LINE__, message); \
    }
    
typedef struct
{

    int32_t ip;
    int32_t mask;
    int32_t gw;

} eth_data_t;

esp_err_t ethernet_init(eth_data_t *config);

#endif //<-- __ESP_ETHERNET___H -->