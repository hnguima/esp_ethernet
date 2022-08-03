#ifndef __ESP_ETHERNET___H
#define __ESP_ETHERNET___H

/**
 * Lib C
 */
#include <stdio.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>
/**
 * FreeRTOS
 */
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"

/**
 * Logs;
 */
#include "esp_system.h"
#include "esp_log.h"

/**
 * Callbacks
 */
#include "esp_event.h"

/**
 * Ethernet lib
 */
#include "esp_eth.h"
#include "esp_netif.h"

/**
 * Drivers;
 */
#include "esp32/rom/gpio.h"
#include "driver/gpio.h"
#include "driver/periph_ctrl.h"

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