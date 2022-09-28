#include "ethernet.h"

// local variables
static const char *TAG = "eth";
esp_netif_t *eth_netif;

// callbacks
static void eth_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data);
static void got_ip_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data);


esp_err_t ethernet_init(eth_data_t *config)
{
  esp_err_t err = ESP_OK;
  /*
   * Inicializa o stack TCP;
   */

  ESP_ETH_ERR_CHECK(esp_netif_init(), err, TAG, "esp_netif_init error");

  /*
   * Registra a função de callback do stack TCP;
   * Esta função irá notificar os status de cada estado da conexão ethernet;
   */

  esp_netif_config_t cfg = ESP_NETIF_DEFAULT_ETH();
  eth_netif = esp_netif_new(&cfg);
  if (eth_netif == NULL)
  {
    ESP_LOGE(TAG, "Failed to create Ethernet netif!");
    goto err;
  }
  // Set default handlers to process TCP/IP stuffs
  ESP_ETH_ERR_CHECK(esp_eth_set_default_handlers(eth_netif), err, TAG, "esp_eth_set_default_handlers error");

  // IP fixo ou DHCP?
  uint8_t dhcp_temp = 0; // Força o produto a sempre utilizar ip fixo.
                         // No futuro a ideia é dar ao usuario domestico a opção do DHCP

  if (dhcp_temp > 0)
  {
    ESP_ETH_ERR_CHECK(esp_netif_dhcpc_start(eth_netif), err, TAG, "esp_netif_dhcpc_start error");
  }
  else
  {
    ESP_ETH_ERR_CHECK(esp_netif_dhcpc_stop(eth_netif), err, TAG, "esp_netif_dhcpc_stop error");
    esp_netif_ip_info_t ip_info;

    ip_info.ip.addr = config->ip;
    ip_info.netmask.addr = config->mask;
    ip_info.gw.addr = config->gw;

    esp_netif_set_ip_info(eth_netif, &ip_info);
  }

  // Registra os eventos ocorridos durante o processo de conexão
  // Em IP_EVENT somente GOT_IP é interessante quando utilizando só a conexãoethernet
  ESP_ETH_ERR_CHECK(esp_event_handler_register(ETH_EVENT, ESP_EVENT_ANY_ID, &eth_event_handler, NULL), err, TAG, "esp_event_handler_register error");
  ESP_ETH_ERR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_ETH_GOT_IP, &got_ip_event_handler, NULL), err, TAG, "esp_event_handler_register error");

  /*
   * Configura o descritor ethernet do ESP32;
   */
  eth_phy_config_t phy_config = ETH_PHY_DEFAULT_CONFIG();
  phy_config.phy_addr = CONFIG_ETH_PHY_ADDR;
  phy_config.reset_gpio_num = CONFIG_ETH_PHY_RST;

  eth_mac_config_t mac_config = ETH_MAC_DEFAULT_CONFIG();
  mac_config.smi_mdc_gpio_num = CONFIG_ETH_PHY_MDC;
  mac_config.smi_mdio_gpio_num = CONFIG_ETH_PHY_MDIO;

  esp_eth_mac_t *mac = esp_eth_mac_new_esp32(&mac_config);
  if (mac == NULL)
  {
    ESP_LOGE(TAG, "Failed to create MAC");
    goto err;
  }
  esp_eth_phy_t *phy = esp_eth_phy_new_lan8720(&phy_config);
  if (phy == NULL)
  {
    ESP_LOGE(TAG, "Failed to init PHY");
    goto err;
  }

  /*
   *   Inicializa ethernet
   */
  esp_eth_config_t eth_config = ETH_DEFAULT_CONFIG(mac, phy);

  esp_eth_handle_t eth_handle = NULL;
  ESP_ETH_ERR_CHECK(esp_eth_driver_install(&eth_config, &eth_handle), err, TAG, "esp_eth_driver_install error");
  /* attach Ethernet driver to TCP/IP stack */
  ESP_ETH_ERR_CHECK(esp_netif_attach(eth_netif, esp_eth_new_netif_glue(eth_handle)), err, TAG, "esp_netif_attach error");
  /* start Ethernet driver state machine */
  ESP_ETH_ERR_CHECK(esp_eth_start(eth_handle), err, TAG, "esp_eth_start error");

  return ESP_OK;

err:
  // deinit ethernet
  esp_eth_driver_uninstall(eth_handle);
  // if (mac != NULL)
  // {
  //     mac->del(mac);
  // }
  if (phy != NULL)
  {
    phy->del(phy);
  }
  esp_netif_destroy(eth_netif);
  esp_netif_deinit();

  return err;
}

#ifdef CONFIG_ETH_DNS_ENABLE

static void eth_dns_configure(void)
{
  // Set DNS servers for internet connection
  esp_netif_dns_info_t dns_info;
  dns_info.ip.type = IPADDR_TYPE_V4;

  IP_ADDR4(&dns_info.ip, 8, 8, 8, 8);
  esp_netif_set_dns_info(eth_netif, ESP_NETIF_DNS_MAIN, &dns_info);
  IP_ADDR4(&dns_info.ip, 1, 1, 1, 1);
  esp_netif_set_dns_info(eth_netif, ESP_NETIF_DNS_BACKUP, &dns_info);
}

#ifdef CONFIG_ETH_SNTP_ENABLE

static void eth_sntp_init(void)
{
  setenv("TZ", "BRST+3BRDT+2,M10.3.0,M2.3.0", 1);
  tzset();

  // yield
  vTaskDelay(1000 / portTICK_PERIOD_MS);

  sntp_setoperatingmode(SNTP_OPMODE_POLL);
  sntp_setservername(0, "pool.ntp.org");
  sntp_init();
}

#endif // CONFIG_ETH_SNTP_ENABLE

#endif // CONFIG_ETH_DNS_ENABLE

// callbacks:

/** Event handler for IP_EVENT_ETH_GOT_IP */
static void got_ip_event_handler(void *arg, esp_event_base_t event_base,
                                 int32_t event_id, void *event_data)
{
  // Se o IP foi recebido pelo ESP32, significa que o serviço dhcp_c (client) está habilitado por padrão
  ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
  const esp_netif_ip_info_t *ip_info = &event->ip_info;

  ESP_LOGI(TAG, "\nESP_LOGIIP Address"
                "\n~~~~~~~~~~~~~~~~~~~~~~~~"
                "\nETHIP:   %03d.%03d.%03d.%03d"
                "\nETHMASK: %03d.%03d.%03d.%03d"
                "\nETHGW:   %03d.%03d.%03d.%03d"
                "\n~~~~~~~~~~~~~~~~~~~~~~~~",
           IP2STR(&ip_info->ip), IP2STR(&ip_info->netmask), IP2STR(&ip_info->gw));

#ifdef CONFIG_ETH_DNS_ENABLE
  //configure DNS servers
  eth_dns_configure();

#ifdef CONFIG_ETH_SNTP_ENABLE
  //configure SNTP
  if(sntp_restart() == false)
  {
    eth_sntp_init();
  }
#endif //CONFIG_ETH_SNTP_ENABLE

#endif //CONFIG_ETH_DNS_ENABLE

}

// Event handler for IP_EVENT_ETH_GOT_IP
static void eth_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
  uint8_t mac_addr[6] = {0};
  /* we can get the ethernet driver handle from event data */
  esp_eth_handle_t eth_handle = *(esp_eth_handle_t *)event_data;

  switch (event_id)
  {

  case ETHERNET_EVENT_START:

    esp_eth_ioctl(eth_handle, ETH_CMD_G_MAC_ADDR, mac_addr);
    ESP_LOGI(TAG, "Ethernet Started");
    ESP_LOGI(TAG, "Ethernet HW Addr %02x:%02x:%02x:%02x:%02x:%02x",
             mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);

    break;

  case ETHERNET_EVENT_CONNECTED:

    ESP_LOGI(TAG, "Ethernet Link Up");

    break;

  case ETHERNET_EVENT_DISCONNECTED:

    ESP_LOGI(TAG, "Ethernet Link Down");
    break;

  case ETHERNET_EVENT_STOP:

    ESP_LOGI(TAG, "Ethernet Stopped");
    break;

  default:
    break;
  }
  // return ESP_OK;
}