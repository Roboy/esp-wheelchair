#include <string.h>
#include "tcpip_adapter.h"
#include "esp_event.h"
#include "esp_err.h"
#include "esp_log.h"

const char * ETH_EVENT = "eth_event";
static bool s_tcpip_adapter_compat = false;

#define ETH_DEFAULT_CONFIG(emac, ephy)   \
{                                    \
    .mac = emac,                     \
    .phy = ephy,                     \
    .check_link_period_ms = 2000,    \
    .stack_input = NULL,             \
    .on_lowlevel_init_done = NULL,   \
    .on_lowlevel_deinit_done = NULL, \
    .read_phy_reg = NULL,            \
    .write_phy_reg = NULL,           \
}

static void handleETH_ap_start(void *arg, esp_event_base_t base, int32_t event_id, void *data){
  printf("ETH start Hdl");

}

static void handleETH_ap_stop(void *arg, esp_event_base_t base, int32_t event_id, void *data){

}

static void handleETH_sta_start(void *arg, esp_event_base_t base, int32_t event_id, void *data){

}

static void handleETH_sta_stop(void *arg, esp_event_base_t base, int32_t event_id, void *data){

}

static void handleETH_sta_connected(void *arg, esp_event_base_t base, int32_t event_id, void *data){

}

static void handleETH_sta_disconnected(void *arg, esp_event_base_t base, int32_t event_id, void *data){

}

static void handleETH_sta_got_ip(void *arg, esp_event_base_t base, int32_t event_id, void *data){

}
