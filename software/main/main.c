/* Hello World Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"

#include "esp_system.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_event_loop.h"
#include "nvs_flash.h"

#include "sys/socket.h"
#include "lwip/err.h"
#include "lwip/sys.h"

#include "driver/gpio.h"
#include "driver/pwm.h"
#include "esp_timer.h"

#include "sdkconfig.h"
#include "ros_comms.h"

// WIFI and Network config
#define HOST_IP_ADDR "192.168.0.105"
#define PORT 8000

#define ESP_WIFI_SSID       "roboy-robot"
#define ESP_WIFI_PASS       "wiihackroboy"
#define ESP_MAXIMUM_RETRY   10
#define ESP_HOSTNAME        "wifeel-chair"

static const char* TAG = "WIFEEL-CHAIR_INFO";

/* FreeRTOS event group to signal when we are connected*/
static EventGroupHandle_t wifi_event_group;

/* The event group allows multiple bits for each event, but we only care about one event
 * - are we connected to the AP with an IP? */
const int WIFI_CONNECTED_BIT = BIT0;

tcpip_adapter_ip_info_t ipInfo;     //Current IP info

// PWM Parameters

// const uint32_t pwm_pins[N_PWM_PINS] = {
//             GPIO_NUM_4,
//             GPIO_NUM_12,
//             GPIO_NUM_13,
//             GPIO_NUM_14
// };


const uint32_t pwm_pins[N_PWM_PINS] = {
            GPIO_NUM_14,
            GPIO_NUM_12,
            GPIO_NUM_15,
            GPIO_NUM_13

};



static esp_err_t event_handler(void *ctx, system_event_t *event)
{
    static const char* TAG = "wifi-event";

    /* For accessing reason codes in case of disconnection */
    system_event_info_t *info = &event->event_info;

    switch(event->event_id) {
        case SYSTEM_EVENT_STA_START:
            esp_wifi_connect();
            ESP_ERROR_CHECK(tcpip_adapter_set_hostname(TCPIP_ADAPTER_IF_STA, ESP_HOSTNAME));
            break;
        case SYSTEM_EVENT_STA_GOT_IP:
            ESP_LOGI(TAG, "got ip:%s",
                     ip4addr_ntoa(&event->event_info.got_ip.ip_info.ip));
            xEventGroupSetBits(wifi_event_group, WIFI_CONNECTED_BIT);
            ESP_ERROR_CHECK(tcpip_adapter_get_ip_info(TCPIP_ADAPTER_IF_STA, &ipInfo));
            break;
        case SYSTEM_EVENT_AP_STACONNECTED:
            ESP_LOGI(TAG, "station:"MACSTR" join, AID=%d",
                     MAC2STR(event->event_info.sta_connected.mac),
                     event->event_info.sta_connected.aid);
            break;
        case SYSTEM_EVENT_AP_STADISCONNECTED:
            ESP_LOGI(TAG, "station:"MACSTR"leave, AID=%d",
                     MAC2STR(event->event_info.sta_disconnected.mac),
                     event->event_info.sta_disconnected.aid);
            break;
        case SYSTEM_EVENT_STA_DISCONNECTED:
            ESP_LOGE(TAG, "Disconnect reason : %d", info->disconnected.reason);
            if (info->disconnected.reason == WIFI_REASON_BASIC_RATE_NOT_SUPPORT) {
                /*Switch to 802.11 bgn mode */
                esp_wifi_set_protocol(ESP_IF_WIFI_STA, WIFI_PROTOCOL_11B | WIFI_PROTOCOL_11G | WIFI_PROTOCOL_11N);
            }
            duties[0] = 0;                    //Set all PWM to 0
            duties[1] = 0;
            duties[2] = 0;
            duties[3] = 0;
            for ( int i = 0; i < 6; i++){       //Stop motors on connection loss
              
              gpio_set_level(gpio_pins[i],1);   
            }

            ESP_ERROR_CHECK( pwm_set_duties(duties) );
            ESP_ERROR_CHECK( pwm_start() );

            esp_wifi_connect();                 //Try to reconnect
            xEventGroupClearBits(wifi_event_group, WIFI_CONNECTED_BIT);
            ESP_LOGI(TAG,"Retrying connection to AP");
            break;

        default:
            break;
    }
    return ESP_OK;
}


void wifi_init_sta()
{
    // WIFI_CONNECTED = WIFI_CONNECTED_BIT;
    ESP_LOGI(TAG, "wifi connected: %d", WIFI_CONNECTED_BIT);
    nvs_flash_init();  

    wifi_event_group = xEventGroupCreate();

    tcpip_adapter_init();
    ESP_ERROR_CHECK(esp_event_loop_init(event_handler, NULL) );

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    wifi_config_t wifi_config = {
            .sta = {
                    .ssid = ESP_WIFI_SSID,
                    .password = ESP_WIFI_PASS
            },
    };

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA) );
    ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config) );
    ESP_ERROR_CHECK(esp_wifi_start());


    ESP_LOGI(TAG, "Waiting for AP connection...");
    xEventGroupWaitBits(wifi_event_group, WIFI_CONNECTED_BIT, false, true, portMAX_DELAY);
    ESP_LOGI(TAG, "Connected to AP with SSID:%s", ESP_WIFI_SSID);
}

static void ros_spin_task(void *pvParameters)
{
  TickType_t xLastWakeTime;
  xLastWakeTime = xTaskGetTickCount();    // Get initial count at task start

  for ( ;; )
  {
    rosserial_spinonce();

    vTaskDelayUntil( &xLastWakeTime, pdMS_TO_TICKS( 10 ));
  }
}

static void status_task(void *pvParameters)
{
    char addr_str[128];
    int addr_family;
    int ip_protocol;
    char* test_str = "Hola\n";
    char test_buf[100] = "Hola\n";

    printf("status_task started!");

    while (1) {

        struct sockaddr_in dest_addr;
        dest_addr.sin_addr.s_addr = inet_addr(HOST_IP_ADDR);
        dest_addr.sin_family = AF_INET;
        dest_addr.sin_port = htons(PORT);
        addr_family = AF_INET;
        ip_protocol = IPPROTO_IP;
        inet_ntoa_r(dest_addr.sin_addr, addr_str, sizeof(addr_str) - 1);
        snprintf(test_buf, 100, "%sMy IP address is: " IPSTR "\n", test_str, 
            IP2STR(&ipInfo.ip));

        int sock = socket(addr_family, SOCK_DGRAM, ip_protocol);
        if (sock < 0) {
            ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
            break;
        }

        while (1) {
            int err = sendto(sock, test_buf, sizeof(test_buf), 0, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
            if (err < 0) {
//                ESP_LOGE(TAG, "Error occurred during sending: errno %d", errno);
                break;
            }
            ESP_LOGI(TAG, "Message sent");
            vTaskDelay(2000 / portTICK_PERIOD_MS);
        }

        if (sock != -1) {
//            ESP_LOGE(TAG, "Shutting down socket and restarting...");
            shutdown(sock, 0);
            close(sock);
        }
    }
    vTaskDelete(NULL);
}


// void app_main()
// {
//     wifi_init_sta();
//     rosserial_setup();
//     while(1) {
//         rosserial_spinonce();
//         vTaskDelay(100);
//     }
// }

void app_main()
{
    // printf("Hello world!\n");
    // ESP_LOGI(TAG, "wifi connected: %d", WIFI_CONNECTED_BIT);
   
    taskENTER_CRITICAL();  // Disable interrupts while setting up tasks and config

    // xTaskCreate(&status_task,"status_task",2048,NULL,2,NULL);
    // printf("status_task starting\n");

    
    gpio_config_t io_conf;

    io_conf.intr_type = GPIO_INTR_DISABLE;    //Disable interrupts
    io_conf.mode = GPIO_MODE_OUTPUT;          //Set as outputs

    io_conf.pin_bit_mask = (GPIO_Pin_13|GPIO_Pin_14|GPIO_Pin_15|GPIO_Pin_12|GPIO_Pin_4|GPIO_Pin_5);
    io_conf.pull_down_en = 0;                 //Disable pull up/downs
    io_conf.pull_up_en = 0;
    gpio_config(&io_conf);
    

    gpio_set_level(GPIO_NUM_14, 0);
    // gpio_set_level(GPIO_NUM_13, 0);
    // pull enable pins HIGH
    gpio_set_level(GPIO_NUM_4, 1);
    gpio_set_level(GPIO_NUM_5, 1);

    // //Init PWM
    duties[0] = 0;
    duties[1] = 0;
    duties[2] = 0;
    duties[2] = 0;
    pwm_init(PWM_PERIOD, duties, N_PWM_PINS, pwm_pins);
    pwm_set_phases(phases);
    pwm_start();
    

    taskEXIT_CRITICAL();  // Reenable interrupts

    printf("PWM online\n");

     // Init wifi and network stack
    //wifi_init_sta();

    
    // Init ROS
    rosserial_setup();


    vTaskDelay(1000 / portTICK_PERIOD_MS);

    // Spinning ROS
    // xTaskCreate(&ros_spin_task,"ros_spin_task",4096,NULL,4,NULL);

    // Enable timer as ROS communication watchdog
    //ESP_ERROR_CHECK( esp_timer_start_once(timer_handle, TIMEOUT_IN_US) );
    bool connected = true;
    for (;;) {    // Spin ROS every ~1 ms -ish

        
        if (!connected) {
            // vTaskDelay(1000 / portTICK_PERIOD_MS);
            // connected = rosserial_spinonce();

            // if (! connected) {

                ESP_LOGI(TAG,"Restarting now. (Not really, just exited the main loop though!)\n");
                esp_restart();
            // }
        }

        connected = rosserial_spinonce();
        vTaskDelay(1 / portTICK_PERIOD_MS);
        // ESP_LOGI(TAG, "wifi connected: %d", connected);
        
    }


    ESP_LOGI(TAG,"Restarting now. (Not really, just exited the main loop though!)\n");
    esp_restart();
    // fflush(stdout);
}
