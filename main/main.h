
#include <stdint.h>
#include <string.h>
//#include <stdbool.h>
#include <stdio.h>
//#include <stdlib.h>


#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_task_wdt.h"
//#include "freertos/event_groups.h"

#include "esp_system.h"
//#include "efuse_reg.h"

#include "esp_log.h"


#include "nvs.h"
#include "nvs_flash.h"

#include "esp_bt.h"
//#include "esp_bt_device.h"
//#include "esp_bt_defs.h"
#include "esp_bt_main.h"
//#include "esp_gatt_defs.h"
#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"

//#include "core.h"

#include "driver/uart.h"
#include "driver/adc.h"
#include "driver/periph_ctrl.h"
#include "driver/gpio.h"

#include "driver/timer.h"
//#include "soc/timer_group_struct.h"

/*
 *START Alvaro Patacchiola WIFi prisma 23/04/2020 p
*/
#include "freertos/event_groups.h"
#include "esp_event_loop.h"

#include "esp_wifi.h"
#include "lwip/sockets.h"
#include "lwip/dns.h"
#include "lwip/netdb.h"
#include "mqtt_client.h"
#include "tcpip_adapter.h"
#include "esp_smartconfig.h"
#include "esp_wpa2.h"
#include "esp_err.h"

enum enum_Status{mqttDisconnected = 0, mqttConnected = 1};

static wifi_config_t wifi_config = {
	.sta = {
	.ssid = "",
	.password = "",
},
};
typedef struct {
	uint8_t mode;
	uint32_t ip;
	uint32_t subnet;
	uint32_t gateway;
	uint32_t dns;
}ip_;
typedef  union  bit32
{
	uint32_t value;
	struct {
		uint8_t ad_low;
		uint8_t ad_low1;
		uint8_t ad_high;
		uint8_t ad_high1;
	}BIT;
} bit32_  ;

static EventGroupHandle_t wifi_event_group;
const static int CONNECTED_BIT = BIT0;
static const char *TAG = "MQTTS_PRISMA";
//static const char *serialNumber = "00000000000000000";
uint8_t serialNumber[18] = {'0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '1',0};
uint8_t macAddress[18] = { 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0 };
//static const char *topicNameW = "/appW";
char topicNameW[30] = { 0 };
//static const char *topicNameR = "/appR";
char topicNameR[30] = { 0 };
static const char *topicNameMail = "/00000000000000000/appMail";
const int WIFI_CONNECTED_BIT = BIT0;

static ip_ ip;
esp_mqtt_client_handle_t client;
uint8_t statusConnectionMQTT;	

static void Decodifica_Comando_wifi(uint8_t *);
#define MAX_APs 20
#define debug // decommentare per hw prisma, usato per testarlo sul hw MDWifi con rs485


void scanNetworkWiFi(uint8_t *);
uint8_t calcolaChecksum(uint8_t );
void getIPNetwork(uint8_t *);
void setIPNetwork(uint8_t *);
void put32(uint32_t, uint8_t *);
uint32_t get32(uint8_t *, uint8_t *);

void setInitNetwork();
void setSSID(uint8_t *);
void writeEEpromData();
void writeEEpromIP();
void readEEpromData();
//uso il pin LED_BLE per gestire la direzione
static void wifi_connect(void);
void getSSID(uint8_t * );
void getConnectionInfo(uint8_t *);
void setSerialNumber(uint8_t *);
	

/*
 *END Alvaro Patacchiola WIFi prisma 23/04/2020
  */

