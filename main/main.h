#pragma once

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
/*
#include "esp_wifi.h"
#include "lwip/sockets.h"
#include "lwip/dns.h"
#include "lwip/netdb.h"
#include "mqtt_client.h"
*/
/*
 *END Alvaro Patacchiola WIFi prisma 23/04/2020
  */