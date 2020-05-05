#include "main.h"

/*
 *START Alvaro Patacchiola WIFi prisma 23/04/2020 p
*/
//#include "WiFi.h"
/*
 *END START Alvaro Patacchiola WIFi prisma 23/04/2020 p
*/

//#include "time.c"

/*----------------------------------------------------------------------------------------------------------------------------------------------------*/

#define spp_sprintf(s,...)          sprintf((char*)(s), ##__VA_ARGS__)

#define NOP() asm volatile ("nop")

#define SPP_DATA_MAX_LEN			(256)
//#define SPP_DATA_BUFF_MAX_LEN     (2*1024)

#define GATTS_TABLE_TAG				"SEC_GATTS_DEMO"
#define SPP_PROFILE_NUM             1
#define SPP_PROFILE_APP_IDX         0
#define ESP_SPP_APP_ID              0x56
//#define SAMPLE_DEVICE_NAME          "EMEC PRISMA"  
#define SPP_SVC_INST_ID	            0

#define LED_UART		(GPIO_NUM_0)	
#define LED_BLE			(GPIO_NUM_2)
#define ECHO_TEST_TXD	(GPIO_NUM_4)
#define ECHO_TEST_RXD	(GPIO_NUM_5)
#define ECHO_TEST_RTS	(UART_PIN_NO_CHANGE)
#define ECHO_TEST_CTS	(UART_PIN_NO_CHANGE)
#define IN_SEFL			(GPIO_NUM_27) //27
#define PWM_OUT			(GPIO_NUM_21) //21
static const uint16_t spp_service_uuid = 0xABF0;   	// SPP over BLE Service UUID
#define ESP_GATT_UUID_SPP_DATA			 0xABF1		// Characteristic UUID
#define ESP_GATT_UUID_SPP_NOTIFY		 0xABF2		// Characteristic UUID

//Attributes State Machine
enum {
	SPP_IDX_SVC,

	SPP_IDX_SPP_DATA_RECV_CHAR,
	SPP_IDX_SPP_DATA_RECV_VAL,
	
	SPP_IDX_SPP_DATA_NOTIFY_CHAR,
	SPP_IDX_SPP_DATA_NTY_VAL,
	SPP_IDX_SPP_DATA_NTF_CFG,

	SPP_IDX_NB,
}
;
static uint16_t spp_handle_table[SPP_IDX_NB];

//pacchetto di advertising 
static uint8_t spp_adv_data[25] = { 0x02, 0x01, 0x06, 0x15, 0x09, 0x50, 0x52, 0x49, 0x53, 0x4D, 0x41, 0x5B, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x5D };    //rimosso EMEC e gli spazi 21/10/2019 per problematica visibilità MAC Address su iOS
static char gap_complete_local_name[21] = { 0x50, 0x52, 0x49, 0x53, 0x4D, 0x41, 0x5B, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x5D, '\0' };   // tutti e soli i bytes del campo 0x09 di spp_data_adv "Nome[MACaddress]" più il terminatore

							  /*--------------------------*/ /*-----------------------PRISMA-MACaddress---------------------------------*/ /*-------*/		/*-------------Scrivo alla pompa-----------------------*/
static uint8_t status1[20] = { 0xFF, 0x30, 0x01, 0xFE, 0x01, 0x50, 0x52, 0x49, 0x53, 0x4D, 0x41, 0x2D, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0x00 };     //msg stato -> cmd ricevuto e valido 
static uint8_t status2[20] = { 0xFF, 0x30, 0x01, 0xFE, 0x02, 0x50, 0x52, 0x49, 0x53, 0x4D, 0x41, 0x2D, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0x00 };     //msg stato -> ripeti comando (cmd ricevuto non valido)
static uint8_t status3[20] = { 0xFF, 0x30, 0x01, 0xFE, 0x03, 0x50, 0x52, 0x49, 0x53, 0x4D, 0x41, 0x2D, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0x00 };    	//msg stato -> bluetooth non connesso
static uint8_t status4[20] = { 0xFF, 0x30, 0x01, 0xFE, 0x04, 0x50, 0x52, 0x49, 0x53, 0x4D, 0x41, 0x2D, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0x00 };    	//msg stato -> bluetooth connesso
static uint8_t status6[20] = { 0xFF, 0x30, 0x01, 0xFE, 0x06, 0x50, 0x52, 0x49, 0x53, 0x4D, 0x41, 0x2D, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0x00 };     //msg stato -> sefl allarme
static uint8_t status7[20] = { 0xFF, 0x30, 0x01, 0xFE, 0x07, 0x50, 0x52, 0x49, 0x53, 0x4D, 0x41, 0x2D, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0x00 };     //msg stato -> uscita pwm abilitata
static uint8_t status8[20] = { 0xFF, 0x30, 0x01, 0xFE, 0x08, 0x50, 0x52, 0x49, 0x53, 0x4D, 0x41, 0x2D, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0x00 };     //msg stato -> uscita pwm disabilitata
static uint8_t status9[20] = { 0xFF, 0x30, 0x01, 0xFE, 0x09, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xC6 };     //msg stato -> bluetooth N/A 

static uint32_t prc					= 0;     //dato "percentuale" comunicato dalla prisma 0 --> 0%,  100000 --> 100%
static uint8_t max_fail				= 10;
static bool negedge					= false;
static bool fail					= false;
static uint64_t timer00_val			= 0;
static uint16_t spp_conn_id			= 0xffff;
static esp_gatt_if_t spp_gatts_if	= 0xff;
static uint8_t bt_mac_address[6] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
static esp_bd_addr_t spp_remote_bda = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
static bool bt_enable_data_ntf		= false;
static bool bt_fault				= false;
static bool bt_is_connected			= false;
static bool sefl_en					= false;
static bool pwm_en					= false;
static bool status_ack				= false;
static bool led_ble_on				= false;
static bool	led_uart_on				= false;
static bool app_uart_write_req		= false;
static uint16_t uart_write_req_id	= 0;
static uint8_t  app_uart_write_value[150];
static uint16_t app_uart_write_len  = 0;

/*
 *START Alvaro Patacchiola WIFi prisma 27/04/2020 p
*/
#define sizeUartWiFi 250
static uint8_t  uart_wifi_status[sizeUartWiFi];
/*
 *STOP Alvaro Patacchiola WIFi prisma 27/04/2020 p
*/


// config adv params
static esp_ble_adv_params_t spp_adv_params = {
	.adv_int_min = 0x0800,
	  //0x100,
	.adv_int_max = 0x0800,
	  //0x100,
	.adv_type = ADV_TYPE_IND,
	.own_addr_type = BLE_ADDR_TYPE_PUBLIC,
	.channel_map = ADV_CHNL_ALL,
	.adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};

struct gatts_profile_inst {
	esp_gatts_cb_t gatts_cb;
	uint16_t gatts_if;
	uint16_t app_id;
	uint16_t conn_id;
	uint16_t service_handle;
	esp_gatt_srvc_id_t service_id;
	uint16_t char_handle;
	esp_bt_uuid_t char_uuid;
	esp_gatt_perm_t perm;
	esp_gatt_char_prop_t property;
	uint16_t descr_handle;
	esp_bt_uuid_t descr_uuid;
};

typedef struct spp_receive_data_node {
	int32_t len;
	uint8_t * node_buff;
	struct spp_receive_data_node * next_node;
}spp_receive_data_node_t;

static spp_receive_data_node_t * temp_spp_recv_data_node_p1 = NULL;
static spp_receive_data_node_t * temp_spp_recv_data_node_p2 = NULL;

typedef struct spp_receive_data_buff {
	int32_t node_num;
	int32_t buff_size;
	spp_receive_data_node_t * first_node;
}spp_receive_data_buff_t;

static spp_receive_data_buff_t SppRecvDataBuff = {
	.node_num = 0,
	.buff_size = 0,
	.first_node = NULL
};

static void gatts_profile_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);

/* One gatt-based profile one app_id and one gatts_if, this array will store the gatts_if returned by ESP_GATTS_REG_EVT */
static struct gatts_profile_inst spp_profile_tab[SPP_PROFILE_NUM] = {
	[SPP_PROFILE_APP_IDX] = {
	.gatts_cb = gatts_profile_event_handler,
	.gatts_if = ESP_GATT_IF_NONE,
	/* Not get the gatt_if, so initial is ESP_GATT_IF_NONE */
},
};


/*SPP PROFILE ATTRIBUTES*/
#define CHAR_DECLARATION_SIZE   (sizeof(uint8_t))
static const uint16_t primary_service_uuid			= ESP_GATT_UUID_PRI_SERVICE;
static const uint16_t character_declaration_uuid	= ESP_GATT_UUID_CHAR_DECLARE;
static const uint16_t character_client_config_uuid	= ESP_GATT_UUID_CHAR_CLIENT_CONFIG;
static const uint8_t char_prop_notify				= ESP_GATT_CHAR_PROP_BIT_NOTIFY;
static const uint8_t char_prop_read_write			= ESP_GATT_CHAR_PROP_BIT_WRITE | ESP_GATT_CHAR_PROP_BIT_READ;
static const uint16_t spp_data_receive_uuid			= ESP_GATT_UUID_SPP_DATA;
static uint8_t spp_data_receive_val[SPP_DATA_MAX_LEN];  //= { 0x00 };
static const uint16_t spp_data_notify_uuid			= ESP_GATT_UUID_SPP_NOTIFY;
//static uint8_t  spp_data_notify_val[128];
static uint8_t spp_data_notify_val[1] = { 0x00 };
static const uint8_t spp_data_notify_ccc[2] = { 0x00, 0x00 };


///Full HRS Database Description - Used to add attributes into the database
static const esp_gatts_attr_db_t spp_gatt_db[SPP_IDX_NB] =
{
	//SPP -  Service Declaration
	[SPP_IDX_SVC] = {
	{ ESP_GATT_AUTO_RSP },
{
	ESP_UUID_LEN_16,
	(uint8_t *)&primary_service_uuid,
	ESP_GATT_PERM_READ,
	sizeof(spp_service_uuid),
	sizeof(spp_service_uuid),
	(uint8_t *)&spp_service_uuid 
} 
},

	//SPP -  data receive characteristic Declaration
	[SPP_IDX_SPP_DATA_RECV_CHAR] = {
	{ ESP_GATT_AUTO_RSP },
{ 
	ESP_UUID_LEN_16,
	(uint8_t *)&character_declaration_uuid,
	ESP_GATT_PERM_READ,
	CHAR_DECLARATION_SIZE,
	CHAR_DECLARATION_SIZE,
	(uint8_t *)&char_prop_read_write
}
},

	//SPP -  data receive characteristic Value
	[SPP_IDX_SPP_DATA_RECV_VAL] = { 
	{ ESP_GATT_AUTO_RSP },
{
	ESP_UUID_LEN_16,
	(uint8_t *)&spp_data_receive_uuid,
	ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
	SPP_DATA_MAX_LEN,
	sizeof(spp_data_receive_val),
	(uint8_t *)spp_data_receive_val 
} 
},
	
	//SPP -  data notify characteristic Declaration
	[SPP_IDX_SPP_DATA_NOTIFY_CHAR] = { 
	{ ESP_GATT_AUTO_RSP },
{
	ESP_UUID_LEN_16,
	(uint8_t *)&character_declaration_uuid,
	ESP_GATT_PERM_READ,
	CHAR_DECLARATION_SIZE,
	CHAR_DECLARATION_SIZE,
	(uint8_t *)&char_prop_notify 
}
},

	//SPP -  data notify characteristic Value
	[SPP_IDX_SPP_DATA_NTY_VAL] = {
	{ ESP_GATT_AUTO_RSP },
{
	ESP_UUID_LEN_16,
	(uint8_t *)&spp_data_notify_uuid,
	ESP_GATT_PERM_READ,
	SPP_DATA_MAX_LEN,
	sizeof(spp_data_notify_val),
	(uint8_t *)spp_data_notify_val
} 
},

	//SPP -  data notify characteristic - Client Characteristic Configuration Descriptor
	[SPP_IDX_SPP_DATA_NTF_CFG] = { 
	{ ESP_GATT_AUTO_RSP },
{
	ESP_UUID_LEN_16,
	(uint8_t *)&character_client_config_uuid,
	ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
	sizeof(uint16_t), 
	sizeof(spp_data_notify_ccc),
	(uint8_t *)spp_data_notify_ccc
}
},

};


static uint8_t find_char_and_desr_index(uint16_t handle)
{
	uint8_t error = 0xff;

	for (int i = 0; i < SPP_IDX_NB; i++) {
		if (handle == spp_handle_table[i]) {
			return i;
		}
	}

	return error;
}

static bool store_wr_buffer(esp_ble_gatts_cb_param_t *p_data)
{
	temp_spp_recv_data_node_p1 = (spp_receive_data_node_t *)malloc(sizeof(spp_receive_data_node_t));

	if (temp_spp_recv_data_node_p1 == NULL) {
		ESP_LOGI(GATTS_TABLE_TAG, "malloc error %s %d\n", __func__, __LINE__);
		return false;
	}
	if (temp_spp_recv_data_node_p2 != NULL) {
		temp_spp_recv_data_node_p2->next_node = temp_spp_recv_data_node_p1;
	}
	temp_spp_recv_data_node_p1->len = p_data->write.len;
	SppRecvDataBuff.buff_size += p_data->write.len;
	temp_spp_recv_data_node_p1->next_node = NULL;
	temp_spp_recv_data_node_p1->node_buff = (uint8_t *)malloc(p_data->write.len);
	temp_spp_recv_data_node_p2 = temp_spp_recv_data_node_p1;
	memcpy(temp_spp_recv_data_node_p1->node_buff, p_data->write.value, p_data->write.len);
	if (SppRecvDataBuff.node_num == 0) {
		SppRecvDataBuff.first_node = temp_spp_recv_data_node_p1;
		SppRecvDataBuff.node_num++;
	}
	else {
		SppRecvDataBuff.node_num++;
	}

	return true;
}

//static void free_write_buffer(void) {
//	temp_spp_recv_data_node_p1 = SppRecvDataBuff.first_node;
//
//	while (temp_spp_recv_data_node_p1 != NULL) {
//		temp_spp_recv_data_node_p2 = temp_spp_recv_data_node_p1->next_node;
//		free(temp_spp_recv_data_node_p1->node_buff);
//		free(temp_spp_recv_data_node_p1);
//		temp_spp_recv_data_node_p1 = temp_spp_recv_data_node_p2;
//	}
//	SppRecvDataBuff.node_num = 0;
//	SppRecvDataBuff.buff_size = 0;
//	SppRecvDataBuff.first_node = NULL;
//}
//
//static void print_write_buffer(void) {
//	temp_spp_recv_data_node_p1 = SppRecvDataBuff.first_node;
//	while (temp_spp_recv_data_node_p1 != NULL) {
//		uart_write_bytes(UART_NUM_2, (char *)(temp_spp_recv_data_node_p1->node_buff), temp_spp_recv_data_node_p1->len);
//		temp_spp_recv_data_node_p1 = temp_spp_recv_data_node_p1->next_node;
//	}
//}

/*-------------------------------------------------Lampeggio LED--------------------------------------------------------------*/
static void led_uart_blink(void) {
	led_uart_on = true;
}

static void led_ble_blink(void) {
	led_ble_on = true;
}


/*-------------------------------------------------Interrupt Service Routine--------------------------------------------------------------*/
static void isr_ext_gpio_intr() {
	timer_pause(TIMER_GROUP_0, TIMER_0);
	timer_get_counter_value(TIMER_GROUP_0, TIMER_0, &timer00_val);
	timer_set_counter_value(TIMER_GROUP_0, TIMER_0, (uint64_t)0);
	timer_start(TIMER_GROUP_0, TIMER_0);
	negedge = true;
}

static void isr_timer00_intr() {
	fail = true;
	timer_set_alarm(TIMER_GROUP_0, TIMER_0, TIMER_ALARM_EN);
	TIMERG0.int_clr_timers.t0 = 1;
}

static void isr_timer10_intr() {
	timer_pause(TIMER_GROUP_1, TIMER_0);
	timer_set_counter_value(TIMER_GROUP_1, TIMER_0, (uint64_t)0);
	timer_set_alarm(TIMER_GROUP_1, TIMER_0, TIMER_ALARM_EN);
	TIMERG1.int_clr_timers.t0 = 1;
	timer_start(TIMER_GROUP_1, TIMER_1);
	gpio_set_level(PWM_OUT, 0);
}

static void isr_timer11_intr() {
	timer_pause(TIMER_GROUP_1, TIMER_1);
	timer_set_counter_value(TIMER_GROUP_1, TIMER_1, (uint64_t)0);
	timer_set_alarm(TIMER_GROUP_1, TIMER_1, TIMER_ALARM_EN);
	TIMERG1.int_clr_timers.t1 = 1;
	timer_start(TIMER_GROUP_1, TIMER_0);
	gpio_set_level(PWM_OUT, 1);
}
/*-------------------------------------------------End Interrupt Service Routine--------------------------------------------------------------*/

static bool Invia_stato_pm1wifi(uint8_t *pm1wifi_status, uint16_t len, bool pump_rsp)
{   
	bool rsp = false;
	if (pump_rsp == false)														//se la pompa non comunica la avvenuta ricezione...
		{
			uart_write_bytes(UART_NUM_2, (char *)pm1wifi_status, len);            	//reinvia msg alla pompa 
			led_uart_blink();      													//risegnala evento scrittura su uart
			rsp = false;
		}
	else
	{
		rsp = true;
	}
	return rsp;
}

static bool Interpreta_frame_uart(uint8_t byte_num0, uint8_t byte_num1, uint8_t byte_num3) {
	
	bool is_int_frame = false;
	
	if ((byte_num0 == 0xFF) && (byte_num3 == 0xFE))
	{
		/*
		 *START Alvaro Patacchiola WIFi prisma 23/04/2020 p
		 */
		
		if ((byte_num1 >= 0x30) && (byte_num1 <= 0x4F))//I COMANDI 0X30 TO 0X3F SONO PER BLE(ANGELO) 0X40 TO 0X4F SONO WIFI(ALVARO)
		/*
		 *END Alvaro Patacchiola WIFi prisma 23/04/2020 p
	    */
			
		{
			is_int_frame = true;
		}
		else
		{
			is_int_frame = false;
		}
	}
	return (is_int_frame);
}

static bool Verifica_Checksum(uint16_t frame_len, uint8_t frame[frame_len]) {

	bool is_valid_chksm = false;
	uint8_t val_chksm = 0x00;

	if (frame[frame_len - 1] == 0x00)
	{
		is_valid_chksm = false;
	}
	else
	{
		for (int i = 0; i <= frame_len - 2; i++) {
			val_chksm ^= frame[i];
		}
	
		if (val_chksm == frame[frame_len - 1])
		{
			is_valid_chksm = true;
		}
		else
		{
			is_valid_chksm = false;
		}
	}
	
	return is_valid_chksm;
}
/*
 *START Alvaro Patacchiola WIFi prisma 23/04/2020 p
*/

static void Decodifica_Comando_wifi(uint8_t * arrayTemp,uint16_t lenArray)
{
	uint16_t cmd_code = 0x0000;
	uint8_t lunghezzaMessaggio = 0;
	int msg_id=0;
	memset(uart_wifi_status, 0, sizeUartWiFi);
	
	cmd_code += arrayTemp[1];
	cmd_code <<= 8;
	cmd_code += arrayTemp[2];
	uart_wifi_status[0] = 0xFF;
	uart_wifi_status[1] = arrayTemp[1];
	uart_wifi_status[2] = arrayTemp[2];
	uart_wifi_status[3] = 0xFE;
	lunghezzaMessaggio = 4;
	switch (cmd_code) {
		case 0x4000://scansione delle reti wifi
			scanNetworkWiFi(&lunghezzaMessaggio);
		break;
		case 0x4100://lettura parametri di rete
			getIPNetwork(&lunghezzaMessaggio);
		break;
		case 0x4200://impostazione AP
			setSSID(arrayTemp);
		break;
		case 0x4300://restituisce l'attuale AP o SSID impostato
			getSSID(&lunghezzaMessaggio);
		break;
		case 0x4400://restituisce il livello del segnale wifi e dello stato di connessione al broker
			getConnectionInfo(&lunghezzaMessaggio);
		break;
		case 0x4500://impostazione dei parametri IP, statico o DHCP, ip, subnetmask, gateway e dns
			setIPNetwork(arrayTemp);
		break;
		case 0x4600://lettura del serialNumber dalla pompa
			setSerialNumber(arrayTemp);
		break;
		case 0x4700://invio messagi di allarme verso il server
				if (statusConnectionMQTT == mqttConnected) // se connesso invio il messaggio di allarme al server broker
					//esp_mqtt_client_subscribe(client, topicNameMail, 1);
					msg_id = esp_mqtt_client_publish(client, topicNameMail, (char *)arrayTemp, lenArray, 0, 0);
		
				if (msg_id > 0)
					uart_wifi_status[lunghezzaMessaggio] = 1;
				else
					uart_wifi_status[lunghezzaMessaggio] = 0;
			lunghezzaMessaggio++;
		break;
			
	}
	uart_write_req_id = 0x10; //codici di risposta del wifi
	uart_wifi_status[lunghezzaMessaggio] = 0xFF;
	uart_wifi_status[lunghezzaMessaggio + 1] = calcolaChecksum(lunghezzaMessaggio);
}

uint8_t calcolaChecksum(uint8_t lunghezzaMessaggio)
{
	uint8_t xorResult = 0;
	
	for (uint8_t i = 0; i <= lunghezzaMessaggio; i++) {
		xorResult = xorResult ^ uart_wifi_status[i];
	}
	return xorResult;
	
}

/*
 *END Alvaro Patacchiola WIFi prisma 23/04/2020 p
*/
	
static void Decodifica_Comando(uint8_t byte_num1, uint8_t byte_num2, uint8_t byte_num4, uint8_t byte_num5, uint8_t byte_num6, uint8_t byte_num7) {	
	
	uint16_t cmd_code = 0x0000;
	cmd_code += byte_num1;
	cmd_code <<= 8;
	cmd_code += byte_num2;
	
	uint32_t data_code = 0x00000000;
	data_code += byte_num7;
	data_code <<= 8;
	data_code += byte_num6;
	data_code <<= 8;
	data_code += byte_num5;
	data_code <<= 8;
	data_code += byte_num4;
	
	switch (cmd_code) {
		
	case 0x3000:	//la pompa comunica di aver ricevuto il msg di stato correttamente
		printf("CMD 0x3000 Response to a pm1wifi Status send\n");
		status_ack = (data_code >> 24) & 0x00000001;
		//byte_num1 = 0x00;
		//byte_num2 = 0x00;
		//byte_num4 = 0x00;
		//byte_num5 = 0x00;
		break;

	case 0x3300:	//abilitazione ingresso sefl
		printf("CMD 0x3300 IN SE-FL Abilitato\n");
		prc = data_code;
		
		gpio_intr_enable(IN_SEFL); 															//configuro e abilito interrupt IO esterno
		gpio_install_isr_service(ESP_INTR_FLAG_LEVEL2);
		gpio_isr_handler_add(IN_SEFL, isr_ext_gpio_intr, NULL);
		
		float new_prc = (float)prc / 100000;   												//dalla PRISMA ricevo il dato (di tipo uint16_t) relativo ""tempo di sefl" il cui valore se 0 --> 0%, 3000 --> 3%, 100000 --> 100%
		printf("%1.2f\n", new_prc);
		uint32_t T_l00prc = 334;   															//minimo "tempo di sefl" in ms --> è una caratteristica della pompa
		uint32_t T_nom_sefl = (float)T_l00prc / new_prc;   									//calcolo tempo tra due impulsi consecutivi 
		printf("T_nom_sefl: %0d ms\n", T_nom_sefl);
		const uint8_t dT = 10;   															//massimo eccesso consentito sul tempo di sefl in percentuale
		uint32_t T_max_sefl = (float)T_nom_sefl + ((float)T_nom_sefl * (float)1 / dT);   	//calcolo massimo tempo tra due impulsi consecutivi
		printf("T_max_sefl: %d ms\n", T_max_sefl);
		printf("Inizializzo Timer 0 Gruppo 0 ...\n");
		const uint32_t timer00_div = 40000;  												//definisco divisore di frequenza del timer
		//utilizzare la 1) o la 2)
		//1) risoluzione conteggio timer t_tick = 80MHz/ 40000 = 0.5ms
		//const float timer00_res_ms = 0.5;	
		//2) definisco un moltiplicatore pari a 1/risoluzione in ms
		const uint8_t timer00_mult = 2;
		
		timer_set_divider(TIMER_GROUP_0, TIMER_0, timer00_div);  							//configuro primo timer del primo gruppo
		timer_set_counter_mode(TIMER_GROUP_0, TIMER_0, TIMER_COUNT_UP);  					//modalità up-counting
		timer_pause(TIMER_GROUP_0, TIMER_0);  												//pausa
		timer_set_alarm(TIMER_GROUP_0, TIMER_0, TIMER_ALARM_EN);  							//abilito allarme
		TIMERG0.hw_timer[0].config.level_int_en = 1;  										//abilito interrupt sul conteggio
		timer_set_auto_reload(TIMER_GROUP_0, TIMER_0, TIMER_AUTORELOAD_EN);  				//abilito auto-ripartenza 
		timer_set_counter_value(TIMER_GROUP_0, TIMER_0, (uint64_t)0);  						//definisco valore di autoripartenza
		//utilizzare la 1) o la 2) coerentemente con sopra 
		/*1)*/ //uint64_t timer00_alrm_val = (float)T_max_sefl / timer00_res_ms;
		/*2)*/ uint64_t timer00_alrm_val = T_max_sefl * timer00_mult;
		printf("Valore arresto timer: %llu\n", timer00_alrm_val);
		timer_set_alarm_value(TIMER_GROUP_0, TIMER_0, timer00_alrm_val);  					//imposto il valore del conteggio per l'allarme
		timer_enable_intr(TIMER_GROUP_0, TIMER_0);  											//configuro ed abilito l'interrupt
		timer_isr_register(TIMER_GROUP_0, TIMER_0, isr_timer00_intr, NULL, ESP_INTR_FLAG_LEVEL1, NULL);
		timer_start(TIMER_GROUP_0, TIMER_0); 												//avvio contatore
		
		sefl_en = true;
		//byte_num1 = 0x00;
		//byte_num2 = 0x00;
		//byte_num4 = 0x00;
		//byte_num5 = 0x00;
		break;

	case 0x3400:	//disabilitazione ingresso sefl
		printf("CMD 0x3400 IN SE-FL Disabilitato\n");
		timer_pause(TIMER_GROUP_0, TIMER_0);
		gpio_uninstall_isr_service(ESP_INTR_FLAG_LEVEL2);
		gpio_intr_disable(IN_SEFL);
		timer_disable_intr(TIMER_GROUP_0, TIMER_0); 	
		sefl_en = false;
		//byte_num1 = 0x00;
		//byte_num2 = 0x00;
		//byte_num4 = 0x00;
		//byte_num5 = 0x00;
		break;
		
	case 0x3500:	//impostazione duty-cycle uscita impulsiva
		printf("CMD 0x3500 Abilita uscita PWM\n");
		
		uint16_t dutycycle_int = (uint16_t)data_code / 100;
		uint16_t dutycycle_dec = (uint16_t)data_code - (dutycycle_int * 100);
		printf("%d\n", dutycycle_int);
		printf("%d\n", dutycycle_dec);
		
		const uint16_t numTick_T = 10000; 
		uint16_t numTick_Ton = ((dutycycle_int * 100) + dutycycle_dec); 						//Ton
		uint16_t numTick_Toff = numTick_T - numTick_Ton;  									//Toff
		
		const uint32_t timerg1_div = 64;
		timer_set_divider(TIMER_GROUP_1, TIMER_0, timerg1_div);    							//configuro primo timer del secondo gruppo
		timer_set_counter_mode(TIMER_GROUP_1, TIMER_0, TIMER_COUNT_UP);   					//modalità up-counting
		timer_pause(TIMER_GROUP_1, TIMER_0);   												//pausa
		timer_set_alarm(TIMER_GROUP_1, TIMER_0, TIMER_ALARM_EN);   							//abilito allarme
		TIMERG1.hw_timer[0].config.level_int_en = 1;   										//abilito interrupt sul conteggio
		timer_set_auto_reload(TIMER_GROUP_1, TIMER_0, TIMER_AUTORELOAD_EN);   				//abilito auto-ripartenza 
		timer_set_counter_value(TIMER_GROUP_1, TIMER_0, (uint64_t)0);   						//definisco valore di autoripartenza
		printf("Valore arresto timer: %llu\n", (uint64_t)numTick_Ton);
		timer_set_alarm_value(TIMER_GROUP_1, TIMER_0, numTick_Ton);    						//imposto il valore del conteggio per l'allarme
		timer_enable_intr(TIMER_GROUP_1, TIMER_0);   										//configuro ed abilito l'interrupt
		timer_isr_register(TIMER_GROUP_1, TIMER_0, isr_timer10_intr, NULL, ESP_INTR_FLAG_LEVEL3, NULL);
		
		timer_set_divider(TIMER_GROUP_1, TIMER_1, timerg1_div);     							//configuro secondo timer del secondo gruppo
		timer_set_counter_mode(TIMER_GROUP_1, TIMER_1, TIMER_COUNT_UP);    					//modalità up-counting
		timer_pause(TIMER_GROUP_1, TIMER_1);    												//pausa
		timer_set_alarm(TIMER_GROUP_1, TIMER_1, TIMER_ALARM_EN);    							//abilito allarme
		TIMERG1.hw_timer[1].config.level_int_en = 1;    										//abilito interrupt sul conteggio
		timer_set_auto_reload(TIMER_GROUP_1, TIMER_1, TIMER_AUTORELOAD_EN);    				//abilito auto-ripartenza 
		timer_set_counter_value(TIMER_GROUP_1, TIMER_1, (uint64_t)0);    					//definisco valore di autoripartenza
		printf("Valore arresto timer: %llu\n", (uint64_t)numTick_Toff);
		timer_set_alarm_value(TIMER_GROUP_1, TIMER_1, numTick_Toff);     					//imposto il valore del conteggio per l'allarme
		timer_enable_intr(TIMER_GROUP_1, TIMER_1);    										//configuro ed abilito l'interrupt
		timer_isr_register(TIMER_GROUP_1, TIMER_1, isr_timer11_intr, NULL, ESP_INTR_FLAG_LEVEL2, NULL);
		if (numTick_Ton < 100)																//se duty cycle < 1% 
			{
				gpio_set_level(PWM_OUT, 0); 														//forza l'uscita a livello basso
			}
		else																				//se 1% <= duty cycle <= 99.9%
			{
				timer_start(TIMER_GROUP_1, TIMER_0);    											//avvio il conteggio del Ton
				gpio_set_level(PWM_OUT, 1); 														//forzo l'uscita pwm a livello alto 
			}
		pwm_en = true;
		//byte_num1 = 0x00;
		//byte_num2 = 0x00;
		//byte_num4 = 0x00;
		//byte_num5 = 0x00;
		break;
	
	case 0x3600:	//impostazione duty-cycle uscita impulsiva
		printf("CMD 0x3600 Disabilita uscita PWM\n");
		timer_pause(TIMER_GROUP_1, TIMER_0);
		timer_pause(TIMER_GROUP_1, TIMER_1);
		timer_disable_intr(TIMER_GROUP_1, TIMER_0);
		timer_disable_intr(TIMER_GROUP_1, TIMER_1);
		gpio_set_level(PWM_OUT, 0);
		pwm_en = false;
		//byte_num1 = 0x00;
		//byte_num2 = 0x00;
		//byte_num4 = 0x00;
		//byte_num5 = 0x00;
		break;
		
	default:
		break;
	}
}


static void spp_uart_init(void)
{
	uart_config_t uart_config = {
		.baud_rate = 38400,
		 					//per comunicazione pompa PRISMA
		.data_bits = UART_DATA_8_BITS,
		.parity = UART_PARITY_DISABLE,
		.stop_bits = UART_STOP_BITS_1,
		.flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
	};
	//Set UART parameters
	uart_param_config(UART_NUM_2, &uart_config);
	//Set UART pins
	uart_set_pin(UART_NUM_2, ECHO_TEST_TXD, ECHO_TEST_RXD, ECHO_TEST_RTS, ECHO_TEST_CTS);
	//Install UART driver, and get the queue.
	uart_driver_install(UART_NUM_2, 2048, 0, 0, NULL, 0);
	//Set UART mode
	uart_set_mode(UART_NUM_2, UART_MODE_UART); 
/*
 *START Alvaro Patacchiola WIFi prisma 23/04/2020 p
*/

}

static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
	
	ESP_LOGV(GATTS_TABLE_TAG, "GAP_EVT, event %d\n", event);

	switch (event) {
		
	case ESP_GAP_BLE_ADV_DATA_RAW_SET_COMPLETE_EVT:
		//adv_config_done &= (~ADV_CONFIG_FLAG);
		//if (adv_config_done == 0) {
		esp_ble_gap_start_advertising(&spp_adv_params);
		//}
		break;
	    
	case ESP_GAP_BLE_ADV_START_COMPLETE_EVT:
		//advertising start complete event to indicate advertising start successfully or failed
		if(param->adv_start_cmpl.status != ESP_BT_STATUS_SUCCESS) {
			ESP_LOGE(GATTS_TABLE_TAG, "advertising start failed, error status = %x", param->adv_start_cmpl.status);
			break;
		}
		ESP_LOGI(GATTS_TABLE_TAG, "advertising start success");
		break;
			
	default:
		break;
	}
		
}

static void gatts_profile_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param)
{
	esp_ble_gatts_cb_param_t *p_data = (esp_ble_gatts_cb_param_t *) param;
	uint8_t res = 0xff;
	
	ESP_LOGV(GATTS_TABLE_TAG, "event = %x\n", event);
	switch (event) {
		
	case ESP_GATTS_REG_EVT:
		ESP_LOGI(GATTS_TABLE_TAG, "%s %d\n", __func__, __LINE__);
		//esp_ble_gap_set_device_name(SAMPLE_DEVICE_NAME);
		esp_ble_gap_set_device_name(gap_complete_local_name);

		ESP_LOGI(GATTS_TABLE_TAG, "%s %d\n", __func__, __LINE__);
		esp_ble_gap_config_adv_data_raw((uint8_t *)spp_adv_data, sizeof(spp_adv_data));
	    
		ESP_LOGI(GATTS_TABLE_TAG, "%s %d\n", __func__, __LINE__);
		esp_ble_gatts_create_attr_tab(spp_gatt_db, gatts_if, SPP_IDX_NB, SPP_SVC_INST_ID);
	    
	case ESP_GATTS_READ_EVT:
		led_ble_blink();
		res = find_char_and_desr_index(p_data->read.handle);
		//sprintf((char*)spp_data_notify_val, "%d", 0);
		//esp_ble_gatts_send_indicate(spp_gatts_if, spp_conn_id, spp_handle_table[SPP_IDX_SPP_DATA_NTY_VAL], strlen((char*)spp_data_notify_val), (uint8_t *)spp_data_notify_val, false);
		break;
	    
	case ESP_GATTS_WRITE_EVT:
		led_ble_blink();
		res = find_char_and_desr_index(p_data->write.handle);
		if (p_data->write.is_prep == false) {
			ESP_LOGI(GATTS_TABLE_TAG, "ESP_GATTS_WRITE_EVT : handle = %d\n", res);
			if (res == SPP_IDX_SPP_DATA_NTF_CFG) {
				if ((p_data->write.len == 2)&&(p_data->write.value[0] == 0x01)&&(p_data->write.value[1] == 0x00)) {
					bt_enable_data_ntf = true;
				}
				else if ((p_data->write.len == 2)&&(p_data->write.value[0] == 0x00)&&(p_data->write.value[1] == 0x00)) {
					bt_enable_data_ntf = false;
				}
			}
			else if (res == SPP_IDX_SPP_DATA_RECV_VAL) {
				app_uart_write_len = p_data->write.len;
				memcpy(app_uart_write_value, (char *)(p_data->write.value), p_data->write.len);
				for (int i = 0; i < p_data->write.len; i++)
				{
					printf("%x\t", app_uart_write_value[i]);
				}
				printf("\n");
				app_uart_write_req = true;
			}
			else {
				;
			}   
		}
		else if ((p_data->write.is_prep == true)&&(res == SPP_IDX_SPP_DATA_RECV_VAL)) {
			ESP_LOGI(GATTS_TABLE_TAG, "ESP_GATTS_PREP_WRITE_EVT : handle = %d\n", res);
			store_wr_buffer(p_data);
		}
		break;
	    
	case ESP_GATTS_EXEC_WRITE_EVT:
		//		ESP_LOGI(GATTS_TABLE_TAG, "ESP_GATTS_EXEC_WRITE_EVT\n");
		//		if (p_data->exec_write.exec_write_flag) {
		//			print_write_buffer();
		//			free_write_buffer();
		//		}
				break;
	    
	case ESP_GATTS_MTU_EVT:
		led_ble_blink();
		//		int a = p_data->mtu.mtu;
				break;
	    
	case ESP_GATTS_CONF_EVT:
		break;
	case ESP_GATTS_UNREG_EVT:
		break;
	case ESP_GATTS_DELETE_EVT:
		break;
	case ESP_GATTS_START_EVT:
		break;
	case ESP_GATTS_STOP_EVT:
		break;
	    
	case ESP_GATTS_CONNECT_EVT:
		led_ble_blink();
		ESP_LOGI(GATTS_TABLE_TAG, "ESP_GATTS_CONNECT_EVT");
		/* start security connect with peer device when receive the connect event sent by the master */
		//esp_ble_set_encryption(param->connect.remote_bda, ESP_BLE_SEC_ENCRYPT_MITM);
		bt_is_connected = true;
		spp_conn_id = p_data->connect.conn_id;
		spp_gatts_if = gatts_if;
		memcpy(&spp_remote_bda, &p_data->connect.remote_bda, sizeof(esp_bd_addr_t));
		break;
	    
	case ESP_GATTS_DISCONNECT_EVT:
		led_ble_blink();
		ESP_LOGI(GATTS_TABLE_TAG, "ESP_GATTS_DISCONNECT_EVT");
		/* start advertising again when missing the connect */
		esp_ble_gap_start_advertising(&spp_adv_params);
		bt_is_connected = false;
		bt_enable_data_ntf = false;
		break;
	    
	case ESP_GATTS_OPEN_EVT:
		break;
	case ESP_GATTS_CANCEL_OPEN_EVT:
		break;
	case ESP_GATTS_CLOSE_EVT:
		break;
	case ESP_GATTS_LISTEN_EVT:
		break;
	case ESP_GATTS_CONGEST_EVT:
		break;
		
	case ESP_GATTS_CREAT_ATTR_TAB_EVT:
		ESP_LOGI(GATTS_TABLE_TAG, "The number handle = %x", param->add_attr_tab.num_handle);
		if (param->create.status == ESP_GATT_OK)
		{
			if (param->add_attr_tab.num_handle == SPP_IDX_NB)
			{
				memcpy(spp_handle_table, param->add_attr_tab.handles, sizeof(spp_handle_table));
				esp_ble_gatts_start_service(spp_handle_table[SPP_IDX_SVC]);
			}
			else
			{
				ESP_LOGE(GATTS_TABLE_TAG, "Create attribute table abnormally, num_handle (%d) doesn't equal to SPP_IDX_NB(%d)", param->add_attr_tab.num_handle, SPP_IDX_NB);
			}
		}
		else
		{
			ESP_LOGE(GATTS_TABLE_TAG, " Create attribute table failed, error code = %x", param->create.status);
		}
		break;

	default:
		break;
	}
}


static void gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param)
{
	/* If event is register event, store the gatts_if for each profile */
	if (event == ESP_GATTS_REG_EVT) {
		if (param->reg.status == ESP_GATT_OK) {
			spp_profile_tab[SPP_PROFILE_APP_IDX].gatts_if = gatts_if;
		}
		else {
			ESP_LOGI(GATTS_TABLE_TAG,
				"Reg app failed, app_id %04x, status %d\n",
				param->reg.app_id,
				param->reg.status);
			return;
		}
	}

	do {
		int idx;
		for (idx = 0; idx < SPP_PROFILE_NUM; idx++) {
			if (gatts_if == ESP_GATT_IF_NONE || /* ESP_GATT_IF_NONE, not specify a certain gatt_if, need to call every profile cb function */
			        gatts_if == spp_profile_tab[idx].gatts_if) {
				if (spp_profile_tab[idx].gatts_cb) {
					spp_profile_tab[idx].gatts_cb(event, gatts_if, param);
				}
			}
		}
	} while (0);
}


//uint16_t exec_ad_conv()
//{
//	uint32_t adc_reading = 0;
//	uint16_t raw = 0;
//	const uint16_t num_avg = 128;
//	
//	//adc2_config_channel_atten(ADC2_CHANNEL_7, ADC_ATTEN_DB_11);
//	adc2_config_channel_atten(ADC2_CHANNEL_5, ADC_ATTEN_DB_11);
//	for (int i = 0; i < num_avg; i++)
//	{
//		raw = 0;
//		//adc2_get_raw(ADC2_CHANNEL_7, ADC_WIDTH_BIT_12, &raw);
//		adc2_get_raw(ADC2_CHANNEL_5, ADC_WIDTH_BIT_12, &raw);
//		adc_reading += raw;
//	}
//	adc_reading /= num_avg;
//	//esp_err_t esp_adc_off();
//	
//	//y = -0.00003*(x^2) + 0.8684*(x) + 90 -> funzione di interpolazione vedi file excel
//	const float coeff_a = -0.00003;
//	const float coeff_b = 0.86840;
//	const uint8_t coeff_c = 90;
//				
//	float anlg_vdc = 0.0;
//	if (adc_reading < 1)
//	{
//		anlg_vdc = 0.0;
//	}
//	else if ((adc_reading >= 1) && (adc_reading <= 4094))
//	{
//		//termine di secondo grado
//		anlg_vdc = (float)adc_reading * (float)adc_reading;
//		anlg_vdc *= coeff_a;
//		//termine di primo grado
//		anlg_vdc = anlg_vdc + ((float)adc_reading * coeff_b);
//		//termine noto
//		anlg_vdc = anlg_vdc + (float)coeff_c;	
//	}
//	else
//	{
//		anlg_vdc = 3165.0;
//	}		
//	printf("Passi ADC %d\t Vin %dmV\n", adc_reading, (uint16_t)anlg_vdc);
//	return ((uint16_t)anlg_vdc);
//}


/*-------------------------------------------------Task--------------------------------------------------------------*/

void task_principale(void *pvParameters) 
{
		
	static uint8_t temp[150]; 				  																	//array che conterrà il frame RX UART1 max 120 bytes
	memset(temp, 0x00, sizeof(temp));
	
	static bool esp32_cmd = false;
	static bool valid_frame = false;																			
	static uint16_t len = 0;  																					//lunghezza frame RX UART1
	static uint16_t tick = 0;  																					//contatore del numero delle ripetizioni del task
	
	//variabile di appoggio per il conteggio
	//uint64_t timer00_cnt = 0;
	
	static uint16_t cnt_negedge = 0; 																			//contatore fronti di discesa
	static uint8_t cnt_fail = 0; 																				//contatori fallimenti
	static bool sefl_alrm = false;
	
	//uart_flush_input(UART_NUM_2);
	//if (!bt_enable_data_ntf) { ESP_LOGE(GATTS_TABLE_TAG, "%s do not enable data Notify\n", __func__); }		//segnala l'abilitazione o meno della notifica
	
	while(1) {
		
		//BROADCASTING MSG STATO 
		tick += 1;       						//ogni ripetizione del task, incrementa di 1 il contatore (ogni 100ms)
		if(tick == 150)							//se trascorsi 15 sec (150 * 100ms vedi sotto il delay del task)
		{
			if (bt_fault == true)				//se errore BT
				{
					uart_write_req_id = 9;    		//vedi task_scritura_uart
				}
			else
			{
				if (bt_is_connected == false)	//in base allo stato della connessione BT
					{
						uart_write_req_id = 3;    	//vedi task_scritura_uart
					}								 
				else
				{ 
					uart_write_req_id = 4;    	//vedi task_scritura_uart
				}
			}
			tick = 0;        					//azzero var contatore	
		}
		
		
		
		//UART
		len = uart_read_bytes(UART_NUM_2, (uint8_t *)temp, sizeof(temp), 50);  							//lettura uart
		if(len > 0)																						//se almeno un byte
		{
			uart_flush_input(UART_NUM_2); 																//svuoto buffer rx
			led_uart_blink();  																		    //segnalo lettura su uart
		
			esp32_cmd = Interpreta_frame_uart(temp[0], temp[1], temp[3]);  								//esp32_cmd=1 se comando interno per modulo BT, esp32_cmd=0 se frame da scambiare con l'App
			printf("Dato Ricevuto %d\n", len);
			
			if(esp32_cmd == true)	 																	//se è un comando interno
			{																		
				valid_frame = Verifica_Checksum(len, temp);  											//valid_frame = 1 se checksum corretta, valid_frame = 0 se checksum errata
				if(valid_frame == true) {
																					//se la checksum è corretta
					
					if(status_ack == false) { uart_write_req_id = 1; }	
					//else {
					//	;
					//} //status_ack = true; }																
/*
 *START Alvaro Patacchiola WIFi prisma 23/04/2020 p
*/
					ESP_LOGI(TAG, "dATO RICEVUTO");
					if (temp[1] < 0x3F)
						Decodifica_Comando(temp[1], temp[2], temp[4], temp[5], temp[6], temp[7]);      		//decodifica il comando
					else
						Decodifica_Comando_wifi(temp, len);        		//decodifica il comando
/*
 *END Alvaro Patacchiola WIFi prisma 23/04/2020 p
*/
					
				}
				else {
					if (status_ack == false) { uart_write_req_id = 2; }	
					//else {
					//	;
					//}//status_ack = true; }
				}
			}
			else	//se non è un comando interno, il frame è da inviarlo all'App
			{																																											
				//sprintf((char*)spp_data_notify_val, "%d", 0);
				//esp_ble_gatts_send_indicate(spp_gatts_if, spp_conn_id, spp_handle_table[SPP_IDX_SPP_DATA_NTY_VAL], strlen((char*)spp_data_notify_val), (uint8_t *)spp_data_notify_val, false);
				
				if(bt_is_connected)																		//se bluetooth connesso
				{
					sprintf((char*)spp_data_notify_val, "%d", 1);										
					esp_ble_gatts_send_indicate(spp_gatts_if, spp_conn_id, spp_handle_table[SPP_IDX_SPP_DATA_NTY_VAL], strlen((char*)spp_data_notify_val), (uint8_t *)spp_data_notify_val, false);   	//invio la notifica all'App	
					
					esp_ble_gatts_set_attr_value(spp_handle_table[SPP_IDX_SPP_DATA_RECV_VAL], len, (uint8_t *)temp);  	//invia il frame all'App
					for(int i = 0 ; i < len ; i++)
					{
						printf("%x\t", temp[i]);
					}
					printf("\n");
					led_ble_blink(); 																	//segnalo evento su bluetooth
				}
/*
 *START Alvaro Patacchiola WIFi prisma 30/04/2020 p
*/
				if (statusConnectionMQTT == mqttConnected)
				{
					esp_mqtt_client_publish(client, topicNameR, (char *)temp, len, 0, 0);
					printf("invio Al Server MQTT");
					
				}
/*
 *START Alvaro Patacchiola WIFi prisma 30/04/2020 p
*/

			}
			memset(temp, 0x00, sizeof(temp)); 															//azzero il buffer temporaneo
		}
		
		//INGRESSO SEFL
		if(sefl_en == true) {
			//senza allarme
			if(sefl_alrm == false)
			{
				//fronte di discesa e no fallimento
				if(negedge == true)
				{
					cnt_negedge += 1;
					printf("Num Fronti: %d\n", (uint32_t)cnt_negedge);
					cnt_fail = 0;
					printf("Num Fallimenti: %d\n", (uint32_t)cnt_fail);
					//printf("Conteggio: %llu\n", timer00_val);
					negedge = false;
					sefl_alrm = false; 
				}
				
				//si detecta un fallimento
				if(fail == true)
				{
					cnt_fail += 1;
					if ((cnt_fail > 0) && (cnt_fail < max_fail))
					{
						printf("Num Fronti: %d\n", (uint32_t)cnt_negedge);
						printf("Num Fallimenti: %d\n", (uint32_t)cnt_fail);
						//timer00_cnt = timer00_val + timer00_alrm_val;
						//timer00_cnt *= cnt_fail;
						//printf("Conteggio: %llu\n", timer00_cnt);
						fail = false;
						sefl_alrm = false;
					}
					else
					{
						printf("Num Fallimenti: %d\n", (uint32_t)cnt_fail);
						printf("Allarme SEnsore FLusso!\n");
						sefl_alrm = true;
						cnt_fail = max_fail;
					}
				}
			}
			//in caso di allarme
			else {
				if (status_ack == false)
				{
					uart_write_req_id = 6;
				}	
				else
				{
					cnt_fail = 0;
					status_ack = false; 
					sefl_alrm = false;
					sefl_en = false;
				}
			}
		}
		else {
			cnt_negedge = 0;
			cnt_fail = 0;
			negedge = false;
			fail = false;
			sefl_alrm = false;
		}
		
		//USCITA PWM
		if(pwm_en == true)
		{
			if (status_ack == false)
			{
				uart_write_req_id = 7;
			}	
			//else
			//{
				//; //status_ack = true; 
			//}
		}
		else
		{
			if (status_ack == false)
			{
				/*
				 *START Alvaro Patacchiola WIFi prisma 27/04/2020 p
				 */
				
				//uart_write_req_id = 8;
				
				//COMMENTATO DA ALVARO PATACCHIOLA PER CERCARE DI INVIARE LE RISPOSTE DEL WIFI
				/*
				 *END Alvaro Patacchiola WIFi prisma 27/04/2020 p
				 */

			}	
			//else
			//{
				//; //status_ack = true; 
			//}
		}

		vTaskDelay(100);   		//ripetizione task ogni 100ms
		
	}
	vTaskDelete(NULL);
}


void task_lampeggio_led(void *pvParameters)
{
	while (1)
	{
		//lampeggio led traffico bluetooth
/*
 *START Alvaro Patacchiola WIFi prisma 23/04/2020 p
*/
#ifndef debug		
		if(led_ble_on == true)
		{
			gpio_set_level(LED_BLE, 1);
			led_ble_on = false;
		}
		else
		{
			gpio_set_level(LED_BLE, 0);
		}
#endif		
/*
*END Alvaro Patacchiola WIFi prisma 23/04/2020 p
*/
		
		//lampeggio led traffico uart
		if(led_uart_on == true)
		{
			gpio_set_level(LED_UART, 1);
			led_uart_on = false;
		}
		else
		{
			gpio_set_level(LED_UART, 0);
		}

		vTaskDelay(1);   		//ripetizione task ogni 1ms
	}
	vTaskDelete(NULL);
}



void task_scrittura_uart(void *pvParameters)
{
	uint16_t tick = 0;
	
	while (1)
	{
		tick += 1;
		if (tick >= 40)
		{																																																					

			if (app_uart_write_req == false)
			{
				/*
				*START Alvaro Patacchiola WIFi prisma 27/04/2020 p
				*/
				printf("invio rs 485 %d\n", uart_write_req_id);
				/*
				 *END Alvaro Patacchiola WIFi prisma 27/04/2020 p
				*/

				if (uart_write_req_id == 0)
				{
					;
					
				}	
				else if (uart_write_req_id == 1)
				{
					Invia_stato_pm1wifi(status1, sizeof(status1), status_ack); uart_write_req_id = 0;
				}
				else if (uart_write_req_id == 2)
				{
					Invia_stato_pm1wifi(status2, sizeof(status2), status_ack); uart_write_req_id = 0;
				}		
				else if (uart_write_req_id == 3)
				{
					uart_write_bytes(UART_NUM_2, (char*)status3, sizeof(status3)); led_uart_blink(); uart_write_req_id = 0;
				}		
				else if (uart_write_req_id == 4)
				{
					uart_write_bytes(UART_NUM_2, (char*)status4, sizeof(status4)); led_uart_blink(); uart_write_req_id = 0;
				}		
				else if (uart_write_req_id == 6)
				{
					Invia_stato_pm1wifi(status6, sizeof(status6), status_ack); uart_write_req_id = 0;
				}		
				else if (uart_write_req_id == 7)
				{
					Invia_stato_pm1wifi(status7, sizeof(status7), status_ack); uart_write_req_id = 0;
				}		
				else if (uart_write_req_id == 8)
				{
					Invia_stato_pm1wifi(status8, sizeof(status8), status_ack); uart_write_req_id = 0;
				}		
				else if (uart_write_req_id == 9)
				{
					uart_write_bytes(UART_NUM_2, (char*)status9, sizeof(status9)); led_uart_blink(); uart_write_req_id = 0;
				}
				else
				{
					/*
					 *START Alvaro Patacchiola WIFi prisma 27/04/2020 p
					*/
					if (uart_write_req_id == 0x10)
					{
						ESP_LOGI(TAG, "485 event send Alvaro");
						gpio_set_level(LED_BLE, 1);
						Invia_stato_pm1wifi(uart_wifi_status, sizeof(uart_wifi_status), false); uart_write_req_id = 0;
						vTaskDelay(200);  //
						gpio_set_level(LED_BLE, 0);

					}
					else
						uart_write_req_id = 0;
					/*
					 *STOP Alvaro Patacchiola WIFi prisma 27/04/2020 p
					*/
					
				}
			}
			else 
			{
				/*
				 *START Alvaro Patacchiola WIFi prisma 30/04/2020 p
				*/
#ifdef debug
				ESP_LOGI(TAG, "485 event send Remote APP Alvaro %d,",app_uart_write_len);
				gpio_set_level(LED_BLE, 1);
				uart_write_bytes(UART_NUM_2, (char *)(app_uart_write_value), app_uart_write_len);
				app_uart_write_len = 0;
				app_uart_write_req = false;
				vTaskDelay(200);   //
				gpio_set_level(LED_BLE, 0);
				
#else				
				uart_write_bytes(UART_NUM_2, (char *)(app_uart_write_value), app_uart_write_len);
				led_uart_blink();
				memset(app_uart_write_value, 0x00, sizeof(app_uart_write_value));
				app_uart_write_len = 0;
				app_uart_write_req = false;
#endif				
				/*
				 *START Alvaro Patacchiola WIFi prisma 30/04/2020 p
				*/
				
			}
			tick = 0;
		}
		vTaskDelay(10);     		//ripetizione task ogni 10ms
	}
	vTaskDelete(NULL);
}

/*
void task_sensore_pressione(void *pvParameters)
{
	while (1)
	{
		
		//fai la conversione analogico-digitale
		//if(adc_conv_req) {
			uint16_t res_conv = 0;
			res_conv = exec_ad_conv();
//			printf( "Vanlg %dmV\n", (uint16_t)res_conv );
//			char res_conv_str[6];
//			sprintf(res_conv_str, "%d\r", res_conv);
//			uart_write_req_id = 10;
//			uart_write_bytes(UART_NUM_1, res_conv_str, strlen(res_conv_str));
			//adc_conv_req = false;
		//}
		
		vTaskDelay(1000);      	//ripetizione task ogni 1s
	}
	vTaskDelete(NULL);
}
*/

/*
//aggiunto per test display 8x2

void Display_PulseEnable()
{
	gpio_set_level(GPIO_OUTPUT_E_disp, 1);
	//NOP();
	//NOP();
	//NOP();
	vTaskDelay(1);
	gpio_set_level(GPIO_OUTPUT_E_disp, 0);
}

void Display_InitSeq()
{	
	//1 function set
	gpio_set_level(GPIO_OUTPUT_RS_disp, 0);
	gpio_set_level(GPIO_OUTPUT_DB7_disp, 0);
	gpio_set_level(GPIO_OUTPUT_DB6_disp, 0);
	gpio_set_level(GPIO_OUTPUT_DB5_disp, 1);  //1
	gpio_set_level(GPIO_OUTPUT_DB4_disp, 0);  //DL --> datalength : 0 4-bit, 1 8-bit
	Display_PulseEnable();
	vTaskDelay(1);
	
	//2 function set
	gpio_set_level(GPIO_OUTPUT_RS_disp, 0);
	gpio_set_level(GPIO_OUTPUT_DB7_disp, 0);
	gpio_set_level(GPIO_OUTPUT_DB6_disp, 0);
	gpio_set_level(GPIO_OUTPUT_DB5_disp, 1);  //1
	gpio_set_level(GPIO_OUTPUT_DB4_disp, 0);  //DL --> datalength : 0 4-bit, 1 8-bit
	Display_PulseEnable();
	gpio_set_level(GPIO_OUTPUT_RS_disp, 0);
	gpio_set_level(GPIO_OUTPUT_DB7_disp, 1);  //N
	gpio_set_level(GPIO_OUTPUT_DB6_disp, 0);  //F
	gpio_set_level(GPIO_OUTPUT_DB5_disp, 0);  //x
	gpio_set_level(GPIO_OUTPUT_DB4_disp, 0);  //x
	Display_PulseEnable();
	vTaskDelay(1);
	
	//3 function set
	gpio_set_level(GPIO_OUTPUT_RS_disp, 0);
	gpio_set_level(GPIO_OUTPUT_DB7_disp, 0);
	gpio_set_level(GPIO_OUTPUT_DB6_disp, 0);
	gpio_set_level(GPIO_OUTPUT_DB5_disp, 1);  //1
	gpio_set_level(GPIO_OUTPUT_DB4_disp, 0);  //DL --> datalength : 0 4-bit, 1 8-bit
	Display_PulseEnable();
	gpio_set_level(GPIO_OUTPUT_RS_disp, 0);
	gpio_set_level(GPIO_OUTPUT_DB7_disp, 1);  //N
	gpio_set_level(GPIO_OUTPUT_DB6_disp, 0);  //F
	gpio_set_level(GPIO_OUTPUT_DB5_disp, 0);  //x
	gpio_set_level(GPIO_OUTPUT_DB4_disp, 0);  //x
	Display_PulseEnable();
	vTaskDelay(1);
	
	//4 on/off
	gpio_set_level(GPIO_OUTPUT_RS_disp, 0);
	gpio_set_level(GPIO_OUTPUT_DB7_disp, 0);
	gpio_set_level(GPIO_OUTPUT_DB6_disp, 0);
	gpio_set_level(GPIO_OUTPUT_DB5_disp, 0);
	gpio_set_level(GPIO_OUTPUT_DB4_disp, 0);
	Display_PulseEnable();
	gpio_set_level(GPIO_OUTPUT_RS_disp, 0);
	gpio_set_level(GPIO_OUTPUT_DB7_disp, 1);  //1
	gpio_set_level(GPIO_OUTPUT_DB6_disp, 1);  //D --> display: 0 off, 1 on
	gpio_set_level(GPIO_OUTPUT_DB5_disp, 1);  //C --> cursor: 0 off, 1 on
	gpio_set_level(GPIO_OUTPUT_DB4_disp, 1);  //B --> blink: 0 off, 1 on
	Display_PulseEnable();
	vTaskDelay(1);
	
	//5 clear
	gpio_set_level(GPIO_OUTPUT_RS_disp, 0);
	gpio_set_level(GPIO_OUTPUT_DB7_disp, 0);
	gpio_set_level(GPIO_OUTPUT_DB6_disp, 0);
	gpio_set_level(GPIO_OUTPUT_DB5_disp, 0);
	gpio_set_level(GPIO_OUTPUT_DB4_disp, 0);
	Display_PulseEnable();
	gpio_set_level(GPIO_OUTPUT_RS_disp, 0);
	gpio_set_level(GPIO_OUTPUT_DB7_disp, 0);
	gpio_set_level(GPIO_OUTPUT_DB6_disp, 0);
	gpio_set_level(GPIO_OUTPUT_DB5_disp, 0);
	gpio_set_level(GPIO_OUTPUT_DB4_disp, 1);
	Display_PulseEnable();
	vTaskDelay(3);
	
	//6 mode set
	gpio_set_level(GPIO_OUTPUT_RS_disp, 0);
	gpio_set_level(GPIO_OUTPUT_DB7_disp, 0);
	gpio_set_level(GPIO_OUTPUT_DB6_disp, 0);
	gpio_set_level(GPIO_OUTPUT_DB5_disp, 0);
	gpio_set_level(GPIO_OUTPUT_DB4_disp, 0);
	Display_PulseEnable();
	gpio_set_level(GPIO_OUTPUT_RS_disp, 0);
	gpio_set_level(GPIO_OUTPUT_DB7_disp, 0);
	gpio_set_level(GPIO_OUTPUT_DB6_disp, 1);  //1
	gpio_set_level(GPIO_OUTPUT_DB5_disp, 1);  //I/D --> 0 increment 1 decrement 
	gpio_set_level(GPIO_OUTPUT_DB4_disp, 0);  //S --> display shift: 0 right, 1 left
	Display_PulseEnable();
	vTaskDelay(100);
}

void LCD_SetData(unsigned int cX)
{
	gpio_set_level(GPIO_OUTPUT_DB4_disp, cX & 0x01);
	gpio_set_level(GPIO_OUTPUT_DB5_disp, cX & 0x02);
	gpio_set_level(GPIO_OUTPUT_DB6_disp, cX & 0x04);
	gpio_set_level(GPIO_OUTPUT_DB7_disp, cX & 0x08);	
}

void LCD_SetPosition(unsigned int cX)
{
	LCD_SetData( (cX >> 4) | 0x08 );	//bit alti
	Display_PulseEnable();
	LCD_SetData(cX);					//bit bassi
	Display_PulseEnable();
}

void WriteDataXLCD(char data)
{
	gpio_set_level(GPIO_OUTPUT_RS_disp, 1);
	LCD_SetData(data >> 4);
	Display_PulseEnable();
	LCD_SetData(data);
	Display_PulseEnable();
	gpio_set_level(GPIO_OUTPUT_RS_disp, 0);
}

void PrintfsnXLCD(char *buffer, unsigned int n)
{		
	while (n)						// Write data to LCD up to null
	{
		WriteDataXLCD(*buffer);		// Write character to LCD
		buffer++;					// Increment buffer
		--n;
	}
	return;
}
*/

void Estrai_MAC_address(uint8_t addr5, uint8_t addr4, uint8_t addr3, uint8_t addr2, uint8_t addr1, uint8_t addr0)
{
	uint16_t hex1 = 0x00;
	uint8_t hex2 = 0x00;

	hex1 = addr5 & 0xF0;
	hex1 = hex1 >> 4;
	hex2 = addr5 & 0x0F;
	if ((hex1 >= 0) && (hex1 <= 9)){ hex1 += 48; }
	if ((hex1 >= 10) && (hex1 <= 15)){ hex1 += 55; }
	if ((hex2 >= 0) && (hex2 <= 9)){ hex2 += 48; }
	if ((hex2 >= 10) && (hex2 <= 15)){ hex2 += 55; }
	spp_adv_data[12] = hex1;
	spp_adv_data[13] = hex2;
	gap_complete_local_name[7] = hex1;
	gap_complete_local_name[8] = hex2;
	
	hex1 = addr4 & 0xF0;
	hex1 = hex1 >> 4;
	hex2 = addr4 & 0x0F;
	if ((hex1 >= 0) && (hex1 <= 9)){ hex1 += 48; }
	if ((hex1 >= 10) && (hex1 <= 15)){ hex1 += 55; }
	if ((hex2 >= 0) && (hex2 <= 9)){ hex2 += 48; }
	if ((hex2 >= 10) && (hex2 <= 15)){ hex2 += 55; }
	spp_adv_data[14] = hex1;
	spp_adv_data[15] = hex2;
	gap_complete_local_name[9] = hex1;
	gap_complete_local_name[10] = hex2;
	
	
	hex1 = addr3 & 0xF0;
	hex1 = hex1 >> 4;
	hex2 = addr3 & 0x0F;
	if ((hex1 >= 0) && (hex1 <= 9)){ hex1 += 48; }
	if ((hex1 >= 10) && (hex1 <= 15)){ hex1 += 55; }
	if ((hex2 >= 0) && (hex2 <= 9)){ hex2 += 48; }
	if ((hex2 >= 10) && (hex2 <= 15)){ hex2 += 55; }
	spp_adv_data[16] = hex1;
	spp_adv_data[17] = hex2;
	gap_complete_local_name[11] = hex1;
	gap_complete_local_name[12] = hex2;
	
	hex1 = addr2 & 0xF0;
	hex1 = hex1 >> 4;
	hex2 = addr2 & 0x0F;
	if ((hex1 >= 0) && (hex1 <= 9)){ hex1 += 48; }
	if ((hex1 >= 10) && (hex1 <= 15)){ hex1 += 55; }
	if ((hex2 >= 0) && (hex2 <= 9)){ hex2 += 48; }
	if ((hex2 >= 10) && (hex2 <= 15)){ hex2 += 55; }
	spp_adv_data[18]  = hex1;
	spp_adv_data[19] = hex2;
	gap_complete_local_name[13] = hex1;
	gap_complete_local_name[14] = hex2;
	
	hex1 = addr1 & 0xF0;
	hex1 = hex1 >> 4;
	hex2 = addr1 & 0x0F;
	if ((hex1 >= 0) && (hex1 <= 9)){ hex1 += 48; }
	if ((hex1 >= 10) && (hex1 <= 15)){ hex1 += 55; }
	if ((hex2 >= 0) && (hex2 <= 9)){ hex2 += 48; }
	if ((hex2 >= 10) && (hex2 <= 15)){ hex2 += 55; }
	spp_adv_data[20] = hex1;
	spp_adv_data[21] = hex2;
	gap_complete_local_name[15] = hex1;
	gap_complete_local_name[16] = hex2;
	
	hex1 = addr0 & 0xF0;
	hex1 = hex1 >> 4;
	hex2 = addr0 & 0x0F;
	if ((hex1 >= 0) && (hex1 <= 9)){ hex1 += 48; }
	if ((hex1 >= 10) && (hex1 <= 15)){ hex1 += 55; }
	if ((hex2 >= 0) && (hex2 <= 9)){ hex2 += 48; }
	if ((hex2 >= 10) && (hex2 <= 15)){ hex2 += 55; }
	spp_adv_data[22] = hex1;
	spp_adv_data[23] = hex2;
	gap_complete_local_name[17] = hex1;
	gap_complete_local_name[18] = hex2;
	
}
/*
 *START Alvaro Patacchiola WIFi prisma 23/04/2020 p
*/

static esp_err_t mqtt_event_handler(esp_mqtt_event_handle_t event)
{
	esp_mqtt_client_handle_t client = event->client;
	int msg_id;
	const char* found;
	// your_context_t *context = event->context;
	/*
	 *Per operare in questo modo, l’MQTT segue un paradigma di pubblicazione e sottoscrizione classico, definito “publish and subscribe”, ossia asincrono. Semplificando al massimo la questione: quando il nodo A vuole comunicare con il nodo B, non lo fa in modo sincrono, ovvero come se fosse una telefonata a cui occorre rispondere immediatamente. Al contrario, nel protocollo MQTT il messaggio viene pubblicato dal nodo A (publish) e viene ricevuto dai nodi che sottoscrivono la ricezione del messaggio stesso (subscribe). Sostanzialmente, dunque, con MQTT si disaccoppia fortemente la produzione dalla ricezione del messaggio stesso, anche da un punto di vista temporale*/
	
    switch(event->event_id) {
	case MQTT_EVENT_CONNECTED:
		ESP_LOGI(TAG, "MQTT_EVENT_CONNECTED");
		msg_id = esp_mqtt_client_subscribe(client, topicNameW, 0); //contiene i messaggi provenienti dall app da inviare alla pompa
		ESP_LOGI(TAG, "sent subscribe successful, msg_id=%d", msg_id);
		//con il subscribe il client aderisce alla ricezione dei messaggi con etichetta "/topic/qos0"
		// qualunque client che pubblica con etichetta "/topic/qos0", i clienet che hanno subscribe "/topic/qos0" ricevono il messaggio
			/*msg_id = esp_mqtt_client_subscribe(client, "/topic/qos1", 1);//prova per verificare l'evento MQTT_EVENT_UNSUBSCRIBED
			ESP_LOGI(TAG, "sent subscribe successful, msg_id=%d", msg_id);

			msg_id = esp_mqtt_client_unsubscribe(client, "/topic/qos1");
			ESP_LOGI(TAG, "sent unsubscribe successful, msg_id=%d", msg_id);*/
			statusConnectionMQTT = mqttConnected;
			//msg_id = esp_mqtt_client_subscribe(client, topicNameR, 1); //contiene i messaggi provenienti dalla pompa da restituire all app
		break;
	case MQTT_EVENT_DISCONNECTED:
	    statusConnectionMQTT = mqttDisconnected;
		ESP_LOGI(TAG, "MQTT_EVENT_DISCONNECTED");
		break;

	case MQTT_EVENT_SUBSCRIBED:
		//quando sono
			ESP_LOGI(TAG, "MQTT_EVENT_SUBSCRIBED, msg_id=%d", event->msg_id);
	    /*found = strstr(event->topic, topicNameMail);
	    if (found != NULL)// se messaggio mail lo invio al server
	    {
		    
	    }
	    */
		//msg_id = esp_mqtt_client_publish(client, topicNameR, "connected", 0, 0, 0);
		//metodo publish:primoParametro:identificativo, (Topic Name ) stringa che identifica il messaggio da inviare, "data" è proprio il contenuto del messaggio da inviare
	    
			//ESP_LOGI(TAG, "sent publish successful, msg_id=%d", msg_id);
		break;
	case MQTT_EVENT_UNSUBSCRIBED:
		ESP_LOGI(TAG, "MQTT_EVENT_UNSUBSCRIBED, msg_id=%d", event->msg_id);
		break;
	case MQTT_EVENT_PUBLISHED:
		ESP_LOGI(TAG, "MQTT_EVENT_PUBLISHED, msg_id=%d", event->msg_id);
		break;
	case MQTT_EVENT_DATA:
		ESP_LOGI(TAG, "MQTT_EVENT_DATA");
		//esempio
		
		found = strstr(event->topic, topicNameW);
	    if (found != NULL)
	    {
		    app_uart_write_len = event->data_len;
		    memcpy(app_uart_write_value, (char *)(event->data), event->data_len);
		    for (int i = 0; i < event->data_len; i++)
		    {
			    printf("%x\t", app_uart_write_value[i]);
		    }
		    printf("\n");
		    
		    app_uart_write_req = true;

	    }
			//msg_id = esp_mqtt_client_publish(client, topicNameR, "risposta", 0, 0, 0);
		printf("TOPIC=%.*s\r\n", event->topic_len, event->topic);
		printf("DATA=%.*s\r\n", event->data_len, event->data);
		break;
	case MQTT_EVENT_ERROR:
		ESP_LOGI(TAG, "MQTT_EVENT_ERROR");
		break;
	}
	return ESP_OK;
}
static esp_err_t wifi_event_handler(void *ctx, system_event_t *event)
{
	switch (event->event_id) {
	case SYSTEM_EVENT_STA_START:
		esp_wifi_connect();
		break;
	case SYSTEM_EVENT_STA_GOT_IP:
		ESP_LOGI(TAG,
			"got ip:%s",
			ip4addr_ntoa(&event->event_info.got_ip.ip_info.ip));
		xEventGroupSetBits(wifi_event_group, WIFI_CONNECTED_BIT);
		
		esp_mqtt_client_start(client);
		
		break;
	case SYSTEM_EVENT_AP_STACONNECTED:
		ESP_LOGI(TAG,
			"station:"MACSTR" join, AID=%d",
			MAC2STR(event->event_info.sta_connected.mac),
			event->event_info.sta_connected.aid);
		break;
	case SYSTEM_EVENT_AP_STADISCONNECTED:
		ESP_LOGI(TAG,
			"station:"MACSTR"leave, AID=%d",
			MAC2STR(event->event_info.sta_disconnected.mac),
			event->event_info.sta_disconnected.aid);
			esp_mqtt_client_stop(client);
		break;
	case SYSTEM_EVENT_STA_DISCONNECTED:
		esp_wifi_connect();
		xEventGroupClearBits(wifi_event_group, WIFI_CONNECTED_BIT);
		break;
	default:
		break;
	}
	return ESP_OK;
}
void  wifi_init(bool intiStatus)
{
	tcpip_adapter_init();
	wifi_event_group = xEventGroupCreate();
	if (intiStatus)//da chiamare sono all'avvio dell esp32
		ESP_ERROR_CHECK(esp_event_loop_init(wifi_event_handler, NULL));
	wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
	ESP_ERROR_CHECK(esp_wifi_init(&cfg));
	ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
	
}

static void wifi_connect(void)
{
	
	setInitNetwork(); //inizializzo le modalita di lavoro della rete, static o dhcp
/*
	sprintf((char *)wifi_config.sta.ssid, "ASIA");
	sprintf((char *)wifi_config.sta.password, "same2600");
*/	
	ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
	
	//ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config));
	
	ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config));
	ESP_LOGI(TAG, "start the WIFI SSID:[%s][%s]", wifi_config.sta.ssid, wifi_config.sta.password);
	ESP_ERROR_CHECK(esp_wifi_start());
	ESP_LOGI(TAG, "Waiting for wifi");
	//xEventGroupWaitBits(wifi_event_group, CONNECTED_BIT, false, true, portMAX_DELAY);//devio torgliere altrimenti si blocca ogni volta aspetta la connessione di rete
}
void getIPNetwork(uint8_t * lunghezzTemp)
{
	tcpip_adapter_ip_info_t ip_info;
	tcpip_adapter_dns_info_t dns_info;
	tcpip_adapter_dhcp_status_t dhcp_info;
	
	ESP_ERROR_CHECK(tcpip_adapter_get_ip_info(TCPIP_ADAPTER_IF_STA, &ip_info));
	tcpip_adapter_get_dns_info(TCPIP_ADAPTER_IF_STA, TCPIP_ADAPTER_DNS_MAIN, &dns_info);
	tcpip_adapter_dhcpc_get_status(TCPIP_ADAPTER_IF_STA, &dhcp_info);

	printf("IP Address:  %s\n", ip4addr_ntoa(&ip_info.ip));
	printf("Subnet mask: %s\n", ip4addr_ntoa(&ip_info.netmask));
	printf("Gateway:     %s\n", ip4addr_ntoa(&ip_info.gw));
	printf("DNS:     %s\n", ip4addr_ntoa(&dns_info.ip));
	printf("ip:     %X\n", (ip_info.ip.addr));
	/*
 formato risposta alla scansione rete
 0xff 0x41 0x00 0xfe mode 0x00 rss1 auth Ap1 0x00 rss1 auth .. .. .. .. 0xFF XX
 *ff 40 00 fe 00 48 03 41 53 49 41 00 4e 03 6f 73 70 69 74 65 5f 61 73 69 61 00 53 03 ff a3 // ESEMPIO RISPOSTA
 **/

	if (dhcp_info == TCPIP_ADAPTER_DHCP_STARTED)// sta in modalida dhcp
	{
		uart_wifi_status[*lunghezzTemp] = 0;
	}
	else// sta in modalida static 
	{
		uart_wifi_status[*lunghezzTemp] = 1;
	}
	*lunghezzTemp = *lunghezzTemp + 1;
	put32(ip_info.ip.addr, &*lunghezzTemp);
	put32(ip_info.netmask.addr, & *lunghezzTemp);
	put32(ip_info.gw.addr, & *lunghezzTemp);
	put32(dns_info.ip.u_addr.ip4.addr, & *lunghezzTemp);
}
void setSerialNumber(uint8_t * arrayTemp)
{
#ifdef debug
	uint8_t i = 0;   //posizione primo carattere della stringa wifi
#else	
	uint8_t i = 4;  //posizione primo carattere della stringa wifi
#endif	
	/*
	//COMANDO che legge il serial number dalla pompa
	0X46
	FF 46 00 FE 00 FF B8 // comando lettura serial number dello strumento
	FF 46 00 FE 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 FF B8 //serial number cifra per cifra in decimale
	*/
    for(int j = 0 ; j < 17 ; j++) {
	    serialNumber[j] = arrayTemp[i + j];
	}
	//impostazione dei topic
		sprintf(topicNameW, "/%s/appW", serialNumber);
		sprintf(topicNameR, "/%s/appR", serialNumber);
}
void setIPNetwork(uint8_t * arrayTemp)
{
	
	uint8_t i = 4; //posizione primo carattere della stringa wifi
	uint8_t j = 0;
	
	/*
		//COMANDO CHE per impostare i parametri di rete IP, modalita static o DHCP, ip, subnet mask, gateway e dns
		0X45
		FF 45 00 FE Type Ip4 Ip3 Ip2 Ip1 SNM4 SNM3 SNM2 SNM1 GTW4 GTW3 GTW2 GTW1 DNS4 DNS3 DNS2 DNS1 FF XX // type:0 DHCP    mode 1: static mode  IP: indirizzo ip SNM: subnet mask GTW: Gateway DNS: dns
		FF 45 00 FE 01 c0 a8 01 55 ff ff ff 00 c0 a8 01 fb 08 08 08 08 ff eb // IMPOSTAZIONE DI STATIC, CON ip 192.168.1.85 SubNet: 255.255.255.0 Gateway: 192.168.1.251 dns: 8.8.8.8
		FF 45 00 FE 00 ff bb // IMPOSTAZIONE Dhcp, NON SONO NECESSARI GLI INDIRIZZI ip
	*/
	ip.mode = arrayTemp[i];
	i++;
	
	if (ip.mode == 1)// impostazione dello staic mode
	{
		tcpip_adapter_ip_info_t ipInfo;
		tcpip_adapter_dns_info_t dns_info;

		tcpip_adapter_dhcpc_stop(TCPIP_ADAPTER_IF_STA);  // blocco il dhcp
		ip.ip = get32(arrayTemp, &i);
		ipInfo.ip.addr = ip.ip;
		ip.subnet = get32(arrayTemp, &i);
		ipInfo.netmask.addr = ip.subnet;
		ip.gateway = get32(arrayTemp, &i);
		ipInfo.gw.addr = ip.gateway;
		ip.dns = get32(arrayTemp, &i);
		dns_info.ip.u_addr.ip4.addr = ip.dns;
		
		printf("IP Address:  %s\n", ip4addr_ntoa(&ipInfo.ip));
		printf("Subnet mask: %s\n", ip4addr_ntoa(&ipInfo.netmask));
		printf("Gateway:     %s\n", ip4addr_ntoa(&ipInfo.gw));
		printf("DNS:     %s\n", ip4addr_ntoa(&dns_info.ip));

		tcpip_adapter_set_ip_info(TCPIP_ADAPTER_IF_STA, &ipInfo);
		tcpip_adapter_set_dns_info(TCPIP_ADAPTER_IF_STA, TCPIP_ADAPTER_DNS_MAIN, &dns_info);
	}
	else
	{// impostazione dhcp mode
		tcpip_adapter_dhcpc_start(TCPIP_ADAPTER_IF_STA);  // Don't run a DHCP client
	}
	
	
	writeEEpromIP();
	ESP_ERROR_CHECK(esp_wifi_disconnect());
	esp_wifi_stop();
	esp_wifi_deinit();
	wifi_init(false);
	wifi_connect();
	

}
uint32_t get32(uint8_t * dataIn, uint8_t * lunghezzTemp)
{	
	bit32_ ipNum;
	ipNum.BIT.ad_low = dataIn[*lunghezzTemp];
	*lunghezzTemp = *lunghezzTemp + 1;
	ipNum.BIT.ad_low1 = dataIn[*lunghezzTemp];
	*lunghezzTemp = *lunghezzTemp + 1;
	ipNum.BIT.ad_high = dataIn[*lunghezzTemp];
	*lunghezzTemp = *lunghezzTemp + 1;
	ipNum.BIT.ad_high1 = dataIn[*lunghezzTemp];
	*lunghezzTemp = *lunghezzTemp + 1;

	return ipNum.value ;
}
void put32(uint32_t dataIn, uint8_t * lunghezzTemp)
{
	bit32_ ipNum;
	ipNum.value = dataIn;
	uart_wifi_status[*lunghezzTemp] = ipNum.BIT.ad_low;
	*lunghezzTemp = *lunghezzTemp + 1;
	uart_wifi_status[*lunghezzTemp] = ipNum.BIT.ad_low1;
	*lunghezzTemp = *lunghezzTemp + 1;
	uart_wifi_status[*lunghezzTemp] = ipNum.BIT.ad_high;
	*lunghezzTemp = *lunghezzTemp + 1;
	uart_wifi_status[*lunghezzTemp] = ipNum.BIT.ad_high1;
	*lunghezzTemp = *lunghezzTemp + 1;

}
void readEEpromData()
{
	//lettura dati eeprom
	esp_err_t err;
	printf("Opening Non-Volatile Storage (NVS) handle... ");
	nvs_handle my_handle;
	err = nvs_open("storage", NVS_READONLY, &my_handle);

	if (err != ESP_OK) {
		printf("Error (%d) opening NVS handle!\n", err);
	}
	else {
		printf("Done\n");
		
		size_t size = sizeof(wifi_config.sta.ssid);
		err = nvs_get_blob(my_handle, "dst_ssid_addr", wifi_config.sta.ssid, &size);
		size = sizeof(wifi_config.sta.password);
		nvs_get_blob(my_handle, "dst_pass_addr", wifi_config.sta.password, &size);
		size = sizeof(ip);
		nvs_get_blob(my_handle, "dst_ser_ip", &ip, &size);


	}
	// Close
	nvs_close(my_handle);
}
void writeEEpromIP()
{
	//lettura dati eeprom
	esp_err_t err;
	printf("Opening Non-Volatile Storage (NVS) handle... ");
	nvs_handle my_handle;
	err = nvs_open("storage", NVS_READWRITE, &my_handle);
	//nvs_erase_all(&my_handle);
	if(err != ESP_OK) {
		printf("Error (%d) opening NVS handle!\n", err);
	} else {
		printf("Done\n");
		size_t size = sizeof(ip);
		nvs_set_blob(my_handle, "dst_ser_ip", &ip, size);
	}

	printf("Committing updates in NVS ... ");
	err = nvs_commit(my_handle);
	printf((err != ESP_OK) ? "Failed!\n" : "Done\n");
	// Close
	nvs_close(my_handle);
}
void writeEEpromData()
{
	//lettura dati eeprom

	esp_err_t err;
	nvs_handle my_handle;
	printf("Opening Non-Volatile Storage (NVS) handle... ");

	err = nvs_open("storage", NVS_READWRITE, &my_handle);
	//nvs_erase_all(&my_handle);
	if(err != ESP_OK) {
		printf("Error (%d) opening NVS handle!\n", err);
	} else {
		printf("Done\n");
		size_t size = sizeof(wifi_config.sta.ssid);
		nvs_set_blob(my_handle, "dst_ssid_addr", wifi_config.sta.ssid, size);

		printf("Committing updates in NVS ... ");
		err = nvs_commit(my_handle);
		printf((err != ESP_OK) ? "Failed!\n" : "Done\n");
		nvs_close(my_handle);

		nvs_open("storage", NVS_READWRITE, &my_handle);
		size_t size1 = sizeof(wifi_config.sta.password);
		nvs_set_blob(my_handle, "dst_pass_addr", wifi_config.sta.password, size1);


	}

	printf("Committing updates in NVS ... ");
	err = nvs_commit(my_handle);
	printf((err != ESP_OK) ? "Failed!\n" : "Done\n");


	// Close
	nvs_close(my_handle);

}
char * getMACWiFi()
{
	uint8_t MAC_addr[6];
	
	
	esp_wifi_get_mac(WIFI_IF_STA, MAC_addr);
	
	sprintf((char *)macAddress, "%2X.%2X.%2X.%2X.%2X.%2X", MAC_addr[0], MAC_addr[1], MAC_addr[2], MAC_addr[3], MAC_addr[4], MAC_addr[5]);
	return macAddress;
	
}	
void scanNetworkWiFi(uint8_t * lunghezzTemp)
{
	wifi_scan_config_t scan_config = {
		.ssid = 0,
		.bssid = 0,
		.channel = 0,
		.show_hidden = true
	};
	uint16_t ap_num = MAX_APs;
	wifi_ap_record_t ap_records[MAX_APs];
	const char* found;
	printf("Start scanning...");
	ESP_ERROR_CHECK(esp_wifi_scan_start(&scan_config, true));
	ESP_ERROR_CHECK(esp_wifi_scan_get_ap_records(&ap_num, ap_records));
	printf("Found %d access points:\n", ap_num);
	/*
	 *formato risposta alla scansione rete
	 *0xff 0x40 0x00 0xfe Ap 0x00 rss1 auth Ap1 0x00 rss1 auth .. .. .. .. 0xFF XX
	 *ff 40 00 fe 00 48 03 41 53 49 41 00 4e 03 6f 73 70 69 74 65 5f 61 73 69 61 00 53 03 ff a3 // ESEMPIO RISPOSTA
	 **/
	
	for (int i = 0; i < ap_num; i++) {
		for (int j = 0; j < strlen((char *)ap_records[i].ssid); j++) {
			uart_wifi_status[*lunghezzTemp] = ap_records[i].ssid[j];
			*lunghezzTemp = *lunghezzTemp + 1;
			//printf("costruzione stringa  %d \n", *lunghezzTemp);
		}
		uart_wifi_status[*lunghezzTemp] = 0; // zero di suddivisione tra la stringa del AP
		*lunghezzTemp = *lunghezzTemp + 1;
		uart_wifi_status[*lunghezzTemp] = abs(ap_records[i].rssi);   // valore assoluto del livello del segale tolgo il meno
		*lunghezzTemp = *lunghezzTemp + 1;
		uart_wifi_status[*lunghezzTemp] = ap_records[i].authmode;     // tipo di autenticazione della rete 
		*lunghezzTemp = *lunghezzTemp + 1;		 /**0 - < authenticate mode : open */		 /**1 - < authenticate mode : WEP */		 /** 2 -< authenticate mode : WPA_PSK */		 /** 3 - < authenticate mode : WPA2_PSK */		 /** 4 - < authenticate mode : WPA_WPA2_PSK */		 /**5 - < authenticate mode : WPA2_ENTERPRISE */		 /* 6 */
	}
	
}
void setInitNetwork()
{
	ESP_LOGI(TAG, "EEPROM START\n");

	if (ip.mode == 0) {// DHCP MODE
		ESP_LOGI(TAG, "system dynamic");
		tcpip_adapter_dhcpc_start(TCPIP_ADAPTER_IF_STA);   
	}
	else {
		//mode static
		ESP_LOGI(TAG, "system static:%d\n", ip.ip);
		tcpip_adapter_dhcpc_stop(TCPIP_ADAPTER_IF_STA);  
		tcpip_adapter_ip_info_t ipInfo;
		tcpip_adapter_dns_info_t dns_info;
		ipInfo.ip.addr = ip.ip;
		ipInfo.netmask.addr = ip.subnet;
		ipInfo.gw.addr = ip.gateway;
		tcpip_adapter_set_ip_info(TCPIP_ADAPTER_IF_STA, &ipInfo);

		dns_info.ip.u_addr.ip4.addr = ip.dns;
		//dns
		tcpip_adapter_set_dns_info(TCPIP_ADAPTER_IF_STA, TCPIP_ADAPTER_DNS_MAIN, &dns_info);

	}

}
void getConnectionInfo(uint8_t * lunghezzTemp)
{
	wifi_ap_record_t info;
	uart_wifi_status[*lunghezzTemp] = 0;
	//FF 44 00 FE 00 FF BA // COMANDO DI RICHIESTA
		
	ESP_ERROR_CHECK(esp_wifi_sta_get_ap_info(&info));
	
	if (!esp_wifi_sta_get_ap_info(&info)) {
		uart_wifi_status[*lunghezzTemp] = abs(info.rssi); // potenza in db positivi del segnale WiFi
		*lunghezzTemp = *lunghezzTemp + 1;
	}
	uart_wifi_status[*lunghezzTemp] = statusConnectionMQTT;
	*lunghezzTemp = *lunghezzTemp + 1;
	
}
void getSSID(uint8_t * lunghezzTemp)
{
	//FF 43 00 FE 00 FF BD FORMATO RICHIESTA ATTUALE SSID
	//ff 43 00 fe 41 53 49 41 00 ff a7 esempio di risposta con il WiFi dui casa ASIA
	for (int j = 0; j < strlen((char *)wifi_config.sta.ssid); j++) {
		uart_wifi_status[*lunghezzTemp] = wifi_config.sta.ssid[j];
		*lunghezzTemp = *lunghezzTemp + 1;
	}
	uart_wifi_status[*lunghezzTemp] = 0;  // zero di suddivisione tra la stringa del AP
	*lunghezzTemp = *lunghezzTemp + 1;
}	
void setSSID(uint8_t * arrayTemp)
{
	
	uint8_t i = 4;//posizione primo carattere della stringa wifi
	uint8_t j = 0;
	 
	memset(wifi_config.sta.ssid, 0, sizeof(wifi_config.sta.ssid));
	memset(wifi_config.sta.password, 0, sizeof(wifi_config.sta.password));
	/*
	0X42
	FF 42 00 FE SSID 00 PWD 00 FF XX // ssid: stringa alfanumerica contenete SSID ap, PWD: eventuale password se la rete lo necessita
	FF 42 00 FE 41 53 49 41 00 73 61 6d 65 32 36 30 30 00 FF B8  //ESEMPIO IMPOSTAZIONE SSID:ASIA PWD:same2600
	*/
	while ((arrayTemp[i] != 0)&&(j < 32)) {
		wifi_config.sta.ssid[j] = arrayTemp[i];
		i++;j++;
	}
	i++;
	j = 0;
	while ((arrayTemp[i] != 0)&&(j < 64)) {
		wifi_config.sta.password[j] = arrayTemp[i];
		i++; j++;
	}
	printf("%s|%s|\n", (char *)wifi_config.sta.ssid, (char *)wifi_config.sta.password);
	
	writeEEpromData();
	ESP_ERROR_CHECK(esp_wifi_disconnect());
	esp_wifi_stop();
	esp_wifi_deinit();
	wifi_init(false);
	wifi_connect();

}
static void mqtt_app_init(void)
{
	const esp_mqtt_client_config_t mqtt_cfg = {
		//.uri = "mqtts://iot.eclipse.org:8883",
		 .uri = "mqtt://www.ermes-server.com:1883",
		//.uri = "mqtts://www.ermes-server.com:8883",
		    //.uri = "mqtts://95.110.157.172:8883",
	    .keepalive = 10,
		 //10 secondi il keep alive, verificare
	    .username = "emecsrl",
		.password = "emecsrl",
		.client_id = getMACWiFi(),
		  //serialnumber dispositivo
		.event_handle = mqtt_event_handler,
		//.cert_pem = (const char *)iot_mosquitto_org_pem_start,
	};

	//sprintf((char*)mqtt_cfg.client_id, "%s", macAddress);
	
	ESP_LOGI(TAG, "[APP] Free memory: %d bytes", esp_get_free_heap_size());
	client = esp_mqtt_client_init(&mqtt_cfg);
	printf("MQTT UID ... %s\n", mqtt_cfg.client_id);
	//esp_mqtt_client_start(client);
}
/*
 *END Alvaro Patacchiola WIFi prisma 23/04/2020 p
*/

/*-------------------------------------------------Main Program--------------------------------------------------------------*/
void app_main()
{
	//printf("Inizio main program");
	/*
	//aggiunto per test display 8x2
	
	//printf("Inizializzo linee display ...\n");
	gpio_set_direction(GPIO_OUTPUT_RS_disp, GPIO_MODE_OUTPUT);
	gpio_set_level(GPIO_OUTPUT_RS_disp, 0);
	gpio_set_direction(GPIO_OUTPUT_E_disp, GPIO_MODE_OUTPUT);
	gpio_set_level(GPIO_OUTPUT_E_disp, 0);
	gpio_set_direction(GPIO_OUTPUT_DB4_disp, GPIO_MODE_OUTPUT);
	gpio_set_level(GPIO_OUTPUT_DB4_disp, 0);
	gpio_set_direction(GPIO_OUTPUT_DB5_disp, GPIO_MODE_OUTPUT);
	gpio_set_level(GPIO_OUTPUT_DB5_disp, 0);
	gpio_set_direction(GPIO_OUTPUT_DB6_disp, GPIO_MODE_OUTPUT);
	gpio_set_level(GPIO_OUTPUT_DB6_disp, 0);
	gpio_set_direction(GPIO_OUTPUT_DB7_disp, GPIO_MODE_OUTPUT);
	gpio_set_level(GPIO_OUTPUT_DB7_disp, 0);
	
	Display_InitSeq();
		
	sprintf(BuffDisp.Line0, "%s", "  VMS   ");
	sprintf(BuffDisp.Line1, "%s", "Ver 1.8 ");
	
	LCD_SetPosition(0x00);															//inizio prima riga
	PrintfsnXLCD(BuffDisp.Line0, sizeof(BuffDisp.Line0));
	LCD_SetPosition(0x40);															//inizio seconda riga
	PrintfsnXLCD(BuffDisp.Line1, sizeof(BuffDisp.Line1));
	*/
	
	printf("Inizializzo I/O ...\n");
	gpio_set_direction(LED_UART, GPIO_MODE_OUTPUT); 									//configuro GPIO led uart
	gpio_set_level(LED_UART, 0); 													//forzo a 0
	gpio_set_direction(LED_BLE, GPIO_MODE_OUTPUT); 									//configuro GPIO led ble
	gpio_set_level(LED_BLE, 0); 														//forzo a 0

	gpio_reset_pin(ECHO_TEST_TXD);
	gpio_set_direction(ECHO_TEST_TXD, GPIO_MODE_OUTPUT); 
	gpio_set_level(ECHO_TEST_TXD, 0);
	gpio_set_direction(ECHO_TEST_TXD, GPIO_MODE_OUTPUT);
	gpio_set_pull_mode(ECHO_TEST_TXD, GPIO_PULLUP_ONLY);
	gpio_pullup_en(ECHO_TEST_TXD);
	
	gpio_reset_pin(ECHO_TEST_RXD);
	//gpio_set_direction(ECHO_TEST_RXD, GPIO_MODE_OUTPUT); 
	//gpio_set_level(ECHO_TEST_RXD, 0);
	gpio_set_direction(ECHO_TEST_RXD, GPIO_MODE_INPUT);
	gpio_set_pull_mode(ECHO_TEST_RXD, GPIO_FLOATING);
	//gpio_pullup_en(ECHO_TEST_RXD);

	gpio_reset_pin(IN_SEFL); 														//inizializzo GPIO15 SEnsore FLusso
	gpio_set_direction(IN_SEFL, GPIO_MODE_INPUT); 									//configuro GPIO15 SEnsore FLusso
	gpio_set_pull_mode(IN_SEFL, GPIO_FLOATING); 										//no pull-up o pull-down
	gpio_set_intr_type(IN_SEFL, GPIO_INTR_NEGEDGE);   								//configuro interrupt su transizione H --> L

	gpio_set_direction(PWM_OUT, GPIO_MODE_OUTPUT); 									//configuro uscita pwm
	gpio_set_level(PWM_OUT, 0); 														//forzo a 0

	//printf("Inizializzo UART_NUM_2 ...\n");
	spp_uart_init(); 																//inizializzo UART2

	//vTaskDelay(100);
	//printf("Inizializzo I/O -> OK\n");
	
	esp_err_t ret;
	esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
	
	//printf("Inizializzo NVS ...\n");
    ret = nvs_flash_init();
	if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
		ESP_ERROR_CHECK(nvs_flash_erase());
		ESP_ERROR_CHECK(nvs_flash_init());
	}
	//ESP_ERROR_CHECK( ret );
	
	//printf("Inizializzo NVS -> OK\n");
	
	//printf("Inizializzo Bluetooth ...\n");
	ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));
    
	ret = esp_bt_controller_init(&bt_cfg);
	if (ret) {
		ESP_LOGE(GATTS_TABLE_TAG, "%s init controller failed: %s", __func__, esp_err_to_name(ret));
		bt_fault = true;
		return;
	}
	ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
	if (ret) {
		ESP_LOGE(GATTS_TABLE_TAG, "%s enable controller failed: %s", __func__, esp_err_to_name(ret));
		bt_fault = true;
		return;
	}
	ESP_LOGI(GATTS_TABLE_TAG, "%s init bluetooth", __func__);
	ret = esp_bluedroid_init();
	if (ret) {
		ESP_LOGE(GATTS_TABLE_TAG, "%s init bluetooth failed: %s", __func__, esp_err_to_name(ret));
		bt_fault = true;
		return;
	}
	ret = esp_bluedroid_enable();
	if (ret) {
		ESP_LOGE(GATTS_TABLE_TAG, "%s enable bluetooth failed: %s", __func__, esp_err_to_name(ret));
		bt_fault = true;
		return;
	}
	
	//imposto la potenza di trasmissione al minimo valore ammesso
	esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_DEFAULT, ESP_PWR_LVL_N12);
	esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_CONN_HDL0, ESP_PWR_LVL_N12);
	esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_CONN_HDL1, ESP_PWR_LVL_N12);
	esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_CONN_HDL2, ESP_PWR_LVL_N12);
	esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_CONN_HDL3, ESP_PWR_LVL_N12);
	esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_CONN_HDL4, ESP_PWR_LVL_N12);
	esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_CONN_HDL5, ESP_PWR_LVL_N12);
	esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_CONN_HDL6, ESP_PWR_LVL_N12);
	esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_CONN_HDL8, ESP_PWR_LVL_N12);
	esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_ADV, ESP_PWR_LVL_N12);

	
	//printf("Inizializzo Bluetooth -> ottengo MAC ADDRESS\n");
	esp_efuse_mac_get_default(bt_mac_address);
	
	//inserisco il mac address nel campo nome del pacchetto advertising spp_adv_data --> trasformo i 6 byte in 12 byte ciascuno relativo al singolo carattere
	Estrai_MAC_address(bt_mac_address[0], bt_mac_address[1], bt_mac_address[2], bt_mac_address[3], bt_mac_address[4], bt_mac_address[5] + 0x02);
	
	
/*
 *START Alvaro Patacchiola WIFi prisma 23/04/2020 p
*/
	statusConnectionMQTT = mqttDisconnected;
	readEEpromData();//leggo i paramtri della rete wifi impostati nella eeprom
	getMACWiFi();
	//setInitNetwork();//inizializzo le modalita di lavoro della rete, static o dhcp
	
	wifi_init(true);
	
	setSerialNumber(serialNumber);//eliminare questa chiamata quando si sostano le 2 linee di codice successive
	wifi_connect();//queste 2 linee di codice vanno spostate nella procedura di lettura del serial number, dopo aver letto il serial number mi connetto altrimenti non ha senso
	mqtt_app_init();
/*
 *END Alvaro Patacchiola WIFi prisma 23/04/2020 p
*/
	
	//inserisco mac-address nel pacchetto di advertising
//	spp_adv_data[27] = bt_mac_address[0];  			//OUI 1st byte
//	spp_adv_data[26] = bt_mac_address[1];  			//OUI 2nd byte
//	spp_adv_data[25] = bt_mac_address[2];  			//OUI 3rd byte
//	spp_adv_data[24] = bt_mac_address[3];  			//NIC 4th byte
//	spp_adv_data[23] = bt_mac_address[4];  			//NIC 5th byte
//	spp_adv_data[22] = bt_mac_address[5] + 0x02;  	//NIC 6th byte

	//inserisco mac-address nei messaggi di stato
	status1[12] = bt_mac_address[0]; 				//OUI 1st byte
	status1[13] = bt_mac_address[1]; 				//OUI 2nd byte
	status1[14] = bt_mac_address[2]; 				//OUI 3rd byte
	status1[15] = bt_mac_address[3]; 				//NIC 4th byte
	status1[16] = bt_mac_address[4]; 				//NIC 5th byte
	status1[17] = bt_mac_address[5] + 0x02; 			//NIC 6th byte
	for(int i = 0 ; i <= (sizeof(status1) - 2) ; i++) { status1[(sizeof(status1) - 1)] ^= status1[i]; }
	
	status2[12] = bt_mac_address[0];
	status2[13] = bt_mac_address[1];
	status2[14] = bt_mac_address[2];
	status2[15] = bt_mac_address[3];
	status2[16] = bt_mac_address[4];
	status2[17] = bt_mac_address[5] + 0x02;
	for (int i = 0; i <= (sizeof(status2) - 2); i++) { status2[(sizeof(status2) - 1)] ^= status2[i]; }
	
	status3[12] = bt_mac_address[0];
	status3[13] = bt_mac_address[1];
	status3[14] = bt_mac_address[2];
	status3[15] = bt_mac_address[3];
	status3[16] = bt_mac_address[4];
	status3[17] = bt_mac_address[5] + 0x02;
	for (int i = 0; i <= (sizeof(status3) - 2); i++) { status3[(sizeof(status3) - 1)] ^= status3[i]; }

	status4[12] = bt_mac_address[0];
	status4[13] = bt_mac_address[1];
	status4[14] = bt_mac_address[2];
	status4[15] = bt_mac_address[3];
	status4[16] = bt_mac_address[4];
	status4[17] = bt_mac_address[5] + 0x02;
	for (int i = 0; i <= (sizeof(status4) - 2); i++) { status4[(sizeof(status4) - 1)] ^= status4[i]; }
	
	//	status5[12] = bt_mac_address[0];
	//	status5[13] = bt_mac_address[1];
	//	status5[14] = bt_mac_address[2];
	//	status5[15] = bt_mac_address[3];
	//	status5[16] = bt_mac_address[4];
	//	status5[17] = bt_mac_address[5] + 0x02;
	//	for (int i = 0; i <= (sizeof(status5) - 2); i++) { status5[(sizeof(status5) - 1)] ^= status5[i]; }

		status6[12] = bt_mac_address[0];
	status6[13] = bt_mac_address[1];
	status6[14] = bt_mac_address[2];
	status6[15] = bt_mac_address[3];
	status6[16] = bt_mac_address[4];
	status6[17] = bt_mac_address[5] + 0x02;
	for (int i = 0; i <= (sizeof(status6) - 2); i++) { status6[(sizeof(status6) - 1)] ^= status6[i]; }
	
	status7[12] = bt_mac_address[0];
	status7[13] = bt_mac_address[1];
	status7[14] = bt_mac_address[2];
	status7[15] = bt_mac_address[3];
	status7[16] = bt_mac_address[4];
	status7[17] = bt_mac_address[5] + 0x02;
	for (int i = 0; i <= (sizeof(status7) - 2); i++) { status7[(sizeof(status7) - 1)] ^= status7[i]; }
	
	status8[12] = bt_mac_address[0];
	status8[13] = bt_mac_address[1];
	status8[14] = bt_mac_address[2];
	status8[15] = bt_mac_address[3];
	status8[16] = bt_mac_address[4];
	status8[17] = bt_mac_address[5] + 0x02;
	for (int i = 0; i <= (sizeof(status8) - 2); i++) { status8[(sizeof(status8) - 1)] ^= status8[i]; }
	
	esp_ble_gatts_register_callback(gatts_event_handler);
	esp_ble_gap_register_callback(gap_event_handler);
	esp_ble_gatts_app_register(ESP_SPP_APP_ID);
	//printf("Inizializzo Bluetooth -> OK\n");	
	
	printf("Avvio i tasks ...\n");
	/*
	 *START Alvaro Patacchiola WIFi prisma 27/04/2020 p
	 */
	//xTaskCreate(task_principale, "task principale", 2048, NULL, 8, NULL);
	xTaskCreate(task_principale, "task principale", 4096, NULL, 8, NULL);
	/*
	 *END Alvaro Patacchiola WIFi prisma 27/04/2020 p
	 */

	xTaskCreate(task_lampeggio_led, "task_lampeggio_led", 2048, NULL, 4, NULL);
	xTaskCreate(task_scrittura_uart, "task_scrittura_uart", 2048, NULL, 7, NULL);
	//printf("Avvio i tasks -> OK\n");
	
	//printf("Fine main program\n");
}