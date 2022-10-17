/*!
    \file    main.c
    \brief   led spark with systick, USART print and key example
    
    \version 2019-02-19, V1.0.0, firmware for GD32E23x
    \version 2020-12-12, V1.1.0, firmware for GD32E23x
*/

/*
    Copyright (c) 2020, GigaDevice Semiconductor Inc.

    Redistribution and use in source and binary forms, with or without modification, 
are permitted provided that the following conditions are met:

    1. Redistributions of source code must retain the above copyright notice, this 
       list of conditions and the following disclaimer.
    2. Redistributions in binary form must reproduce the above copyright notice, 
       this list of conditions and the following disclaimer in the documentation 
       and/or other materials provided with the distribution.
    3. Neither the name of the copyright holder nor the names of its contributors 
       may be used to endorse or promote products derived from this software without 
       specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" 
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED 
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. 
IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, 
INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT 
NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR 
PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, 
WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY 
OF SUCH DAMAGE.
*/

#include "gd32e23x.h"
#include "systick.h"
#include <stdio.h>
#include "main.h"
#include "gd32e230c_eval.h"
#include "board_hw/board_hw.h"
#include "lwrb/include/lwrb/lwrb.h"
#include "min_protocol/min.h"
#include "min_protocol/min_id.h"
#include "app_mqtt_protocol/include/app_mqtt.h"
#include "flash.h"
#include "gd32e23x_it.h"
#include "u8g2.h"
#include "app_debug.h"
#include "SEGGER_RTT.h"
#include "SEGGER_RTT_Conf.h"

/*
    if you need find prototype search "PROTOTYPE PLACE"

*/



#define RTC_CLOCK_SOURCE_IRC40K
#define APP_GD32_SPI_TOKEN      0x6699
/* constants definitions */
#define BKP_VALUE    0x32F0

#define LCD_HORIZONTAL 128
#define LCD_VERTICAL 64

#define LCD_MEASURE_X_CENTER(x) ((LCD_HORIZONTAL - x) / 2)
#define LCD_MEASURE_Y_CENTER(x) ((LCD_VERTICAL - x) / 2)

#define APP_SPI_PING_MSG    0x01
#define APP_SPI_BEACON_MSG  0x02
#define APP_SPI_KEY_CONBFIG 0x03

#define MAX_BEACON_SENSOR 10

//time defination
#define FIRSTYEAR 2000 // start year
#define FIRSTDAY 6	   // 0 = Sunday

typedef struct
{
	uint8_t year;
	uint8_t month;
	uint8_t day;
	uint8_t hour;
	uint8_t minute;
	uint8_t second;
	uint8_t weekday;
} date_time_t;

static const uint8_t day_in_month[12] = {31, 29, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};

typedef enum
{
	NO_CONNECT,
	WIFI_CONNECT,
	ETH_CONNECT,
	GSM_CONNECT
} network_status_t;

typedef struct
{
	uint32_t timestamp;
	network_status_t  network_status;
	bool need_update_time;
} esp_status_infor_t;


// ble communication define
typedef struct __attribute((packed))
{
    uint8_t netkey[16];
    uint8_t appkey[16];
} app_provision_key_t;

typedef union
{
    struct __attribute((packed))
    {
     uint16_t token;
     uint8_t msg_id;
     uint8_t msg_length;
     uint8_t payload[251];
    }format;
    uint8_t value[255];
}nrf52_format_packet_t;

typedef struct __attribute((packed))
{
  uint32_t alarm_value;
  uint16_t sensor_count;
  uint8_t gateway_mac[6];
  app_provision_key_t mesh_key;
  uint8_t in_pair_mode;
}app_beacon_ping_msg_t;

typedef struct __attribute((packed))
{
  uint8_t device_mac[6];
  uint8_t device_type;// Maybe Bitfield
  //app_beacon_tid_t beacon_tid;
  uint8_t battery;
}app_beacon_msg_t;

// BLE NOTE INFO VAR
//char trouble_MAC_list[10][6];
typedef struct __attribute((packed))
{
    char MAC[6];
    date_time_t time_got_trouble;
} trouble_info_of_note_t;

static trouble_info_of_note_t trouble_table_info_of_note [MAX_BEACON_SENSOR];
bool there_are_trouble = false;


// TIME VAR
rtc_timestamp_struct rtc_timestamp;
rtc_tamper_struct rtc_tamper;
rtc_parameter_struct rtc_initpara;
__IO uint32_t prescaler_a = 0, prescaler_s = 0;

uint32_t timestamp_temp; // for update
// communication var
lwrb_t uart_rb;
uint8_t uart_buffer[512];
uint8_t min_txbuffer[1024];

uint8_t min_rx_buffer[1024];
min_context_t m_min_context;
min_frame_cfg_t m_min_setting;

uint8_t spi_header[2];
uint8_t spi_data_send_ping[256];
uint8_t spi_data_send[256];
uint8_t spi_data_recieve[256];
bool ready_to_send = false;

// esp32 info var
esp_status_infor_t esp_status_infor;
network_status_t network_status;

static const min_msg_t ping_min_msg = {
    .id = MIN_ID_PING_ESP_ALIVE,
    .len = 0,
    .payload = NULL
};
//LCD PLACE
u8g2_t m_u8g2;
static void delay_ns(uint32_t ns);
uint8_t u8g2_gpio_8080_update_and_delay(U8X8_UNUSED u8x8_t *u8x8,
										U8X8_UNUSED uint8_t msg,
										U8X8_UNUSED uint8_t arg_int,
										U8X8_UNUSED void *arg_ptr);
//PROTOTYPE PLACE
void irc40k_config(void);
static uint8_t convert_input_to_bcd_format(uint8_t value);
uint8_t convert_bcd_to_dec (uint8_t bcd);
void rtc_setup(date_time_t *t);
void rtc_pre_config(void);
void rtc_show_timestamp(void);
void update_time(uint32_t timestamp);
uint32_t convert_date_time_to_second(date_time_t *t);

uint8_t get_weekday(date_time_t time);

void convert_second_to_date_time(uint32_t sec, date_time_t *t, uint8_t Calyear);
//PROTOTYPE PLACE END    

bool check_ping_esp_s = false;
bool esp32_started_s = false;



// spi var
volatile uint32_t send_n = 0, receive_n = 0;
uint8_t spi0_send_array[ARRAYSIZE];
uint8_t spi0_receive_array[ARRAYSIZE];
uint8_t spi_data_flag = 0;
volatile uint32_t sys_counter;

/*!
    \brief      configure the SPI peripheral
    \param[in]  none
    \param[out] none
    \retval     none
*/
void spi_config(void)
{
    gpio_bit_set (GPIO_NRF_CS_PORT, GPIO_NRF_CS_PIN);
    spi_parameter_struct spi_init_struct;

    /* deinitilize SPI and the parameters */
    spi_i2s_deinit(SPI0);
//    spi_i2s_deinit(SPI1);
    spi_struct_para_init(&spi_init_struct);

    /* configure SPI0 parameter */
    spi_init_struct.trans_mode           = SPI_TRANSMODE_FULLDUPLEX;
    spi_init_struct.device_mode          = SPI_MASTER;
    spi_init_struct.frame_size           = SPI_FRAMESIZE_8BIT;
    spi_init_struct.clock_polarity_phase = SPI_CK_PL_HIGH_PH_2EDGE;
    spi_init_struct.nss                  = SPI_NSS_SOFT;
    spi_init_struct.prescale             = SPI_PSC_32;
    spi_init_struct.endian               = SPI_ENDIAN_MSB;
    spi_init(SPI0, &spi_init_struct);

    /* configure SPI1 parameter */
//    spi_init_struct.trans_mode           = SPI_TRANSMODE_FULLDUPLEX;
//    spi_init_struct.device_mode          = SPI_SLAVE;
//    spi_init(SPI1, &spi_init_struct);

    /* configure SPI1 byte access to FIFO */
//    spi_fifo_access_size_config(SPI1, SPI_BYTE_ACCESS);

#if SPI_CRC_ENABLE
    /* configure SPI CRC function */
    spi_crc_length_set(SPI1, SPI_CRC_8BIT);
    spi_crc_polynomial_set(SPI0, 7);
    spi_crc_polynomial_set(SPI1, 7);
    spi_crc_on(SPI0);
    spi_crc_on(SPI1);
#endif /* enable CRC function */
}

//example for transmit data

void spi_send_data (uint8_t* data, uint8_t size)
{
    while(RESET == spi_i2s_flag_get(SPI0, SPI_FLAG_TBE)) {
    }
    //spi_i2s_data_transmit(SPI0, spi0_send_array[send_n++]);
    for (uint8_t i = 0; i < size; i++)
    {
        spi_i2s_data_transmit(SPI0, *(data++));
    }
}

void SPI_transmit_recieve_msg (uint32_t target_port, uint32_t target_pin,uint8_t * data_send, uint32_t size, uint8_t* data_receive)
{
    gpio_bit_reset (target_port, target_pin);
    
    while(spi_i2s_interrupt_flag_get(SPI1, SPI_I2S_INT_FLAG_RBNE) != RESET) {
        *(data_receive++) = spi_i2s_data_receive(SPI0);
    }
    gpio_bit_set (target_port, target_pin);
}

void SPI_transmit_recieve (uint32_t target_port, uint32_t target_pin, uint8_t * data_send, uint32_t size, uint8_t* data_receive)
{
    gpio_bit_reset (target_port, target_pin);
    while(RESET == spi_i2s_flag_get(SPI0, SPI_FLAG_TBE)) {
    }
    for (uint8_t i = 0; i < size; i++)
    {
        spi_i2s_data_transmit(SPI0, *(data_send++));
    }
    while(spi_i2s_interrupt_flag_get(SPI1, SPI_I2S_INT_FLAG_RBNE) != RESET) {
        *(data_receive++) = spi_i2s_data_receive(SPI0);
    }
    gpio_bit_set (target_port, target_pin);
}

void SPI_transmit_self_recieve (uint8_t* data_handle) // NEED PULL CS TO LOW BEFORE CALL THIS FUNCTION
{
    while(RESET == spi_i2s_flag_get(SPI0, SPI_FLAG_TBE)) {
    }
    spi_i2s_data_transmit(SPI0, *data_handle);
    while(spi_i2s_interrupt_flag_get(SPI1, SPI_I2S_INT_FLAG_RBNE) != RESET) {
        *data_handle = spi_i2s_data_receive(SPI0);
    }
}

int fputc(int ch, FILE *f)
{
    usart_data_transmit(USART0, (uint8_t)ch);
    while(RESET == usart_flag_get(USART0, USART_FLAG_TBE));
    return ch;
}
//get tick
uint32_t sys_get_ms(void)
{
    return sys_counter;
}

//Handle min data after receive
uint8_t spi_data_buffer[256];

void min_rx_callback(void *min_context, min_msg_t *frame)
{
    switch (frame->id)
    {
    case MIN_ID_PING_ESP_ALIVE:
        /* code */
        check_ping_esp_s = true;
        //min_msg_t  min_ping_response;
        DEBUG_INFO ("PING OK\r\n");
        //DEBUG_INFO ("PAY LOAD:%s", frame->payload);        
        break;
    case MIN_ID_ESP32_STARTED:
        esp32_started_s = true;
        break;
    case MIN_ID_SEND_SPI_FROM_ESP32:    
        memcpy(spi_data_send, frame->payload,frame->len);
        spi_send_data (spi_data_send, strlen ((char*)spi_data_send));
        ready_to_send = true;
        break;
    case MIN_ID_SEND_HEARTBEAT_MSG:
        break;
    case MIN_ID_SEND_BEACON_MSG:
        break;
    case MIN_ID_SEND_KEY_CONFIG:
        break;
    case MIN_ID_PING_ESP_DEAD:
        check_ping_esp_s = false;
        DEBUG_INFO ("PING DEAD, WILL RESET SOON\r\n");
        break;
    case MIN_ID_TIMESTAMP:
        esp_status_infor = *((esp_status_infor_t*)frame->payload);
        network_status = esp_status_infor.network_status;
        if (esp_status_infor.need_update_time)
        {
            timestamp_temp = (uint32_t)esp_status_infor.timestamp;
            DEBUG_INFO ("update time now\r\n");
            update_time (timestamp_temp);
        }
        else 
        {
            // update display only
            DEBUG_INFO ("NO NEED update time \r\n");
        }
        break;
    default:
        break;
    }
}

bool min_tx_byte(void *ctx, uint8_t byte)
{
	(void)ctx;
	usart_data_transmit(USART0, (uint8_t)byte);
    while(RESET == usart_flag_get(USART0, USART_FLAG_TBE));
	return true;
}

void send_min_data(min_msg_t *min_msg)
{
	min_send_frame(&m_min_context, min_msg);
}

void build_min_tx_data_from_spi(min_msg_t* min_msg, uint8_t* data_spi, uint8_t size)
{
    min_msg->id = MIN_ID_RECIEVE_SPI_FROM_GD32;
    memcpy (min_msg->payload, data_spi, size);
    min_msg->len = size;
}
//MIN DATA PLACE END

//LCD PLACE

void lcd_clr_screen(void)
{
	u8g2_ClearBuffer(&m_u8g2);
	u8g2_ClearDisplay(&m_u8g2);
	u8g2_FirstPage(&m_u8g2);
}

void lcd_display_content(const char *msg)
{
	lcd_clr_screen();

	//lcd_display_header("JIG TEST");
	do
	{
		u8g2_DrawUTF8(&m_u8g2,
					  LCD_MEASURE_X_CENTER(u8g2_GetUTF8Width(&m_u8g2, msg)),
					  20+LCD_MEASURE_Y_CENTER(u8g2_GetMaxCharHeight(&m_u8g2)),
					  msg);
	} while (u8g2_NextPage(&m_u8g2));
	u8g2_SendBuffer(&m_u8g2);
}


void lcd_display_content_at_pos(const char *msg, u8g2_uint_t x, u8g2_uint_t y)
{
	//	lcd_clr_screen();
	//	u8g2_ClearBuffer(&m_u8g2);
	//	u8g2_FirstPage(&m_u8g2);
	char tmp[24];

	u8g2_SetFont(&m_u8g2, u8g2_font_6x13_tf);
	do
	{
		u8g2_DrawUTF8(&m_u8g2, x, y, msg);
	} while (u8g2_NextPage(&m_u8g2));
	u8g2_SendBuffer(&m_u8g2);
}

void lcd_display_status (void)
{
    char counter[24];
    u8g2_ClearBuffer(&m_u8g2);

    // Display header
    //RTC_TimeTypeDef time;
    //HAL_RTC_GetTime(&hrtc, &time, RTC_FORMAT_BIN);
    rtc_timestamp_get(&rtc_timestamp);
//    /* get the subsecond value of timestamp time, and convert it into fractional format */
//    uint32_t ts_subsecond;
//    uint8_t ts_subsecond_ss,ts_subsecond_ts,ts_subsecond_hs;
//    ts_subsecond = rtc_timestamp_subsecond_get();
//    ts_subsecond_ss=(1000-(ts_subsecond*1000+1000)/400)/100;
//    ts_subsecond_ts=(1000-(ts_subsecond*1000+1000)/400)%100/10;
//    ts_subsecond_hs=(1000-(ts_subsecond*1000+1000)/400)%10;
    sprintf(counter, "%02u:%02u:%02u", rtc_timestamp.rtc_timestamp_hour, rtc_timestamp.rtc_timestamp_minute, rtc_timestamp.rtc_timestamp_second);
    u8g2_SetFont(&m_u8g2, u8g2_font_unifont_t_vietnamese1); // can chon font cho header
    do
    {
        u8g2_DrawUTF8(&m_u8g2,
                      LCD_MEASURE_X_CENTER(u8g2_GetUTF8Width(&m_u8g2, counter)),
                      15,
                      counter);
    } while (u8g2_NextPage(&m_u8g2));

    u8g2_SetFont(&m_u8g2, u8g2_font_6x13_tf);
    if (network_status == WIFI_CONNECT)
    {
        sprintf(counter, "WIFI CONNECTED");
    }
    else if (network_status == ETH_CONNECT)
    {
        sprintf(counter, "ETH CONNECTED");
    }
    else if (network_status == GSM_CONNECT)
    {
        sprintf(counter, "GSM CONNECTED");
    }
    else
    {
        sprintf(counter, "LOST CONNECTION");
    }
    u8g2_DrawUTF8(&m_u8g2, 32, 38, counter);
    
    //sprintf(counter, "IMEI %s", final_jig_value.gsm_imei);
    //u8g2_DrawUTF8(&m_u8g2, 5,55, counter);
    u8g2_SendBuffer(&m_u8g2);
}

void lcd_display_trouble (trouble_info_of_note_t* trouble_info_table, uint8_t trouble_sensor_cnt)
{
    char counter[24];
    u8g2_ClearBuffer(&m_u8g2);
    u8g2_SetFont(&m_u8g2, u8g2_font_unifont_t_vietnamese1); // can chon font cho header
    
    sprintf(counter, "TIME %02u:%02u:%02u", (trouble_info_table[trouble_sensor_cnt]).time_got_trouble.hour,
                                   (trouble_info_table[trouble_sensor_cnt]).time_got_trouble.minute,
                                   (trouble_info_table[trouble_sensor_cnt]).time_got_trouble.second);
    do
    {
        u8g2_DrawUTF8(&m_u8g2,
                      LCD_MEASURE_X_CENTER(u8g2_GetUTF8Width(&m_u8g2, counter)),
                      15,
                      counter);
    }while (u8g2_NextPage(&m_u8g2));
    u8g2_SetFont(&m_u8g2, u8g2_font_6x13_tf);
    sprintf(counter, "MAC ALARM:");
    u8g2_DrawUTF8(&m_u8g2, 32, 38, counter);
    sprintf(counter, "%s", (trouble_info_table[trouble_sensor_cnt]).MAC);
    u8g2_DrawUTF8(&m_u8g2, 5,55, counter);
}

/*
    COPY DATA FROM SPI INTO TABLE
*/

void add_info_into_trouble_mac_table (trouble_info_of_note_t* trouble_info_table, char* MAC)
{
    uint32_t ts_subsecond = 0;
    uint8_t ts_subsecond_ss,ts_subsecond_ts,ts_subsecond_hs ;
  
    rtc_timestamp_get(&rtc_timestamp);
//    /* get the subsecond value of timestamp time, and convert it into fractional format */
//    ts_subsecond = rtc_timestamp_subsecond_get();
//    ts_subsecond_ss=(1000-(ts_subsecond*1000+1000)/400)/100;
//    ts_subsecond_ts=(1000-(ts_subsecond*1000+1000)/400)%100/10;
//    ts_subsecond_hs=(1000-(ts_subsecond*1000+1000)/400)%10;
    date_time_t time_now;
    time_now.day = convert_bcd_to_dec (rtc_timestamp.rtc_timestamp_date);
    time_now.month = (rtc_timestamp.rtc_timestamp_month);
    time_now.year = convert_bcd_to_dec (rtc_initpara.rtc_year);
    time_now.hour = (rtc_timestamp.rtc_timestamp_hour);
    time_now.minute = convert_bcd_to_dec (rtc_timestamp.rtc_timestamp_minute);
    time_now.second = convert_bcd_to_dec (rtc_timestamp.rtc_timestamp_second);
    memcpy (&(trouble_info_table->time_got_trouble), &time_now, sizeof(date_time_t));
    memcpy (&(trouble_info_table->MAC), MAC, sizeof(trouble_info_table->MAC));
}

void check_trouble_MAC (trouble_info_of_note_t* trouble_info_table)
{
    char mac_sample[6];
    memset (mac_sample,'\0', sizeof (mac_sample));
    static uint8_t i = 0;
    
    if (i < MAX_BEACON_SENSOR)
    {
        if (memcmp (trouble_info_table[i].MAC, mac_sample, sizeof (mac_sample)) != 0)
        {
            lcd_display_trouble (trouble_info_table, i);
            i++;
        }
        else
        {
            // display again from start or no one got trouble
            i = 0;
        }
    }
    else
    {
        //all sensor got trouble
        i = 0;
    }
}

//LCD PLACE END

//DEBUG OUTPUT

uint32_t rtt_tx(const void *buffer, uint32_t size)
{
    return SEGGER_RTT_Write(0, buffer, size);
}


// TEST SPI PING

uint8_t request_spi_message(uint8_t* p_out, uint8_t MSG_ID)
{
    nrf52_format_packet_t nrf52_local_packet;
    nrf52_local_packet.format.token = APP_GD32_SPI_TOKEN;
    nrf52_local_packet.format.msg_id = MSG_ID;
    nrf52_local_packet.format.msg_length = 0;
    memset(&nrf52_local_packet.format.payload, 0, sizeof(nrf52_local_packet.format.payload));
    /*Write*/
    /*nrF52 CS Pin*/
    gpio_bit_reset(GPIO_NRF_CS_PORT, GPIO_NRF_CS_PIN);
    DEBUG_INFO ("send test spi msg");
    for(uint16_t i = 0; i < sizeof(nrf52_format_packet_t); i++)
    {
        //SPI_transmit_self_recieve (&(nrf52_local_packet.value[i]));  // if there are hardware UNCOMMENT THIS LINE 
        while(RESET == spi_i2s_flag_get(SPI0, SPI_FLAG_TBE)) {
        }
            spi_i2s_data_transmit(SPI0, nrf52_local_packet.value[i]);
        while(spi_i2s_interrupt_flag_get(SPI1, SPI_I2S_INT_FLAG_RBNE) != RESET) {
            nrf52_local_packet.value[i] = spi_i2s_data_receive(SPI0);
        }
    }
    uint8_t ping_msg_length = 0;
    if(nrf52_local_packet.format.token == APP_GD32_SPI_TOKEN && nrf52_local_packet.format.msg_id == MSG_ID)
    {
        DEBUG_INFO("Found msg response\r\n");
        ping_msg_length = nrf52_local_packet.format.msg_length;
        app_beacon_ping_msg_t *p_ping_msg = (app_beacon_ping_msg_t*)&nrf52_local_packet.format.payload;
        /*<Only for Debug>*/
        if (MSG_ID == APP_SPI_PING_MSG)
        {
            DEBUG_INFO("Alarm value:%0x08x\r\nTotal Sensor count:%u\r\nPair mode:%d\r\nGateway Mac:", p_ping_msg->alarm_value, p_ping_msg->sensor_count, p_ping_msg->in_pair_mode);
            for(uint8_t  i = 0; i < 6; i++)
            {
                DEBUG_RAW("%02x-", p_ping_msg->gateway_mac[i]);
            }
            DEBUG_RAW("\r\nAppKey");
            for(uint8_t  i = 0; i < 16; i++)
            {
                DEBUG_RAW("%02x-", p_ping_msg->mesh_key.appkey[i]);
            }
            DEBUG_RAW("\r\nnetKey");
            for(uint8_t  i = 0; i < 16; i++)
            {
                DEBUG_RAW("%02x-", p_ping_msg->mesh_key.netkey[i]);
            }
            DEBUG_RAW("\r\n");
        }
    }
    else
    {
        DEBUG_ERROR("Wrong ping response format\r\n");
    }
    memcpy (p_out, nrf52_local_packet.format.payload, ping_msg_length);
    // nRF52 CS PIN
    gpio_bit_set(GPIO_NRF_CS_PORT, GPIO_NRF_CS_PIN);
    return ping_msg_length;
}


/*!
    \brief      main function
    \param[in]  none
    \param[out] none
    \retval     none
*/

int main(void)
{
    /* configure systick */
    irc40k_config();
    systick_config();
    
    board_hw_initialize();
    
    lwrb_init(&uart_rb, uart_buffer, sizeof(uart_buffer));
    m_min_setting.get_ms = sys_get_ms;
    m_min_setting.last_rx_time = 0x00;
	m_min_setting.rx_callback = min_rx_callback;
	m_min_setting.timeout_not_seen_rx = 5000;
	m_min_setting.tx_byte = min_tx_byte;
	m_min_setting.use_timeout_method = 0;

    m_min_context.callback = &m_min_setting;
	m_min_context.rx_frame_payload_buf = min_rx_buffer;
	min_init_context(&m_min_context);
    
//*********  CONFIG BACKUP VALUE AND RTC*******    
    /* enable access to RTC registers in backup domain */
    rcu_periph_clock_enable(RCU_PMU);
    pmu_backup_write_enable();
  
    rtc_pre_config();
    rtc_tamper_disable(RTC_TAMPER0);
  
    /* check if RTC has aready been configured */
    if (BKP_VALUE != RTC_BKP0){    
        rtc_setup(NULL); 
    }else{
        /* detect the reset source */
        if (RESET != rcu_flag_get(RCU_FLAG_PORRST)){
            DEBUG_INFO("power on reset occurred....\n\r");
        }else if (RESET != rcu_flag_get(RCU_FLAG_EPRST)){
            DEBUG_INFO("external reset occurred....\n\r");
        }
        DEBUG_INFO("no need to configure RTC....\n\r");
               
        //rtc_show_time(); // WE NEED DISPLAY TIME ON LCD ONCE A SECOND
    }
    
//    com_usart_init();
//    usart_enable(USART1);
//    //spi configuration
//    nvic_irq_enable (SPI0_IRQn, 1);
//    spi_config ();
//    spi_enable (SPI0);
    // app debug
    app_debug_init(sys_get_ms,NULL);
    app_debug_register_callback_print(rtt_tx);
    DEBUG_INFO ("APP DEBUG OK\r\n");
    while(RESET == usart_flag_get(USART0, USART_FLAG_TC));
    usart_interrupt_enable(USART0, USART_INT_RBNE);
	//usart_interrupt_enable(USART0, USART_INT_TBE);
    uint8_t databuff1[128];
    while (0)
    {   
        //min_send_frame(&m_min_context, (min_msg_t*)&ping_min_msg);
        DEBUG_INFO ("STILL RUN \r\n");
        uint16_t uart_data_leng = lwrb_read (&uart_rb, databuff1, 128);
        if (uart_data_leng)
        {
            DEBUG_INFO ("DATA UART FROM ESP32");
            min_rx_feed (&m_min_context, databuff1, uart_data_leng);
        }
        delay_1ms (200);
    }
#if 1 // paralle mode
	u8g2_Setup_st7920_p_128x64_f(&m_u8g2, U8G2_R2, u8x8_byte_8bit_8080mode, u8g2_gpio_8080_update_and_delay);
#else
#warning "do nothing"
//	software_spi_cfg_t spi_cfg;
//	spi_cfg.cs_out = software_spi_set_cs;
//	spi_cfg.miso_in = software_spi_read_miso;
//	spi_cfg.mosi_out = software_spi_set_mosi;
//	spi_cfg.sck_out = software_spi_set_sck;
//	software_spi_inititalize(&spi_cfg);
//	u8g2_Setup_st7920_s_128x64_f(&m_u8g2, U8G2_R0, u8x8_byte_3wire_sw_spi, u8g2_gpio_3w_spi_update_and_delay);
#endif
	u8g2_InitDisplay(&m_u8g2);	   // send init sequence to the display, display is in sleep mode after this,
	u8g2_SetPowerSave(&m_u8g2, 0); // wake up display
	u8g2_ClearBuffer(&m_u8g2);
	u8g2_ClearDisplay(&m_u8g2);
    
    
//    /* print out the clock frequency of system, AHB, APB1 and APB2 */
//    DEBUG_INFO("\r\nCK_SYS is %d", rcu_clock_freq_get(CK_SYS));
//    DEBUG_INFO("\r\nCK_AHB is %d", rcu_clock_freq_get(CK_AHB));
//    DEBUG_INFO("\r\nCK_APB1 is %d", rcu_clock_freq_get(CK_APB1));
//    DEBUG_INFO("\r\nCK_APB2 is %d", rcu_clock_freq_get(CK_APB2));
    
    static uint32_t now = 0;
    static uint32_t last_time = 0;
    static uint32_t last_time_ping = 0;
    uint8_t databuff[128];
    size_t btr_m;
    min_msg_t min_data_buff;
    
//    lcd_clr_screen();
//    lcd_display_content ("lcd test");
//    DEBUG_INFO ("PASS LCD \r\n");
    fwdgt_config(1250, FWDGT_PSC_DIV64);
    fwdgt_enable();
    DEBUG_INFO ("enable wdt \r\n");
    gpio_bit_set(GPIO_ESP_EN_PORT, GPIO_ESP_EN_PIN);
    
    //init some para
    memset (trouble_table_info_of_note,'\0', sizeof (trouble_table_info_of_note)); //reset list trouble mac
    
    
    
    while(1){
        /* reload FWDGT counter */
        fwdgt_counter_reload();
        
        now = sys_get_ms();
        if (check_ping_esp_s)
        {
            check_ping_esp_s = false;
            last_time = now;
        }
        
        if (((now - last_time) > 6000) && (!check_ping_esp_s) && esp32_started_s) //need to wait esp32 stable then check ping signal
        {
            // reset esp32
            last_time = now;
            DEBUG_INFO ("TIME OUT RESET ESP32\r\n");
            gpio_bit_reset(GPIO_ESP_EN_PORT, GPIO_ESP_EN_PIN);
            
            delay_1ms (1000);
            
            gpio_bit_set(GPIO_ESP_EN_PORT, GPIO_ESP_EN_PIN);
            esp32_started_s = false; // this time esp was dead, signal will be send again when esp32 is started
            continue;
            //restart loop
        }
        static uint8_t sensor_cnt = 0;
        if ((now - last_time_ping) > 1000)
        {
            static uint8_t spi_msg_data[251];
            app_beacon_ping_msg_t* ble_status;
            uint8_t byte_read = request_spi_message (spi_msg_data, APP_SPI_PING_MSG);
            // process and send data to esp
            ble_status = (app_beacon_ping_msg_t*)&spi_msg_data;
            if (ble_status->alarm_value > 1)
            {
                there_are_trouble = true;
                //add mac to trouble mac list
                add_info_into_trouble_mac_table(trouble_table_info_of_note, (char *)(ble_status->gateway_mac));
                sensor_cnt ++;
            }
        }
        if (there_are_trouble)
        {
            // add delay time
            check_trouble_MAC(trouble_table_info_of_note);
        }
        //feed data to min progress

        uint16_t uart_data_leng = lwrb_read (&uart_rb, databuff, 1);
        if (uart_data_leng)
        {
            DEBUG_INFO ("DATA UART FROM ESP32\r\n");
            min_rx_feed (&m_min_context, databuff, uart_data_leng);
        }
        
        //transmit data to ask for spi data
        //spi_send_data ((char *)"need you send data", strlen ("need you send data"));
        //ping piority
        
        //wait till there no data in register
//        if (ready_to_send)
//        {
//            // recieve data 
//            DEBUG_INFO ("DATA GET FROM ESP32 NEED TO SEND");
//            SPI_transmit_recieve(GPIO_NRF_CS_PORT, GPIO_NRF_CS_PIN, spi_data_send, 128, spi_data_recieve);
//            ready_to_send = false;
//        }
        
        //if need send lora data
        {
           // SPI_transmit_recieve (GPIO_LORA_CS_PORT, GPIO_LORA_CS_PIN, spi_data_send, 128, spi_data_recieve);
        }
        
    }
}

static void delay_ns(uint32_t ns)
{
	for (uint32_t i = 0; i < ns * 2; i++)
	{
		__NOP();
		__NOP();
		__NOP();
		__NOP();
		__NOP();
		__NOP();
	}
}

uint8_t u8g2_gpio_8080_update_and_delay(U8X8_UNUSED u8x8_t *u8x8, U8X8_UNUSED uint8_t msg, U8X8_UNUSED uint8_t arg_int, U8X8_UNUSED void *arg_ptr)
{
	switch (msg)
	{
	// Initialize SPI peripheral
	case U8X8_MSG_GPIO_AND_DELAY_INIT:
	{
		gpio_bit_set(GPIO_LCD_PWM_PORT, GPIO_LCD_PWM_PIN);
	}
	break;

	case U8X8_MSG_DELAY_MILLI:
	{
        delay_1ms(arg_int);
	}
	break;

	// Function which delays 10us
	case U8X8_MSG_DELAY_10MICRO:
	{
		for (volatile uint32_t n = 0; n < 720 * arg_int; n++)
		{
			__NOP();
			__NOP();
		}
	}
	break;
	// Function which delays 100ns
	case U8X8_MSG_DELAY_100NANO:
	{
		delay_ns(100 * arg_int);
	}
	break;

	case U8X8_MSG_DELAY_NANO:
	{
		delay_ns(arg_int);
	}
	break;

	// Function to define the logic level of the clockline
	case U8X8_MSG_GPIO_D0:
	{
		if (arg_int)
		{
			gpio_bit_reset(GPIO_LCD_D0_PORT, GPIO_LCD_D0_PIN);
		}
		else
		{
			gpio_bit_set(GPIO_LCD_D0_PORT, GPIO_LCD_D0_PIN);
		}
	}
	break;

	case U8X8_MSG_GPIO_D1:
	{
		if (arg_int)
		{
			gpio_bit_reset(GPIO_LCD_D1_PORT, GPIO_LCD_D1_PIN);
		}
		else
		{
			gpio_bit_set(GPIO_LCD_D1_PORT, GPIO_LCD_D1_PIN);
		}
	}
	break;

	case U8X8_MSG_GPIO_D2:
	{
		if (arg_int)
		{
			gpio_bit_reset(GPIO_LCD_D2_PORT, GPIO_LCD_D2_PIN);
		}
		else
		{
			gpio_bit_set(GPIO_LCD_D2_PORT, GPIO_LCD_D2_PIN);
		}
	}
	break;

	case U8X8_MSG_GPIO_D3:
	{
		if (arg_int)
		{
			gpio_bit_reset(GPIO_LCD_D3_PORT, GPIO_LCD_D3_PIN);
		}
		else
		{
            gpio_bit_set(GPIO_LCD_D3_PORT, GPIO_LCD_D3_PIN);
		}
	}
	break;

	case U8X8_MSG_GPIO_D4:
	{
		if (arg_int)
		{
			gpio_bit_reset(GPIO_LCD_D4_PORT, GPIO_LCD_D4_PIN);
		}
		else
		{
			gpio_bit_set(GPIO_LCD_D4_PORT, GPIO_LCD_D4_PIN);
		}
	}
	break;

	case U8X8_MSG_GPIO_D5:
	{
        if (arg_int)
        {
            gpio_bit_reset(GPIO_LCD_D5_PORT, GPIO_LCD_D5_PIN);
        }
        else
        {
            gpio_bit_set(GPIO_LCD_D5_PORT, GPIO_LCD_D5_PIN);
        }
	}
	break;

	case U8X8_MSG_GPIO_D6:
	{
		if (arg_int)
		{
			gpio_bit_reset(GPIO_LCD_D6_PORT, GPIO_LCD_D6_PIN);
		}
		else
		{
			gpio_bit_set(GPIO_LCD_D6_PORT, GPIO_LCD_D6_PIN);
		}
	}
	break;

	case U8X8_MSG_GPIO_D7:
	{
		if (arg_int)
		{
			gpio_bit_reset(GPIO_LCD_D7_PORT, GPIO_LCD_D7_PIN);
		}
		else
		{
			gpio_bit_set(GPIO_LCD_D7_PORT, GPIO_LCD_D7_PIN);
		}
	}
	break;

	case U8X8_MSG_GPIO_E:
	{
		if (arg_int)
		{
			gpio_bit_reset(GPIO_LCD_EN_PORT, GPIO_LCD_EN_PIN);
		}
		else
		{
			gpio_bit_set(GPIO_LCD_EN_PORT, GPIO_LCD_EN_PIN);
		}
	}
	break;

	case U8X8_MSG_GPIO_CS:
	{
		if (arg_int)
		{
			gpio_bit_reset(GPIO_LCD_RW_PORT, GPIO_LCD_RW_PIN);
		}
		else
		{
			gpio_bit_set(GPIO_LCD_RW_PORT, GPIO_LCD_RW_PIN);
		}
	}
	break;

	case U8X8_MSG_GPIO_DC:
	{
		if (arg_int)
		{
			gpio_bit_reset(GPIO_LCD_DI_PORT, GPIO_LCD_DI_PIN);
		}
		else
		{
			gpio_bit_set(GPIO_LCD_DI_PORT, GPIO_LCD_DI_PIN);
		}
	}
	break;

	case U8X8_MSG_GPIO_RESET:
	{
        
	}
	break;

	case U8X8_MSG_GPIO_MENU_SELECT:
	{
		u8x8_SetGPIOResult(u8x8, /* get menu select pin state */ 0);
	}
	break;

	case U8X8_MSG_GPIO_MENU_NEXT:
	{
		u8x8_SetGPIOResult(u8x8, /* get menu next pin state */ 0);
	}
	break;

	case U8X8_MSG_GPIO_MENU_PREV:
	{
		u8x8_SetGPIOResult(u8x8, /* get menu prev pin state */ 0);
	}
	break;

	case U8X8_MSG_GPIO_MENU_HOME:
	{
		u8x8_SetGPIOResult(u8x8, /* get menu home pin state */ 0);
	}
	break;

	default:
		//DEBUG_ERROR("Unhandled case %d\r\n", msg);
		return 0; // A message was received which is not implemented, return 0 to indicate an error
	}

	return 1; // command processed successfully.
}


// TIME AND RTC PLACE

/*!
    \brief      IRC40K configuration function
    \param[in]  none
    \param[out] none
    \retval     none
*/
void irc40k_config(void)
{
    /* enable IRC40K */
    rcu_osci_on(RCU_IRC40K);
    /* wait till IRC40K is ready */
    rcu_osci_stab_wait(RCU_IRC40K);
}

/*!
    \brief      RTC configuration function
    \param[in]  none
    \param[out] none
    \retval     none
*/
void rtc_pre_config(void)
{
    #if defined (RTC_CLOCK_SOURCE_IRC40K) 
          //rcu_osci_on(RCU_IRC40K); //use same clock source with watchdog so don't need turn it on again
          //rcu_osci_stab_wait(RCU_IRC40K);
          rcu_rtc_clock_config(RCU_RTCSRC_IRC40K);
  
          prescaler_s = 0x18F;
          prescaler_a = 0x63;
    #elif defined (RTC_CLOCK_SOURCE_LXTAL)
          rcu_osci_on(RCU_LXTAL);
          rcu_osci_stab_wait(RCU_LXTAL);
          rcu_rtc_clock_config(RCU_RTCSRC_LXTAL);
          prescaler_s = 0xFF;
          prescaler_a = 0x7F;
    #else
    #error RTC clock source should be defined.
    #endif /* RTC_CLOCK_SOURCE_IRC40K */

    rcu_periph_clock_enable(RCU_RTC);
    rtc_register_sync_wait();
}

static uint8_t convert_input_to_bcd_format(uint8_t value)
{
    uint32_t index = 0;
    uint32_t tmp[2] = {0, 0};
    if (value < 10)
    {
        tmp[0] = 0;
        tmp[1] = value;
    }
    else if ((value >= 10) && (value < 100))
    {
        tmp[0] = value / 10;
        tmp[1] = value % 10;
    }
    else
    {
        DEBUG_ERROR ("INPUT TIME VALUE WRONG: %d", value);
    }
    index = (tmp[1]) + ((tmp[0]) <<4);
    return index;
}
uint8_t convert_bcd_to_dec (uint8_t bcd)
{
   uint32_t index = 0;
   uint32_t tmp[2] = {0, 0};
   if (bcd < 0x10)
   {
       tmp[0] = 0;
       tmp[1] = bcd;
   }
   else
   {
       tmp[1] = bcd & 0x0f;
       tmp[0] = (bcd & 0xf0) >> 4;
   }
   index = (tmp[1]) + ((tmp[0])*10);
   return index;
}


void rtc_setup(date_time_t *t)
{
    /* setup RTC time value */
    uint32_t tmp_hh = 0xFF, tmp_mm = 0xFF, tmp_ss = 0xFF;

    rtc_initpara.rtc_factor_asyn = prescaler_a;
    rtc_initpara.rtc_factor_syn = prescaler_s;
    rtc_initpara.rtc_display_format = RTC_24HOUR;
    rtc_initpara.rtc_am_pm = RTC_AM;
    if (t == NULL)
    {
        rtc_initpara.rtc_year = 0x16;
        rtc_initpara.rtc_day_of_week = RTC_SATURDAY;
        rtc_initpara.rtc_month = RTC_APR;
        rtc_initpara.rtc_date = 0x30;
        rtc_initpara.rtc_hour = 0;
        rtc_initpara.rtc_minute = 0;
        rtc_initpara.rtc_second = 0;
    }
    else
    {
        rtc_initpara.rtc_year = convert_input_to_bcd_format (t->year);
        rtc_initpara.rtc_month = (t->month);
        rtc_initpara.rtc_date = convert_input_to_bcd_format (t->day);
        if (get_weekday (*t) == 0)
        {
           rtc_initpara.rtc_day_of_week = RTC_SUNDAY; 
        }
        else
        {
           rtc_initpara.rtc_day_of_week =  get_weekday (*t);
        }
        rtc_initpara.rtc_hour =(t->hour);
        rtc_initpara.rtc_minute = convert_input_to_bcd_format (t->minute);
        rtc_initpara.rtc_second = convert_input_to_bcd_format (t->second);
    }
    
    /* RTC current time configuration */
    if(ERROR == rtc_init(&rtc_initpara)){    
        DEBUG_INFO("** RTC time configuration failed! **\n\r");
    }else{
        DEBUG_INFO("** RTC time configuration success! **\n\r");
        rtc_show_timestamp();
        RTC_BKP0 = BKP_VALUE;
    }   
}

void rtc_show_timestamp(void)
{
    uint32_t ts_subsecond = 0;
    uint8_t ts_subsecond_ss,ts_subsecond_ts,ts_subsecond_hs ;
  
    rtc_timestamp_get(&rtc_timestamp);
    /* get the subsecond value of timestamp time, and convert it into fractional format */
    ts_subsecond = rtc_timestamp_subsecond_get();
    ts_subsecond_ss=(1000-(ts_subsecond*1000+1000)/400)/100;
    ts_subsecond_ts=(1000-(ts_subsecond*1000+1000)/400)%100/10;
    ts_subsecond_hs=(1000-(ts_subsecond*1000+1000)/400)%10;

    DEBUG_INFO("Get the time-stamp time: %0.2x:%0.2x:%0.2x .%d%d%d \n\r", \
          rtc_timestamp.rtc_timestamp_hour, rtc_timestamp.rtc_timestamp_minute, rtc_timestamp.rtc_timestamp_second,\
          ts_subsecond_ss, ts_subsecond_ts, ts_subsecond_hs);
}

void convert_second_to_date_time(uint32_t sec, date_time_t *t, uint8_t Calyear)
{
	uint16_t day;
	uint8_t year;
	uint16_t days_of_year;
	uint8_t leap400;
	uint8_t month;

	t->second = sec % 60;
	sec /= 60;
	t->minute = sec % 60;
	sec /= 60;
	t->hour = sec % 24;

	if (Calyear == 0)
		return;

	day = (uint16_t)(sec / 24);

	year = FIRSTYEAR % 100;					   // 0..99
	leap400 = 4 - ((FIRSTYEAR - 1) / 100 & 3); // 4, 3, 2, 1

	for (;;)
	{
		days_of_year = 365;
		if ((year & 3) == 0)
		{
			days_of_year = 366; // leap year
			if (year == 0 || year == 100 || year == 200)
			{ // 100 year exception
				if (--leap400)
				{ // 400 year exception
					days_of_year = 365;
				}
			}
		}
		if (day < days_of_year)
		{
			break;
		}
		day -= days_of_year;
		year++; // 00..136 / 99..235
	}
	t->year = year + FIRSTYEAR / 100 * 100 - 2000; // + century
	if (days_of_year & 1 && day > 58)
	{		   // no leap year and after 28.2.
		day++; // skip 29.2.
	}

	for (month = 1; day >= day_in_month[month - 1]; month++)
	{
		day -= day_in_month[month - 1];
	}

	t->month = month; // 1..12
	t->day = day + 1; // 1..31
}

uint8_t get_weekday(date_time_t time)
{
	time.weekday = (time.day +=
					time.month < 3 ? time.year-- : time.year - 2,
					23 * time.month / 9 + time.day + 4 + time.year / 4 -
						time.year / 100 + time.year / 400);
	return time.weekday % 7;
}

uint32_t convert_date_time_to_second(date_time_t *t)
{
	uint8_t i;
	uint32_t result = 0;
	uint16_t idx, year;

	year = t->year + 2000;

	/* Calculate days of years before */
	result = (uint32_t)year * 365;
	if (t->year >= 1)
	{
		result += (year + 3) / 4;
		result -= (year - 1) / 100;
		result += (year - 1) / 400;
	}

	/* Start with 2000 a.d. */
	result -= 730485UL;

	/* Make month an array index */
	idx = t->month - 1;

	/* Loop thru each month, adding the days */
	for (i = 0; i < idx; i++)
	{
		result += day_in_month[i];
	}

	/* Leap year? adjust February */
	if (!(year % 400 == 0 || (year % 4 == 0 && year % 100 != 0)))
	{
		if (t->month > 2)
		{
			result--;
		}
	}

	/* Add remaining days */
	result += t->day;

	/* Convert to seconds, add all the other stuff */
	result = (result - 1) * 86400L + (uint32_t)t->hour * 3600 +
			 (uint32_t)t->minute * 60 + t->second;
	return result;
}

void update_time(uint32_t timestamp)
{
    date_time_t time_now;
    convert_second_to_date_time (timestamp, &time_now, 1);
    rtc_setup (&time_now);
}
// RTC AND TIME PLACE END

// UART0
void uart0_handler(void)
{
    uint8_t data = usart_data_receive(USART0);
    lwrb_write (&uart_rb, &data, 1);
}
