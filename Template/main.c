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

rtc_timestamp_struct rtc_timestamp;
rtc_tamper_struct rtc_tamper;
rtc_parameter_struct rtc_initpara;
__IO uint32_t prescaler_a = 0, prescaler_s = 0;


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

uint8_t min_data_send [256];


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
void rtc_setup(void);
void rtc_pre_config(void);
void rtc_show_timestamp(void);
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

//handle min data after receive
uint8_t spi_data_buffer[256];
void min_rx_callback(void *min_context, min_msg_t *frame)
{
    switch (frame->id)
    {
    case MIN_ID_PING_ESP_ALIVE:
        /* code */
        check_ping_esp_s = true;
        min_msg_t  min_ping_response;
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

void lcd_clr_screen(void)
{
	u8g2_ClearBuffer(&m_u8g2);
	u8g2_ClearDisplay(&m_u8g2);
	//	u8g2_SetFont(&m_u8g2, u8g2_font_unifont_t_vietnamese1);
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

uint32_t rtt_tx(const void *buffer, uint32_t size)
{
    return SEGGER_RTT_Write(0, buffer, size);
}

void request_ping_message(uint8_t* p_out)
{
    nrf52_format_packet_t nrf52_local_packet;
    nrf52_local_packet.format.token = APP_GD32_SPI_TOKEN;
    nrf52_local_packet.format.msg_id = APP_SPI_PING_MSG;
    nrf52_local_packet.format.msg_length = 0;
    memset(&nrf52_local_packet.format.payload, 0, sizeof(nrf52_local_packet.format.payload));
    /*Write*/
    /*nrF52 CS Pin*/
    gpio_bit_reset(GPIO_NRF_CS_PORT, GPIO_NRF_CS_PIN);
    DEBUG_INFO ("send test spi msg");
    for(uint16_t i = 0; i < sizeof(nrf52_format_packet_t); i++)
    {
        //nrf52_local_packet.value[i] = m_write(nrf52_local_packet.value[i]);
        while(RESET == spi_i2s_flag_get(SPI0, SPI_FLAG_TBE)) {
        }
            spi_i2s_data_transmit(SPI0, nrf52_local_packet.value[i]);
        while(spi_i2s_interrupt_flag_get(SPI1, SPI_I2S_INT_FLAG_RBNE) != RESET) {
            nrf52_local_packet.value[i] = spi_i2s_data_receive(SPI0);
        }
    }
    
    if(nrf52_local_packet.format.token == APP_GD32_SPI_TOKEN && nrf52_local_packet.format.msg_id == APP_SPI_PING_MSG)
    {
            DEBUG_INFO("Found Ping response\r\n");
            uint8_t ping_msg_length = nrf52_local_packet.format.msg_length;
            app_beacon_ping_msg_t *p_ping_msg = (app_beacon_ping_msg_t*)&nrf52_local_packet.format.payload;
            /*<Only for Debug>*/
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
    else
    {
            DEBUG_ERROR("Wrong ping response format\r\n");
    }
    memcpy (p_out, &nrf52_local_packet, 256);
    // nRF52 CS PIN
    gpio_bit_set(GPIO_NRF_CS_PORT, GPIO_NRF_CS_PIN);
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
        rtc_setup(); 
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
    uint8_t databuff[128];
    size_t btr_m;
    min_msg_t min_data_buff;
    
    //lcd_clr_screen();
    //lcd_display_content ("lcd test");
    DEBUG_INFO ("PASS LCD \r\n");
    fwdgt_config(1250, FWDGT_PSC_DIV64);
    fwdgt_enable();
    DEBUG_INFO ("enable wdt \r\n");
    gpio_bit_set(GPIO_ESP_EN_PORT, GPIO_ESP_EN_PIN);
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

void rtc_setup(void)
{
    /* setup RTC time value */
    uint32_t tmp_hh = 0xFF, tmp_mm = 0xFF, tmp_ss = 0xFF;

    rtc_initpara.rtc_factor_asyn = prescaler_a;
    rtc_initpara.rtc_factor_syn = prescaler_s;
    rtc_initpara.rtc_year = 0x16;
    rtc_initpara.rtc_day_of_week = RTC_SATURDAY;
    rtc_initpara.rtc_month = RTC_APR;
    rtc_initpara.rtc_date = 0x30;
    rtc_initpara.rtc_display_format = RTC_24HOUR;
    rtc_initpara.rtc_am_pm = RTC_AM;

    /* current time input 
    DEBUG_INFO("=======Configure RTC Time========\n\r");
    DEBUG_INFO("  please input hour:\n\r");
    while (tmp_hh == 0xFF){    
        tmp_hh = usart_input_threshold(23);
        rtc_initpara.rtc_hour = tmp_hh;
    }
    DEBUG_INFO("  %0.2x\n\r", tmp_hh);
    
    DEBUG_INFO("  please input minute:\n\r");
    while (tmp_mm == 0xFF){    
        tmp_mm = usart_input_threshold(59);
        rtc_initpara.rtc_minute = tmp_mm;
    }
    DEBUG_INFO("  %0.2x\n\r", tmp_mm);

    DEBUG_INFO("  please input second:\n\r");
    while (tmp_ss == 0xFF){
        tmp_ss = usart_input_threshold(59);
        rtc_initpara.rtc_second = tmp_ss;
    }
    DEBUG_INFO("  %0.2x\n\r", tmp_ss);
*/
    /* RTC current time configuration */
    if(ERROR == rtc_init(&rtc_initpara)){    
        DEBUG_INFO("** RTC time configuration failed! **\n\r");
    }else{
        DEBUG_INFO("** RTC time configuration success! **\n\r");
        //rtc_show_time();
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

    printf("Get the time-stamp time: %0.2x:%0.2x:%0.2x .%d%d%d \n\r", \
          rtc_timestamp.rtc_timestamp_hour, rtc_timestamp.rtc_timestamp_minute, rtc_timestamp.rtc_timestamp_second,\
          ts_subsecond_ss, ts_subsecond_ts, ts_subsecond_hs);
}


void uart0_handler(void)
{
    uint8_t data = usart_data_receive(USART0);
    lwrb_write (&uart_rb, &data, 1);
}

