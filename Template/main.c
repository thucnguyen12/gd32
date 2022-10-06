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


#define LCD_MEASURE_X_CENTER(x) ((LCD_HORIZONTAL - x) / 2)
#define LCD_MEASURE_Y_CENTER(x) ((LCD_VERTICAL - x) / 2)
/*!
    \brief      toggle the led every 500ms
    \param[in]  none
    \param[out] none
    \retval     none
*/

lwrb_t uart_rb;
uint8_t uart_buffer[1024];
uint8_t min_txbuffer[1024];

uint8_t min_rx_buffer[1024];
min_context_t m_min_context;
min_frame_cfg_t m_min_setting;

uint8_t spi_header[2];

static const min_msg_t ping_min_msg = {
    .id = MIN_ID_PING_ESP_ALIVE,
    .len = 0,
    .payload = NULL
};

u8g2_t m_u8g2;

uint8_t u8g2_gpio_8080_update_and_delay(U8X8_UNUSED u8x8_t *u8x8,
										U8X8_UNUSED uint8_t msg,
										U8X8_UNUSED uint8_t arg_int,
										U8X8_UNUSED void *arg_ptr);

bool check_ping_esp_s = false;

// spi var
volatile uint32_t send_n = 0, receive_n = 0;
uint8_t spi0_send_array[ARRAYSIZE];
uint8_t spi0_receive_array[ARRAYSIZE];
uint8_t spi_data_flag = 0;

/*!
    \brief      configure the SPI peripheral
    \param[in]  none
    \param[out] none
    \retval     none
*/
void spi_config(void)
{
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
    return SysTick->VAL;
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
        break;
    case MIN_ID_SEND_SPI_FROM_ESP32:    
        memcpy(spi_data_buffer, frame->payload,frame->len);
        spi_send_data (spi_data_buffer, strlen ((char*)spi_data_buffer));
        break;
    case MIN_ID_SEND_HEARTBEAT_MSG:
            
        break;
    case MIN_ID_SEND_BEACON_MSG:
        
        break;
    case MIN_ID_SEND_KEY_CONFIG:
        
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

/*!
    \brief      main function
    \param[in]  none
    \param[out] none
    \retval     none
*/

int main(void)
{
    board_hw_initialize();
    lwrb_init(&uart_rb, uart_buffer, 1024);
    m_min_setting.get_ms = sys_get_ms;
    m_min_setting.last_rx_time = 0x00;
	m_min_setting.rx_callback = min_rx_callback;
	m_min_setting.timeout_not_seen_rx = 5000;
	m_min_setting.tx_byte = min_tx_byte;
	m_min_setting.use_timeout_method = 1;

    m_min_context.callback = &m_min_setting;
	m_min_context.rx_frame_payload_buf = min_rx_buffer;
	min_init_context(&m_min_context);
    
    /* configure systick */
    systick_config();
    /* initilize the LEDs, USART and key */
//    gd_eval_led_init(LED1); 
//    gd_eval_led_init(LED2); 
//    gd_eval_led_init(LED3);
//    gd_eval_com_init(EVAL_COM);
//    gd_eval_key_init(KEY_WAKEUP, KEY_MODE_GPIO);
    
    //spi configuration
    nvic_irq_enable (SPI0_IRQn, 1);
    spi_config ();
    spi_enable (SPI0);
    
    if(RESET != rcu_flag_get(RCU_FLAG_WWDGTRST)){
    /* WWDGTRST flag set */
    gd_eval_led_on(LED1);
    rcu_all_reset_flag_clear();
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
//    while(1);
    }
    /* print out the clock frequency of system, AHB, APB1 and APB2 */
    printf("\r\nCK_SYS is %d", rcu_clock_freq_get(CK_SYS));
    printf("\r\nCK_AHB is %d", rcu_clock_freq_get(CK_AHB));
    printf("\r\nCK_APB1 is %d", rcu_clock_freq_get(CK_APB1));
    printf("\r\nCK_APB2 is %d", rcu_clock_freq_get(CK_APB2));
    
    
    static uint32_t now;
    static uint32_t last_time = 0;
    uint8_t databuff[128];
    size_t btr_m;
    min_msg_t min_data_buff;
    
    /* enable WWDGT clock */
    rcu_periph_clock_enable(RCU_WWDGT);
    /*
     *  set WWDGT clock = (PCLK1 (72MHz)/4096)/8 = 2197Hz (~455 us)
     *  set counter value to 127
     *  set window value to 80
     *  refresh window is: ~455 * (127-80)= 21.3ms < refresh window < ~455 * (127-63) =29.1ms.
     */
    wwdgt_config(127,80,WWDGT_CFG_PSC_DIV8);
    wwdgt_enable();
    lcd_clr_screen();
    lcd_display_content ("lcd test");
    while(1){ 
        /* update WWDGT counter */
        wwdgt_counter_update(127);
        now = sys_get_ms();
        if (check_ping_esp_s)
        {
            check_ping_esp_s = false;
        }
        if (((now - last_time) > 5000) && (!check_ping_esp_s))
        {
            // reset esp32
            gpio_bit_reset(GPIO_ESP_EN_PORT, GPIO_ESP_EN_PIN);
            delay_1ms (5000);
            gpio_bit_set(GPIO_ESP_EN_PORT, GPIO_ESP_EN_PIN);
            continue;
            //restart loop
        }
        //feed data to min progress
        
        lwrb_read (&uart_rb, databuff, btr_m);
        min_rx_feed (&m_min_context, databuff, btr_m);
        //transmit data to ask for spi data
        //spi_send_data ((char *)"need you send data", strlen ("need you send data"));
        //example header
        spi_header[0] = 0x01;
        spi_header[1] = 0xFF;
        spi_send_data (spi_header, 2);
        //wait till there no data in register
                                                                                                                    
        if (spi_data_flag)//check whether there are spi data
        {
            build_min_tx_data_from_spi(&min_data_buff, spi0_receive_array, ARRAYSIZE);
            send_min_data (&min_data_buff);
            memset (spi0_receive_array,'\0' ,ARRAYSIZE);
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

void uart0_handler(void)
{
    uint16_t data = usart_data_receive(EVAL_COM);
    lwrb_write (&uart_rb, &data, 1);
}

