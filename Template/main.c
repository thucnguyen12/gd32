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

#define ARRAYSIZE 128

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
static const min_msg_t ping_min_msg = {
    .id = MIN_ID_PING_ESP_ALIVE,
    .len = 0,
    .payload = NULL
};

bool check_ping_esp_s = false;

void led_spark(void)
{
    static __IO uint32_t timingdelaylocal = 0U;

    if(timingdelaylocal){

        if(timingdelaylocal < 500U){
            gd_eval_led_on(LED1);
        }else{
            gd_eval_led_off(LED1);
        }

        timingdelaylocal--;
    }else{
        timingdelaylocal = 1000U;
    }
}

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
void min_rx_callback(void *min_context, min_msg_t *frame)
{
    switch (frame->id)
    {
    case MIN_ID_RECIEVE_SPI_FROM_GD32:
        /* code */
        check_ping_esp_s = true;
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
    gd_eval_led_init(LED1); 
    gd_eval_led_init(LED2); 
    gd_eval_led_init(LED3);
    gd_eval_com_init(EVAL_COM);
    gd_eval_key_init(KEY_WAKEUP, KEY_MODE_GPIO);
    
    //spi configuration
    spi_config ();
    spi_enable(SPI0);
    
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
    while(1){
        
        now = sys_get_ms();
        if (check_ping_esp_s)
        {
            check_ping_esp_s = false;
        }
        if (((now - last_time) > 10000) && (!check_ping_esp_s))
        {
            // reset esp32
            gpio_bit_reset(GPIO_ESP_EN_PORT, GPIO_ESP_EN_PIN);
            delay_1ms (50000);
            gpio_bit_set(GPIO_ESP_EN_PORT, GPIO_ESP_EN_PIN);
        }
        //feed data to min progress
        lwrb_read (&uart_rb, databuff, btr_m);
        min_rx_feed (&m_min_context, databuff, btr_m);
        if (spi_data_flag)
        {
            build_min_tx_data_from_spi(&min_data_buff, spi0_receive_array, ARRAYSIZE);
            send_min_data (&min_data_buff);
            memset (spi0_receive_array,'\0' ,ARRAYSIZE);
        }
        
        if(SET == gd_eval_key_state_get(KEY_WAKEUP)){
            gd_eval_led_on(LED2);
            delay_1ms(500);
            gd_eval_led_off(LED2);
            gd_eval_led_toggle(LED3);
        }
    }
}



void uart0_handler(void)
{
    uint16_t data = usart_data_receive(EVAL_COM);
    lwrb_write (&uart_rb, &data, 1);
}

