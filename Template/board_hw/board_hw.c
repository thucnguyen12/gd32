#include "board_hw.h"
#include "gd32e23x.h"
#include "gd32e23x_usart.h"

//#include "app_debug.h"
#include "main.h"
#include "math.h"

#define RTC_BKP_VALUE    0x32F0

#ifndef __ALIGNED
#define _align
#endif


static void config_led_pwm(void);
void board_hw_initialize(void)
{
    //nvic_irq_enable(LVD_IRQn, 1);
    /* RCC configurations */    
    rcu_periph_clock_enable(RCU_GPIOA);
    rcu_periph_clock_enable(RCU_GPIOB);
    rcu_periph_clock_enable(RCU_GPIOC);
    rcu_periph_clock_enable(RCU_GPIOF);
    //rcu_periph_clock_enable(RCU_PMU);
    
    	/* Init io */
	// Step 1 All LCD Output IO
    gpio_mode_set(GPIO_EN_PWR_LCD_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_PULLUP, GPIO_EN_PWR_LCD_PIN);
    gpio_output_options_set(GPIO_EN_PWR_LCD_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, GPIO_EN_PWR_LCD_PIN);
    gpio_bit_set(GPIO_EN_PWR_LCD_PORT, GPIO_EN_PWR_LCD_PIN);
    
	gpio_mode_set(GPIO_LCD_DI_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_PULLUP, GPIO_LCD_DI_PIN);
    gpio_output_options_set(GPIO_LCD_DI_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, GPIO_LCD_DI_PIN);
    gpio_bit_reset(GPIO_LCD_DI_PORT, GPIO_LCD_DI_PIN);
    
	gpio_mode_set(GPIO_LCD_RW_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_PULLUP, GPIO_LCD_RW_PIN);
    gpio_output_options_set(GPIO_LCD_RW_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, GPIO_LCD_RW_PIN);
    gpio_bit_reset(GPIO_LCD_RW_PORT, GPIO_LCD_RW_PIN);
    
    gpio_mode_set(GPIO_LCD_EN_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_PULLUP, GPIO_LCD_EN_PIN);
    gpio_output_options_set(GPIO_LCD_EN_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, GPIO_LCD_EN_PIN);
    gpio_bit_reset(GPIO_LCD_EN_PORT, GPIO_LCD_EN_PIN);
    
    gpio_mode_set(GPIO_LCD_D0_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_PULLUP, GPIO_LCD_D0_PIN);
    gpio_output_options_set(GPIO_LCD_D0_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, GPIO_LCD_D0_PIN);
    gpio_bit_reset(GPIO_LCD_D0_PORT, GPIO_LCD_D0_PIN);
    
    gpio_mode_set(GPIO_LCD_D1_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_PULLUP, GPIO_LCD_D1_PIN);
    gpio_output_options_set(GPIO_LCD_D1_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, GPIO_LCD_D1_PIN);
    gpio_bit_reset(GPIO_LCD_D1_PORT, GPIO_LCD_D1_PIN);
    
    gpio_mode_set(GPIO_LCD_D2_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_PULLUP, GPIO_LCD_D2_PIN);
    gpio_output_options_set(GPIO_LCD_D2_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, GPIO_LCD_D2_PIN);
    gpio_bit_reset(GPIO_LCD_D2_PORT, GPIO_LCD_D2_PIN);
    
    gpio_mode_set(GPIO_LCD_D3_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_PULLUP, GPIO_LCD_D3_PIN);
    gpio_output_options_set(GPIO_LCD_D3_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, GPIO_LCD_D3_PIN);
    gpio_bit_reset(GPIO_LCD_D3_PORT, GPIO_LCD_D3_PIN);
    
    gpio_mode_set(GPIO_LCD_D4_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_PULLUP, GPIO_LCD_D4_PIN);
    gpio_output_options_set(GPIO_LCD_D4_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, GPIO_LCD_D4_PIN);
    gpio_bit_reset(GPIO_LCD_D4_PORT, GPIO_LCD_D4_PIN);
    
    gpio_mode_set(GPIO_LCD_D5_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_PULLUP, GPIO_LCD_D5_PIN);
    gpio_output_options_set(GPIO_LCD_D5_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, GPIO_LCD_D5_PIN);
    gpio_bit_reset(GPIO_LCD_D5_PORT, GPIO_LCD_D5_PIN);
    
    gpio_mode_set(GPIO_LCD_D6_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_PULLUP, GPIO_LCD_D6_PIN);
    gpio_output_options_set(GPIO_LCD_D6_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, GPIO_LCD_D6_PIN);
    gpio_bit_reset(GPIO_LCD_D6_PORT, GPIO_LCD_D6_PIN);
    
    gpio_mode_set(GPIO_LCD_D7_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_PULLUP, GPIO_LCD_D7_PIN);
    gpio_output_options_set(GPIO_LCD_D7_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, GPIO_LCD_D7_PIN);
    gpio_bit_reset(GPIO_LCD_D7_PORT, GPIO_LCD_D7_PIN);
    
    gpio_mode_set(GPIO_LCD_RST_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_PULLUP, GPIO_LCD_RST_PIN);
    gpio_output_options_set(GPIO_LCD_RST_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, GPIO_LCD_RST_PIN);
    gpio_bit_reset(GPIO_LCD_RST_PORT, GPIO_LCD_RST_PIN);
    
    gpio_mode_set(GPIO_LCD_PWM_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_PULLUP, GPIO_LCD_PWM_PIN);
    gpio_output_options_set(GPIO_LCD_PWM_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, GPIO_LCD_PWM_PIN);
    gpio_bit_reset(GPIO_LCD_PWM_PORT, GPIO_LCD_PWM_PIN);
    
    //GPIO 
//    gpio_mode_set (GPIO_ESP_EN_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_PULLUP ,GPIO_ESP_EN_PIN);
//    gpio_output_options_set(GPIO_ESP_EN_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, GPIO_ESP_EN_PIN);
//    gpio_bit_set(GPIO_ESP_EN_PORT, GPIO_ESP_EN_PIN);
    
    //spi gpio
//    gpio_mode_set(GPIO_SPI0_SCK_PORT, GPIO_MODE_AF, GPIO_PUPD_PULLUP, GPIO_SPI0_SCK_PIN);
//    gpio_output_options_set(GPIO_SPI0_SCK_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, GPIO_SPI0_SCK_PIN);
//    
//    gpio_mode_set(GPIO_SPI0_MISO_PORT, GPIO_MODE_AF, GPIO_PUPD_PULLUP, GPIO_SPI0_MISO_PIN);
//    gpio_output_options_set(GPIO_SPI0_MISO_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, GPIO_SPI0_MISO_PIN);
//    
//    gpio_mode_set(GPIO_SPI0_MOSI_PORT, GPIO_MODE_AF, GPIO_PUPD_PULLUP, GPIO_SPI0_MOSI_PIN);
//    gpio_output_options_set(GPIO_SPI0_MOSI_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, GPIO_SPI0_MOSI_PIN);
    
//    gpio_af_set(GPIO_SPI0_SCK_PORT, GPIO_AF_0, GPIO_SPI0_SCK_PIN);
//    gpio_af_set(GPIO_SPI0_MISO_PORT, GPIO_AF_0, GPIO_SPI0_MISO_PIN);
//    gpio_af_set(GPIO_SPI0_MOSI_PORT, GPIO_AF_0, GPIO_SPI0_MOSI_PIN);
    
    //spi cs
//    gpio_mode_set(GPIO_NRF_CS_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_PULLUP, GPIO_NRF_CS_PIN);
//    gpio_output_options_set(GPIO_NRF_CS_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, GPIO_NRF_CS_PIN);
//    
//    //lora
//    gpio_mode_set(GPIO_LORA_CS_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO_LORA_CS_PIN);
//    gpio_output_options_set(GPIO_LORA_CS_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, GPIO_LORA_CS_PIN);
//    
//    gpio_mode_set(GPIO_LORA_IO4_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO_LORA_IO4_PIN);
//    gpio_output_options_set(GPIO_LORA_IO4_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, GPIO_LORA_IO4_PIN);
//    gpio_mode_set(GPIO_LORA_EN_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_PULLUP, GPIO_LORA_EN_PIN);
//    gpio_output_options_set(GPIO_LORA_EN_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, GPIO_LORA_EN_PIN);
//    
//    //zigbee
//    gpio_mode_set(GPIO_ZIGBEE_CS_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO_ZIGBEE_CS_PIN);
//    gpio_output_options_set(GPIO_ZIGBEE_CS_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, GPIO_ZIGBEE_CS_PIN);
//    
    //uart config
//    usart_deinit(USART0);
//    rcu_periph_clock_enable(RCU_USART0);
//    
//    gpio_af_set(GPIO_UART_PORT, GPIO_AF_1, GPIO_ESP_RX_PIN);
//    gpio_af_set(GPIO_UART_PORT, GPIO_AF_1, GPIO_ESP_TX_PIN);
//    
//    gpio_mode_set(GPIO_UART_PORT, GPIO_MODE_AF, GPIO_PUPD_PULLUP, GPIO_ESP_RX_PIN);
//    gpio_output_options_set(GPIO_UART_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, GPIO_ESP_RX_PIN);
//    
//    gpio_mode_set(GPIO_UART_PORT, GPIO_MODE_AF, GPIO_PUPD_PULLUP, GPIO_ESP_TX_PIN);
//    gpio_output_options_set(GPIO_UART_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, GPIO_ESP_TX_PIN);
//    
//    
//    usart_baudrate_set(USART0, 115200U);
//    usart_receive_config(USART0, USART_RECEIVE_ENABLE);
//    usart_transmit_config(USART0, USART_TRANSMIT_ENABLE);

//    usart_enable(USART0);
//    usart_interrupt_enable(USART0, USART_INT_RBNE);
//	nvic_irq_enable(USART0_IRQn, 1);
//    
//    //rs232 config
//    rcu_periph_clock_enable(RCU_USART1);
//    gpio_af_set(GPIO_RS232_PORT, GPIO_AF_1, GPIO_RS232_RX_PIN);
//    gpio_af_set(GPIO_RS232_PORT, GPIO_AF_1, GPIO_RS232_TX_PIN);
//    gpio_mode_set(GPIO_RS232_PORT, GPIO_MODE_AF, GPIO_PUPD_PULLUP, GPIO_RS232_RX_PIN);
//    gpio_output_options_set(GPIO_RS232_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, GPIO_RS232_RX_PIN);
//    gpio_mode_set(GPIO_RS232_PORT, GPIO_MODE_AF, GPIO_PUPD_PULLUP, GPIO_RS232_TX_PIN);
//    gpio_output_options_set(GPIO_RS232_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, GPIO_RS232_TX_PIN);
//    usart_deinit(USART1);
//    usart_baudrate_set(USART1, 115200U);
//    usart_receive_config(USART1, USART_RECEIVE_ENABLE);
//    usart_transmit_config(USART1, USART_TRANSMIT_ENABLE);

//    usart_enable(USART1);
//    usart_interrupt_enable(USART1, USART_INT_RBNE);
//	// usart_interrupt_enable(USART1, USART_INT_TBE);
//	nvic_irq_enable(USART1_IRQn, 0);
    
}

void board_hw_reset(void)
{
    NVIC_SystemReset();
}

//__align(4) uint16_t m_adc_value[1];

