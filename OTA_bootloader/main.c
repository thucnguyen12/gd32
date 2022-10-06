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
#include "flash/flash.h"
#include "iap/iap.h"
/*!
    \brief      toggle the led every 500ms
    \param[in]  none
    \param[out] none
    \retval     none
*/




int main(void)
{
    board_hw_initialize();

    /* configure systick */
    systick_config();
    /* initilize the LEDs, USART and key */
//    gd_eval_led_init(LED1); 
//    gd_eval_led_init(LED2); 
//    gd_eval_led_init(LED3);
//    gd_eval_com_init(EVAL_COM);
//    gd_eval_key_init(KEY_WAKEUP, KEY_MODE_GPIO);
    
    //spi configuration
    nvic_irq_enable(SPI0_IRQn, 1);

    spi_enable(SPI0);
    
    if(RESET != rcu_flag_get(RCU_FLAG_WWDGTRST)){
    /* WWDGTRST flag set */
    gd_eval_led_on(LED1);
    rcu_all_reset_flag_clear();
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
    
    /* enable WWDGT clock */
    rcu_periph_clock_enable(RCU_WWDGT);
    /*
     *  set WWDGT clock = (PCLK1 (72MHz)/4096)/8 = 2197Hz (~455 us)
     *  set counter value to 127
     *  set window value to 80
     *  refresh window is: ~455 * (127-80)= 21.3ms < refresh window < ~455 * (127-63) =29.1ms.
     */
//    wwdgt_config(127,80,WWDGT_CFG_PSC_DIV8);
//    wwdgt_enable();
    __IO uint32_t UserDataAddr = FLASH_DATA_ADDR;//user data address, 4k in all	
     uint32_t LoaderKey = 0xAA55DD99;		
    __IO uint32_t UpdateFlag = *(__IO uint32_t*)UserDataAddr;		//update flag
    
    
    if(UpdateFlag != LoaderKey)
    {
        iap_load_app(FLASH_APP1_ADDR);//run FLASH APP
    }
    // if reach here we need update
    
    while(1){ 
        /* update WWDGT counter */
        wwdgt_counter_update(127);
        
    }
}



