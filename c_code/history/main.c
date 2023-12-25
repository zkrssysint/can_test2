/*!
    \file    main.c
    \brief   CAN network communication
    
    \version 2023-08-17, V1.3.1, demo for GD32E50x
*/

/*
    Copyright (c) 2023, GigaDevice Semiconductor Inc.

    All rights reserved.

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

#include "gd32e50x.h"
#include <stdio.h>
#include "gd32e503z_eval.h"
#include "systick.h"

#define DEV_ADDRESS_ID	0x01
#define DEV_CMD_READ		0x03
#define DEV_CMD_WRITE		0x06
#define DEV_SET_ID			0x180180A0U
#define DEV_ACK_ID			0x1802A080U
#define DEV_DATA_ID1		0x1810A080U
#define DEV_DATA_ID2		0x1811A080U

#define BUFFER_SIZE	8
uint8_t rx_buffer[BUFFER_SIZE];
uint8_t tx_buffer[BUFFER_SIZE] = {0x00, 0x01, 0x02, 0x03, 0xff, 0xfe, 0xfd, 0xfc};
/* counter of transmit buffer and receive buffer */
__IO uint16_t  tx_count = 0, rx_count = 0;
/* size of transmit buffer and receive buffer */
uint32_t  rx_buffer_size = BUFFER_SIZE, tx_buffer_size = BUFFER_SIZE;
/* result of the transfer */
__IO ErrStatus transfer_status = ERROR; 

uint8_t Byte_read;
can_trasnmit_message_struct g_transmit_message;
can_receive_message_struct g_receive_message;
uint8_t holdingregister[0x28]={0};
FlagStatus can0_receive_flag, modbus_receive_flag, modbus_transmit_flag;
/*!
    \brief      configure GPIO
    \param[in]  none
    \param[out] none
    \retval     none
*/
void can_gpio_config(void)
{
    /* enable CAN clock */
    rcu_periph_clock_enable(RCU_CAN0);
    rcu_periph_clock_enable(RCU_GPIOD);
    rcu_periph_clock_enable(RCU_AF);
    
    /* configure CAN0 GPIO */
    gpio_init(GPIOD,GPIO_MODE_IPU,GPIO_OSPEED_50MHZ,GPIO_PIN_0);
    gpio_init(GPIOD,GPIO_MODE_AF_PP,GPIO_OSPEED_50MHZ,GPIO_PIN_1);
    
    gpio_pin_remap_config(GPIO_CAN0_FULL_REMAP,ENABLE);
}

/*!
    \brief      configure BSP
    \param[in]  none
    \param[out] none
    \retval     none
*/
void bsp_board_config(void)
{
    /* configure USART */
    gd_eval_com_init(EVAL_COM0);
        
    /* configure KEY_B key */
    gd_eval_key_init(KEY_B, KEY_MODE_GPIO);
    
    /* configure leds */
    gd_eval_led_init(LED2);
    gd_eval_led_off(LED2);

}

/*!
    \brief      initialize CAN function
    \param[in]  none
    \param[out] none
    \retval     none
*/
void can_config(void)
{
    can_parameter_struct can_parameter;
    can_filter_parameter_struct can_filter;

    can_struct_para_init(CAN_INIT_STRUCT, &can_parameter);
    /* initialize CAN register */
    can_deinit(CAN0);
    
    /* initialize CAN parameters */
    can_parameter.time_triggered = DISABLE;
    can_parameter.auto_bus_off_recovery = DISABLE;
    can_parameter.auto_wake_up = DISABLE;
    can_parameter.auto_retrans = DISABLE;
    can_parameter.rec_fifo_overwrite = DISABLE;
    can_parameter.trans_fifo_order = DISABLE;
    can_parameter.working_mode = CAN_NORMAL_MODE;
    /* configure CAN0 baud rate 125kbps */
    can_parameter.resync_jump_width = CAN_BT_SJW_1TQ;
    can_parameter.time_segment_1 = CAN_BT_BS1_5TQ;
    can_parameter.time_segment_2 = CAN_BT_BS2_2TQ;
    can_parameter.prescaler = 84U;
    /* initialize CAN */
    can_init(CAN0, &can_parameter);
    
    /* initialize filter */
    /* configure filter mode */
    can_filter.filter_mode = CAN_FILTERMODE_MASK;
    can_filter.filter_bits = CAN_FILTERBITS_32BIT;
    /* configure filter ID, only the 0xaabb expand ID can be received */
    can_filter.filter_list_high = 0x0000U;
    can_filter.filter_list_low = 0x0000U;
    /* configure filter mask */
    can_filter.filter_mask_high = 0x0000U;
    can_filter.filter_mask_low = 0x0000U;
    /* select receiver fifo */
    can_filter.filter_fifo_number = CAN_FIFO0;
    can_filter.filter_number = 0U;
    can_filter.filter_enable = ENABLE;
    can_filter_init(CAN0, &can_filter);
    
    /* configure CAN0 NVIC */
    nvic_irq_enable(USBD_LP_CAN0_RX0_IRQn, 0U, 0U);
    /* enable can receive FIFO0 not empty interrupt */
    can_interrupt_enable(CAN0, CAN_INTEN_RFNEIE0);
}

void communication_check(void)
{
    uint8_t i = 0U, j;
    /* CAN0 receive data correctly, the received data is printed */
    if(SET == can0_receive_flag){
        can0_receive_flag = RESET;
				if(g_receive_message.rx_efid == DEV_ACK_ID)
				{
						holdingregister[0x21] = g_receive_message.rx_data[0];
						holdingregister[0x23] = g_receive_message.rx_data[1];
						holdingregister[0x24] = g_receive_message.rx_data[2];
						holdingregister[0x25] = g_receive_message.rx_data[3];
						holdingregister[0x26] = g_receive_message.rx_data[4];
						holdingregister[0x27] = g_receive_message.rx_data[5];
				}
				else if(g_receive_message.rx_efid == DEV_DATA_ID1)
				{
						holdingregister[1] = g_receive_message.rx_data[0];
						holdingregister[3] = g_receive_message.rx_data[1];
						holdingregister[5] = g_receive_message.rx_data[2];
						holdingregister[7] = g_receive_message.rx_data[3];
						holdingregister[8] = g_receive_message.rx_data[4];
						holdingregister[9] = g_receive_message.rx_data[5];
						holdingregister[10] = g_receive_message.rx_data[6];
						holdingregister[11] = g_receive_message.rx_data[7];
				}
				else if(g_receive_message.rx_efid == DEV_DATA_ID2)
				{
						j = 0;
						for(i = 12; i < 20; i++)
						{
								holdingregister[i] = g_receive_message.rx_data[j++];
						}
				}
        gd_eval_led_toggle(LED2);
    }
}

uint16_t mbcrc16(uint8_t *buf, uint8_t len)
{
		unsigned int crc = 0xFFFF;
		for (int pos = 0; pos < len; pos++)
		{
				crc ^= (unsigned int)buf[pos]; // XOR byte into least sig. byte of crc
				for (int i = 8; i != 0; i--)   // Loop over each bit
				{
						if ((crc & 0x0001) != 0)   // If the LSB is set
						{
								crc >>= 1; // Shift right and XOR 0xA001
								crc ^= 0xA001;
						}
						else // LSB is not set
						{
								crc >>= 1;    // Just shift right
						}
				}
		}
		//Swap low and high byte
		crc = ((crc & 0x00ff) << 8) | ((crc & 0xff00) >> 8);
		return crc;
}

/*!
    \brief      main function
    \param[in]  none
    \param[out] none
    \retval     none
*/
int main(void)
{ 
    uint8_t i = 0, j, crc_recv, crc_calc;
    /* configure NVIC and systick */
    nvic_irq_enable(USART0_IRQn, 0, 0);
    /* configure systick */
    systick_config();
    /* configure board */
    bsp_board_config();
    /* configure GPIO */
    can_gpio_config();
    /* initialize CAN and filter */
    can_config();

    printf("\r\n communication test CAN0, please press KEY_B key to start! \r\n");
    
    /* initialize transmit message */
    can_struct_para_init(CAN_TX_MESSAGE_STRUCT, &g_transmit_message);
    g_transmit_message.tx_sfid = 0x00U;
    g_transmit_message.tx_efid = 0x00U;
    g_transmit_message.tx_ft = CAN_FT_DATA;
    g_transmit_message.tx_ff = CAN_FF_EXTENDED;
    g_transmit_message.tx_dlen = 8U;
    /* initialize receive message */
    can_struct_para_init(CAN_RX_MESSAGE_STRUCT, &g_receive_message);
		
		modbus_receive_flag = 0;		
		modbus_transmit_flag = 0;
    
    while(1)
		{
			/* enable USART RBNE interrupt */
			usart_interrupt_enable(EVAL_COM0, USART_INT_RBNE);
			/* wait until USART receive the rx_buffer */
			//while(modbus_receive_flag != 1);
			//modbus_receive_flag = 0;
			while(rx_count < rx_buffer_size);
			//if(modbus_receive_flag == 1)
			//{
					modbus_transmit_flag = 1;
					delay_1ms(1);
			//}
			rx_count = 0;
		
			if(rx_buffer[0] == DEV_ADDRESS_ID)
			{
					crc_recv = mbcrc16(rx_buffer, 6);
					crc_calc = (uint16_t)(rx_buffer[7] << 8) & rx_buffer[6];
					if(crc_recv == crc_calc)
					{
							if(rx_buffer[1] == DEV_CMD_READ)
							{
									tx_buffer[0] = DEV_ADDRESS_ID;
									tx_buffer[1] = DEV_CMD_READ;
									j = rx_buffer[3] << 1;
									Byte_read = rx_buffer[5] << 1;
									tx_buffer[2] = Byte_read;
									for(i = 3; i < Byte_read + 3; i = i + 2)
									{
											tx_buffer[i] = holdingregister[j++];
											tx_buffer[i+1] = holdingregister[j++];											
									}
									mbcrc16(tx_buffer, Byte_read + 3);
							}
					}
			}
			if(modbus_transmit_flag == 1)
			{
					//usart_flag_clear(EVAL_COM0, USART_FLAG_TC);
					/* enable USART TBE interrupt */  
					usart_interrupt_enable(EVAL_COM0, USART_INT_TBE);
					/* wait until USART send the tx_buffer */
					//while(modbus_transmit_flag == 1);
					modbus_transmit_flag = 0;
					while(tx_count < tx_buffer_size);

					while (RESET == usart_flag_get(EVAL_COM0, USART_FLAG_TC));
					tx_count = 0;		
			}		
			if(can0_receive_flag == SET)
			{
			}
			
			if(0)
			{
					i = 0;
					//if((command_send.mode == 1 || command_send.mode == 2) && (command_send.power == 0 || command_send.power == 1))
					if(1)
					{
							g_transmit_message.tx_efid = DEV_SET_ID;
							g_transmit_message.tx_data[6] = 0;
							g_transmit_message.tx_data[7] = 0;
							/* transmit message */
							can_message_transmit(CAN0, &g_transmit_message);
					}
					i++;
    }}
}

/* retarget the C library printf function to the usart */
int fputc(int ch, FILE *f)
{
    usart_data_transmit(EVAL_COM0, (uint8_t)ch);
    while (RESET == usart_flag_get(EVAL_COM0, USART_FLAG_TBE));
    return ch;
}
