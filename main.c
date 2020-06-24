/*
 * main.c
 *
 *  Created on: 02.04.2020
 *      Author: silvere
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>


#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/flash.h>

#include "systick.h"


#define STM32L4

static void clock_setup(void)
{
	/* FIXME - this should eventually become a clock struct helper setup */
		rcc_osc_on(RCC_HSI16);

		flash_prefetch_enable();
		flash_set_ws(4);
		flash_dcache_enable();
		flash_icache_enable();
		/* 16MHz / 4 = > 4 * 40 = 160MHz VCO => 80MHz main pll  */
		rcc_set_main_pll(RCC_PLLCFGR_PLLSRC_HSI16, 4, 40,
				0, 0, RCC_PLLCFGR_PLLR_DIV2);
		rcc_osc_on(RCC_PLL);

		rcc_periph_clock_enable(RCC_GPIOC);
		rcc_periph_clock_enable(RCC_GPIOA);
		rcc_periph_clock_enable(RCC_GPIOB);
		rcc_periph_clock_enable(RCC_UART4);
		rcc_periph_clock_enable(RCC_USART3);

		rcc_set_sysclk_source(RCC_CFGR_SW_PLL); /* careful with the param here! */
		rcc_wait_for_sysclk_status(RCC_PLL);

		/* FIXME - eventually handled internally */
		rcc_ahb_frequency = 80e6;
		rcc_apb1_frequency = 80e6;
		rcc_apb2_frequency = 80e6;
}


static void uart_setup(void)
{
	/* Setup GPIO pins for UART4 transmit. */
	gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO0);
	gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO1);

	gpio_mode_setup(GPIOC, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO4);
	gpio_mode_setup(GPIOC, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO5);

	/* Setup USART3 TX and RX pin as alternate function. */
	gpio_set_af(GPIOA, GPIO_AF8, GPIO0);
	gpio_set_af(GPIOA, GPIO_AF8, GPIO1);

	gpio_set_af(GPIOC, GPIO_AF7, GPIO5);
	gpio_set_af(GPIOC, GPIO_AF7, GPIO4);

	//UART4 setup
	nvic_enable_irq(NVIC_UART4_IRQ);
	usart_set_databits(UART4, 8);
	usart_set_baudrate(UART4, 115200);
	usart_set_stopbits(UART4, USART_STOPBITS_1);
	usart_set_mode(UART4, USART_MODE_TX_RX);
	usart_set_parity(UART4, USART_PARITY_NONE);
	usart_set_flow_control(UART4, USART_FLOWCONTROL_NONE);

	//USART3 setup for printf commands
	usart_set_baudrate(USART3, 115200);
	usart_set_databits(USART3, 8);
	usart_set_stopbits(USART3, USART_STOPBITS_1);
	usart_set_mode(USART3, USART_MODE_TX_RX);
	usart_set_parity(USART3, USART_PARITY_NONE);
	usart_set_flow_control(USART3, USART_FLOWCONTROL_NONE);

	/*Enable RX interrupt*/
	usart_enable_rx_interrupt(UART4);

	/* Finally enable USART3. */
	usart_enable(UART4);
	usart_enable(USART3);
}


static void gpio_setup(void)
{
	gpio_mode_setup(GPIOB, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO14);
	gpio_mode_setup(GPIOA, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO5);
}

static volatile char recv[30];
static volatile int flag = 0;
int j = 0;
int joined = 0;

void uart4_isr(void)
{
	if(usart_get_flag(UART4, USART_ISR_RXNE)){
		/* Get downlink
		* EX: +RCV=2,1,1
		* The gateway send packet to module for APP port 2
		* The packet's payload size is 1
		* The payload data in Hex-format string
		*/

		recv[j] = usart_recv(UART4);
		j++;

		if(j == 11){
			nvic_disable_irq(NVIC_UART4_IRQ);
			usart_disable_rx_interrupt(UART4);
			usart_disable(UART4);
			j = 0;
			flag = 1;
		}
	}
}


static char *get_downlink(void){
	char *rcv_payload = (char *)malloc(64 * sizeof(char));
	int i;

	for(i = 0; i < 2; i++){
		rcv_payload[i] = recv[9+i];
	}
	return rcv_payload;
}

static void put_s(char *c)
{
	while(*c != '\0'){
		usart_send_blocking(USART3, *c);
		c++;
	}
}

int main(void)
{
	clock_setup();
	systick_ms_setup();
	uart_setup();
	gpio_setup();

	int i;

	while (1){

		if(flag){
			put_s("ersfdg\r\n");
			char *data = get_downlink();
			put_s(recv);

			put_s("\r\n");
			put_s(data);

			if(strcmp(data, "11") == 0){

				gpio_toggle(GPIOB, GPIO14);
				msleep(500);
				gpio_toggle(GPIOB, GPIO14);
				msleep(500);

				gpio_toggle(GPIOA, GPIO5);
				msleep(500);
				gpio_toggle(GPIOA, GPIO5);
				msleep(500);
			}
			else if(strcmp(data, "10") == 0){

				gpio_clear(GPIOA, GPIO5);

				gpio_toggle(GPIOB, GPIO14);
				msleep(500);
				gpio_toggle(GPIOB, GPIO14);
				msleep(500);

			}
			else if(strcmp(data, "01") == 0){

				gpio_clear(GPIOB, GPIO14);
				gpio_toggle(GPIOA, GPIO5);
				msleep(500);
				gpio_toggle(GPIOA, GPIO5);
				msleep(500);
			}
			else if(strcmp(data, "00") == 0){
				gpio_clear(GPIOB, GPIO14);
				gpio_clear(GPIOA, GPIO5);

			}
			else
			{
				put_s("Don't match\r\n");
			}

			free(data);

			usart_enable(UART4);
			nvic_enable_irq(NVIC_UART4_IRQ);
			usart_enable_rx_interrupt(UART4);
			put_s("END\r\n");
		}
	}
	return 0;
}
