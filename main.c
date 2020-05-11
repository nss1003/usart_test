/*
 * main.c
 *
 *  Created on: 02.04.2020
 *      Author: silvere
 */

#include <stdio.h>
#include <stdlib.h>

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>
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
		rcc_periph_clock_enable(RCC_GPIOB);
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
	/* Setup GPIO pins for USART3 transmit. */
	gpio_mode_setup(GPIOC, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO4);
	gpio_mode_setup(GPIOC, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO5);

	/* Setup USART3 TX and RX pin as alternate function. */
	gpio_set_af(GPIOC, GPIO_AF7, GPIO4);
	gpio_set_af(GPIOC, GPIO_AF7, GPIO5);

	//USART3 setup
	usart_set_baudrate(USART3, 115200);
	usart_set_databits(USART3, 8);
	usart_set_stopbits(USART3, USART_STOPBITS_1);
	usart_set_mode(USART3, USART_MODE_TX_RX);
	usart_set_parity(USART3, USART_PARITY_NONE);
	usart_set_flow_control(USART3, USART_FLOWCONTROL_NONE);

	/* Enable USART3 receive interrupt*/
	usart_enable_rx_interrupt(USART3);

	/* Finally enable USART3. */
	usart_enable(USART3);
}

static void gpio_setup(void)
{
	gpio_mode_setup(GPIOB, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO14);
}


char *recv;
static void put_s(char *c)
{
	while(*c != '\0'){
		usart_send_blocking(USART3, *c);
		c++;
	}
}

static char *get_s(void)
{
	int i = 0;
	char *recv = (char *)malloc(20);
	do{
		recv[i] = usart_recv_blocking(USART3);
		i++;

	}while(recv[i] != '\0' && i < 20);

	return recv;
}


int main(void)
{
	char *buffer = "Some string\r\n\0";
	clock_setup();
	uart_setup();
	gpio_setup();

	while (1) {

		put_s(buffer);
		recv = get_s();
		gpio_toggle(GPIOB, GPIO14);
		put_s(recv);

//		recv = usart_recv_blocking(USART3);
//		gpio_toggle(GPIOB, GPIO14);
//		//put_s(recv);
//		usart_send_blocking(USART3, recv);
//		msleep(2000);

		free(recv);
	}
	return 0;
}
