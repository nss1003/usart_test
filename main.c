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

//		rcc_periph_clock_enable(RCC_GPIOC);
		rcc_periph_clock_enable(RCC_GPIOA);
		rcc_periph_clock_enable(RCC_GPIOB);
		rcc_periph_clock_enable(RCC_UART4);

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

	/* Setup USART3 TX and RX pin as alternate function. */
	gpio_set_af(GPIOA, GPIO_AF8, GPIO0);
	gpio_set_af(GPIOA, GPIO_AF8, GPIO1);

	//UART4 setup
	nvic_enable_irq(NVIC_UART4_IRQ);
	usart_set_databits(UART4, 8);
	usart_set_baudrate(UART4, 115200);
	usart_set_stopbits(UART4, USART_STOPBITS_1);
	usart_set_mode(UART4, USART_MODE_TX_RX);
	usart_set_parity(UART4, USART_PARITY_NONE);
	usart_set_flow_control(UART4, USART_FLOWCONTROL_NONE);

	/*Enable RX interrupt*/
	usart_enable_rx_interrupt(UART4);

	/* Finally enable USART3. */
	usart_enable(UART4);
}


static void gpio_setup(void)
{
	gpio_mode_setup(GPIOB, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO14);
}

static volatile char recv[10];
int count = 0;

static void put_s(char *c)
{
	while(*c != '\0'){
		usart_send_blocking(UART4, *c);
		c++;
	}
}

void uart4_isr(void)
{
	int i;
	if(usart_get_flag(UART4, USART_ISR_RXNE)){
		for(i = 0; i < 6; i++){
			recv[i] = usart_recv_blocking(UART4);
			count = i;
		}

		gpio_toggle(GPIOB, GPIO14);
		//put_s(recv);
	}
}

int main(void)
{
	clock_setup();
	uart_setup();
	gpio_setup();

	int i;
	while (1){
		if(recv[5] == '6'){
			for(i = 0; i < 6; i++)
				usart_send_blocking(UART4, recv[i]);
		}
		msleep(2000);
	}
	return 0;
}
