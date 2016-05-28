/*
 * This file is part of the libopencm3 project.
 *
 * Copyright (C) 2009 Uwe Hermann <uwe@hermann-uwe.de>
 * Copyright (C) 2011 Stephen Caudle <scaudle@doceme.com>
 * Modified by Fernando Cortes <fermando.corcam@gmail.com>
 * modified by Guillermo Rivera <memogrg@gmail.com>
 *
 * This library is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this library.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/i2c.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/f4/nvic.h>


static volatile uint8_t error = 0;


static void clock_setup(void)
{
//  rcc_clock_setup_hse_3v3(&hse_8mhz_3v3[CLOCK_3V3_84MHZ]);
  
        /* Enable GPIOD clock for LED & USARTs. */
        rcc_periph_clock_enable(RCC_GPIOA);
        rcc_periph_clock_enable(RCC_GPIOB);


        /* Enable clocks for USART2. */
        rcc_periph_clock_enable(RCC_USART2);
        
        /* Enable clocks from I2C1 */
      	rcc_periph_clock_enable(RCC_I2C1);
      	rcc_periph_clock_enable(RCC_GPIOB);
}

static void usart_setup(void)
{
        /* Setup USART2 parameters. */
        usart_set_baudrate(USART2, 115200);
        usart_set_databits(USART2, 8);
        usart_set_stopbits(USART2, USART_STOPBITS_1);
        usart_set_mode(USART2, USART_MODE_TX);
        usart_set_parity(USART2, USART_PARITY_NONE);
        usart_set_flow_control(USART2, USART_FLOWCONTROL_NONE);

        /* Finally enable the USART. */
        usart_enable(USART2);
}

static void gpio_setup(void)
{
        /* Setup GPIO pin GPIO5 on GPIO port A for LED. */
        gpio_mode_setup(GPIOA, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO5);

        /* Setup GPIO pins for USART2 transmit. */
        gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO2);

        /* Setup USART2 TX pin as alternate function. */
        gpio_set_af(GPIOA, GPIO_AF7, GPIO2);

      	/* Setup GPIO pin GPIO_USART2_TX/GPIO9 on GPIO port A for transmit. */
      	gpio_mode_setup(GPIOB, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO8 | GPIO9);
      	gpio_set_af(GPIOB, GPIO_AF4, GPIO8 | GPIO9);
}


static void i2c_setup(void)
{
	i2c_reset(I2C1);
  
	i2c_peripheral_disable(I2C1);

  i2c_set_standard_mode(I2C1);
  
  /*
     Set I2C bus master clock (PCLK1) to 8 MHz, and CCR divider to 128, 
     so that t_HIGH and t_LOW become 16 us, which is above the minimum 
     values as specified by the NXP PCA9685 datasheet, 4.7 and 4 us,  
     respectively => T_PCLK1 = 125 ns.
   */
  i2c_set_clock_frequency(I2C1, I2C_CR2_FREQ_8MHZ);
  i2c_set_ccr(I2C1, 128); 
  
  
  i2c_set_dutycycle(I2C1, I2C_CCR_DUTY_DIV2);
  
  /* Enable events interrupts */
  i2c_enable_interrupt(I2C1, I2C_CR2_ITERREN);
  
  /* 
     NXP PCA9685 datasheet specifies maximum rise time to be 1000 ns, giving
     maximum rise time for I2C subsystem as specified by the STM32 datasheet:
     1000 ns / T_PCLK1 = 8 (+ 1) = _9_ 
   */
  i2c_set_trise(I2C1, 8 + 1);


	i2c_peripheral_enable(I2C1);
}


static void WaitSR1FlagsSet (uint32_t Flags)
{
  while(((I2C1_SR1) & Flags) != Flags) {
    
    if (error != 0) {
      usart_send_blocking(USART2, '!');
    }
    
    asm("nop");
  }
}


void i2c1_er_isr(void)
{
  error = I2C1_SR1 & (I2C_SR1_AF | I2C_SR1_BERR | I2C_SR1_ARLO | I2C_SR1_OVR);
}

int main(void)
{
  uint8_t i;
  uint32_t j;
  
	clock_setup();
	gpio_setup();
	usart_setup();
	i2c_setup();

  while(1) {
//    for (i = 50; i < 80; i++) {
      i2c_send_start(I2C1);
      usart_send_blocking(USART2, '.');
      WaitSR1FlagsSet(I2C_SR1_SB);
      
      i2c_send_7bit_address(I2C1, 0x60, 0);
      usart_send_blocking(USART2, '.');
      WaitSR1FlagsSet(I2C_SR1_ADDR);
      
      i2c_send_stop(I2C1);
      usart_send_blocking(USART2, '.');


      
      // No errors mean slave responded
      if(error == 0) {
        usart_send_blocking(USART2, ' ');
        usart_send_blocking(USART2, 'O');
        usart_send_blocking(USART2, 'K');
      } else {
        usart_send_blocking(USART2, '!');
        usart_send_blocking(USART2, 'O');
        usart_send_blocking(USART2, 'K');        
      }
    
      // Otherwise wait
      for (j=0; j < 5000000; j++){
        asm("nop");
      }
  }
   
	return 0;
}