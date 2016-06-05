/*
 * Copyright (C) 2016 Sami Kujala <skujala@iki.fi>
 * 
 */

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/i2c.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/f4/nvic.h>
#include <libopencm3/stm32/f4/rcc.h>

#define I2C_EV1_SLAVE_RECEIVER     ((uint32_t)0x00020002)
#define I2C_EV1_SLAVE_TRANSMITTER  ((uint32_t)0x00060082)

#define I2C_EV2                    ((uint32_t)0x00020040)
#define I2C_EV3_BYTE_TRANSMITTED   ((uint32_t)0x00060084)
#define I2C_EV3_BYTE_TRANSMITTING  ((uint32_t)0x00060080)

#define I2C_EV3_2                  ((uint32_t)0x00000400)
#define I2C_EV4                    ((uint32_t)0x00000010)
#define I2C_EV5                    ((uint32_t)0x00030001)
#define I2C_EV6_MASTER_TRANSMITTER ((uint32_t)0x00070082)
#define I2C_EV6_MASTER_RECEIVER    ((uint32_t)0x00030002)

#define I2C_EV7                    ((uint32_t)0x00030040)
#define I2C_EV7_1
#define I2C_EV8                    ((uint32_t)0x00070080)
#define I2C_EV8_1                 
#define I2C_EV8_2                 ((uint32_t)0x00070084)
#define I2C_EV9                    ((uint32_t)0x00030008)


#define I2C_EV5_MASTER_MODE_STARTED \
  ((uint32_t) (I2C_SR1_SB) | (I2C_SR2_BUSY | I2C_SR2_MSL) << 16)

#define I2C_EV6_MASTER_TRANSMITTER_ADDR_SENT \
  ((uint32_t)(I2C_SR1_ADDR | I2C_SR1_TxE) | (I2C_SR2_TRA | I2C_SR2_BUSY | I2C_SR2_MSL) << 16)

#define I2C_EV8_MASTER_TRANSMITTING_BYTE \
  ((uint32_t)(I2C_SR1_TxE) | (I2C_SR2_TRA | I2C_SR2_BUSY | I2C_SR2_MSL) << 16)

#define I2C_EV8_2_MASTER_BYTE_TRANSMITTED \
  ((uint32_t)(I2C_SR1_TxE | I2C_SR1_BTF) | (I2C_SR2_TRA | I2C_SR2_BUSY | I2C_SR2_MSL) << 16)


typedef enum {
  SUCCESS,
  FAILURE
} result_t;


uint32_t last_event = 0;

uint8_t msg[5];
uint8_t *pmsg = &msg[0];
uint8_t msglen;

static volatile uint8_t error = 0;




static void nvic_setup(void)
{
  nvic_enable_irq(NVIC_I2C1_EV_IRQ);
  nvic_enable_irq(NVIC_I2C1_ER_IRQ);
}


static void rcc_setup(void)
{
  rcc_clock_setup_hse_3v3(&rcc_hse_8mhz_3v3[RCC_CLOCK_3V3_84MHZ]);
  
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
  
  /* Enable events and errors interrupts */
  //i2c_enable_interrupt(I2C1, I2C_CR2_ITERREN);
  i2c_enable_interrupt(I2C1, I2C_CR2_ITEVTEN);
  
  /* 
     NXP PCA9685 datasheet specifies maximum rise time to be 1000 ns, giving
     maximum rise time for I2C subsystem as specified by the STM32 datasheet:
     1000 ns / T_PCLK1 = 8 (+ 1) = _9_ 
   */
  i2c_set_trise(I2C1, 8 + 1);
  
  i2c_peripheral_enable(I2C1);
}


static void WaitForEvent(uint32_t event) 
{
  uint32_t j;
    
  while(last_event != event) {
    GPIOA_ODR ^= GPIO5|GPIO1;
    
    for(j=0;j<500000;j++)
      asm("nop");
  }    
}


void i2c1_er_isr(void)
{
  error = I2C1_SR1 & (I2C_SR1_AF | I2C_SR1_BERR | I2C_SR1_ARLO | I2C_SR1_OVR);
}

void i2c1_ev_isr(void)
{
  #define FLAG_MASK ((uint32_t)0x00FFFFFF)  /*<! I2C FLAG mask */
    
  uint32_t volatile flag1 = 0, flag2 = 0;  
    
  flag1 = I2C1_SR1;
  
  if(I2C1_SR1 & I2C_SR1_SB) {
    i2c_enable_interrupt(I2C1, I2C_CR2_ITBUFEN);
    GPIOA_ODR ^= GPIO5|GPIO1;
  }
  

  /* Only read SR2 if ADDR bit is set as per documentation */
  if(flag1 & I2C_SR1_ADDR) {
    flag2 = I2C1_SR2;
    flag2 = flag2 << 16; 
  }
    
  last_event = (flag1 | flag2) & FLAG_MASK;
}


int main(void)
{
  uint32_t j;
  
	rcc_setup();
  nvic_setup();
	gpio_setup();
	usart_setup();
	i2c_setup();

    usart_send_blocking(USART2, '.');
    
    i2c_send_start(I2C1);
    
    usart_send_blocking(USART2, '.');
    WaitForEvent(I2C_EV5_MASTER_MODE_STARTED); /* EV5 */

    usart_send_blocking(USART2, '.');
      
    i2c_send_7bit_address(I2C1, 0x60, 0);
    WaitForEvent(I2C_EV6_MASTER_TRANSMITTER_ADDR_SENT); /* EV6 -- address sent*/
    
    usart_send_blocking(USART2, '.');
        
    i2c_send_data(I2C1, 0x00); // MODE1
    WaitForEvent(I2C_EV8_2_MASTER_BYTE_TRANSMITTED);
    
    i2c_send_data(I2C1, 0x80); // RESET
    WaitForEvent(I2C_EV8_2_MASTER_BYTE_TRANSMITTED);
      
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
    
  while(1) {
      asm("nop");
  }
   
	return 0;
}