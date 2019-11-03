#include "stm32_F407.h"
#include "FreeRTOS.h"
#include "task.h"
#include "clib.h"

extern xTaskHandle xHandle_comm;
extern xTaskHandle task1;
extern xTaskHandle task2;
extern xTaskHandle task3;
extern xTaskHandle xHandle_led;
extern void command_prompt(void *pvParameters);


const uint16_t LEDS = GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
const uint16_t LED[4] = {GPIO_Pin_12, GPIO_Pin_13, GPIO_Pin_14, GPIO_Pin_15};

const portTickType xDelay = 1000 / portTICK_RATE_MS;

uint32_t iii = 0;
// uint32_t k = 0;



void init_led(void)
{
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);  // Enable AHB1 RCC clock 
    GPIO_InitTypeDef gpio;
    GPIO_StructInit(&gpio);
    gpio.GPIO_Mode = GPIO_Mode_OUT;
    gpio.GPIO_Pin = LEDS;
    GPIO_Init(GPIOD, &gpio);
}

void led_blink(void *pvParameters) { 
    uint32_t ii = 0;
    uint16_t i = 0;
    while (i < 4) {
        GPIO_ResetBits(GPIOD, LEDS);
        GPIO_SetBits(GPIOD, LED[i % 4]);
        ++i;
        for (ii = 0; ii < 10000000; ++ii);
        // vTaskDelay(xDelay);
    }
    GPIO_ResetBits(GPIOD, LEDS);
    vTaskDelete(xHandle_led);
    return;
}

void init_button(void)
{
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE); // Enable AHB1 RCC GPIOA

    GPIO_InitTypeDef gpio;
    GPIO_StructInit(&gpio);
    gpio.GPIO_Mode = GPIO_Mode_IN;
    gpio.GPIO_Pin = GPIO_Pin_0; // User botton at PA0
    GPIO_Init(GPIOA, &gpio);
}

void enable_button_interrupts(void)
{
    EXTI_InitTypeDef EXTI_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    /* Enable the SYSCFG clock.  GPIO_EXTILineConfig sets registers in
     * the SYSCFG.
     */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);

    /* Connect EXTI Line 0 to the button GPIO Pin */
    SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOA, EXTI_PinSource0);

    /* Configure the EXTI line to generate an interrupt when the button is
     * pressed.  The button pin is high when pressed, so it needs to trigger
     * when rising from low to high. */
    EXTI_InitStructure.EXTI_Line = EXTI_Line0;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);

    /* Enable and set Button EXTI Interrupt to the lowest priority */
    NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0F;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x0F;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

void init_rs232(void)
{
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);  
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
    GPIO_InitTypeDef gpio;
    
    //PC10 init
    gpio.GPIO_Pin = GPIO_Pin_2;
    gpio.GPIO_Mode = GPIO_Mode_AF; // PC10 is Tx pin
    gpio.GPIO_Speed = GPIO_Speed_100MHz;
    gpio.GPIO_OType = GPIO_OType_PP;
    gpio.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_USART2);
    GPIO_Init(GPIOA, &gpio);
    
    //PC11 init
    gpio.GPIO_Pin = GPIO_Pin_3;
    gpio.GPIO_Mode = GPIO_Mode_AF; // PC11 is Rx pin
    gpio.GPIO_Speed = GPIO_Speed_100MHz;
    gpio.GPIO_OType = GPIO_OType_PP;
    gpio.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_USART2);
    GPIO_Init(GPIOA, &gpio);

    USART_InitTypeDef usart2;
    USART_StructInit(&usart2);
    usart2.USART_BaudRate = 115200;
    USART_Init(USART2, &usart2);
}

void enable_rs232_interrupts(void)
{
    NVIC_InitTypeDef NVIC_InitStructure;

    /* Enable transmit and receive interrupts for the USART2. */
    USART_ITConfig(USART2, USART_IT_TXE, DISABLE);
    USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);

    /* Enable the USART2 IRQ in the NVIC module (so that the USART2 interrupt
     * handler is enabled). */
    NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

void enable_rs232(void)
{
    /* Enable the RS232 port. */
    USART_Cmd(USART2, ENABLE);
}

void delay(uint32_t ms) {
    ms *= 3360;
    while(ms--) {
        __NOP();
    }
}

void Task1( void* pvParameters )
{
	while(1){
        __NOP();
        __NOP();
        __NOP();
        ++iii;
	}
}


void Task2( void* pvParameters )
{
	while(1){
		vTaskDelay(1000 / portTICK_RATE_MS);
        fio_printf(1, "K:%d\n", iii);
        iii = 0;
	}
}

void Task3( void* pvParameters )
{
    vTaskDelay(10000 / portTICK_RATE_MS);

    fio_printf(1, "final K:%d\n", iii);

    vTaskDelete( task1 );
	vTaskDelete( task2 );
	vTaskDelete( task3 );
    // vTaskResume(xHandle_comm);
}