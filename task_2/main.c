#include "stm32f4xx.h"
#include "util.h"

volatile int ticks = 0;

void delayMs(int ms) { // delay in miliseconds
    ticks = 0;
    while(ticks < ms);
}

void SysTick_Handler(void) { // redefinition of SysTick_Handler
    ticks++;
}

void txUp() {
    GPIOG->BSRR |= GPIO_BSRR_BS14;
}

void txDown() {
    GPIOG->BSRR |= GPIO_BSRR_BR14;
}

void sendViaUsart(const char* msg) {

    for(int i = 0; msg[i] != '\0'; i++) {
        txDown();
        delayMs(1);
        for(int j = 0; j < 8; j++) {
            if((msg[i] >> j) & 1) { txUp(); }
            else txDown();
            delayMs(1);
        }
        txUp();
        delayMs(1);
    }

}

void UART6Config() {

    RCC->APB2ENR |= RCC_APB2ENR_USART6EN;  // enable USART6 clock, bit 5 on APB2ENR
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOGEN;  // enable GPIOG clock, bit 6 on AHB1ENR

    GPIOG->MODER &= ~(GPIO_MODER_MODER14_Msk);
    GPIOG->MODER |= GPIO_MODER_MODER14_0; //set PG14 to output mode (USART6_TX pin)
    GPIOG->MODER &= ~(GPIO_MODER_MODER9_Msk);
    GPIOG->MODER |= GPIO_MODER_MODER14_0; //set PG9 to output mode (USART6_RX pin)

    GPIOG->OSPEEDR &= ~(GPIO_OSPEEDR_OSPEED14_Msk|GPIO_OSPEEDR_OSPEED9_Msk);
    GPIOG->OSPEEDR |= (GPIO_OSPEEDR_OSPEED14_1|GPIO_OSPEEDR_OSPEED9_1); // Set USART6_TX and USART6_RX pinsto high speed mode (0b10)

}

char* msg = "Tony Pridybailo\n\r\0";

int main(void) {

    set_sysclk_max();
    UART6Config();
    SysTick_Config(SystemCoreClock/(820)); //configuring systick for delay function

    txUp();

    while(1) {

      	sendViaUsart(msg); //baudrate = 9600 baud
        delayMs(1000);

    }
    return 0;

}
