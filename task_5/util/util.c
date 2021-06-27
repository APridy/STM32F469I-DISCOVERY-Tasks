#include "stm32f4xx.h"
#include "util.h"

void set_sysclk_max(void)
{   //FLASH init
    FLASH->ACR &= ~FLASH_ACR_LATENCY;
    FLASH->ACR |= FLASH_ACR_LATENCY_5WS;
    FLASH->ACR |= FLASH_ACR_ICEN;
    FLASH->ACR |= FLASH_ACR_DCEN;
    FLASH->ACR |= FLASH_ACR_PRFTEN;

    //HSE on
    RCC->CR |= RCC_CR_HSEON;
    while(!(RCC->CR & RCC_CR_HSERDY));

    //HSI trim and on
    RCC->CR &= ~RCC_CR_HSITRIM;
    RCC->CR |= 16 << RCC_CR_HSITRIM_Pos;
    RCC->CR |= RCC_CR_HSION;
    while(!(RCC->CR & RCC_CR_HSIRDY));

    //PLL init
    RCC->PLLCFGR &= ~(RCC_PLLCFGR_PLLSRC | RCC_PLLCFGR_PLLM | RCC_PLLCFGR_PLLN);
    RCC->PLLCFGR |= RCC_PLLCFGR_PLLSRC_HSE
                  | RCC_PLLCFGR_PLLM_3
                  | (360 << RCC_PLLCFGR_PLLN_Pos);

    RCC->PLLCFGR &= ~RCC_PLLCFGR_PLLR;

    //PLL on
    RCC->CR |= RCC_CR_PLLON;
    while(!(RCC->CR & RCC_CR_PLLRDY));

    //AHB clock
    RCC->CFGR &= ~RCC_CFGR_HPRE;
    RCC->CFGR |= RCC_CFGR_HPRE_DIV1;
    //APB1 clock
    RCC->CFGR &= ~RCC_CFGR_PPRE1;
    RCC->CFGR |= RCC_CFGR_PPRE1_DIV4;
    //APB2 clock
    RCC->CFGR &= ~RCC_CFGR_PPRE2;
    RCC->CFGR |= RCC_CFGR_PPRE2_DIV2;
    //System clock (SysClk)
    RCC->CFGR &= ~RCC_CFGR_SW;
    RCC->CFGR |= RCC_CFGR_SW_PLL;
    while((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL);

#ifdef MCO1
    //MCO1 output is PLLCLK / 4 = 45MHz
    RCC->CFGR &= ~RCC_CFGR_MCO1PRE;
    RCC->CFGR |= RCC_CFGR_MCO1PRE_2 | RCC_CFGR_MCO1PRE_1;
    RCC->CFGR |= RCC_CFGR_MCO1;

    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;

    GPIOA->MODER |= GPIO_MODER_MODER8_1;
    GPIOA->AFR[1] &= ~(1<<GPIO_AFRH_AFRH0);
    GPIOA->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR8;
#endif
}

void usart3_init(void)
{
    RCC->APB1ENR |= RCC_APB1ENR_USART3EN;
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;

    GPIOB->MODER &= ~(GPIO_MODER_MODER10 | GPIO_MODER_MODER11);
    GPIOB->MODER |= GPIO_MODER_MODER10_1 | GPIO_MODER_MODER11_1;

    GPIOB->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR10;
    GPIOB->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR11;

    GPIOB->PUPDR |= GPIO_PUPDR_PUPDR10_0 | GPIO_PUPDR_PUPDR11_0;

    //AF7
    GPIOB->AFR[1] |= GPIO_AFRH_AFSEL10 | GPIO_AFRH_AFSEL11;
    GPIOB->AFR[1] &= ~(GPIO_AFRH_AFSEL10_3 | GPIO_AFRH_AFSEL11_3);

    USART3->BRR = (45000000 / 115200);
    USART3->CR1 = USART_CR1_TE | USART_CR1_RE;
    USART3->CR1 |= USART_CR1_UE;
}

void usart3_putchar(char ch)
{
    USART3->DR = ch;
    while(!(USART3->SR & USART_SR_TC));
}

void usart3_puts(char *s)
{
    while(*s) {
        usart3_putchar(*s++);
    }
}

void delay_ms(int ms)
{
    for (int i = 0; i < 10000 * ms; i++) {
        asm("nop");
    }
}
