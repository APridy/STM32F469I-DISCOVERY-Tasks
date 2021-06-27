#include "stm32f4xx.h"
#include <stdbool.h>

volatile int ticks = 0; //ticks for delay function

#define LED_1 1 << 2
#define LED_2 1 << 5
#define LED_3 1 << 6
#define LED_4 1 << 1
#define KEY_B1 1 << 0
#define KEY_B2 1 << 8
#define KEY_B3 1 << 9
#define KEY_B4 1 << 10
#define A 0b1100100010011000
#define B 0b1101100000010000
#define C 0b0001100000011000
#define D 0b1101100010000000
#define E 0b1001100000011000
#define F 0b1000100000011000
#define G 0b0101100000011000
#define H 0b1100100010010000
#define I 0b0000100000010000
#define J 0b0101100010000000
//#define K 0bgc0de000b00fa000
#define L 0b0001100000010000
//#define N 0bgc0de000b00fa000
#define O 0b0101100010011000
#define P 0b1000100010011000
//#define Q 0bgc0de000b00fa000
#define R 0b0000100000011000
#define S 0b1101000000011000
//#define T 0bgc0de000b00fa000
#define U 0b0101100010010000
//#define V 0bgc0de000b00fa000
//#define W 0bgc0de000b00fa000
//#define X 0bgc0de000b00fa000
#define Y 0b1101000010010000
#define Z 0b1001100010001000
#define ZERO 0b0101100010011000
#define ONE 0b0100000010000000
#define TWO 0b1001100010001000
#define THREE 0b1101000010001000
#define FOUR 0b1100000010010000
#define FIVE 0b1101000000011000
#define SIX 0b1101100000011000
#define SEVEN 0b0100000010001000
#define EIGHT 0b1101100010011000
#define NINE 0b1101000010011000

#define NUMBER_OF_LEDS 4
#define NUMBER_OF_KEY_PINS 4
#define NUMBER_OF_LETTERS 18
#define NUMBER_OF_DIGITS 10

#define key_on() ((GPIOG->IDR & GPIO_IDR_ID11) >> 11) //macro to define if any button is pressed

uint16_t led_array[NUMBER_OF_LEDS] = {LED_1, LED_2, LED_3, LED_4};
uint16_t key_array[NUMBER_OF_KEY_PINS] = {KEY_B3, KEY_B2, KEY_B1, KEY_B4};
uint16_t letter_array[NUMBER_OF_LETTERS] = {A, B, C, D, E, F, G, H, I, J, L, O, P, R, S, U, Y, Z};
uint16_t digit_array[NUMBER_OF_DIGITS] = {ZERO, ONE, TWO, THREE, FOUR, FIVE, SIX, SEVEN, EIGHT, NINE};

void delayMs(int ms) { // delay function
	ticks = 0;
	while(ticks<ms);
}

void SysTick_Handler(void) { // redefinition of SysTick_Handler
    ticks++;
}

void mosiUp() { //MOSI init 1
	GPIOB->BSRR |= GPIO_BSRR_BS15;
}

void mosiDown() { //MOSI init 0
	GPIOB->BSRR |= GPIO_BSRR_BR15;
}

void sckRise() { // rising edge SCK
	GPIOD->BSRR |= GPIO_BSRR_BR3;
  	GPIOD->BSRR |= GPIO_BSRR_BS3;
}

void rclkRise() { // rising edge RCLK
	GPIOG->BSRR |= GPIO_BSRR_BR10;
	GPIOG->BSRR |= GPIO_BSRR_BS10;
}

void registerPush1(bool val) { // pushes 1 bit to shift register

  	if(val) { mosiUp(); }
	else { mosiDown(); }
	sckRise();

}

void registerPush16(uint16_t val) {

   	for(int i = 15 ; i >= 0; i--) { //reads val bits and pushes them to shift register
		registerPush1((val >> i) & 1);
	}

}

void registerUpdate() { // copies shift register to non-shift register
	rclkRise();
}

void registerClear() {  // clears shift-register and non-shift register

	for(int i = 0; i < 16; i++) {
		registerPush1(0);
	}
	registerUpdate();

}

void blinkLed() { // for debugging purposes

  	GPIOA->BSRR |= GPIO_BSRR_BS7;
	delayMs(2);
	GPIOA->BSRR |= GPIO_BSRR_BR7;

}

void setSymbol(uint16_t symbol, uint16_t led_num) { // sets symbol on selected led

	if((led_num > NUMBER_OF_LEDS) || (led_num < 1)) { blinkLed(); return; }
	registerPush16(symbol | led_array[led_num - 1] | key_array[led_num - 1]);
	registerUpdate();

}

void blinkAllSymbols() { // sequentually lights up 1-2-3-4 leds on 4-7 display

	// if i+1 button is pressed i+1 led will not light up
	for(int i = 0; i < NUMBER_OF_LEDS; i++) {
		if(key_on()) { registerPush16(key_array[i]); registerUpdate(); }
		else setSymbol(letter_array[i],i + 1); // set symbol A on 1st led of 4-7 display
		delayMs(4);
	}

}

int main(void) {

	SysTick_Config(SystemCoreClock / 1000); //

  	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN; //enable used clocks
  	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
  	RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN;
  	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOGEN;

	GPIOB->MODER &= ~(GPIO_MODER_MODER15_Msk); // set needed pins to output mode
  	GPIOB->MODER |= GPIO_MODER_MODER15_0;
	GPIOG->MODER &= ~(GPIO_MODER_MODER13_Msk);
  	GPIOG->MODER |= GPIO_MODER_MODER13_0;
	GPIOD->MODER &= ~(GPIO_MODER_MODER3_Msk);
  	GPIOD->MODER |= GPIO_MODER_MODER3_0;
	GPIOG->MODER &= ~(GPIO_MODER_MODER10_Msk);
  	GPIOG->MODER |= GPIO_MODER_MODER10_0;
	GPIOA->MODER &= ~(GPIO_MODER_MODER7_Msk);
  	GPIOA->MODER |= GPIO_MODER_MODER7_0;

  	registerClear();
  	while(1) { // endless cycle of sequential lighting of 1-2-3-4 leds on 4-7 display
  		blinkAllSymbols();
  	}

	return 0;
}
