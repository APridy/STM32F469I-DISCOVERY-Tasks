#include "stm32f4xx.h"
#include "util.h"
#include "stdbool.h"

#define LED_1 1 << 2
#define LED_2 1 << 5
#define LED_3 1 << 6
#define LED_4 1 << 1

#define KEY_B1 1 << 0
#define KEY_B2 1 << 8
#define KEY_B3 1 << 9
#define KEY_B4 1 << 10
#define KEY_EVERY KEY_B1 | KEY_B2 | KEY_B3 | KEY_B4

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
#define NUMBER_OF_DIGITS 10

#define MAX_ITOA_OUTPUT_LENGTH 12
#define DUMMY_WORD 0xFFFF
#define EEPROM_ADDRESS_LENGTH 10

#define key_on() ((GPIOG->IDR & GPIO_IDR_ID11) >> 11) //macro to define if any button is pressed

const char* errorInvalidMode = "ERROR: Invalid memory output mode\n\r\0";
const char* errorInvalidValue = "ERROR: Value should vary between 0 and 9999\n\r\0";

enum EEPROMWriteMode {
	BINARY = 1,
	ASCII = 2
};

enum opCode {
	OPCODE_WRITE = 0b101,
	OPCODE_WEN = 0b100,
};

enum dummyAddress {
	DUMMYADDRESS_WEN = 3 << 8,
};

uint16_t led_array[NUMBER_OF_LEDS] = {LED_1, LED_2, LED_3, LED_4};
uint16_t key_array[NUMBER_OF_KEY_PINS] = {KEY_B3, KEY_B2, KEY_B1, KEY_B4};
uint16_t digit_array[NUMBER_OF_DIGITS] = {ZERO, ONE, TWO, THREE, FOUR, FIVE, SIX, SEVEN, EIGHT, NINE};

volatile int ticks = 0;
volatile int sTicks = 0;
uint16_t displayValue = 9999;
uint16_t displayValueUsart3 = 0;
uint16_t displayValueUsart3Symbols = 0;

void delayMs(unsigned int ms) { // delay in miliseconds

	ticks = 0;
	while(ticks<ms*1000);

}

void delayMcs(int mcs) { // delay in microseconds

    ticks = 0;
    while(ticks<mcs);

}

void blinkLed() { // for debugging purposes

  	GPIOA->BSRR |= GPIO_BSRR_BS7;
	delayMs(2);
	GPIOA->BSRR |= GPIO_BSRR_BR7;

}

void sendViaUsart3(const char* msg) {

	for(int i = 0; msg[i] != '\0'; i++) {

		USART3->DR = msg[i]; // put one character in data register
		while(!(USART3->SR & (USART_SR_TC))); // wait until transmit is over
		USART3->SR &= ~(USART_SR_TC); // clear transmit value

	}

}

void sendViaSPI6(uint16_t value) {

		SPI6->DR = value; // put one character in data register
		while(!(SPI6->SR & (SPI_SR_TXE))); // wait until transmit is over

}

void sendViaSPI2(uint16_t value) {

		SPI2->DR = value; // put one character in data register
		while(!(SPI2->SR & (SPI_SR_TXE))); // wait until transmit is over

}

void csUp() {
  	GPIOH->BSRR |= GPIO_BSRR_BS6;
}

void csDown() {
  	GPIOH->BSRR |= GPIO_BSRR_BR6;
}

void enableEEPROM() {

	csUp();
	delayMcs(40);
	sendViaSPI2((OPCODE_WEN << EEPROM_ADDRESS_LENGTH) | DUMMYADDRESS_WEN);
	delayMcs(40);
	csDown();

	csUp();
	delayMcs(40);
	sendViaSPI2((OPCODE_WRITE << EEPROM_ADDRESS_LENGTH));
	sendViaSPI2((32 << 8) | (32)); //clear EEPROM memory on reserved address
	delayMcs(80);
	csDown();

	delayMs(3);

	csUp();
	delayMcs(40);
	sendViaSPI2((OPCODE_WRITE << EEPROM_ADDRESS_LENGTH) + 1); //clear EEPROM memory on reserved address
	sendViaSPI2((32 << 8) | (32));
	delayMcs(80);
	csDown();

	delayMs(3);

}

void writeToEEPROM(int value, int mode) {

	if((value < 0) || (value > 9999)) {

		sendViaUsart3("\n\r");
		sendViaUsart3(errorInvalidValue);
		sendViaUsart3("\n\r");
		return;

	}

	switch (mode) {

		case 1: { //binary mode

			csUp();
			delayMcs(40);
			sendViaSPI2((OPCODE_WRITE << EEPROM_ADDRESS_LENGTH));
			sendViaSPI2(value);
			delayMcs(80);
			csDown();

		} break;

		case 2: { //ASCII mode

			csUp();
			delayMcs(40);
			sendViaSPI2((OPCODE_WRITE << EEPROM_ADDRESS_LENGTH));
			sendViaSPI2(((value/1000 + 48) << 8) | ((value/100)%10 + 48));
			delayMcs(80);
			csDown();

			delayMs(3);

			csUp();
			delayMcs(40);
			sendViaSPI2((OPCODE_WRITE << EEPROM_ADDRESS_LENGTH) + 1);
			sendViaSPI2((((value/10)%10 + 48) << 8) | (value%10 + 48));
			delayMcs(80);
			csDown();

		} break;

		default: {

			sendViaUsart3("\n\r");
			sendViaUsart3(errorInvalidMode);
			sendViaUsart3("\n\r");
			return;

		} break;

	}

}

void SysTick_Handler(void) { // redefinition of SysTick_Handler

	ticks++;
    sTicks++;
	if(!(sTicks%5000) && !(key_on()) && (SPI6->CR1 & SPI_CR1_SPE)) { sendViaSPI6(DUMMY_WORD); } // 200HZ

}

int power(int base, int a) { // calculates power a of base

	if(a < 0) return 0;

    if (a != 0)
        return (base * power(base, a - 1));
    else
        return 1;

}

int strlen(const char* string) { // returns string length

    int length = 0;
    for(int i = 0; string[i] !=0; i++) {
        length++;
    }
    return length;

}

char itoa_output[MAX_ITOA_OUTPUT_LENGTH];

char* itoa(int value) { // converts integer to string

	if(value == 0) { return "0"; }

	int n = 0;
	if(value < 0) {
		value *= -1;
		itoa_output[n] = '-';
		n++;
	}

 	while(value != 0) {
		itoa_output[n] = value % 10 + 48;
    	value /= 10;
    	n++;
  	}

  	itoa_output[n] ='\0';

  	int i = 0, j = n - 1; // reverse
  	char buffer;

  	while (i < j) {
    	buffer = itoa_output[i];
    	itoa_output[i] = itoa_output[j];
    	itoa_output[j] = buffer;
    	i++;
    	j--;
  	}

  	return itoa_output;

}

int isize(int number) { // calculates number size in symbols

	int size = 0;
	if(number < 0) {
		number *= -1;
		size++;
	}
	if(number < 10) return 1;
	size++;
	size += isize(number/10);
    return size;

}

void USART3_IRQHandler (void) { // redefinition of USART3 rx interrupt handler, provides convinient way to set display value

	if (USART3->SR & USART_SR_RXNE) { // check "data recieved" flag

		uint8_t byte = USART3->DR;
		switch(byte) {

			case 48 ... 58 : { // Digits

				if(displayValueUsart3Symbols < 4) {

					displayValueUsart3Symbols++;
					char symbol[2];
					symbol[0] = byte;
					symbol[1] = '\0';
					displayValueUsart3 *= 10;
					displayValueUsart3 += byte - 48;
					sendViaUsart3(symbol);

				}

			} break;

			case 13: { // Enter

				if(!displayValueUsart3Symbols) break;
				sendViaUsart3("\n\r");
				if(displayValueUsart3) SPI6->CR1 &= ~SPI_CR1_SPE;
				else SPI6->CR1 |= SPI_CR1_SPE;

				sTicks = 0;
				displayValue = displayValueUsart3;
				displayValueUsart3 = 0;
				displayValueUsart3Symbols = 0;

			} break;

			case 127: { // Backspace

				sendViaUsart3("\r    \r");
				switch(displayValueUsart3Symbols) {

					case 0: break;

					case 1: {

						displayValueUsart3 = 0;
						displayValueUsart3Symbols--;

					} break;

					default: {

						for(int i = 0; i < displayValueUsart3Symbols - isize(displayValueUsart3); i++) {
							sendViaUsart3("0");
						}
						displayValueUsart3 /= 10;
						if(displayValueUsart3) sendViaUsart3(itoa(displayValueUsart3));
						displayValueUsart3Symbols--;

					} break;

				}

			} break;

			default: break;

		}

		USART3->SR &= ~USART_SR_RXNE; // clear "data recieved" flag

	}

}

void UART3Init() {

  	RCC->APB1ENR |= RCC_APB1ENR_USART3EN;  // enable USART3 clock
  	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;  // enable GPIOB clock

  	GPIOB->MODER &= ~(GPIO_MODER_MODER11_Msk);
  	GPIOB->MODER |=  GPIO_MODER_MODER11_1; //set PB11 to alternate mode (USART3_TX pin)
  	GPIOB->MODER &= ~(GPIO_MODER_MODER10_Msk);
  	GPIOB->MODER |=  GPIO_MODER_MODER10_1; //set PB10 to alternate mode (USART3_RX pin)

	GPIOB->OSPEEDR &= ~(GPIO_OSPEEDR_OSPEED11_Msk|GPIO_OSPEEDR_OSPEED10_Msk);
  	GPIOB->OSPEEDR |= (GPIO_OSPEEDR_OSPEED11_1|GPIO_OSPEEDR_OSPEED10_1); // Set USART6_TX and USART6_RX pinsto high speed mode (0b10)

	GPIOB->AFR[1] &= ~(GPIO_AFRH_AFSEL10_Msk|GPIO_AFRH_AFSEL11_Msk);
  	GPIOB->AFR[1] |= (GPIO_AFRH_AFSEL10_2|GPIO_AFRH_AFSEL10_1|GPIO_AFRH_AFSEL10_0); // assign alternate function number 7 (USART_RX) to PB10 (USART3_RX pin)
  	GPIOB->AFR[1] |= (GPIO_AFRH_AFSEL11_2|GPIO_AFRH_AFSEL11_1|GPIO_AFRH_AFSEL11_0); // assign alternate function number 7 (USART_TX) to PB11 (USART3_TX pin)

    USART3->BRR = 0;
    USART3->BRR |= (24 << 4); // set BRR (frequency = 45 MHZ), BRR = frk/(baudrate * 16) = 45 * 10^6 / (115200 * 16) = 48,8
    USART3->BRR |= 8;

	USART3->CR1 |= USART_CR1_RXNEIE; //enable rx interrupts

	NVIC_EnableIRQ(USART3_IRQn);

  	USART3->CR1 |= USART_CR1_TE; // enable tx pin
  	USART3->CR1 |= USART_CR1_RE; // enable rx pin

  	USART3->CR1 |= USART_CR1_UE; //enable usart3

}

void SPI2Init() {

    RCC->APB1ENR |= RCC_APB1ENR_SPI2EN;

    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN;

    //gpio config
    GPIOB->MODER &= ~(GPIO_MODER_MODER15_Msk);
  	GPIOB->MODER |=  GPIO_MODER_MODER15_1; //set PB15 to alternate mode

    GPIOD->MODER &= ~(GPIO_MODER_MODER3_Msk);
    GPIOD->MODER |=  GPIO_MODER_MODER3_1; //set PD3 to alternate mode

    GPIOB->AFR[1] &= ~(GPIO_AFRH_AFSEL15_Msk);
  	GPIOB->AFR[1] |= (GPIO_AFRH_AFSEL15_2|GPIO_AFRH_AFSEL15_0);

    GPIOD->AFR[0] &= ~(GPIO_AFRL_AFSEL3_Msk);
  	GPIOD->AFR[0] |= (GPIO_AFRL_AFSEL3_2|GPIO_AFRL_AFSEL3_0);

    //cr1 config
    SPI2->CR1 &= ~(SPI_CR1_BR_Msk);
    SPI2->CR1 |= SPI_CR1_BR_0;
    SPI2->CR1 |= SPI_CR1_BR_1;
    SPI2->CR1 |= SPI_CR1_BR_2;

    SPI2->CR1 &= ~(SPI_CR1_CPOL_Msk);
    SPI2->CR1 &= ~(SPI_CR1_CPHA_Msk);

    SPI2->CR1 &= ~(SPI_CR1_BIDIMODE_Msk);
    SPI2->CR1 &= ~(SPI_CR1_RXONLY_Msk);

    SPI2->CR1 &= ~(SPI_CR1_LSBFIRST_Msk);

    SPI2->CR1 &= ~(SPI_CR1_CRCEN_Msk);
    SPI2->CR1 &= ~(SPI_CR1_CRCNEXT_Msk);

    SPI2->CR1 &= ~(SPI_CR1_SSM_Msk);
    SPI2->CR1 |= SPI_CR1_SSM;
    SPI2->CR1 &= ~(SPI_CR1_SSI_Msk);
    SPI2->CR1 |= SPI_CR1_SSI;

    SPI2->CR1 &= ~(SPI_CR1_MSTR_Msk);
    SPI2->CR1 |= SPI_CR1_MSTR;

    SPI2->CR1 &= ~(SPI_CR1_DFF_Msk);
    SPI2->CR1 |= SPI_CR1_DFF;

    SPI2->CR1 |= SPI_CR1_SPE;

}

void SPI6Init() {

    RCC->APB2ENR |= RCC_APB2ENR_SPI6EN;

    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOGEN;

    //gpio config
    GPIOG->MODER &= ~(GPIO_MODER_MODER14_Msk);
  	GPIOG->MODER |=  GPIO_MODER_MODER14_1; //set PG14to alternate mode (MOSI)

    GPIOG->MODER &= ~(GPIO_MODER_MODER13_Msk);
    GPIOG->MODER |=  GPIO_MODER_MODER13_1; //set PG13 to alternate mode (SCK)

    GPIOG->AFR[1] &= ~(GPIO_AFRH_AFSEL14_Msk);
  	GPIOG->AFR[1] |= (GPIO_AFRH_AFSEL14_2|GPIO_AFRH_AFSEL14_0);

	GPIOG->AFR[1] &= ~(GPIO_AFRH_AFSEL13_Msk);
  	GPIOG->AFR[1] |= (GPIO_AFRH_AFSEL13_2|GPIO_AFRH_AFSEL13_0);

    //cr1 config
    SPI6->CR1 &= ~(SPI_CR1_BR_Msk);
    SPI6->CR1 |= SPI_CR1_BR_0;
    SPI6->CR1 |= SPI_CR1_BR_1;
    SPI6->CR1 |= SPI_CR1_BR_2;

    SPI6->CR1 &= ~(SPI_CR1_CPOL_Msk);
    SPI6->CR1 &= ~(SPI_CR1_CPHA_Msk);

    SPI6->CR1 &= ~(SPI_CR1_BIDIMODE_Msk);
    SPI6->CR1 &= ~(SPI_CR1_RXONLY_Msk);

    SPI6->CR1 &= ~(SPI_CR1_LSBFIRST_Msk);

    SPI6->CR1 &= ~(SPI_CR1_CRCEN_Msk);
    SPI6->CR1 &= ~(SPI_CR1_CRCNEXT_Msk);

    SPI6->CR1 &= ~(SPI_CR1_SSM_Msk);
    SPI6->CR1 |= SPI_CR1_SSM;
    SPI6->CR1 &= ~(SPI_CR1_SSI_Msk);
    SPI6->CR1 |= SPI_CR1_SSI;

    SPI6->CR1 &= ~(SPI_CR1_MSTR_Msk);
    SPI6->CR1 |= SPI_CR1_MSTR;

    SPI6->CR1 &= ~(SPI_CR1_DFF_Msk);
    SPI6->CR1 |= SPI_CR1_DFF;

    SPI6->CR1 |= SPI_CR1_SPE;

}

void rclkRise() { // rising edge RCLK

	GPIOG->BSRR |= GPIO_BSRR_BR10;
    delayMcs(300);
	GPIOG->BSRR |= GPIO_BSRR_BS10;

}

void registerUpdate() { // copies shift register to non-shift register

	rclkRise();

}

void setDigit(uint16_t digit, uint16_t led_num) { // sets symbol on selected led

	if((led_num > NUMBER_OF_LEDS) || (led_num < 1)) { blinkLed(); return; }
    sendViaSPI2(digit | led_array[led_num - 1] | KEY_EVERY);
	registerUpdate();

}

void driveShield() { // main function to control extension shield

	if((displayValue < 0) || (displayValue > 9999)) { blinkLed(); return; }

	if((sTicks >= 1000000) && displayValue) { //each second changes displayValue
        sTicks = 0;
		switch (displayValue) {
			case 1:  {
				displayValue--;
				writeToEEPROM(displayValue,BINARY);
				SPI6->CR1 |= SPI_CR1_SPE;
			} break;
			default: {
				displayValue--;
				writeToEEPROM(displayValue,BINARY);
			} break;
		}
    }

    for(int i = 0; i < NUMBER_OF_LEDS; i++) { // blinks displayValue on display
		setDigit(digit_array[((displayValue/power(10,i)) % 10)],NUMBER_OF_LEDS - i);
		delayMs(3);
	}

}

int main(void)
{

    set_sysclk_max();
  	UART3Init();
    SPI2Init();
	SPI6Init();

	SPI6->CR1 &= ~SPI_CR1_SPE; //disable SPI6

    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN; //enable led pins
    GPIOA->MODER |= GPIO_MODER_MODER7_0;

	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOHEN;
	GPIOH->MODER &= ~(GPIO_MODER_MODER6_Msk); // CS output mode
	GPIOH->MODER |= GPIO_MODER_MODER6_0;

    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOGEN;  // enable GPIOG clock
    GPIOG->MODER &= ~(GPIO_MODER_MODER10_Msk); //rclk
  	GPIOG->MODER |=  GPIO_MODER_MODER10_0;
    GPIOG->BSRR |= GPIO_BSRR_BR10;


  	SysTick_Config(SystemCoreClock/(800000/9));

    sendViaUsart3("\n\r\n\r");
	sendViaUsart3("Enter value to send on display:\n\r");

    displayValue = 9999;
	enableEEPROM();
	writeToEEPROM(displayValue,BINARY);
	sTicks = 0;

    while(1) {

		driveShield();

    }

    return 0;
}
