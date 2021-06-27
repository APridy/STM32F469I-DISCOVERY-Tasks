#include "stm32f4xx.h"
#include "util.h"

#define MAX_ITOA_OUTPUT_LENGTH 12

#define address_invalid(addr) ((addr > 511) || (addr < 0))

const char* errorInvalidMode = "ERROR: Invalid memory output mode\n\r\0";
const char* errorInvalidAddress = "ERROR: Address must vary between 0 and 2047\n\r\0";
const char* errorBeyondMemory = "ERROR: String goes beyond memory size\n\r\0";

enum memoryDisplayMode {
	BINARY = 1,
	DECIMAL = 2,
	ASCII = 3
};

enum opCode {
	OPCODE_READ = 0b10,
	OPCODE_WRITE = 0b01,
	OPCODE_WEN = 0b00,
	OPCODE_WDS = 0b00,
	OPCODE_ERASE = 0b11,
	OPCODE_ERAL = 0b00,
	OPCODE_WRAL = 0b00
};

enum dummyAddress {
	DUMMYADDRESS_WEN = 3 << 8,
	DUMMYADDRESS_WDS = 0 << 9,
	DUMMYADDRESS_ERAL = 1 << 9,
	DUMMYADDRESS_WRAL = 1 << 8
};

int delvalue = 750;

volatile int ticks = 0;

void delayMs(unsigned int ms) { // delay in miliseconds
	ticks = 0;
	while(ticks<ms*1000);
}

void delayMcs(int mcs) { // delay in microseconds
    ticks = 0;
    while(ticks<mcs);
}

void SysTick_Handler(void) { // redefinition of SysTick_Handler
	ticks++;
}


void sendViaUsart(const char* msg) {
	for(int i = 0; msg[i] != '\0'; i++) {
		USART6->DR = msg[i]; // put one character in data register
		while(!(USART6->SR & (USART_SR_TC))); // wait until transmit is over
		USART6->SR &= ~(USART_SR_TC); // clear transmit value
	}
}

void UART6Config() {

  	RCC->APB2ENR |= RCC_APB2ENR_USART6EN;  // enable USART6 clock, bit 5 on APB2ENR
  	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOGEN;  // enable GPIOG clock, bit 6 on AHB1ENR

  	GPIOG->MODER &= ~(GPIO_MODER_MODER14_Msk);
  	GPIOG->MODER |=  GPIO_MODER_MODER14_1; //set PG14 to alternate mode (USART6_TX pin)
  	GPIOG->MODER &= ~(GPIO_MODER_MODER9_Msk);
  	GPIOG->MODER |=  GPIO_MODER_MODER14_1; //set PG9 to alternate mode (USART6_RX pin)

	GPIOG->OSPEEDR &= ~(GPIO_OSPEEDR_OSPEED14_Msk|GPIO_OSPEEDR_OSPEED9_Msk);
  	GPIOG->OSPEEDR |= (GPIO_OSPEEDR_OSPEED14_1|GPIO_OSPEEDR_OSPEED9_1); // Set USART6_TX and USART6_RX pinsto high speed mode (0b10)

	GPIOG->AFR[1] &= ~(GPIO_AFRH_AFSEL9_Msk|GPIO_AFRH_AFSEL14_Msk);
  	GPIOG->AFR[1] |= GPIO_AFRH_AFSEL9_3; // assign alternate function number 8 (USART_RX) to PG9 (USART6_RX pin)
  	GPIOG->AFR[1] |= GPIO_AFRH_AFSEL14_3; // assign alternate function number 8 (USART_TX) to PG14 (USART6_TX pin)

  	USART6->BRR = 0;
  	USART6->BRR |= (48 << 4); // set BRR (frequency = 90 MHZ), BRR = frk/(baudrate * 16) = 90 * 10^6 / (115200 * 16) = 48,8
  	USART6->BRR |= 13;

  	USART6->CR1 |= USART_CR1_TE; // enable tx pin
  	USART6->CR1 |= USART_CR1_RE; // enable rx pin

  	USART6->CR1 |= USART_CR1_UE; //enable usart6

}

void sckUp() {
  	GPIOD->BSRR |= GPIO_BSRR_BS3;
}

void sckDown() {
  	GPIOD->BSRR |= GPIO_BSRR_BR3;
}

void csUp() {
  	GPIOH->BSRR |= GPIO_BSRR_BS6;
}

void csDown() {
  	GPIOH->BSRR |= GPIO_BSRR_BR6;
}

void mosiUp() {
  	GPIOB->BSRR |= GPIO_BSRR_BS15;
}

void mosiDown() {
  	GPIOB->BSRR |= GPIO_BSRR_BR15;
}

char itoa_output[MAX_ITOA_OUTPUT_LENGTH];

char* itoa(int value) {

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

void sckRise() { // rises sck from 0 to 1
	sckDown();
  	sckUp();
}

void M93C76SetOperationCode(enum opCode code) {

	mosiUp(); // start bit
	sckRise();
	if(code >> 1) mosiUp();
	else mosiDown();
	sckRise();
	if(code % 2) mosiUp();
	else mosiDown();
 	sckRise();

}

void M93C76SetAddress(unsigned int addr) {

	for(int i = 9; i >= 0; i--) {
		if (((addr >> i) & 1) == 1) mosiUp();
		else mosiDown();
		sckRise();
	}

}

void M93C76setValue(uint16_t value) {

	for(int i = 15; i >= 0; i--) { // data
		if(((value >> i) & 1) == 1) mosiUp();
		else mosiDown();
		sckRise();
	}

}

void M93C76WaitReady() {

	csDown();
  	sckRise();
  	delayMcs(delvalue);
  	csUp();
  	while (((GPIOB->IDR & GPIO_IDR_ID14) >> 14) == 1) {} // waiting till chip is ready
  	csDown();
  	delayMcs(delvalue*3);

}

uint16_t M93C76Read(unsigned int addr) {

	if(address_invalid(addr)) { sendViaUsart(errorInvalidAddress); return 0; }

  	csUp();
	M93C76SetOperationCode(OPCODE_READ);
  	M93C76SetAddress(addr);

  	uint16_t value = 0;
  	for(int i = 0; i < 16; i++) { // recieving data (16 bits)
		sckDown();
		sckUp();
		delayMcs(delvalue);
		if(((GPIOB->IDR & GPIO_IDR_ID14) >> 14) == 1) { value = value*2 + 1;}
		else value = value*2 + 0;
  	}

  	csDown();
  	return value;
}

void M93C76Dump(int mode) { // mode: 1 - binary, 2 - decimal, 3 - ASCII

	switch(mode) {
		case 1: { //outputs memory in binary
			sendViaUsart("\n\r");
			for(int i = 0; i < 64; i++) {
				for(int j = 0; j < 8; j++) {
					uint16_t word = M93C76Read(i*8 + j);

					sendViaUsart("0b");
					for(int i = 15; i >=8;i-- ) {
						if(((word >> i) & 1) == 1) { sendViaUsart("1"); }
						else sendViaUsart("0");
					}
					sendViaUsart(" ");

					sendViaUsart("0b");
					for(int i = 7; i >=0;i--) {
						if(((word >> i) & 1) == 1) { sendViaUsart("1"); }
						else sendViaUsart("0");
					}
					sendViaUsart(" ");

					sendViaUsart("   ");
				}
				sendViaUsart("\n\r");
			}
		} break;
		case 2: { //outputs memory in decimal
			sendViaUsart("\n\r");
			for(int i = 0; i < 64; i++) {
				for(int j = 0; j < 8; j++) {
					uint16_t word = M93C76Read(i*8 + j);
					uint8_t hvalue = word >> 8;
					uint8_t lvalue = word;
					sendViaUsart(itoa(hvalue));
					sendViaUsart(" ");
					sendViaUsart(itoa(lvalue));
					sendViaUsart("  ");
				}
				sendViaUsart("\n\r");
			}
		} break;
		case 3: { //outputs memory in ASCII
			sendViaUsart("\n\r");
			for(int i = 0; i < 64; i++) {
				for(int j = 0; j < 8; j++) {
					uint16_t word = M93C76Read(i*8 + j);
					char symbol[3];

					symbol[2] = '\0';
					symbol[1] = word;
					symbol[0] = word >> 8;

					sendViaUsart(symbol);
					sendViaUsart("  ");
				}
				sendViaUsart("\n\r");
			}
		} break;
		default: {
			sendViaUsart(errorInvalidMode);
		} break;
	}

}

void M93C76Erase(unsigned int addr) {

	if(address_invalid(addr)) { sendViaUsart(errorInvalidAddress); return; }

	csUp();
	M93C76SetOperationCode(OPCODE_ERASE);
  	M93C76SetAddress(addr);
	M93C76WaitReady();

}

void M93C76Write(unsigned int addr, uint16_t value) {

	if(address_invalid(addr)) { sendViaUsart(errorInvalidAddress); return; }

	csUp();
  	M93C76SetOperationCode(OPCODE_WRITE);
  	M93C76SetAddress(addr);
  	M93C76setValue(value);
  	M93C76WaitReady();

}

void M93C76WriteAll(uint16_t value) {

  	csUp();
	M93C76SetOperationCode(OPCODE_WRAL);
	M93C76SetAddress(DUMMYADDRESS_WRAL);
  	M93C76setValue(value);
	M93C76WaitReady();

}

void M93C76EraseAll() {

  	csUp();
  	M93C76SetOperationCode(OPCODE_ERAL);
  	M93C76SetAddress(DUMMYADDRESS_ERAL);
  	M93C76WaitReady();

}

void M93C76Wen() {

  	csUp();
  	M93C76SetOperationCode(OPCODE_WEN);
  	M93C76SetAddress(DUMMYADDRESS_WEN);
  	csDown();
  	sckRise();

}

void M93C76Wds() {

	csUp();
	M93C76SetOperationCode(OPCODE_WDS);
  	M93C76SetAddress(DUMMYADDRESS_WDS);
  	csDown();
  	sckRise();

}

int strlen(const char* string) {
	int length = 0;
	for(int i = 0; string[i] !=0; i++) {
		length++;
	}
	return length;
}

void M93C76WriteString(unsigned int addr, char* string) {

	if(address_invalid(addr)) { sendViaUsart(errorInvalidAddress); return; }
	if(strlen(string) > (512 - addr)*2) { sendViaUsart(errorBeyondMemory); return; }

	int i = 0;
	for(i = 0; (string[i] != '\0') && (string[i + 1] != '\0'); i += 2) {
		M93C76Write(addr + i/2, (string[i] << 8) | string[i+1]);
	}

	if(string[i] != '\0') M93C76Write(addr + i/2, (string[i] << 8) | (M93C76Read(addr + i/2) & 0x00FF) ); // executes if string length is uneven
	return;
}

// 1 - cs d10 ph6
// 2 - sck d13 pd3
// 3 - mosi d11 pb15
// 4 - miso d12 pb14

int main(void) {

  	set_sysclk_max();
  	UART6Config();
  	SysTick_Config(SystemCoreClock/(800000/9));

  	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN; //enable used clocks
  	RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN;
  	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOHEN;

  	GPIOB->MODER &= ~(GPIO_MODER_MODER15_Msk); // set needed pins to output mode
  	GPIOB->MODER |= GPIO_MODER_MODER15_0; // MOSI output mode

  	GPIOB->MODER &= ~(GPIO_MODER_MODER14_Msk); // MISO input mode

  	GPIOD->MODER &= ~(GPIO_MODER_MODER3_Msk); // SCK output mode
  	GPIOD->MODER |= GPIO_MODER_MODER3_0; // MOSI output mode

  	GPIOH->MODER &= ~(GPIO_MODER_MODER6_Msk); // CS output mode
  	GPIOH->MODER |= GPIO_MODER_MODER6_0;

  	sendViaUsart("\n\r");

  	mosiDown();
  	sckDown();
  	csDown();

  	M93C76Wen(); // enables writing and erasing

  	M93C76EraseAll(); //erases everything on chip

	int j = 48;
	for(int i = 0; i < 512; i++) { // fills memory with the set of ASCII characters
		M93C76Write(i,256*j + j);
		j++;
		if(j == 127) j = 48;
	}

	M93C76WriteString(0, "                                                "); // bordering my surname
	M93C76WriteString(9, "PRIDYBAILO");

  	M93C76Wds(); //disables writing and erasing

  	sendViaUsart("\n\r");
  	M93C76Dump(ASCII); // reads and displays chip memory via USART6 (baudrate = 115200)
  	sendViaUsart("\n\r");
  	//M93C76Dump(DECIMAL); // uncomment to print all 3 views
	//sendViaUsart("\n\r");
  	//M93C76Dump(BINARY);
  	//sendViaUsart("\n\r");

  	return 0;
}
