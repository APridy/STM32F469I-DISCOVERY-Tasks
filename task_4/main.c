#include "stm32f4xx.h"
#include "util.h"
#include "stdbool.h"

#define MAX_ITOA_OUTPUT_LENGTH 12

#define DUMMY_BYTE 0xFF
#define ERASE_BYTE 0xAA
#define ZERO_ADDRESS 0

#define device_select_with_address_bits(opcode,addr) (opcode | (((addr >> 8) & 0b111) << 1)) // initializes address bits (A10,A9,A8) in DevSel byte
#define address_invalid(addr) ((addr > 2047) || (addr < 0))

const char* errorInvalidMode = "ERROR: Invalid memory output mode\n\r\0";
const char* errorInvalidAddress = "ERROR: Address must vary between 0 and 2047\n\r\0";
const char* errorBeyondMemoryWrite = "ERROR: String goes beyond memory size\n\r\0";
const char* errorBeyondMemoryRead = "ERROR: Read length goes beyond memory size\n\r\0";
const char* errorBeyondMemoryErase = "ERROR: Erase length goes beyond memory size\n\r\0";

enum memoryDisplayMode {
    BINARY = 1,
    DECIMAL = 2,
    ASCII = 3
};

enum opCode {
    DEVICE_SELECT = 0b10100000,
    OPCODE_READ = DEVICE_SELECT | 1,
    OPCODE_WRITE = DEVICE_SELECT
};

enum ackCode {
    ACK = false,
    NO_ACK = true
};

volatile int ticks = 0;

int delvalue = 25;

void delayMs(int ms) { // delay in miliseconds
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

// clk - d15 - pb8
// sda - d14 - pb9

void setClkOutput() {
    GPIOB->MODER &= ~(GPIO_MODER_MODER8_Msk);
    GPIOB->MODER |= GPIO_MODER_MODER8_0;
}

void setSdaOutput() {
    GPIOB->MODER &= ~(GPIO_MODER_MODER9_Msk);
    GPIOB->MODER |= GPIO_MODER_MODER9_0;
}

void setSdaInput() {
    GPIOB->MODER &= ~(GPIO_MODER_MODER9_Msk);
}

void clkUp() {
    GPIOB->BSRR |= GPIO_BSRR_BS8;
}

void clkDown() {
    GPIOB->BSRR |= GPIO_BSRR_BR8;
}

void sdaUp() {
    GPIOB->BSRR |= GPIO_BSRR_BS9;
}

void sdaDown() {
    GPIOB->BSRR |= GPIO_BSRR_BR9;
}

int strlen(const char* string) {

    int length = 0;
    for(int i = 0; string[i] !=0; i++) {
        length++;
    }
    return length;

}

char itoa_output[MAX_ITOA_OUTPUT_LENGTH];

char* itoa(int value) {

    if(value == 0) return "0";

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

void sendStartCondition() {

    sdaUp();
    clkUp();
    delayMcs(delvalue);

    sdaDown();
    delayMcs(delvalue);
    clkDown();
    delayMcs(delvalue);

}

void sendStopCondition() {

    clkUp();
    delayMcs(delvalue);
    sdaUp();
    delayMcs(delvalue);

}

void sendBit(bool bit) {

    if(bit) { sdaUp(); }
    else { sdaDown(); }
    delayMcs(delvalue);
    clkUp();
    delayMcs(delvalue);
    clkDown();
    delayMcs(delvalue);

}

void sendByte(uint8_t byte) {

    for(int i = 7; i >= 0; i--) {
        sendBit((byte >> i) & 1);
    }
    delayMcs(delvalue);

}

bool readBit() {

    bool bit;

    clkUp();
    delayMcs(delvalue);
    bit = ((GPIOB->IDR & GPIO_IDR_ID9) >> 9);
    clkDown();
    delayMcs(delvalue);

    return bit;

}

uint8_t readByte() {

    uint8_t byte = 0;

    for(int i = 7; i >= 0; i--) {
        byte *= 2;
        if(readBit()) { byte += 1; }
        else { byte += 0; }
    }
    delayMcs(delvalue);

    return byte;

}

uint8_t M24C16RandomAdddressRead(uint16_t address) {

    if(address_invalid(address)) { sendViaUsart(errorInvalidAddress); return 0; }

    uint8_t byte = 0;

    sendStartCondition();

    sendByte(device_select_with_address_bits(OPCODE_WRITE,address));
    sendBit(ACK);
    sendByte(address);
    sendBit(ACK);
    sendByte(DUMMY_BYTE);
    sendBit(ACK);

    sendStartCondition();

    sendByte(OPCODE_READ);
    sendBit(ACK);

    setSdaInput();
    byte = readByte();
    setSdaOutput();
    sendBit(NO_ACK);

    sendStopCondition();

    return byte;

}


void M24C16ByteWrite(uint16_t address, uint8_t value) {

    if(address_invalid(address)) { sendViaUsart(errorInvalidAddress); return; }

    sendStartCondition();

    sendByte(device_select_with_address_bits(OPCODE_WRITE,address));
    sendBit(ACK);
    sendByte(address);
    sendBit(ACK);
    sendByte(value);
    sendBit(ACK);

    sendStopCondition();

}

void M24C16Erase(uint16_t address, int range) {

    if(address_invalid(address)) { sendViaUsart(errorInvalidAddress); return; }
    if(range > 2048 - address) { sendViaUsart(errorBeyondMemoryErase); return; }

    for(int i = address; i < address + range; i++) {
        M24C16RandomAdddressRead(0); // RandomRead necessary according to manual
        M24C16ByteWrite(i, ERASE_BYTE);
    }

}
void M24C16WriteString(uint16_t address, const char* string) {

    if(address_invalid(address)) { sendViaUsart(errorInvalidAddress); return; }
    if(strlen(string) > 2048 - address) { sendViaUsart(errorBeyondMemoryWrite); return; }

    for(int i = 0; i < strlen(string); i++) {
        M24C16RandomAdddressRead(0); // RandomRead necessary according to manual
        M24C16ByteWrite(address + i, string[i]);
    }

}

uint8_t memory_dump[2048];

uint8_t* M24C16SequentialRandomRead(uint16_t address,int length) {

    if(address_invalid(address)) { sendViaUsart(errorInvalidAddress); return 0; }
    if(length > 2048 - address) { sendViaUsart(errorBeyondMemoryRead); return 0; }

    M24C16RandomAdddressRead(0); // according to M24C16 manual sequential read operation should be called only after RandomRead or CurrentRead operations

    setSdaOutput();

    sendStartCondition();

    sendByte(device_select_with_address_bits(OPCODE_WRITE,address));
    sendBit(ACK);
    sendByte(0);
    sendBit(ACK);
    sendByte(DUMMY_BYTE);
    sendBit(ACK);

    sendStartCondition();
    sendByte(OPCODE_READ);
    sendBit(ACK);

    for(int i = 0; i < length; i++) {
        setSdaInput();
        memory_dump[i] = readByte();
        setSdaOutput();
        if(i == length - 1) sendBit(NO_ACK);
        else sendBit(ACK);
    }

    sendStopCondition();

    return memory_dump;

}

void M24C16Dump(int mode) { // mode: 1 - binary, 2 - decimal, 3 - ASCII

    M24C16SequentialRandomRead(0,2048);

    switch(mode) {

        case 1: { //outputs memory in binary

            for(int i = 0; i < 128; i++) {

                for(int j = 0; j < 16; j++) {

                    sendViaUsart("0b");
                    uint8_t byte = memory_dump[i*16 + j];
                    for(int l = 7; l >=0; l--) {

                        char symbol[2];
                        symbol[0] = ((byte >> l) & 1) + 48;
                        symbol[1] = '\0';
                        sendViaUsart(symbol);

                    }
                    sendViaUsart(" ");

                }
                sendViaUsart("\n\r");

            }

            sendViaUsart("\n\r");

        } break;

        case 2: { //outputs memory in decimal

            for(int i = 0; i < 128; i++) {

                for(int j = 0; j < 16; j++) {

                    sendViaUsart(itoa(memory_dump[i*16 + j]));
                    sendViaUsart(" ");

                }
                sendViaUsart("\n\r");

            }

            sendViaUsart("\n\r");

        } break;

        case 3: { //outputs memory in ASCII

            for(int i = 0; i < 128; i++) {

                for(int j = 0; j < 16; j++) {

                    char symbol[2];
                    symbol[0] = memory_dump[i*16 + j];
                    symbol[1] = '\0';
                    sendViaUsart(symbol);
                    sendViaUsart(" ");

                }
                sendViaUsart("\n\r");

            }

            sendViaUsart("\n\r");

        } break;

        default: {

             sendViaUsart("\n\r");
             sendViaUsart(errorInvalidMode);
             sendViaUsart("\n\r");

         } break;

    }

}

int main(void) {

    set_sysclk_max();
    UART6Config();
    SysTick_Config(SystemCoreClock/(800000/9)); //configuring systick for delay function

    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN; // enable GPIOB clock

    setClkOutput();
    setSdaOutput();

    sendViaUsart("\n\r\n\r");

    clkUp();
    sdaUp();
    delayMcs(delvalue);

    M24C16Erase(208,10);
    M24C16WriteString(208,"PRIDYBAILO"); // writes my surname beginning from address 208 (this is where token was)

    M24C16Dump(ASCII);
    M24C16Dump(DECIMAL); //uncomment to view memory in different modes
    M24C16Dump(BINARY);

    return 0;

}
