#ifndef __UTIL_H__
#define __UTIL_H__

void set_sysclk_max(void);

void usart3_init(void);
void usart3_putchar(char);
void usart3_puts(char *);

void delay_ms(int);

#endif
