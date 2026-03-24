#ifndef LCD_H
#define LCD_H

#include <stdint.h>

#define RS      15
#define E       23
#define D4      13
#define D5      14
#define D6      27
#define D7      3

void lcd_init(void);
void lcd_send_nibble(uint8_t nibble, uint8_t rs);
void lcd_send_byte(uint8_t byte, uint8_t rs);
void lcd_print(const char *str);

#endif