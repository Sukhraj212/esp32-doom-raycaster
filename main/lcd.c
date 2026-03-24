#include "lcd.h"
#include "driver/gpio.h"
#include "esp_rom_sys.h"

void lcd_send_nibble(uint8_t nibble, uint8_t rs) {
    gpio_set_level(RS, rs);
    gpio_set_level(D4, (nibble >> 0) & 1);
    gpio_set_level(D5, (nibble >> 1) & 1);
    gpio_set_level(D6, (nibble >> 2) & 1);
    gpio_set_level(D7, (nibble >> 3) & 1);
    gpio_set_level(E, 1);
    esp_rom_delay_us(10);
    gpio_set_level(E, 0);
    esp_rom_delay_us(100);
}

void lcd_send_byte(uint8_t byte, uint8_t rs) {
    lcd_send_nibble((byte >> 4) & 0x0F, rs);
    lcd_send_nibble(byte & 0x0F, rs);
}

void lcd_init(void) {
    gpio_set_direction(RS, GPIO_MODE_OUTPUT);
    gpio_set_direction(E, GPIO_MODE_OUTPUT);
    gpio_set_direction(D4, GPIO_MODE_OUTPUT);
    gpio_set_direction(D5, GPIO_MODE_OUTPUT);
    gpio_set_direction(D6, GPIO_MODE_OUTPUT);
    gpio_set_direction(D7, GPIO_MODE_OUTPUT);

    // Boot up
    esp_rom_delay_us(50000);
    lcd_send_nibble(0x03, 0);
    esp_rom_delay_us(100000);
    lcd_send_nibble(0x03, 0);
    esp_rom_delay_us(100000);
    lcd_send_nibble(0x03, 0);

    // switches to 4-bit mode
    lcd_send_nibble(0x02, 0);
    esp_rom_delay_us(1000);

    // 2 lines, 5x8 font
    lcd_send_byte(0x28, 0);

    // display on, cursor off, blink off
    lcd_send_byte(0x0C, 0);
    // clear display
    lcd_send_byte(0x01, 0);
    esp_rom_delay_us(2000);
    // entry mode: increment, no shift
    lcd_send_byte(0x06, 0);

}

void lcd_print(const char *str) {
    for (int i = 0; str[i] != '\0'; i++) {
        lcd_send_byte(str[i], 1);
    }
}
