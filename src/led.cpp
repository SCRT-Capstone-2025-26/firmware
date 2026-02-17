// largely stolen from https://github.com/xyphro/XiaoNeoPixel_RP2040

#include "led.h"
#include "util.h"
#include "pins.h"

// TODO: Investigate the seemlingly needed sleeps

RGB leds[6];

RGB::RGB(uint8_t r, uint8_t g, uint8_t b) {
    this->r = r;
    this->g = g;
    this->b = b;
}

RGB::RGB() {
    r = 0;
    g = 0;
    b = 0;
}

void led_init() {
    uart_init(uart0, 6660000ul);
    uart_set_format(uart0, 6, 1, UART_PARITY_NONE);
    gpio_set_function(LED_DATA, GPIO_FUNC_UART);
    io_bank0_hw->io[LED_DATA].ctrl = (io_bank0_hw->io[LED_DATA].ctrl & ~(3u<<8)) | (1u<<8); // invert UART output signal

    sleep_us(100); // This seems to be needed to properly init

    for (int i = 0; i < LED_COUNT; i++) {
      leds[i] = LED_DISABLE;
    }

    led_show();
}

#define L0 0b111111
#define L1 0b110000

uint8_t write_buffer[24 * 6];

uint led_channel = 0;

void write_leds_dma() {
    // copied from https://github.com/rossihwang/pico_dma_uart
    dma_channel_config tx_config = dma_channel_get_default_config(led_channel);
    channel_config_set_transfer_data_size(&tx_config, DMA_SIZE_8);
    channel_config_set_read_increment(&tx_config, true);
    channel_config_set_write_increment(&tx_config, false);
    channel_config_set_dreq(&tx_config, DREQ_UART0_TX);
    dma_channel_set_config(led_channel, &tx_config, false);
    dma_channel_set_write_addr(led_channel, &uart0_hw->dr, false);
    dma_channel_set_read_addr(led_channel, &write_buffer, false);
    dma_channel_set_transfer_count(led_channel, dma_encode_transfer_count(24 * 6), true);
}

void __not_in_flash_func(write_leds)(RGB* values, int count) {
    int j = 0;
    for (int i = 0; i < count; i++) {
        RGB v = values[i];
        uint32_t dataword = (((uint32_t)v.g)<<16) | (((uint32_t)v.r)<<8) | (((uint32_t)v.b)<<0);

        for (uint8_t bitcounter=0; bitcounter<24; bitcounter++) {
            dataword <<= 1;
            if (dataword & (1<<24)) {
                write_buffer[j++] = L1;
                //uart_putc(uart0, L1); // send a 1
            } else {
                write_buffer[j++] = L0;
                //uart_putc(uart0, L0); // send a 0
            }
        }
    }
    write_leds_dma();
}

void led_show() {
    write_leds(leds, 6);

    sleep_us(1000); // This seems to be needed to properly show leds in some cases
}

