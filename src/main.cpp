#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/timer.h>

void setup() {

rcc_periph_clock_enable(RCC_GPIOA);
gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO9 | GPIO10);
gpio_set_af(GPIOA,GPIO_AF7, GPIO9 | GPIO10);

rcc_periph_clock_enable(RCC_USART1);
usart_set_baudrate(USART1, 115200);
usart_set_databits(USART1, 8);
usart_set_stopbits(USART1, USART_STOPBITS_1);
usart_set_parity(USART1, USART_PARITY_NONE);
usart_set_mode(USART1, USART_MODE_TX_RX);
usart_set_flow_control(USART1, USART_FLOWCONTROL_NONE);
usart_enable(USART1);

}

void loop() {
    usart_send_blocking(USART1, 0x55);
    
}

int main () {

    setup();

    while (true) {
        loop();
    }
}