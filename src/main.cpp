#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>

void setup() {

rcc_periph_clock_enable(RCC_GPIOA);
gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO2 | GPIO3);
gpio_set_af(GPIOA,GPIO_AF7, GPIO2 | GPIO3);

rcc_periph_clock_enable(RCC_USART2);
usart_set_baudrate(USART2, 19200);
usart_set_databits(USART2, 8);
usart_set_stopbits(USART2, USART_STOPBITS_1);
usart_set_parity(USART2, USART_PARITY_NONE);
usart_set_mode(USART2, USART_MODE_TX_RX);
usart_set_flow_control(USART2, USART_FLOWCONTROL_NONE);
usart_enable(USART2);

}
void loop() {
    uint16_t c = usart_recv_blocking(USART2);
    usart_send_blocking(USART2, c);
    
}

int main () {

    setup();

    while (true) {
        loop();
    }
}