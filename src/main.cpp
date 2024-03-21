#include <ring_buf/ring_buf.hpp>

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>

#include <libopencm3/cm3/nvic.h>

uint8_t c{'a'};

void setup() {

rcc_periph_clock_enable(RCC_GPIOA);
gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO2 | GPIO3);
gpio_set_af(GPIOA,GPIO_AF7, GPIO2 | GPIO3);

rcc_periph_clock_enable(RCC_USART2);
usart_set_baudrate(USART2, 115200);
usart_set_databits(USART2, 8);
usart_set_stopbits(USART2, USART_STOPBITS_1);
usart_set_parity(USART2, USART_PARITY_NONE);
usart_set_mode(USART2, USART_MODE_TX_RX);
usart_set_flow_control(USART2, USART_FLOWCONTROL_NONE);
usart_enable_rx_interrupt(USART2);
nvic_enable_irq(NVIC_USART2_IRQ);
usart_enable(USART2);

}
void loop() {
    
    uint16_t c = usart_recv_blocking(USART2);
    usart_send_blocking(USART2, 'c');
    for (volatile uint32_t i=0; i<2000000;i++);
    gpio_toggle(GPIOE, GPIO9);
    
}

int main () {

    setup(); //После настройки можно подать сигнал - включить светодиод PE9(порт должен быть настроен)
    gpio_set(GPIOE, GPIO9);
    while (true) {
        loop();
    }
}
void usart2_isr(void){


    //очистить флаг запроса прерывания
    //Сохранить принятый символ в переменную (глобально надо определить)
    //переключить светодиод, например PE11 (настроить порт для работы со светодиодом, в setup) 
}