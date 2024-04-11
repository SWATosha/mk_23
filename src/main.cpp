#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>


#include <libopencm3/cm3/nvic.h>
#include "ring_buf/ring_buf.hpp"
#include <libopencm3/stm32/adc.h>
Ring_buffer buf;

uint8_t c{'a'};

void setup() {

// rcc_periph_clock_enable(RCC_GPIOA);
// gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO2 | GPIO3);
// gpio_set_af(GPIOA,GPIO_AF7, GPIO2 | GPIO3);

// rcc_periph_clock_enable(RCC_USART2);
// usart_set_baudrate(USART2, 115200);
// usart_set_databits(USART2, 8);
// usart_set_stopbits(USART2, USART_STOPBITS_1);
// usart_set_parity(USART2, USART_PARITY_NONE);
// usart_set_mode(USART2, USART_MODE_TX_RX);
// usart_set_flow_control(USART2, USART_FLOWCONTROL_NONE);
// usart_enable_rx_interrupt(USART2);
// nvic_enable_irq(NVIC_USART2_EXTI26_IRQ);

// usart_enable(USART2); //Включение ПУ


rcc_periph_clock_enable(RCC_ADC12);
adc_power_off(ADC1);
rcc_periph_reset_pulse(RST_ADC12);

rcc_periph_clock_enable(RCC_GPIOA);
gpio_mode_setup(GPIOA, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, GPIO0); //Режим альтернативной функции

adc_set_clk_prescale(ADC1, ADC_CCR_CKMODE_DIV2);
// adc_disable_scan_mode(ADC1);
adc_set_single_conversion_mode(ADC1); //однократное преобразование
adc_set_sample_time(ADC1, 1, ADC_SMPR_SMP_7DOT5CYC); //канал 0, длительность выборки - 7.5 тактов
uint8_t channel_array[] = {1}; /* ADC1_IN1 (PA0) */
adc_set_regular_sequence(ADC1, 1, channel_array);
adc_power_on(ADC1); //подаём опорное напряжение на преобразователь 
adc_start_conversion_regular(ADC1); //запуск преобразования (программный)
while (!adc_eoc(ADC1)); 

rcc_periph_clock_enable(RCC_GPIOE);
gpio_mode_setup(GPIOE, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO9|GPIO11);

}
void loop() {
    // if(!buf.empty()) {
    //     c = buf.get();
    //     }
    // // uint16_t c = usart_recv_blocking(USART2);
    // usart_send_blocking(USART2, c);
    // for (volatile uint32_t i=0; i<2000000;i++);
    // gpio_toggle(GPIOE, GPIO9);
    uint32_t res;
    char str[20];
    
    gpio_toggle(GPIOE, GPIO9);
    
    adc_start_conversion_regular(ADC1); //запуск преобразования (программный)
    while (!adc_eoc(ADC1)); 
    res = adc_read_regular(ADC1);
    if (res > 2048) gpio_set(GPIOE, GPIO11);
    else gpio_clear(GPIOE, GPIO11);

    usart_send_blocking(USART2, res>>8);
    usart_send_blocking(USART2, res);
    usart_send_blocking(USART2, "\r");
    usart_send_blocking(USART2, "\n");
}


int main () {
    
    
    setup(); //После настройки можно подать сигнал - включить светодиод PE9(порт должен быть настроен)
    gpio_set(GPIOE, GPIO15);
    while (true) {
        loop();
    }
}
// void usart2_exti26_isr (void){
//     USART_RQR(USART2) &= ~(USART_RQR_RXFRQ);
//     // c = static_cast<uint8_t>(usart_recv(USART2));
    
//     buf.put(static_cast<uint8_t>(usart_recv(USART2)));
    
//     gpio_toggle(GPIOE, GPIO11);
//     //очистить флаг запроса прерывания
//     //Сохранить принятый символ в переменную (глобально надо определить)
//     //переключить светодиод, например PE11 (настроить порт для работы со светодиодом, в setup) 
// }