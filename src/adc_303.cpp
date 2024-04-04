    rcc_periph_clock_enable(RCC_ADC1);
    adc_power_off(ADC1);
    rcc_periph_reset_pulse(RST_ADC1);

    adc_set_clk_prescale(ADC1, ADC_CCR_ADCPRE_BY2);
    adc_disable_scan_mode(ADC1);
    adc_set_single_conversion_mode(ADC1);
    adc_set_sample_time(ADC1, ADC_CHANNEL0, ADC_SMPR_SMP_56CYC);
    adc_power_on(ADC1);
    adc_start_conversion_regular(ADC1);
    while (!adc_eoc(ADC1));
    /// @attention Некрасивый способ подавить предупреждение о возможной потере точности, подумать о решении
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wfloat-conversion"
    /// @todo Протестировать тщательно возможность простого вызова функции чтения, без присваивания результата
    [[maybe_unused]] volatile uint16_t res = static_cast<float>(adc_read_regular(ADC1));
#pragma GCC diagnostic pop


