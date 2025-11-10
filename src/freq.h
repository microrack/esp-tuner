#pragma once

#include "driver/gpio.h"
#include "esp_err.h"
#include "esp_cpu.h"
#include "soc/soc.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp32-hal-cpu.h"
#include <cmath>

class FreqCapture {
public:
    explicit FreqCapture(int pin, int average_samples = 20)
        : pin(pin), average_samples(average_samples) {
        last_time = 0;
        period_accumulator = 0;
        period_count = 0;
    }

    void begin() {
        period_accumulator = 0;
        period_count = 0;
        last_time = 0;

        // Настройка GPIO
        gpio_config_t io_conf = {};
        io_conf.pin_bit_mask = (1ULL << pin);
        io_conf.mode = GPIO_MODE_INPUT;
        io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
        io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
        io_conf.intr_type = GPIO_INTR_POSEDGE;
        ESP_ERROR_CHECK(gpio_config(&io_conf));

        // Установка обработчика прерывания
        gpio_install_isr_service(0);
        gpio_isr_handler_add((gpio_num_t)pin, gpio_isr_handler, this);
    }
    
    bool wait() {
        return period_count >= average_samples;
    }

    // Средняя частота по накопленным периодам
    double read() {
        if (!wait()) return 0.0;
        
        // Читаем значения (могут изменяться в ISR, но это безопасно для чтения)
        uint32_t acc = period_accumulator;
        uint32_t cnt = period_count;
        
        if (cnt == 0) return 0.0;
        
        // Получаем реальную частоту CPU в МГц
        uint32_t cpu_freq_mhz = getCpuFrequencyMhz();
        
        // Конвертируем циклы в микросекунды
        // cpu_freq_mhz МГц = cpu_freq_mhz циклов на микросекунду
        double mean_period_cycles = (double)acc / (double)cnt;
        
        // Фильтруем слишком большие периоды (больше 1 сек)
        uint64_t max_cycles = (uint64_t)cpu_freq_mhz * 1000000ULL;
        if (mean_period_cycles > (double)max_cycles) {
            return 0.0;
        }
        
        double mean_period_us = mean_period_cycles / (double)cpu_freq_mhz;
        
        // Конвертируем микросекунды в секунды и вычисляем частоту
        double mean_period_sec = mean_period_us / 1000000.0;
        double frequency = (mean_period_sec > 0) ? (1.0 / mean_period_sec) : 0.0;
        
        return frequency;
    }

    void reset() {
        period_accumulator = 0;
        period_count = 0;
        last_time = 0;
    }

private:
    // Обработчик прерывания GPIO
    static void IRAM_ATTR gpio_isr_handler(void* arg) {
        FreqCapture* self = static_cast<FreqCapture*>(arg);
        if (!self) return;

        // Получаем текущее время в циклах CPU (работает в IRAM)
        uint64_t current_cycles = esp_cpu_get_cycle_count();
        
        if (self->last_time > 0) {
            // Вычисляем период между прерываниями в циклах
            uint64_t period_cycles = current_cycles - self->last_time;
            
            // Фильтруем слишком короткие периоды (защита от шума)
            // Минимум 80 циклов (1 мкс при 80 МГц)
            // Максимум не проверяем в ISR для избежания проблем с литералами
            // Большие периоды отфильтруются при чтении
            if (period_cycles >= 80) {
                // Накопление в циклах, конвертацию делаем в read()
                // Используем явные присваивания для volatile переменных
                uint32_t period_cycles_32 = (uint32_t)period_cycles;
                uint32_t acc = self->period_accumulator;
                acc = acc + period_cycles_32;
                self->period_accumulator = acc;
                
                uint32_t cnt = self->period_count;
                cnt = cnt + 1;
                self->period_count = cnt;
            }
        }
        
        self->last_time = current_cycles;
    }

private:
    int         pin;
    const int   average_samples;
    
    volatile uint64_t last_time = 0;
    volatile uint32_t period_accumulator = 0;  // Накопление в циклах CPU
    volatile uint32_t period_count = 0;
};
