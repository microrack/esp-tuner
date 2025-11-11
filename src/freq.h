#pragma once

#include "driver/gpio.h"
#include "esp_err.h"
#include "esp_cpu.h"
#include "soc/soc.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp32-hal-cpu.h"
#include <algorithm>
#include <cmath>

template<size_t BUFFER_SIZE>
class FreqCapture {
public:
    explicit FreqCapture(int pin)
        : pin(pin) {
        last_time = 0;
        buffer_count = 0;
    }

    void begin() {
        buffer_count = 0;
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
        return buffer_count >= BUFFER_SIZE;
    }

    // Вычисление частоты с применением медианного фильтра
    double read() {
        if (buffer_count < BUFFER_SIZE) return 0.0;
        
        // Копируем буфер для сортировки (ISR может продолжать писать)
        uint32_t sorted_buffer[BUFFER_SIZE];
        size_t count = buffer_count;
        
        // Копируем данные из volatile буфера
        for (size_t i = 0; i < count && i < BUFFER_SIZE; ++i) {
            sorted_buffer[i] = buffer[i];
        }
        
        if (count == 0) return 0.0;
        
        // Сортируем для медианного фильтра
        std::sort(sorted_buffer, sorted_buffer + count);
        
        // Берем медиану (средний элемент)
        uint32_t median_cycles;
        if (count % 2 == 0) {
            // Четное количество - берем среднее двух центральных
            median_cycles = (sorted_buffer[count / 2 - 1] + sorted_buffer[count / 2]) / 2;
        } else {
            // Нечетное количество - берем центральный элемент
            median_cycles = sorted_buffer[count / 2];
        }
        
        // Получаем реальную частоту CPU в МГц
        uint32_t cpu_freq_mhz = getCpuFrequencyMhz();
        
        // Фильтруем слишком большие периоды (больше 1 сек)
        uint64_t max_cycles = (uint64_t)cpu_freq_mhz * 1000000ULL;
        if (median_cycles > max_cycles) {
            return 0.0;
        }
        
        // Конвертируем циклы в микросекунды
        double median_period_us = (double)median_cycles / (double)cpu_freq_mhz;
        
        // Конвертируем микросекунды в секунды и вычисляем частоту
        double median_period_sec = median_period_us / 1000000.0;
        double frequency = (median_period_sec > 0) ? (1.0 / median_period_sec) : 0.0;
        
        return frequency;
    }

    void reset() {
        buffer_count = 0;
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
            if (period_cycles >= 80) {
                // Добавляем в буфер, если есть место
                size_t idx = self->buffer_count;
                if (idx < BUFFER_SIZE) {
                    self->buffer[idx] = (uint32_t)period_cycles;
                    // Увеличиваем счетчик атомарно
                    size_t new_count = idx + 1;
                    self->buffer_count = new_count;
                }
                // Если буфер полон, просто не добавляем новые элементы
            }
        }
        
        self->last_time = current_cycles;
    }

private:
    int         pin;
    
    volatile uint64_t last_time = 0;
    volatile uint32_t buffer[BUFFER_SIZE];  // Буфер периодов в циклах CPU
    volatile size_t buffer_count = 0;        // Количество элементов в буфере
};
