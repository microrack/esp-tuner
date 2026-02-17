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

        // GPIO configuration
        gpio_config_t io_conf = {};
        io_conf.pin_bit_mask = (1ULL << pin);
        io_conf.mode = GPIO_MODE_INPUT;
        io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
        io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
        io_conf.intr_type = GPIO_INTR_POSEDGE;
        ESP_ERROR_CHECK(gpio_config(&io_conf));

        // Set interrupt handler
        gpio_install_isr_service(0);
        gpio_isr_handler_add((gpio_num_t)pin, gpio_isr_handler, this);
    }
    
    bool wait() {
        return buffer_count >= BUFFER_SIZE;
    }

    // Calculate frequency using median filter
    double read() {
        if (buffer_count < BUFFER_SIZE) return 0.0;
        
        // Copy buffer for sorting (ISR may continue writing)
        uint32_t sorted_buffer[BUFFER_SIZE];
        size_t count = buffer_count;
        
        // Copy data from volatile buffer
        for (size_t i = 0; i < count && i < BUFFER_SIZE; ++i) {
            sorted_buffer[i] = buffer[i];
        }
        
        if (count == 0) return 0.0;
        
        // Sort for median filter
        std::sort(sorted_buffer, sorted_buffer + count);
        
        // Take median (middle element)
        uint32_t median_cycles;
        if (count % 2 == 0) {
            // Even count - take average of two central elements
            median_cycles = (sorted_buffer[count / 2 - 1] + sorted_buffer[count / 2]) / 2;
        } else {
            // Odd count - take central element
            median_cycles = sorted_buffer[count / 2];
        }
        
        // Get actual CPU frequency in MHz
        uint32_t cpu_freq_mhz = getCpuFrequencyMhz();
        
        // Filter out periods that are too large (more than 1 sec)
        uint64_t max_cycles = (uint64_t)cpu_freq_mhz * 1000000ULL;
        if (median_cycles > max_cycles) {
            return 0.0;
        }
        
        // Convert cycles to microseconds
        double median_period_us = (double)median_cycles / (double)cpu_freq_mhz;
        
        // Convert microseconds to seconds and calculate frequency
        double median_period_sec = median_period_us / 1000000.0;
        double frequency = (median_period_sec > 0) ? (1.0 / median_period_sec) : 0.0;
        
        return frequency;
    }

    void reset() {
        buffer_count = 0;
        last_time = 0;
    }

private:
    // GPIO interrupt handler
    static void IRAM_ATTR gpio_isr_handler(void* arg) {
        FreqCapture* self = static_cast<FreqCapture*>(arg);
        if (!self) return;

        // Get current time in CPU cycles (runs in IRAM)
        uint64_t current_cycles = esp_cpu_get_cycle_count();
        
        if (self->last_time > 0) {
            // Calculate period between interrupts in cycles
            uint64_t period_cycles = current_cycles - self->last_time;
            
            // Filter out periods that are too short (noise protection)
            // Minimum 80 cycles (1 μs at 80 MHz)
            if (period_cycles >= 80) {
                // Add to buffer if there's space
                size_t idx = self->buffer_count;
                if (idx < BUFFER_SIZE) {
                    self->buffer[idx] = (uint32_t)period_cycles;
                    // Increment counter atomically
                    size_t new_count = idx + 1;
                    self->buffer_count = new_count;
                }
                // If buffer is full, simply don't add new elements
            }
        }
        
        self->last_time = current_cycles;
    }

private:
    int         pin;
    
    volatile uint64_t last_time = 0;
    volatile uint32_t buffer[BUFFER_SIZE];  // Buffer of periods in CPU cycles
    volatile size_t buffer_count = 0;        // Number of elements in buffer
};
