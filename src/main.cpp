#include <Arduino.h>
#include <cstdint>
#include <SPI.h>
#include <Wire.h>
#include <nvs_flash.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_NeoPixel.h>
#include "sigscoper.h"

// OLED display configuration
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
#define SCREEN_ADDRESS 0x3C

#define OUT_CHANNEL_A_PIN 26
#define PWM_FREQ 32768
#define PWM_RESOLUTION 10
const uint32_t PWM_MAX_VAL = (1 << PWM_RESOLUTION) - 1;

const int ADC_0 = 36;

// Create display object
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

void setup() {
    pinMode(12, OUTPUT);
    pinMode(13, OUTPUT);

    Serial.begin(115200);

    Serial.printf("setup\n");

    // Initialize display
    if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
        Serial.println(F("SSD1306 allocation failed"));
        for(;;);
    }
    display.setRotation(2);
    display.clearDisplay();
    display.display();
}

void loop() {
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.println("Hello, world!");
    display.display();
    delay(1000);
}