#include <Arduino.h>
#include <cstdint>
#include <SPI.h>
#include <Wire.h>
#include <nvs_flash.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_NeoPixel.h>
#include "freq.h"

// OLED display configuration
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
#define SCREEN_ADDRESS 0x3C

#define OUT_CHANNEL_A_PIN 13
#define PWM_FREQ 8192
#define PWM_RESOLUTION 10
const uint32_t PWM_MAX_VAL = (1 << PWM_RESOLUTION) - 1;

const int ADC_0 = 12;

static constexpr float PWM_NOTE_SCALE = (1 << PWM_RESOLUTION) / (12 * 10.99); // 10.99 Vpp, 12 notes per octave (1 V/oct)
static const int PWM_ZERO_OFFSET = 498; // 0 V at MIDDLE_NOTE
static const int MIDDLE_NOTE = 60; // C4 (middle C)

// Create display object
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

FreqCapture freq(ADC_0);

void set_note_out(uint8_t note) {
    int v = (note - MIDDLE_NOTE) * PWM_NOTE_SCALE + PWM_ZERO_OFFSET;
    if (v > int(PWM_MAX_VAL)) return;

    ledcWrite(OUT_CHANNEL_A_PIN, v);
}


void setup() {
    // pinMode(12, OUTPUT);
    // pinMode(13, OUTPUT);

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
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);

    ledcAttach(OUT_CHANNEL_A_PIN, PWM_FREQ, PWM_RESOLUTION);

    freq.begin();
    freq.reset();

    Serial.printf("freq.begin()\n");
}

void loop() {
    set_note_out(60);
    delay(50);

    uint32_t timeout = 0;
    while(!freq.wait() && timeout < 1000) {
        timeout += 10;
        delay(10);
    }

    if (timeout >= 1000) {
        Serial.printf("timeout\n");
        return;
    }

    float frequency = freq.read();

    display.clearDisplay();
    display.setCursor(0, 0);
    display.print("Freq: ");
    display.println(frequency);
    display.display();

    freq.reset();

    delay(20);

    /*
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.println("Set note 72");
    display.display();

    set_note_out(72);
    delay(1000);
    */
}