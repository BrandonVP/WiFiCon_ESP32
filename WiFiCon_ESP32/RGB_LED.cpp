// 
// 
// 
#pragma once
#include "RGB_LED.h"

#ifndef _RGB_LED_CPP
#define _RGB_LED_CPP

bool cycleLED = true;
uint32_t LEDTimer = 0;

// Set color and on/off status of RGB LED
void RBG_LED(uint8_t color, bool isOn)
{
    if (color == RED)
    {
        digitalWrite(GPIO_NUM_15, HIGH);
        digitalWrite(GPIO_NUM_16, HIGH);
        digitalWrite(GPIO_NUM_17, HIGH);

        digitalWrite(GPIO_NUM_32, HIGH);
        digitalWrite(GPIO_NUM_33, HIGH);
        digitalWrite(GPIO_NUM_14, HIGH);

        (isOn) ? digitalWrite(GPIO_NUM_17, LOW) : digitalWrite(GPIO_NUM_17, HIGH);
        (isOn) ? digitalWrite(GPIO_NUM_14, LOW) : digitalWrite(GPIO_NUM_14, HIGH);
    }
    if (color == BLUE)
    {
        digitalWrite(GPIO_NUM_15, HIGH);
        digitalWrite(GPIO_NUM_16, HIGH);
        digitalWrite(GPIO_NUM_17, HIGH);

        digitalWrite(GPIO_NUM_32, HIGH);
        digitalWrite(GPIO_NUM_33, HIGH);
        digitalWrite(GPIO_NUM_14, HIGH);

        (isOn) ? digitalWrite(GPIO_NUM_15, LOW) : digitalWrite(GPIO_NUM_15, HIGH);
        (isOn) ? digitalWrite(GPIO_NUM_32, LOW) : digitalWrite(GPIO_NUM_32, HIGH);
    }
    if (color == GREEN)
    {
        digitalWrite(GPIO_NUM_15, HIGH);
        digitalWrite(GPIO_NUM_16, HIGH);
        digitalWrite(GPIO_NUM_17, HIGH);

        digitalWrite(GPIO_NUM_32, HIGH);
        digitalWrite(GPIO_NUM_33, HIGH);
        digitalWrite(GPIO_NUM_14, HIGH);

        (isOn) ? digitalWrite(GPIO_NUM_16, LOW) : digitalWrite(GPIO_NUM_16, HIGH);
        (isOn) ? digitalWrite(GPIO_NUM_33, LOW) : digitalWrite(GPIO_NUM_33, HIGH);
    }
}

// Strobe Red Blue Green (RBG) colors
void strobe_LED_RGB(void)
{
    // Cycles values 1-3 for the RGB colors
    static uint8_t color = RED;

    uint32_t currentTime = millis();

    if (currentTime - LEDTimer > LED_STROBE_INTERVAL)
    {
        RBG_LED(color, true);
        (color > BLUE) ? color = RED : color++;
        LEDTimer = currentTime;
    }
}

// Strobe Red Blue Green (RBG) colors
void strobe_LED(uint8_t color)
{
    uint32_t currentTime = millis();

    if (currentTime - LEDTimer > LED_STROBE_INTERVAL)
    {
        RBG_LED(color, cycleLED);
        cycleLED = !cycleLED;
        LEDTimer = currentTime;
    }
}

// Strobe Red Blue Green (RBG) colors
void fast_strobe_LED(uint8_t color)
{
    uint32_t currentTime = millis();

    if (currentTime - LEDTimer > (LED_STROBE_INTERVAL / 2))
    {
        RBG_LED(color, cycleLED);
        cycleLED = !cycleLED;
        LEDTimer = currentTime;
    }
}

struct strobeLEDs
{
    uint8_t color;
    uint8_t interval;
};

uint8_t strobeQuene[16];
uint8_t strobeQuePtr = 0;

void strobeQue(uint8_t color)
{
    strobeQuePtr++;
    strobeQuene[strobeQuePtr] = color;
}

uint16_t getInterval()
{
    if (strobeQuePtr > 0)
    {
        strobeQuePtr--;
    }
    if (strobeQuePtr > 0)
    {
        return LED_STROBE_INTERVAL - ((strobeQuePtr + 1) * 28);
    }
    else
    {
        return LED_STROBE_INTERVAL;
    }
}

#endif // !_RGB_LED_CPP