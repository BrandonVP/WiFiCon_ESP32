// 
// 
// 
#pragma once
#include "RGB_LED.h"

#ifndef _RGB_LED_CPP
#define _RGB_LED_CPP

bool cycleLED = true;
uint32_t LEDTimer = 0;
uint16_t strobeSpeed = 500;

/*=========================================================
            Circular Buffer
===========================================================*/
uint8_t colorBufferInPtr = 0;
uint8_t colorBufferOutPtr = 0;
uint8_t strobeQuene[16];

// LED color in
uint8_t colorBufferIn()
{
    uint8_t temp = colorBufferInPtr;
    (colorBufferInPtr < COLOR_BUFFER_SIZE - 1) ? colorBufferInPtr++ : colorBufferInPtr = 0;
    if (colorBufferInPtr == colorBufferOutPtr)
    {
        (colorBufferOutPtr < (COLOR_BUFFER_SIZE - 1)) ? colorBufferOutPtr++ : colorBufferOutPtr = 0;
    }
    return temp;
}

// LED color out
uint8_t colorBufferOut()
{
    uint8_t temp = colorBufferOutPtr;
    (colorBufferOutPtr < COLOR_BUFFER_SIZE - 1) ? colorBufferOutPtr++ : colorBufferOutPtr = 0;
    return temp;
}

// Calculates current structures in buffer by subtracting points then anding with max buffer size value
uint8_t colorStackSize()
{
    uint8_t size = (colorBufferInPtr - colorBufferOutPtr) & (COLOR_BUFFER_SIZE - 1);
    return size;
}

/*=========================================================
            RGB LED
===========================================================*/
// Increase speed to match traffic
void decreaseStrobeSpeed()
{
    if (strobeSpeed > 10)
    {
        strobeSpeed = strobeSpeed - 10;
    }
}

// Set color and on/off status of RGB LED
void RBG_LED(RGB color, bool isOn)
{
    if (color == BLUE)
    {
        digitalWrite(GPIO_NUM_16, HIGH);
        digitalWrite(GPIO_NUM_17, HIGH);
        digitalWrite(GPIO_NUM_32, HIGH);
        digitalWrite(GPIO_NUM_33, HIGH);
        
        (isOn) ? digitalWrite(GPIO_NUM_15, LOW) : digitalWrite(GPIO_NUM_15, HIGH);
        (isOn) ? digitalWrite(GPIO_NUM_14, LOW) : digitalWrite(GPIO_NUM_14, HIGH);
    }
    if (color == GREEN)
    {
        digitalWrite(GPIO_NUM_15, HIGH);
        digitalWrite(GPIO_NUM_17, HIGH);
        digitalWrite(GPIO_NUM_14, HIGH);
        digitalWrite(GPIO_NUM_33, HIGH);

        (isOn) ? digitalWrite(GPIO_NUM_16, LOW) : digitalWrite(GPIO_NUM_16, HIGH);
        (isOn) ? digitalWrite(GPIO_NUM_32, LOW) : digitalWrite(GPIO_NUM_32, HIGH);
    }
    if (color == RED)
    {
        digitalWrite(GPIO_NUM_15, HIGH);
        digitalWrite(GPIO_NUM_16, HIGH);
        digitalWrite(GPIO_NUM_14, HIGH);
        digitalWrite(GPIO_NUM_32, HIGH);

        (isOn) ? digitalWrite(GPIO_NUM_17, LOW) : digitalWrite(GPIO_NUM_17, HIGH);
        (isOn) ? digitalWrite(GPIO_NUM_33, LOW) : digitalWrite(GPIO_NUM_33, HIGH);
    }
    if (color == TEAL)
    {
        digitalWrite(GPIO_NUM_17, HIGH);
        digitalWrite(GPIO_NUM_33, HIGH);

        (isOn) ? digitalWrite(GPIO_NUM_15, LOW) : digitalWrite(GPIO_NUM_15, HIGH);
        (isOn) ? digitalWrite(GPIO_NUM_16, LOW) : digitalWrite(GPIO_NUM_16, HIGH);
        (isOn) ? digitalWrite(GPIO_NUM_14, LOW) : digitalWrite(GPIO_NUM_14, HIGH);
        (isOn) ? digitalWrite(GPIO_NUM_32, LOW) : digitalWrite(GPIO_NUM_32, HIGH);
    }
    if (color == PURPLE)
    {
        digitalWrite(GPIO_NUM_16, HIGH);
        digitalWrite(GPIO_NUM_32, HIGH);

        (isOn) ? digitalWrite(GPIO_NUM_15, LOW) : digitalWrite(GPIO_NUM_15, HIGH);
        (isOn) ? digitalWrite(GPIO_NUM_17, LOW) : digitalWrite(GPIO_NUM_17, HIGH);
        (isOn) ? digitalWrite(GPIO_NUM_14, LOW) : digitalWrite(GPIO_NUM_14, HIGH);
        (isOn) ? digitalWrite(GPIO_NUM_33, LOW) : digitalWrite(GPIO_NUM_33, HIGH);
    }
}

// Strobe Red Blue Green (RBG) colors
void strobe_LED_RGB(void)
{
    // Cycles values 1-3 for the RGB colors
    static uint8_t color = RED;

    uint32_t currentTime = millis();

    if (currentTime - LEDTimer > LED_STROBE_INTERVAL)//LED_STROBE_INTERVAL)
    {
        RBG_LED((RGB)color, true);
        (color > BLUE) ? color = RED : color++;
        LEDTimer = currentTime;
    }
}

// Strobe Red Blue Green (RBG) colors
void strobe_LED(RGB color)
{
    uint32_t currentTime = millis();

    if (currentTime - LEDTimer > getInterval() )//LED_STROBE_INTERVAL
    {
        //Serial.print("getInterval(): ");
        //Serial.println(getInterval());
        //Serial.print("Stack Size: ");
        //Serial.println(colorStackSize());
        if (colorStackSize() > 0)
        {
            //Serial.print("Color ");
            //Serial.println(strobeQuene[colorBufferOut()]);
            RBG_LED((RGB)strobeQuene[colorBufferOut()], cycleLED);
            //RBG_LED((RGB)BLUE, cycleLED);
        }
        else
        {
            RBG_LED(color, cycleLED);
        }
        cycleLED = !cycleLED;
        LEDTimer = currentTime;
    }
}

// Add color to buffer
void strobeQue(RGB color)
{
    strobeQuene[colorBufferIn()] = color;
}

// Strobe Red Blue Green (RBG) colors
void fast_strobe_LED(RGB color)
{
    uint32_t currentTime = millis();

    if (currentTime - LEDTimer > (LED_STROBE_INTERVAL / 2))
    {
        RBG_LED(color, cycleLED);
        cycleLED = !cycleLED;
        LEDTimer = currentTime;
    }
}

// Calculate the current interval
uint16_t getInterval()
{
    if (colorStackSize() > 0)
    {
        return LED_STROBE_INTERVAL - (colorStackSize() * 33);
    }
    else
    {
        return LED_STROBE_INTERVAL;
    }
}

#endif // !_RGB_LED_CPP