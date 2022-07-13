// 
// 
// 
#pragma once
#include "RGB_LED.h"

#ifndef _RGB_LED_CPP
#define _RGB_LED_CPP

static bool cycleLED = true;
static uint32_t LEDTimer = 0;
static uint16_t strobeSpeed = 500;

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
    static uint8_t oldColor = 0;
    int temp = 0;
    if (currentTime - LEDTimer > getInterval() )//LED_STROBE_INTERVAL
    {
        if (cycleLED)
        {
            //Serial.print("getInterval(): ");
            //Serial.println(getInterval());
            //Serial.print("Stack Size: ");
            //Serial.println(colorStackSize());
            if (colorStackSize() > 0)
            {
                //Serial.print("Color ");
                temp = strobeQuene[colorBufferOut()];
                //Serial.println(temp);
                //RBG_LED((RGB)strobeQuene[colorBufferOut()], cycleLED);
                RBG_LED((RGB)temp, true);
            }
            else
            {
                RBG_LED(color, cycleLED);
            }
        }
        else
        {
            digitalWrite(GPIO_NUM_16, HIGH);
            digitalWrite(GPIO_NUM_17, HIGH);
            digitalWrite(GPIO_NUM_32, HIGH);
            digitalWrite(GPIO_NUM_33, HIGH);
            digitalWrite(GPIO_NUM_15, HIGH);
            digitalWrite(GPIO_NUM_14, HIGH);
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
    // Rolling it out instead of multiplication to observe if there is any performance benefit
    switch (colorStackSize())
    {
    case 0:
        return LED_STROBE_INTERVAL;
    case 1:
        return 492;
    case 2:
        return 459;
    case 3:
        return 426;
    case 4:
        return 393;
    case 5:
        return 360;
    case 6:
        return 327;
    case 7:
        return 294;
    case 8:
        return 261;
    case 9:
        return 228;
    case 10:
        return 195;
    case 11:
        return 162;
    case 12:
        return 129;
    case 13:
        return 96;
    case 14:
        return 63;
    case 15:
        return 20;
    case 16: // Incase I adjust something with the buffer and hit this value
        return 10;
    default:
        return LED_STROBE_INTERVAL;
    }

    /*
    if (colorStackSize() > 0)
    {
        return LED_STROBE_INTERVAL - (colorStackSize() * 33);
    }
    else
    {
        return LED_STROBE_INTERVAL;
    }
    */
}

#endif // !_RGB_LED_CPP