// RGB_LED.h
//#pragma once

#define LED_STROBE_INTERVAL        (525)
#define COLOR_BUFFER_SIZE		   (16)

enum RGB
{
	RED = 1,
	BLUE = 2,
	GREEN = 3,
	PURPLE = 4,
	TEAL = 5,
};

#ifndef _RGB_LED_h
#define _RGB_LED_h

#if defined(ARDUINO) && ARDUINO >= 100
#include "arduino.h"
#else
#include "WProgram.h"
#endif
#if defined _RGB_LED_CPP
uint16_t getInterval();
#endif // _RGB_LED_CPP
extern void strobeQue(uint8_t);
extern void RBG_LED(RGB, bool);
extern void strobe_LED_RGB(void);
extern void strobe_LED(RGB);
extern void fast_strobe_LED(RGB);
extern uint16_t getInterval();
extern void strobeQue(RGB);
#endif

