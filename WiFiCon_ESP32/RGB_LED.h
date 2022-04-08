// RGB_LED.h
//#pragma once
#define RED                       (1)
#define BLUE                      (2)
#define GREEN                     (3)
#define LED_STROBE_INTERVAL       (525)
#define COLOR_BUFFER_SIZE		  (16)

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
extern void RBG_LED(uint8_t, bool);
extern void strobe_LED_RGB(void);
extern void strobe_LED(uint8_t);
extern void fast_strobe_LED(uint8_t);
extern uint16_t getInterval();
extern void strobeQue(uint8_t);
extern void decreaseStrobeSpeed();
#endif

