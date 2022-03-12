// RGB_LED.h
#pragma once
#define RED                       (1)
#define BLUE                      (2)
#define GREEN                     (3)
#define LED_STROBE_INTERVAL       (500)

#ifndef _RGB_LED_h
#define _RGB_LED_h

#if defined(ARDUINO) && ARDUINO >= 100
#include "arduino.h"
#else
#include "WProgram.h"
#endif
extern void RBG_LED(uint8_t, bool);
extern void strobe_LED_RGB(void);
extern void strobe_LED(uint8_t);
extern void fast_strobe_LED(uint8_t);
#endif

