/**
 * WS2812B (NeoPixel-compatible) driver for the 6-LED daisy chain (D5-D10)
 * on PA8, driven via TIM1_CH1 PWM + DMA (see MX_TIM1_Init in main.c: this
 * relies on TIM1's Period=99 giving a 100-tick/800kHz bit period).
 */
#ifndef WS2812_H
#define WS2812_H

#include "main.h"

#define WS2812_NUM_LEDS  6u

/* Sets all LEDs to the same solid color and blocks (a few hundred us) until
 * the DMA transfer to TIM1_CH1 completes. Call after MX_TIM1_Init(). */
void WS2812_SetAll(uint8_t r, uint8_t g, uint8_t b);

/* Turns all LEDs off. */
void WS2812_Init(void);

#endif /* WS2812_H */
