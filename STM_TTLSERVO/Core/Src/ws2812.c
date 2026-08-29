#include "ws2812.h"

extern TIM_HandleTypeDef htim1;

/* TIM1 runs at Prescaler=0, Period=99 -> 100 timer ticks per bit period
 * (80 MHz / 100 = 800 kHz), matching the WS2812B ~1.25us bit period.
 * CCR values below are in those same ticks (12.5ns each). */
#define WS2812_T0H  32u  /* ~400ns high  - "0" bit */
#define WS2812_T1H  64u  /* ~800ns high  - "1" bit */

#define WS2812_BITS_PER_LED  24u
/* >=50us of low output terminates the frame (WS2812B reset/latch time);
 * 50 zero-duty slots = 50 * 1.25us = 62.5us, comfortably over that. */
#define WS2812_RESET_SLOTS   50u
#define WS2812_BUF_LEN  (WS2812_NUM_LEDS * WS2812_BITS_PER_LED + WS2812_RESET_SLOTS)

static uint16_t s_buf[WS2812_BUF_LEN];
static volatile uint8_t s_dma_done = 1u;

static void WS2812_EncodeByte(uint16_t *dst, uint8_t byte)
{
    for (uint8_t bit = 0; bit < 8u; bit++) {
        uint8_t mask = (uint8_t)(0x80u >> bit); /* WS2812B is MSB-first */
        dst[bit] = (byte & mask) ? WS2812_T1H : WS2812_T0H;
    }
}

void WS2812_Init(void)
{
    WS2812_SetAll(0u, 0u, 0u);
}

void WS2812_SetAll(uint8_t r, uint8_t g, uint8_t b)
{
    uint32_t start;

    for (uint8_t led = 0; led < WS2812_NUM_LEDS; led++) {
        uint16_t *p = &s_buf[led * WS2812_BITS_PER_LED];
        /* WS2812B wire order is Green, Red, Blue. */
        WS2812_EncodeByte(p, g);
        WS2812_EncodeByte(p + 8, r);
        WS2812_EncodeByte(p + 16, b);
    }
    for (uint16_t i = WS2812_NUM_LEDS * WS2812_BITS_PER_LED; i < WS2812_BUF_LEN; i++) {
        s_buf[i] = 0u;
    }

    s_dma_done = 0u;
    HAL_TIM_PWM_Start_DMA(&htim1, TIM_CHANNEL_1, (uint32_t *)s_buf, WS2812_BUF_LEN);

    /* Transfer is ~240us; 20ms is just a generous safety bound so a DMA
     * glitch can never hang the caller forever. */
    start = HAL_GetTick();
    while (!s_dma_done) {
        if ((HAL_GetTick() - start) > 20u) {
            break;
        }
    }
    HAL_TIM_PWM_Stop_DMA(&htim1, TIM_CHANNEL_1);
}

/**
  * @brief TIM PWM pulse-finished callback - fires when the DMA transfer to
  * TIM1_CH1's CCR register completes.
  */
void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM1 && htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1) {
        s_dma_done = 1u;
    }
}
