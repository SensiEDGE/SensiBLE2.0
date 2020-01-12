#ifndef MAIN_H_
#define MAIN_H_

#include <stdint.h>

#define LED_WITH_GPIO 0
#define LED_BLINK_IN_SYSTICK 0

#define LOOP_BLINK_AFTER_CLOCK_INIT 0
#define LOOP_BLINK_AFTER_CLOCK_INIT_DED_BLINK 0

#define LOOP_BLINK_AFTER_BLUENRG_INIT 0

#define USE_REAL_SLEEPMODE_WAKETIMER 0

uint32_t tick_ms(void);
void DelayMs(volatile uint32_t lTimeMs);

#endif /*MAIN_H_*/
