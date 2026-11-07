// SPDX-License-Identifier: GPL-3.0-or-later
#if defined(TTIMU_F411_FC_EXAMPLE)
#include <Arduino.h>

#if HSE_VALUE != 8000000U
#error "This FC profile requires an 8 MHz crystal."
#endif

// Strong override of the BlackPill variant's weak 25 MHz clock configuration.
// HSE 8 MHz / M=8 * N=192: VCO 192 MHz, CPU /2=96 MHz, USB /4=48 MHz.
extern "C" void SystemClock_Config(void) {
    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    RCC_OscInitTypeDef oscillator = {};
    oscillator.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    oscillator.HSEState = RCC_HSE_ON;
    oscillator.PLL.PLLState = RCC_PLL_ON;
    oscillator.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    oscillator.PLL.PLLM = 8;
    oscillator.PLL.PLLN = 192;
    oscillator.PLL.PLLP = RCC_PLLP_DIV2;
    oscillator.PLL.PLLQ = 4;
    if (HAL_RCC_OscConfig(&oscillator) != HAL_OK) Error_Handler();

    RCC_ClkInitTypeDef clocks = {};
    clocks.ClockType = RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK |
                       RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    clocks.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    clocks.AHBCLKDivider = RCC_SYSCLK_DIV1;
    clocks.APB1CLKDivider = RCC_HCLK_DIV2;
    clocks.APB2CLKDivider = RCC_HCLK_DIV1;
    if (HAL_RCC_ClockConfig(&clocks, FLASH_LATENCY_3) != HAL_OK) Error_Handler();
}
#endif
