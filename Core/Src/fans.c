#include "main.h"

HAL_StatusTypeDef MX_PWM_Init(TIM_HandleTypeDef *htim, uint32_t Channel, uint32_t Prescaler, uint32_t Period)
{
    TIM_OC_InitTypeDef sConfigOC = {0};

    // 1. Assign Base Timer Settings
    // htim->Instance must already be set (e.g., htim->Instance = TIM2) before calling this
    if (htim->Instance == NULL)
    {
        return HAL_ERROR;
    }

    htim->Init.Prescaler         = Prescaler;
    htim->Init.CounterMode       = TIM_COUNTERMODE_UP;
    htim->Init.Period            = Period;
    htim->Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    htim->Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE; // Smooth duty cycle transitions

    // Initialize the Timer base for PWM mode
    if (HAL_TIM_PWM_Init(htim) != HAL_OK)
    {
        return HAL_ERROR;
    }

    // 2. Configure the PWM Channel Properties
    sConfigOC.OCMode       = TIM_OCMODE_PWM1;       // Edge-aligned PWM
    sConfigOC.Pulse        = 0;                     // Start at 0% duty cycle (Fan OFF)
    sConfigOC.OCPolarity   = TIM_OCPOLARITY_HIGH;   // Active High
    sConfigOC.OCFastMode   = TIM_OCFAST_DISABLE;

    if (HAL_TIM_PWM_ConfigChannel(htim, &sConfigOC, Channel) != HAL_OK)
    {
        return HAL_ERROR;
    }

    // 3. Start the Hardware PWM Generation
    if (HAL_TIM_PWM_Start(htim, Channel) != HAL_OK)
    {
        return HAL_ERROR;
    }

    return HAL_OK;
}

void Set_Fan_DutyCycle(TIM_HandleTypeDef *htim, uint32_t Channel, uint8_t DutyCycle)
{
    // Get the Auto-Reload Register (ARR) value to scale the percentage correctly
    uint32_t arr = __HAL_TIM_GET_AUTORELOAD(htim);
    uint32_t dc = ((uint32_t)DutyCycle * 4294967295U) / 255U;
    uint32_t pulse = (dc / (arr + 1)) * 100;

    // Update the Compare register safely
    __HAL_TIM_SET_COMPARE(htim, Channel, pulse);
}