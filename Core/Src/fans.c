#include "main.h"
#include "FreeRTOS.h"
#include "stm32f429xx.h"
#include "stm32f4xx_hal_tim.h"

const int DEFAULT_PRESCALER = 89;    // Divide 90 MHz clock by 90 (1 MHz tick)
const int DEFAULT_PERIOD = 1000;     // 1000 ticks = 1 kHz PWM frequency
const int DEFAULT_DUTY_CYCLE = 250;  // 250 ticks HIGH = 25% duty cycle

void PWM_Init(TIM_HandleTypeDef *htim, 
              TIM_OC_InitTypeDef *config,
              int *timerNumber,
              int *timerChannel) {
    // 1. Initialize Time Base
    htim.Instance = timerNumber;
    htim.Init.Prescaler = DEFAULT_PRESCALER;           
    htim.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim.Init.Period = DEFAULT_PERIOD - 1;       
    htim.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    HAL_TIM_PWM_Init(&htim);

    // 2. Configure PWM Channel
    config.OCMode = TIM_OCMODE_PWM1;
    config.Pulse = DEFAULT_DUTY_CYCLE;             
    config.OCPolarity = TIM_OCPOLARITY_HIGH;
    config.OCFastMode = TIM_OCFAST_DISABLE;
    HAL_TIM_PWM_ConfigChannel(&htim, config, timerChannel);
    
    // 3. Start the PWM Generation
    HAL_TIM_PWM_Start(&htim, timerChannel);
}

void PWM_Update(TIM_HandleTypeDef *htim,
                int *channel,
                int *dutyCycle) {
    __HAL_TIM_SET_COMPARE(&htim, &channel, &dutyCycle)
}