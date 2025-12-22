/*
 * stm32f407xx_gpio_driver.h
 *
 *  Created on: Dec 3, 2025
 *      Author: Gleed
 */

#ifndef INC_STM32F407XX_GPIO_DRIVER_H_
#define INC_STM32F407XX_GPIO_DRIVER_H_


#include "stm32f407xx.h"

typedef struct
{
    uint8_t GPIO_PinNumber;
    uint8_t GPIO_PinMode; //possible values from @GPIO_PIN_MODES
    uint8_t GPIO_PinSpeed; //possible values from @GPIO_PIN_SPEEDS
    uint8_t GPIO_PuPdProtocol;
    uint8_t GPIO_PinOPType;
    uint8_t GPIO_PinAltFunMode;
} GPIO_PinConfig;

typedef struct
{
    GPIO_RegDef *pGPIOx;
    GPIO_PinConfig GPIO_PinConfig;
} GPIO_handle;


#define GPIO_PIN_NO_0 0
#define GPIO_PIN_NO_1 1
#define GPIO_PIN_NO_2 2
#define GPIO_PIN_NO_3 3
#define GPIO_PIN_NO_4 4
#define GPIO_PIN_NO_5 5
#define GPIO_PIN_NO_6 6
#define GPIO_PIN_NO_7 7
#define GPIO_PIN_NO_8 8
#define GPIO_PIN_NO_9 9
#define GPIO_PIN_NO_10 10
#define GPIO_PIN_NO_11 11
#define GPIO_PIN_NO_12 12
#define GPIO_PIN_NO_13 13
#define GPIO_PIN_NO_14 14
#define GPIO_PIN_NO_15 15

// @GPIO_PIN_MODES
#define GPIO_MODE_IN 0
#define GPIO_MODE_OUT 1
#define GPIO_MODE_ALTFN 2
#define GPIO_MODE_ANALOG 3
#define GPIO_MODE_IT_FT 4
#define GPIO_MODE_IT_RT 5
#define GPIO_MODE_IT_RFT 6

#define GPIO_OP_TYPE_PP 0
#define GPIO_OP_TYPE_OD 1

// @GPIO_PIN_SPEEDS
#define GPIO_SPEED_LOW 0
#define GPIO_SPEED_MEDIUM 1
#define GPIO_SPEED_FAST 2
#define GPIO_SPEED_HIGH 3

#define GPIO_PIN_NO_PUPD 0
#define GPIO_PIN_PU 1
#define GPIO_PIN_PD 2


void GPIO_PClkControl(GPIO_RegDef *pGPIOx, uint8_t enable);

void GPIO_Init(GPIO_handle *pGPIOHandle);
void GPIO_DeInit(GPIO_RegDef *pGPIOx);

uint8_t GPIO_ReadFromInputPin(GPIO_RegDef *pGPIOx, uint8_t pinNumber);
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef *pGPIOx);
void GPIO_WriteToOutputPin(GPIO_RegDef *pGPIOx, uint8_t pinNumber, uint8_t value);
void GPIO_WriteToOutputPort(GPIO_RegDef *pGPIOx, uint16_t value);
void GPIO_ToggleOutputPin(GPIO_RegDef *pGPIOx, uint8_t pinNumber);

void GPIO_IRQConfig(uint8_t irqNumber, uint8_t irqPriority, uint8_t enable);
void GPIO_IRQHandling(uint8_t pinNumber);

#endif /* INC_STM32F407XX_GPIO_DRIVER_H_ */
