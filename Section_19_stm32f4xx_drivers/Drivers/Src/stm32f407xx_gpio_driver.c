/*
 * stm32f407xx_gpio_driver.c
 *
 *  Created on: Dec 3, 2025
 *      Author: Gleed
 */

#include "stm32f407xx_gpio_driver.h"


void GPIO_PClkControl(GPIO_RegDef *pGPIOx, uint8_t enable)
{
    if (enable)
    {
        if (pGPIOx == GPIOA)
            GPIOA_PCLK_EN;
        else if (pGPIOx == GPIOB)
            GPIOB_PCLK_EN;
        else if (pGPIOx == GPIOC)
            GPIOC_PCLK_EN;
        else if (pGPIOx == GPIOD)
            GPIOD_PCLK_EN;
        else if (pGPIOx == GPIOE)
            GPIOE_PCLK_EN;
        else if (pGPIOx == GPIOF)
            GPIOF_PCLK_EN;
        else if (pGPIOx == GPIOG)
            GPIOG_PCLK_EN;
        else if (pGPIOx == GPIOH)
            GPIOH_PCLK_EN;
        else if (pGPIOx == GPIOI)
            GPIOI_PCLK_EN;
    }
    else
    {
        if (pGPIOx == GPIOA)
            GPIOA_PCLK_DI;
        else if (pGPIOx == GPIOB)
            GPIOB_PCLK_DI;
        else if (pGPIOx == GPIOC)
            GPIOC_PCLK_DI;
        else if (pGPIOx == GPIOD)
            GPIOD_PCLK_DI;
        else if (pGPIOx == GPIOE)
            GPIOE_PCLK_DI;
        else if (pGPIOx == GPIOF)
            GPIOF_PCLK_DI;
        else if (pGPIOx == GPIOG)
            GPIOG_PCLK_DI;
        else if (pGPIOx == GPIOH)
            GPIOH_PCLK_DI;
        else if (pGPIOx == GPIOI)
            GPIOI_PCLK_DI;
    }
}

void GPIO_Init(GPIO_handle *pGPIOHandle)
{
    //configure GPIO pin mode
    uint8_t pinNumber = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber;
    if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG)
    {
        uint32_t mode = pGPIOHandle->GPIO_PinConfig.GPIO_PinMode << (2 * pinNumber); //we multiply by two because mode use two bits
        pGPIOHandle->pGPIOx->MODER &= ~(0x3 << pinNumber);
        pGPIOHandle->pGPIOx->MODER |= mode;
    }
    else
    {
        //interrupt mode
    }

    //configure pin speed
    uint32_t speed = pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed << (2 * pinNumber);
    pGPIOHandle->pGPIOx->OSPEEDR &= ~(0x3 << pinNumber);
    pGPIOHandle->pGPIOx->OSPEEDR |= speed;

    //configure pupd settings
    uint32_t pupd = pGPIOHandle->GPIO_PinConfig.GPIO_PuPdProtocol << (2 * pinNumber);
    pGPIOHandle->pGPIOx->PUPDR &= ~(0x3 << pinNumber);
    pGPIOHandle->pGPIOx->PUPDR |= pupd;

    //configure optype
    uint32_t optype = pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType << pinNumber;
    pGPIOHandle->pGPIOx->OTYPER &= ~(0x1 << pinNumber);
    pGPIOHandle->pGPIOx->OTYPER |= optype;

    //configure alt functionality
    if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALTFN)
    {
        //configure alt function registers
        uint8_t altFuncLevel = pinNumber / 8;
        uint8_t altFuncBitPosition = pinNumber % 8;
        pGPIOHandle->pGPIOx->AFR[altFuncLevel] &= ~(0xF << 4 * altFuncBitPosition);
        pGPIOHandle->pGPIOx->AFR[altFuncLevel] |= pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode << (4 * altFuncBitPosition);
    }
}

void GPIO_DeInit(GPIO_RegDef *pGPIOx)
{
    if (pGPIOx == GPIOA)
        GPIOA_REG_RESET;
    else if (pGPIOx == GPIOB)
        GPIOB_REG_RESET;
    else if (pGPIOx == GPIOC)
        GPIOC_REG_RESET;
    else if (pGPIOx == GPIOD)
        GPIOD_REG_RESET;
    else if (pGPIOx == GPIOE)
        GPIOE_REG_RESET;
    else if (pGPIOx == GPIOF)
        GPIOF_REG_RESET;
    else if (pGPIOx == GPIOG)
        GPIOG_REG_RESET;
    else if (pGPIOx == GPIOH)
        GPIOH_REG_RESET;
    else if (pGPIOx == GPIOI)
        GPIOI_REG_RESET;
}

uint8_t GPIO_ReadFromInputPin(GPIO_RegDef *pGPIOx, uint8_t pinNumber)
{
    return (uint8_t) ((pGPIOx->IDR >> pinNumber) & 0x00000001);
}

uint16_t GPIO_ReadFromInputPort(GPIO_RegDef *pGPIOx)
{
    return (uint16_t) pGPIOx->IDR;
}

void GPIO_WriteToOutputPin(GPIO_RegDef *pGPIOx, uint8_t pinNumber, uint8_t value)
{

    if(value == SET)
    {
        /* Write 1 to the output data register at the bit field corresponding to the pin number */
        pGPIOx->ODR |= (1 << pinNumber);
    }
    else
    {
        /* Write 0 to the output data register at the bit field corresponding to the pin number */
        pGPIOx->ODR &= ~(1 << pinNumber); //Clear pin
    }
}

void GPIO_WriteToOutputPort(GPIO_RegDef *pGPIOx, uint16_t value)
{
    pGPIOx->ODR = value;
}

void GPIO_ToggleOutputPin(GPIO_RegDef *pGPIOx, uint8_t pinNumber)
{
    pGPIOx->ODR ^= (1 << pinNumber);
}

void GPIO_IRQConfig(uint8_t irqNumber, uint8_t irqPriority, uint8_t enable);

void GPIO_IRQHandling(uint8_t pinNumber);
