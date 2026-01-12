#include "stm32f407xx.h"
#include "stm32f407xx_gpio_driver.h"

void delay(void)
{
    for(uint32_t i = 0; i < 250000; i++);
}

int main(void)
{
    GPIO_handle gpioLed, gpioBtn;

    gpioLed.pGPIOx = GPIOA;
    gpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_8;
    gpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
    gpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
    gpioLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
    gpioLed.GPIO_PinConfig.GPIO_PuPdProtocol = GPIO_PIN_NO_PUPD;

    GPIO_PClkControl(gpioLed.pGPIOx, ENABLE);

    GPIO_Init(&gpioLed);

    gpioBtn.pGPIOx = GPIOB;
    gpioBtn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
    gpioBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
    gpioBtn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
    gpioBtn.GPIO_PinConfig.GPIO_PuPdProtocol = GPIO_PIN_PU;

    GPIO_PClkControl(gpioBtn.pGPIOx, ENABLE);

    GPIO_Init(&gpioBtn);

    while(1)
    {
        if (!GPIO_ReadFromInputPin(gpioBtn.pGPIOx, gpioBtn.GPIO_PinConfig.GPIO_PinNumber))
        {
            GPIO_ToggleOutputPin(gpioLed.pGPIOx, gpioLed.GPIO_PinConfig.GPIO_PinNumber);
            delay();
        }
    }

    return 0;
}
