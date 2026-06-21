#ifndef __LED_H_
#define __LED_H_

#include "driver/gpio.h"

#define LED_GPIO_PIN GPIO_NUM_41

void LED_Init(void);

enum GPIO_OUTPUT_STATE
{
    PIN_RESET,
    PIN_SET
};

#define LED(X) do{X?\
                    gpio_set_level(LED_GPIO_PIN, 1):\
                    gpio_set_level(LED_GPIO_PIN, 0);\
                  }while(0)   

#define LED_TOGGLE() do{gpio_set_level\
    (LED_GPIO_PIN, !gpio_get_level(LED_GPIO_PIN));}while(0)

#endif