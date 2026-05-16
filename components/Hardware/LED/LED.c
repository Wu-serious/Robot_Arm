#include "LED.h"

void LED_Init(void)
{
    gpio_config_t gpio_cig={0};
    gpio_cig.pin_bit_mask=1ULL<<LED_GPIO_PIN;
    gpio_cig.mode=GPIO_MODE_INPUT_OUTPUT;
    gpio_cig.pull_up_en=GPIO_PULLUP_ENABLE;
    gpio_cig.pull_down_en=GPIO_PULLDOWN_DISABLE;
    gpio_config(&gpio_cig);

    LED(0);
}

