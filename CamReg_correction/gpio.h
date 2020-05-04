#ifndef GPIO_H_
#define GPIO_H_

#include <stm32f407xx.h>
#include <stdbool.h>

void gpio_config_output_opendrain(GPIO_TypeDef *port, unsigned int pin);
void gpio_set(GPIO_TypeDef *port, unsigned int pin);
void gpio_clear(GPIO_TypeDef *port, unsigned int pin);
void gpio_toggle(GPIO_TypeDef *port, unsigned int pin);

#endif /* GPIO_H_ */
