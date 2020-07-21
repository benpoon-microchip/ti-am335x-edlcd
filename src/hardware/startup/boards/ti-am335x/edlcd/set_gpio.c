/*
 * set_gpio.c
 *
 *  Created on: 2019-2-20
 *      Author: edlcd_qnx
 */
#include "startup.h"
#include <arm/am335x.h>
#include "am335x_pinmux.h"
int set_gpio_value(uintptr_t gpio_base,int gpio_num,int value){
	uint32_t r;
	/* Switch GPIOx_x to output mode */
	r = in32(gpio_base + GPIO_OE);
	r &= ~(1 << gpio_num); /* set gpio_num bit low to enable output. */
	out32(gpio_base + GPIO_OE, r);
	r = in32(gpio_base + GPIO_SETDATAOUT);
	if(value){
		r |= 1 << gpio_num;
	}
	else{
		r &= ~(1 << gpio_num);
	}
	out32(gpio_base + GPIO_SETDATAOUT, r);

	return 0;
}

