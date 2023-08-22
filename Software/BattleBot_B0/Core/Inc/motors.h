/*
 * motors.h
 *
 *  Created on: Aug 19, 2023
 *      Author: ayokupoluyi
 */

#ifndef INC_MOTORS_H_
#define INC_MOTORS_H_

/* Include necessary libraries */
#include "main.h"

/* Define the motor structure */

struct motor {

	/* Motor enable port and pin */
	GPIO_TypeDef *motor_en_gpio_port;
	uint16_t motor_en_pin;

	/* Motor direction pin - A */
	GPIO_TypeDef *motor_A_dir_gpio_port;
	uint16_t motor_A_dir_pin;

	/* Motor direction pin - B */
	GPIO_TypeDef *motor_B_dir_gpio_port;
	uint16_t motor_B_dir_pin;

};

/* Low level motor control functions */

/* Enable motor */
void motor_enable(struct motor *);

/* Disable motor */
void motor_disable(struct motor *);

/* Directional motor control */
void motor_move(struct motor *, int dir, int speed);

/* Stop motor movement */
void motor_stop(struct motor *);

#endif /* INC_MOTORS_H_ */
