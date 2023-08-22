/*
 * motors.c
 *
 *  Created on: Aug 19, 2023
 *      Author: ayokupoluyi
 */

/* Include header file */
#include <motors.h>

/* Motor states */
enum state {
	LOW, HIGH
};

/* Implementing declared functions */

/* Enable motor */
void motor_enable(struct motor *mot) {
	HAL_GPIO_WritePin(mot->motor_en_gpio_port, mot->motor_en_pin, HIGH);

}

/* Disable motor */
void motor_disable(struct motor *mot) {
	HAL_GPIO_WritePin(mot->motor_en_gpio_port, mot->motor_en_pin, LOW);
}

/* Directional motor control */
void motor_move(struct motor *mot, int dir, int speed) {
	if (dir == 0) {
		HAL_GPIO_WritePin(mot->motor_A_dir_gpio_port, mot->motor_A_dir_pin,
				HIGH);

		HAL_GPIO_WritePin(mot->motor_B_dir_gpio_port, mot->motor_B_dir_pin,
				LOW);
	} else if (dir == 1) {
		HAL_GPIO_WritePin(mot->motor_B_dir_gpio_port, mot->motor_B_dir_pin,
				HIGH);

		HAL_GPIO_WritePin(mot->motor_A_dir_gpio_port, mot->motor_A_dir_pin,
				LOW);
	}

}

/* Stop motor movement */
void motor_stop(struct motor *mot) {
	HAL_GPIO_WritePin(mot->motor_B_dir_gpio_port, mot->motor_B_dir_pin, LOW);

	HAL_GPIO_WritePin(mot->motor_A_dir_gpio_port, mot->motor_A_dir_pin, LOW);
}

