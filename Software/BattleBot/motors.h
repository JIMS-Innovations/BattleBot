/*
 * motors.h
 *
 *  Created on: Aug 20, 2023
 *      Author: Jesutofunmi Kupoluyi
 */

#ifndef INC_MOTORS_H_
#define INC_MOTORS_H_

/* Include necessary libraries */
#include "Arduino.h"

/* Define the motor structure */
struct motor 
{

	/* Motor enable pin */
	uint8_t motor_en_pin;

	/* Motor direction pin - A */
	uint8_t motor_A_dir_pin;

	/* Motor direction pin - B */
	uint8_t motor_B_dir_pin;

};

struct motors
{
  struct motor * motor_1;
  struct motor * motor_2;
  struct motor * motor_3;
  struct motor * motor_4;

};

/* Low level motor control function prototypes */

/* Initialize motor */
void motor_init(struct motor *);

/* Enable motor */
void motor_enable(struct motor *);

/* Disable motor */
void motor_disable(struct motor *);

/* Directional motor control */
void motor_move(struct motor *, int dir, int speed);

/* Stop motor movement */
void motor_stop(struct motor *);

#endif /* INC_MOTORS_H_ */
