/*
 * motors.cpp
 *
 *  Created on: Aug 20, 2023
 *      Author: Jesutofunmi Kupoluyi
 */

/* Include header file */
#include "motors.h"


/* Implementing declared functions */
void motor_init(struct motor *mot)
{
  pinMode(mot->motor_A_dir_pin, OUTPUT);
  pinMode(mot->motor_B_dir_pin, OUTPUT);
  pinMode(mot->motor_en_pin, OUTPUT);
}

/* Enable motor */
void motor_enable(struct motor *mot) 
{
	digitalWrite(mot->motor_en_pin, HIGH);
}

/* Disable motor */
void motor_disable(struct motor *mot) 
{
	digitalWrite(mot->motor_en_pin, LOW);
}

/* Directional motor control */
void motor_move(struct motor *mot, int dir, int speed) 
{
	if (dir == 0) 
  {
		analogWrite(mot->motor_A_dir_pin, speed);

		digitalWrite(mot->motor_B_dir_pin, LOW);
	} 
  else if (dir == 1) 
  {
		analogWrite(mot->motor_B_dir_pin, speed);

		digitalWrite(mot->motor_A_dir_pin, LOW);
	}

}

/* Stop motor movement */
void motor_stop(struct motor *mot) 
{
	digitalWrite(mot->motor_B_dir_pin, LOW);

	digitalWrite(mot->motor_A_dir_pin, LOW);
}

