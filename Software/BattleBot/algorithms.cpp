/*
 * algorithms.cpp
 *
 *  Created on: Aug 21, 2023
 *      Author: Jesutofunmi Kupoluyi
 */

 /* Including required header file */
 #include "algorithms.h"

 /* Function definitions */

/* Move forward */
void move_forward(struct motors *_motors, int speed)
{
  motor_enable(_motors->motor_1);
  motor_enable(_motors->motor_2);
  motor_enable(_motors->motor_3);
  motor_enable(_motors->motor_4);

  motor_move(_motors->motor_1, FW_R_FW, speed);
  motor_move(_motors->motor_2, BW_R_FW, speed);
  motor_move(_motors->motor_3, FW_L_FW, speed);
  motor_move(_motors->motor_4, BW_L_FW, speed);
}

/* Move backward */
void move_backward(struct motors *_motors, int speed)
{
  motor_enable(_motors->motor_1);
  motor_enable(_motors->motor_2);
  motor_enable(_motors->motor_3);
  motor_enable(_motors->motor_4);

  motor_move(_motors->motor_1, FW_R_BW, speed);
  motor_move(_motors->motor_2, BW_R_BW, speed);
  motor_move(_motors->motor_3, FW_L_BW, speed);
  motor_move(_motors->motor_4, BW_L_BW, speed);
}

/* Turn right */
void turn_right(struct motors *_motors, int speed)
{
  motor_enable(_motors->motor_1);
  motor_enable(_motors->motor_2);
  motor_enable(_motors->motor_3);
  motor_enable(_motors->motor_4);

  motor_move(_motors->motor_1, FW_R_BW, speed);
  motor_move(_motors->motor_2, BW_R_BW, speed);
  motor_move(_motors->motor_3, FW_L_FW, speed);
  motor_move(_motors->motor_4, BW_L_FW, speed);
}

/* Turn left */
void turn_left(struct motors *_motors, int speed)
{
  

  motor_enable(_motors->motor_1);
  motor_enable(_motors->motor_2);
  motor_enable(_motors->motor_3);
  motor_enable(_motors->motor_4);

  motor_move(_motors->motor_1, FW_R_FW, speed);
  motor_move(_motors->motor_2, BW_R_FW, speed);
  motor_move(_motors->motor_3, FW_L_BW, speed);
  motor_move(_motors->motor_4, BW_L_BW, speed);
}

/* Stop */
void stop(struct motors *_motors)
{
  motor_stop(_motors->motor_1);
  motor_stop(_motors->motor_2);
  motor_stop(_motors->motor_3);
  motor_stop(_motors->motor_4);
} 
