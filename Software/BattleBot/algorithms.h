/*
 * algorithms.h
 *
 *  Created on: Aug 21, 2023
 *      Author: Jesutofunmi Kupoluyi
 */

#ifndef __ALGORITHMS_H__
#define __ALGORITHMS_H__

/* Including required libraries */
#include "BattleBot.h"

/* Function prototypes */

/* Open Loop Movements */
/* Move forward */
void move_forward(struct motors *, int speed);

/* Move backward */
void move_backward(struct motors *, int speed);

/* Turn right */
void turn_right(struct motors *, int speed);

/* Turn left */
void turn_left(struct motors *, int speed);

/* Stop */
void stop(struct motors *);




#endif /*__ALGORITHMS_H__*/

