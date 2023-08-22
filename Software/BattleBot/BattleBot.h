/*
 * BattleBot.h
 *
 *  Created on: Aug 20, 2023
 *      Author: Jesutofunmi Kupoluyi
 */

#ifndef __BATTLE_BOT_H__
#define __BATTLE_BOT_H__

/* including required libraries */
#include "Arduino.h"
#include "motors.h"
#include "sensors.h"

/* Program details */
#define AUTHOR "Jesutofunmi Kupoluyi"
#define VERSION 2.0

/* Set serial port as debug */
// #define SERIAL_DEBUG

/* Modes */
#define RAMMING_MODE 200
#define PUSH_MODE 100
// #define TEST_ROBOT
#define FIGHT_MODE

#define BATTLE_DIST 50



/*-------------- Motor driver pin definition ----------------*/

/* Forward right motor */
#define FW_RIGHT_EN_Pin   30
#define FW_RIGHT_A_Pin    6
#define FW_RIGHT_B_Pin    7
#define FW_R_FW 1
#define FW_R_BW 0

/* Backward right motor */
#define BW_RIGHT_EN_Pin   34
#define BW_RIGHT_A_Pin    8
#define BW_RIGHT_B_Pin    9
#define BW_R_FW 0
#define BW_R_BW 1

/* Forward left motor */
#define FW_LEFT_EN_Pin    22
#define FW_LEFT_A_Pin     2
#define FW_LEFT_B_Pin     3
#define FW_L_FW 0
#define FW_L_BW 1

/* Backward left motor */
#define BW_LEFT_EN_Pin    26
#define BW_LEFT_A_Pin     4
#define BW_LEFT_B_Pin     5
#define BW_L_FW 0
#define BW_L_BW 1

/*-------------- Sensor pin definitions -------------------*/

/* Ultrasonic sensor */
#define SONAR_TRIG        38
#define SONAR_ECHO        42

/* Forward right IR sensor */
#define FW_R_IR           54

/* Forward left IR sensor */
#define FW_L_IR           55

/* Backward right IR sensor */
#define BW_R_IR           56

/* Backward left IR sensor */
#define BW_L_IR           57

/*-------------- Indicators ----------------*/

/* Object detection indicator */
#define OBJ_DET           46

/* Boundary detection indicator */
#define BOUND_DET         48

/* Motion detection indicator*/
#define MOT_DET           50

/* Error detection indicator */
#define ERR_DET           52

/* Function prototype */
void fight(motors*, sensors*, int, int);

#endif /*__BATTLE_BOT_H__*/