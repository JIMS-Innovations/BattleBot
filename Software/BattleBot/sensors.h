/*
 * sensors.h
 *
 *  Created on: Aug 20, 2023
 *      Author: Jesutofunmi Kupoluyi
 */

#ifndef __SENSOR_H__
#define __SENSOR_H__

/* Including required libraries */
#include "Arduino.h"



/*------------- Sensor data structures -------------*/

/* IR sensor structure */
struct IR_sensor 
{
  uint8_t pin;

  int dist;
};

/* Ultrasonic sensor structure */
struct sonar 
{
  uint8_t trig_pin;

  uint8_t echo_pin;

  float dist;
};

/* IMU sensor structure */
struct IMU 
{
  float mag[3];

  float gyro[3];

  float acc[3];

  float temp;
};

/* Full sensor structure */
struct sensors
{
  /* IR sensor structures */
  struct IR_sensor *IR_FW_R;
  struct IR_sensor *IR_FW_L;
  struct IR_sensor *IR_BW_R;
  struct IR_sensor *IR_BW_L;

  /* Ultrasonic sensor structure*/
  struct sonar *_sonar;

  /* IMU ssensor structure */
  struct IMU *imu;
};

/*---------- function prototypes ------------*/

/* Initialise */

/* IR sensor initialization */
void IR_init(struct IR_sensor *);

/* Ultrasonic sensor initialiation */
void sonar_init(struct sonar *);

/* IMU sensor initialization */
void IMU_init(struct IMU *);

/* Full sensor initialization */
int sensors_init(struct sensors *);

/* Read in data */

/* IR sensor read */
int IR_read(struct IR_sensor* );

/* Sonar sensor read */
int sonar_read(struct sonar *);

/* IMU sensor read */
int IMU_read(struct IMU *);

/* Full sensor read */
int sensors_read(struct sensors *);


#endif /*__SENSOR_H__*/