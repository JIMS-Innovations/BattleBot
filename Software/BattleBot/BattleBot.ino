/*
  @title BattleBot.ino
  @author Jesutofunmi Kupoluyi
  @brief  Battle bot firmware
  @version 2.0
  @date 19 August, 2023
*/

/* Including required libraries */
#include "BattleBot.h"
#include "algorithms.h"
#include <Wire.h>
#include "i2c.h"
#include "i2c_MAG3110.h"
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

/* Creating necessary objects */
MAG3110 IMU_mag;
Adafruit_MPU6050 IMU_acc_gyro;

/* Forward right motor */
struct motor FW_RIGHT = {
  .motor_en_pin = FW_RIGHT_EN_Pin,
  .motor_A_dir_pin = FW_RIGHT_A_Pin,
  .motor_B_dir_pin = FW_RIGHT_B_Pin
};

/* Backward right motor */
struct motor BW_RIGHT = {
  .motor_en_pin = BW_RIGHT_EN_Pin,
  .motor_A_dir_pin = BW_RIGHT_A_Pin,
  .motor_B_dir_pin = BW_RIGHT_B_Pin
};

/* Forward left motor */
struct motor FW_LEFT = {
  .motor_en_pin = FW_LEFT_EN_Pin,
  .motor_A_dir_pin = FW_LEFT_A_Pin,
  .motor_B_dir_pin = FW_LEFT_B_Pin
};

/* Backward left motor */
struct motor BW_LEFT = {
  .motor_en_pin = BW_LEFT_EN_Pin,
  .motor_A_dir_pin = BW_LEFT_A_Pin,
  .motor_B_dir_pin = BW_LEFT_B_Pin
};


/* IR sensors */
struct IR_sensor IR_FW_R = {  
  .pin = FW_R_IR,  
  .dist = 0
  };

struct IR_sensor IR_FW_L = {  
  .pin = FW_L_IR,
  .dist = 0
  };

struct IR_sensor IR_BW_R = {  
  .pin = BW_R_IR,
  .dist = 0
  };

struct IR_sensor IR_BW_L = {  
  .pin = BW_L_IR,
  .dist = 0
  };

/* Ultrasonic sensor */
struct sonar SONAR = {  
  .trig_pin = SONAR_TRIG,
  .echo_pin = SONAR_ECHO,
  .dist = 0  
  };

/* IMU */
struct IMU _imu;


/* Full sensors */
struct sensors sense = {
  .IR_FW_R = &IR_FW_R,
  .IR_FW_L = &IR_FW_L,
  .IR_BW_R = &IR_BW_R,
  .IR_BW_L = &IR_BW_L,
  ._sonar = &SONAR,
  .imu = &_imu,
};

struct motors _motors = {
  .motor_1 = &FW_RIGHT,
  .motor_2 = &BW_RIGHT,
  .motor_3 = &FW_LEFT,
  .motor_4 = &BW_LEFT
};

/*---- Setup section ----*/
void setup() 
{
  /* Initialize sensors */
  sensors_init(&sense);

  /* Initialize motors */
  motor_init(&FW_RIGHT);
  motor_init(&BW_RIGHT);
  motor_init(&FW_LEFT);
  motor_init(&BW_LEFT);

  #ifdef SERIAL_DEBUG

  /* Serial port initialization */
  Serial.begin(9600);

  /* Print out firmware details  */

  Serial.println ("****** BATTLEBOT ******");
  Serial.print ("Author: ");
  Serial.println (AUTHOR);
  Serial.print ("Firmware version: ");
  Serial.println (VERSION);
  Serial.println ("***********************\n\n");

  delay(2000);

  #endif 

}

/*---- Loop section ----*/
void loop() 
{
  sensors_read(&sense);

  #ifdef SERIAL_DEBUG

  /* Print out sensor readings  */

  /* IR sensors */
  Serial.println ("****** Sensor Readings ******\n");
  Serial.print ("Forward right IR: ");
  Serial.println (sense.IR_FW_R->dist);
  Serial.print ("Backward right IR: ");
  Serial.println (sense.IR_BW_R->dist);
  Serial.print ("Forward left IR: ");
  Serial.println (sense.IR_FW_L->dist);
  Serial.print ("Backward left IR: ");
  Serial.println (sense.IR_BW_L->dist);
  Serial.println();

  /* Ultrasonic sensor */
  Serial.print ("Ultrasonic: ");
  Serial.println (sense._sonar->dist);
  Serial.println();


  /* IMU */
  /* Gryoscope */
  Serial.println ("*****  IMU  *****\n");
  Serial.println ("Gyro: ");
  Serial.print (sense.imu->gyro[0]);
  Serial.print (" , ");
  Serial.print (sense.imu->gyro[1]);
  Serial.print (" , ");
  Serial.println (sense.imu->gyro[2]);

  /* Accelerometer */
  Serial.println ("Acc: ");
  Serial.print (sense.imu->acc[0]);
  Serial.print (" , ");
  Serial.print (sense.imu->acc[1]);
  Serial.print (" , ");
  Serial.println (sense.imu->acc[2]);

  /* Magnetometer */
  Serial.println ("Mag: ");
  Serial.print (sense.imu->mag[0]);
  Serial.print (" , ");
  Serial.print (sense.imu->mag[1]);
  Serial.print (" , ");
  Serial.println (sense.imu->mag[2]);
  Serial.println();

  /* Loop time */
  delay(1000);

  /* Get heading in degrees */
  float heading = atan2(sense.imu->mag[0] * - 1, sense.imu->mag[1] * -1) * 180.0 / PI;
  Serial.print("Heading: ");
  Serial.println(heading);


  #endif 


  #ifdef TEST_ROBOT

  move_forward(&_motors, 50);
  delay(1000);
  stop(&_motors);
  delay(1000);
  move_backward(&_motors, 50);
  delay(1000);
  stop(&_motors);
  delay(1000);
  turn_right(&_motors, 50);
  delay(1000);
  stop(&_motors);
  delay(1000);
  turn_left(&_motors, 50);
  delay(1000);
  stop(&_motors);
  delay(1000);

  #endif

  #ifdef FIGHT_MODE

  fight(&_motors, &sense, (int)PUSH_MODE, 700);

  #endif


}


/*------------ Function definitions -------------*/

/***** Initialise *****/

void escape (struct motors * mot, struct sensors *sense, int mode, int _delay) 
{
  /* Front Left sense -> turn right */
  if (IR_read(sense->IR_FW_L) )
  {
    stop(mot);
    delay(_delay / 4);
    move_backward(&_motors, mode / 2);
    delay(_delay / 2);
    turn_right(mot, mode / 2);
    delay(_delay);
    
  }

  /* Front right sense -> turn left */
  if (IR_read(sense->IR_FW_R))
  {
    stop(mot);
    delay(_delay / 4);
    move_backward(&_motors, mode / 2);
    delay(_delay / 2);
    turn_left(mot, mode / 2);
    delay(_delay);
    
  }

  /* Back Right sense -> turn left */
  if (IR_read(sense->IR_BW_R) )
  {
    stop(mot);
    delay(_delay / 4);
    move_forward(&_motors, mode / 2);
    delay(_delay / 2);
    turn_left(mot, mode / 2);
    delay(_delay);
    
  }

  /* Back Left sense -> turn right */
  if (IR_read(sense->IR_BW_L))
  {
    stop(mot);
    delay(_delay / 4);
    move_forward(&_motors, mode / 2);
    delay(_delay / 2);
    turn_right(mot, mode / 2);
    delay(_delay);
    
  }

}
void fight(struct motors * mot, struct sensors *sense, int mode, int _delay)
{
  /*------ Boundary detection -------*/

  /* Move forward */
  move_forward(&_motors, mode / 2);
  delay(_delay);

  /* Scan for challengers */
  
  for(int i = 0; i < 20; i++)
  {
    sonar_read(sense->_sonar);
    if(sense->_sonar->dist < BATTLE_DIST)
    {
    while (sense->_sonar->dist < BATTLE_DIST && !(sense->IR_FW_L->dist || sense->IR_FW_R->dist || sense->IR_BW_L->dist || sense->IR_BW_R->dist))
    {
      move_forward(&_motors, RAMMING_MODE); /* Harcoded attack to ramming mode */
      
      sonar_read(sense->_sonar);
      IR_read(sense->IR_FW_L);
      IR_read(sense->IR_FW_R);
      IR_read(sense->IR_BW_R);
      IR_read(sense->IR_BW_L);

      if(sense->IR_FW_L->dist || sense->IR_FW_R->dist || sense->IR_BW_L->dist || sense->IR_BW_R->dist)
      {
        goto escape;
      }
    
    }
    }
    
    turn_left(&_motors, mode / 2);
    delay(_delay / 4);
  }

  for(int i = 0; i < 20; i++)
  {
    sonar_read(sense->_sonar);
    if(sense->_sonar->dist < BATTLE_DIST)
    {
    while (sense->_sonar->dist < BATTLE_DIST && !(sense->IR_FW_L->dist || sense->IR_FW_R->dist || sense->IR_BW_L->dist || sense->IR_BW_R->dist))
    {
      move_forward(&_motors, RAMMING_MODE); /* Harcoded attack to ramming mode */
      sonar_read(sense->_sonar);
      IR_read(sense->IR_FW_L);
      IR_read(sense->IR_FW_R);
      IR_read(sense->IR_BW_R);

       if(sense->IR_FW_L->dist || sense->IR_FW_R->dist || sense->IR_BW_L->dist || sense->IR_BW_R->dist)
      {
        goto escape;
      }
      }
    }

   
    turn_right(&_motors, mode / 2);
    delay(_delay / 4);
  }
  // delay(300);

escape:
     /* Front Left sense -> turn right */
  if (IR_read(sense->IR_FW_L))
  {
    stop(mot);
    delay(_delay / 4);
    move_backward(&_motors, mode / 2);
    delay(_delay / 2);
    turn_right(mot, mode / 2);
    delay(_delay);
    
  }

  /* Front right sense -> turn left */
  if (IR_read(sense->IR_FW_R))
  {
    stop(mot);
    delay(_delay / 4);
    move_backward(&_motors, mode / 2);
    delay(_delay / 2);
    turn_left(mot, mode / 2);
    delay(_delay);
    
  }

  /* Back Right sense -> turn left */
  if (IR_read(sense->IR_BW_R))
  {
    stop(mot);
    delay(_delay / 4);
    move_forward(&_motors, mode / 2);
    delay(_delay / 2);
    turn_left(mot, mode / 2);
    delay(_delay);
    
  }

  /* Back Left sense -> turn right */
  if (IR_read(sense->IR_BW_L))
  {
    stop(mot);
    delay(_delay / 4);
    move_forward(&_motors, mode / 2);
    delay(_delay / 2);
    turn_right(mot, mode / 2);
    delay(_delay);
    
  }
}

/* IR sensor initialization */
void IR_init(struct IR_sensor *IR) 
{
  pinMode(IR->pin, INPUT);

}

/* Ultrasonic sensor initialiation */
void sonar_init(struct sonar *_sonar) 
{
  pinMode(_sonar->trig_pin, OUTPUT);

  pinMode(_sonar->echo_pin, INPUT);
}

/* IMU sensor initialization */
void IMU_init(struct IMU *imu) 
{
  IMU_mag.initialize();

  IMU_acc_gyro.begin();
  IMU_acc_gyro.setAccelerometerRange(MPU6050_RANGE_8_G);
  IMU_acc_gyro.setGyroRange(MPU6050_RANGE_500_DEG);
  IMU_acc_gyro.setFilterBandwidth(MPU6050_BAND_21_HZ);
}

/* Full sensor initialization */
int sensors_init(struct sensors *_sensors) 
{
  IR_init(_sensors->IR_FW_R);
  IR_init(_sensors->IR_FW_L);
  IR_init(_sensors->IR_BW_R);
  IR_init(_sensors->IR_BW_L);

  sonar_init(_sensors->_sonar);

  IMU_init(_sensors->imu);
}

/***** Read in data *****/

/* IR sensor read */
int IR_read(struct IR_sensor *IR) 
{
  IR->dist = digitalRead(IR->pin);

  return (IR->dist > 0 ? 1 : 0);
}

/* Sonar sensor read */
int sonar_read(struct sonar *_sonar) 
{
  /* Clear trigger */
  digitalWrite(_sonar->trig_pin, LOW);
  delayMicroseconds(2);

  /* Set trigger high for 10 us */
  digitalWrite(_sonar->trig_pin, HIGH);
  delayMicroseconds(10);
  digitalWrite(_sonar->trig_pin, LOW);

  /* Read echo pin */
  _sonar->dist = (pulseIn(_sonar->echo_pin, HIGH) * 0.034 / 2);

  /* return state */
  return (_sonar->dist > 0 ? 1 : 0);
}

/* IMU sensor read */
int IMU_read(struct IMU *imu) 
{
  IMU_mag.getMeasurement(imu->mag);

  sensors_event_t acc, gyro, temp;
  IMU_acc_gyro.getEvent(&acc, &gyro, &temp);

  imu->acc[0] = acc.acceleration.x;
  imu->acc[1] = acc.acceleration.y;
  imu->acc[2] = acc.acceleration.z;

  imu->gyro[0] = gyro.gyro.x;
  imu->gyro[1] = gyro.gyro.y;
  imu->gyro[2] = gyro.gyro.z;

  imu->temp = temp.temperature;

  return (imu->temp > 0 ? 1 : 0);
}

/* Full sensor read */
int sensors_read(struct sensors *_sensors) 
{
  int status = 0;
  status += IR_read(_sensors->IR_FW_R);
  status += IR_read(_sensors->IR_FW_L);
  status += IR_read(_sensors->IR_BW_R);
  status += IR_read(_sensors->IR_BW_L);

  status += sonar_read(_sensors->_sonar);

  status += IMU_read(_sensors->imu);

  return (status >= 6 ? 1 : 0);
}
