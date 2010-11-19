/*
 * SF9DOF_AHRS.h
 *
 *  Created on: Nov 15, 2010
 *      Author: asher
 */

#ifndef SF9DOF_AHRS_H_
#define SF9DOF_AHRS_H_

extern float Accel_Vector[3]; //Store the acceleration in a vector
extern float Gyro_Vector[3];//Store the gyros turn rate in a vector

// Euler angles
extern float roll;
extern float pitch;
extern float yaw;


void stepIMU();
void setupIMU();

#endif /* SF9DOF_AHRS_H_ */
