#include "WProgram.h"
#include <stdio.h>
#include "avrRos/Ros.h"
#include "avrRos/ros_string.h".
#include "avrRos/RazorImu.h"
#include "SF9DOF_AHRS.h"


imu_9drazor::RazorImu imu_msg;

extern "C" void __cxa_pure_virtual()
{
  cli();
  for (;;);
}


unsigned long pubTimer=0;
void setup(){
	initRos();
	setupIMU();
	//set up the imu message


	pubTimer = millis();
}

void loop(){
	ros.spin();
	stepIMU();


	if ( (millis() - pubTimer) >=20) {
		imu_msg.angular_velocity.x = Gyro_Vector[0];
		imu_msg.angular_velocity.y = Gyro_Vector[1];
		imu_msg.angular_velocity.z = Gyro_Vector[2];

		imu_msg.linear_acceleration.x = Accel_Vector[0]/256;
		imu_msg.linear_acceleration.y = Accel_Vector[1]/256;
		imu_msg.linear_acceleration.z = Accel_Vector[2]/256;

		imu_msg.roll = roll;
		imu_msg.pitch = pitch;
		imu_msg.yaw = yaw;


		ros.publish(0,&imu_msg);
		pubTimer = millis();
	}

	delay(1);
}

