/*
 * avrRos_user.cpp
 *
 *  Created on: Jan 4, 2011
 *      Author: asher
 */
#include "WProgram.h"
#include <stdio.h>
#include "avrRos/Ros.h"
#include "avrRos/String.h"

void Ros::initCommunication(){
	Serial.begin(57600);
}

int ros_putchar(char c, FILE *stream)
{
	Serial.write(c);
  return 0;
}

int ros_getchar(FILE *stream)
{
	return Serial.read();
}
