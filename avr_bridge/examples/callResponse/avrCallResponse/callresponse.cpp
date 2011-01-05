#include "WProgram.h"
#include <stdio.h>
#include "avrRos/Ros.h"
#include "avrRos/String.h"
extern "C" void __cxa_pure_virtual()
{
  cli();
  for (;;);
}


Publisher resp;

std_msgs::String call_msg;
char buff[20];

std_msgs::String response_msg;


void toggle(){
	static char t=0;
	if (!t ) {
			 digitalWrite(13, HIGH);   // set the LED on
			 t = 1;
		 }
	else {
			 digitalWrite(13, LOW);    // set the LED off
			 t = 0;
		 }
}

void response(Msg *msg){
	toggle();
	ros.publish(resp, &call_msg);
}

void setup(){
	ros.initCommunication();

	pinMode(13, OUTPUT);
	resp = ros.advertise("response");

	ros.subscribe("call",response, &call_msg);

	response_msg.data.setMaxLength(20);
	response_msg.data.setString("testing...");

	call_msg.data.setMaxLength(30);
	call_msg.data.setString("adam");

}

int i=0;
void loop(){
	ros.spin();
	delay(10);
	i++;
	if (i>100) {
		//ros.publish(resp, &response_msg);
		//toggle();
		 i=0;
	}
}

