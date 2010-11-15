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

int toggle = 1;

void response(Msg *msg){
	//response_msg.deserialize(data);
	 if (!toggle ) {
		 digitalWrite(13, HIGH);   // set the LED on
		 toggle = 1;
	 }
	 else {
		 digitalWrite(13, LOW);    // set the LED off
		 toggle = 0;
	 }
	ros.publish(1, &response_msg);

}

void setup(){
	Serial.begin(115200);

	resp = ros.advertise("response");


	ros.subscribe("call",response, &call_msg);
	response_msg.data.setString("testing...");
	call_msg.data.setMaxLength(30);
}

int i=0;
void loop(){
	ros.spin();
	delay(10);
	i++;
	if (i>10) {
		//ros.publish(resp, &response_msg);

		i=0;

	}
}

