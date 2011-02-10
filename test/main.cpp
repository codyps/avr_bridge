#include <stdio.h>
#include <avr/io.h>
#include "avr_ros/ros.h"
#include "avr_ros/String.h"
#include "libarduino2.h"

namespace ros {
	int fputc(char c, FILE *f) {
		return serial_putchar(c, f);
	}
}

ros::Publisher resp;

std_msgs::String call_msg;
std_msgs::String response_msg;

void toggle(void)
{
	static char t = 0;
	if (!t ) {
		digital_set(13, true);   // set the LED on
		t = 1;
	} else {
		digital_set(13, false);    // set the LED off
		t = 0;
	}
}

void response(ros::Msg const *msg)
{
	toggle();

	/* note that this is unsafe as the result could be larger than
	 * the space avaliable in 'data' */
	sprintf(response_msg.data.getRawString(),
			"You sent : %s", call_msg.data.getRawString());
	node.publish(resp, &response_msg);
}


__attribute__((OS_main))
int main(void) {
	serial_init();

	digital_init(13, PIN_OUTPUT);
	resp = node.advertise("response");
	node.subscribe("call",response, &call_msg);

	call_msg.data.setMaxLength(30);
	response_msg.data.setMaxLength(60);


	for(;;) {
		for(;;) {
			int c = serial_getc();
			if (c == EOF)
				break;
			node.spin(c);
		}

		/* Do other work */
	}
}

