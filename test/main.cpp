#include <stdio.h>
#include <avr/io.h>
#include "avr_ros/ros.h"
#include "avr_ros/String.h"
#include "libarduino2.h"

namespace ros {
	FILE _byte_io;
	FILE *byte_io = &_byte_io;
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
int main(void)
{
	fdev_setup_stream(&ros::_byte_io, serial_putchar,
			serial_getchar_nonblock, _FDEV_SETUP_RW);
	serial_init();

	sei();

	digital_init(13, PIN_OUTPUT);
	resp = node.advertise("response");
	node.subscribe("call",response, &call_msg);

	call_msg.data.setMaxLength(30);
	response_msg.data.setMaxLength(60);

	for(;;) {
		node.spin();
		/* Do other work */
	}
}

