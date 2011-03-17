#include <stdio.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include "avr_ros/ros.h"
#include "avr_ros/String.h"
#include "libarduino2.h"

namespace ros {
	void byte_put(uint8_t c) {
		return serial_putbyte(c);
	}
	int byte_get(void) {
		return serial_getbyte_nonblock();
	}
}

ros::Publisher resp;

std_msgs::String call_msg;
std_msgs::String response_msg;

static void toggle(void)
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

static void response(ros::Msg const *msg)
{
	toggle();

	/* note that this is unsafe as the result could be larger than
	 * the space avaliable in 'data' */
	ros::MsgSz l = snprintf(response_msg.data.c_str(),
			response_msg.data.size(),
			"You sent : %s", call_msg.data.c_str());
	response_msg.data.set_len(l);
	node.publish(resp, &response_msg);
}

static uint8_t call_mem[30];
static uint8_t response_mem[60];

__attribute__((OS_main))
int main(void)
{
	serial_init();
	sei();

	digital_init(13, PIN_OUTPUT);
	resp = node.advertise("response");
	node.subscribe("call",response, &call_msg);



	call_msg.data.set_mem(call_mem, sizeof(call_mem));
	response_msg.data.set_mem(response_mem, sizeof(response_mem));

	for(;;) {
		node.spin();
		/* Do other work */
	}
}

