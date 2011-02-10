#include <avr_ros/ros.h>
#include <stdio.h>

namespace ros {
	int fputc(char c, FILE *f) {
		return -1;
	}
}

__attribute__((OS_main))
int main(void)
{


}
