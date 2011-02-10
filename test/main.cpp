#include <avrRos/ros.h>
#include <stdio.h>

namespace ros {
	int ros_getchar(FILE *f) {
		return -1;
	}

	int ros_putchar(char c, FILE *f) {
		return -1;
	}
}

__attribute__((OS_main))
int main(void)
{


}
