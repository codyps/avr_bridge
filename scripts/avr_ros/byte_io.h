#ifndef AVRROS_BYTE_IO_H_
#define AVRROS_BYTE_IO_H_

namespace ros {
	void byte_put(uint8_t);

	/* returns EOF when no data is avaliable. */
	int  byte_get(void);
}


#endif
