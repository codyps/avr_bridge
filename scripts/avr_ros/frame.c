typedef struct frame_send_ctx {
	uint16_t crc;
	void (*putchar)(uint8_t c);
} frame_send_ctx;

void frame_send_start(frame_send_ctx *fc)
{
	fc->putchar(FRAME_START);
	fc->crc = FRAME_CRC_INIT;
}

void frame_send_byte(frame_send_ctx *fc, uint8_t c)
{
	fc->crc = _crc_ccitt_update(fc->crc, c);
	_frame_send_byte(fc, c);
}

void _frame_send_byte(frame_send_ctx *fc, uint8_t c)
{
	if (FRAME_NEED_ESC(c)) {
		fc->putchar(FRAME_ESC);
		c ^= FRAME_ESC_MASK;
	}

	fc->putchar(c);
}

void frame_send_done(frame_send_ctx *fc)
{
	uint16_t crc = htole16(fc->crc);

	_frame_send_byte(fc, crc >> 8);
	_frame_send_byte(fc, crc & 0xff);

	fc->putchar(FRAME_START);
}

typedef struct frame_recv_ctx {
	uint8_t head;
	uint8_t tail;
	uint8_t data[64];

	uint16_t crc;

	bool started;
	bool esc;
	bool x; 
} frame_recv_ctx;

void frame_recv_feed(frame_recv_ctx *fc, uint8_t c)
{
	uint8_t head = fc->head;
	uint8_t phead = fc->data[head];

	if (c == FRAME_START) {
		fc->started = true;
		fc->esc = false;

		if (phead != 0) {
			/* current packet has data */
			uint8_t next_pos = (head + phead + 1) & (sizeof(fc->data) - 1);
			if (fc->crc != 0 || next_pos == fc->tail) {
				phead = 0;
				fc->data[head] = phead;
			} else {
				/* trim crc */
				phead -= FRAME_CRC_SZ;
				fc->data[head] = phead;

				fc->head = next_pos;
			}

		}

		fc->crc = FRAME_CRC_INIT;


	}


	if (fc->started) {

	} else {

	}
}

void frame_recv_error(frame_recv_ctx *fc)
{
	/* drop current packet. */
}

/* returns len */
uint8_t frame_recv_start(frame_recv_ctx *fc)
{

}

uint8_t frame_recv_byte(frame_recv_ctx *fc)
{

}

void frame_recv_done(frame_recv_ctx *fc)
{

}

void frame_recv_copy(frame_recv_ctx *fc, uint8_t *dst, uint8_t len)
{
	
}
