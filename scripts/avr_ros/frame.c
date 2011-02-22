typedef struct frame_ctx {
	uint16_t crc;
	void (*putchar)(uint8_t c);
} frame_ctx;

void frame_send_start(frame_ctx *fc)
{
	fc->putchar(FRAME_START);
	fc->crc = FRAME_CRC_INIT;
}

void frame_send_byte(frame_ctx *fc, uint8_t c)
{
	fc->crc = _crc_ccitt_update(fc->crc, c);
	_frame_send_byte(fc, c);
}

void _frame_send_byte(frame_ctx *fc, uint8_t c)
{
	if (FRAME_NEED_ESC(c)) {
		fc->putchar(FRAME_ESC);
		c ^= FRAME_ESC_MASK;
	}

	fc->putchar(c);
}

void frame_send_done(frame_ctx *fc)
{
	uint16_t crc = htole16(fc->crc);

	_frame_send_byte(fc, crc >> 8);
	_frame_send_byte(fc, crc & 0xff);

	fc->putchar(FRAME_START);
}
