/*
 * oled_font.h
 *
 * Font structure
 *
 */

#ifndef INC_OLED_FONT_H_
#define INC_OLED_FONT_H_

#include <stddef.h>
#include <stdint.h>

typedef struct {
	const char *characters;
	const uint8_t *columns;
	size_t glyph_count;
	uint8_t width;
	uint8_t height;
	uint8_t spacing;
} OledFont;

#endif /* INC_OLED_FONT_H_ */
