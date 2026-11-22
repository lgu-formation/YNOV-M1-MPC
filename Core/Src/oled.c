/*
 * oled.c
 *
 * OLED driver implementation
 *
 */

#include "oled.h"
#include "font5x7.h"
#include <stddef.h>

/* Adresse 7 bits du SSD1306 */
#define OLED_ADDR7 			(0x3CU)
#define OLED_ADDR_HAL 		(OLED_ADDR7 << 1U)
#define OLED_TIMEOUT_MS 	(20U)
#define OLED_PAGES 			(8U)
#define OLED_WIDTH 			(128U)

static I2C_HandleTypeDef *s_i2c = NULL;

static const OledFont *s_font = &OLED_FONT_5X7;

/*
* Les fontes sur 2 pages (8x16...) ne sont pas prises en charge.
* Les pointeurs doivent désigner des tableaux (valides et persistants).
*/
bool Oled_SetFont(const OledFont *font)
{
	size_t step;
	if ((font == NULL) || (font->characters == NULL) ||
			(font->columns == NULL) || (font->glyph_count == 0U) ||
			(font->width == 0U) || (font->height == 0U) ||
			(font->height > 8U))
	{
		return false;
	}
	step = (size_t)font->width + (size_t)font->spacing;
	if ((step > OLED_WIDTH) ||
			(font->glyph_count > (SIZE_MAX / (size_t)font->width)))
	{
		return false;
	}
	s_font = font;
	return true;
}


/* Renvoie un pointeur de lecture sur le glyphe, ou NULL si absent. */
static const uint8_t *font_get(const OledFont *font, char c)
{
	size_t index;
	for (index = 0U; index < font->glyph_count; ++index)
	{
		if (font->characters[index] == c)
		{
			return &font->columns[index * (size_t)font->width];
		}
	}
	return NULL;
}


static bool send_command(uint8_t command)
{
	uint8_t buffer[2] = {0x00U, command};
	if (s_i2c == NULL) { return false; }

	return HAL_I2C_Master_Transmit(s_i2c, (uint16_t)OLED_ADDR_HAL,
									buffer, 2U, OLED_TIMEOUT_MS) == HAL_OK;
}


bool Oled_WritePage(uint8_t page, const char *text)
{
	/*
	* 129 octets de données : octet de controle 0x40 + 128 colonnes.
	* Buffer sur la pile, initialisé à 0 pour nettoyer les anciennes
	* colonnes de la page
	*/
	uint8_t pixels[OLED_WIDTH + 1U] = {0U};
	size_t col = 0U;
	size_t n = 0U;
	size_t j;
	size_t step;
	const OledFont *font = s_font;

	const uint8_t *glyph;

	if ((s_i2c == NULL) || (page >= OLED_PAGES) || (text == NULL))
	{
		return false;
	}
	pixels[0] = 0x40U; /* SSD1306 control byte: display data. */
	step = (size_t)font->width + (size_t)font->spacing;
	while ((text[n] != '\0') && (col <= (OLED_WIDTH - step)))
	{
		glyph = font_get(font, text[n]);
		if (glyph != NULL)
		{
			for (j = 0U; j < (size_t)font->width; ++j)
			{
				pixels[1U + col + j] = glyph[j];
			}
		}
		col += step;
		++n;
	}

	if (!send_command((uint8_t)(0xB0U + page)))
	{
		return false;
	}
	if (!send_command(0x00U))
	{
		return false;
	}
	if (!send_command(0x10U))
	{
		return false;
	}

	return HAL_I2C_Master_Transmit( s_i2c,
									(uint16_t)OLED_ADDR_HAL,
									pixels,
									(uint16_t)sizeof(pixels),
									OLED_TIMEOUT_MS) == HAL_OK;
}


bool Oled_Init(I2C_HandleTypeDef *i2c)
{
	/* Valid for 128x64 SSD1306 modules, page-addressing, charge pump. */
	/*
	* Commandes en ROM et taille déduite : uint8_t pour le protocole.
	*/
	static const uint8_t init_cmds[] = {
		0xAEU, 0xD5U, 0x80U, 0xA8U, 0x3FU, 0xD3U, 0x00U, 0x40U,
		0x8DU, 0x14U, 0x20U, 0x02U, 0xA1U, 0xC8U, 0xDAU, 0x12U,
		0x81U, 0xCFU, 0xD9U, 0xF1U, 0xDBU, 0x40U, 0xA4U, 0xA6U,
		0x2EU, 0xAFU
	};

	/*
	* size_t indexe le tableau des commandes, borne par sizeof ;
	*/
	size_t i;
	uint8_t page;

	if (i2c == NULL)
	{
		return false;
	}
	s_i2c = i2c;

	for (i = 0U; i < sizeof(init_cmds); ++i)
	{
		if (!send_command(init_cmds[i]))
		{
			return false;
		}
	}
	for (page = 0U; page < OLED_PAGES; ++page)
	{
		if (!Oled_WritePage(page, ""))
		{
			return false;
		}
	}
	return true;
}



