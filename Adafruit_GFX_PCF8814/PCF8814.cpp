//***************************************************************************
//  File........: PCF8814.h
//  Author(s)...: Chiper
//  Porting.....: Igorok107
//  URL(s)......: http://digitalchip.ru
//  Description.: Драйвер LCD-контроллера от Nokia1100 с графическими функциями
//  Data........: 07.11.13
//  Version.....: 2.2.0
//***************************************************************************
#include <Arduino.h>
#include <Adafruit_GFX.h>
#include "PCF8814.h"

/*
 some constants that will be ored with command to effect
 ON : turn on the command
 OFF : turn off the command
 DISPLAY: turn display on/of used with LCD_MODE only, (LCD_MODE|DISPLAY|ON/OFF)
 ALL : turn on all , only used with LCD_MODE , (LCD_MODE|ALL|ON/OFF) use off for normal display
 INVERT : invert pixels, only used with LCD_MODE , (LCD_MODE|INVERT|ON/OFF) , it bring lcd into normal form use off
 *note: you can use (LCD_MODE|ALL/INVERT|OFF)  to bring lcd into normal mode
 */
#define ON 0x01
#define OFF 0x00
#define ALL 0x04
#define INVERT 0x06
#define DISPLAY 0x0E
/*
 Command list of list
 LCD_NOP                                 : no operation
 LCD_MODE				: lcd  mode, LCD_MODE|(ALL/INVERT/DISPLAY|ON/OFF)
 LCD_VOB_MSB				: use LCD_VOB_MSB|0x04 ,the value after | is a mystry,dont mess(previos notice)
 LCD_VOB_LSB				: use LCD_VOB_LSB|(contrast value,0x00 to 0x1F)
 LCD_CHARGE_PUMP_ON 		: read the datasheet , i could nt understand
 voltage muliplication          value
 X2							0x00
 X3							0x01
 X4							0x02
 X5							0x03
 LCD_RAM_ADDR_MODE		: use LCD_RAM_ADDR_MODE|(conditon ,OFF/ON),write in RAM,
 OFF : write horizontally (by default)
 ON : write vertically
 LCD_CHANGE_ROW_LSB				: accessed by LCD_ROW_LSB|(b3 b2 b1 b0), last four bits of the address
 LCD_CHANGE_ROW_MSB				: accessed by LCD_ROW_MSB|(b6 b5 b4),first 3 bits of the address; alias is 0x18
 LCD_CHANGE_COL					: move to col,LCD_COL|(b2 b1 b0)
 LCD_MIRROR_Y			: mirror on y axis , use(LCD_MIRROR_Y| condition 0x08 or OFF)
 turn on/enable mirroring, conditon->0x08 , dont use ON because its 0x01
 turn off/disable mirroring, conditon->OFF
 LCD_MIRROR_X			: turn on mirroring on x axis . this is a speical instruction &
 i couldt found|dont exists reset counter; its alias is 0xA0,didnt worked,
 and datasheet says , NOP: MX is pad selected?
 LCD_EXT_OSC				: use a external oscillator (LCD_EXT_OSC|ON / OFF)
 LCD_SOFT_RESET			: internal or software reset
 * special instruction: use 1 not ON for enabling LCD_MIRROR_X
 */
#define LCD_NOP 0xE3
#define LCD_MODE 0xA0
#define LCD_VOB_MSB 0x20
#define LCD_VOB_LSB 0x80
#define LCD_CHARGE_PUMP_ON 0x2F
#define LCD_RAM_ADDR_MODE 0xAA
#define LCD_CHANGE_ROW_LSB 0x00
#define LCD_CHANGE_ROW_MSB 0x10
#define LCD_CHANGE_COL 0xB0
#define LCD_MIRROR_Y 0xC0
#define LCD_MIRROR_X 0xA0
#define LCD_EXT_OSC 0x3A
#define LCD_SOFT_RESET 0xE2

// Видеобуфер. Работаем через буффер, так как из контроллера читать данные нельзя, а для
// графического режима нам нужно знать содержимое видеопамяти. (9 банков по 96 байт)
static uint8_t lcd_memory[LCD_X_RES][(LCD_Y_RES / 8) + 1];

// Порты к которым подключен дисплей в нумерации Arduino
volatile uint8_t pin_sclk, pin_sda, pin_cs, pin_rst;

#define enablePartialUpdate

#ifdef enablePartialUpdate
static uint8_t xUpdateMin, xUpdateMax, yUpdateMin, yUpdateMax;
#endif

static void updateBoundingBox(uint8_t xmin, uint8_t ymin, uint8_t xmax, uint8_t ymax) {
#ifdef enablePartialUpdate
	if (xmin < xUpdateMin)
		xUpdateMin = xmin;
	if (xmax > xUpdateMax)
		xUpdateMax = xmax;
	if (ymin < yUpdateMin)
		yUpdateMin = ymin;
	if (ymax > yUpdateMax)
		yUpdateMax = ymax;
#endif
}

PCF8814::PCF8814(uint8_t sclk, uint8_t sda, uint8_t cs, uint8_t rst) :
		Adafruit_GFX(LCD_X_RES, LCD_Y_RES) {
	pinMode(rst, OUTPUT);
	pinMode(sclk, OUTPUT);
	pinMode(sda, OUTPUT);
	pinMode(cs, OUTPUT);
	pin_rst = rst;
	pin_sclk = sclk;
	pin_sda = sda;
	pin_cs = cs;
	digitalWrite(pin_cs, HIGH);
}

void PCF8814::send(uint8_t type, uint8_t data) {
	digitalWrite(pin_sda, type);
	digitalWrite(pin_cs, LOW);
	digitalWrite(pin_sclk, HIGH);
	digitalWrite(pin_sclk, LOW);
	shiftOut(pin_sda, pin_sclk, MSBFIRST, data);
	digitalWrite(pin_cs, HIGH);
}

void PCF8814::command(uint8_t c) {
	send(CMD_LCD_MODE, c);
}

void PCF8814::data(uint8_t c) {
	send(DATA_LCD_MODE, c);
}

//******************************************************************************
// Инициализация контроллера
void PCF8814::begin(void) {
	digitalWrite(pin_sclk, LOW);
	digitalWrite(pin_sda, LOW);
	digitalWrite(pin_cs, LOW);
	digitalWrite(pin_rst, LOW);
	delay(10);
	digitalWrite(pin_rst, HIGH);
	digitalWrite(pin_cs, HIGH);

	command(LCD_SOFT_RESET); // *** SOFTWARE RESET
	command(LCD_EXT_OSC); // *** Use internal oscillator
	command(0xEF); // *** FRAME FREQUENCY:
	command(0x04); // *** 80Hz
	command(0xD0); // *** 1:65 divider

	refresh();
	delay(200);
	clearDisplay();
}

void PCF8814::off() {
	this->clearDisplay();
	this->command(LCD_MODE | DISPLAY | OFF);   //
	this->command(LCD_MODE | ALL | ON);        //
}

void PCF8814::refresh() {
	digitalWrite(this->pin_rst, LOW);
	digitalWrite(this->pin_rst, HIGH);
	this->command(LCD_CHARGE_PUMP_ON);   //LCD_CHARGE_PUMP_ON == 0x2F
//	this->setContrast(LCD_CONTRAST);   // LCD_CONTRAST == 0x05
	this->on();
}

void PCF8814::on() {
	this->command(LCD_MODE | DISPLAY | ON);
	this->command(LCD_MODE | ALL | OFF);
	this->command(LCD_MODE | INVERT | OFF);
}

//******************************************************************************
// Очистка экрана
void PCF8814::clearDisplay(void) {
//	memset(lcd_memory, 0, LCD_X_RES * LCD_Y_RES / 8);
	cursor_y = cursor_x = 0;
	for (uint8_t i = 0; i < LCD_X_RES; i++) {
		for (uint8_t j = 0; j < LCD_Y_RES / 8 + 1; j++) {
			lcd_memory[i][j] = 0;
		}
	}
	updateBoundingBox(0, 0, LCD_X_RES - 1, LCD_Y_RES - 1);
}

//******************************************************************************
// Зеркалирование LCD-экрана по оси x и y соответственно.
//  ON: Отразить
//  OFF: Не отражатьж
void PCF8814::mirror(uint8_t x, uint8_t y) {
	command(LCD_MIRROR_X | x);
	command(LCD_MIRROR_Y | y << 3);
}

//******************************************************************************
// Контрасиность LCD-экрана.
//  с: принимает значения от 0 до 31.
void PCF8814::setContrast(byte value) {
	command(LCD_VOB_MSB | 0x04);
	command(LCD_VOB_LSB | (value & 0x1F));
}

// the most basic function, set a single pixel
void PCF8814::drawPixel(int16_t x, int16_t y, uint16_t color) {
	if ((x < 0) || (x >= _width) || (y < 0) || (y >= _height))
		return;

	int16_t t;
	switch (rotation) {
	case 1:
		t = x;
		x = y;
		y = LCD_Y_RES - 1 - t;
		break;
	case 2:
		x = LCD_X_RES - 1 - x;
		y = LCD_Y_RES - 1 - y;
		break;
	case 3:
		t = x;
		x = LCD_X_RES - 1 - y;
		y = t;
		break;
	}

	if ((x < 0) || (x >= LCD_X_RES) || (y < 0) || (y >= LCD_Y_RES))
		return;

	uint8_t temp = lcd_memory[x][y / 8];
	switch (color) {
	case BLACK:
		SetBit(temp, y % 8);			// Включаем пиксел
		break;
	case WHITE:
		ClearBit(temp, y % 8);		// Выключаем пиксел
		break;
	case INVERSE:
		InvBit(temp, y % 8);			// Инвертируем пиксел
		break;
	}
	lcd_memory[x][y / 8] = temp; // Передаем байт в видеобуфер

	updateBoundingBox(x, y, x, y);
}

void PCF8814::display(void) {
	uint8_t col, maxcol, p;

	for (p = 0; p < LCD_Y_RES / 8 + 1; p++) {
#ifdef enablePartialUpdate
		// check if this page is part of update
		if (yUpdateMin >= ((p + 1) * 8)) {
			continue;   // nope, skip it!
		}
		if (yUpdateMax < p * 8) {
			break;
		}
#endif

		command(LCD_CHANGE_COL | p);

#ifdef enablePartialUpdate
		col = xUpdateMin;
		maxcol = xUpdateMax;
#else
		// start at the beginning of the row
		col = 0;
		maxcol = LCD_X_RES - 1;
#endif

		command(LCD_CHANGE_ROW_LSB | (col & 0x0F)); // установка адреса по X: 0000 xxxx - биты (x3 x2 x1 x0)
		command(LCD_CHANGE_ROW_MSB | ((col >> 4) & 0x07)); // установка адреса по X: 0010 0xxx - биты (x6 x5 x4)

		for (; col <= maxcol; col++) {
			data(lcd_memory[col][p]);
		}
	}

	command(LCD_CHANGE_COL); // no idea why this is necessary but it is to finish the last byte?
#ifdef enablePartialUpdate
	xUpdateMin = LCD_X_RES - 1;
	xUpdateMax = 0;
	yUpdateMin = LCD_Y_RES - 1;
	yUpdateMax = 0;
#endif

}
