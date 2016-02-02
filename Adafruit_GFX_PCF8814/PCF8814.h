//***************************************************************************
//  File........: PCF8814.h
//  Author(s)...: Chiper
//  Porting.....: Igorok107
//  URL(s)......: http://digitalchip.ru
//  Description.: Драйвер LCD-контроллера от Nokia1100 с графическими функциями
//  Data........: 07.11.13
//  Version.....: 2.2.0
//***************************************************************************
#ifndef PCF8814_H
#define PCF8814_H

#if defined(ARDUINO) && ARDUINO >= 100
	#include "Arduino.h"
#else
	#include "WProgram.h"
	#include "pins_arduino.h"
#endif

//******************************************************************************
#define SOFT_SPI // Включение програмного SPI, если LCD не справляется со скоростью
#define LCD_MIN_DELAY	2 // *****!!!!! Минимальная задержка, при которой работает LCD-контроллер
// *****!!!!! Подбирается экспериментально под конкретный контроллер

// Макросы для работы с битами
#define ClearBit(reg, bit)       reg &= (~(1<<(bit)))
#define SetBit(reg, bit)         reg |= (1<<(bit))
#define InvBit(reg, bit)         reg ^= 1<<bit

#define CMD_LCD_MODE	0
#define DATA_LCD_MODE	1

#define BLACK	0
#define WHITE	1
#define INVERSE	2

#define ON	1
#define OFF	0

// Разрешение дисплея в пикселях
#define LCD_X_RES	96		// разрешение по горизонтали
#define LCD_Y_RES	68		// разрешение по вертикали

//******************************************************************************
class PCF8814 : public Adafruit_GFX {
	public:
		PCF8814(uint8_t sclk = 9, uint8_t sda = 10, uint8_t cs = 11, uint8_t rst = 12);
		void begin(void);
		void clearDisplay(void);
		void display();

        void off();
		void refresh();
		void on();

		void mirror(uint8_t x, uint8_t y);
		void setContrast(uint8_t c = 0x0D);

		void send(uint8_t mode, uint8_t data);
		void command(uint8_t command);
		void data(uint8_t data);

		// GFX class virtual members
		virtual void drawPixel(int16_t x, int16_t y, uint16_t color);
	private:
		uint8_t pin_sclk;
		uint8_t pin_sda;
		uint8_t pin_cs;
		uint8_t pin_rst;
};
#endif /* PCF8814_H */
