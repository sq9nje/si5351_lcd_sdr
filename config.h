#ifndef __CONFIG_H__
#define __CONFIG_H__

/*********************************/
/*     LCD Pin Connections       */
/*********************************/
#define LCD_RS		5
#define LCD_E		6
#define LCD_D4		7
#define LCD_D5		8
#define LCD_D6		9
#define LCD_D7		10

/*********************************/
/* Encoder & Buttons Connections */
/*********************************/
#define ENCODER_A   3                   // Encoder pin A
#define ENCODER_B   2                   // Encoder pin B
#define ENCODER_BTN 4                   // Button pin
#define BUTTON_1    11                  // Buttons for memory support
#define BUTTON_2    12

/*********************************/
/*   Filter Control Connections  */
/*********************************/
#define FIL_A		14
#define FIL_B		17
#define FIL_C		16
#define FIL_D		15

/*********************************/
/*           Memories            */
/*********************************/
#define MEM_MAXNUM	20		// Maximum memory number

/*********************************/
/*      Frequency Settings       */
/*********************************/
#define F_MULT       4                  // Output frequency multiplier. If set to 1 display frq. = output frq.
#define F_MIN        1000000L           // Lower frequency limit
#define F_MAX        30000000L          // Upper frequency limit

/*********************************/
/*       Bands & Filters         */
/*********************************/
#define BAND_NUM	9

PROGMEM const uint32_t lower_limit[] = {1810000, 3500000, 7000000, 10100000, 14000000, 18068000, 21000000, 24890000, 28000000};
PROGMEM const uint32_t upper_limit[] = {2000000, 3800000, 7200000, 10150000, 14350000, 18168000, 21450000, 24990000, 28700000};
PROGMEM const uint8_t band_filter[] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9};

PROGMEM const char label_0[] = " -- ";
PROGMEM const char label_1[] = "160m";
PROGMEM const char label_2[] = " 80m";
PROGMEM const char label_3[] = " 40m";
PROGMEM const char label_4[] = " 30m";
PROGMEM const char label_5[] = " 20m";
PROGMEM const char label_6[] = " 17m";
PROGMEM const char label_7[] = " 15m";
PROGMEM const char label_8[] = " 12m";
PROGMEM const char label_9[] = " 10m";

PROGMEM const char * const band_label[] = {label_0, label_1, label_2, label_3, label_4, label_5, label_6, label_7, label_8, label_9};


#endif /* End of config.h */
