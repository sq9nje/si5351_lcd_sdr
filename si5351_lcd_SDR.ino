/*******************************************/
/*     A simple generator with Si5351A     */
/*******************************************/
/* The frequency is displayed on 16x2 LCD  */
/* the settings are controlled with a      */
/* standard mechanical encoder,            */
/* the frequency setting step can be       */ 
/* changed with a button.                  */
/*******************************************/
/* Changelog:                              */
/* 1.0 - basic tunable generator           */
/* 1.1 - added some memories               */
/* 1.2 - automatic band detection          */
/* 1.3 - filter selection based on band    */
/*                                         */
/*******************************************/

#include <Wire.h>
#include <si5351.h>
#include <Rotary.h>
#include <LiquidCrystal.h>
#include <EEPROM.h>
#include <avr/pgmspace.h>
#include "config.h"

typedef enum {                              // TRX modes for FSM
  NORMAL_TRX, MEM_SAVE, MEM_RECALL
} trx_mode_t;

trx_mode_t trx_mode = NORMAL_TRX;

LiquidCrystal lcd(LCD_RS, LCD_E, LCD_D4, LCD_D5, LCD_D6, LCD_D7);       // LCD - pin assignement in config.h
si5351 synt;                                // Si5351
Rotary r(ENCODER_A, ENCODER_B);             // Rotary encoder
volatile int encoder_value = 0;             // variable to store amount of encoder increment/decrement

uint32_t frequency = 14285000L;             // Power on frequency setting
uint32_t mem_frequency;
volatile uint32_t radix = 100;              // Initial frequency change step

uint8_t mem_num;

uint8_t band;

/**************************************/
/* Reads frequency from EEPROM        */
/**************************************/
uint32_t read_frequency(byte addr)
{
  uint32_t value;
  uint8_t i;
  byte *p = (byte *)&value;

  for(i=0;i<4;i++)
    *p++ = EEPROM.read(addr++);

  return value;
}

/**************************************/
/* Initializes EEPROM                 */
/**************************************/
void init_memory()
{
  uint8_t i,j;

  for(i=0;i<=MEM_MAXNUM;i++)
    if(read_frequency(4*i) == 0xFFFFFFFF)
      for(j=0;j<4;j++)
        EEPROM.write(4*i+j, 0);
}

/**************************************/
/* Writes frequency from EEPROM       */
/**************************************/
void write_frequency(byte addr, uint32_t *value)
{
  uint8_t i;
  byte * p = (byte *)value;
  
  for(i=0;i<4;i++)
    EEPROM.write(addr++, *p++);
}

/**************************************/
/* Read the button with debouncing    */
/**************************************/
boolean get_button(int button)
{
  if(!digitalRead(button))
  {
    delay(20);
    if(!digitalRead(button))
    {
      while(!digitalRead(button));
      return 1;
    }
  }
  return 0;
}

/**************************************/
/* Change the frequency               */
/**************************************/
void change_frequency()
{
  frequency += encoder_value * radix;

  if(frequency > F_MAX)
    frequency = F_MAX;
  if(frequency < F_MIN)
    frequency = F_MIN;
}

/**************************************/
/* Detect current band based on       */
/* frequency                          */
/**************************************/
uint8_t detect_band(uint32_t *freq)
{
  uint8_t i;

  for(i=0;i<BAND_NUM;i++)
    if(*freq >= pgm_read_dword(&lower_limit[i]) && *freq <= pgm_read_dword(&upper_limit[i]))
      return i+1;

  return 0;
}

/**************************************/
/* Display current band information   */
/**************************************/
void display_band(uint8_t b)
{
  char label[5];
  PGM_P p;

  memcpy_P(&p, &band_label[b], sizeof(PGM_P));
  strcpy_P(label, p);
  lcd.setCursor(5,1);
  lcd.print(label);
}

/**************************************/
/*   Outputs selected filter number   */
/**************************************/ 
void set_filter(uint8_t b)
{ 
  uint8_t filter;

  filter = pgm_read_byte(&band_filter[b]);

  digitalWrite(FIL_A, filter & 0x01);
  digitalWrite(FIL_B, (filter >> 1) & 0x01);
  digitalWrite(FIL_C, (filter >> 2) & 0x01);
  digitalWrite(FIL_D, (filter >> 3) & 0x01);
}

/**************************************/
/* Interrupt service routine for      */
/* rotary encoder                     */
/**************************************/
ISR(PCINT2_vect) {
  unsigned char result = r.process();
  if (result == DIR_CW)
    encoder_value++;
  else if (result == DIR_CCW) 
    encoder_value--;
}

/**************************************/
/* Displays the frequency             */
/**************************************/
void display_frequency(uint32_t *freq)
{
  uint16_t f;
  
  lcd.setCursor(4, 0);
  f = *freq / 1000000;
  if(f<10)
    lcd.print(' ');
  lcd.print(f);
  lcd.print('.');
  f = (*freq % 1000000)/1000;
  if(f<100)
    lcd.print('0');
  if(f<10)
    lcd.print('0');
  lcd.print(f);
  lcd.print('.');
  f = *freq % 1000;
  if(f<100)
    lcd.print('0');
  if(f<10)
    lcd.print('0');
  lcd.print(f);
  lcd.print("Hz");
}

/**************************************/
/* Displays the frequency change step */
/**************************************/
void display_radix()
{
  lcd.setCursor(10, 1);
  switch(radix)
  {
    case 10:
      lcd.print("  10");
      break;
    case 100:
      lcd.print(" 100");
      break;
    case 1000:
      lcd.print("  1k");
      break;
    case 10000:
      lcd.print(" 10k");
      break;
    case 100000:
      lcd.print("100k");
      break;
    case 1000000:
      lcd.print("  1M");
      break;
  }
  lcd.print("Hz");
}

/**************************************/
/* Displays memory number and mode    */
/* Mode:                              */
/*      0, 'c' - clear mem. info      */
/*      1, 's' - mem. save mode       */
/*      2, 'r' - mem. recall mode     */
/*      3, '!' - mem. action mode     */
/**************************************/
void display_mem(uint8_t mode)
{
  switch (mode)
  {
    case 0:
    case 'c':
      lcd.setCursor(0,1);
      lcd.print("    ");
      break;
    case 1:
    case 's':     /* M00s */
      lcd.setCursor(0,1);
      lcd.print('M');
      if(mem_num < 10)
        lcd.print('0');
      lcd.print(mem_num);
      lcd.print('s');
      break;
    case 2:
    case 'r':     /* M00r */
      lcd.setCursor(0,1);
      lcd.print('M');
      if(mem_num < 10)
        lcd.print('0');
      lcd.print(mem_num);
      lcd.print('r');
      break;
    case 3:
    case '!':     /* M00* */
      lcd.setCursor(0,1);
      lcd.print('M');
      if(mem_num < 10)
        lcd.print('0');
      lcd.print(mem_num);
      lcd.print('*');
      break;
  }
}

void setup()
{
  lcd.begin(16, 2);                                                    // Initialize and clear the LCD
  lcd.clear();
  
  Wire.begin();
  synt.begin();
  synt.set_xtal(27000000, -8);						// XTAL = 27.000 MHz with -9ppm correction
  synt.pll_integer_config(PLL_A, 35);					// PLL_A = 30 * XTAL
  synt.clk_config(CLK0, (SRC_PLL << CLKx_SRC) | (IDRV_8 << CLKx_IDRV)); // CLK0 powerd up, fractional mode, MultiSynth0, PLL A, not inverted, 8mA drive
  synt.set_phase(CLK0, 0);						// no phase offset for CLK0
  synt.clk_enable(CLK0);                                                // Enable CLK0 output
  synt.simple_set_frequency(CLK0, frequency*F_MULT);                    // Set the frequency

  pinMode(ENCODER_BTN, INPUT_PULLUP);                                   // Input with internal pullup for the buttons
  pinMode(BUTTON_1, INPUT_PULLUP);
  pinMode(BUTTON_2, INPUT_PULLUP);

  pinMode(FIL_A, OUTPUT);                                               // Outputs for band/filter BCD decoder
  pinMode(FIL_B, OUTPUT);
  pinMode(FIL_C, OUTPUT);
  pinMode(FIL_D, OUTPUT);
  
  PCICR |= bit(digitalPinToPCICRbit(ENCODER_A));                       // Enable pin change interrupt for the encoder
  *digitalPinToPCMSK(ENCODER_A)  |= bit(digitalPinToPCMSKbit(ENCODER_A)) | bit(digitalPinToPCMSKbit(ENCODER_B));
  sei();
  
  init_memory();                                                        // Zeroes out unused memories

  band = detect_band(&frequency);
  display_band(band);
  set_filter(band);
  display_frequency(&frequency);                                        // Update the display
  display_radix();
}

void loop()
{
  uint8_t i, ee_addr;

  switch(trx_mode)
  {
    // Normal TRX operating mode
    case NORMAL_TRX:
      // Update frequency if encoder has been turned
      if(encoder_value)
      {
        change_frequency();
        encoder_value = 0;
        synt.simple_set_frequency(CLK0, frequency*F_MULT);
        display_frequency(&frequency);
        band = detect_band(&frequency);
        set_filter(band);
        display_band(band);
      }
      // Encoder button press changes the frequency change step
      if(get_button(ENCODER_BTN))
      {
        switch(radix)
        {
          case 10:
            radix = 100;
            break;
          case 100:
            radix = 1000;
            break;
          case 1000:
            radix = 10000;
            break;
            case 10000:
            radix = 100000;
            break;
          case 100000:
            radix = 1000000;
            break;
          case 1000000:
            radix = 10;
            break;
        }
        display_radix();
      }
      // Button 1 pressed -> enter memory recall mode
      if(get_button(BUTTON_1))
      {
        ee_addr = mem_num * 4;
        mem_frequency = read_frequency(ee_addr);
        display_frequency(&mem_frequency);
        display_band(detect_band(&mem_frequency));
        //set_filter(detect_band(&mem_frequency));
        display_mem('r');
        trx_mode = MEM_RECALL;
      }
      // Button 2 pressed -> enter memory save mode  
      if(get_button(BUTTON_2))
      {
        display_mem('s');
        trx_mode = MEM_SAVE;
      }
      break;

    // Memory recall mode
    case MEM_RECALL:
      if(encoder_value)
      {
        if(encoder_value < 0 && -1*encoder_value > mem_num)
          mem_num = 0;
        else if (encoder_value > 0 && encoder_value > MEM_MAXNUM - mem_num)
          mem_num = MEM_MAXNUM;
        else 
          mem_num += encoder_value;
        encoder_value = 0;

        ee_addr = mem_num * 4;
        mem_frequency = read_frequency(ee_addr);
        display_frequency(&mem_frequency);
        display_band(detect_band(&mem_frequency));
        //set_filter(detect_band(&mem_frequency));
        display_mem('r');
      }  

      if(get_button(BUTTON_1))  // set frequency from memory and return to TRX mode
      {
          display_mem('!');
          band = detect_band(&mem_frequency);
          set_filter(band);
          frequency = mem_frequency;
          synt.simple_set_frequency(CLK0, frequency*F_MULT);
          delay(500);
          display_mem('c');
          trx_mode = NORMAL_TRX;
      }

      if(get_button(BUTTON_2)) // retutn to TRX mode
      {
        display_mem('c');
        display_frequency(&frequency);
        display_band(detect_band(&frequency));
        trx_mode = NORMAL_TRX;
      }

      break;

    // Memory save mode  
    case MEM_SAVE: 
      if(encoder_value)
      {
        if(encoder_value < 0 && -1*encoder_value > mem_num)
          mem_num = 0;
        else if (encoder_value > 0 && encoder_value > MEM_MAXNUM - mem_num)
          mem_num = MEM_MAXNUM;
        else 
          mem_num += encoder_value;
        encoder_value = 0;

        display_mem('s');
      }  
      
      if(get_button(BUTTON_1))  // save & return to normal TRX
      {
          display_mem('!');
          ee_addr = mem_num * 4;
          write_frequency(ee_addr, &frequency);

          delay(500);
          display_mem('c');
          trx_mode = NORMAL_TRX;
      }

      if(get_button(BUTTON_2))  // cancel & return to normal TRX
      {
        display_mem('c');
        display_frequency(&frequency);
        display_band(detect_band(&frequency));
        trx_mode = NORMAL_TRX;
      }
  }
}
