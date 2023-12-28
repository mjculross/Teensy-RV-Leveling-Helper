//
// Teensy RV Leveling Helper Pair using NRF24L01 - version 1.1 dated 12/27/2023 @2145
//    created by Mark J Culross, KD5RXT (kd5rxt@arrl.net)
//
// HARDWARE:
//
//    Teensy 4.0
//       available from PJRC.com https://www.pjrc.com/teensy-4-0/
//
//    ILI9341 Color 320x240 TFT Touchscreen display
//       available from PJRC.com https://www.pjrc.com/store/display_ili9341_touch.html
//
//    HiLetgo NRF24L01+ Wireless Transceiver Module 2.4GHz
//       available from https://www.amazon.com/HiLetgo-NRF24L01-Wireless-Transceiver-Module/dp/B00LX47OCY
//
//    BMA400 Triple Axis Accelerometer Breakout
//       available from Sparkfun https://www.sparkfun.com/products/21208
//
//    PowerBoost 1000 Charger - Rechargeable 5V Lipo USB Boost @ 1A - 1000C
//       available from Adafruit https://www.adafruit.com/product/2465
//
//    Lithium Ion Polymer Battery - 3.7v 2500mAh
//       available from Adafruit https://www.adafruit.com/product/328
//
// Uses touchscreen input & color display to create a visual (graphical) indication of how many inches of
//    leveling blocks are required to be put under each wheel of an RV to level it
//
// Uses the ILI9341_t3 library, which has been optimized for use with the Teensy TFT display, as well as the
//    XPT2046_Touchscreen library for the touchscreen of that same display
//
// Uses the HiLetgo NRF24L01+ 2.4GHz wireless module to allow two Teensy modules to sync up & exchange
//    RV leveling info on the display
//
// Also properly detects the absence of the HiLetgo NRF24L01+ 2.4GHz wireless module & operates stand-alone
//
// Allows & stores configurable wheel base (front to back wheels distance) from 60" to 255"
// Allows & stores configurable wheel distance (front wheels, left to right distance) from 60" to 84"
//
// Internet reference (with equation derivation) for transforming accelerometer x,y,z vector data into pitch
//    & roll angles:
//
//    https://mwrona.com/posts/accel-roll-pitch/
//

//#define DEBUG_RADIO_MODE                // uncomment to troubleshoot radio mode transitions
//#define DEBUG_RADIO_MSGS                // uncomment to troubleshoot radio mode messages
//#define DEBUG_TS                        // uncomment to troubleshoot touchscreen touch locations
//#define DEBUG_ACCELEROMETER             // uncomment to troubleshoot accelerometer vector
//#define DISABLE_EEPROM_READ_SETTINGS    // uncomment to not read settings from EEPROM (to simply use program defaults for all settings)
//#define DISABLE_EEPROM_WRITE_SETTINGS   // uncomment to not write settings to EEPROM
//#define DEBUG_EEPROM_READ               // uncomment to debug reads from the EEPROM
//#define DEBUG_EEPROM_WRITE              // uncomment to debug writes to the EEPROM

//
// The following pins are used in this project:
//
// PIN  D0      = (not used)
// PIN  D1      = (not used)
// PIN  D2      = (not used)
// PIN  D3      = (not used)
// PIN  D4      = check battery pin
// PIN  D5      = TS CS (TS chip select)
// PIN  D6      = nRF24L01+ 2.4GHZ wireless module ~chip select (CSN)
// PIN  D7      = nRF24L01+ 2.4GHZ wireless module chip enable (CE)
// PIN  D8      = (not used)
// PIN  D9      = TFT/TS data/command select
// PIN D10      = (not used)
// PIN D11      = SPI MOSI (TFT/TS - data in)
// PIN D12      = SPI MISO (TFT/TS - data out)
// PIN D13      = SPI serial clock (TFT/TS) + on-board LED
// PIN D14/A0   = TFT CS (TFT chip select)
// PIN D15/A1   = (not used)
// PIN D16/A2   = (not used)
// PIN D17/A3   = (not used)
// PIN D18/A4   = I2C SDA (BMA400 data in/out)
// PIN D19/A5   = I2C SCL (BMA400 clock)
// PIN D20/A6   = (not used)
// PIN D21/A7   = (not used)
// PIN D22/A8   = (not used)
// PIN D23/A9   = (not used)

// define constant display strings
String TITLE                  = ("Teensy RV Leveling Helper Pair");
String TITLE_A                = ("Teensy");
String TITLE_B                = ("RV Leveling");
String TITLE_C                = ("Helper Pair");
String VERSION0               = ("version 1.1 - 12/27/2023-2145");
String VERSION0_A             = ("Version 1.1");
String VERSION0_B             = ("12/27/2023-2145");
String VERSION1               = ("created by: MJCulross (KD5RXT)");
String VERSION1_A             = ("created by:");
String VERSION1_B             = ("Mark J Culross");
String VERSION1_C             = ("kd5rxt@arrl.net");
String SERNUM                 = ("S/N: xxxxx");
String INSIDE_UNIT            = ("[ INSIDE UNIT ]");
String STANDALONE_UNIT        = ("[ STANDALONE UNIT ]");
String OUTSIDE_UNIT           = ("[ OUTSIDE UNIT ]");
String PAIRING                = ("[ PAIRING ]");

#include <SPI.h>
#include <RF24.h>
#include <EEPROM.h>
#include <ILI9341_t3.h>
#include <font_Arial.h> // from ILI9341_t3
#include <XPT2046_Touchscreen.h>
#include <SPI.h>

// This is calibration data for the raw touch data to the screen coordinates
// (NOTE: run the TFTcal-Teensy.ino sketch to determine the calibration values
//        for your specific touchscreen display, by touching the top-left
//        corner to find TS_MINX & TS_MINY, then rouching the bottom-right
//        corner to find TX_MAXX & TS_MAXY)
//
//const int TS_MINX = 240;
//const int TS_MINY = 260;
//const int TS_MAXX = 3860;
//const int TS_MAXY = 3800;

const int TS_MINX = 240;
const int TS_MINY = 260;
const int TS_MAXX = 3860;
const int TS_MAXY = 3800;

// pin for checking battery status, as read from the Adafruit PowerBoost 1000c:
//    true = BATT OK, false = BATT LOW
#define CHECK_BATTERY_PIN 4

// keep track of how often to check battery status
const int CHECK_BATTERY_MILLIS = 500;
unsigned long check_battery_time = millis();
bool battery_low_toggle = true;
bool initial_battery_update_required = true;
bool previous_battery_state = false;


// The display uses hardware SPI, with pin #9 as DataCommand & pin #14 as ChipSelect
// MOSI=11, MISO=12, SCK=13
const int TFT_CHIP_SELECT = 14;
const int TFT_DATA_COMMAND = 9;
ILI9341_t3 tft = ILI9341_t3(TFT_CHIP_SELECT, TFT_DATA_COMMAND);

// The touchscreen uses hardware SPI, with pin #9 as DataCommand & pin #5 as ChipSelect
// MOSI=11, MISO=12, SCK=13
#define TS_CS_PIN  5

XPT2046_Touchscreen ts(TS_CS_PIN);

// The nRF24L01+ 2.4GHz wireless module also uses hardware SPI, plus pin #7 as CE & pin #6 as CSN
#define RF24_CHIP_ENABLE 7
#define RF24_CHIP_SELECT_NOT 6

RF24 radio(RF24_CHIP_ENABLE, RF24_CHIP_SELECT_NOT);

#define IS_PRIMARY true
#define NOT_PRIMARY false

#define INITIAL_LISTEN_FOR_PRIMARY_SYNC_IN_MILLIS 4000
#define SEND_SYNC_INTERVAL_IN_MILLIS               200

#define RADIO_PACKET_SIZE         32

// Pipe addresses to allow the primary & secondary to communicate
const byte primary_tx_pipe[] = "P_TXP";
const byte secondary_tx_pipe[] = "S_TXP";

// define constant message strings
const char MSG_I_AM_PRIMARY[] = "P-SYNC";
const char MSG_I_AM_SECONDARY[]  = "S-SYNC";

typedef enum
{
   RADIO_MODE_INITIAL_SYNC = 0, RADIO_MODE_SYNC_UP, RADIO_MODE_I_AM_PRIMARY, RADIO_MODE_I_AM_SECONDARY,
} RADIO_MODE_TYPE;

RADIO_MODE_TYPE radio_mode;

boolean radio_not_present = false;

const int MAX_BLOCK_INCHES = 12;

const int block_00_inch_color = 0x0600;   // green
const int block_01_inch_color = 0x0600;   //
const int block_02_inch_color = 0xff60;   // yellow
const int block_03_inch_color = 0xff60;   //
const int block_04_inch_color = 0xff60;   //
const int block_05_inch_color = 0xfc00;   // orange
const int block_06_inch_color = 0xfc00;   //
const int block_07_inch_color = 0xfc00;   //
const int block_08_inch_color = 0xfc00;   //
const int block_09_inch_color = 0xf800;   // red
const int block_10_inch_color = 0xf800;   //
const int block_11_inch_color = 0xf800;   //
const int block_12_inch_color = 0xf800;   //

#include "SparkFun_BMA400_Arduino_Library.h"

// Create a new sensor object
BMA400 accelerometer;

// I2C address selection
uint8_t i2cAddress = BMA400_I2C_ADDRESS_DEFAULT; // 0x14
//uint8_t i2cAddress = BMA400_I2C_ADDRESS_SECONDARY; // 0x15

typedef enum
{
   TRLH_MOTORHOME_MODE = 0, TRLH_TRAILER_MODE, TRLH_BUBBLE_MODE,
} TRLH_MODE_TYPE;

TRLH_MODE_TYPE trlh_mode = TRLH_MOTORHOME_MODE;

const int FRONT_TO_REAR_MIN_WHEEL_DISTANCE_IN_INCHES =  60;
const int FRONT_TO_REAR_MAX_WHEEL_DISTANCE_IN_INCHES = 255;

const int LEFT_TO_RIGHT_MIN_WHEEL_DISTANCE_IN_INCHES =  60;
const int LEFT_TO_RIGHT_MAX_WHEEL_DISTANCE_IN_INCHES =  84;

const int DEFAULT_FRONT_TO_REAR_MAX_WHEEL_DISTANCE_IN_INCHES = 212;
const int DEFAULT_LEFT_TO_RIGHT_MAX_WHEEL_DISTANCE_IN_INCHES = 72;

int front_to_rear_wheel_distance_in_inches = DEFAULT_FRONT_TO_REAR_MAX_WHEEL_DISTANCE_IN_INCHES;
int left_to_right_wheel_distance_in_inches = DEFAULT_LEFT_TO_RIGHT_MAX_WHEEL_DISTANCE_IN_INCHES;

float pitch_avg = 0.0;
float roll_avg = 0.0;

int counter = 0;
const int NUM_SAMPLES_PER_AVG = 200;

float pitch = 0.0;
float roll  = 0.0;

int hitch_inch_blocks_needed = 0;
int lf_inch_blocks_needed = 0;
int rf_inch_blocks_needed = 0;
int lr_inch_blocks_needed = 0;
int rr_inch_blocks_needed = 0;

bool save_settings_needed = false;

unsigned long base_time;

char radio_rx_buffer[RADIO_PACKET_SIZE];

const char EEPROM_HEADER[] = "TRLH";

typedef enum
{
   EEPROM_INDEX_HEADER_00 = 0, EEPROM_INDEX_HEADER_01, EEPROM_INDEX_HEADER_02, EEPROM_INDEX_HEADER_03,

   EEPROM_INDEX_MODE,

   EEPROM_INDEX_FRONT_TO_REAR_WHEEL_DISTANCE_IN_INCHES,
   EEPROM_INDEX_LEFT_TO_RIGHT_WHEEL_DISTANCE_IN_INCHES,

   EEPROM_INDEX_CHECKSUM, EEPROM_INDEX_INV_CHECKSUM,

   EEPROM_INDEX_VALUE_COUNT,
}  EEPROM_INDEX;

#define EEPROM_INDEX_HEADER_FIRST      EEPROM_INDEX_HEADER_00
#define EEPROM_INDEX_HEADER_LAST       EEPROM_INDEX_HEADER_03


// forward definitions
void calculate_results(void);
void center_draw_text(const String text, unsigned int textSize, unsigned int xCenterLoc, unsigned int yCenterLoc, uint16_t textColor, uint16_t textBackground);
void draw_initial_screen(void);
void draw_results(void);
void draw_splash_screen(void);
void loop(void);
void process_buttons(void);
void process_button_inputs(void);
void process_radio(void);
boolean radio_receive(void);
void radio_send(const char *tx_string);
void radio_setup(boolean is_primary);
bool read_settings(void);
void save_settings(void);
void setup(void);
void show_battery_status(void);
void show_console_results();
void showIndex(int index);



// calculate the number of blocks required for each wheel
void calculate_results(void)
{
   switch (trlh_mode)
   {
      case TRLH_MOTORHOME_MODE:
         {
            // left-front is highest wheel
            if ((pitch >= 0) && (roll >= 0))
            {
               lf_inch_blocks_needed = 0;
               rf_inch_blocks_needed = int(0.1 + abs(left_to_right_wheel_distance_in_inches * sin(roll * PI / 180.0)));
               lr_inch_blocks_needed = int(0.1 + abs(front_to_rear_wheel_distance_in_inches * sin(pitch * PI / 180.0)));
               rr_inch_blocks_needed = int(0.1 + abs(left_to_right_wheel_distance_in_inches * sin(roll * PI / 180.0)) + abs(front_to_rear_wheel_distance_in_inches * sin(pitch * PI / 180.0)));
            }

            // right-front is highest wheel
            if ((pitch >= 0) && (roll < 0))
            {
               lf_inch_blocks_needed = int(0.1 + abs(left_to_right_wheel_distance_in_inches * sin(roll * PI / 180.0)));
               rf_inch_blocks_needed = 0;
               lr_inch_blocks_needed = int(0.1 + abs(front_to_rear_wheel_distance_in_inches * sin(pitch * PI / 180.0)) + abs(left_to_right_wheel_distance_in_inches * sin(roll * PI / 180.0)));
               rr_inch_blocks_needed = int(0.1 + abs(front_to_rear_wheel_distance_in_inches * sin(pitch * PI / 180.0)));
            }

            // left-rear is highest wheel
            if ((pitch < 0) && (roll >= 0))
            {
               lf_inch_blocks_needed = int(0.1 + abs(front_to_rear_wheel_distance_in_inches * sin(pitch * PI / 180.0)));
               rf_inch_blocks_needed = int(0.1 + abs(front_to_rear_wheel_distance_in_inches * sin(pitch * PI / 180.0)) + abs(left_to_right_wheel_distance_in_inches * sin(roll * PI / 180.0)));
               lr_inch_blocks_needed = 0;
               rr_inch_blocks_needed = int(0.1 + abs(left_to_right_wheel_distance_in_inches * sin(roll * PI / 180.0)));
            }

            // right-rear is highest wheel
            if ((pitch < 0) && (roll < 0))
            {
               lf_inch_blocks_needed = int(0.1 + abs(front_to_rear_wheel_distance_in_inches * sin(pitch * PI / 180.0)) + abs(left_to_right_wheel_distance_in_inches * sin(roll * PI / 180.0)));
               rf_inch_blocks_needed = int(0.1 + abs(front_to_rear_wheel_distance_in_inches * sin(pitch * PI / 180.0)));
               lr_inch_blocks_needed = int(0.1 + abs(left_to_right_wheel_distance_in_inches * sin(roll * PI / 180.0)));
               rr_inch_blocks_needed = 0;
            }
         }
         break;

      case TRLH_TRAILER_MODE:
         {
            // hitch is highest, & leaning right
            if ((pitch >= 0) && (roll >= 0))
            {
               hitch_inch_blocks_needed = 0;
               lr_inch_blocks_needed = int(0.1 + abs(front_to_rear_wheel_distance_in_inches * sin(pitch * PI / 180.0)));
               rr_inch_blocks_needed = int(0.1 + abs(left_to_right_wheel_distance_in_inches * sin(roll * PI / 180.0)) + abs(front_to_rear_wheel_distance_in_inches * sin(pitch * PI / 180.0)));
            }

            // hitch is highest, & leaning left
            if ((pitch >= 0) && (roll < 0))
            {
               hitch_inch_blocks_needed = 0;
               lr_inch_blocks_needed = int(0.1 + abs(front_to_rear_wheel_distance_in_inches * sin(pitch * PI / 180.0)) + abs(left_to_right_wheel_distance_in_inches * sin(roll * PI / 180.0)));
               rr_inch_blocks_needed = int(0.1 + abs(front_to_rear_wheel_distance_in_inches * sin(pitch * PI / 180.0)));
            }

            // left-rear is highest wheel
            if ((pitch < 0) && (roll >= 0))
            {
               hitch_inch_blocks_needed = int(0.1 + abs(front_to_rear_wheel_distance_in_inches * sin(pitch * PI / 180.0)));
               lr_inch_blocks_needed = 0;
               rr_inch_blocks_needed = int(0.1 + abs(left_to_right_wheel_distance_in_inches * sin(roll * PI / 180.0)));
            }

            // right-rear is highest wheel
            if ((pitch < 0) && (roll < 0))
            {
               hitch_inch_blocks_needed = int(0.1 + abs(front_to_rear_wheel_distance_in_inches * sin(pitch * PI / 180.0)));
               lr_inch_blocks_needed = int(0.1 + abs(left_to_right_wheel_distance_in_inches * sin(roll * PI / 180.0)));
               rr_inch_blocks_needed = 0;
            }
         }
         break;

      case TRLH_BUBBLE_MODE:
         {
         }
         break;
   }
}  // calculate_results()


// draw text, in the specified size & color, centered around xLoc & yLoc
void center_draw_text(const String text, unsigned int textSize, unsigned int xCenterLoc, unsigned int yCenterLoc, uint16_t textColor, uint16_t textBackground)
{
   unsigned int xOffset;
   unsigned int yOffset;

   if ((textSize >= 1) && (textSize <= 3))
   {
      tft.setTextSize(textSize);
   } else {
      tft.setTextSize(1);
      textSize = 1;
   }

   switch (textSize)
   {
      case 3:  // 15x21, maximum chars = 13
         {
            xOffset = (text.length() * 18) / 2 + 7;
            yOffset = 10;

            if (text.length() % 2)
            {
               xOffset -= 7;
            }
         }
         break;

      case 2:  // 10x14, maximum chars = 20
         {
            xOffset = ((text.length() * 12) / 2) + 5;
            yOffset = 7;

            if (text.length() % 2)
            {
               xOffset -= 5;
            }
         }
         break;

      case 1:  // 5x7, maximum chars = 40
      default:
         {
            xOffset = ((text.length() * 6) / 2) - 1;
            yOffset = 4;
         }
         break;
   }

   tft.setTextColor(textColor, textBackground);

   tft.setCursor(xCenterLoc - xOffset, yCenterLoc - yOffset);
   tft.print(text);
}  // centerDrawText


// draw the axle info/mod buttons
void draw_axle_info(void)
{
   tft.setTextSize(1);
   tft.fillRect(6, 290, 110, 30, ILI9341_YELLOW);
   tft.setTextColor(ILI9341_BLACK, ILI9341_YELLOW);
   tft.setCursor(12, 296);
   tft.print("WHEEL");
   tft.setCursor(12, 306);
   tft.print("BASE");
   tft.setTextSize(2);
   tft.setCursor(45, 298);
   tft.print("   ");
   tft.setCursor(45, 298);
   if (front_to_rear_wheel_distance_in_inches < 100)
   {
      tft.print(" ");
   }
   tft.print(front_to_rear_wheel_distance_in_inches);
   tft.print("\"");
   tft.setCursor(100, 293);
   tft.print("+");
   tft.setCursor(100, 305);
   tft.print("-");
   tft.drawRect(8, 292, 106, 26, ILI9341_BLACK);

   tft.setTextSize(1);
   tft.fillRect(126, 290, 110, 30, ILI9341_YELLOW);
   tft.setTextColor(ILI9341_BLACK, ILI9341_YELLOW);
   tft.setCursor(132, 296);
   tft.print("AXLE");
   tft.setCursor(132, 306);
   tft.print("WIDTH");
   tft.setTextSize(2);
   tft.setCursor(172, 298);
   tft.print("  ");
   tft.setCursor(172, 298);
   tft.print(left_to_right_wheel_distance_in_inches);
   tft.print("\"");
   tft.setCursor(220, 293);
   tft.print("+");
   tft.setCursor(220, 305);
   tft.print("-");
   tft.drawRect(128, 292, 106, 26, ILI9341_BLACK);
}  // draw_axle_info()


// draw the base screen (generally unchanging info)
void draw_initial_screen(void)
{
   tft.fillScreen(ILI9341_WHITE);

   tft.fillRect(0, 0, 240, 40, ILI9341_BLACK);

   center_draw_text(TITLE, 1, 100, 5, ILI9341_GREEN, ILI9341_BLACK);
   center_draw_text(VERSION0, 1, 100, 20, ILI9341_RED, ILI9341_BLACK);
   center_draw_text(VERSION1, 1, 100, 30, ILI9341_YELLOW, ILI9341_BLACK);

   switch (radio_mode)
   {
      case RADIO_MODE_INITIAL_SYNC:
      case RADIO_MODE_SYNC_UP:
         {
            center_draw_text(PAIRING, 1, tft.width() / 2, 50, ILI9341_BLACK, ILI9341_WHITE);
         }
         break;

      case RADIO_MODE_I_AM_PRIMARY:
         {
            if (!(radio_not_present))
            {
               center_draw_text(INSIDE_UNIT, 1, tft.width() / 2, 50, ILI9341_BLACK, ILI9341_WHITE);
            } else {
               center_draw_text(STANDALONE_UNIT, 1, tft.width() / 2, 50, ILI9341_BLACK, ILI9341_WHITE);
            }
         }
         break;

      case RADIO_MODE_I_AM_SECONDARY:
         {
            center_draw_text(OUTSIDE_UNIT, 1, tft.width() / 2, 50, ILI9341_BLACK, ILI9341_WHITE);
         }
         break;
   }

   switch (trlh_mode)
   {
      case TRLH_MOTORHOME_MODE:
         {
            tft.drawRect(75, 100, 90, 170, ILI9341_BLACK);      // box outline
            tft.drawRect(85, 80, 70, 50, ILI9341_BLACK);        // cab outline
            tft.drawRect(95, 65, 50, 5, ILI9341_BLACK);         // front bumper
            tft.drawRect(80, 272, 80, 10, ILI9341_BLACK);       // rear bumper

            tft.drawLine(85, 80, 150, 80, ILI9341_WHITE);       // erase front of cab outline
            tft.drawCircle(95, 80, 10, ILI9341_BLACK);          // lf cab
            tft.drawCircle(144, 80, 10, ILI9341_BLACK);         // rf cab
            tft.drawLine(95, 70, 140, 70, ILI9341_BLACK);       // front of cab
            tft.fillRect(95, 71, 50, 20, ILI9341_WHITE);        // erase inside of cab top
            tft.fillRect(86, 80, 20, 20, ILI9341_WHITE);        // erase inside of cab lh side
            tft.fillRect(134, 80, 20, 20, ILI9341_WHITE);       // erase inside of cab rh side

            tft.fillRect(89, 85, 10, 30, ILI9341_BLACK);        // lf tire
            tft.fillRect(141, 85, 10, 30, ILI9341_BLACK);       // rf tire

            tft.fillRect(89, 205, 10, 30, ILI9341_BLACK);       // lr inside tire
            tft.fillRect(141, 205, 10, 30, ILI9341_BLACK);      // rr inside tire
            tft.fillRect(77, 205, 10, 30, ILI9341_BLACK);       // lr outside tire
            tft.fillRect(153, 205, 10, 30, ILI9341_BLACK);      // rr outside tire

            tft.setTextColor(ILI9341_BLACK, ILI9341_WHITE);

            tft.setTextSize(2);

            tft.setCursor(5, 70);
            tft.print("RAISE");

            tft.setCursor(175, 70);
            tft.print("RAISE");

            center_draw_text("(front)", 1, tft.width() / 2, 80, ILI9341_BLACK, ILI9341_WHITE);
            center_draw_text("(rear)", 1, tft.width() / 2, 260, ILI9341_BLACK, ILI9341_WHITE);
         }
         break;

      case TRLH_TRAILER_MODE:
         {
            tft.drawRect(75, 100, 90, 170, ILI9341_BLACK);      // box outline

            tft.drawLine(118, 67, 118, 62, ILI9341_BLACK);      // lh side of toungue
            tft.drawLine(122, 67, 122, 62, ILI9341_BLACK);      // rh side of toungue
            tft.drawCircle(120, 59, 4, ILI9341_BLACK);          // hitch

            tft.drawLine(100, 100, 118, 67, ILI9341_BLACK);     // lh side of toungue triangle
            tft.drawLine(105, 100, 120, 70, ILI9341_BLACK);

            tft.drawLine(140, 100, 122, 67, ILI9341_BLACK);     // rh side of toungue triangle
            tft.drawLine(135, 100, 120, 70, ILI9341_BLACK);

            tft.drawLine(118, 100, 118, 75, ILI9341_BLACK);     // center of toungue triangle
            tft.drawLine(122, 100, 122, 75, ILI9341_BLACK);

            tft.fillRect(77, 205, 10, 30, ILI9341_BLACK);       // lr inside tire
            tft.fillRect(153, 205, 10, 30, ILI9341_BLACK);      // rr inside tire

            tft.setTextColor(ILI9341_BLACK, ILI9341_WHITE);

            tft.setTextSize(2);

            tft.setCursor(5, 70);
            tft.print("RAISE");

            tft.setTextSize(1);
         }
         break;

      case TRLH_BUBBLE_MODE:
         {
         }
         break;
   }

   tft.setTextSize(2);

   if (trlh_mode != TRLH_BUBBLE_MODE)
   {
      tft.setCursor(5, 190);
      tft.print("RAISE");

      tft.setCursor(175, 190);
      tft.print("RAISE");

      tft.setTextSize(1);

      center_draw_text("PITCH (F to B)", 1, tft.width() / 2, 140, ILI9341_BLACK, ILI9341_WHITE);
      center_draw_text("ROLL (L to R)", 1, tft.width() / 2, 175, ILI9341_BLACK, ILI9341_WHITE);

      draw_axle_info();
   } else {
      center_draw_text("PITCH (F to B)", 1, 60, 315, ILI9341_BLACK, ILI9341_WHITE);
      center_draw_text("ROLL (L to R)", 1, 180, 315, ILI9341_BLACK, ILI9341_WHITE);
   }

   tft.fillRect(6, 255, 65, 30, ILI9341_YELLOW);

   switch (trlh_mode)
   {
      case TRLH_MOTORHOME_MODE:
         {
            center_draw_text("MOTORHOME", 1, 39, 265, ILI9341_BLACK, ILI9341_YELLOW);
         }
         break;

      case TRLH_TRAILER_MODE:
         {
            center_draw_text("TRAILER", 1, 39, 265, ILI9341_BLACK, ILI9341_YELLOW);
         }
         break;

      case TRLH_BUBBLE_MODE:
         {
            center_draw_text("BUBBLE", 1, 39, 265, ILI9341_BLACK, ILI9341_YELLOW);
         }
         break;
   }

   center_draw_text("MODE", 1, 39, 275, ILI9341_BLACK, ILI9341_YELLOW);

   tft.fillRect(170, 255, 65, 30, ILI9341_YELLOW);
   tft.setCursor(188, 261);
   tft.print("TOUCH");
   tft.setCursor(176, 271);
   tft.print("TO RESUME");

   tft.drawRect(8, 257, 61, 26, ILI9341_BLACK);
   tft.drawRect(172, 257, 61, 26, ILI9341_BLACK);
}  // draw_initial_screen()


// show the calculation results on the display
void draw_results(void)
{
   static float x_roll, y_pitch;
   int tire_color;

   switch (trlh_mode)
   {
      case TRLH_MOTORHOME_MODE:
         {
            switch (lf_inch_blocks_needed)
            {
               case 0:
                  {
                     tire_color = block_00_inch_color;
                  }
                  break;

               case 1:
                  {
                     tire_color = block_01_inch_color;
                  }
                  break;

               case 2:
                  {
                     tire_color = block_02_inch_color;
                  }
                  break;

               case 3:
                  {
                     tire_color = block_03_inch_color;
                  }
                  break;

               case 4:
                  {
                     tire_color = block_04_inch_color;
                  }
                  break;

               case 5:
                  {
                     tire_color = block_05_inch_color;
                  }
                  break;

               case 6:
                  {
                     tire_color = block_06_inch_color;
                  }
                  break;

               case 7:
                  {
                     tire_color = block_07_inch_color;
                  }
                  break;

               case 8:
                  {
                     tire_color = block_08_inch_color;
                  }
                  break;

               case 9:
                  {
                     tire_color = block_09_inch_color;
                  }
                  break;

               case 10:
                  {
                     tire_color = block_10_inch_color;
                  }
                  break;

               case 11:
                  {
                     tire_color = block_11_inch_color;
                  }
                  break;

               default:
                  {
                     tire_color = block_12_inch_color;
                  }
                  break;
            }
            tft.fillRect(89, 85, 10, 30, tire_color);        // lf tire

            switch (rf_inch_blocks_needed)
            {
               case 0:
                  {
                     tire_color = block_00_inch_color;
                  }
                  break;

               case 1:
                  {
                     tire_color = block_01_inch_color;
                  }
                  break;

               case 2:
                  {
                     tire_color = block_02_inch_color;
                  }
                  break;

               case 3:
                  {
                     tire_color = block_03_inch_color;
                  }
                  break;

               case 4:
                  {
                     tire_color = block_04_inch_color;
                  }
                  break;

               case 5:
                  {
                     tire_color = block_05_inch_color;
                  }
                  break;

               case 6:
                  {
                     tire_color = block_06_inch_color;
                  }
                  break;

               case 7:
                  {
                     tire_color = block_07_inch_color;
                  }
                  break;

               case 8:
                  {
                     tire_color = block_08_inch_color;
                  }
                  break;

               case 9:
                  {
                     tire_color = block_09_inch_color;
                  }
                  break;

               case 10:
                  {
                     tire_color = block_10_inch_color;
                  }
                  break;

               case 11:
                  {
                     tire_color = block_11_inch_color;
                  }
                  break;

               default:
                  {
                     tire_color = block_12_inch_color;
                  }
                  break;
            }
            tft.fillRect(141, 85, 10, 30, tire_color);        // rf tire
         }
         break;

      case TRLH_TRAILER_MODE:
         {
            switch (hitch_inch_blocks_needed)
            {
               case 0:
                  {
                     tire_color = block_00_inch_color;
                  }
                  break;

               case 1:
                  {
                     tire_color = block_01_inch_color;
                  }
                  break;

               case 2:
                  {
                     tire_color = block_02_inch_color;
                  }
                  break;

               case 3:
                  {
                     tire_color = block_03_inch_color;
                  }
                  break;

               case 4:
                  {
                     tire_color = block_04_inch_color;
                  }
                  break;

               case 5:
                  {
                     tire_color = block_05_inch_color;
                  }
                  break;

               case 6:
                  {
                     tire_color = block_06_inch_color;
                  }
                  break;

               case 7:
                  {
                     tire_color = block_07_inch_color;
                  }
                  break;

               case 8:
                  {
                     tire_color = block_08_inch_color;
                  }
                  break;

               case 9:
                  {
                     tire_color = block_09_inch_color;
                  }
                  break;

               case 10:
                  {
                     tire_color = block_10_inch_color;
                  }
                  break;

               case 11:
                  {
                     tire_color = block_11_inch_color;
                  }
                  break;

               default:
                  {
                     tire_color = block_12_inch_color;
                  }
                  break;
            }

            tft.fillCircle(120, 59, 3, tire_color);          // hitch
         }
         break;

      case TRLH_BUBBLE_MODE:
         {
            tft.setTextColor(ILI9341_BLACK, ILI9341_WHITE);

            tft.fillRect(0, 290, 100, 15, ILI9341_WHITE);

            tft.setTextSize(2);

            if (abs(pitch) < 10)
            {
               tft.setCursor(30, 290);
            } else {
               tft.setCursor(24, 290);
            }
            if (pitch >= 0.0)
            {
               tft.print("+");
            }
            tft.print(pitch);

            tft.fillRect(120, 290, 100, 15, ILI9341_WHITE);

            if (abs(roll) < 10)
            {
               tft.setCursor(150, 290);
            } else {
               tft.setCursor(144, 290);
            }
            if (roll >= 0.0)
            {
               tft.print("+");
            }
            tft.print(roll);

            tft.fillCircle(map(x_roll, -45, +45, 200, 40), map(y_pitch, -45, +45, 240, 80), 10, ILI9341_WHITE);

            if (pitch > 45.0)
            {
               y_pitch = 45.0;
            } else {
               if (pitch < -45.0)
               {
                  y_pitch = -45.0;
               } else {
                  y_pitch = pitch;
               }
            }

            if (roll > 45.0)
            {
               x_roll = 45.0;
            } else {
               if (roll < -45.0)
               {
                  x_roll = -45.0;
               } else {
                  x_roll = roll;
               }
            }

            //   tft.fillRect(0, 40, 240, 242, ILI9341_WHITE);

            tft.fillCircle(map(x_roll, -45, +45, 200, 40), map(y_pitch, -45, +45, 240, 80), 10, ILI9341_GREEN);
            tft.drawCircle(map(x_roll, -45, +45, 200, 40), map(y_pitch, -45, +45, 240, 80), 10, ILI9341_BLACK);

            tft.drawCircle(120, 160, 14, ILI9341_BLACK);
            tft.drawCircle(120, 160, 15, ILI9341_BLACK);

            tft.drawCircle(120, 160, 34, ILI9341_BLACK);
            tft.drawCircle(120, 160, 35, ILI9341_BLACK);

            tft.drawCircle(120, 160, 54, ILI9341_BLACK);
            tft.drawCircle(120, 160, 55, ILI9341_BLACK);

            tft.drawCircle(120, 160, 74, ILI9341_BLACK);
            tft.drawCircle(120, 160, 75, ILI9341_BLACK);

            tft.drawLine(30, 160, 100, 160, ILI9341_BLACK);
            tft.drawLine(140, 160, 210, 160, ILI9341_BLACK);

            tft.drawLine(120, 70, 120, 140, ILI9341_BLACK);
            tft.drawLine(120, 180, 120, 250, ILI9341_BLACK);

            tft.drawLine(110, 160, 130, 160, ILI9341_BLACK);
            tft.drawLine(120, 150, 120, 170, ILI9341_BLACK);
         }
         break;
   }

   if (trlh_mode != TRLH_BUBBLE_MODE)
   {
      switch (lr_inch_blocks_needed)
      {
         case 0:
            {
               tire_color = block_00_inch_color;
            }
            break;

         case 1:
            {
               tire_color = block_01_inch_color;
            }
            break;

         case 2:
            {
               tire_color = block_02_inch_color;
            }
            break;

         case 3:
            {
               tire_color = block_03_inch_color;
            }
            break;

         case 4:
            {
               tire_color = block_04_inch_color;
            }
            break;

         case 5:
            {
               tire_color = block_05_inch_color;
            }
            break;

         case 6:
            {
               tire_color = block_06_inch_color;
            }
            break;

         case 7:
            {
               tire_color = block_07_inch_color;
            }
            break;

         case 8:
            {
               tire_color = block_08_inch_color;
            }
            break;

         case 9:
            {
               tire_color = block_09_inch_color;
            }
            break;

         case 10:
            {
               tire_color = block_10_inch_color;
            }
            break;

         case 11:
            {
               tire_color = block_11_inch_color;
            }
            break;

         default:
            {
               tire_color = block_12_inch_color;
            }
            break;
      }

      if (trlh_mode == TRLH_MOTORHOME_MODE)
      {
         tft.fillRect(89, 205, 10, 30, tire_color);       // lr inside tire
      }
      tft.fillRect(77, 205, 10, 30, tire_color);       // lr outside tire

      switch (rr_inch_blocks_needed)
      {
         case 0:
            {
               tire_color = block_00_inch_color;
            }
            break;

         case 1:
            {
               tire_color = block_01_inch_color;
            }
            break;

         case 2:
            {
               tire_color = block_02_inch_color;
            }
            break;

         case 3:
            {
               tire_color = block_03_inch_color;
            }
            break;

         case 4:
            {
               tire_color = block_04_inch_color;
            }
            break;

         case 5:
            {
               tire_color = block_05_inch_color;
            }
            break;

         case 6:
            {
               tire_color = block_06_inch_color;
            }
            break;

         case 7:
            {
               tire_color = block_07_inch_color;
            }
            break;

         case 8:
            {
               tire_color = block_08_inch_color;
            }
            break;

         case 9:
            {
               tire_color = block_09_inch_color;
            }
            break;

         case 10:
            {
               tire_color = block_10_inch_color;
            }
            break;

         case 11:
            {
               tire_color = block_11_inch_color;
            }
            break;

         default:
            {
               tire_color = block_12_inch_color;
            }
            break;
      }
      if (trlh_mode == TRLH_MOTORHOME_MODE)
      {
         tft.fillRect(141, 205, 10, 30, tire_color);      // rr inside tire
      }
      tft.fillRect(153, 205, 10, 30, tire_color);      // rr outside tire

      center_draw_text("*", 3, tft.width() / 2, 215, ILI9341_BLUE, ILI9341_WHITE);

      delay(100);

      center_draw_text("*", 3, tft.width() / 2, 215, ILI9341_WHITE, ILI9341_WHITE);

      tft.setTextColor(ILI9341_BLACK, ILI9341_WHITE);

      tft.fillRect(76, 143, 88, 20, ILI9341_WHITE);

      tft.setTextSize(2);

      if (abs(pitch) < 10)
      {
         tft.setCursor(90, 147);
      } else {
         tft.setCursor(84, 147);
      }
      if (pitch >= 0.0)
      {
         tft.print("+");
      }
      tft.print(pitch);

      tft.fillRect(76, 178, 88, 20, ILI9341_WHITE);

      if (abs(roll) < 10)
      {
         tft.setCursor(90, 182);
      } else {
         tft.setCursor(84, 182);
      }
      if (roll >= 0.0)
      {
         tft.print("+");
      }
      tft.print(roll);

      tft.setTextSize(3);

      switch (trlh_mode)
      {
         case TRLH_MOTORHOME_MODE:
            {
               tft.setCursor(15, 90);
               if (lf_inch_blocks_needed <= MAX_BLOCK_INCHES)
               {
                  if (lf_inch_blocks_needed < 10)
                  {
                     tft.print("0");
                  }
                  tft.print((int)lf_inch_blocks_needed);
                  tft.print("\"");
               } else {
                  tft.print("-- ");
               }

               tft.setCursor(185, 90);
               if (rf_inch_blocks_needed <= MAX_BLOCK_INCHES)
               {
                  if (rf_inch_blocks_needed < 10)
                  {
                     tft.print("0");
                  }
                  tft.print((int)rf_inch_blocks_needed);
                  tft.print("\"");
               } else {
                  tft.print("-- ");
               }
            }
            break;

         case TRLH_TRAILER_MODE:
            {
               tft.setCursor(15, 90);
               if (hitch_inch_blocks_needed <= MAX_BLOCK_INCHES)
               {
                  if (hitch_inch_blocks_needed < 10)
                  {
                     tft.print("0");
                  }
                  tft.print((int)hitch_inch_blocks_needed);
                  tft.print("\"");
               } else {
                  tft.print("-- ");
               }
            }
            break;

         case TRLH_BUBBLE_MODE:
            {
            }
            break;
      }

      tft.setCursor(15, 210);
      if (lr_inch_blocks_needed <= MAX_BLOCK_INCHES)
      {
         if (lr_inch_blocks_needed < 10)
         {
            tft.print("0");
         }
         tft.print((int)lr_inch_blocks_needed);
         tft.print("\"");
      } else {
         tft.print("-- ");
      }

      tft.setCursor(185, 210);
      if (rr_inch_blocks_needed <= MAX_BLOCK_INCHES)
      {
         if (rr_inch_blocks_needed < 10)
         {
            tft.print("0");
         }
         tft.print((int)rr_inch_blocks_needed);
         tft.print("\"");
      } else {
         tft.print("-- ");
      }
   }
}  // draw_results()


// draw the initial splash screen
void draw_splash_screen(void)
{
   tft.fillScreen(ILI9341_BLACK);

   center_draw_text(TITLE_A, 2, tft.width() / 2, 25, ILI9341_GREEN, ILI9341_BLACK);
   center_draw_text(TITLE_B, 2, tft.width() / 2, 45, ILI9341_GREEN, ILI9341_BLACK);
   center_draw_text(TITLE_C, 2, tft.width() / 2, 65, ILI9341_GREEN, ILI9341_BLACK);

   center_draw_text(VERSION0_A, 2, tft.width() / 2, 120, ILI9341_RED, ILI9341_BLACK);
   center_draw_text(VERSION0_B, 2, tft.width() / 2, 140, ILI9341_RED, ILI9341_BLACK);

   center_draw_text(VERSION1_A, 2, tft.width() / 2, 200, ILI9341_YELLOW, ILI9341_BLACK);
   center_draw_text(VERSION1_B, 2, tft.width() / 2, 225, ILI9341_YELLOW, ILI9341_BLACK);
   center_draw_text(VERSION1_C, 2, tft.width() / 2, 245, ILI9341_YELLOW, ILI9341_BLACK);

   center_draw_text(SERNUM, 2, tft.width() / 2, 290, ILI9341_GREEN, ILI9341_BLACK);
}  // draw_splash_screen()


// loop forever
void loop(void)
{
   char send_update_buffer[16];

   if (radio_mode != RADIO_MODE_I_AM_SECONDARY)
   {
      // Get measurements from the sensor. This must be called before accessing
      // the acceleration data, otherwise it will never update
      accelerometer.getSensorData();

      if (++counter <= NUM_SAMPLES_PER_AVG)
      {
         pitch = asin(accelerometer.data.accelX / sqrt(accelerometer.data.accelX * accelerometer.data.accelX + accelerometer.data.accelY * accelerometer.data.accelY + accelerometer.data.accelZ * accelerometer.data.accelZ)) * 180 / PI;
         roll  = atan(accelerometer.data.accelY / accelerometer.data.accelZ) * 180 / PI;

         pitch_avg += pitch;
         roll_avg += roll;
      } else {
         pitch = pitch_avg / (float)NUM_SAMPLES_PER_AVG;
         roll = roll_avg / (float)NUM_SAMPLES_PER_AVG;

         if (!(radio_not_present))
         {
            if (radio_mode == RADIO_MODE_I_AM_PRIMARY)
            {
               // send an UPDATE message
               send_update_buffer[0] = 'U';
               // pitch = xx.xxx
               if (pitch < 0.00)
               {
                  send_update_buffer[1] = '-';
                  pitch *= -1.0;
               } else {
                  send_update_buffer[1] = '+';
               }
               send_update_buffer[2] = ((int)pitch / 10) + '0';
               send_update_buffer[3] = ((int)pitch % 10) + '0';
               send_update_buffer[4] = '.';
               send_update_buffer[5] = (((int)(pitch * 10.0)) % 10) + '0';
               send_update_buffer[6] = (((int)(pitch * 100.0)) % 10) + '0';
               send_update_buffer[7] = (((int)(pitch * 1000.0)) % 10) + '0';

               if (send_update_buffer[1] == '-')
               {
                  pitch *= -1.0;
               }

               // roll = xx.xxx
               if (roll < 0.00)
               {
                  send_update_buffer[8] = '-';
                  roll *= -1.0;
               } else {
                  send_update_buffer[8] = '+';
               }
               send_update_buffer[9] = ((int)roll / 10) + '0';
               send_update_buffer[10] = ((int)roll % 10) + '0';
               send_update_buffer[11] = '.';
               send_update_buffer[12] = (((int)(roll * 10.0)) % 10) + '0';
               send_update_buffer[13] = (((int)(roll * 100.0)) % 10) + '0';
               send_update_buffer[14] = (((int)(roll * 1000.0)) % 10) + '0';

               if (send_update_buffer[8] == '-')
               {
                  roll *= -1.0;
               }

               send_update_buffer[15] = 0x00;

               radio_send(send_update_buffer);
            }
         }

         counter = 0;
         pitch_avg = 0.0;
         roll_avg = 0.0;

#ifdef DEBUG_ACCELEROMETER
         // Print acceleration data
         Serial.println("");
         Serial.println("");
         Serial.print("Acceleration in g's");
         Serial.print("\t");
         Serial.print("X: ");
         Serial.print(accelerometer.data.accelX, 3);
         Serial.print("\t");
         Serial.print("Y: ");
         Serial.print(accelerometer.data.accelY, 3);
         Serial.print("\t");
         Serial.print("Z: ");
         Serial.println(accelerometer.data.accelZ, 3);
#endif

         calculate_results();
         show_console_results();
         draw_results();
      }
   }

   if (save_settings_needed)
   {
      save_settings_needed = false;

      save_settings();
   }

   process_buttons();

   tft.setTextSize(1);

   if ((millis() - check_battery_time) > CHECK_BATTERY_MILLIS)
   {
      // reset the check battery delay timer
      check_battery_time = millis();

      show_battery_status();
   }

   if (!(radio_not_present))
   {
      process_radio();
   }
}  // loop()


// detect button presses
void process_buttons()
{
   boolean button_pressed = false;
   boolean wait_for_release = false;

   // See if there's any touch data for us
   if (ts.bufferEmpty())
   {
      return;
   }

   // a point object holds x y and z coordinates.
   TS_Point p = ts.getPoint();

   // Scale from ~0->4000 to tft.width using the calibration #'s
   p.x = map(p.x, TS_MINX, TS_MAXX, 0, tft.width());
   p.y = map(p.y, TS_MINY, TS_MAXY, 0, tft.height());

#ifdef DEBUG_TS
   // show where the screen was touched (for debugging purposes)
   tft.fillRect(0, 50, 240, 10, ILI9341_BLACK);
   tft.setTextColor(ILI9341_WHITE);
   tft.setTextSize(1);
   tft.setCursor(80, 0);
   tft.println("X: ");
   tft.setCursor(95, 50);
   tft.println(p.x);
   tft.setCursor(130, 50);
   tft.println("Y: ");
   tft.setCursor(145, 50);
   tft.println(p.y);
#endif

   // click on wheel base increase (+)
   if ((p.x >= 100) && (p.x <= 120) && (p.y >= 290) && (p.y <= 300))
   {
      button_pressed = true;
      wait_for_release = true;
   }

   // click on wheel base decrease (-)
   if ((p.x >= 100) && (p.x <= 120) && (p.y >= 310) && (p.y <= 320))
   {
      button_pressed = true;
      wait_for_release = true;
   }

   // click on axle width increase (+)
   if ((p.x >= 220) && (p.x <= 240) && (p.y >= 290) && (p.y <= 300))
   {
      button_pressed = true;
      wait_for_release = true;
   }

   // click on axle width decrease (-)
   if ((p.x >= 220) && (p.x <= 240) && (p.y >= 310) && (p.y <= 320))
   {
      button_pressed = true;
      wait_for_release = true;
   }

   // click on mode button
   if ((p.x >= 6) && (p.x <= 71) && (p.y >= 255) && (p.y <= 285))
   {
      button_pressed = true;
      wait_for_release = true;
   }

   if (button_pressed)
   {
      boolean debounce = true;

      while ((wait_for_release) && (debounce))
      {
         while (ts.touched())
         {
            delay(50);
         }

         // if currently not being touched, then we're done
         if (! ts.touched())
         {
            debounce = false;
         }
      }

      process_button_inputs(p);

      while (! ts.bufferEmpty())
      {
         TS_Point p_discard = ts.getPoint();

         // this is here to keep the compiler from complaining that p_discard is set but not used
         if (p_discard.x == 0)
         {
            p_discard.x = 0;
         } else {
            p_discard.x = 0;
         }
      }
   }
} // process_buttons()


// act on button presses
void process_button_inputs(TS_Point p)
{
   // click on wheel base increase (+)
   if ((trlh_mode != TRLH_BUBBLE_MODE) && (p.x >= 100) && (p.x <= 120) && (p.y >= 290) && (p.y <= 305))
   {
      if (front_to_rear_wheel_distance_in_inches < FRONT_TO_REAR_MAX_WHEEL_DISTANCE_IN_INCHES)
      {
         front_to_rear_wheel_distance_in_inches++;

         draw_axle_info();

         save_settings_needed = true;
      }
   }

   // click on wheel base decrease (-)
   if ((trlh_mode != TRLH_BUBBLE_MODE) && (p.x >= 100) && (p.x <= 120) && (p.y >= 310) && (p.y <= 320))
   {
      if (front_to_rear_wheel_distance_in_inches > FRONT_TO_REAR_MIN_WHEEL_DISTANCE_IN_INCHES)
      {
         front_to_rear_wheel_distance_in_inches--;

         draw_axle_info();

         save_settings_needed = true;
      }
   }

   // click on axle width increase (+)
   if ((trlh_mode != TRLH_BUBBLE_MODE) && (p.x >= 220) && (p.x <= 240) && (p.y >= 290) && (p.y <= 305))
   {
      if (left_to_right_wheel_distance_in_inches < LEFT_TO_RIGHT_MAX_WHEEL_DISTANCE_IN_INCHES)
      {
         left_to_right_wheel_distance_in_inches++;

         draw_axle_info();

         save_settings_needed = true;
      }
   }

   // click on axle width decrease (-)
   if ((trlh_mode != TRLH_BUBBLE_MODE) && (p.x >= 220) && (p.x <= 240) && (p.y >= 310) && (p.y <= 320))
   {
      if (left_to_right_wheel_distance_in_inches > LEFT_TO_RIGHT_MIN_WHEEL_DISTANCE_IN_INCHES)
      {
         left_to_right_wheel_distance_in_inches--;

         draw_axle_info();

         save_settings_needed = true;
      }
   }

   // click on mode button
   if ((p.x >= 6) && (p.x <= 71) && (p.y >= 255) && (p.y <= 285))
   {
      if (trlh_mode == TRLH_MOTORHOME_MODE)
      {
         trlh_mode = TRLH_TRAILER_MODE;
      } else {
         if (trlh_mode == TRLH_TRAILER_MODE)
         {
            trlh_mode = TRLH_BUBBLE_MODE;
         } else {
            trlh_mode = TRLH_MOTORHOME_MODE;
         }
      }

      initial_battery_update_required = true;

      draw_initial_screen();

      show_battery_status();

      save_settings_needed = true;
   }
}  // process_button_inputs()


// process radio operations
void process_radio(void)
{
   switch (radio_mode)
   {
      case RADIO_MODE_INITIAL_SYNC:
         {
            if ((millis() - base_time) >= INITIAL_LISTEN_FOR_PRIMARY_SYNC_IN_MILLIS)
            {
               radio_mode = RADIO_MODE_SYNC_UP;

               initial_battery_update_required = true;

               draw_initial_screen();

#ifdef DEBUG_RADIO_MODE
               Serial.println("");
               Serial.println("MODE = RADIO_MODE_SYNC_UP...");
               Serial.println("");
#endif

               base_time = millis();

               radio_setup(IS_PRIMARY);

            } else {
               if (radio_receive())
               {
                  if (strncmp(radio_rx_buffer, MSG_I_AM_PRIMARY, strlen(MSG_I_AM_PRIMARY)) == 0)
                  {
                     radio_mode = RADIO_MODE_I_AM_SECONDARY;

                     initial_battery_update_required = true;

                     draw_initial_screen();

#ifdef DEBUG_RADIO_MODE
                     Serial.println("");
                     Serial.println("MODE = RADIO_MODE_I_AM_SECONDARY...");
                     Serial.println("");
#endif

                     base_time = millis();

                     radio_send(MSG_I_AM_SECONDARY);
                  }

                  // if we received an UPDATE message
                  if ((radio_rx_buffer[0] == 'U') &&
                        // pitch = xx.xxx
                        ((radio_rx_buffer[1] == '-') || (radio_rx_buffer[1] == '+')) &&
                        ((radio_rx_buffer[2] >= '0') && (radio_rx_buffer[2] <= '9')) &&
                        ((radio_rx_buffer[3] >= '0') && (radio_rx_buffer[3] <= '9')) &&
                        (radio_rx_buffer[4] == '.') &&
                        ((radio_rx_buffer[5] >= '0') && (radio_rx_buffer[5] <= '9')) &&
                        ((radio_rx_buffer[6] >= '0') && (radio_rx_buffer[6] <= '9')) &&
                        ((radio_rx_buffer[7] >= '0') && (radio_rx_buffer[7] <= '9')) &&

                        // roll = xx.xxx
                        ((radio_rx_buffer[8] == '-') || (radio_rx_buffer[8] == '+')) &&
                        ((radio_rx_buffer[9] >= '0') && (radio_rx_buffer[9] <= '9')) &&
                        ((radio_rx_buffer[10] >= '0') && (radio_rx_buffer[10] <= '9')) &&
                        (radio_rx_buffer[11] == '.') &&
                        ((radio_rx_buffer[12] >= '0') && (radio_rx_buffer[12] <= '9')) &&
                        ((radio_rx_buffer[13] >= '0') && (radio_rx_buffer[13] <= '9')) &&
                        ((radio_rx_buffer[14] >= '0') && (radio_rx_buffer[14] <= '9')))
                  {
                     radio_mode = RADIO_MODE_I_AM_SECONDARY;

                     initial_battery_update_required = true;

                     draw_initial_screen();

#ifdef DEBUG_RADIO_MODE
                     Serial.println("");
                     Serial.println("MODE = RADIO_MODE_I_AM_SECONDARY...");
                     Serial.println("");
#endif

                     pitch = (float)(radio_rx_buffer[2] - '0') * 10.0;
                     pitch += (float)(radio_rx_buffer[3] - '0');
                     pitch += (float)(radio_rx_buffer[5] - '0') / 10.0;
                     pitch += (float)(radio_rx_buffer[6] - '0') / 100.0;
                     pitch += (float)(radio_rx_buffer[7] - '0') / 1000.0;

                     if (radio_rx_buffer[1] == '-')
                     {
                        pitch *= -1.0;
                     }

                     roll = (float)(radio_rx_buffer[9] - '0') * 10.0;
                     roll += (float)(radio_rx_buffer[10] - '0');
                     roll += (float)(radio_rx_buffer[12] - '0') / 10.0;
                     roll += (float)(radio_rx_buffer[13] - '0') / 100.0;
                     roll += (float)(radio_rx_buffer[14] - '0') / 1000.0;

                     if (radio_rx_buffer[8] == '-')
                     {
                        roll *= -1.0;
                     }

                     calculate_results();
                     show_console_results();
                     draw_results();

                     radio_send(MSG_I_AM_SECONDARY);
                  }
               }
            }
         }
         break;

      case RADIO_MODE_SYNC_UP:
         {
            if ((millis() - base_time) > SEND_SYNC_INTERVAL_IN_MILLIS)
            {
               // send a PRIMARY message
               radio_send(MSG_I_AM_PRIMARY);

               base_time = millis();
            }

            if (radio_receive())
            {
               if (strncmp(radio_rx_buffer, MSG_I_AM_PRIMARY, strlen(MSG_I_AM_PRIMARY)) == 0)
               {
                  radio_mode = RADIO_MODE_I_AM_SECONDARY;

                  initial_battery_update_required = true;

                  draw_initial_screen();

#ifdef DEBUG_RADIO_MODE
                  Serial.println("");
                  Serial.println("MODE = RADIO_MODE_I_AM_SECONDARY...");
                  Serial.println("");
#endif

                  radio_setup(NOT_PRIMARY);

                  radio_send(MSG_I_AM_SECONDARY);
               }

               if (strncmp(radio_rx_buffer, MSG_I_AM_SECONDARY, strlen(MSG_I_AM_SECONDARY)) == 0)
               {
                  radio_mode = RADIO_MODE_I_AM_PRIMARY;

                  initial_battery_update_required = true;

                  draw_initial_screen();

#ifdef DEBUG_RADIO_MODE
                  Serial.println("");
                  Serial.println("MODE = RADIO_MODE_I_AM_PRIMARY...");
                  Serial.println("");
#endif

                  radio_setup(IS_PRIMARY);

                  radio_send(MSG_I_AM_PRIMARY);
               }
            }
         }
         break;

      case RADIO_MODE_I_AM_PRIMARY:
         {
            if (radio_receive())
            {
               if (strncmp(radio_rx_buffer, MSG_I_AM_PRIMARY, strlen(MSG_I_AM_PRIMARY)) == 0)
               {
                  radio_mode = RADIO_MODE_I_AM_SECONDARY;

                  initial_battery_update_required = true;

                  draw_initial_screen();

#ifdef DEBUG_RADIO_MODE
                  Serial.println("");
                  Serial.println("MODE = RADIO_MODE_I_AM_SECONDARY...");
                  Serial.println("");
#endif

                  radio_setup(NOT_PRIMARY);

                  radio_send(MSG_I_AM_SECONDARY);
               }
            }
         }
         break;

      case RADIO_MODE_I_AM_SECONDARY:
         {
            if (radio_receive())
            {
               if (strncmp(radio_rx_buffer, MSG_I_AM_PRIMARY, strlen(MSG_I_AM_PRIMARY)) == 0)
               {
                  radio_mode = RADIO_MODE_I_AM_SECONDARY;

#ifdef DEBUG_RADIO_MODE
                  Serial.println("");
                  Serial.println("MODE = RADIO_MODE_I_AM_SECONDARY...");
                  Serial.println("");
#endif

                  radio_setup(NOT_PRIMARY);

                  radio_send(MSG_I_AM_SECONDARY);
               }

               // if we received an UPDATE message
               if ((radio_rx_buffer[0] == 'U') &&
                     // pitch = xx.xxx
                     ((radio_rx_buffer[1] == '-') || (radio_rx_buffer[1] == '+')) &&
                     ((radio_rx_buffer[2] >= '0') && (radio_rx_buffer[2] <= '9')) &&
                     ((radio_rx_buffer[3] >= '0') && (radio_rx_buffer[3] <= '9')) &&
                     (radio_rx_buffer[4] == '.') &&
                     ((radio_rx_buffer[5] >= '0') && (radio_rx_buffer[5] <= '9')) &&
                     ((radio_rx_buffer[6] >= '0') && (radio_rx_buffer[6] <= '9')) &&
                     ((radio_rx_buffer[7] >= '0') && (radio_rx_buffer[7] <= '9')) &&

                     // roll = xx.xxx
                     ((radio_rx_buffer[8] == '-') || (radio_rx_buffer[8] == '+')) &&
                     ((radio_rx_buffer[9] >= '0') && (radio_rx_buffer[9] <= '9')) &&
                     ((radio_rx_buffer[10] >= '0') && (radio_rx_buffer[10] <= '9')) &&
                     (radio_rx_buffer[11] == '.') &&
                     ((radio_rx_buffer[12] >= '0') && (radio_rx_buffer[12] <= '9')) &&
                     ((radio_rx_buffer[13] >= '0') && (radio_rx_buffer[13] <= '9')) &&
                     ((radio_rx_buffer[14] >= '0') && (radio_rx_buffer[14] <= '9')))
               {
                  pitch = (float)(radio_rx_buffer[2] - '0') * 10.0;
                  pitch += (float)(radio_rx_buffer[3] - '0');
                  pitch += (float)(radio_rx_buffer[5] - '0') / 10.0;
                  pitch += (float)(radio_rx_buffer[6] - '0') / 100.0;
                  pitch += (float)(radio_rx_buffer[7] - '0') / 1000.0;

                  if (radio_rx_buffer[1] == '-')
                  {
                     pitch *= -1.0;
                  }

                  roll = (float)(radio_rx_buffer[9] - '0') * 10.0;
                  roll += (float)(radio_rx_buffer[10] - '0');
                  roll += (float)(radio_rx_buffer[12] - '0') / 10.0;
                  roll += (float)(radio_rx_buffer[13] - '0') / 100.0;
                  roll += (float)(radio_rx_buffer[14] - '0') / 1000.0;

                  if (radio_rx_buffer[8] == '-')
                  {
                     roll *= -1.0;
                  }

                  calculate_results();
                  show_console_results();
                  draw_results();
               }
            }
            break;
         }
   }
}  // process_radio()


// receive from the radio
boolean radio_receive(void)
{
   boolean read_something = false;

   // flush the buffer before reading
   for (int i = 0; i < RADIO_PACKET_SIZE; i++)
   {
      radio_rx_buffer[i] = 0x00;
   }

   while (radio.available())
   {
      // flush the buffer before reading
      for (int i = 0; i < RADIO_PACKET_SIZE; i++)
      {
         radio_rx_buffer[i] = 0x00;
      }

      radio.read(&radio_rx_buffer, RADIO_PACKET_SIZE);

      read_something = true;
   }

   if (read_something)
   {
#ifdef DEBUG_RADIO_MSGS
      Serial.print("RX: ");
      Serial.println(radio_rx_buffer);
#endif
   }

   return (read_something);
}  // radio_receive()


// send a buffer thru the radio
void radio_send(const char *tx_string)
{
   radio.stopListening();

   radio.write(tx_string, RADIO_PACKET_SIZE);

#ifdef DEBUG_RADIO_MSGS
   Serial.print("TX: ");
   Serial.println(tx_string);
#endif

   radio.startListening();
}  // radio_send()


// setup radio
void radio_setup(boolean is_primary)
{
   radio.powerDown();
   if (radio.begin())
   {
      radio_not_present = false;

      radio.setChannel(112);
      radio.setPALevel(RF24_PA_HIGH);
      radio.setDataRate(RF24_250KBPS);
      radio.setAutoAck(1);
      radio.setRetries(2, 15);
      radio.setCRCLength(RF24_CRC_16);

      if (is_primary)
      {
         radio.openWritingPipe(primary_tx_pipe);
         radio.openReadingPipe(1, secondary_tx_pipe);
      } else {
         radio.openWritingPipe(secondary_tx_pipe);
         radio.openReadingPipe(1, primary_tx_pipe);
      }

      radio.flush_rx();
      radio.flush_tx();

      radio.startListening();
   } else {
      radio_not_present = true;

      radio_mode = RADIO_MODE_I_AM_PRIMARY;
   }
}  // radio_setup()


// read the current settings from EEPROM
bool read_settings(void)
{
   byte eeprom_value, xor_value, inv_xor_value;
   byte xor_result = 0x4d;  // start with a non-zero value
   bool header_is_good = true;
   bool eeprom_settings_good = false;

   Serial.println("");
   Serial.print("Attempting to read/verify saved settingsfrom EEPROM...");

   for (byte eeprom_index = EEPROM_INDEX_HEADER_FIRST; eeprom_index < EEPROM_INDEX_CHECKSUM; eeprom_index++)
   {
      EEPROM.get((int)(eeprom_index), eeprom_value);

      xor_result = xor_result ^ eeprom_value;

#ifdef DEBUG_EEPROM_READ
      if (eeprom_index == EEPROM_INDEX_HEADER_FIRST)
      {
         Serial.println("");
      }
      Serial.print("(READ ");
      showIndex((int)(eeprom_index));
      Serial.print(": ");
      showByteValue(eeprom_value);
      Serial.println(")");
#endif

      if (eeprom_index <= EEPROM_INDEX_HEADER_LAST)
      {
         if (eeprom_value != EEPROM_HEADER[eeprom_index])
         {
            header_is_good = false;
         }
      }
   }

   // read the checksum & inverse checksum
   EEPROM.get(EEPROM_INDEX_CHECKSUM, xor_value);
   EEPROM.get(EEPROM_INDEX_INV_CHECKSUM, inv_xor_value);

   // if the checksums match & the header values match, then we seem to have valid settings in EEPROM, so read all of the settings
   if ((xor_value == xor_result) &&
         (inv_xor_value == (byte)~xor_result) &&
         (header_is_good))
   {
      Serial.print("verified settings (");
      Serial.print(EEPROM_INDEX_VALUE_COUNT);
      Serial.println(" values) in EEPROM are valid...");
      Serial.println("");

#ifndef DISABLE_EEPROM_READ_SETTINGS
      for (byte eeprom_index = EEPROM_INDEX_HEADER_FIRST; eeprom_index <= EEPROM_INDEX_INV_CHECKSUM; eeprom_index++)
      {
         EEPROM.get((int)(eeprom_index), eeprom_value);

#ifdef DEBUG_EEPROM_READ
         Serial.print("(READ ");
         showIndex((int)(eeprom_index));
         Serial.print(": ");
         showByteValue(eeprom_value);
#endif

         switch (eeprom_index)
         {
            case EEPROM_INDEX_HEADER_00:
            case EEPROM_INDEX_HEADER_01:
            case EEPROM_INDEX_HEADER_02:
            case EEPROM_INDEX_HEADER_03:
               {
#ifdef DEBUG_EEPROM_READ
                  Serial.print(") Header[");
                  Serial.print(eeprom_index / 10);
                  Serial.print(eeprom_index % 10);
                  Serial.print("]                                     = ");
                  Serial.println((char)eeprom_value);
#endif
               }
               break;


            case EEPROM_INDEX_MODE:
               {
                  trlh_mode = (TRLH_MODE_TYPE)eeprom_value;

                  if ((trlh_mode != TRLH_MOTORHOME_MODE) && (trlh_mode != TRLH_TRAILER_MODE) && (trlh_mode != TRLH_BUBBLE_MODE))
                  {
                     trlh_mode = TRLH_MOTORHOME_MODE;

                     eeprom_settings_good = false;
                  }

#ifdef DEBUG_EEPROM_READ
                  Serial.print(") TRLH mode:                               ");

                  Serial.print(" 0x");
                  if (eeprom_value < 16)
                  {
                     Serial.print("0");
                  }
                  Serial.print(eeprom_value, HEX);

                  Serial.print(" = ");
                  if (eeprom_value < 100)
                  {
                     Serial.print(" ");
                  }
                  if (eeprom_value < 10)
                  {
                     Serial.print(" ");
                  }
                  Serial.println(eeprom_value);
#endif
               }
               break;


            case EEPROM_INDEX_FRONT_TO_REAR_WHEEL_DISTANCE_IN_INCHES:
               {
                  front_to_rear_wheel_distance_in_inches = eeprom_value;

#ifdef DEBUG_EEPROM_READ
                  Serial.print(") Front to Rear Wheel Distance (in inches):");

                  Serial.print(" 0x");
                  if (eeprom_value < 16)
                  {
                     Serial.print("0");
                  }
                  Serial.print(eeprom_value, HEX);

                  Serial.print(" = ");
                  if (eeprom_value < 100)
                  {
                     Serial.print(" ");
                  }
                  if (eeprom_value < 10)
                  {
                     Serial.print(" ");
                  }
                  Serial.println(eeprom_value);
#endif
               }
               break;

            case EEPROM_INDEX_LEFT_TO_RIGHT_WHEEL_DISTANCE_IN_INCHES:
               {
                  left_to_right_wheel_distance_in_inches = eeprom_value;

#ifdef DEBUG_EEPROM_READ
                  Serial.print(") Left to Right Wheel Distance (in inches):");

                  Serial.print(" 0x");
                  if (eeprom_value < 16)
                  {
                     Serial.print("0");
                  }
                  Serial.print(eeprom_value, HEX);

                  Serial.print(" = ");
                  if (eeprom_value < 100)
                  {
                     Serial.print(" ");
                  }
                  if (eeprom_value < 10)
                  {
                     Serial.print(" ");
                  }
                  Serial.println(eeprom_value);
#endif
               }
               break;


            case EEPROM_INDEX_CHECKSUM:
               {
                  eeprom_value = (char)(xor_result);

#ifdef DEBUG_EEPROM_READ
                  Serial.print(") Calculated CHECKSUM                            = ");
                  showByteValue(xor_result);
                  Serial.println("");
#endif
               }
               break;

            case EEPROM_INDEX_INV_CHECKSUM:
               {
                  eeprom_value = (char)(~xor_result);

#ifdef DEBUG_EEPROM_READ
                  Serial.print(") Calculated INVERSE CHECKSUM                    = ");
                  showByteValue((byte)~xor_result);
                  Serial.println("");
#endif
               }
               break;
         }
      }
#endif

      eeprom_settings_good = true;
   } else {
      Serial.print("EEPROM values failed checksum verification...");
      Serial.println("");
      Serial.print("Discarding invalid EEPROM value settings & storing/using defaults...");
      Serial.println("");

#ifdef DEBUG_EEPROM_READ
      if (!header_is_good)
      {
         Serial.print("HEADER mismatch between EEPROM values & expected values...");
      } else {
         Serial.print("CHECKSUM mismatch between EEPROM values & expected values...");
         Serial.print("(READ ");
         showIndex((int)(EEPROM_INDEX_CHECKSUM));
         Serial.print  (") xor_value          = ");
         Serial.println(xor_value);
         Serial.print("(CALC) xor_result             = ");
         Serial.println(xor_result);
         Serial.print("(READ ");
         showIndex((int)(EEPROM_INDEX_INV_CHECKSUM));
         Serial.print  (") inv_xor_value      = ");
         Serial.println(inv_xor_value);
         Serial.print("(CALC) inv_xor_result         = ");
         Serial.println((byte)~xor_result);
      }

      Serial.println("");
#endif

      eeprom_settings_good = false;
   }

   return (eeprom_settings_good);
}  // read_settings()


// save the current settings to EEPROM
void save_settings(void)
{
   byte xor_result = 0x4D;  // start with a non-zero value
   char eeprom_value = 0x00;

   Serial.println("");
   Serial.print("Saving settings (");
   Serial.print(EEPROM_INDEX_VALUE_COUNT);
   Serial.print(" values) to EEPROM...");
   Serial.println("");

   for (byte eeprom_index = (byte)(EEPROM_INDEX_HEADER_FIRST); eeprom_index <= (byte)(EEPROM_INDEX_INV_CHECKSUM); eeprom_index++)
   {
#ifdef DEBUG_EEPROM_WRITE
      Serial.print("(WRITE ");
      showIndex((int)(eeprom_index));
      Serial.print(": ");
#endif

      switch (eeprom_index)
      {
         case EEPROM_INDEX_HEADER_00:
         case EEPROM_INDEX_HEADER_01:
         case EEPROM_INDEX_HEADER_02:
         case EEPROM_INDEX_HEADER_03:
            {
               eeprom_value = EEPROM_HEADER[eeprom_index];

#ifdef DEBUG_EEPROM_WRITE
               showByteValue(eeprom_value);
               Serial.print(") Header[");
               Serial.print(eeprom_index / 10);
               Serial.print(eeprom_index % 10);
               Serial.print("]                                     = ");
               Serial.println(eeprom_value);
#endif
            }
            break;


         case EEPROM_INDEX_MODE:
            {
               eeprom_value = (byte)trlh_mode;

#ifdef DEBUG_EEPROM_WRITE
               showByteValue(eeprom_value);

               Serial.print(") TRLH mode:                               ");

               Serial.print(" 0x");
               if (eeprom_value < 16)
               {
                  Serial.print("0");
               }
               Serial.print(eeprom_value, HEX);

               Serial.print(" = ");
               if (eeprom_value < 100)
               {
                  Serial.print(" ");
               }
               if (eeprom_value < 10)
               {
                  Serial.print(" ");
               }
               Serial.println((byte)eeprom_value);
#endif
            }
            break;


         case EEPROM_INDEX_FRONT_TO_REAR_WHEEL_DISTANCE_IN_INCHES:
            {
               eeprom_value = front_to_rear_wheel_distance_in_inches;

#ifdef DEBUG_EEPROM_WRITE
               showByteValue(eeprom_value);

               Serial.print(") Front to Rear Wheel Distance (in inches):");

               Serial.print(" 0x");
               if (eeprom_value < 16)
               {
                  Serial.print("0");
               }
               Serial.print(eeprom_value, HEX);

               Serial.print(" = ");
               if (eeprom_value < 100)
               {
                  Serial.print(" ");
               }
               if (eeprom_value < 10)
               {
                  Serial.print(" ");
               }
               Serial.println((byte)eeprom_value);
#endif
            }
            break;

         case EEPROM_INDEX_LEFT_TO_RIGHT_WHEEL_DISTANCE_IN_INCHES:
            {
               eeprom_value = left_to_right_wheel_distance_in_inches;

#ifdef DEBUG_EEPROM_WRITE
               showByteValue(eeprom_value);

               Serial.print(") Left to Right Wheel Distance (in inches):");

               Serial.print(" 0x");
               if (eeprom_value < 16)
               {
                  Serial.print("0");
               }
               Serial.print(eeprom_value, HEX);

               Serial.print(" = ");
               if (eeprom_value < 100)
               {
                  Serial.print(" ");
               }
               if (eeprom_value < 10)
               {
                  Serial.print(" ");
               }
               Serial.println((byte)eeprom_value);
#endif
            }
            break;



         case EEPROM_INDEX_CHECKSUM:
            {
               eeprom_value = (char)(xor_result);

#ifdef DEBUG_EEPROM_WRITE
               showByteValue(eeprom_value);
               Serial.print(") Calculated CHECKSUM                            = ");
               showByteValue(xor_result);
               Serial.println("");
#endif
            }
            break;

         case EEPROM_INDEX_INV_CHECKSUM:
            {
               eeprom_value = (char)(~xor_result);

#ifdef DEBUG_EEPROM_WRITE
               showByteValue(eeprom_value);
               Serial.print(") Calculated INVERSE CHECKSUM                    = ");
               showByteValue((byte)~xor_result);
               Serial.println("");
#endif
            }
            break;
      }

#ifndef DISABLE_EEPROM_WRITE_SETTINGS
      EEPROM.update((int)(eeprom_index), (byte)(eeprom_value));
#endif
      if (eeprom_index < EEPROM_INDEX_CHECKSUM)
      {
         xor_result = (byte)(xor_result ^ eeprom_value);
      }
   }
}  // save_settings()


// one-time setup
void setup(void)
{
   bool toggler = false;

   // Start serial
   Serial.begin(115200);

   delay(500);
   tft.begin();
   tft.setRotation(0);

   delay(100);
   ts.begin();
   ts.setRotation(2);

   pinMode(CHECK_BATTERY_PIN, INPUT);

   draw_splash_screen();

   delay(3000);

   tft.fillScreen(ILI9341_WHITE);

   Serial.println("Initializing BMA400 in I2C mode");

   center_draw_text("Initializing BMA400", 2, tft.width() / 2, 120, ILI9341_BLACK, ILI9341_WHITE);
   center_draw_text("in I2C mode...", 2, tft.width() / 2, 150, ILI9341_BLACK, ILI9341_WHITE);

   delay(1000);

   // Initialize the I2C library
   Wire.begin();

   // Check if sensor is connected and initialize
   // Address is optional (defaults to 0x14)
   while (accelerometer.beginI2C(i2cAddress) != BMA400_OK)
   {
      // Not connected, inform user
      Serial.println("Error: BMA400 not connected, check wiring and I2C address!");

      tft.fillScreen(ILI9341_WHITE);

      if (toggler)
      {
         center_draw_text("BMA400 not found !!", 2, tft.width() / 2, 140, ILI9341_BLACK, ILI9341_WHITE);
      } else {
         center_draw_text("BMA400 not found !!", 2, tft.width() / 2, 140, ILI9341_RED, ILI9341_WHITE);
      }

      toggler = !toggler;

      Serial.println("Initializing BMA400 in I2C mode");

      // Wait a bit to see if connection is established
      delay(1000);
   }

   Serial.println("BMA400 successfully connected !!");

   tft.fillScreen(ILI9341_WHITE);

   center_draw_text("BMA400 connected !!", 2, tft.width() / 2, 140, ILI9341_BLACK, ILI9341_WHITE);

   delay(1000);

   // try to read the settings from EEPROM for this index
   if (!read_settings())
   {
      // if the setting in EEPROM for this index are invalid, then save defaults to this index
      front_to_rear_wheel_distance_in_inches = DEFAULT_FRONT_TO_REAR_MAX_WHEEL_DISTANCE_IN_INCHES;
      left_to_right_wheel_distance_in_inches = DEFAULT_LEFT_TO_RIGHT_MAX_WHEEL_DISTANCE_IN_INCHES;

      save_settings();
      read_settings();
   }

   if (!(radio_not_present))
   {
#ifdef DEBUG_RADIO_MODE
      Serial.println("");
      Serial.println("MODE = RADIO_MODE_INITIAL_SYNC...");
      Serial.println("");
#endif

      radio_mode = RADIO_MODE_INITIAL_SYNC;

      // start by listening for PRIMARY sync msg for three seconds
      radio_setup(NOT_PRIMARY);

      base_time = millis();
   } else {

#ifdef DEBUG_RADIO_MODE
      Serial.println("");
      Serial.println("MODE = RADIO_MODE_I_AM_PRIMARY...");
      Serial.println("");
#endif

      radio_mode = RADIO_MODE_I_AM_PRIMARY;
   }

   initial_battery_update_required = true;

   draw_initial_screen();
}  // setup()


// display the charge status of the battery
void show_battery_status(void)
{
   if (digitalReadFast(CHECK_BATTERY_PIN))
   {
      if ((previous_battery_state) || (initial_battery_update_required))
      {
         initial_battery_update_required = false;

         previous_battery_state = false;

         tft.fillRect(201, 0, 39, 35, ILI9341_GREEN);
         tft.drawRect(203, 2, 35, 31, ILI9341_BLACK);

         center_draw_text("BATT", 1, 221, 13, ILI9341_BLACK, ILI9341_GREEN);
         center_draw_text("OK", 1, 221, 24, ILI9341_BLACK, ILI9341_GREEN);
      }
   } else {
      initial_battery_update_required = false;

      previous_battery_state = true;

      battery_low_toggle = !battery_low_toggle;

      if (battery_low_toggle)
      {
         tft.fillRect(201, 0, 39, 35, ILI9341_RED);
         tft.drawRect(203, 2, 35, 31, ILI9341_BLACK);

         center_draw_text("BATT", 1, 221, 13, ILI9341_BLACK, ILI9341_RED);
         center_draw_text("OK", 1, 221, 24, ILI9341_BLACK, ILI9341_RED);
      } else {
         tft.fillRect(201, 0, 39, 35, ILI9341_BLACK);
         tft.drawRect(203, 2, 35, 31, ILI9341_RED);

         center_draw_text("BATT", 1, 221, 13, ILI9341_RED, ILI9341_BLACK);
         center_draw_text("OK", 1, 221, 24, ILI9341_RED, ILI9341_BLACK);
      }
   }
}  // show_battery_status();


// send the results to the serial console
void show_console_results()
{
   Serial.println("");

   Serial.print("Pitch = ");
   Serial.println(pitch);
   Serial.print("Roll  = ");
   Serial.println(roll);

   Serial.println("");

   switch (trlh_mode)
   {
      case TRLH_MOTORHOME_MODE:
         {
            Serial.print("LEFT FRONT : INCHES TO RAISE = ");
            if (lf_inch_blocks_needed <= MAX_BLOCK_INCHES)
            {
               Serial.println(lf_inch_blocks_needed);
            } else {
               Serial.println("--");
            }
            Serial.print("RIGHT FRONT: INCHES TO RAISE = ");
            if (rf_inch_blocks_needed <= MAX_BLOCK_INCHES)
            {
               Serial.println(rf_inch_blocks_needed);
            } else {
               Serial.println("--");
            }
         }
         break;

      case TRLH_TRAILER_MODE:
         {
            Serial.print("HITCH      : INCHES TO RAISE = ");
            if (hitch_inch_blocks_needed <= MAX_BLOCK_INCHES)
            {
               Serial.println(hitch_inch_blocks_needed);
            } else {
               Serial.println("--");
            }
         }
         break;

      case TRLH_BUBBLE_MODE:
         {
         }
         break;
   }

   if (trlh_mode != TRLH_BUBBLE_MODE)
   {
      Serial.print("LEFT REAR  : INCHES TO RAISE = ");
      if (lr_inch_blocks_needed <= MAX_BLOCK_INCHES)
      {
         Serial.println(lr_inch_blocks_needed);
      } else {
         Serial.println("--");
      }
      Serial.print("RIGHT REAR : INCHES TO RAISE = ");
      if (rr_inch_blocks_needed <= MAX_BLOCK_INCHES)
      {
         Serial.println(rr_inch_blocks_needed);
      } else {
         Serial.println("--");
      }
   }
}  // show_console_results()


// show index of EEPROM reads/writes
void showByteValue(unsigned char byte_value)
{
   if (byte_value < 100)
   {
      Serial.print("0");
   } else {
      Serial.print((char)(((byte_value / 100) % 10) + 0x30));
   }

   if (byte_value < 10)
   {
      Serial.print("0");
   } else {
      Serial.print((char)(((byte_value / 10) % 10) + 0x30));
   }

   Serial.print((char)((byte_value % 10) + 0x30));
}  // showByteValue()


// show index of EEPROM reads/writes
void showIndex(int index)
{
   if (index < 1000)
   {
      Serial.print("0");
   } else {
      Serial.print((char)((index / 1000) + 0x30));
   }

   if (index < 100)
   {
      Serial.print("0");
   } else {
      Serial.print((char)(((index / 100) % 10) + 0x30));
   }

   if (index < 10)
   {
      Serial.print("0");
   } else {
      Serial.print((char)(((index / 10) % 10) + 0x30));
   }

   Serial.print((char)((index % 10) + 0x30));
}  // showIndex()


// EOF
