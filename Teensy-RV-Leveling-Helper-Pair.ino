//
// Teensy RV Leveling Helper Pair using NRF24L01 - version 1.6 dated 07/30/2024 @0635
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
// Also allows touchscreen input & color display to allow custom color selection, mapping, and storage for
//    WS2812B LED strip lighting
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

// #define DEBUG_RADIO_MODE                // uncomment to troubleshoot radio mode transitions
// #define DEBUG_RADIO_MSGS                // uncomment to troubleshoot radio mode messages
// #define DEBUG_TS                        // uncomment to troubleshoot touchscreen touch locations
// #define DEBUG_ACCELEROMETER             // uncomment to troubleshoot accelerometer vector
// #define DEBUG_RESULTS                   // uncomment to troubleshoot leveling results
// #define DISABLE_EEPROM_READ_SETTINGS    // uncomment to not read settings from EEPROM (to simply use program defaults for all settings)
// #define DISABLE_EEPROM_WRITE_SETTINGS   // uncomment to not write settings to EEPROM
// #define DEBUG_EEPROM_READ               // uncomment to debug reads from the EEPROM
// #define DEBUG_EEPROM_WRITE              // uncomment to debug writes to the EEPROM

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
// PIN  D8      = WS2812B RGB LED strip is connected to this pin
// PIN  D9      = TFT/TS data/command select
// PIN D10      = ~FORCE_LED_COLOR_MAPPER_MODE
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
String DATETIME = ("20240730-0635");
String VERSION = ("1.1");
String TITLE_RV = ("Teensy RV Leveling Helper Pair");
String TITLE_LED = ("Teensy RGB Color Mapper v" + VERSION);
String TITLE_A = ("Teensy RV");
String TITLE_B = ("Leveling Helper");
String TITLE_C = ("w/ LED Control");
String AUTHOR = ("by MJCulross (KD5RXT) " + DATETIME);
String VERSION0 = ("version " + VERSION + " - " + DATETIME);
String VERSION0_A = ("Version " + VERSION);
String VERSION0_B = DATETIME;
String VERSION1 = ("created by: MJCulross (KD5RXT)");
String VERSION1_A = ("created by:");
String VERSION1_B = ("Mark J Culross");
String VERSION1_C = ("kd5rxt@arrl.net");
String SERNUM = ("S/N: A-xxxxx");
String RV_INSIDE_UNIT = ("[ INSIDE UNIT ]");
String RV_STANDALONE_UNIT = ("[ STANDALONE UNIT ]");
String RV_OUTSIDE_UNIT = ("[ OUTSIDE UNIT ]");
String PAIRING = ("[ PAIRING ]");
String LED_INSIDE_UNIT = ("[ LED CONTROL ]");
String LED_STANDALONE_UNIT = ("[ STANDALONE ]");
String LED_OUTSIDE_UNIT = ("[ REMOTE CONTROL ]");

#include <SPI.h>
#include <RF24.h>
#include <OctoWS2811.h>
#include <EEPROM.h>
#include <ILI9341_t3.h>
#include <XPT2046_Touchscreen.h>
#include "RGB_GRB_colors.h"

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

// connect this pin to GND to force operation into LED MAPPER mode (for an LED controller unit without an attached display)
#define FORCE_LED_COLOR_MAPPER_MODE 10

// keep track of how often to check battery status
const int CHECK_BATTERY_MILLIS = 500;
unsigned long check_battery_time = millis();
boolean battery_low_toggle = true;
boolean initial_battery_update_required = true;
boolean previous_battery_state = false;

// keep track of how often to check buttons
const int CHECK_BUTTONS_MILLIS = 50;
unsigned long check_buttons_time = millis();


// The display uses hardware SPI, with pin #9 as DataCommand & pin #14 as ChipSelect
// MOSI=11, MISO=12, SCK=13
const int TFT_CHIP_SELECT = 14;
const int TFT_DATA_COMMAND = 9;
ILI9341_t3 tft = ILI9341_t3(TFT_CHIP_SELECT, TFT_DATA_COMMAND);

// The touchscreen uses hardware SPI, with pin #9 as DataCommand & pin #5 as ChipSelect
// MOSI=11, MISO=12, SCK=13
#define TS_CS_PIN 5

XPT2046_Touchscreen ts(TS_CS_PIN);

// The nRF24L01+ 2.4GHz wireless module also uses hardware SPI, plus pin #7 as CE & pin #6 as CSN
#define RF24_CHIP_ENABLE 7
#define RF24_CHIP_SELECT_NOT 6
// >> special for RV LED controller on which pins 6 & 7 were shorted to ground
//#define RF24_CHIP_ENABLE 4
//#define RF24_CHIP_SELECT_NOT 5

RF24 radio(RF24_CHIP_ENABLE, RF24_CHIP_SELECT_NOT);

#define IS_PRIMARY true
#define NOT_PRIMARY false

#define INITIAL_LISTEN_FOR_PRIMARY_SYNC_IN_MILLIS 4000
#define SEND_SYNC_INTERVAL_IN_MILLIS 200
#define SEND_KEEPALIVE_INTERVAL_IN_MILLIS 1000

#define RADIO_PACKET_SIZE 32

// Pipe addresses to allow the primary & secondary to communicate
const byte primary_tx_pipe[] = "P_TXP";
const byte secondary_tx_pipe[] = "S_TXP";

#define RADIO_CHANNEL_MIN 1
#define RADIO_CHANNEL_MAX 125
#define RV_RADIO_CHANNEL_DEFAULT 112
#define LED_RADIO_CHANNEL_DEFAULT 113
int radio_channel = RV_RADIO_CHANNEL_DEFAULT;

typedef enum {
   RADIO_POWER_LEVEL_LOW = 0,
   RADIO_POWER_LEVEL_MID,
   RADIO_POWER_LEVEL_HIGH,
} RADIO_POWER_LEVEL;
RADIO_POWER_LEVEL radio_power_level = RADIO_POWER_LEVEL_HIGH;

// define constant message strings
const char MSG_I_AM_PRIMARY[] = "P-SYNC";
const char MSG_I_AM_SECONDARY[] = "S-SYNC";

typedef enum {
   RADIO_MODE_INITIAL_SYNC = 0,
   RADIO_MODE_SYNC_UP,
   RADIO_MODE_I_AM_PRIMARY,
   RADIO_MODE_I_AM_SECONDARY,
} RADIO_MODE_TYPE;

RADIO_MODE_TYPE radio_mode;

boolean radio_not_present = false;

const int MAX_BLOCK_INCHES = 12;

const int block_00_inch_color = 0x0600;  // green
const int block_01_inch_color = 0x0600;  //
const int block_02_inch_color = 0xff60;  // yellow
const int block_03_inch_color = 0xff60;  //
const int block_04_inch_color = 0xff60;  //
const int block_05_inch_color = 0xfc00;  // orange
const int block_06_inch_color = 0xfc00;  //
const int block_07_inch_color = 0xfc00;  //
const int block_08_inch_color = 0xfc00;  //
const int block_09_inch_color = 0xf800;  // red
const int block_10_inch_color = 0xf800;  //
const int block_11_inch_color = 0xf800;  //
const int block_12_inch_color = 0xf800;  //

#include "SparkFun_BMA400_Arduino_Library.h"

// Create a new sensor object
BMA400 accelerometer;

// I2C address selection
uint8_t i2cAddress = BMA400_I2C_ADDRESS_DEFAULT;  // 0x14
//uint8_t i2cAddress = BMA400_I2C_ADDRESS_SECONDARY; // 0x15

typedef enum {
   OPS_MODE_TYPE_RV_LEVELING = 0,
   OPS_MODE_TYPE_LED_COLOR_MAPPER,
} OPS_MODE_TYPE;

OPS_MODE_TYPE ops_mode = OPS_MODE_TYPE_RV_LEVELING;

typedef enum {
   TRLH_MOTORHOME_MODE = 0,
   TRLH_TRAILER_MODE,
   TRLH_BUBBLE_MODE,
} TRLH_MODE_TYPE;

TRLH_MODE_TYPE trlh_mode = TRLH_MOTORHOME_MODE;

const int FRONT_TO_REAR_MIN_WHEEL_DISTANCE_IN_INCHES = 60;
const int FRONT_TO_REAR_MAX_WHEEL_DISTANCE_IN_INCHES = 255;

const int LEFT_TO_RIGHT_MIN_WHEEL_DISTANCE_IN_INCHES = 60;
const int LEFT_TO_RIGHT_MAX_WHEEL_DISTANCE_IN_INCHES = 84;

const int DEFAULT_FRONT_TO_REAR_MAX_WHEEL_DISTANCE_IN_INCHES = 212;
const int DEFAULT_LEFT_TO_RIGHT_MAX_WHEEL_DISTANCE_IN_INCHES = 72;

int front_to_rear_wheel_distance_in_inches = DEFAULT_FRONT_TO_REAR_MAX_WHEEL_DISTANCE_IN_INCHES;
int left_to_right_wheel_distance_in_inches = DEFAULT_LEFT_TO_RIGHT_MAX_WHEEL_DISTANCE_IN_INCHES;

float pitch_avg = 0.0;
float roll_avg = 0.0;

int counter = 0;
const int NUM_SAMPLES_PER_AVG = 200;

float pitch = 0.0;
float roll = 0.0;

int hitch_inch_blocks_needed = 0;
int lf_inch_blocks_needed = 0;
int rf_inch_blocks_needed = 0;
int lr_inch_blocks_needed = 0;
int rr_inch_blocks_needed = 0;

float previous_x_roll = 0;
float previous_y_pitch = 0;


// Any group of digital pins may be used
const int numPins = 1;
byte pinList[numPins] = { 8 };

#define NUM_LEDS 300

const int ledsPerStrip = NUM_LEDS;

// These buffers need to be large enough for all the pixels.
// The total number of pixels is "ledsPerStrip * numPins".
// Each pixel needs 3 bytes, so multiply by 3.  An "int" is
// 4 bytes, so divide by 4.  The array is created using "int"
// so the compiler will align it to 32 bit memory.
DMAMEM int displayMemory[ledsPerStrip * numPins * 3 / 4];
int drawingMemory[ledsPerStrip * numPins * 3 / 4];

const int config = WS2811_GRB | WS2811_800kHz;

OctoWS2811 leds(ledsPerStrip, displayMemory, drawingMemory, config, numPins, pinList);

uint32_t LEDimage[NUM_LEDS];

// keep track of how often to check for time for LED updates
const int UPDATE_REQUIRED_MILLIS = 25;
unsigned long update_required_time = millis();

boolean update_required = false;
int update_delay_counter = 0;

#define UV_PURPLE 0x3800FF
#define SPLASH_COLOR UV_PURPLE

int cycle_index = 0;
int marquis_index = 0;
int fade_index = 0;
int fade_level = 0;
int xfade_index = 0;
int xfade_level = 0;
int zipper_index = 0;

typedef enum
{
   PATTERN_TYPE_MARQUIS_FORWARD = 0,
   PATTERN_TYPE_MARQUIS_REVERSE,
   PATTERN_TYPE_CYCLE_FORWARD,
   PATTERN_TYPE_CYCLE_REVERSE,
   PATTERN_TYPE_FADE_FORWARD,
   PATTERN_TYPE_FADE_REVERSE,
   PATTERN_TYPE_CROSSFADE_FORWARD,
   PATTERN_TYPE_CROSSFADE_REVERSE,
   PATTERN_TYPE_ZIPPER_FORWARD,
   PATTERN_TYPE_ZIPPER_REVERSE,
   PATTERN_TYPE_RANDOM,
} PATTERN_TYPE;

PATTERN_TYPE pattern_type = PATTERN_TYPE_MARQUIS_FORWARD;

int display_color;
int led_strip_color;
int red_level = 255;
int green_level = 255;
int blue_level = 255;

boolean leds_on = true;
boolean read_led = false;

#define PATTERN_LENGTH_MAX 32

int led_list[PATTERN_LENGTH_MAX] =
{
   ILI9341_WHITE,
   ILI9341_WHITE,
   ILI9341_WHITE,
   ILI9341_WHITE,
   ILI9341_WHITE,
   ILI9341_WHITE,
   ILI9341_WHITE,
   ILI9341_WHITE,
   ILI9341_WHITE,
   ILI9341_WHITE,
   ILI9341_WHITE,
   ILI9341_WHITE,
   ILI9341_WHITE,
   ILI9341_WHITE,
   ILI9341_WHITE,
   ILI9341_WHITE,
   ILI9341_WHITE,
   ILI9341_WHITE,
   ILI9341_WHITE,
   ILI9341_WHITE,
   ILI9341_WHITE,
   ILI9341_WHITE,
   ILI9341_WHITE,
   ILI9341_WHITE,
   ILI9341_WHITE,
   ILI9341_WHITE,
   ILI9341_WHITE,
   ILI9341_WHITE,
   ILI9341_WHITE,
   ILI9341_WHITE,
   ILI9341_WHITE,
   ILI9341_WHITE,
};

#define MY_ILI9341_GRAY 0x8410
#define MY_ILI9341_ORANGE 0xFB00

#define NUMBER_OF_PATTERNS 10

int pattern_number = 0;
int pattern_length = 32;
int pattern_speed = 0;

int bright_level = 5;
int dynamic_bright_level = 5;

typedef enum
{
   BUTTON_PRESS_TYPE_NOT_PRESSED = 0,
   BUTTON_PRESS_TYPE_SHORT_PRESS,
   BUTTON_PRESS_TYPE_MEDIUM_PRESS,
   BUTTON_PRESS_TYPE_LONG_PRESS,
} BUTTON_PRESS_TYPE;

BUTTON_PRESS_TYPE button_press_type = BUTTON_PRESS_TYPE_NOT_PRESSED;

boolean save_settings_needed = false;

unsigned long base_time;

char radio_rx_buffer[RADIO_PACKET_SIZE];

const char RV_EEPROM_HEADER[] = "TRLH";
const char LED_EEPROM_HEADER[] = "TLCM";

typedef enum
{
   EEPROM_INDEX_RV_LEVELING_HEADER_00 = 0,
   EEPROM_INDEX_RV_LEVELING_HEADER_01,
   EEPROM_INDEX_RV_LEVELING_HEADER_02,
   EEPROM_INDEX_RV_LEVELING_HEADER_03,

   EEPROM_INDEX_RV_LEVELING_MODE,

   EEPROM_INDEX_FRONT_TO_REAR_WHEEL_DISTANCE_IN_INCHES,
   EEPROM_INDEX_LEFT_TO_RIGHT_WHEEL_DISTANCE_IN_INCHES,

   EEPROM_INDEX_RV_LEVELING_CHECKSUM,
   EEPROM_INDEX_RV_LEVELING_INV_CHECKSUM,

   EEPROM_INDEX_RV_LEVELING_VALUE_COUNT,

   EEPROM_INDEX_RV_RADIO_CHANNEL_NUMBER = EEPROM_INDEX_RV_LEVELING_VALUE_COUNT,
   EEPROM_INDEX_RV_INV_RADIO_CHANNEL_NUMBER,

   EEPROM_INDEX_RADIO_POWER_LEVEL,
   EEPROM_INDEX_INV_RADIO_POWER_LEVEL,

   EEPROM_INDEX_LED_HEADER_00,
   EEPROM_INDEX_LED_HEADER_01,
   EEPROM_INDEX_LED_HEADER_02,
   EEPROM_INDEX_LED_HEADER_03,

   EEPROM_INDEX_LED_PATTERN_TYPE,
   EEPROM_INDEX_LED_PATTERN_SPEED,
   EEPROM_INDEX_LED_PATTERN_LENGTH,
   EEPROM_INDEX_LED_COLOR_01_HI_BYTE,
   EEPROM_INDEX_LED_COLOR_01_MID_BYTE,
   EEPROM_INDEX_LED_COLOR_01_LO_BYTE,
   EEPROM_INDEX_LED_COLOR_02_HI_BYTE,
   EEPROM_INDEX_LED_COLOR_02_MID_BYTE,
   EEPROM_INDEX_LED_COLOR_02_LO_BYTE,
   EEPROM_INDEX_LED_COLOR_03_HI_BYTE,
   EEPROM_INDEX_LED_COLOR_03_MID_BYTE,
   EEPROM_INDEX_LED_COLOR_03_LO_BYTE,
   EEPROM_INDEX_LED_COLOR_04_HI_BYTE,
   EEPROM_INDEX_LED_COLOR_04_MID_BYTE,
   EEPROM_INDEX_LED_COLOR_04_LO_BYTE,
   EEPROM_INDEX_LED_COLOR_05_HI_BYTE,
   EEPROM_INDEX_LED_COLOR_05_MID_BYTE,
   EEPROM_INDEX_LED_COLOR_05_LO_BYTE,
   EEPROM_INDEX_LED_COLOR_06_HI_BYTE,
   EEPROM_INDEX_LED_COLOR_06_MID_BYTE,
   EEPROM_INDEX_LED_COLOR_06_LO_BYTE,
   EEPROM_INDEX_LED_COLOR_07_HI_BYTE,
   EEPROM_INDEX_LED_COLOR_07_MID_BYTE,
   EEPROM_INDEX_LED_COLOR_07_LO_BYTE,
   EEPROM_INDEX_LED_COLOR_08_HI_BYTE,
   EEPROM_INDEX_LED_COLOR_08_MID_BYTE,
   EEPROM_INDEX_LED_COLOR_08_LO_BYTE,
   EEPROM_INDEX_LED_COLOR_09_HI_BYTE,
   EEPROM_INDEX_LED_COLOR_09_MID_BYTE,
   EEPROM_INDEX_LED_COLOR_09_LO_BYTE,
   EEPROM_INDEX_LED_COLOR_10_HI_BYTE,
   EEPROM_INDEX_LED_COLOR_10_MID_BYTE,
   EEPROM_INDEX_LED_COLOR_10_LO_BYTE,
   EEPROM_INDEX_LED_COLOR_11_HI_BYTE,
   EEPROM_INDEX_LED_COLOR_11_MID_BYTE,
   EEPROM_INDEX_LED_COLOR_11_LO_BYTE,
   EEPROM_INDEX_LED_COLOR_12_HI_BYTE,
   EEPROM_INDEX_LED_COLOR_12_MID_BYTE,
   EEPROM_INDEX_LED_COLOR_12_LO_BYTE,
   EEPROM_INDEX_LED_COLOR_13_HI_BYTE,
   EEPROM_INDEX_LED_COLOR_13_MID_BYTE,
   EEPROM_INDEX_LED_COLOR_13_LO_BYTE,
   EEPROM_INDEX_LED_COLOR_14_HI_BYTE,
   EEPROM_INDEX_LED_COLOR_14_MID_BYTE,
   EEPROM_INDEX_LED_COLOR_14_LO_BYTE,
   EEPROM_INDEX_LED_COLOR_15_HI_BYTE,
   EEPROM_INDEX_LED_COLOR_15_MID_BYTE,
   EEPROM_INDEX_LED_COLOR_15_LO_BYTE,
   EEPROM_INDEX_LED_COLOR_16_HI_BYTE,
   EEPROM_INDEX_LED_COLOR_16_MID_BYTE,
   EEPROM_INDEX_LED_COLOR_16_LO_BYTE,
   EEPROM_INDEX_LED_COLOR_17_HI_BYTE,
   EEPROM_INDEX_LED_COLOR_17_MID_BYTE,
   EEPROM_INDEX_LED_COLOR_17_LO_BYTE,
   EEPROM_INDEX_LED_COLOR_18_HI_BYTE,
   EEPROM_INDEX_LED_COLOR_18_MID_BYTE,
   EEPROM_INDEX_LED_COLOR_18_LO_BYTE,
   EEPROM_INDEX_LED_COLOR_19_HI_BYTE,
   EEPROM_INDEX_LED_COLOR_19_MID_BYTE,
   EEPROM_INDEX_LED_COLOR_19_LO_BYTE,
   EEPROM_INDEX_LED_COLOR_20_HI_BYTE,
   EEPROM_INDEX_LED_COLOR_20_MID_BYTE,
   EEPROM_INDEX_LED_COLOR_20_LO_BYTE,
   EEPROM_INDEX_LED_COLOR_21_HI_BYTE,
   EEPROM_INDEX_LED_COLOR_21_MID_BYTE,
   EEPROM_INDEX_LED_COLOR_21_LO_BYTE,
   EEPROM_INDEX_LED_COLOR_22_HI_BYTE,
   EEPROM_INDEX_LED_COLOR_22_MID_BYTE,
   EEPROM_INDEX_LED_COLOR_22_LO_BYTE,
   EEPROM_INDEX_LED_COLOR_23_HI_BYTE,
   EEPROM_INDEX_LED_COLOR_23_MID_BYTE,
   EEPROM_INDEX_LED_COLOR_23_LO_BYTE,
   EEPROM_INDEX_LED_COLOR_24_HI_BYTE,
   EEPROM_INDEX_LED_COLOR_24_MID_BYTE,
   EEPROM_INDEX_LED_COLOR_24_LO_BYTE,
   EEPROM_INDEX_LED_COLOR_25_HI_BYTE,
   EEPROM_INDEX_LED_COLOR_25_MID_BYTE,
   EEPROM_INDEX_LED_COLOR_25_LO_BYTE,
   EEPROM_INDEX_LED_COLOR_26_HI_BYTE,
   EEPROM_INDEX_LED_COLOR_26_MID_BYTE,
   EEPROM_INDEX_LED_COLOR_26_LO_BYTE,
   EEPROM_INDEX_LED_COLOR_27_HI_BYTE,
   EEPROM_INDEX_LED_COLOR_27_MID_BYTE,
   EEPROM_INDEX_LED_COLOR_27_LO_BYTE,
   EEPROM_INDEX_LED_COLOR_28_HI_BYTE,
   EEPROM_INDEX_LED_COLOR_28_MID_BYTE,
   EEPROM_INDEX_LED_COLOR_28_LO_BYTE,
   EEPROM_INDEX_LED_COLOR_29_HI_BYTE,
   EEPROM_INDEX_LED_COLOR_29_MID_BYTE,
   EEPROM_INDEX_LED_COLOR_29_LO_BYTE,
   EEPROM_INDEX_LED_COLOR_30_HI_BYTE,
   EEPROM_INDEX_LED_COLOR_30_MID_BYTE,
   EEPROM_INDEX_LED_COLOR_30_LO_BYTE,
   EEPROM_INDEX_LED_COLOR_31_HI_BYTE,
   EEPROM_INDEX_LED_COLOR_31_MID_BYTE,
   EEPROM_INDEX_LED_COLOR_31_LO_BYTE,
   EEPROM_INDEX_LED_COLOR_32_HI_BYTE,
   EEPROM_INDEX_LED_COLOR_32_MID_BYTE,
   EEPROM_INDEX_LED_COLOR_32_LO_BYTE,

   EEPROM_INDEX_LED_CHECKSUM,
   EEPROM_INDEX_LED_INV_CHECKSUM,

   EEPROM_INDEX_LED_VALUE_COUNT = EEPROM_INDEX_LED_INV_CHECKSUM - EEPROM_INDEX_INV_RADIO_POWER_LEVEL,

   EEPROM_INDEX_LED_PATTERN_NUMBER = EEPROM_INDEX_LED_HEADER_00 + (NUMBER_OF_PATTERNS * (EEPROM_INDEX_LED_VALUE_COUNT)),
   EEPROM_INDEX_INV_LED_PATTERN_NUMBER,

   EEPROM_INDEX_LED_RADIO_CHANNEL_NUMBER,
   EEPROM_INDEX_LED_INV_RADIO_CHANNEL_NUMBER,

   EEPROM_INDEX_LED_BRIGHT_LEVEL,
   EEPROM_INDEX_LED_INV_BRIGHT_LEVEL,
} EEPROM_INDEX;

#define EEPROM_INDEX_RV_LEVELING_HEADER_FIRST EEPROM_INDEX_RV_LEVELING_HEADER_00
#define EEPROM_INDEX_RV_LEVELING_HEADER_LAST EEPROM_INDEX_RV_LEVELING_HEADER_03

#define EEPROM_INDEX_LED_HEADER_FIRST EEPROM_INDEX_LED_HEADER_00
#define EEPROM_INDEX_LED_HEADER_LAST EEPROM_INDEX_LED_HEADER_03

unsigned int MAX_T4X_EEPROM_SIZE_ALLOWED = EEPROM.length();



// forward definitions
uint32_t apply_intensity_to_color(uint32_t color);
void apply_pattern(void);
void calculate_results(void);
void center_draw_text(const String text, unsigned int textSize, unsigned int xCenterLoc, unsigned int yCenterLoc, uint16_t textColor, uint16_t textBackground);
void change_operating_mode(void);
void debug_touch(TS_Point p);
void display_splash(void);
void draw_initial_screen(void);
void draw_results(void);
void draw_splash_screen(void);
void draw_tire(int x, int y, int x_size, int y_size, uint32_t color);
void initialize_for_led_color_mapper(void);
void initialize_for_rv_leveling(void);
void led_color_mapper_radio_status(void);
void loop(void);
void process_buttons(void);
void process_button_inputs(TS_Point p);
void process_radio(void);
boolean radio_receive(void);
void radio_send(const char *tx_string);
void radio_setup(boolean is_primary);
void radio_status(void);
void read_radio_channel_number(void);
boolean read_settings(void);
void rv_radio_status(void);
void save_bright_level(void);
void save_pattern_number(void);
void save_radio_channel_number(void);
void save_settings(void);
void setup(void);
void share_blue_value(void);
void share_green_value(void);
void share_leds(void);
void share_leds_on_button(void);
void share_bright_level(void);
void share_pattern_length(void);
void share_pattern_number(void);
void share_pattern_speed(void);
void share_pattern_type(void);
void share_red_value(void);
void show_battery_status(void);
void show_console_results();
void show_index(int index);
void update_bright_level(void);
void update_led_radio_channel(void);
void update_led_radio_power(void);
void update_leds(void);
void update_leds_on_button(void);
void update_pattern_length(void);
void update_pattern_number(void);
void update_pattern_speed(void);
void update_pattern_type(void);
void update_radio_channel(void);
void update_rv_radio_channel(void);
void update_rv_radio_power(void);
void update_settings(void);
void update_value(unsigned int currentValue, unsigned int xCenter, unsigned int yCenter, unsigned int textColor);



// apply intensity setting to a color
uint32_t apply_intensity_to_color(uint32_t color)
{
   uint32_t adjusted_color;

   adjusted_color = ((color / 0x10000) / (1 << (8 - dynamic_bright_level))) * 0x10000;  // RED

   adjusted_color += (((color / 0x100) % 0x100) / (1 << (8 - dynamic_bright_level))) * 0x100;  // GREEN

   adjusted_color += ((color % 0x100) / (1 << (8 - dynamic_bright_level)));  // BLUE

   return (adjusted_color);
}  // apply_intensity_to_color()


// apply the currently selected pattern number
void apply_pattern(void)
{
   cycle_index = 0;
   marquis_index = 0;
   fade_index = 0;
   fade_level = 0;
   xfade_index = 0;
   xfade_level = 0;
   zipper_index = 0;

   read_settings();

   update_value(red_level, 20, 50, ILI9341_RED);
   update_value(green_level, 60, 50, ILI9341_GREEN);
   update_value(blue_level, 100, 50, ILI9341_BLUE);

   tft.setTextSize(1);
   tft.setTextColor(ILI9341_WHITE, ILI9341_BLACK);

   update_settings();

   update_leds();

   update_pattern_number();

   update_pattern_type();

   update_pattern_speed();

   update_pattern_length();

   update_required = true;
}  // apply_pattern()


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

               hitch_inch_blocks_needed -= lr_inch_blocks_needed;
               rr_inch_blocks_needed -= lr_inch_blocks_needed;
               lr_inch_blocks_needed = 0;
            }

            // hitch is highest, & leaning left
            if ((pitch >= 0) && (roll < 0))
            {
               hitch_inch_blocks_needed = 0;
               lr_inch_blocks_needed = int(0.1 + abs(front_to_rear_wheel_distance_in_inches * sin(pitch * PI / 180.0)) + abs(left_to_right_wheel_distance_in_inches * sin(roll * PI / 180.0)));
               rr_inch_blocks_needed = int(0.1 + abs(front_to_rear_wheel_distance_in_inches * sin(pitch * PI / 180.0)));

               hitch_inch_blocks_needed -= rr_inch_blocks_needed;
               lr_inch_blocks_needed -= rr_inch_blocks_needed;
               rr_inch_blocks_needed = 0;
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
            xOffset = (text.length() * 18) / 2 - 1;
            yOffset = 10;
         }
         break;

      case 2:  // 10x14, maximum chars = 20
         {
            xOffset = ((text.length() * 12) / 2) - 1;
            yOffset = 7;
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


// change the current operating mode
void change_operating_mode(void)
{
   Serial.println(TITLE_RV);
   Serial.println(VERSION0);
   Serial.println(VERSION1);

   switch (ops_mode)
   {
      case OPS_MODE_TYPE_RV_LEVELING:
         {
            initialize_for_rv_leveling();
         }
         break;

      case OPS_MODE_TYPE_LED_COLOR_MAPPER:
         {
            initialize_for_led_color_mapper();
         }
         break;
   }
}  // change_operating_mode()


// debug the touchscreen touches
void debug_touch(TS_Point p)
{
   switch (ops_mode)
   {
      case OPS_MODE_TYPE_RV_LEVELING:
         {
            // show where the screen was touched (for debugging purposes)
            tft.fillRect(65, 249, 110, 10, ILI9341_WHITE);
            tft.setTextColor(ILI9341_BLACK);
            tft.setTextSize(1);
            tft.setCursor(68, 250);
            tft.print("X:");
            tft.print(p.x);
            tft.print(" Y:");
            tft.print(p.y);
            tft.print(" Z:");
            tft.print(p.z);

            tft.fillRect(220, 240, 110, 10, ILI9341_WHITE);
            tft.setCursor(222, 240);
            tft.print("B:");
            tft.print(button_press_type);
         }
         break;

      case OPS_MODE_TYPE_LED_COLOR_MAPPER:
         {
            // show where the screen was touched (for debugging purposes)
            tft.fillRect(0, 230, 105, 10, ILI9341_BLACK);
            tft.setTextColor(ILI9341_WHITE);
            tft.setTextSize(1);
            tft.setCursor(3, 231);
            tft.print("X:");
            tft.print(p.x);
            tft.print(" Y:");
            tft.print(p.y);
            tft.print(" Z:");
            tft.print(p.z);

            tft.fillRect(280, 230, 20, 10, ILI9341_BLACK);
            tft.setCursor(283, 231);
            tft.print("B:");
            tft.print(button_press_type);
         }
         break;
   }
}  // debug_touch


// generate the splash display
void display_splash(void)
{
   dynamic_bright_level = 2;

   // blink the LED with increasing intensity at startup
   for (int i = 0; i < 5; i++)
   {
      for (int j = 0; j < 6; j++)
      {
         // turn all pixels on
         for (int i = 0; i < leds.numPixels(); i++)
         {
            leds.setPixel(i, apply_intensity_to_color(SPLASH_COLOR));
         }
         leds.show();

         // wait for 1/20 second
         delay(50);

         // turn all pixels off
         for (int i = 0; i < leds.numPixels(); i++)
         {
            leds.setPixel(i, 0x000000);
         }
         leds.show();

         // wait for 1/20 second
         delay(50);
      }

      dynamic_bright_level++;

      // wait for 1/4 second
      delay(250);
   }
}  // display_splash()


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

   center_draw_text(TITLE_RV, 1, 100, 5, ILI9341_GREEN, ILI9341_BLACK);
   center_draw_text(VERSION0, 1, 100, 20, ILI9341_RED, ILI9341_BLACK);
   center_draw_text(VERSION1, 1, 100, 30, ILI9341_YELLOW, ILI9341_BLACK);

   tft.setCursor(5, 45);
   tft.setTextColor(ILI9341_BLACK, ILI9341_WHITE);
   tft.print("RADIO CHAN:");

   update_rv_radio_channel();

   tft.setCursor(5, 60);
   tft.setTextColor(ILI9341_BLACK, ILI9341_WHITE);
   tft.print("RADIO PWR :");

   update_rv_radio_power();

   radio_status();

   switch (trlh_mode)
   {
      case TRLH_MOTORHOME_MODE:
         {
            tft.drawRect(75, 110, 90, 170, ILI9341_BLACK);  // box outline
            tft.drawRect(85, 90, 70, 50, ILI9341_BLACK);    // cab outline
            tft.drawRect(95, 75, 50, 5, ILI9341_BLACK);     // front bumper
            tft.drawRect(80, 281, 80, 8, ILI9341_BLACK);    // rear bumper

            tft.drawLine(85, 90, 150, 90, ILI9341_WHITE);  // erase front of cab outline
            tft.drawCircle(95, 90, 10, ILI9341_BLACK);     // lf cab
            tft.drawCircle(144, 90, 10, ILI9341_BLACK);    // rf cab
            tft.drawLine(95, 80, 140, 80, ILI9341_BLACK);  // front of cab
            tft.fillRect(95, 81, 50, 20, ILI9341_WHITE);   // erase inside of cab top
            tft.fillRect(86, 90, 20, 20, ILI9341_WHITE);   // erase inside of cab lh side
            tft.fillRect(134, 90, 20, 20, ILI9341_WHITE);  // erase inside of cab rh side

            draw_tire(89, 95, 10, 30, ILI9341_BLACK);   // lf tire
            draw_tire(141, 95, 10, 30, ILI9341_BLACK);  // rf tire

            draw_tire(89, 215, 10, 30, ILI9341_BLACK);   // lr inside tire
            draw_tire(141, 215, 10, 30, ILI9341_BLACK);  // rr inside tire
            draw_tire(77, 215, 10, 30, ILI9341_BLACK);   // lr outside tire
            draw_tire(153, 215, 10, 30, ILI9341_BLACK);  // rr outside tire

            tft.setTextColor(ILI9341_BLACK, ILI9341_WHITE);

            tft.setTextSize(2);

            tft.setCursor(5, 80);
            tft.print("RAISE");

            tft.setCursor(175, 80);
            tft.print("RAISE");

            center_draw_text("(front)", 1, tft.width() / 2, 90, ILI9341_BLACK, ILI9341_WHITE);
            center_draw_text("(rear)", 1, tft.width() / 2, 270, ILI9341_BLACK, ILI9341_WHITE);
         }
         break;

      case TRLH_TRAILER_MODE:
         {
            tft.drawRect(75, 120, 90, 160, ILI9341_BLACK);  // box outline

            for (int x = 118; x <= 122; x++)
            {
               tft.drawLine(x, 120, x, 82, ILI9341_BLACK);  // tongue
            }

            for (int x = 90; x <= 95; x++)
            {
               tft.drawLine(x, 120, 118, (x - 3), ILI9341_BLACK);  // lh side of tongue triangle
            }

            for (int x = 150; x >= 145; x--)
            {
               tft.drawLine(x, 120, 122, (237 - x), ILI9341_BLACK);  // rh side of tongue triangle
            }

            tft.drawCircle(120, 79, 4, ILI9341_BLACK);      // hitch

            draw_tire(77, 215, 10, 30, ILI9341_BLACK);   // lr inside tire
            draw_tire(153, 215, 10, 30, ILI9341_BLACK);  // rr inside tire

            tft.setTextColor(ILI9341_BLACK, ILI9341_WHITE);

            tft.setTextSize(2);

            tft.setCursor(5, 80);
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
      tft.setCursor(5, 200);
      tft.print("RAISE");

      tft.setCursor(175, 200);
      tft.print("RAISE");

      tft.setTextSize(1);

      center_draw_text("PITCH (F to B)", 1, tft.width() / 2, 150, ILI9341_BLACK, ILI9341_WHITE);
      center_draw_text("ROLL (L to R)", 1, tft.width() / 2, 185, ILI9341_BLACK, ILI9341_WHITE);

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

   tft.drawRect(8, 257, 61, 26, ILI9341_BLACK);
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
            draw_tire(89, 95, 10, 30, tire_color);  // lf tire

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
            draw_tire(141, 95, 10, 30, tire_color);  // rf tire
         }
         break;

      case TRLH_TRAILER_MODE:
         {
            switch (hitch_inch_blocks_needed)
            {
               case -11:
                  {
                     tire_color = block_11_inch_color;
                  }
                  break;

               case -10:
                  {
                     tire_color = block_10_inch_color;
                  }
                  break;

               case -9:
                  {
                     tire_color = block_09_inch_color;
                  }
                  break;

               case -8:
                  {
                     tire_color = block_08_inch_color;
                  }
                  break;

               case -7:
                  {
                     tire_color = block_07_inch_color;
                  }
                  break;

               case -6:
                  {
                     tire_color = block_06_inch_color;
                  }
                  break;

               case -5:
                  {
                     tire_color = block_05_inch_color;
                  }
                  break;

               case -4:
                  {
                     tire_color = block_04_inch_color;
                  }
                  break;

               case -3:
                  {
                     tire_color = block_03_inch_color;
                  }
                  break;

               case -2:
                  {
                     tire_color = block_02_inch_color;
                  }
                  break;

               case -1:
                  {
                     tire_color = block_01_inch_color;
                  }
                  break;

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

            tft.fillCircle(120, 79, 3, tire_color);  // hitch
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

            // erase the previous bubble
            tft.fillCircle(map(previous_x_roll, -5, +5, 200, 40), map(previous_y_pitch, -5, +5, 240, 80), 10, ILI9341_WHITE);

            if (pitch > 5.0)
            {
               y_pitch = 5.0;
            } else {
               if (pitch < -5.0)
               {
                  y_pitch = -5.0;
               } else {
                  y_pitch = pitch;
               }
            }

            if (roll > 5.0)
            {
               x_roll = 5.0;
            } else {
               if (roll < -5.0)
               {
                  x_roll = -5.0;
               } else {
                  x_roll = roll;
               }
            }

            // apply a small amount of smoothing (to reduce the amount of "jerky jumping" by the bubble)
            previous_x_roll = x_roll = ((previous_x_roll * 3.0f) + x_roll) / 4.0f;
            previous_y_pitch = y_pitch = ((previous_y_pitch * 3.0f) + y_pitch) / 4.0f;

            // draw the bubble
            tft.fillCircle(map(x_roll, -5, +5, 200, 40), map(y_pitch, -5, +5, 240, 80), 10, ILI9341_GREEN);
            tft.drawCircle(map(x_roll, -5, +5, 200, 40), map(y_pitch, -5, +5, 240, 80), 10, ILI9341_BLACK);

            // draw the cross-hairs
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
         draw_tire(89, 215, 10, 30, tire_color);  // lr inside tire
      }
      draw_tire(77, 215, 10, 30, tire_color);  // lr outside tire

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
         draw_tire(141, 215, 10, 30, tire_color);  // rr inside tire
      }
      draw_tire(153, 215, 10, 30, tire_color);  // rr outside tire

      center_draw_text("*", 3, tft.width() / 2, 230, ILI9341_BLUE, ILI9341_WHITE);

      delay(100);

      center_draw_text("*", 3, tft.width() / 2, 230, ILI9341_WHITE, ILI9341_WHITE);

      tft.setTextColor(ILI9341_BLACK, ILI9341_WHITE);

      tft.fillRect(76, 153, 88, 20, ILI9341_WHITE);

      tft.setTextSize(2);

      if (abs(pitch) < 10)
      {
         tft.setCursor(90, 157);
      } else {
         tft.setCursor(84, 157);
      }
      if (pitch >= 0.0)
      {
         tft.print("+");
      }
      tft.print(pitch);

      tft.fillRect(76, 188, 88, 20, ILI9341_WHITE);

      if (abs(roll) < 10)
      {
         tft.setCursor(90, 192);
      } else {
         tft.setCursor(84, 192);
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
               tft.setCursor(15, 100);
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

               tft.setCursor(185, 100);
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
               if (hitch_inch_blocks_needed < 0)
               {
                  tft.setTextColor(ILI9341_BLACK, ILI9341_WHITE);

                  tft.setTextSize(2);

                  tft.setCursor(5, 80);
                  tft.print("LOWER");

                  tft.setTextSize(3);

                  tft.setCursor(15, 100);

                  if (hitch_inch_blocks_needed >= (MAX_BLOCK_INCHES * -2))
                  {
                     if (abs(hitch_inch_blocks_needed) < 10)
                     {
                        tft.print("0");
                     }
                     tft.print((abs((int)hitch_inch_blocks_needed)));
                     tft.print("\"");
                  } else {
                     tft.print("-- ");
                  }
               } else {
                  tft.setTextColor(ILI9341_BLACK, ILI9341_WHITE);

                  tft.setTextSize(2);

                  tft.setCursor(5, 80);
                  tft.print("RAISE");

                  tft.setTextSize(3);

                  tft.setCursor(15, 100);

                  if (hitch_inch_blocks_needed <= (MAX_BLOCK_INCHES * 2))
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
            }
            break;

         case TRLH_BUBBLE_MODE:
            {
            }
            break;
      }

      tft.setCursor(15, 220);
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

      tft.setCursor(185, 220);
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


// draw a (colored) tire
void draw_tire(int x, int y, int x_size, int y_size, uint32_t color)
{
   tft.fillRect(x, y, x_size, y_size, color);  // tire
   tft.drawPixel(x, y, ILI9341_WHITE);
   tft.drawPixel(x + x_size - 1, y, ILI9341_WHITE);
   tft.drawPixel(x, y + y_size - 1, ILI9341_WHITE);
   tft.drawPixel(x + x_size - 1, y + y_size - 1, ILI9341_WHITE);
}  // draw_tire()


// initialize the screen for LED color mapper mode
void initialize_for_led_color_mapper(void)
{
   tft.setRotation(0);

   delay(100);

   ts.setRotation(2);

   draw_splash_screen();

   // initialize the LED library
   leds.begin();

   display_splash();

   tft.setRotation(1);

   delay(100);

   ts.setRotation(3);

   tft.fillScreen(ILI9341_BLACK);

   tft.setTextSize(1);
   tft.setTextColor(ILI9341_WHITE, ILI9341_GREEN);

   center_draw_text(TITLE_LED, 1, 115, 7, ILI9341_GREEN, ILI9341_BLACK);
   center_draw_text(AUTHOR, 1, 115, 17, ILI9341_YELLOW, ILI9341_BLACK);

   tft.setTextColor(ILI9341_WHITE, ILI9341_BLACK);

   tft.setCursor(18, 30);
   tft.print("+");
   tft.drawCircle(20, 33, 5, ILI9341_GREEN);

   tft.setCursor(58, 30);
   tft.print("+");
   tft.drawCircle(60, 33, 5, ILI9341_GREEN);

   tft.setCursor(98, 30);
   tft.print("+");
   tft.drawCircle(100, 33, 5, ILI9341_GREEN);


   tft.setCursor(18, 65);
   tft.print("-");
   tft.drawCircle(20, 68, 5, ILI9341_RED);

   tft.setCursor(58, 65);
   tft.print("-");
   tft.drawCircle(60, 68, 5, ILI9341_RED);

   tft.setCursor(98, 65);
   tft.print("-");
   tft.drawCircle(100, 68, 5, ILI9341_RED);

   update_leds_on_button();
   update_read_led_button();

   tft.fillRect(140, 35, 60, 30, display_color);
   tft.fillRect(130, 25, 80, 50, ILI9341_WHITE);
   tft.fillRect(135, 30, 70, 40, ILI9341_BLACK);

   for (pattern_number = NUMBER_OF_PATTERNS - 1; pattern_number >= 0; pattern_number--)
   {
      // try to read the settings from EEPROM for this index
      if (!read_settings())
      {
         // if the setting in EEPROM for this index are invalid, then save defaults to this index (as well as the default radio power level)

         radio_power_level = RADIO_POWER_LEVEL_HIGH;
         save_radio_power_level();

         red_level = 255;
         green_level = 255;
         blue_level = 255;

         read_led = false;

         for (int i = 0; i < PATTERN_LENGTH_MAX; i++)
         {
            led_list[i] = 0xFFFFFF;
         }

         switch (pattern_number)
         {
            case 0:
               {
                  pattern_type = PATTERN_TYPE_MARQUIS_FORWARD;
                  pattern_speed = 15;
                  pattern_length = 7;

                  led_list[0] = RGB_red;
                  led_list[1] = RGB_orange_red;
                  led_list[2] = RGB_yellow;
                  led_list[3] = RGB_dark_green;
                  led_list[4] = RGB_dark_blue;
                  led_list[5] = UV_PURPLE;
                  led_list[6] = RGB_white;
               }
               break;

            case 1:
               {
                  pattern_type = PATTERN_TYPE_RANDOM;
                  pattern_speed = 20;
                  pattern_length = 13;

                  led_list[0] = RGB_midnight_blue;
                  led_list[1] = RGB_dark_blue;
                  led_list[2] = RGB_navy;
                  led_list[3] = RGB_medium_blue;
                  led_list[4] = RGB_sea_green;
                  led_list[5] = RGB_teal;
                  led_list[6] = RGB_cadet_blue;
                  led_list[7] = RGB_blue;
                  led_list[8] = RGB_dark_cyan;
                  led_list[9] = RGB_corn_flower_blue;
                  led_list[10] = RGB_aqua_marine;
                  led_list[11] = RGB_aqua;
                  led_list[12] = RGB_light_sky_blue;
               }
               break;

            case 2:
               {
                  pattern_type = PATTERN_TYPE_RANDOM;
                  pattern_speed = 20;
                  pattern_length = 12;

                  led_list[0] = RGB_dark_green;
                  led_list[1] = RGB_dark_olive_green;
                  led_list[2] = RGB_green;
                  led_list[3] = RGB_forest_green;
                  led_list[4] = RGB_olive_drab;
                  led_list[5] = RGB_sea_green;
                  led_list[6] = RGB_medium_aqua_marine;
                  led_list[7] = RGB_lime_green;
                  led_list[8] = RGB_yellow_green;
                  led_list[9] = RGB_light_green;
                  led_list[10] = RGB_lawn_green;
                  led_list[11] = RGB_medium_aqua_marine;
               }
               break;

            case 3:
               {
                  pattern_type = PATTERN_TYPE_RANDOM;
                  pattern_speed = 20;
                  pattern_length = 7;

                  led_list[0] = RGB_dark_orange;
                  led_list[1] = RGB_dark_red;
                  led_list[2] = RGB_fire_brick;
                  led_list[3] = RGB_gold;
                  led_list[4] = RGB_orange;
                  led_list[5] = RGB_orange_red;
                  led_list[6] = RGB_red;
               }
               break;

            case 4:
               {
                  pattern_type = PATTERN_TYPE_MARQUIS_FORWARD;
                  pattern_speed = 15;
                  pattern_length = 3;

                  led_list[0] = RGB_red;
                  led_list[1] = RGB_silver;
                  led_list[2] = RGB_dark_blue;
               }
               break;

            case 5:
               {
                  pattern_type = PATTERN_TYPE_RANDOM;
                  pattern_speed = 20;
                  pattern_length = 7;

                  led_list[0] = RGB_brown;
                  led_list[1] = RGB_dark_golden_rod;
                  led_list[2] = RGB_saddle_brown;
                  led_list[3] = RGB_sienna;
                  led_list[4] = RGB_chocolate;
                  led_list[5] = RGB_olive_drab;
                  led_list[6] = RGB_rosy_brown;
               }
               break;

            case 6:
               {
                  pattern_type = PATTERN_TYPE_MARQUIS_FORWARD;
                  pattern_speed = 15;
                  pattern_length = 16;

                  led_list[0] = RGB_gold;
                  led_list[1] = RGB_gold;
                  led_list[2] = RGB_gold;
                  led_list[3] = RGB_gold;
                  led_list[4] = RGB_gold;
                  led_list[5] = RGB_gold;
                  led_list[6] = RGB_gold;
                  led_list[7] = RGB_gold;
                  led_list[8] = RGB_gold;
                  led_list[9] = RGB_gold;
                  led_list[10] = RGB_gold;
                  led_list[11] = RGB_gold;
                  led_list[12] = RGB_gold;
                  led_list[13] = RGB_gold;
                  led_list[14] = RGB_gold;
                  led_list[15] = RGB_gold;
               }
               break;

            case 7:
               {
                  pattern_type = PATTERN_TYPE_MARQUIS_FORWARD;
                  pattern_speed = 19;
                  pattern_length = 32;

                  led_list[0] = 0xFF00BF;
                  led_list[1] = 0xFF00BF;
                  led_list[2] = 0xFF00BF;
                  led_list[3] = 0xFF00BF;
                  led_list[4] = 0xFF00BF;
                  led_list[5] = 0xFF00BF;
                  led_list[6] = 0xFF00BF;
                  led_list[7] = 0xFF00BF;
                  led_list[8] = 0xFF00BF;
                  led_list[9] = 0xFF00BF;
                  led_list[10] = 0xFF00BF;
                  led_list[11] = 0xFF00BF;
                  led_list[12] = 0xFF00BF;
                  led_list[13] = 0xFF00BF;
                  led_list[14] = 0xFF00BF;
                  led_list[15] = 0xFF00BF;
                  led_list[16] = RGB_silver;
                  led_list[17] = RGB_silver;
                  led_list[18] = RGB_silver;
                  led_list[19] = RGB_silver;
                  led_list[20] = RGB_silver;
                  led_list[21] = RGB_silver;
                  led_list[22] = RGB_silver;
                  led_list[23] = RGB_silver;
                  led_list[24] = RGB_silver;
                  led_list[25] = RGB_silver;
                  led_list[26] = RGB_silver;
                  led_list[27] = RGB_silver;
                  led_list[28] = RGB_silver;
                  led_list[29] = RGB_silver;
                  led_list[30] = RGB_silver;
                  led_list[31] = RGB_silver;
               }
               break;

            case 9:
               {
                  pattern_type = PATTERN_TYPE_CROSSFADE_FORWARD;
                  pattern_speed = 20;
                  pattern_length = 7;

                  led_list[0] = 0xFF00BF;
                  led_list[1] = 0xFF00BF;
                  led_list[2] = 0xFF00BF;
                  led_list[3] = 0xFF00BF;
                  led_list[4] = RGB_silver;
                  led_list[5] = 0xFF00BF;
                  led_list[6] = RGB_silver;
                  led_list[7] = RGB_silver;
                  led_list[8] = RGB_silver;
                  led_list[9] = RGB_silver;
                  led_list[10] = RGB_silver;
                  led_list[11] = RGB_silver;
                  led_list[12] = RGB_silver;
                  led_list[13] = RGB_silver;
                  led_list[14] = RGB_silver;
                  led_list[15] = RGB_silver;
                  led_list[16] = RGB_silver;
                  led_list[17] = RGB_silver;
                  led_list[18] = RGB_silver;
                  led_list[19] = RGB_silver;
                  led_list[20] = RGB_silver;
                  led_list[21] = RGB_silver;
                  led_list[22] = RGB_silver;
                  led_list[23] = RGB_silver;
                  led_list[24] = RGB_silver;
                  led_list[25] = RGB_silver;
                  led_list[26] = RGB_silver;
                  led_list[27] = RGB_silver;
                  led_list[28] = RGB_silver;
                  led_list[29] = RGB_silver;
                  led_list[30] = RGB_silver;
                  led_list[31] = RGB_silver;
               }
               break;

            default:
               {
                  pattern_type = PATTERN_TYPE_MARQUIS_FORWARD;
                  pattern_speed = 15;
                  pattern_length = 32;

                  for (int i = 0; i < PATTERN_LENGTH_MAX; i++)
                  {
                     led_list[i] = RGB_silver;
                  }
               }
         }

         save_settings();
         read_settings();
      }
   }

   tft.setTextSize(1);
   tft.setTextColor(ILI9341_WHITE, ILI9341_BLACK);

   tft.setCursor(4, 85);
   tft.print("RADIO POWER    :               -  +");

   tft.drawCircle(192, 88, 5, ILI9341_RED);
   tft.drawCircle(210, 88, 5, ILI9341_GREEN);

   tft.setCursor(4, 100);
   tft.print("RADIO CHANNEL  :               -  +");

   tft.drawCircle(192, 103, 5, ILI9341_RED);
   tft.drawCircle(210, 103, 5, ILI9341_GREEN);

   tft.setCursor(4, 115);
   tft.print("BRIGHT LEVEL   :               -  +");

   tft.drawCircle(192, 118, 5, ILI9341_RED);
   tft.drawCircle(210, 118, 5, ILI9341_GREEN);

   tft.setCursor(4, 130);
   tft.print("PATTERN NUMBER :               -  +");

   tft.drawCircle(192, 133, 5, ILI9341_RED);
   tft.drawCircle(210, 133, 5, ILI9341_GREEN);

   tft.setCursor(4, 145);
   tft.print("PATTERN TYPE   :               -  +");

   tft.drawCircle(192, 148, 5, ILI9341_RED);
   tft.drawCircle(210, 148, 5, ILI9341_GREEN);

   tft.setCursor(4, 160);
   tft.print("PATTERN SPEED  :               -  +");

   tft.drawCircle(192, 163, 5, ILI9341_RED);
   tft.drawCircle(210, 163, 5, ILI9341_GREEN);

   tft.setCursor(4, 175);
   tft.print("PATTERN LENGTH :               -  +");

   tft.drawCircle(192, 178, 5, ILI9341_RED);
   tft.drawCircle(210, 178, 5, ILI9341_GREEN);

   tft.fillRect(225, 100, 90, 25, ILI9341_YELLOW);
   tft.drawRect(227, 102, 86, 21, ILI9341_BLACK);
   center_draw_text("LOAD PATTERN", 1, 270, 113, ILI9341_BLACK, ILI9341_YELLOW);

   tft.fillRect(225, 130, 90, 25, ILI9341_YELLOW);
   tft.drawRect(227, 132, 86, 21, ILI9341_BLACK);
   center_draw_text("SAVE PATTERN", 1, 270, 143, ILI9341_BLACK, ILI9341_YELLOW);

   tft.fillRect(225, 160, 90, 25, ILI9341_YELLOW);
   tft.drawRect(227, 162, 86, 21, ILI9341_BLACK);
   center_draw_text("APPLY PATTERN", 1, 270, 173, ILI9341_BLACK, ILI9341_YELLOW);

   // blank out the image storage for the LED array
   for (int i = 0; i < NUM_LEDS; i++)
   {
      LEDimage[i] = 0x000000;
   }

   if (!(radio_not_present))
   {
#ifdef DEBUG_RADIO_MODE
      Serial.println("");
      Serial.println("MODE = RADIO_MODE_INITIAL_SYNC...");
      Serial.println("");
#endif

      radio_mode = RADIO_MODE_INITIAL_SYNC;

      base_time = millis();

      read_radio_channel_number();

      update_radio_channel();

      read_radio_power_level();

      update_radio_power();

      // start by listening for PRIMARY sync msg for three seconds
      radio_setup(NOT_PRIMARY);
   } else {

#ifdef DEBUG_RADIO_MODE
      Serial.println("");
      Serial.println("MODE = RADIO_MODE_I_AM_PRIMARY...");
      Serial.println("");
#endif

      radio_mode = RADIO_MODE_I_AM_PRIMARY;

      update_radio_channel();
   }

   radio_status();

   // wait for a small bit before starting
   delay(100);

   // read from A1 twice to initialize the Random Number Generator
   randomSeed(analogRead(1) * analogRead(1));

   read_bright_level();

   update_bright_level();

   share_bright_level();

   read_pattern_number();

   apply_pattern();

   save_pattern_number();

   dynamic_bright_level = bright_level;

   fade_level = 0;
   xfade_level = 0;

   read_led = false;

   update_read_led_button();

   update_required_time = millis() + ((11 - pattern_speed) * UPDATE_REQUIRED_MILLIS);
}  // initialize_for_led_color_mapper()


// initialize the screen for RV leveling mode
void initialize_for_rv_leveling(void)
{
   boolean toggler = false;
   int max_tries = 10;
   boolean bma400_connected = true;

   tft.setRotation(0);

   delay(100);

   ts.setRotation(2);

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
   while ((accelerometer.beginI2C(i2cAddress) != BMA400_OK) && (--max_tries))
   {
      bma400_connected = false;

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

   tft.fillScreen(ILI9341_WHITE);

   if (bma400_connected)
   {
      center_draw_text("BMA400 connected !!", 2, tft.width() / 2, 140, ILI9341_BLACK, ILI9341_WHITE);

      Serial.println("BMA400 successfully connected !!");
   } else {
      Serial.println("BMA400 not connected !!");
   }

   delay(1000);

   // try to read the settings from EEPROM for this index
   if (!read_settings())
   {
      // if the setting in EEPROM for this index are invalid, then save defaults to this index
      front_to_rear_wheel_distance_in_inches = DEFAULT_FRONT_TO_REAR_MAX_WHEEL_DISTANCE_IN_INCHES;
      left_to_right_wheel_distance_in_inches = DEFAULT_LEFT_TO_RIGHT_MAX_WHEEL_DISTANCE_IN_INCHES;

      save_settings();
      read_settings();

      save_radio_channel_number();
   }

   read_radio_channel_number();

   if (!(radio_not_present))
   {
#ifdef DEBUG_RADIO_MODE
      Serial.println("");
      Serial.println("MODE = RADIO_MODE_INITIAL_SYNC...");
      Serial.println("");
#endif

      radio_mode = RADIO_MODE_INITIAL_SYNC;

      base_time = millis();

      read_radio_channel_number();

      update_radio_channel();

      read_radio_power_level();

      update_radio_power();

      radio_status();

      // start by listening for PRIMARY sync msg for three seconds
      radio_setup(NOT_PRIMARY);
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
}  // initialize_for_rv_leveling()


// update the radio status when in LED color mapper mode
void led_color_mapper_radio_status(void)
{
   tft.fillRect(100, 230, 120, 10, ILI9341_BLACK);

   switch (radio_mode)
   {
      case RADIO_MODE_INITIAL_SYNC:
      case RADIO_MODE_SYNC_UP:
         {
            center_draw_text(PAIRING, 1, tft.width() / 2, 235, ILI9341_WHITE, ILI9341_BLACK);
         }
         break;

      case RADIO_MODE_I_AM_PRIMARY:
         {
            if (!(radio_not_present))
            {
               center_draw_text(LED_INSIDE_UNIT, 1, tft.width() / 2, 235, ILI9341_WHITE, ILI9341_BLACK);
            } else {
               center_draw_text(LED_STANDALONE_UNIT, 1, tft.width() / 2, 235, ILI9341_WHITE, ILI9341_BLACK);
            }
         }
         break;

      case RADIO_MODE_I_AM_SECONDARY:
         {
            center_draw_text(LED_OUTSIDE_UNIT, 1, tft.width() / 2, 235, ILI9341_WHITE, ILI9341_BLACK);
         }
         break;
   }
}  // led_color_mapper_radio_status()


// loop forever
void loop(void)
{
   switch (ops_mode)
   {
      case OPS_MODE_TYPE_RV_LEVELING:
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
                  roll = atan(accelerometer.data.accelY / accelerometer.data.accelZ) * 180 / PI;

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
         }
         break;

      case OPS_MODE_TYPE_LED_COLOR_MAPPER:
         {
            if ((pattern_speed) &&
                  ((millis() - update_required_time) > UPDATE_REQUIRED_MILLIS) &&
                  ((radio_mode == RADIO_MODE_I_AM_PRIMARY) || (radio_mode == RADIO_MODE_SYNC_UP)))
            {
               if (++update_delay_counter >= (21 - pattern_speed))
               {
                  update_required = true;

                  update_delay_counter = 0;
               }

               update_required_time = millis();
            }

            // patterns
            if (update_required)
            {
               switch (pattern_type)
               {
                  case PATTERN_TYPE_MARQUIS_FORWARD:
                     {
                        marquis_index--;

                        if (marquis_index < 0)
                        {
                           marquis_index = pattern_length - 1;
                        }

                        for (int i = 0; i < NUM_LEDS; i++)
                        {
                           LEDimage[i] = led_list[(marquis_index + pattern_length + i) % pattern_length];
                        }
                     }
                     break;

                  case PATTERN_TYPE_MARQUIS_REVERSE:
                     {
                        marquis_index++;

                        if (marquis_index >= pattern_length)
                        {
                           marquis_index = 0;
                        }

                        for (int i = 0; i < NUM_LEDS; i++)
                        {
                           LEDimage[i] = led_list[(marquis_index + pattern_length + i) % pattern_length];
                        }
                     }
                     break;

                  case PATTERN_TYPE_CYCLE_FORWARD:
                     {
                        if (++cycle_index >= pattern_length)
                        {
                           cycle_index = 0;
                        }

                        for (int i = 0; i < NUM_LEDS; i++)
                        {
                           LEDimage[i] = led_list[cycle_index];
                        }
                     }
                     break;

                  case PATTERN_TYPE_CYCLE_REVERSE:
                     {
                        if (--cycle_index < 0)
                        {
                           cycle_index = pattern_length - 1;
                        }

                        for (int i = 0; i < NUM_LEDS; i++)
                        {
                           LEDimage[i] = led_list[cycle_index];
                        }
                     }
                     break;

                  case PATTERN_TYPE_FADE_FORWARD:
                     {
                        if (++fade_level <= (bright_level * 2))
                        {
                           if (fade_level <= bright_level)
                           {
                              dynamic_bright_level = fade_level;

                              for (int i = 0; i < NUM_LEDS; i++)
                              {
                                 LEDimage[i] = led_list[fade_index];
                              }
                           } else {
                              if (fade_level > bright_level)
                              {
                                 dynamic_bright_level = (bright_level * 2) - fade_level;

                                 for (int i = 0; i < NUM_LEDS; i++)
                                 {
                                    LEDimage[i] = led_list[fade_index];
                                 }
                              }
                           }
                        } else {
                           fade_level = 0;

                           if (++fade_index >= pattern_length)
                           {
                              fade_index = 0;
                           }
                        }
                     }
                     break;

                  case PATTERN_TYPE_FADE_REVERSE:
                     {
                        if (++fade_level <= (bright_level * 2))
                        {
                           if (fade_level <= bright_level)
                           {
                              dynamic_bright_level = fade_level;

                              for (int i = 0; i < NUM_LEDS; i++)
                              {
                                 LEDimage[i] = led_list[fade_index];
                              }
                           } else {
                              if (fade_level > bright_level)
                              {
                                 dynamic_bright_level = (bright_level * 2) - fade_level;

                                 for (int i = 0; i < NUM_LEDS; i++)
                                 {
                                    LEDimage[i] = led_list[fade_index];
                                 }
                              }
                           }
                        } else {
                           fade_level = 0;

                           if (--fade_index < 0)
                           {
                              fade_index = pattern_length - 1;
                           }
                        }
                     }
                     break;

                  case PATTERN_TYPE_CROSSFADE_FORWARD:
                     {
                        uint32_t xfade_color = 0;
                        static boolean xfade_toggle;

                        if (++xfade_level < 16)
                        {
                           if (pattern_length > 1)
                           {
                              xfade_color = ((led_list[xfade_index] / 0x10000) * (15 - xfade_level) / 15) * 0x10000;           // RED portion of the "from" color
                              xfade_color += (((led_list[xfade_index] % 0x10000) / 0x100) * (15 - xfade_level) / 15) * 0x100;  // BLUE portion of the "from" color
                              xfade_color += ((led_list[xfade_index] % 0x100) * (15 - xfade_level) / 15);                      // GREEN portion of the "from" color

                              xfade_color += ((led_list[(xfade_index + 1) % pattern_length] / 0x10000) * xfade_level / 15) * 0x10000;          // RED portion of the "to" color
                              xfade_color += (((led_list[(xfade_index + 1) % pattern_length] % 0x10000) / 0x100) * xfade_level / 15) * 0x100;  // BLUE portion of the "to" color
                              xfade_color += ((led_list[(xfade_index + 1) % pattern_length] % 0x100) * xfade_level / 15);                      // GREEN portion of the "to" color
                           } else {
                              if (xfade_toggle)
                              {
                                 xfade_color = ((led_list[xfade_index] / 0x10000) * xfade_level / 15) * 0x10000;           // RED portion of the "from" color
                                 xfade_color += (((led_list[xfade_index] % 0x10000) / 0x100) * xfade_level / 15) * 0x100;  // BLUE portion of the "from" color
                                 xfade_color += ((led_list[xfade_index] % 0x100) * xfade_level / 15);                      // GREEN portion of the "from" color
                              } else {
                                 xfade_color = ((led_list[xfade_index] / 0x10000) * (15 - xfade_level) / 15) * 0x10000;           // RED portion of the "from" color
                                 xfade_color += (((led_list[xfade_index] % 0x10000) / 0x100) * (15 - xfade_level) / 15) * 0x100;  // BLUE portion of the "from" color
                                 xfade_color += ((led_list[xfade_index] % 0x100) * (15 - xfade_level) / 15);                      // GREEN portion of the "from" color
                              }
                           }

                           for (int i = 0; i < NUM_LEDS; i++)
                           {
                              LEDimage[i] = xfade_color;
                           }
                        } else {
                           xfade_level = 0;
                           xfade_toggle = !xfade_toggle;

                           if (++xfade_index >= pattern_length)
                           {
                              xfade_index = 0;
                           }
                        }
                     }
                     break;

                  case PATTERN_TYPE_CROSSFADE_REVERSE:
                     {
                        uint32_t xfade_color = 0;
                        static boolean xfade_toggle;

                        if (++xfade_level < 16)
                        {
                           if (pattern_length > 1)
                           {
                              xfade_color = ((led_list[(xfade_index + 1) % pattern_length] / 0x10000) * (15 - xfade_level) / 15) * 0x10000;           // RED portion of the "from" color
                              xfade_color += (((led_list[(xfade_index + 1) % pattern_length] % 0x10000) / 0x100) * (15 - xfade_level) / 15) * 0x100;  // BLUE portion of the "from" color
                              xfade_color += ((led_list[(xfade_index + 1) % pattern_length] % 0x100) * (15 - xfade_level) / 15);                      // GREEN portion of the "from" color

                              xfade_color += ((led_list[xfade_index] / 0x10000) * xfade_level / 15) * 0x10000;          // RED portion of the "to" color
                              xfade_color += (((led_list[xfade_index] % 0x10000) / 0x100) * xfade_level / 15) * 0x100;  // BLUE portion of the "to" color
                              xfade_color += ((led_list[xfade_index] % 0x100) * xfade_level / 15);                      // GREEN portion of the "to" color
                           } else {
                              if (xfade_toggle)
                              {
                                 xfade_color = ((led_list[xfade_index] / 0x10000) * xfade_level / 15) * 0x10000;           // RED portion of the "from" color
                                 xfade_color += (((led_list[xfade_index] % 0x10000) / 0x100) * xfade_level / 15) * 0x100;  // BLUE portion of the "from" color
                                 xfade_color += ((led_list[xfade_index] % 0x100) * xfade_level / 15);                      // GREEN portion of the "from" color
                              } else {
                                 xfade_color = ((led_list[xfade_index] / 0x10000) * (15 - xfade_level) / 15) * 0x10000;           // RED portion of the "from" color
                                 xfade_color += (((led_list[xfade_index] % 0x10000) / 0x100) * (15 - xfade_level) / 15) * 0x100;  // BLUE portion of the "from" color
                                 xfade_color += ((led_list[xfade_index] % 0x100) * (15 - xfade_level) / 15);                      // GREEN portion of the "from" color
                              }
                           }

                           for (int i = 0; i < NUM_LEDS; i++)
                           {
                              LEDimage[i] = xfade_color;
                           }
                        } else {
                           xfade_level = 0;
                           xfade_toggle = !xfade_toggle;

                           if (--xfade_index < 0)
                           {
                              xfade_index = pattern_length - 1;
                           }
                        }
                     }
                     break;

                  case PATTERN_TYPE_ZIPPER_FORWARD:
                     {
                        zipper_index++;

                        if (zipper_index >= NUM_LEDS)
                        {
                           zipper_index = 0;

                           cycle_index++;

                           if (cycle_index >= pattern_length)
                           {
                              cycle_index = 0;
                           }
                        }

                        for (int i = 0; i < NUM_LEDS; i++)
                        {
                           if (i >= zipper_index)
                           {
                              LEDimage[i] = led_list[cycle_index];
                           } else {
                              LEDimage[i] = led_list[(cycle_index + 1) % pattern_length];
                           }
                        }
                     }
                     break;

                  case PATTERN_TYPE_ZIPPER_REVERSE:
                     {
                        zipper_index--;

                        if (zipper_index < 0)
                        {
                           zipper_index = NUM_LEDS - 1;

                           cycle_index--;

                           if (cycle_index < 0)
                           {
                              cycle_index = pattern_length - 1;
                           }
                        }

                        for (int i = 0; i < NUM_LEDS; i++)
                        {
                           if (i >= zipper_index)
                           {
                              LEDimage[i] = led_list[cycle_index];
                           } else {
                              LEDimage[i] = led_list[(cycle_index + 1) % pattern_length];
                           }
                        }
                     }
                     break;

                  case PATTERN_TYPE_RANDOM:
                     {
                        for (int i = 0; i < NUM_LEDS; i++)
                        {
                           LEDimage[i] = led_list[random(pattern_length)];
                        }
                     }
                     break;
               }
            }

            // send RGB LED command here
            if (update_required)
            {
               for (int i = 0; i < NUM_LEDS; i++)
               {
                  if (leds_on == true)
                  {
                     leds.setPixel(i, apply_intensity_to_color(LEDimage[i]));
                  } else {
                     leds.setPixel(i, 0x000000);
                  }
               }

               // make sure LEDs are not busy before updating
               while (leds.busy());

               leds.show();

               update_required = false;
            }
         }
         break;
   }

   if ((millis() - check_buttons_time) > CHECK_BUTTONS_MILLIS)
   {
      check_buttons_time = millis();

      process_buttons();
   }

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
   static int repeat_delay;

   // a point object holds x y and z coordinates.
   TS_Point p = ts.getPoint();

   // Scale from ~0->4000 to tft.width using the calibration #'s
   p.x = map(p.x, TS_MINX, TS_MAXX, 0, tft.width());
   p.y = map(p.y, TS_MINY, TS_MAXY, 0, tft.height());

#ifdef DEBUG_TS
   debug_touch(p);
#endif

   if (!(p.z))
   {
      repeat_delay = 0;

      button_press_type = BUTTON_PRESS_TYPE_NOT_PRESSED;

      return;
   } else {
      switch (ops_mode)
      {
         case OPS_MODE_TYPE_RV_LEVELING:
            {
               // long click on the battery status box
               if ((p.x >= 200) && (p.x <= 240) && (p.y >= 0) && (p.y <= 35))
               {
                  button_pressed = true;
                  wait_for_release = true;
               }

               // click on the - radio power_level
               if ((p.x >= 70) && (p.x <= 90) && (p.y >= 55) && (p.y <= 75))
               {
                  button_pressed = true;
                  wait_for_release = true;
               }

               // click on the + radio power_level
               if ((p.x >= 105) && (p.x <= 125) && (p.y >= 55) && (p.y <= 75))
               {
                  button_pressed = true;
                  wait_for_release = true;
               }

               // click on the - radio channel number
               if ((p.x >= 70) && (p.x <= 90) && (p.y >= 40) && (p.y <= 60))
               {
                  button_pressed = true;
               }

               // click on the + radio channel number
               if ((p.x >= 105) && (p.x <= 125) && (p.y >= 40) && (p.y <= 60))
               {
                  button_pressed = true;
               }

               // click on wheel base increase (+)
               if ((p.x >= 100) && (p.x <= 120) && (p.y >= 285) && (p.y <= 300))
               {
                  button_pressed = true;
               }

               // click on wheel base decrease (-)
               if ((p.x >= 100) && (p.x <= 120) && (p.y >= 305) && (p.y <= 320))
               {
                  button_pressed = true;
               }

               // click on axle width increase (+)
               if ((p.x >= 220) && (p.x <= 240) && (p.y >= 285) && (p.y <= 300))
               {
                  button_pressed = true;
               }

               // click on axle width decrease (-)
               if ((p.x >= 220) && (p.x <= 240) && (p.y >= 305) && (p.y <= 320))
               {
                  button_pressed = true;
               }

               // click on mode button
               if ((p.x >= 6) && (p.x <= 71) && (p.y >= 255) && (p.y <= 285))
               {
                  button_pressed = true;
                  wait_for_release = true;
               }
            }
            break;

         case OPS_MODE_TYPE_LED_COLOR_MAPPER:
            {
               // long click on the battery status box
               if ((p.x >= 250) && (p.x <= 290) && (p.y >= 0) && (p.y <= 35))
               {
                  button_pressed = true;
                  wait_for_release = true;
               }

               // click on the RED level
               if ((p.x >= 10) && (p.x <= 30) && (p.y >= 42) && (p.y <= 53))
               {
                  button_pressed = true;
                  wait_for_release = true;
               }

               // click on the RED increase
               if ((p.x >= 10) && (p.x <= 30) && (p.y >= 20) && (p.y <= 40))
               {
                  button_pressed = true;
               }

               // click on the RED decrease
               if ((p.x >= 10) && (p.x <= 30) && (p.y >= 55) && (p.y <= 80))
               {
                  button_pressed = true;
               }

               // click on the GREEN level
               if ((p.x >= 50) && (p.x <= 70) && (p.y >= 42) && (p.y <= 53))
               {
                  button_pressed = true;
                  wait_for_release = true;
               }

               // click on the GREEN increase
               if ((p.x >= 50) && (p.x <= 70) && (p.y >= 20) && (p.y <= 40))
               {
                  button_pressed = true;
               }

               // click on the GREEN decrease
               if ((p.x >= 50) && (p.x <= 70) && (p.y >= 55) && (p.y <= 80))
               {
                  button_pressed = true;
               }

               // click on the BLUE level
               if ((p.x >= 90) && (p.x <= 110) && (p.y >= 42) && (p.y <= 53))
               {
                  button_pressed = true;
                  wait_for_release = true;
               }

               // click on the BLUE increase
               if ((p.x >= 90) && (p.x <= 110) && (p.y >= 20) && (p.y <= 40))
               {
                  button_pressed = true;
               }

               // click on the BLUE decrease
               if ((p.x >= 90) && (p.x <= 110) && (p.y >= 55) && (p.y <= 80))
               {
                  button_pressed = true;
               }

               // click on the LEDs ON/OFF button
               if ((p.x >= 225) && (p.x <= 315) && (p.y >= 40) && (p.y <= 65))
               {
                  button_pressed = true;
                  wait_for_release = true;
               }

               // click on the read/set LED button
               if ((p.x >= 225) && (p.x <= 315) && (p.y >= 70) && (p.y <= 95))
               {
                  button_pressed = true;
                  wait_for_release = true;
               }


               // click on the - radio power level
               if ((p.x >= 185) && (p.x <= 195) && (p.y >= 85) && (p.y <= 95))
               {
                  if (!(radio_not_present))
                  {
                     button_pressed = true;
                     wait_for_release = true;
                  }
               }

               // click on the + radio power level
               if ((p.x >= 200) && (p.x <= 210) && (p.y >= 85) && (p.y <= 95))
               {
                  if (!(radio_not_present))
                  {
                     button_pressed = true;
                     wait_for_release = true;
                  }
               }


               // click on the - radio channel number
               if ((p.x >= 185) && (p.x <= 195) && (p.y >= 100) && (p.y <= 110))
               {
                  if (!(radio_not_present))
                  {
                     button_pressed = true;
                  }
               }

               // click on the + radio channel number
               if ((p.x >= 200) && (p.x <= 210) && (p.y >= 100) && (p.y <= 110))
               {
                  if (!(radio_not_present))
                  {
                     button_pressed = true;
                  }
               }


               // click on the - pattern bright
               if ((p.x >= 185) && (p.x <= 195) && (p.y >= 115) && (p.y <= 125))
               {
                  button_pressed = true;
                  wait_for_release = true;
               }

               // click on the + pattern bright
               if ((p.x >= 200) && (p.x <= 210) && (p.y >= 115) && (p.y <= 125))
               {
                  button_pressed = true;
                  wait_for_release = true;
               }


               // click on the - pattern number
               if ((p.x >= 185) && (p.x <= 195) && (p.y >= 130) && (p.y <= 140))
               {
                  button_pressed = true;
                  wait_for_release = true;
               }

               // click on the + pattern number
               if ((p.x >= 200) && (p.x <= 210) && (p.y >= 130) && (p.y <= 140))
               {
                  button_pressed = true;
                  wait_for_release = true;
               }


               // click on the - pattern type
               if ((p.x >= 185) && (p.x <= 195) && (p.y >= 145) && (p.y <= 155))
               {
                  button_pressed = true;
                  wait_for_release = true;
               }

               // click on the + pattern type
               if ((p.x >= 200) && (p.x <= 210) && (p.y >= 145) && (p.y <= 155))
               {
                  button_pressed = true;
                  wait_for_release = true;
               }


               // click on the - pattern speed
               if ((p.x >= 185) && (p.x <= 195) && (p.y >= 160) && (p.y <= 170))
               {
                  button_pressed = true;
                  wait_for_release = true;
               }

               // click on the + pattern speed
               if ((p.x >= 200) && (p.x <= 210) && (p.y >= 160) && (p.y <= 170))
               {
                  button_pressed = true;
                  wait_for_release = true;
               }


               // click on the - pattern length
               if ((p.x >= 185) && (p.x <= 195) && (p.y >= 175) && (p.y <= 185))
               {
                  button_pressed = true;
                  wait_for_release = true;
               }
               // MJC
               // click on the + pattern length
               if ((p.x >= 200) && (p.x <= 210) && (p.y >= 175) && (p.y <= 185))
               {
                  button_pressed = true;
                  wait_for_release = true;
               }

               // click on the LOAD PATTERN button
               if ((p.x >= 225) && (p.x <= 315) && (p.y >= 100) && (p.y <= 125))
               {
                  button_pressed = true;
                  wait_for_release = true;
               }

               // click on the SAVE PATTERN button
               if ((p.x >= 225) && (p.x <= 315) && (p.y >= 130) && (p.y <= 155))
               {
                  button_pressed = true;
                  wait_for_release = true;
               }

               // click on the APPLY PATTERN button
               if ((p.x >= 225) && (p.x <= 315) && (p.y >= 160) && (p.y <= 185))
               {
                  button_pressed = true;
                  wait_for_release = true;
               }

               // click on the first row of LEDs
               if ((p.y >= 197) && (p.y <= 212))
               {
                  button_pressed = true;
                  wait_for_release = true;
               }

               // click on the second row of LEDs
               if ((p.y >= 217) && (p.y <= 232))
               {
                  button_pressed = true;
                  wait_for_release = true;
               }
            }
            break;
      }
   }

   if (button_pressed)
   {
      boolean debounce = true;

      if (wait_for_release)
      {
         while ((wait_for_release) && (debounce))
         {
#ifdef DEBUG_TS
            debug_touch(p);
#endif

            while (ts.touched())
            {
               delay(CHECK_BUTTONS_MILLIS);

               repeat_delay++;

               if (repeat_delay == 1)
               {
                  button_press_type = BUTTON_PRESS_TYPE_SHORT_PRESS;
               } else {
                  if (repeat_delay >= 50)
                  {
                     repeat_delay = 50;

                     button_press_type = BUTTON_PRESS_TYPE_LONG_PRESS;
                  } else {
                     if (repeat_delay > 20)
                     {
                        button_press_type = BUTTON_PRESS_TYPE_MEDIUM_PRESS;
                     } else {
                        button_press_type = BUTTON_PRESS_TYPE_NOT_PRESSED;
                     }
                  }
               }

#ifdef DEBUG_TS
               debug_touch(p);
#endif

               // if currently not being touched, then we're done
               if (!ts.touched())
               {
                  debounce = false;
               }
            }
         }
      } else {
         repeat_delay++;
      }

      if (repeat_delay == 1)
      {
         button_press_type = BUTTON_PRESS_TYPE_SHORT_PRESS;
      } else {
         if (repeat_delay >= 50)
         {
            repeat_delay = 50;

            button_press_type = BUTTON_PRESS_TYPE_LONG_PRESS;
         } else {
            if (repeat_delay > 20)
            {
               button_press_type = BUTTON_PRESS_TYPE_MEDIUM_PRESS;
            } else {
               button_press_type = BUTTON_PRESS_TYPE_NOT_PRESSED;
            }
         }
      }

#ifdef DEBUG_TS
      debug_touch(p);
#endif

      process_button_inputs(p);
   }
}  // process_buttons()


// act on button presses
void process_button_inputs(TS_Point p)
{
   switch (ops_mode)
   {
      case OPS_MODE_TYPE_RV_LEVELING:
         {
            // long click on the battery status box
            if ((p.x >= 200) && (p.x <= 240) && (p.y >= 0) && (p.y <= 35))
            {
               if (button_press_type == BUTTON_PRESS_TYPE_LONG_PRESS)
               {
                  switch (ops_mode)
                  {
                     case OPS_MODE_TYPE_RV_LEVELING:
                        {
                           ops_mode = OPS_MODE_TYPE_LED_COLOR_MAPPER;
                        }
                        break;

                     case OPS_MODE_TYPE_LED_COLOR_MAPPER:
                        {
                           ops_mode = OPS_MODE_TYPE_RV_LEVELING;
                        }
                        break;
                  }

                  change_operating_mode();
               }
            }

            // click on the - radio power level
            if ((p.x >= 70) && (p.x <= 90) && (p.y >= 55) && (p.y <= 75))
            {
               if (!(radio_not_present))
               {
                  switch (radio_power_level)
                  {
                     case RADIO_POWER_LEVEL_LOW:
                        {
                           radio_power_level = RADIO_POWER_LEVEL_HIGH;

                           save_radio_power_level();

                           update_radio_power();

                           if (radio_mode == RADIO_MODE_I_AM_PRIMARY)
                           {
                              radio_setup(IS_PRIMARY);
                           } else {
                              if (radio_mode == RADIO_MODE_I_AM_SECONDARY)
                              {
                                 radio_setup(NOT_PRIMARY);
                              }
                           }
                        }
                        break;

                     case RADIO_POWER_LEVEL_MID:
                        {
                           radio_power_level = RADIO_POWER_LEVEL_LOW;

                           save_radio_power_level();

                           update_radio_power();

                           if (radio_mode == RADIO_MODE_I_AM_PRIMARY)
                           {
                              radio_setup(IS_PRIMARY);
                           } else {
                              if (radio_mode == RADIO_MODE_I_AM_SECONDARY)
                              {
                                 radio_setup(NOT_PRIMARY);
                              }
                           }
                        }
                        break;

                     case RADIO_POWER_LEVEL_HIGH:
                        {
                           radio_power_level = RADIO_POWER_LEVEL_MID;

                           save_radio_power_level();

                           update_radio_power();

                           if (radio_mode == RADIO_MODE_I_AM_PRIMARY)
                           {
                              radio_setup(IS_PRIMARY);
                           } else {
                              if (radio_mode == RADIO_MODE_I_AM_SECONDARY)
                              {
                                 radio_setup(NOT_PRIMARY);
                              }
                           }
                        }
                        break;
                  }
               }
            }

            // click on the + radio power level
            if ((p.x >= 105) && (p.x <= 125) && (p.y >= 55) && (p.y <= 75))
            {
               if (!(radio_not_present))
               {
                  switch (radio_power_level)
                  {
                     case RADIO_POWER_LEVEL_LOW:
                        {
                           radio_power_level = RADIO_POWER_LEVEL_MID;

                           save_radio_power_level();

                           update_radio_power();

                           if (radio_mode == RADIO_MODE_I_AM_PRIMARY)
                           {
                              radio_setup(IS_PRIMARY);
                           } else {
                              if (radio_mode == RADIO_MODE_I_AM_SECONDARY)
                              {
                                 radio_setup(NOT_PRIMARY);
                              }
                           }
                        }
                        break;

                     case RADIO_POWER_LEVEL_MID:
                        {
                           radio_power_level = RADIO_POWER_LEVEL_HIGH;

                           save_radio_power_level();

                           update_radio_power();

                           if (radio_mode == RADIO_MODE_I_AM_PRIMARY)
                           {
                              radio_setup(IS_PRIMARY);
                           } else {
                              if (radio_mode == RADIO_MODE_I_AM_SECONDARY)
                              {
                                 radio_setup(NOT_PRIMARY);
                              }
                           }
                        }
                        break;

                     case RADIO_POWER_LEVEL_HIGH:
                        {
                           radio_power_level = RADIO_POWER_LEVEL_LOW;

                           save_radio_power_level();

                           update_radio_power();

                           if (radio_mode == RADIO_MODE_I_AM_PRIMARY)
                           {
                              radio_setup(IS_PRIMARY);
                           } else {
                              if (radio_mode == RADIO_MODE_I_AM_SECONDARY)
                              {
                                 radio_setup(NOT_PRIMARY);
                              }
                           }
                        }
                        break;
                  }
               }
            }

            // click on the - radio channel number
            if ((p.x >= 70) && (p.x <= 90) && (p.y >= 40) && (p.y <= 60))
            {
               if (!(radio_not_present))
               {
                  if (button_press_type == BUTTON_PRESS_TYPE_LONG_PRESS)
                  {
                     if (radio_channel > (RADIO_CHANNEL_MIN + 5))
                     {
                        radio_channel = ((radio_channel / 5) * 5) - 5;

                        update_radio_channel();

                        save_radio_channel_number();

#ifdef DEBUG_RADIO_MODE
                        Serial.println("");
                        Serial.println("MODE = RADIO_MODE_INITIAL_SYNC...");
                        Serial.println("");
#endif

                        radio_mode = RADIO_MODE_INITIAL_SYNC;

                        base_time = millis();

                        radio_status();

                        radio_setup(NOT_PRIMARY);
                     } else {
                        if (radio_channel > RADIO_CHANNEL_MIN)
                        {
                           radio_channel--;

                           update_radio_channel();

                           save_radio_channel_number();

#ifdef DEBUG_RADIO_MODE
                           Serial.println("");
                           Serial.println("MODE = RADIO_MODE_INITIAL_SYNC...");
                           Serial.println("");
#endif

                           radio_mode = RADIO_MODE_INITIAL_SYNC;

                           base_time = millis();

                           radio_status();

                           radio_setup(NOT_PRIMARY);
                        }
                     }
                  } else {
                     if ((button_press_type == BUTTON_PRESS_TYPE_SHORT_PRESS) || (button_press_type == BUTTON_PRESS_TYPE_MEDIUM_PRESS))
                     {
                        if (radio_channel > RADIO_CHANNEL_MIN)
                        {
                           radio_channel--;

                           update_radio_channel();

                           save_radio_channel_number();

#ifdef DEBUG_RADIO_MODE
                           Serial.println("");
                           Serial.println("MODE = RADIO_MODE_INITIAL_SYNC...");
                           Serial.println("");
#endif

                           radio_mode = RADIO_MODE_INITIAL_SYNC;

                           base_time = millis();

                           radio_status();

                           radio_setup(NOT_PRIMARY);
                        }
                     }
                  }
               }
            }

            // click on the + radio channel number
            if ((p.x >= 105) && (p.x <= 125) && (p.y >= 40) && (p.y <= 60))
            {
               if (!(radio_not_present))
               {
                  if (button_press_type == BUTTON_PRESS_TYPE_LONG_PRESS)
                  {
                     if (radio_channel < (RADIO_CHANNEL_MAX - 5))
                     {
                        radio_channel = ((radio_channel / 5) * 5) + 5;

                        update_radio_channel();

                        save_radio_channel_number();

#ifdef DEBUG_RADIO_MODE
                        Serial.println("");
                        Serial.println("MODE = RADIO_MODE_INITIAL_SYNC...");
                        Serial.println("");
#endif

                        radio_mode = RADIO_MODE_INITIAL_SYNC;

                        base_time = millis();

                        radio_status();

                        radio_setup(NOT_PRIMARY);
                     } else {
                        if (radio_channel < RADIO_CHANNEL_MAX)
                        {
                           radio_channel++;

                           update_radio_channel();

                           save_radio_channel_number();

#ifdef DEBUG_RADIO_MODE
                           Serial.println("");
                           Serial.println("MODE = RADIO_MODE_INITIAL_SYNC...");
                           Serial.println("");
#endif

                           radio_mode = RADIO_MODE_INITIAL_SYNC;

                           base_time = millis();

                           radio_status();

                           radio_setup(NOT_PRIMARY);
                        }
                     }
                  } else {
                     if ((button_press_type == BUTTON_PRESS_TYPE_SHORT_PRESS) || (button_press_type == BUTTON_PRESS_TYPE_MEDIUM_PRESS))
                     {
                        if (radio_channel < RADIO_CHANNEL_MAX)
                        {
                           radio_channel++;

                           update_radio_channel();

                           save_radio_channel_number();

#ifdef DEBUG_RADIO_MODE
                           Serial.println("");
                           Serial.println("MODE = RADIO_MODE_INITIAL_SYNC...");
                           Serial.println("");
#endif

                           radio_mode = RADIO_MODE_INITIAL_SYNC;

                           base_time = millis();

                           radio_status();

                           radio_setup(NOT_PRIMARY);
                        }
                     }
                  }
               }
            }


            // click on wheel base increase (+)
            if ((trlh_mode != TRLH_BUBBLE_MODE) && (p.x >= 100) && (p.x <= 120) && (p.y >= 285) && (p.y <= 300))
            {
               if (button_press_type == BUTTON_PRESS_TYPE_LONG_PRESS)
               {
                  if (front_to_rear_wheel_distance_in_inches < (FRONT_TO_REAR_MAX_WHEEL_DISTANCE_IN_INCHES - 5))
                  {
                     front_to_rear_wheel_distance_in_inches = ((front_to_rear_wheel_distance_in_inches / 5) * 5) + 5;
                  } else {
                     if (front_to_rear_wheel_distance_in_inches < FRONT_TO_REAR_MAX_WHEEL_DISTANCE_IN_INCHES)
                     {
                        if (front_to_rear_wheel_distance_in_inches < FRONT_TO_REAR_MAX_WHEEL_DISTANCE_IN_INCHES)
                        {
                           front_to_rear_wheel_distance_in_inches++;
                        }
                     }
                  }
               } else {
                  if ((button_press_type == BUTTON_PRESS_TYPE_SHORT_PRESS) || (button_press_type == BUTTON_PRESS_TYPE_MEDIUM_PRESS))
                  {
                     if (front_to_rear_wheel_distance_in_inches < FRONT_TO_REAR_MAX_WHEEL_DISTANCE_IN_INCHES)
                     {
                        front_to_rear_wheel_distance_in_inches++;
                     }
                  }
               }

               draw_axle_info();

               save_settings_needed = true;
            }

            // click on wheel base decrease (-)
            if ((trlh_mode != TRLH_BUBBLE_MODE) && (p.x >= 100) && (p.x <= 120) && (p.y >= 305) && (p.y <= 320))
            {
               if (button_press_type == BUTTON_PRESS_TYPE_LONG_PRESS)
               {
                  if (front_to_rear_wheel_distance_in_inches > (FRONT_TO_REAR_MIN_WHEEL_DISTANCE_IN_INCHES + 5))
                  {
                     front_to_rear_wheel_distance_in_inches = ((front_to_rear_wheel_distance_in_inches / 5) * 5) - 5;
                  } else {
                     if (front_to_rear_wheel_distance_in_inches > FRONT_TO_REAR_MIN_WHEEL_DISTANCE_IN_INCHES)
                     {
                        front_to_rear_wheel_distance_in_inches--;
                     }
                  }
               } else {
                  if ((button_press_type == BUTTON_PRESS_TYPE_SHORT_PRESS) || (button_press_type == BUTTON_PRESS_TYPE_MEDIUM_PRESS))
                  {
                     if (front_to_rear_wheel_distance_in_inches > FRONT_TO_REAR_MIN_WHEEL_DISTANCE_IN_INCHES)
                     {
                        if (front_to_rear_wheel_distance_in_inches > FRONT_TO_REAR_MIN_WHEEL_DISTANCE_IN_INCHES)
                        {
                           front_to_rear_wheel_distance_in_inches--;
                        }
                     }
                  }
               }

               draw_axle_info();

               save_settings_needed = true;
            }

            // click on axle width increase (+)
            if ((trlh_mode != TRLH_BUBBLE_MODE) && (p.x >= 220) && (p.x <= 240) && (p.y >= 285) && (p.y <= 300))
            {
               if (button_press_type == BUTTON_PRESS_TYPE_LONG_PRESS)
               {
                  if (left_to_right_wheel_distance_in_inches < (LEFT_TO_RIGHT_MAX_WHEEL_DISTANCE_IN_INCHES - 5))
                  {
                     left_to_right_wheel_distance_in_inches = ((left_to_right_wheel_distance_in_inches / 5) * 5) + 5;
                  } else {
                     if (left_to_right_wheel_distance_in_inches < LEFT_TO_RIGHT_MAX_WHEEL_DISTANCE_IN_INCHES)
                     {
                        left_to_right_wheel_distance_in_inches++;
                     }
                  }
               } else {
                  if ((button_press_type == BUTTON_PRESS_TYPE_SHORT_PRESS) || (button_press_type == BUTTON_PRESS_TYPE_MEDIUM_PRESS))
                  {
                     if (left_to_right_wheel_distance_in_inches < LEFT_TO_RIGHT_MAX_WHEEL_DISTANCE_IN_INCHES)
                     {
                        left_to_right_wheel_distance_in_inches++;
                     }
                  }
               }

               draw_axle_info();

               save_settings_needed = true;
            }

            // click on axle width decrease (-)
            if ((trlh_mode != TRLH_BUBBLE_MODE) && (p.x >= 220) && (p.x <= 240) && (p.y >= 305) && (p.y <= 320))
            {
               if (button_press_type == BUTTON_PRESS_TYPE_LONG_PRESS)
               {
                  if (left_to_right_wheel_distance_in_inches > (LEFT_TO_RIGHT_MIN_WHEEL_DISTANCE_IN_INCHES + 5))
                  {
                     left_to_right_wheel_distance_in_inches = ((left_to_right_wheel_distance_in_inches / 5) * 5) - 5;
                  } else {
                     if (left_to_right_wheel_distance_in_inches > LEFT_TO_RIGHT_MIN_WHEEL_DISTANCE_IN_INCHES)
                     {
                        left_to_right_wheel_distance_in_inches--;
                     }
                  }
               } else {
                  if ((button_press_type == BUTTON_PRESS_TYPE_SHORT_PRESS) || (button_press_type == BUTTON_PRESS_TYPE_MEDIUM_PRESS))
                  {
                     if (left_to_right_wheel_distance_in_inches > LEFT_TO_RIGHT_MIN_WHEEL_DISTANCE_IN_INCHES)
                     {
                        left_to_right_wheel_distance_in_inches--;
                     }
                  }
               }

               draw_axle_info();

               save_settings_needed = true;
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
         }
         break;

      case OPS_MODE_TYPE_LED_COLOR_MAPPER:
         {
            // long click on the battery status box
            if ((p.x >= 251) && (p.x <= 290) && (p.y >= 0) && (p.y <= 35))
            {
               if (button_press_type == BUTTON_PRESS_TYPE_LONG_PRESS)
               {
                  switch (ops_mode)
                  {
                     case OPS_MODE_TYPE_RV_LEVELING:
                        {
                           ops_mode = OPS_MODE_TYPE_LED_COLOR_MAPPER;
                        }
                        break;

                     case OPS_MODE_TYPE_LED_COLOR_MAPPER:
                        {
                           ops_mode = OPS_MODE_TYPE_RV_LEVELING;
                        }
                        break;
                  }

                  change_operating_mode();
               }
            }

            // click on the RED level
            if ((p.x >= 10) && (p.x <= 30) && (p.y >= 42) && (p.y <= 53))
            {
               if ((red_level >= 0) && (red_level < 63))
               {
                  red_level = 63;
               } else {
                  if ((red_level >= 63) && (red_level < 127))
                  {
                     red_level = 127;
                  } else {
                     if ((red_level >= 127) && (red_level < 191))
                     {
                        red_level = 191;
                     } else {
                        if ((red_level >= 191) && (red_level < 255))
                        {
                           red_level = 255;
                        } else {
                           red_level = 0;
                        }
                     }
                  }
               }

               update_value(red_level, 20, 50, ILI9341_RED);

               share_red_value();

               read_led = false;

               update_read_led_button();
            }

            // click on the RED increase
            if ((p.x >= 10) && (p.x <= 30) && (p.y >= 20) && (p.y <= 40))
            {
               if (button_press_type == BUTTON_PRESS_TYPE_LONG_PRESS)
               {
                  if (red_level < 255)
                  {
                     red_level = ((red_level / 5) * 5) + 5;
                  }
               } else {
                  if ((button_press_type == BUTTON_PRESS_TYPE_SHORT_PRESS) || (button_press_type == BUTTON_PRESS_TYPE_MEDIUM_PRESS))
                  {
                     if (red_level < 255)
                     {
                        red_level++;
                     }
                  }
               }

               update_value(red_level, 20, 50, ILI9341_RED);

               share_red_value();

               read_led = false;

               update_read_led_button();
            }

            // click on the RED decrease
            if ((p.x >= 10) && (p.x <= 30) && (p.y >= 55) && (p.y <= 80))
            {
               if (button_press_type == BUTTON_PRESS_TYPE_LONG_PRESS)
               {
                  if (red_level >= 5)
                  {
                     red_level = ((red_level / 5) * 5) - 5;
                  }
               } else {
                  if ((button_press_type == BUTTON_PRESS_TYPE_SHORT_PRESS) || (button_press_type == BUTTON_PRESS_TYPE_MEDIUM_PRESS))
                  {
                     if (red_level > 0)
                     {
                        red_level--;
                     }
                  }
               }

               update_value(red_level, 20, 50, ILI9341_RED);

               share_red_value();

               read_led = false;

               update_read_led_button();
            }

            // click on the GREEN level
            if ((p.x >= 50) && (p.x <= 70) && (p.y >= 42) && (p.y <= 53))
            {

               if ((green_level >= 0) && (green_level < 63))
               {
                  green_level = 63;
               } else {
                  if ((green_level >= 63) && (green_level < 127))
                  {
                     green_level = 127;
                  } else {
                     if ((green_level >= 127) && (green_level < 191))
                     {
                        green_level = 191;
                     } else {
                        if ((green_level >= 191) && (green_level < 255))
                        {
                           green_level = 255;
                        } else {
                           green_level = 0;
                        }
                     }
                  }
               }

               update_value(green_level, 60, 50, ILI9341_GREEN);

               share_green_value();

               read_led = false;

               update_read_led_button();
            }

            // click on the GREEN increase
            if ((p.x >= 50) && (p.x <= 70) && (p.y >= 20) && (p.y <= 40))
            {
               if (button_press_type == BUTTON_PRESS_TYPE_LONG_PRESS)
               {
                  if (green_level < 255)
                  {
                     green_level = ((green_level / 5) * 5) + 5;
                  }
               } else {
                  if ((button_press_type == BUTTON_PRESS_TYPE_SHORT_PRESS) || (button_press_type == BUTTON_PRESS_TYPE_MEDIUM_PRESS))
                  {
                     if (green_level < 255)
                     {
                        green_level++;
                     }
                  }
               }

               update_value(green_level, 60, 50, ILI9341_GREEN);

               share_green_value();

               read_led = false;

               update_read_led_button();
            }

            // click on the GREEN decrease
            if ((p.x >= 50) && (p.x <= 70) && (p.y >= 55) && (p.y <= 80))
            {
               if (button_press_type == BUTTON_PRESS_TYPE_LONG_PRESS)
               {
                  if (green_level >= 5)
                  {
                     green_level = ((green_level / 5) * 5) - 5;
                  }
               } else {
                  if ((button_press_type == BUTTON_PRESS_TYPE_SHORT_PRESS) || (button_press_type == BUTTON_PRESS_TYPE_MEDIUM_PRESS))
                  {
                     if (green_level > 0)
                     {
                        green_level--;
                     }
                  }
               }

               update_value(green_level, 60, 50, ILI9341_GREEN);

               share_green_value();

               read_led = false;

               update_read_led_button();
            }

            // click on the BLUE level
            if ((p.x >= 90) && (p.x <= 110) && (p.y >= 42) && (p.y <= 53))
            {

               if ((blue_level >= 0) && (blue_level < 63))
               {
                  blue_level = 63;
               } else {
                  if ((blue_level >= 63) && (blue_level < 127))
                  {
                     blue_level = 127;
                  } else {
                     if ((blue_level >= 127) && (blue_level < 191))
                     {
                        blue_level = 191;
                     } else {
                        if ((blue_level >= 191) && (blue_level < 255))
                        {
                           blue_level = 255;
                        } else {
                           blue_level = 0;
                        }
                     }
                  }
               }

               update_value(blue_level, 100, 50, ILI9341_BLUE);

               share_blue_value();

               read_led = false;

               update_read_led_button();
            }

            // click on the BLUE increase
            if ((p.x >= 90) && (p.x <= 110) && (p.y >= 20) && (p.y <= 40))
            {
               if (button_press_type == BUTTON_PRESS_TYPE_LONG_PRESS)
               {
                  if (blue_level < 255)
                  {
                     blue_level = ((blue_level / 5) * 5) + 5;
                  }
               } else {
                  if ((button_press_type == BUTTON_PRESS_TYPE_SHORT_PRESS) || (button_press_type == BUTTON_PRESS_TYPE_MEDIUM_PRESS))
                  {
                     if (blue_level < 255)
                     {
                        blue_level++;
                     }
                  }
               }

               update_value(blue_level, 100, 50, ILI9341_BLUE);

               share_blue_value();

               read_led = false;

               update_read_led_button();
            }

            // click on the BLUE decrease
            if ((p.x >= 90) && (p.x <= 110) && (p.y >= 55) && (p.y <= 75))
            {
               if (button_press_type == BUTTON_PRESS_TYPE_LONG_PRESS)
               {
                  if (blue_level >= 5)
                  {
                     blue_level = ((blue_level / 5) * 5) - 5;
                  }
               } else {
                  if ((button_press_type == BUTTON_PRESS_TYPE_SHORT_PRESS) || (button_press_type == BUTTON_PRESS_TYPE_MEDIUM_PRESS))
                  {
                     if (blue_level > 0)
                     {
                        blue_level--;
                     }
                  }
               }

               update_value(blue_level, 100, 50, ILI9341_BLUE);

               share_blue_value();

               read_led = false;

               update_read_led_button();
            }

            // click on the LEDs ON/OFF button
            if ((p.x >= 225) && (p.x <= 315) && (p.y >= 40) && (p.y <= 65))
            {
               leds_on = !leds_on;

               update_leds_on_button();

               share_leds_on_button();

               read_led = false;

               update_read_led_button();
            }

            // click on the read/set LED button
            if ((p.x >= 225) && (p.x <= 315) && (p.y >= 70) && (p.y <= 95))
            {
               read_led = !read_led;

               update_read_led_button();
            }

            // click on the - radio power level
            if ((p.x >= 185) && (p.x <= 195) && (p.y >= 90) && (p.y <= 100))
            {
               if (!(radio_not_present))
               {
                  switch (radio_power_level)
                  {
                     case RADIO_POWER_LEVEL_LOW:
                        {
                           radio_power_level = RADIO_POWER_LEVEL_HIGH;

                           save_radio_power_level();

                           update_radio_power();

                           if (radio_mode == RADIO_MODE_I_AM_PRIMARY)
                           {
                              radio_setup(IS_PRIMARY);
                           } else {
                              if (radio_mode == RADIO_MODE_I_AM_SECONDARY)
                              {
                                 radio_setup(NOT_PRIMARY);
                              }
                           }

                           read_led = false;

                           update_read_led_button();
                        }
                        break;

                     case RADIO_POWER_LEVEL_MID:
                        {
                           radio_power_level = RADIO_POWER_LEVEL_LOW;

                           save_radio_power_level();

                           update_radio_power();

                           if (radio_mode == RADIO_MODE_I_AM_PRIMARY)
                           {
                              radio_setup(IS_PRIMARY);
                           } else {
                              if (radio_mode == RADIO_MODE_I_AM_SECONDARY)
                              {
                                 radio_setup(NOT_PRIMARY);
                              }
                           }

                           read_led = false;

                           update_read_led_button();
                        }
                        break;

                     case RADIO_POWER_LEVEL_HIGH:
                        {
                           radio_power_level = RADIO_POWER_LEVEL_MID;

                           save_radio_power_level();

                           update_radio_power();

                           if (radio_mode == RADIO_MODE_I_AM_PRIMARY)
                           {
                              radio_setup(IS_PRIMARY);
                           } else {
                              if (radio_mode == RADIO_MODE_I_AM_SECONDARY)
                              {
                                 radio_setup(NOT_PRIMARY);
                              }
                           }

                           read_led = false;

                           update_read_led_button();
                        }
                        break;
                  }
               }
            }

            // click on the + radio power level
            if ((p.x >= 200) && (p.x <= 210) && (p.y >= 90) && (p.y <= 100))
            {
               if (!(radio_not_present))
               {
                  switch (radio_power_level)
                  {
                     case RADIO_POWER_LEVEL_LOW:
                        {
                           radio_power_level = RADIO_POWER_LEVEL_MID;

                           save_radio_power_level();

                           update_radio_power();

                           if (radio_mode == RADIO_MODE_I_AM_PRIMARY)
                           {
                              radio_setup(IS_PRIMARY);
                           } else {
                              if (radio_mode == RADIO_MODE_I_AM_SECONDARY)
                              {
                                 radio_setup(NOT_PRIMARY);
                              }
                           }

                           read_led = false;

                           update_read_led_button();
                        }
                        break;

                     case RADIO_POWER_LEVEL_MID:
                        {
                           radio_power_level = RADIO_POWER_LEVEL_HIGH;

                           save_radio_power_level();

                           update_radio_power();

                           if (radio_mode == RADIO_MODE_I_AM_PRIMARY)
                           {
                              radio_setup(IS_PRIMARY);
                           } else {
                              if (radio_mode == RADIO_MODE_I_AM_SECONDARY)
                              {
                                 radio_setup(NOT_PRIMARY);
                              }
                           }

                           read_led = false;

                           update_read_led_button();
                        }
                        break;

                     case RADIO_POWER_LEVEL_HIGH:
                        {
                           radio_power_level = RADIO_POWER_LEVEL_LOW;

                           save_radio_power_level();

                           update_radio_power();

                           if (radio_mode == RADIO_MODE_I_AM_PRIMARY)
                           {
                              radio_setup(IS_PRIMARY);
                           } else {
                              if (radio_mode == RADIO_MODE_I_AM_SECONDARY)
                              {
                                 radio_setup(NOT_PRIMARY);
                              }
                           }

                           read_led = false;

                           update_read_led_button();
                        }
                        break;
                  }
               }
            }

            // click on the - radio channel number
            if ((p.x >= 185) && (p.x <= 195) && (p.y >= 105) && (p.y <= 115))
            {
               if (!(radio_not_present))
               {
                  if (button_press_type == BUTTON_PRESS_TYPE_LONG_PRESS)
                  {
                     if (radio_channel > (RADIO_CHANNEL_MIN + 5))
                     {
                        radio_channel = ((radio_channel / 5) * 5) - 5;

                        save_radio_channel_number();

                        update_radio_channel();

#ifdef DEBUG_RADIO_MODE
                        Serial.println("");
                        Serial.println("MODE = RADIO_MODE_INITIAL_SYNC...");
                        Serial.println("");
#endif

                        radio_mode = RADIO_MODE_INITIAL_SYNC;

                        base_time = millis();

                        radio_status();

                        radio_setup(NOT_PRIMARY);

                        read_led = false;

                        update_read_led_button();
                     } else {
                        if (radio_channel > RADIO_CHANNEL_MIN)
                        {
                           radio_channel--;

                           save_radio_channel_number();

                           update_radio_channel();

#ifdef DEBUG_RADIO_MODE
                           Serial.println("");
                           Serial.println("MODE = RADIO_MODE_INITIAL_SYNC...");
                           Serial.println("");
#endif

                           radio_mode = RADIO_MODE_INITIAL_SYNC;

                           base_time = millis();

                           radio_status();

                           radio_setup(NOT_PRIMARY);

                           read_led = false;

                           update_read_led_button();
                        }
                     }
                  } else {
                     if ((button_press_type == BUTTON_PRESS_TYPE_SHORT_PRESS) || (button_press_type == BUTTON_PRESS_TYPE_MEDIUM_PRESS))
                     {
                        if (radio_channel > RADIO_CHANNEL_MIN)
                        {
                           radio_channel--;

                           save_radio_channel_number();

                           update_radio_channel();

#ifdef DEBUG_RADIO_MODE
                           Serial.println("");
                           Serial.println("MODE = RADIO_MODE_INITIAL_SYNC...");
                           Serial.println("");
#endif

                           radio_mode = RADIO_MODE_INITIAL_SYNC;

                           base_time = millis();

                           radio_status();

                           radio_setup(NOT_PRIMARY);

                           read_led = false;

                           update_read_led_button();
                        }
                     }
                  }
               }
            }

            // click on the + radio channel number
            if ((p.x >= 200) && (p.x <= 210) && (p.y >= 105) && (p.y <= 115))
            {
               if (!(radio_not_present))
               {
                  if (button_press_type == BUTTON_PRESS_TYPE_LONG_PRESS)
                  {
                     if (radio_channel < (RADIO_CHANNEL_MAX - 5))
                     {
                        radio_channel = ((radio_channel / 5) * 5) + 5;

                        save_radio_channel_number();

                        update_radio_channel();

#ifdef DEBUG_RADIO_MODE
                        Serial.println("");
                        Serial.println("MODE = RADIO_MODE_INITIAL_SYNC...");
                        Serial.println("");
#endif

                        radio_mode = RADIO_MODE_INITIAL_SYNC;

                        base_time = millis();

                        radio_status();

                        radio_setup(NOT_PRIMARY);

                        read_led = false;

                        update_read_led_button();
                     } else {
                        if (radio_channel < RADIO_CHANNEL_MAX)
                        {
                           radio_channel++;

                           save_radio_channel_number();

                           update_radio_channel();

#ifdef DEBUG_RADIO_MODE
                           Serial.println("");
                           Serial.println("MODE = RADIO_MODE_INITIAL_SYNC...");
                           Serial.println("");
#endif

                           radio_mode = RADIO_MODE_INITIAL_SYNC;

                           base_time = millis();

                           radio_status();

                           radio_setup(NOT_PRIMARY);

                           read_led = false;

                           update_read_led_button();
                        }
                     }
                  } else {
                     if ((button_press_type == BUTTON_PRESS_TYPE_SHORT_PRESS) || (button_press_type == BUTTON_PRESS_TYPE_MEDIUM_PRESS))
                     {
                        if (radio_channel < RADIO_CHANNEL_MAX)
                        {
                           radio_channel++;

                           save_radio_channel_number();

                           update_radio_channel();

#ifdef DEBUG_RADIO_MODE
                           Serial.println("");
                           Serial.println("MODE = RADIO_MODE_INITIAL_SYNC...");
                           Serial.println("");
#endif

                           radio_mode = RADIO_MODE_INITIAL_SYNC;

                           base_time = millis();

                           radio_status();

                           radio_setup(NOT_PRIMARY);

                           read_led = false;

                           update_read_led_button();
                        }
                     }
                  }
               }
            }


            // click on the - pattern number
            if ((p.x >= 185) && (p.x <= 195) && (p.y >= 135) && (p.y <= 145))
            {
               if (pattern_number > 0)
               {
                  pattern_number--;

                  update_pattern_number();

                  dynamic_bright_level = bright_level;

                  fade_level = 0;
                  xfade_level = 0;
               }

               read_led = false;

               update_read_led_button();
            }

            // click on the + pattern number
            if ((p.x >= 200) && (p.x <= 210) && (p.y >= 135) && (p.y <= 145))
            {
               if (pattern_number < (NUMBER_OF_PATTERNS - 1))
               {
                  pattern_number++;

                  update_pattern_number();

                  dynamic_bright_level = bright_level;

                  fade_level = 0;
                  xfade_level = 0;
               }

               read_led = false;

               update_read_led_button();
            }


            // click on the - brightness level
            if ((p.x >= 185) && (p.x <= 195) && (p.y >= 120) && (p.y <= 130))
            {
               if (bright_level > 1)
               {
                  bright_level--;
                  dynamic_bright_level = bright_level;

                  fade_level = 0;
                  xfade_level = 0;

                  update_bright_level();

                  save_bright_level();

                  share_bright_level();

                  update_required = true;
               }

               read_led = false;

               update_read_led_button();
            }

            // click on the + brightness level
            if ((p.x >= 200) && (p.x <= 210) && (p.y >= 120) && (p.y <= 130))
            {
               if (bright_level < 8)
               {
                  bright_level++;
                  dynamic_bright_level = bright_level;

                  fade_level = 0;
                  xfade_level = 0;

                  update_bright_level();

                  save_bright_level();

                  share_bright_level();

                  update_required = true;
               }

               read_led = false;

               update_read_led_button();
            }


            // click on the - pattern type
            if ((p.x >= 185) && (p.x <= 195) && (p.y >= 150) && (p.y <= 160))
            {
               switch (pattern_type)
               {
                  case PATTERN_TYPE_MARQUIS_FORWARD:
                     {
                        pattern_type = PATTERN_TYPE_RANDOM;
                     }
                     break;

                  case PATTERN_TYPE_MARQUIS_REVERSE:
                     {
                        pattern_type = PATTERN_TYPE_MARQUIS_FORWARD;
                     }
                     break;

                  case PATTERN_TYPE_CYCLE_FORWARD:
                     {
                        pattern_type = PATTERN_TYPE_MARQUIS_REVERSE;
                     }
                     break;

                  case PATTERN_TYPE_CYCLE_REVERSE:
                     {
                        pattern_type = PATTERN_TYPE_CYCLE_FORWARD;
                     }
                     break;

                  case PATTERN_TYPE_FADE_FORWARD:
                     {
                        pattern_type = PATTERN_TYPE_CYCLE_REVERSE;
                     }
                     break;

                  case PATTERN_TYPE_FADE_REVERSE:
                     {
                        pattern_type = PATTERN_TYPE_FADE_FORWARD;
                     }
                     break;

                  case PATTERN_TYPE_CROSSFADE_FORWARD:
                     {
                        pattern_type = PATTERN_TYPE_FADE_REVERSE;
                     }
                     break;

                  case PATTERN_TYPE_CROSSFADE_REVERSE:
                     {
                        pattern_type = PATTERN_TYPE_CROSSFADE_FORWARD;
                     }
                     break;

                  case PATTERN_TYPE_ZIPPER_FORWARD:
                     {
                        pattern_type = PATTERN_TYPE_CROSSFADE_REVERSE;
                     }
                     break;

                  case PATTERN_TYPE_ZIPPER_REVERSE:
                     {
                        pattern_type = PATTERN_TYPE_ZIPPER_FORWARD;
                     }
                     break;

                  case PATTERN_TYPE_RANDOM:
                     {
                        pattern_type = PATTERN_TYPE_ZIPPER_REVERSE;
                     }
                     break;
               }

               update_pattern_type();

               dynamic_bright_level = bright_level;

               fade_level = 0;
               xfade_level = 0;

               read_led = false;

               update_read_led_button();
            }

            // click on the + pattern type
            if ((p.x >= 200) && (p.x <= 210) && (p.y >= 150) && (p.y <= 160))
            {
               switch (pattern_type)
               {
                  case PATTERN_TYPE_MARQUIS_FORWARD:
                     {
                        pattern_type = PATTERN_TYPE_MARQUIS_REVERSE;
                     }
                     break;

                  case PATTERN_TYPE_MARQUIS_REVERSE:
                     {
                        pattern_type = PATTERN_TYPE_CYCLE_FORWARD;
                     }
                     break;

                  case PATTERN_TYPE_CYCLE_FORWARD:
                     {
                        pattern_type = PATTERN_TYPE_CYCLE_REVERSE;
                     }
                     break;

                  case PATTERN_TYPE_CYCLE_REVERSE:
                     {
                        pattern_type = PATTERN_TYPE_FADE_FORWARD;
                     }
                     break;

                  case PATTERN_TYPE_FADE_FORWARD:
                     {
                        pattern_type = PATTERN_TYPE_FADE_REVERSE;
                     }
                     break;

                  case PATTERN_TYPE_FADE_REVERSE:
                     {
                        pattern_type = PATTERN_TYPE_CROSSFADE_FORWARD;
                     }
                     break;

                  case PATTERN_TYPE_CROSSFADE_FORWARD:
                     {
                        pattern_type = PATTERN_TYPE_CROSSFADE_REVERSE;
                     }
                     break;

                  case PATTERN_TYPE_CROSSFADE_REVERSE:
                     {
                        pattern_type = PATTERN_TYPE_ZIPPER_FORWARD;
                     }
                     break;

                  case PATTERN_TYPE_ZIPPER_FORWARD:
                     {
                        pattern_type = PATTERN_TYPE_ZIPPER_REVERSE;
                     }
                     break;

                  case PATTERN_TYPE_ZIPPER_REVERSE:
                     {
                        pattern_type = PATTERN_TYPE_RANDOM;
                     }
                     break;

                  case PATTERN_TYPE_RANDOM:
                     {
                        pattern_type = PATTERN_TYPE_MARQUIS_FORWARD;
                     }
                     break;
               }

               update_pattern_type();

               dynamic_bright_level = bright_level;

               fade_level = 0;
               xfade_level = 0;

               read_led = false;

               update_read_led_button();
            }


            // click on the - pattern speed
            if ((p.x >= 185) && (p.x <= 195) && (p.y >= 165) && (p.y <= 175))
            {
               update_required = true;

               if (pattern_speed > 0)
               {
                  pattern_speed--;

                  update_pattern_speed();
               }

               read_led = false;

               update_read_led_button();
            }

            // click on the + pattern speed
            if ((p.x >= 200) && (p.x <= 210) && (p.y >= 165) && (p.y <= 175))
            {
               if (pattern_speed < 20)
               {
                  pattern_speed++;

                  update_pattern_speed();
               }

               read_led = false;

               update_read_led_button();
            }


            // click on the - pattern length
            if ((p.x >= 185) && (p.x <= 195) && (p.y >= 180) && (p.y <= 190))
            {
               if (pattern_length > 1)
               {
                  pattern_length--;

                  update_pattern_length();

                  update_leds();
               }

               read_led = false;

               update_read_led_button();
            }

            // click on the + pattern length
            if ((p.x >= 200) && (p.x <= 210) && (p.y >= 180) && (p.y <= 190))
            {
               if (pattern_length < PATTERN_LENGTH_MAX)
               {
                  pattern_length++;

                  update_pattern_length();

                  update_leds();
               }

               read_led = false;

               update_read_led_button();
            }

            // click on the LOAD PATTERN button
            if ((p.x >= 225) && (p.x <= 315) && (p.y >= 100) && (p.y <= 125))
            {
               apply_pattern();

               save_pattern_number();

               dynamic_bright_level = bright_level;

               fade_level = 0;
               xfade_level = 0;

               read_led = false;

               update_read_led_button();
            }

            // click on the SAVE PATTERN button
            if ((p.x >= 225) && (p.x <= 315) && (p.y >= 130) && (p.y <= 155))
            {
               save_pattern_number();

               save_settings();

               apply_pattern();

               share_pattern_number();
               share_pattern_type();
               share_pattern_speed();
               share_pattern_length();

               share_leds();

               share_red_value();
               share_blue_value();
               share_green_value();

               radio_send("SP");

               dynamic_bright_level = bright_level;

               fade_level = 0;
               xfade_level = 0;

               read_led = false;

               update_read_led_button();
            }

            // click on the APPLY PATTERN button
            if ((p.x >= 225) && (p.x <= 315) && (p.y >= 160) && (p.y <= 185))
            {
               dynamic_bright_level = bright_level;
               share_bright_level();

               share_pattern_number();
               share_pattern_type();
               share_pattern_speed();
               share_pattern_length();

               share_leds();

               share_red_value();
               share_blue_value();
               share_green_value();

               fade_level = 0;
               xfade_level = 0;
            }

            // click on the first row of LEDs
            if ((p.y >= 197) && (p.y <= 212))
            {
               for (int i = 0; i < 16; i++)
               {
                  if ((p.x >= (1 + (i * 20))) && (p.x <= (16 + (i * 20))))
                  {
                     if (read_led)
                     {
                        if (i < pattern_length)
                        {
                           red_level = led_list[i] / 0x10000;
                           green_level = (led_list[i] / 0x100) % 0x100;
                           blue_level = led_list[i] % 0x100;
                           ;

                           update_value(red_level, 20, 50, ILI9341_RED);
                           update_value(green_level, 60, 50, ILI9341_GREEN);
                           update_value(blue_level, 100, 50, ILI9341_BLUE);

                           update_settings();
                        }
                     } else {
                        led_list[i] = led_strip_color;

                        if (i >= pattern_length)
                        {
                           pattern_length = i + 1;

                           update_pattern_length();

                           share_pattern_length();
                        }

                        update_leds();
                     }
                  }
               }

               read_led = false;

               update_read_led_button();
            }

            // click on the second row of LEDs
            if ((p.y >= 217) && (p.y <= 232))
            {
               for (int i = 0; i < 16; i++)
               {
                  if ((p.x >= (1 + (i * 20))) && (p.x <= (16 + (i * 20))))
                  {
                     if (read_led)
                     {
                        if ((i + (PATTERN_LENGTH_MAX / 2)) < pattern_length)
                        {
                           red_level = led_list[i + (PATTERN_LENGTH_MAX / 2)] / 0x10000;
                           green_level = (led_list[i + (PATTERN_LENGTH_MAX / 2)] / 0x100) % 0x100;
                           blue_level = led_list[i + (PATTERN_LENGTH_MAX / 2)] % 0x100;
                           ;

                           update_value(red_level, 20, 50, ILI9341_RED);
                           update_value(green_level, 60, 50, ILI9341_GREEN);
                           update_value(blue_level, 100, 50, ILI9341_BLUE);

                           update_settings();
                        }
                     } else {
                        led_list[i + (PATTERN_LENGTH_MAX / 2)] = led_strip_color;

                        if ((i + (PATTERN_LENGTH_MAX / 2)) >= pattern_length)
                        {
                           pattern_length = i + (PATTERN_LENGTH_MAX / 2) + 1;

                           update_pattern_length();

                           share_pattern_length();
                        }

                        update_leds();
                     }
                  }
               }

               read_led = false;

               update_read_led_button();
            }
         }
         break;
   }
}  // process_button_inputs()


// process radio operations
void process_radio(void)
{
   switch (ops_mode)
   {
      case OPS_MODE_TYPE_RV_LEVELING:
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
                                 ((radio_rx_buffer[1] == '-') || (radio_rx_buffer[1] == '+')) && ((radio_rx_buffer[2] >= '0') && (radio_rx_buffer[2] <= '9')) && ((radio_rx_buffer[3] >= '0') && (radio_rx_buffer[3] <= '9')) && (radio_rx_buffer[4] == '.') && ((radio_rx_buffer[5] >= '0') && (radio_rx_buffer[5] <= '9')) && ((radio_rx_buffer[6] >= '0') && (radio_rx_buffer[6] <= '9')) && ((radio_rx_buffer[7] >= '0') && (radio_rx_buffer[7] <= '9')) &&

                                 // roll = xx.xxx
                                 ((radio_rx_buffer[8] == '-') || (radio_rx_buffer[8] == '+')) && ((radio_rx_buffer[9] >= '0') && (radio_rx_buffer[9] <= '9')) && ((radio_rx_buffer[10] >= '0') && (radio_rx_buffer[10] <= '9')) && (radio_rx_buffer[11] == '.') && ((radio_rx_buffer[12] >= '0') && (radio_rx_buffer[12] <= '9')) && ((radio_rx_buffer[13] >= '0') && (radio_rx_buffer[13] <= '9')) && ((radio_rx_buffer[14] >= '0') && (radio_rx_buffer[14] <= '9')))
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
                              ((radio_rx_buffer[1] == '-') || (radio_rx_buffer[1] == '+')) && ((radio_rx_buffer[2] >= '0') && (radio_rx_buffer[2] <= '9')) && ((radio_rx_buffer[3] >= '0') && (radio_rx_buffer[3] <= '9')) && (radio_rx_buffer[4] == '.') && ((radio_rx_buffer[5] >= '0') && (radio_rx_buffer[5] <= '9')) && ((radio_rx_buffer[6] >= '0') && (radio_rx_buffer[6] <= '9')) && ((radio_rx_buffer[7] >= '0') && (radio_rx_buffer[7] <= '9')) &&

                              // roll = xx.xxx
                              ((radio_rx_buffer[8] == '-') || (radio_rx_buffer[8] == '+')) && ((radio_rx_buffer[9] >= '0') && (radio_rx_buffer[9] <= '9')) && ((radio_rx_buffer[10] >= '0') && (radio_rx_buffer[10] <= '9')) && (radio_rx_buffer[11] == '.') && ((radio_rx_buffer[12] >= '0') && (radio_rx_buffer[12] <= '9')) && ((radio_rx_buffer[13] >= '0') && (radio_rx_buffer[13] <= '9')) && ((radio_rx_buffer[14] >= '0') && (radio_rx_buffer[14] <= '9')))
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
         }
         break;

      case OPS_MODE_TYPE_LED_COLOR_MAPPER:
         {
            switch (radio_mode)
            {
               case RADIO_MODE_INITIAL_SYNC:
                  {
                     if ((millis() - base_time) >= INITIAL_LISTEN_FOR_PRIMARY_SYNC_IN_MILLIS)
                     {
                        radio_mode = RADIO_MODE_SYNC_UP;

                        radio_status();

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

                              radio_status();

#ifdef DEBUG_RADIO_MODE
                              Serial.println("");
                              Serial.println("MODE = RADIO_MODE_I_AM_SECONDARY...");
                              Serial.println("");
#endif

                              base_time = millis();

                              radio_send(MSG_I_AM_SECONDARY);

                              radio_status();
                           }
                        }
                     }
                  }
                  break;

               case RADIO_MODE_SYNC_UP:
               case RADIO_MODE_I_AM_PRIMARY:
               case RADIO_MODE_I_AM_SECONDARY:
                  {
                     if (radio_receive())
                     {
                        if (strncmp(radio_rx_buffer, MSG_I_AM_PRIMARY, strlen(MSG_I_AM_PRIMARY)) == 0)
                        {
                           if (radio_mode != RADIO_MODE_I_AM_SECONDARY)
                           {
                              radio_mode = RADIO_MODE_I_AM_SECONDARY;

                              radio_status();

#ifdef DEBUG_RADIO_MODE
                              Serial.println("");
                              Serial.println("MODE = RADIO_MODE_I_AM_SECONDARY...");
                              Serial.println("");
#endif

                              radio_setup(NOT_PRIMARY);
                           }

                           radio_send(MSG_I_AM_SECONDARY);
                        }

                        if (strncmp(radio_rx_buffer, MSG_I_AM_SECONDARY, strlen(MSG_I_AM_SECONDARY)) == 0)
                        {
                           if (radio_mode != RADIO_MODE_I_AM_PRIMARY)
                           {
                              radio_mode = RADIO_MODE_I_AM_PRIMARY;

                              radio_status();

                              base_time = millis();

#ifdef DEBUG_RADIO_MODE
                              Serial.println("");
                              Serial.println("MODE = RADIO_MODE_I_AM_PRIMARY...");
                              Serial.println("");
#endif

                              radio_setup(IS_PRIMARY);
                           }
                        }

                        if ((radio_rx_buffer[0] == 'S') && (radio_rx_buffer[1] == 'P'))
                        {
                           save_pattern_number();

                           save_settings();

                           save_bright_level();

                           apply_pattern();

                           read_led = false;

                           update_read_led_button();
                        }

                        if ((radio_rx_buffer[0] == 'R') && ((radio_rx_buffer[1] >= '0') && (radio_rx_buffer[1] <= '9')) && ((radio_rx_buffer[2] >= '0') && (radio_rx_buffer[2] <= '9')) && ((radio_rx_buffer[3] >= '0') && (radio_rx_buffer[3] <= '9')))
                        {
                           red_level = (radio_rx_buffer[1] - '0') * 100;
                           red_level += (radio_rx_buffer[2] - '0') * 10;
                           red_level += radio_rx_buffer[3] - '0';

                           update_value(red_level, 20, 50, ILI9341_RED);

                           base_time = millis();
                        }

                        if ((radio_rx_buffer[0] == 'G') && ((radio_rx_buffer[1] >= '0') && (radio_rx_buffer[1] <= '9')) && ((radio_rx_buffer[2] >= '0') && (radio_rx_buffer[2] <= '9')) && ((radio_rx_buffer[3] >= '0') && (radio_rx_buffer[3] <= '9')))
                        {
                           green_level = (radio_rx_buffer[1] - '0') * 100;
                           green_level += (radio_rx_buffer[2] - '0') * 10;
                           green_level += radio_rx_buffer[3] - '0';

                           update_value(green_level, 60, 50, ILI9341_GREEN);

                           base_time = millis();
                        }

                        if ((radio_rx_buffer[0] == 'B') && ((radio_rx_buffer[1] >= '0') && (radio_rx_buffer[1] <= '9')) && ((radio_rx_buffer[2] >= '0') && (radio_rx_buffer[2] <= '9')) && ((radio_rx_buffer[3] >= '0') && (radio_rx_buffer[3] <= '9')))
                        {
                           blue_level = (radio_rx_buffer[1] - '0') * 100;
                           blue_level += (radio_rx_buffer[2] - '0') * 10;
                           blue_level += radio_rx_buffer[3] - '0';

                           update_value(blue_level, 100, 50, ILI9341_BLUE);

                           base_time = millis();
                        }

                        if ((radio_rx_buffer[0] == 'L') && (radio_rx_buffer[1] == 'O') && ((radio_rx_buffer[2] >= '0') && (radio_rx_buffer[2] <= '1')))
                        {
                           leds_on = radio_rx_buffer[2] - '0';

                           update_leds_on_button();

                           base_time = millis();
                        }

                        if ((radio_rx_buffer[0] == 'L') && ((radio_rx_buffer[1] >= '0') && (radio_rx_buffer[1] <= '9')) && ((radio_rx_buffer[2] >= '0') && (radio_rx_buffer[2] <= '9')) && (radio_rx_buffer[3] == '0') && (radio_rx_buffer[4] == 'x') && (((radio_rx_buffer[5] >= '0') && (radio_rx_buffer[5] <= '9')) || ((radio_rx_buffer[5] >= 'A') && (radio_rx_buffer[5] <= 'F'))) && (((radio_rx_buffer[6] >= '0') && (radio_rx_buffer[6] <= '9')) || ((radio_rx_buffer[6] >= 'A') && (radio_rx_buffer[6] <= 'F'))) && (((radio_rx_buffer[7] >= '0') && (radio_rx_buffer[7] <= '9')) || ((radio_rx_buffer[7] >= 'A') && (radio_rx_buffer[7] <= 'F'))) && (((radio_rx_buffer[8] >= '0') && (radio_rx_buffer[8] <= '9')) || ((radio_rx_buffer[8] >= 'A') && (radio_rx_buffer[8] <= 'F'))) && (((radio_rx_buffer[9] >= '0') && (radio_rx_buffer[9] <= '9')) || ((radio_rx_buffer[9] >= 'A') && (radio_rx_buffer[9] <= 'F'))) && (((radio_rx_buffer[10] >= '0') && (radio_rx_buffer[10] <= '9')) || ((radio_rx_buffer[10] >= 'A') && (radio_rx_buffer[10] <= 'F'))))
                        {
                           int lednum = ((radio_rx_buffer[1] - '0') * 10) + (radio_rx_buffer[2] - '0') - 1;

                           if ((radio_rx_buffer[5] >= '0') && (radio_rx_buffer[5] <= '9'))
                           {
                              led_list[lednum] = (radio_rx_buffer[5] - '0') * 0x100000;
                           } else {
                              led_list[lednum] = ((radio_rx_buffer[5] - 'A') + 10) * 0x100000;
                           }

                           if ((radio_rx_buffer[6] >= '0') && (radio_rx_buffer[6] <= '9'))
                           {
                              led_list[lednum] += (radio_rx_buffer[6] - '0') * 0x10000;
                           } else {
                              led_list[lednum] += ((radio_rx_buffer[6] - 'A') + 10) * 0x10000;
                           }

                           if ((radio_rx_buffer[7] >= '0') && (radio_rx_buffer[7] <= '9'))
                           {
                              led_list[lednum] += (radio_rx_buffer[7] - '0') * 0x1000;
                           } else {
                              led_list[lednum] += ((radio_rx_buffer[7] - 'A') + 10) * 0x1000;
                           }

                           if ((radio_rx_buffer[8] >= '0') && (radio_rx_buffer[8] <= '9'))
                           {
                              led_list[lednum] += (radio_rx_buffer[8] - '0') * 0x100;
                           } else {
                              led_list[lednum] += ((radio_rx_buffer[8] - 'A') + 10) * 0x100;
                           }

                           if ((radio_rx_buffer[9] >= '0') && (radio_rx_buffer[9] <= '9'))
                           {
                              led_list[lednum] += (radio_rx_buffer[9] - '0') * 0x10;
                           } else {
                              led_list[lednum] += ((radio_rx_buffer[9] - 'A') + 10) * 0x10;
                           }

                           if ((radio_rx_buffer[10] >= '0') && (radio_rx_buffer[10] <= '9'))
                           {
                              led_list[lednum] += (radio_rx_buffer[10] - '0');
                           } else {
                              led_list[lednum] += ((radio_rx_buffer[10] - 'A') + 10);
                           }

                           update_leds();

                           base_time = millis();
                        }

                        if ((radio_rx_buffer[0] == 'P') && (radio_rx_buffer[1] == 'N') && ((radio_rx_buffer[2] >= '0') && (radio_rx_buffer[2] <= '9')) && ((radio_rx_buffer[3] >= '0') && (radio_rx_buffer[3] <= '9')))
                        {
                           pattern_number = ((radio_rx_buffer[2] - '0') * 10) + (radio_rx_buffer[3] - '0') - 1;

                           update_pattern_number();

                           save_pattern_number();

                           base_time = millis();
                        }

                        if ((radio_rx_buffer[0] == 'B') && (radio_rx_buffer[1] == 'L') && ((radio_rx_buffer[2] >= '0') && (radio_rx_buffer[2] <= '9')) && ((radio_rx_buffer[3] >= '0') && (radio_rx_buffer[3] <= '9')))
                        {
                           bright_level = (((radio_rx_buffer[2] - '0') * 10) + (radio_rx_buffer[3] - '0'));
                           dynamic_bright_level = bright_level;

                           update_bright_level();

                           save_bright_level();

                           base_time = millis();
                        }

                        if ((radio_rx_buffer[0] == 'P') && (radio_rx_buffer[1] == 'T') && ((radio_rx_buffer[2] >= '0') && (radio_rx_buffer[2] <= '9')) && ((radio_rx_buffer[3] >= '0') && (radio_rx_buffer[3] <= '9')))
                        {
                           pattern_type = (PATTERN_TYPE)(((radio_rx_buffer[2] - '0') * 10) + (radio_rx_buffer[3] - '0'));

                           update_pattern_type();

                           base_time = millis();
                        }

                        if ((radio_rx_buffer[0] == 'P') && (radio_rx_buffer[1] == 'S') && ((radio_rx_buffer[2] >= '0') && (radio_rx_buffer[2] <= '9')) && ((radio_rx_buffer[3] >= '0') && (radio_rx_buffer[3] <= '9')))
                        {
                           pattern_speed = (((radio_rx_buffer[2] - '0') * 10) + (radio_rx_buffer[3] - '0'));

                           update_pattern_speed();

                           base_time = millis();
                        }

                        if ((radio_rx_buffer[0] == 'P') && (radio_rx_buffer[1] == 'L') && ((radio_rx_buffer[2] >= '0') && (radio_rx_buffer[2] <= '9')) && ((radio_rx_buffer[3] >= '0') && (radio_rx_buffer[3] <= '9')))
                        {
                           pattern_length = (((radio_rx_buffer[2] - '0') * 10) + (radio_rx_buffer[3] - '0'));

                           update_pattern_length();

                           base_time = millis();
                        }
                     } else {
                        if ((radio_mode == RADIO_MODE_I_AM_PRIMARY) || (radio_mode == RADIO_MODE_SYNC_UP))
                        {
                           if (((radio_mode == RADIO_MODE_I_AM_PRIMARY) && ((millis() - base_time) >= SEND_KEEPALIVE_INTERVAL_IN_MILLIS)) || ((radio_mode == RADIO_MODE_SYNC_UP) && ((millis() - base_time) >= SEND_SYNC_INTERVAL_IN_MILLIS)))
                           {
                              base_time = millis();

                              radio_send(MSG_I_AM_PRIMARY);
                           }
                        }
                     }
                  }
                  break;
            }
         }
         break;
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
      Serial.print("RX (chan ");
      Serial.print(radio_channel);
      Serial.print(") : ");
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
   Serial.print("TX (chan ");
   Serial.print(radio_channel);
   Serial.print(") : ");
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

      radio.setChannel(radio_channel);
      radio.setPALevel(radio_power_level);
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


// update the radio status when in LED mapper mode
void radio_status(void)
{
   switch (ops_mode)
   {
      case OPS_MODE_TYPE_RV_LEVELING:
         {
            rv_radio_status();
         }
         break;

      case OPS_MODE_TYPE_LED_COLOR_MAPPER:
         {
            led_color_mapper_radio_status();
         }
         break;
   }
}


// read the brightness level from EEPROM
void read_bright_level(void)
{
   char eeprom_value = 0x00;
   char eeprom_inv_value = 0x00;
   int eeprom_index = EEPROM_INDEX_LED_BRIGHT_LEVEL;

   EEPROM.get((int)(eeprom_index), eeprom_value);
   EEPROM.get((int)(eeprom_index + 1), eeprom_inv_value);

   if ((eeprom_value >= 1) && (eeprom_value <= 8) && (((char)(~eeprom_value) == eeprom_inv_value)))
   {
      bright_level = eeprom_value;
      dynamic_bright_level = bright_level;
   } else {
      bright_level = 5;
      dynamic_bright_level = bright_level;

      save_bright_level();
   }

   share_bright_level();

#ifdef DEBUG_EEPROM_READ
   Serial.print("(READ ");
   show_index((int)(eeprom_index));
   Serial.print(": ");
   show_byte_value(eeprom_value);
   Serial.print(") Brightness Level: ");
   Serial.println(bright_level, DEC);

   Serial.print("(READ ");
   show_index((int)(eeprom_index + 1));
   Serial.print(": ");
   show_byte_value(eeprom_inv_value);
   Serial.print(") INV Brightness Level: ");
   Serial.println(eeprom_inv_value, DEC);
#endif
}  // read_bright_level()


// read the pattern number from EEPROM
void read_pattern_number(void)
{
   char eeprom_value = 0x00;
   char eeprom_inv_value = 0x00;
   int eeprom_index = EEPROM_INDEX_LED_PATTERN_NUMBER;

   EEPROM.get((int)(eeprom_index), eeprom_value);
   EEPROM.get((int)(eeprom_index + 1), eeprom_inv_value);

   if ((eeprom_value < NUMBER_OF_PATTERNS) && (((char)(~eeprom_value) == eeprom_inv_value)))
   {
      pattern_number = eeprom_value;
   } else {
      pattern_number = 0;

      save_pattern_number();
   }

#ifdef DEBUG_EEPROM_READ
   Serial.print("(READ ");
   show_index((int)(eeprom_index));
   Serial.print(": ");
   show_byte_value(eeprom_value);
   Serial.print(") Pattern Number: ");
   Serial.println(pattern_number, DEC);

   Serial.print("(READ ");
   show_index((int)(eeprom_index + 1));
   Serial.print(": ");
   show_byte_value(eeprom_inv_value);
   Serial.print(") INV Pattern Number: ");
   Serial.println(eeprom_inv_value, DEC);
#endif
}  // read_pattern_number()


// read the radio channel number from EEPROM
void read_radio_channel_number(void)
{
   char eeprom_value = 0x00;
   char eeprom_inv_value = 0x00;
   int eeprom_index = 0x00;

   switch (ops_mode)
   {
      case OPS_MODE_TYPE_RV_LEVELING:
         {
            eeprom_index = EEPROM_INDEX_RV_RADIO_CHANNEL_NUMBER;
         }
         break;

      case OPS_MODE_TYPE_LED_COLOR_MAPPER:
         {
            eeprom_index = EEPROM_INDEX_LED_RADIO_CHANNEL_NUMBER;
         }
         break;
   }

   EEPROM.get((int)(eeprom_index), eeprom_value);
   EEPROM.get((int)(eeprom_index + 1), eeprom_inv_value);

   if (((eeprom_value >= RADIO_CHANNEL_MIN) && (eeprom_value <= RADIO_CHANNEL_MAX)) && (((char)(~eeprom_value) == eeprom_inv_value)))
   {
      radio_channel = eeprom_value;
   } else {
      switch (ops_mode)
      {
         case OPS_MODE_TYPE_RV_LEVELING:
            {
               radio_channel = RV_RADIO_CHANNEL_DEFAULT;
            }
            break;

         case OPS_MODE_TYPE_LED_COLOR_MAPPER:
            {
               radio_channel = LED_RADIO_CHANNEL_DEFAULT;
            }
            break;
      }

      save_radio_channel_number();
   }

#ifdef DEBUG_EEPROM_READ
   Serial.print("(READ ");
   show_index((int)(eeprom_index));
   Serial.print(": ");
   show_byte_value(eeprom_value);
   Serial.print(") Radio Channel Number: ");
   Serial.println(radio_channel, DEC);

   Serial.print("(READ ");
   show_index((int)(eeprom_index + 1));
   Serial.print(": ");
   show_byte_value(eeprom_inv_value);
   Serial.print(") INV Radio Channel Number: ");
   Serial.println(eeprom_inv_value, DEC);
#endif
}  // read_radio_channel_number()


// read the radio power level from EEPROM
void read_radio_power_level(void)
{
   char eeprom_value = 0x00;
   char eeprom_inv_value = 0x00;
   int eeprom_index = 0x00;

   eeprom_index = EEPROM_INDEX_RADIO_POWER_LEVEL;

   EEPROM.get((int)(eeprom_index), eeprom_value);
   EEPROM.get((int)(eeprom_index + 1), eeprom_inv_value);

   if (((eeprom_value >= RADIO_POWER_LEVEL_LOW) && (eeprom_value <= RADIO_POWER_LEVEL_HIGH)) && (((char)(~eeprom_value) == eeprom_inv_value)))
   {
      radio_power_level = (RADIO_POWER_LEVEL)eeprom_value;
   } else {
      radio_power_level = RADIO_POWER_LEVEL_HIGH;

      save_radio_power_level();
   }

#ifdef DEBUG_EEPROM_READ
   Serial.print("(READ ");
   show_index((int)(eeprom_index));
   Serial.print(": ");
   show_byte_value(eeprom_value);
   Serial.print(") Radio Power Level: ");
   Serial.println(radio_power_level, DEC);

   Serial.print("(READ ");
   show_index((int)(eeprom_index + 1));
   Serial.print(": ");
   show_byte_value(eeprom_inv_value);
   Serial.print(") INV Radio Power Level: ");
   Serial.println(eeprom_inv_value, DEC);
#endif
}  // read_radio_power_level()


// read the current settings from EEPROM
boolean read_settings(void)
{
   byte eeprom_value, xor_value, inv_xor_value;
   byte xor_result = 0x4d;  // start with a non-zero value
   boolean header_is_good = true;
   boolean eeprom_settings_good = false;

   switch (ops_mode)
   {
      case OPS_MODE_TYPE_RV_LEVELING:
         {
            Serial.println("");
            Serial.print("Attempting to read/verify saved settings from EEPROM...");

            for (byte eeprom_index = EEPROM_INDEX_RV_LEVELING_HEADER_FIRST; eeprom_index < EEPROM_INDEX_RV_LEVELING_CHECKSUM; eeprom_index++)
            {
               EEPROM.get((int)(eeprom_index), eeprom_value);

               xor_result = xor_result ^ eeprom_value;

#ifdef DEBUG_EEPROM_READ
               if (eeprom_index == EEPROM_INDEX_RV_LEVELING_HEADER_FIRST)
               {
                  Serial.println("");
               }
               Serial.print("(READ ");
               show_index((int)(eeprom_index));
               Serial.print(": ");
               show_byte_value(eeprom_value);
               Serial.println(")");
#endif

               if (eeprom_index <= EEPROM_INDEX_RV_LEVELING_HEADER_LAST)
               {
                  if (eeprom_value != RV_EEPROM_HEADER[eeprom_index])
                  {
                     header_is_good = false;
                  }
               }
            }

            // read the checksum & inverse checksum
            EEPROM.get(EEPROM_INDEX_RV_LEVELING_CHECKSUM, xor_value);
            EEPROM.get(EEPROM_INDEX_RV_LEVELING_INV_CHECKSUM, inv_xor_value);

            // if the checksums match & the header values match, then we seem to have valid settings in EEPROM, so read all of the settings
            if ((xor_value == xor_result) && (inv_xor_value == (byte)~xor_result) && (header_is_good))
            {
               Serial.print("verified settings (");
               Serial.print(EEPROM_INDEX_RV_LEVELING_VALUE_COUNT + 2);  // +2 for radio channel number & inverse storage
               Serial.println(" values) in EEPROM are valid...");
               Serial.println("");

#ifndef DISABLE_EEPROM_READ_SETTINGS
               for (byte eeprom_index = EEPROM_INDEX_RV_LEVELING_HEADER_FIRST; eeprom_index <= EEPROM_INDEX_RV_LEVELING_INV_CHECKSUM; eeprom_index++)
               {
                  EEPROM.get((int)(eeprom_index), eeprom_value);

#ifdef DEBUG_EEPROM_READ
                  Serial.print("(READ ");
                  show_index((int)(eeprom_index));
                  Serial.print(": ");
                  show_byte_value(eeprom_value);
#endif

                  switch (eeprom_index)
                  {
                     case EEPROM_INDEX_RV_LEVELING_HEADER_00:
                     case EEPROM_INDEX_RV_LEVELING_HEADER_01:
                     case EEPROM_INDEX_RV_LEVELING_HEADER_02:
                     case EEPROM_INDEX_RV_LEVELING_HEADER_03:
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


                     case EEPROM_INDEX_RV_LEVELING_MODE:
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


                     case EEPROM_INDEX_RV_LEVELING_CHECKSUM:
                        {
                           eeprom_value = (char)(xor_result);

#ifdef DEBUG_EEPROM_READ
                           Serial.print(") Calculated CHECKSUM                            = ");
                           show_byte_value(xor_result);
                           Serial.println("");
#endif
                        }
                        break;

                     case EEPROM_INDEX_RV_LEVELING_INV_CHECKSUM:
                        {
                           eeprom_value = (char)(~xor_result);

#ifdef DEBUG_EEPROM_READ
                           Serial.print(") Calculated INVERSE CHECKSUM                    = ");
                           show_byte_value((byte)~xor_result);
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
                  show_index((int)(EEPROM_INDEX_RV_LEVELING_CHECKSUM));
                  Serial.print(") xor_value          = ");
                  Serial.println(xor_value);
                  Serial.print("(CALC) xor_result             = ");
                  Serial.println(xor_result);
                  Serial.print("(READ ");
                  show_index((int)(EEPROM_INDEX_RV_LEVELING_INV_CHECKSUM));
                  Serial.print(") inv_xor_value      = ");
                  Serial.println(inv_xor_value);
                  Serial.print("(CALC) inv_xor_result         = ");
                  Serial.println((byte)~xor_result);
               }

               Serial.println("");
#endif

               eeprom_settings_good = false;
            }
         }
         break;

      case OPS_MODE_TYPE_LED_COLOR_MAPPER:
         {
            Serial.println("");
            Serial.print("Attempting to read/verify saved settings (");
            Serial.print(EEPROM_INDEX_LED_VALUE_COUNT);
            Serial.print(" values) from EEPROM for pattern #");
            Serial.print(pattern_number + 1);
            Serial.print(" of ");
            Serial.print(NUMBER_OF_PATTERNS);
            Serial.print("...");

            for (int eeprom_index = EEPROM_INDEX_LED_HEADER_FIRST; eeprom_index < EEPROM_INDEX_LED_CHECKSUM; eeprom_index++)
            {
               EEPROM.get((int)(eeprom_index + (pattern_number * EEPROM_INDEX_LED_VALUE_COUNT)), eeprom_value);

               xor_result = xor_result ^ eeprom_value;

#ifdef DEBUG_EEPROM_READ
               if (eeprom_index == EEPROM_INDEX_LED_HEADER_FIRST)
               {
                  Serial.println("");
               }
               Serial.print("(READ ");
               show_index((int)(eeprom_index + (pattern_number * EEPROM_INDEX_LED_VALUE_COUNT)));
               Serial.print(": ");
               show_byte_value(eeprom_value);
               Serial.println(")");
#endif

               if (eeprom_index <= EEPROM_INDEX_LED_HEADER_LAST)
               {
                  if (eeprom_value != LED_EEPROM_HEADER[eeprom_index - EEPROM_INDEX_LED_HEADER_FIRST])
                  {
                     header_is_good = false;
                  }
               }
            }

            // read the checksum & inverse checksum
            EEPROM.get(EEPROM_INDEX_LED_CHECKSUM + (pattern_number * EEPROM_INDEX_LED_VALUE_COUNT), xor_value);
            EEPROM.get(EEPROM_INDEX_LED_INV_CHECKSUM + (pattern_number * EEPROM_INDEX_LED_VALUE_COUNT), inv_xor_value);

            // if the checksums match & the header values match, then we seem to have valid settings in EEPROM, so read all of the settings
            if ((xor_value == xor_result) && (inv_xor_value == (byte)~xor_result) && (header_is_good))
            {
               Serial.print("verified settings (");
               Serial.print(EEPROM_INDEX_LED_VALUE_COUNT);
               Serial.print(" values) in EEPROM for pattern number ");
               Serial.print(pattern_number + 1);
               Serial.print(" of ");
               Serial.print(NUMBER_OF_PATTERNS);
               Serial.print(" are valid...");
               Serial.println("");

#ifndef DISABLE_EEPROM_READ_SETTINGS
               for (byte eeprom_index = EEPROM_INDEX_LED_HEADER_FIRST; eeprom_index <= EEPROM_INDEX_LED_INV_CHECKSUM; eeprom_index++)
               {
                  EEPROM.get((int)(eeprom_index + (pattern_number * EEPROM_INDEX_LED_VALUE_COUNT)), eeprom_value);

#ifdef DEBUG_EEPROM_READ
                  Serial.print("(READ ");
                  show_index((int)(eeprom_index + (pattern_number * EEPROM_INDEX_LED_VALUE_COUNT)));
                  Serial.print(": ");
                  show_byte_value(eeprom_value);
#endif

                  switch (eeprom_index)
                  {
                     case EEPROM_INDEX_LED_HEADER_00:
                        {
                           if (eeprom_value != EEPROM_INDEX_LED_HEADER_00)
                           {
                              eeprom_settings_good = false;
                           }

#ifdef DEBUG_EEPROM_READ
                           Serial.print(") Header[");
                           Serial.print((eeprom_index - EEPROM_INDEX_LED_HEADER_FIRST) / 10);
                           Serial.print((eeprom_index - EEPROM_INDEX_LED_HEADER_FIRST) % 10);
                           Serial.print("]                                     = ");
                           Serial.println((char)eeprom_value);
#endif
                        }
                        break;

                     case EEPROM_INDEX_LED_HEADER_01:
                        {
                           if (eeprom_value != EEPROM_INDEX_LED_HEADER_01)
                           {
                              eeprom_settings_good = false;
                           }

#ifdef DEBUG_EEPROM_READ
                           Serial.print(") Header[");
                           Serial.print((eeprom_index - EEPROM_INDEX_LED_HEADER_FIRST) / 10);
                           Serial.print((eeprom_index - EEPROM_INDEX_LED_HEADER_FIRST) % 10);
                           Serial.print("]                                     = ");
                           Serial.println((char)eeprom_value);
#endif
                        }
                        break;

                     case EEPROM_INDEX_LED_HEADER_02:
                        {
                           if (eeprom_value != EEPROM_INDEX_LED_HEADER_02)
                           {
                              eeprom_settings_good = false;
                           }

#ifdef DEBUG_EEPROM_READ
                           Serial.print(") Header[");
                           Serial.print((eeprom_index - EEPROM_INDEX_LED_HEADER_FIRST) / 10);
                           Serial.print((eeprom_index - EEPROM_INDEX_LED_HEADER_FIRST) % 10);
                           Serial.print("]                                     = ");
                           Serial.println((char)eeprom_value);
#endif
                        }
                        break;

                     case EEPROM_INDEX_LED_HEADER_03:
                        {
                           if (eeprom_value != EEPROM_INDEX_LED_HEADER_03)
                           {
                              eeprom_settings_good = false;
                           }

#ifdef DEBUG_EEPROM_READ
                           Serial.print(") Header[");
                           Serial.print((eeprom_index - EEPROM_INDEX_LED_HEADER_FIRST) / 10);
                           Serial.print((eeprom_index - EEPROM_INDEX_LED_HEADER_FIRST) % 10);
                           Serial.print("]                                     = ");
                           Serial.println((char)eeprom_value);
#endif
                        }
                        break;


                     case EEPROM_INDEX_LED_PATTERN_TYPE:
                        {
                           pattern_type = (PATTERN_TYPE)eeprom_value;

                           if ((pattern_type < PATTERN_TYPE_MARQUIS_FORWARD) || (pattern_type > PATTERN_TYPE_RANDOM))
                           {
                              pattern_type = PATTERN_TYPE_MARQUIS_FORWARD;

                              eeprom_settings_good = false;
                           }

#ifdef DEBUG_EEPROM_READ
                           Serial.print(") Pattern Type:                            ");

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

                     case EEPROM_INDEX_LED_PATTERN_SPEED:
                        {
                           pattern_speed = eeprom_value;

                           if ((pattern_speed < 0) || (pattern_speed > 20))
                           {
                              pattern_speed = 0;

                              eeprom_settings_good = false;
                           }

#ifdef DEBUG_EEPROM_READ
                           Serial.print(") Pattern Speed:                           ");

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

                     case EEPROM_INDEX_LED_PATTERN_LENGTH:
                        {
                           pattern_length = eeprom_value;

                           if ((pattern_length < 1) || (pattern_length > PATTERN_LENGTH_MAX))
                           {
                              pattern_length = PATTERN_LENGTH_MAX;

                              eeprom_settings_good = false;
                           }

#ifdef DEBUG_EEPROM_READ
                           Serial.print(") Pattern Length:                          ");

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


                     case EEPROM_INDEX_LED_COLOR_01_HI_BYTE:
                        {
                           led_list[0] = eeprom_value * 0x10000;

#ifdef DEBUG_EEPROM_READ
                           Serial.print(") Color 01 (HI BYTE):                      ");

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

                     case EEPROM_INDEX_LED_COLOR_01_MID_BYTE:
                        {
                           led_list[0] += (byte)eeprom_value * 0x100;

#ifdef DEBUG_EEPROM_READ
                           Serial.print(") Color 01 (MID BYTE):                     ");

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

                     case EEPROM_INDEX_LED_COLOR_01_LO_BYTE:
                        {
                           led_list[0] += (byte)eeprom_value;

#ifdef DEBUG_EEPROM_READ
                           Serial.print(") Color 01 (LO BYTE):                      ");

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

                     case EEPROM_INDEX_LED_COLOR_02_HI_BYTE:
                        {
                           led_list[1] = eeprom_value * 0x10000;

#ifdef DEBUG_EEPROM_READ
                           Serial.print(") Color 02 (HI BYTE):                      ");

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

                     case EEPROM_INDEX_LED_COLOR_02_MID_BYTE:
                        {
                           led_list[1] += (byte)eeprom_value * 0x100;

#ifdef DEBUG_EEPROM_READ
                           Serial.print(") Color 02 (MID BYTE):                     ");

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

                     case EEPROM_INDEX_LED_COLOR_02_LO_BYTE:
                        {
                           led_list[1] += (byte)eeprom_value;

#ifdef DEBUG_EEPROM_READ
                           Serial.print(") Color 02 (LO BYTE):                      ");

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

                     case EEPROM_INDEX_LED_COLOR_03_HI_BYTE:
                        {
                           led_list[2] = eeprom_value * 0x10000;

#ifdef DEBUG_EEPROM_READ
                           Serial.print(") Color 03 (HI BYTE):                      ");

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

                     case EEPROM_INDEX_LED_COLOR_03_MID_BYTE:
                        {
                           led_list[2] += (byte)eeprom_value * 0x100;

#ifdef DEBUG_EEPROM_READ
                           Serial.print(") Color 03 (MID BYTE):                     ");

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

                     case EEPROM_INDEX_LED_COLOR_03_LO_BYTE:
                        {
                           led_list[2] += (byte)eeprom_value;

#ifdef DEBUG_EEPROM_READ
                           Serial.print(") Color 03 (LO BYTE):                      ");

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

                     case EEPROM_INDEX_LED_COLOR_04_HI_BYTE:
                        {
                           led_list[3] = eeprom_value * 0x10000;

#ifdef DEBUG_EEPROM_READ
                           Serial.print(") Color 04 (HI BYTE):                      ");

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

                     case EEPROM_INDEX_LED_COLOR_04_MID_BYTE:
                        {
                           led_list[3] += (byte)eeprom_value * 0x100;

#ifdef DEBUG_EEPROM_READ
                           Serial.print(") Color 04 (MID BYTE):                     ");

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

                     case EEPROM_INDEX_LED_COLOR_04_LO_BYTE:
                        {
                           led_list[3] += (byte)eeprom_value;

#ifdef DEBUG_EEPROM_READ
                           Serial.print(") Color 04 (LO BYTE):                      ");

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

                     case EEPROM_INDEX_LED_COLOR_05_HI_BYTE:
                        {
                           led_list[4] = eeprom_value * 0x10000;

#ifdef DEBUG_EEPROM_READ
                           Serial.print(") Color 05 (HI BYTE):                      ");

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

                     case EEPROM_INDEX_LED_COLOR_05_MID_BYTE:
                        {
                           led_list[4] += (byte)eeprom_value * 0x100;

#ifdef DEBUG_EEPROM_READ
                           Serial.print(") Color 05 (MID BYTE):                     ");

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

                     case EEPROM_INDEX_LED_COLOR_05_LO_BYTE:
                        {
                           led_list[4] += (byte)eeprom_value;

#ifdef DEBUG_EEPROM_READ
                           Serial.print(") Color 05 (LO BYTE):                      ");

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

                     case EEPROM_INDEX_LED_COLOR_06_HI_BYTE:
                        {
                           led_list[5] = eeprom_value * 0x10000;

#ifdef DEBUG_EEPROM_READ
                           Serial.print(") Color 06 (HI BYTE):                      ");

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

                     case EEPROM_INDEX_LED_COLOR_06_MID_BYTE:
                        {
                           led_list[5] += (byte)eeprom_value * 0x100;

#ifdef DEBUG_EEPROM_READ
                           Serial.print(") Color 06 (MID BYTE):                     ");

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

                     case EEPROM_INDEX_LED_COLOR_06_LO_BYTE:
                        {
                           led_list[5] += (byte)eeprom_value;

#ifdef DEBUG_EEPROM_READ
                           Serial.print(") Color 06 (LO BYTE):                      ");

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

                     case EEPROM_INDEX_LED_COLOR_07_HI_BYTE:
                        {
                           led_list[6] = eeprom_value * 0x10000;

#ifdef DEBUG_EEPROM_READ
                           Serial.print(") Color 07 (HI BYTE):                      ");

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

                     case EEPROM_INDEX_LED_COLOR_07_MID_BYTE:
                        {
                           led_list[6] += (byte)eeprom_value * 0x100;

#ifdef DEBUG_EEPROM_READ
                           Serial.print(") Color 07 (MID BYTE):                     ");

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

                     case EEPROM_INDEX_LED_COLOR_07_LO_BYTE:
                        {
                           led_list[6] += (byte)eeprom_value;

#ifdef DEBUG_EEPROM_READ
                           Serial.print(") Color 07 (LO BYTE):                      ");

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

                     case EEPROM_INDEX_LED_COLOR_08_HI_BYTE:
                        {
                           led_list[7] = eeprom_value * 0x10000;

#ifdef DEBUG_EEPROM_READ
                           Serial.print(") Color 08 (HI BYTE):                      ");

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

                     case EEPROM_INDEX_LED_COLOR_08_MID_BYTE:
                        {
                           led_list[7] += (byte)eeprom_value * 0x100;

#ifdef DEBUG_EEPROM_READ
                           Serial.print(") Color 08 (MID BYTE):                     ");

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

                     case EEPROM_INDEX_LED_COLOR_08_LO_BYTE:
                        {
                           led_list[7] += (byte)eeprom_value;

#ifdef DEBUG_EEPROM_READ
                           Serial.print(") Color 08 (LO BYTE):                      ");

                           Serial.print(" 0x");
                           if (eeprom_value < 16)
                              \ {
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

                     case EEPROM_INDEX_LED_COLOR_09_HI_BYTE:
                        {
                           led_list[8] = eeprom_value * 0x10000;

#ifdef DEBUG_EEPROM_READ
                           Serial.print(") Color 09 (HI BYTE):                      ");

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

                     case EEPROM_INDEX_LED_COLOR_09_MID_BYTE:
                        {
                           led_list[8] += (byte)eeprom_value * 0x100;

#ifdef DEBUG_EEPROM_READ
                           Serial.print(") Color 09 (MID BYTE):                     ");

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

                     case EEPROM_INDEX_LED_COLOR_09_LO_BYTE:
                        {
                           led_list[8] += (byte)eeprom_value;

#ifdef DEBUG_EEPROM_READ
                           Serial.print(") Color 09 (LO BYTE):                      ");

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

                     case EEPROM_INDEX_LED_COLOR_10_HI_BYTE:
                        {
                           led_list[9] = eeprom_value * 0x10000;

#ifdef DEBUG_EEPROM_READ
                           Serial.print(") Color 10 (HI BYTE):                      ");

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

                     case EEPROM_INDEX_LED_COLOR_10_MID_BYTE:
                        {
                           led_list[9] += (byte)eeprom_value * 0x100;

#ifdef DEBUG_EEPROM_READ
                           Serial.print(") Color 10 (MID BYTE):                     ");

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

                     case EEPROM_INDEX_LED_COLOR_10_LO_BYTE:
                        {
                           led_list[9] += (byte)eeprom_value;

#ifdef DEBUG_EEPROM_READ
                           Serial.print(") Color 10 (LO BYTE):                      ");

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

                     case EEPROM_INDEX_LED_COLOR_11_HI_BYTE:
                        {
                           led_list[10] = eeprom_value * 0x10000;

#ifdef DEBUG_EEPROM_READ
                           Serial.print(") Color 11 (HI BYTE):                      ");

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

                     case EEPROM_INDEX_LED_COLOR_11_MID_BYTE:
                        {
                           led_list[10] += (byte)eeprom_value * 0x100;

#ifdef DEBUG_EEPROM_READ
                           Serial.print(") Color 11 (MID BYTE):                     ");

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

                     case EEPROM_INDEX_LED_COLOR_11_LO_BYTE:
                        {
                           led_list[10] += (byte)eeprom_value;

#ifdef DEBUG_EEPROM_READ
                           Serial.print(") Color 11 (LO BYTE):                      ");

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

                     case EEPROM_INDEX_LED_COLOR_12_HI_BYTE:
                        {
                           led_list[11] = eeprom_value * 0x10000;

#ifdef DEBUG_EEPROM_READ
                           Serial.print(") Color 12 (HI BYTE):                      ");

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

                     case EEPROM_INDEX_LED_COLOR_12_MID_BYTE:
                        {
                           led_list[11] += (byte)eeprom_value * 0x100;

#ifdef DEBUG_EEPROM_READ
                           Serial.print(") Color 12 (MID BYTE):                     ");

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

                     case EEPROM_INDEX_LED_COLOR_12_LO_BYTE:
                        {
                           led_list[11] += (byte)eeprom_value;

#ifdef DEBUG_EEPROM_READ
                           Serial.print(") Color 12 (LO BYTE):                      ");

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

                     case EEPROM_INDEX_LED_COLOR_13_HI_BYTE:
                        {
                           led_list[12] = eeprom_value * 0x10000;

#ifdef DEBUG_EEPROM_READ
                           Serial.print(") Color 13 (HI BYTE):                      ");

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

                     case EEPROM_INDEX_LED_COLOR_13_MID_BYTE:
                        {
                           led_list[12] += (byte)eeprom_value * 0x100;

#ifdef DEBUG_EEPROM_READ
                           Serial.print(") Color 13 (MID BYTE):                     ");

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

                     case EEPROM_INDEX_LED_COLOR_13_LO_BYTE:
                        {
                           led_list[12] += (byte)eeprom_value;

#ifdef DEBUG_EEPROM_READ
                           Serial.print(") Color 13 (LO BYTE):                      ");

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

                     case EEPROM_INDEX_LED_COLOR_14_HI_BYTE:
                        {
                           led_list[13] = eeprom_value * 0x10000;

#ifdef DEBUG_EEPROM_READ
                           Serial.print(") Color 14 (HI BYTE):                      ");

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

                     case EEPROM_INDEX_LED_COLOR_14_MID_BYTE:
                        {
                           led_list[13] += (byte)eeprom_value * 0x100;

#ifdef DEBUG_EEPROM_READ
                           Serial.print(") Color 14 (MID BYTE):                     ");

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

                     case EEPROM_INDEX_LED_COLOR_14_LO_BYTE:
                        {
                           led_list[13] += (byte)eeprom_value;

#ifdef DEBUG_EEPROM_READ
                           Serial.print(") Color 14 (LO BYTE):                      ");

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

                     case EEPROM_INDEX_LED_COLOR_15_HI_BYTE:
                        {
                           led_list[14] = eeprom_value * 0x10000;

#ifdef DEBUG_EEPROM_READ
                           Serial.print(") Color 15 (HI BYTE):                      ");

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

                     case EEPROM_INDEX_LED_COLOR_15_MID_BYTE:
                        {
                           led_list[14] += (byte)eeprom_value * 0x100;

#ifdef DEBUG_EEPROM_READ
                           Serial.print(") Color 15 (MID BYTE):                     ");

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

                     case EEPROM_INDEX_LED_COLOR_15_LO_BYTE:
                        {
                           led_list[14] += (byte)eeprom_value;

#ifdef DEBUG_EEPROM_READ
                           Serial.print(") Color 15 (LO BYTE):                      ");

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

                     case EEPROM_INDEX_LED_COLOR_16_HI_BYTE:
                        {
                           led_list[15] = eeprom_value * 0x10000;

#ifdef DEBUG_EEPROM_READ
                           Serial.print(") Color 16 (HI BYTE):                      ");

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

                     case EEPROM_INDEX_LED_COLOR_16_MID_BYTE:
                        {
                           led_list[15] += (byte)eeprom_value * 0x100;

#ifdef DEBUG_EEPROM_READ
                           Serial.print(") Color 16 (MID BYTE):                     ");

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

                     case EEPROM_INDEX_LED_COLOR_16_LO_BYTE:
                        {
                           led_list[15] += (byte)eeprom_value;

#ifdef DEBUG_EEPROM_READ
                           Serial.print(") Color 16 (LO BYTE):                      ");

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

                     case EEPROM_INDEX_LED_COLOR_17_HI_BYTE:
                        {
                           led_list[16] = eeprom_value * 0x10000;

#ifdef DEBUG_EEPROM_READ
                           Serial.print(") Color 17 (HI BYTE):                      ");

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

                     case EEPROM_INDEX_LED_COLOR_17_MID_BYTE:
                        {
                           led_list[16] += (byte)eeprom_value * 0x100;

#ifdef DEBUG_EEPROM_READ
                           Serial.print(") Color 17 (MID BYTE):                     ");

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

                     case EEPROM_INDEX_LED_COLOR_17_LO_BYTE:
                        {
                           led_list[16] += (byte)eeprom_value;

#ifdef DEBUG_EEPROM_READ
                           Serial.print(") Color 17 (LO BYTE):                      ");

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

                     case EEPROM_INDEX_LED_COLOR_18_HI_BYTE:
                        {
                           led_list[17] = eeprom_value * 0x10000;

#ifdef DEBUG_EEPROM_READ
                           Serial.print(") Color 18 (HI BYTE):                      ");

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

                     case EEPROM_INDEX_LED_COLOR_18_MID_BYTE:
                        {
                           led_list[17] += (byte)eeprom_value * 0x100;

#ifdef DEBUG_EEPROM_READ
                           Serial.print(") Color 18 (MID BYTE):                     ");

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

                     case EEPROM_INDEX_LED_COLOR_18_LO_BYTE:
                        {
                           led_list[17] += (byte)eeprom_value;

#ifdef DEBUG_EEPROM_READ
                           Serial.print(") Color 18 (LO BYTE):                      ");

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

                     case EEPROM_INDEX_LED_COLOR_19_HI_BYTE:
                        {
                           led_list[18] = eeprom_value * 0x10000;

#ifdef DEBUG_EEPROM_READ
                           Serial.print(") Color 19 (HI BYTE):                      ");

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

                     case EEPROM_INDEX_LED_COLOR_19_MID_BYTE:
                        {
                           led_list[18] += (byte)eeprom_value * 0x100;

#ifdef DEBUG_EEPROM_READ
                           Serial.print(") Color 19 (MID BYTE):                     ");

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

                     case EEPROM_INDEX_LED_COLOR_19_LO_BYTE:
                        {
                           led_list[18] += (byte)eeprom_value;

#ifdef DEBUG_EEPROM_READ
                           Serial.print(") Color 19 (LO BYTE):                      ");

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

                     case EEPROM_INDEX_LED_COLOR_20_HI_BYTE:
                        {
                           led_list[19] = eeprom_value * 0x10000;

#ifdef DEBUG_EEPROM_READ
                           Serial.print(") Color 20 (HI BYTE):                      ");

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

                     case EEPROM_INDEX_LED_COLOR_20_MID_BYTE:
                        {
                           led_list[19] += (byte)eeprom_value * 0x100;

#ifdef DEBUG_EEPROM_READ
                           Serial.print(") Color 20 (MID BYTE):                     ");

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

                     case EEPROM_INDEX_LED_COLOR_20_LO_BYTE:
                        {
                           led_list[19] += (byte)eeprom_value;

#ifdef DEBUG_EEPROM_READ
                           Serial.print(") Color 20 (LO BYTE):                      ");

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

                     case EEPROM_INDEX_LED_COLOR_21_HI_BYTE:
                        {
                           led_list[20] = eeprom_value * 0x10000;

#ifdef DEBUG_EEPROM_READ
                           Serial.print(") Color 21 (HI BYTE):                      ");

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

                     case EEPROM_INDEX_LED_COLOR_21_MID_BYTE:
                        {
                           led_list[20] += (byte)eeprom_value * 0x100;

#ifdef DEBUG_EEPROM_READ
                           Serial.print(") Color 21 (MID BYTE):                     ");

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

                     case EEPROM_INDEX_LED_COLOR_21_LO_BYTE:
                        {
                           led_list[20] += (byte)eeprom_value;

#ifdef DEBUG_EEPROM_READ
                           Serial.print(") Color 21 (LO BYTE):                      ");

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

                     case EEPROM_INDEX_LED_COLOR_22_HI_BYTE:
                        {
                           led_list[21] = eeprom_value * 0x10000;

#ifdef DEBUG_EEPROM_READ
                           Serial.print(") Color 22 (HI BYTE):                      ");

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

                     case EEPROM_INDEX_LED_COLOR_22_MID_BYTE:
                        {
                           led_list[21] += (byte)eeprom_value * 0x100;

#ifdef DEBUG_EEPROM_READ
                           Serial.print(") Color 22 (MID BYTE):                     ");

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

                     case EEPROM_INDEX_LED_COLOR_22_LO_BYTE:
                        {
                           led_list[21] += (byte)eeprom_value;

#ifdef DEBUG_EEPROM_READ
                           Serial.print(") Color 22 (LO BYTE):                      ");

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

                     case EEPROM_INDEX_LED_COLOR_23_HI_BYTE:
                        {
                           led_list[22] = eeprom_value * 0x10000;

#ifdef DEBUG_EEPROM_READ
                           Serial.print(") Color 23 (HI BYTE):                      ");

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

                     case EEPROM_INDEX_LED_COLOR_23_MID_BYTE:
                        {
                           led_list[22] += (byte)eeprom_value * 0x100;

#ifdef DEBUG_EEPROM_READ
                           Serial.print(") Color 23 (MID BYTE):                     ");

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

                     case EEPROM_INDEX_LED_COLOR_23_LO_BYTE:
                        {
                           led_list[22] += (byte)eeprom_value;

#ifdef DEBUG_EEPROM_READ
                           Serial.print(") Color 23 (LO BYTE):                      ");

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

                     case EEPROM_INDEX_LED_COLOR_24_HI_BYTE:
                        {
                           led_list[23] = eeprom_value * 0x10000;

#ifdef DEBUG_EEPROM_READ
                           Serial.print(") Color 24 (HI BYTE):                      ");

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

                     case EEPROM_INDEX_LED_COLOR_24_MID_BYTE:
                        {
                           led_list[23] += (byte)eeprom_value * 0x100;

#ifdef DEBUG_EEPROM_READ
                           Serial.print(") Color 24 (MID BYTE):                     ");

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

                     case EEPROM_INDEX_LED_COLOR_24_LO_BYTE:
                        {
                           led_list[23] += (byte)eeprom_value;

#ifdef DEBUG_EEPROM_READ
                           Serial.print(") Color 24 (LO BYTE):                      ");

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

                     case EEPROM_INDEX_LED_COLOR_25_HI_BYTE:
                        {
                           led_list[24] = eeprom_value * 0x10000;

#ifdef DEBUG_EEPROM_READ
                           Serial.print(") Color 25 (HI BYTE):                      ");

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

                     case EEPROM_INDEX_LED_COLOR_25_MID_BYTE:
                        {
                           led_list[24] += (byte)eeprom_value * 0x100;

#ifdef DEBUG_EEPROM_READ
                           Serial.print(") Color 25 (MID BYTE):                     ");

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

                     case EEPROM_INDEX_LED_COLOR_25_LO_BYTE:
                        {
                           led_list[24] += (byte)eeprom_value;

#ifdef DEBUG_EEPROM_READ
                           Serial.print(") Color 25 (LO BYTE):                      ");

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

                     case EEPROM_INDEX_LED_COLOR_26_HI_BYTE:
                        {
                           led_list[25] = eeprom_value * 0x10000;

#ifdef DEBUG_EEPROM_READ
                           Serial.print(") Color 26 (HI BYTE):                      ");

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

                     case EEPROM_INDEX_LED_COLOR_26_MID_BYTE:
                        {
                           led_list[25] += (byte)eeprom_value * 0x100;

#ifdef DEBUG_EEPROM_READ
                           Serial.print(") Color 26 (MID BYTE):                     ");

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

                     case EEPROM_INDEX_LED_COLOR_26_LO_BYTE:
                        {
                           led_list[25] += (byte)eeprom_value;

#ifdef DEBUG_EEPROM_READ
                           Serial.print(") Color 26 (LO BYTE):                      ");

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

                     case EEPROM_INDEX_LED_COLOR_27_HI_BYTE:
                        {
                           led_list[26] = eeprom_value * 0x10000;

#ifdef DEBUG_EEPROM_READ
                           Serial.print(") Color 27 (HI BYTE):                      ");

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

                     case EEPROM_INDEX_LED_COLOR_27_MID_BYTE:
                        {
                           led_list[26] += (byte)eeprom_value * 0x100;

#ifdef DEBUG_EEPROM_READ
                           Serial.print(") Color 27 (MID BYTE):                     ");

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

                     case EEPROM_INDEX_LED_COLOR_27_LO_BYTE:
                        {
                           led_list[26] += (byte)eeprom_value;

#ifdef DEBUG_EEPROM_READ
                           Serial.print(") Color 27 (LO BYTE):                      ");

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

                     case EEPROM_INDEX_LED_COLOR_28_HI_BYTE:
                        {
                           led_list[27] = eeprom_value * 0x10000;

#ifdef DEBUG_EEPROM_READ
                           Serial.print(") Color 28 (HI BYTE):                      ");

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

                     case EEPROM_INDEX_LED_COLOR_28_MID_BYTE:
                        {
                           led_list[27] += (byte)eeprom_value * 0x100;

#ifdef DEBUG_EEPROM_READ
                           Serial.print(") Color 28 (MID BYTE):                     ");

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

                     case EEPROM_INDEX_LED_COLOR_28_LO_BYTE:
                        {
                           led_list[27] += (byte)eeprom_value;

#ifdef DEBUG_EEPROM_READ
                           Serial.print(") Color 28 (LO BYTE):                      ");

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

                     case EEPROM_INDEX_LED_COLOR_29_HI_BYTE:
                        {
                           led_list[28] = eeprom_value * 0x10000;

#ifdef DEBUG_EEPROM_READ
                           Serial.print(") Color 29 (HI BYTE):                      ");

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

                     case EEPROM_INDEX_LED_COLOR_29_MID_BYTE:
                        {
                           led_list[28] += (byte)eeprom_value * 0x100;

#ifdef DEBUG_EEPROM_READ
                           Serial.print(") Color 29 (MID BYTE):                     ");

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

                     case EEPROM_INDEX_LED_COLOR_29_LO_BYTE:
                        {
                           led_list[28] += (byte)eeprom_value;

#ifdef DEBUG_EEPROM_READ
                           Serial.print(") Color 29 (LO BYTE):                      ");

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

                     case EEPROM_INDEX_LED_COLOR_30_HI_BYTE:
                        {
                           led_list[29] = eeprom_value * 0x10000;

#ifdef DEBUG_EEPROM_READ
                           Serial.print(") Color 30 (HI BYTE):                      ");

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

                     case EEPROM_INDEX_LED_COLOR_30_MID_BYTE:
                        {
                           led_list[29] += (byte)eeprom_value * 0x100;

#ifdef DEBUG_EEPROM_READ
                           Serial.print(") Color 30 (MID BYTE):                     ");

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

                     case EEPROM_INDEX_LED_COLOR_30_LO_BYTE:
                        {
                           led_list[29] += (byte)eeprom_value;

#ifdef DEBUG_EEPROM_READ
                           Serial.print(") Color 30 (LO BYTE):                      ");

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

                     case EEPROM_INDEX_LED_COLOR_31_HI_BYTE:
                        {
                           led_list[30] = eeprom_value * 0x10000;

#ifdef DEBUG_EEPROM_READ
                           Serial.print(") Color 31 (HI BYTE):                      ");

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

                     case EEPROM_INDEX_LED_COLOR_31_MID_BYTE:
                        {
                           led_list[30] += (byte)eeprom_value * 0x100;

#ifdef DEBUG_EEPROM_READ
                           Serial.print(") Color 31 (MID BYTE):                     ");

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

                     case EEPROM_INDEX_LED_COLOR_31_LO_BYTE:
                        {
                           led_list[30] += (byte)eeprom_value;

#ifdef DEBUG_EEPROM_READ
                           Serial.print(") Color 31 (LO BYTE):                      ");

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

                     case EEPROM_INDEX_LED_COLOR_32_HI_BYTE:
                        {
                           led_list[31] = eeprom_value * 0x10000;

#ifdef DEBUG_EEPROM_READ
                           Serial.print(") Color 32 (HI BYTE):                      ");

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

                     case EEPROM_INDEX_LED_COLOR_32_MID_BYTE:
                        {
                           led_list[31] += (byte)eeprom_value * 0x100;

#ifdef DEBUG_EEPROM_READ
                           Serial.print(") Color 32 (MID BYTE):                     ");

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

                     case EEPROM_INDEX_LED_COLOR_32_LO_BYTE:
                        {
                           led_list[31] += (byte)eeprom_value;

#ifdef DEBUG_EEPROM_READ
                           Serial.print(") Color 32 (LO BYTE):                      ");

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


                     case EEPROM_INDEX_LED_CHECKSUM:
                        {
                           eeprom_value = (char)(xor_result);

#ifdef DEBUG_EEPROM_READ
                           Serial.print(") Calculated CHECKSUM                            = ");
                           show_byte_value(xor_result);
                           Serial.println("");
#endif
                        }
                        break;

                     case EEPROM_INDEX_LED_INV_CHECKSUM:
                        {
                           eeprom_value = (char)(~xor_result);

#ifdef DEBUG_EEPROM_READ
                           Serial.print(") Calculated INVERSE CHECKSUM                    = ");
                           show_byte_value((byte)~xor_result);
                           Serial.println("");
#endif
                        }
                        break;
                  }
               }
#endif

               eeprom_settings_good = true;
            } else {
               Serial.print("EEPROM pattern #");
               Serial.print(pattern_number + 1);
               Serial.print(" of ");
               Serial.print(NUMBER_OF_PATTERNS);
               Serial.println(" values failed checksum verification...");
               Serial.println("");
               Serial.print("Discarding invalid EEPROM pattern #");
               Serial.print(pattern_number + 1);
               Serial.print(" of ");
               Serial.print(NUMBER_OF_PATTERNS);
               Serial.println(" settings & storing/using defaults...");
               Serial.println("");

#ifdef DEBUG_EEPROM_READ
               if (!header_is_good)
               {
                  Serial.print("HEADER mismatch between EEPROM pattern #");
                  Serial.print(pattern_number + 1);
                  Serial.print(" of ");
                  Serial.print(NUMBER_OF_PATTERNS);
                  Serial.println(" values & expected values...");
               } else {
                  Serial.print("CHECKSUM mismatch between EEPROM pattern #");
                  Serial.print(pattern_number + 1);
                  Serial.print(" of ");
                  Serial.print(NUMBER_OF_PATTERNS);
                  Serial.println(" values & expected values...");
                  Serial.print("(READ ");
                  show_index((int)(EEPROM_INDEX_LED_CHECKSUM + (pattern_number * EEPROM_INDEX_LED_VALUE_COUNT)));
                  Serial.print(") xor_value          = ");
                  Serial.println(xor_value);
                  Serial.print("(CALC) xor_result             = ");
                  Serial.println(xor_result);
                  Serial.print("(READ ");
                  show_index((int)(EEPROM_INDEX_LED_INV_CHECKSUM + (pattern_number * EEPROM_INDEX_LED_VALUE_COUNT)));
                  Serial.print(") inv_xor_value      = ");
                  Serial.println(inv_xor_value);
                  Serial.print("(CALC) inv_xor_result         = ");
                  Serial.println((byte)~xor_result);
               }

               Serial.println("");
#endif

               eeprom_settings_good = false;
            }
         }
         break;
   }

   return (eeprom_settings_good);
}  // read_settings()


// update the radio status when in RV leveling mode
void rv_radio_status(void)
{
   switch (radio_mode)
   {
      case RADIO_MODE_INITIAL_SYNC:
      case RADIO_MODE_SYNC_UP:
         {
            center_draw_text(PAIRING, 1, 180, 47, ILI9341_BLACK, ILI9341_WHITE);
         }
         break;

      case RADIO_MODE_I_AM_PRIMARY:
         {
            if (!(radio_not_present))
            {
               center_draw_text(RV_INSIDE_UNIT, 1, 180, 47, ILI9341_BLACK, ILI9341_WHITE);
            } else {
               center_draw_text(RV_STANDALONE_UNIT, 1, 180, 47, ILI9341_BLACK, ILI9341_WHITE);
            }
         }
         break;

      case RADIO_MODE_I_AM_SECONDARY:
         {
            center_draw_text(RV_OUTSIDE_UNIT, 1, 180, 47, ILI9341_BLACK, ILI9341_WHITE);
         }
         break;
   }
}  // rv_radio_status()


// save the brightness level to EEPROM
void save_bright_level(void)
{
   char eeprom_value = (byte)bright_level;
   char eeprom_inv_value = (byte)(~bright_level);
   int eeprom_index = EEPROM_INDEX_LED_BRIGHT_LEVEL;

#ifdef DEBUG_EEPROM_WRITE
   Serial.print("(WRITE ");
   show_index((int)(eeprom_index));
   Serial.print(": ");
   show_byte_value(eeprom_value);
   Serial.print(") Brightness Level: ");
   Serial.println(eeprom_value, DEC);

   Serial.print("(WRITE ");
   show_index((int)(eeprom_index + 1));
   Serial.print(": ");
   show_byte_value(eeprom_inv_value);
   Serial.print(") INV Brightness Level: ");
   Serial.println(eeprom_inv_value, DEC);
#endif

#ifndef DISABLE_EEPROM_WRITE_SETTINGS
   EEPROM.update((int)(eeprom_index), (byte)(eeprom_value));
   EEPROM.update((int)(eeprom_index + 1), (byte)(eeprom_inv_value));
#endif
}  // save_bright_level()


// save the radio channel number to EEPROM
void save_radio_channel_number(void)
{
   char eeprom_value = (byte)radio_channel;
   char eeprom_inv_value = (byte)(~radio_channel);
   int eeprom_index = 0;

   switch (ops_mode)
   {
      case OPS_MODE_TYPE_RV_LEVELING:
         {
            eeprom_index = EEPROM_INDEX_RV_RADIO_CHANNEL_NUMBER;
         }
         break;

      case OPS_MODE_TYPE_LED_COLOR_MAPPER:
         {
            eeprom_index = EEPROM_INDEX_LED_RADIO_CHANNEL_NUMBER;
         }
         break;
   }

#ifdef DEBUG_EEPROM_WRITE
   Serial.print("(WRITE ");
   show_index((int)(eeprom_index));
   Serial.print(": ");
   show_byte_value(eeprom_value);
   Serial.print(") Radio Channel Number: ");
   Serial.println(eeprom_value, DEC);

   Serial.print("(WRITE ");
   show_index((int)(eeprom_index + 1));
   Serial.print(": ");
   show_byte_value(eeprom_inv_value);
   Serial.print(") INV Radio Channel Number: ");
   Serial.println(eeprom_inv_value, DEC);
#endif

#ifndef DISABLE_EEPROM_WRITE_SETTINGS
   EEPROM.update((int)(eeprom_index), (byte)(eeprom_value));
   EEPROM.update((int)(eeprom_index + 1), (byte)(eeprom_inv_value));
#endif
}  // save_radio_channel_number()


// save the radio power_level to EEPROM
void save_radio_power_level(void)
{
   char eeprom_value = (byte)radio_power_level;
   char eeprom_inv_value = (byte)(~radio_power_level);
   int eeprom_index = 0;

   eeprom_index = EEPROM_INDEX_RADIO_POWER_LEVEL;

#ifdef DEBUG_EEPROM_WRITE
   Serial.print("(WRITE ");
   show_index((int)(eeprom_index));
   Serial.print(": ");
   show_byte_value(eeprom_value);
   Serial.print(") Radio Power Level: ");
   Serial.println(eeprom_value, DEC);

   Serial.print("(WRITE ");
   show_index((int)(eeprom_index + 1));
   Serial.print(": ");
   show_byte_value(eeprom_inv_value);
   Serial.print(") INV Radio Power Level: ");
   Serial.println(eeprom_inv_value, DEC);
#endif

#ifndef DISABLE_EEPROM_WRITE_SETTINGS
   EEPROM.update((int)(eeprom_index), (byte)(eeprom_value));
   EEPROM.update((int)(eeprom_index + 1), (byte)(eeprom_inv_value));
#endif
}  // save_radio_power_level()


// save the pattern number to EEPROM
void save_pattern_number(void)
{
   char eeprom_value = pattern_number;
   char eeprom_inv_value = (char)(~pattern_number);
   int eeprom_index = EEPROM_INDEX_LED_PATTERN_NUMBER;

#ifdef DEBUG_EEPROM_WRITE
   Serial.print("(WRITE ");
   show_index((int)(eeprom_index));
   Serial.print(": ");
   show_byte_value(eeprom_value);
   Serial.print(") Pattern Number: ");
   Serial.println(eeprom_value, DEC);

   Serial.print("(WRITE ");
   show_index((int)(eeprom_index + 1));
   Serial.print(": ");
   show_byte_value(eeprom_inv_value);
   Serial.print(") INV Pattern Number: ");
   Serial.println(eeprom_inv_value, DEC);
#endif

#ifndef DISABLE_EEPROM_WRITE_SETTINGS
   EEPROM.update((int)(eeprom_index), (byte)(eeprom_value));
   EEPROM.update((int)(eeprom_index + 1), (byte)(eeprom_inv_value));
#endif
}  // save_pattern_number()


// save the current settings to EEPROM
void save_settings(void)
{
   byte xor_result = 0x4D;  // start with a non-zero value
   char eeprom_value = 0x00;

   switch (ops_mode)
   {
      case OPS_MODE_TYPE_RV_LEVELING:
         {
            Serial.println("");
            Serial.print("Saving settings (");
            Serial.print(EEPROM_INDEX_RV_LEVELING_VALUE_COUNT + 2);  // +2 for radio channel number & inverse storage
            Serial.print(" values) to EEPROM...");
            Serial.println("");

            for (byte eeprom_index = (byte)(EEPROM_INDEX_RV_LEVELING_HEADER_FIRST); eeprom_index <= (byte)(EEPROM_INDEX_RV_LEVELING_INV_CHECKSUM); eeprom_index++)
            {
#ifdef DEBUG_EEPROM_WRITE
               Serial.print("(WRITE ");
               show_index((int)(eeprom_index));
               Serial.print(": ");
#endif

               switch (eeprom_index)
               {
                  case EEPROM_INDEX_RV_LEVELING_HEADER_00:
                  case EEPROM_INDEX_RV_LEVELING_HEADER_01:
                  case EEPROM_INDEX_RV_LEVELING_HEADER_02:
                  case EEPROM_INDEX_RV_LEVELING_HEADER_03:
                     {
                        eeprom_value = RV_EEPROM_HEADER[eeprom_index];

#ifdef DEBUG_EEPROM_WRITE
                        show_byte_value(eeprom_value);
                        Serial.print(") Header[");
                        Serial.print(eeprom_index / 10);
                        Serial.print(eeprom_index % 10);
                        Serial.print("]                                     = ");
                        Serial.println(eeprom_value);
#endif
                     }
                     break;


                  case EEPROM_INDEX_RV_LEVELING_MODE:
                     {
                        eeprom_value = (byte)trlh_mode;

#ifdef DEBUG_EEPROM_WRITE
                        show_byte_value(eeprom_value);

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
                        show_byte_value(eeprom_value);

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
                        show_byte_value(eeprom_value);

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



                  case EEPROM_INDEX_RV_LEVELING_CHECKSUM:
                     {
                        eeprom_value = (char)(xor_result);

#ifdef DEBUG_EEPROM_WRITE
                        show_byte_value(eeprom_value);
                        Serial.print(") Calculated CHECKSUM                            = ");
                        show_byte_value(xor_result);
                        Serial.println("");
#endif
                     }
                     break;

                  case EEPROM_INDEX_RV_LEVELING_INV_CHECKSUM:
                     {
                        eeprom_value = (char)(~xor_result);

#ifdef DEBUG_EEPROM_WRITE
                        show_byte_value(eeprom_value);
                        Serial.print(") Calculated INVERSE CHECKSUM                    = ");
                        show_byte_value((byte)~xor_result);
                        Serial.println("");
#endif
                     }
                     break;
               }

#ifndef DISABLE_EEPROM_WRITE_SETTINGS
               EEPROM.update((int)(eeprom_index), (byte)(eeprom_value));
#endif
               if (eeprom_index < EEPROM_INDEX_RV_LEVELING_CHECKSUM)
               {
                  xor_result = (byte)(xor_result ^ eeprom_value);
               }
            }
         }
         break;

      case OPS_MODE_TYPE_LED_COLOR_MAPPER:
         {
            Serial.println("");
            Serial.print("Saving settings (");
            Serial.print(EEPROM_INDEX_LED_VALUE_COUNT);
            Serial.print(" values) to EEPROM for pattern #");
            Serial.print(pattern_number + 1);
            Serial.print(" of ");
            Serial.print(NUMBER_OF_PATTERNS);
            Serial.println("...");
            Serial.println("");

            for (byte eeprom_index = (byte)(EEPROM_INDEX_LED_HEADER_FIRST); eeprom_index <= (byte)(EEPROM_INDEX_LED_INV_CHECKSUM); eeprom_index++)
            {
#ifdef DEBUG_EEPROM_WRITE
               Serial.print("(WRITE ");
               show_index((int)(eeprom_index + (pattern_number * EEPROM_INDEX_LED_VALUE_COUNT)));
               Serial.print(": ");
#endif

               switch (eeprom_index)
               {
                  case EEPROM_INDEX_LED_HEADER_00:
                  case EEPROM_INDEX_LED_HEADER_01:
                  case EEPROM_INDEX_LED_HEADER_02:
                  case EEPROM_INDEX_LED_HEADER_03:
                     {
                        eeprom_value = LED_EEPROM_HEADER[(eeprom_index - EEPROM_INDEX_LED_HEADER_FIRST)];

#ifdef DEBUG_EEPROM_WRITE
                        show_byte_value(eeprom_value);
                        Serial.print(") Header[");
                        Serial.print((eeprom_index - EEPROM_INDEX_LED_HEADER_FIRST) / 10);
                        Serial.print((eeprom_index - EEPROM_INDEX_LED_HEADER_FIRST) % 10);
                        Serial.print("]                                     = ");
                        Serial.println(eeprom_value);
#endif
                     }
                     break;


                  case EEPROM_INDEX_LED_PATTERN_TYPE:
                     {
                        eeprom_value = (byte)pattern_type;

#ifdef DEBUG_EEPROM_WRITE
                        show_byte_value(eeprom_value);

                        Serial.print(") Pattern Type:                            ");

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

                  case EEPROM_INDEX_LED_PATTERN_SPEED:
                     {
                        eeprom_value = (byte)pattern_speed;

#ifdef DEBUG_EEPROM_WRITE
                        show_byte_value(eeprom_value);

                        Serial.print(") Pattern Speed:                           ");

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

                  case EEPROM_INDEX_LED_PATTERN_LENGTH:
                     {
                        eeprom_value = (byte)pattern_length;

#ifdef DEBUG_EEPROM_WRITE
                        show_byte_value(eeprom_value);

                        Serial.print(") Pattern Length:                          ");

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

                  case EEPROM_INDEX_LED_COLOR_01_HI_BYTE:
                     {
                        eeprom_value = (byte)(led_list[0] / 0x10000);

#ifdef DEBUG_EEPROM_WRITE
                        show_byte_value(eeprom_value);

                        Serial.print(") Color 01 (HI BYTE)                       ");

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

                  case EEPROM_INDEX_LED_COLOR_01_MID_BYTE:
                     {
                        eeprom_value = ((byte)(led_list[0] / 0x100) % 0x100);

#ifdef DEBUG_EEPROM_WRITE
                        show_byte_value(eeprom_value);

                        Serial.print(") Color 01 (MID BYTE)                      ");

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

                  case EEPROM_INDEX_LED_COLOR_01_LO_BYTE:
                     {
                        eeprom_value = (byte)(led_list[0] % 0x100);

#ifdef DEBUG_EEPROM_WRITE
                        show_byte_value(eeprom_value);

                        Serial.print(") Color 01 (LO BYTE)                       ");

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

                  case EEPROM_INDEX_LED_COLOR_02_HI_BYTE:
                     {
                        eeprom_value = (byte)(led_list[1] / 0x10000);

#ifdef DEBUG_EEPROM_WRITE
                        show_byte_value(eeprom_value);

                        Serial.print(") Color 02 (HI BYTE)                       ");

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

                  case EEPROM_INDEX_LED_COLOR_02_MID_BYTE:
                     {
                        eeprom_value = ((byte)(led_list[1] / 0x100) % 0x100);

#ifdef DEBUG_EEPROM_WRITE
                        show_byte_value(eeprom_value);

                        Serial.print(") Color 02 (MID BYTE)                      ");

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

                  case EEPROM_INDEX_LED_COLOR_02_LO_BYTE:
                     {
                        eeprom_value = (byte)(led_list[1] % 0x100);

#ifdef DEBUG_EEPROM_WRITE
                        show_byte_value(eeprom_value);

                        Serial.print(") Color 02 (LO BYTE)                       ");

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

                  case EEPROM_INDEX_LED_COLOR_03_HI_BYTE:
                     {
                        eeprom_value = (byte)(led_list[2] / 0x10000);

#ifdef DEBUG_EEPROM_WRITE
                        show_byte_value(eeprom_value);

                        Serial.print(") Color 03 (HI BYTE)                       ");

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

                  case EEPROM_INDEX_LED_COLOR_03_MID_BYTE:
                     {
                        eeprom_value = ((byte)(led_list[2] / 0x100) % 0x100);

#ifdef DEBUG_EEPROM_WRITE
                        show_byte_value(eeprom_value);

                        Serial.print(") Color 03 (MID BYTE)                      ");

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

                  case EEPROM_INDEX_LED_COLOR_03_LO_BYTE:
                     {
                        eeprom_value = (byte)(led_list[2] % 0x100);

#ifdef DEBUG_EEPROM_WRITE
                        show_byte_value(eeprom_value);

                        Serial.print(") Color 03 (LO BYTE)                       ");

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

                  case EEPROM_INDEX_LED_COLOR_04_HI_BYTE:
                     {
                        eeprom_value = (byte)(led_list[3] / 0x10000);

#ifdef DEBUG_EEPROM_WRITE
                        show_byte_value(eeprom_value);

                        Serial.print(") Color 04 (HI BYTE)                       ");

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

                  case EEPROM_INDEX_LED_COLOR_04_MID_BYTE:
                     {
                        eeprom_value = ((byte)(led_list[3] / 0x100) % 0x100);

#ifdef DEBUG_EEPROM_WRITE
                        show_byte_value(eeprom_value);

                        Serial.print(") Color 04 (MID BYTE)                      ");

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

                  case EEPROM_INDEX_LED_COLOR_04_LO_BYTE:
                     {
                        eeprom_value = (byte)(led_list[3] % 0x100);

#ifdef DEBUG_EEPROM_WRITE
                        show_byte_value(eeprom_value);

                        Serial.print(") Color 04 (LO BYTE)                       ");

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

                  case EEPROM_INDEX_LED_COLOR_05_HI_BYTE:
                     {
                        eeprom_value = (byte)(led_list[4] / 0x10000);

#ifdef DEBUG_EEPROM_WRITE
                        show_byte_value(eeprom_value);

                        Serial.print(") Color 05 (HI BYTE)                       ");

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

                  case EEPROM_INDEX_LED_COLOR_05_MID_BYTE:
                     {
                        eeprom_value = ((byte)(led_list[4] / 0x100) % 0x100);

#ifdef DEBUG_EEPROM_WRITE
                        show_byte_value(eeprom_value);

                        Serial.print(") Color 05 (MID BYTE)                      ");

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

                  case EEPROM_INDEX_LED_COLOR_05_LO_BYTE:
                     {
                        eeprom_value = (byte)(led_list[4] % 0x100);

#ifdef DEBUG_EEPROM_WRITE
                        show_byte_value(eeprom_value);

                        Serial.print(") Color 05 (LO BYTE)                       ");

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

                  case EEPROM_INDEX_LED_COLOR_06_HI_BYTE:
                     {
                        eeprom_value = (byte)(led_list[5] / 0x10000);

#ifdef DEBUG_EEPROM_WRITE
                        show_byte_value(eeprom_value);

                        Serial.print(") Color 06 (HI BYTE)                       ");

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

                  case EEPROM_INDEX_LED_COLOR_06_MID_BYTE:
                     {
                        eeprom_value = ((byte)(led_list[5] / 0x100) % 0x100);

#ifdef DEBUG_EEPROM_WRITE
                        show_byte_value(eeprom_value);

                        Serial.print(") Color 06 (MID BYTE)                      ");

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

                  case EEPROM_INDEX_LED_COLOR_06_LO_BYTE:
                     {
                        eeprom_value = (byte)(led_list[5] % 0x100);

#ifdef DEBUG_EEPROM_WRITE
                        show_byte_value(eeprom_value);

                        Serial.print(") Color 06 (LO BYTE)                       ");

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

                  case EEPROM_INDEX_LED_COLOR_07_HI_BYTE:
                     {
                        eeprom_value = (byte)(led_list[6] / 0x10000);

#ifdef DEBUG_EEPROM_WRITE
                        show_byte_value(eeprom_value);

                        Serial.print(") Color 07 (HI BYTE)                       ");

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

                  case EEPROM_INDEX_LED_COLOR_07_MID_BYTE:
                     {
                        eeprom_value = ((byte)(led_list[6] / 0x100) % 0x100);

#ifdef DEBUG_EEPROM_WRITE
                        show_byte_value(eeprom_value);

                        Serial.print(") Color 07 (MID BYTE)                      ");

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

                  case EEPROM_INDEX_LED_COLOR_07_LO_BYTE:
                     {
                        eeprom_value = (byte)(led_list[6] % 0x100);

#ifdef DEBUG_EEPROM_WRITE
                        show_byte_value(eeprom_value);

                        Serial.print(") Color 07 (LO BYTE)                       ");

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

                  case EEPROM_INDEX_LED_COLOR_08_HI_BYTE:
                     {
                        eeprom_value = (byte)(led_list[7] / 0x10000);

#ifdef DEBUG_EEPROM_WRITE
                        show_byte_value(eeprom_value);

                        Serial.print(") Color 08 (HI BYTE)                       ");

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

                  case EEPROM_INDEX_LED_COLOR_08_MID_BYTE:
                     {
                        eeprom_value = ((byte)(led_list[7] / 0x100) % 0x100);

#ifdef DEBUG_EEPROM_WRITE
                        show_byte_value(eeprom_value);

                        Serial.print(") Color 08 (MID BYTE)                      ");

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

                  case EEPROM_INDEX_LED_COLOR_08_LO_BYTE:
                     {
                        eeprom_value = (byte)(led_list[7] % 0x100);

#ifdef DEBUG_EEPROM_WRITE
                        show_byte_value(eeprom_value);

                        Serial.print(") Color 08 (LO BYTE)                       ");

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

                  case EEPROM_INDEX_LED_COLOR_09_HI_BYTE:
                     {
                        eeprom_value = (byte)(led_list[8] / 0x10000);

#ifdef DEBUG_EEPROM_WRITE
                        show_byte_value(eeprom_value);

                        Serial.print(") Color 09 (HI BYTE)                       ");

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

                  case EEPROM_INDEX_LED_COLOR_09_MID_BYTE:
                     {
                        eeprom_value = ((byte)(led_list[8] / 0x100) % 0x100);

#ifdef DEBUG_EEPROM_WRITE
                        show_byte_value(eeprom_value);

                        Serial.print(") Color 09 (MID BYTE)                      ");

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

                  case EEPROM_INDEX_LED_COLOR_09_LO_BYTE:
                     {
                        eeprom_value = (byte)(led_list[8] % 0x100);

#ifdef DEBUG_EEPROM_WRITE
                        show_byte_value(eeprom_value);

                        Serial.print(") Color 09 (LO BYTE)                       ");

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

                  case EEPROM_INDEX_LED_COLOR_10_HI_BYTE:
                     {
                        eeprom_value = (byte)(led_list[9] / 0x10000);

#ifdef DEBUG_EEPROM_WRITE
                        show_byte_value(eeprom_value);

                        Serial.print(") Color 10 (HI BYTE)                       ");

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

                  case EEPROM_INDEX_LED_COLOR_10_MID_BYTE:
                     {
                        eeprom_value = ((byte)(led_list[9] / 0x100) % 0x100);

#ifdef DEBUG_EEPROM_WRITE
                        show_byte_value(eeprom_value);

                        Serial.print(") Color 10 (MID BYTE)                      ");

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

                  case EEPROM_INDEX_LED_COLOR_10_LO_BYTE:
                     {
                        eeprom_value = (byte)(led_list[9] % 0x100);

#ifdef DEBUG_EEPROM_WRITE
                        show_byte_value(eeprom_value);

                        Serial.print(") Color 10 (LO BYTE)                       ");

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

                  case EEPROM_INDEX_LED_COLOR_11_HI_BYTE:
                     {
                        eeprom_value = (byte)(led_list[10] / 0x10000);

#ifdef DEBUG_EEPROM_WRITE
                        show_byte_value(eeprom_value);

                        Serial.print(") Color 11 (HI BYTE)                       ");

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

                  case EEPROM_INDEX_LED_COLOR_11_MID_BYTE:
                     {
                        eeprom_value = ((byte)(led_list[10] / 0x100) % 0x100);

#ifdef DEBUG_EEPROM_WRITE
                        show_byte_value(eeprom_value);

                        Serial.print(") Color 11 (MID BYTE)                      ");

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

                  case EEPROM_INDEX_LED_COLOR_11_LO_BYTE:
                     {
                        eeprom_value = (byte)(led_list[10] % 0x100);

#ifdef DEBUG_EEPROM_WRITE
                        show_byte_value(eeprom_value);

                        Serial.print(") Color 11 (LO BYTE)                       ");

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

                  case EEPROM_INDEX_LED_COLOR_12_HI_BYTE:
                     {
                        eeprom_value = (byte)(led_list[11] / 0x10000);

#ifdef DEBUG_EEPROM_WRITE
                        show_byte_value(eeprom_value);

                        Serial.print(") Color 12 (HI BYTE)                       ");

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

                  case EEPROM_INDEX_LED_COLOR_12_MID_BYTE:
                     {
                        eeprom_value = ((byte)(led_list[11] / 0x100) % 0x100);

#ifdef DEBUG_EEPROM_WRITE
                        show_byte_value(eeprom_value);

                        Serial.print(") Color 12 (MID BYTE)                      ");

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

                  case EEPROM_INDEX_LED_COLOR_12_LO_BYTE:
                     {
                        eeprom_value = (byte)(led_list[11] % 0x100);

#ifdef DEBUG_EEPROM_WRITE
                        show_byte_value(eeprom_value);

                        Serial.print(") Color 12 (LO BYTE)                       ");

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

                  case EEPROM_INDEX_LED_COLOR_13_HI_BYTE:
                     {
                        eeprom_value = (byte)(led_list[12] / 0x10000);

#ifdef DEBUG_EEPROM_WRITE
                        show_byte_value(eeprom_value);

                        Serial.print(") Color 13 (HI BYTE)                       ");

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

                  case EEPROM_INDEX_LED_COLOR_13_MID_BYTE:
                     {
                        eeprom_value = ((byte)(led_list[12] / 0x100) % 0x100);

#ifdef DEBUG_EEPROM_WRITE
                        show_byte_value(eeprom_value);

                        Serial.print(") Color 13 (MID BYTE)                      ");

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

                  case EEPROM_INDEX_LED_COLOR_13_LO_BYTE:
                     {
                        eeprom_value = (byte)(led_list[12] % 0x100);

#ifdef DEBUG_EEPROM_WRITE
                        show_byte_value(eeprom_value);

                        Serial.print(") Color 13 (LO BYTE)                       ");

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

                  case EEPROM_INDEX_LED_COLOR_14_HI_BYTE:
                     {
                        eeprom_value = (byte)(led_list[13] / 0x10000);

#ifdef DEBUG_EEPROM_WRITE
                        show_byte_value(eeprom_value);

                        Serial.print(") Color 14 (HI BYTE)                       ");

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

                  case EEPROM_INDEX_LED_COLOR_14_MID_BYTE:
                     {
                        eeprom_value = ((byte)(led_list[13] / 0x100) % 0x100);

#ifdef DEBUG_EEPROM_WRITE
                        show_byte_value(eeprom_value);

                        Serial.print(") Color 14 (MID BYTE)                      ");

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

                  case EEPROM_INDEX_LED_COLOR_14_LO_BYTE:
                     {
                        eeprom_value = (byte)(led_list[13] % 0x100);

#ifdef DEBUG_EEPROM_WRITE
                        show_byte_value(eeprom_value);

                        Serial.print(") Color 14 (LO BYTE)                       ");

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

                  case EEPROM_INDEX_LED_COLOR_15_HI_BYTE:
                     {
                        eeprom_value = (byte)(led_list[14] / 0x10000);

#ifdef DEBUG_EEPROM_WRITE
                        show_byte_value(eeprom_value);

                        Serial.print(") Color 15 (HI BYTE)                       ");

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

                  case EEPROM_INDEX_LED_COLOR_15_MID_BYTE:
                     {
                        eeprom_value = ((byte)(led_list[14] / 0x100) % 0x100);

#ifdef DEBUG_EEPROM_WRITE
                        show_byte_value(eeprom_value);

                        Serial.print(") Color 15 (MID BYTE)                      ");

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

                  case EEPROM_INDEX_LED_COLOR_15_LO_BYTE:
                     {
                        eeprom_value = (byte)(led_list[14] % 0x100);

#ifdef DEBUG_EEPROM_WRITE
                        show_byte_value(eeprom_value);

                        Serial.print(") Color 15 (LO BYTE)                       ");

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

                  case EEPROM_INDEX_LED_COLOR_16_HI_BYTE:
                     {
                        eeprom_value = (byte)(led_list[15] / 0x10000);

#ifdef DEBUG_EEPROM_WRITE
                        show_byte_value(eeprom_value);

                        Serial.print(") Color 16 (HI BYTE)                       ");

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

                  case EEPROM_INDEX_LED_COLOR_16_MID_BYTE:
                     {
                        eeprom_value = ((byte)(led_list[15] / 0x100) % 0x100);

#ifdef DEBUG_EEPROM_WRITE
                        show_byte_value(eeprom_value);

                        Serial.print(") Color 16 (MID BYTE)                      ");

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

                  case EEPROM_INDEX_LED_COLOR_16_LO_BYTE:
                     {
                        eeprom_value = (byte)(led_list[15] % 0x100);

#ifdef DEBUG_EEPROM_WRITE
                        show_byte_value(eeprom_value);

                        Serial.print(") Color 16 (LO BYTE)                       ");

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

                  case EEPROM_INDEX_LED_COLOR_17_HI_BYTE:
                     {
                        eeprom_value = (byte)(led_list[16] / 0x10000);

#ifdef DEBUG_EEPROM_WRITE
                        show_byte_value(eeprom_value);

                        Serial.print(") Color 17 (HI BYTE)                       ");

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

                  case EEPROM_INDEX_LED_COLOR_17_MID_BYTE:
                     {
                        eeprom_value = ((byte)(led_list[16] / 0x100) % 0x100);

#ifdef DEBUG_EEPROM_WRITE
                        show_byte_value(eeprom_value);

                        Serial.print(") Color 17 (MID BYTE)                      ");

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

                  case EEPROM_INDEX_LED_COLOR_17_LO_BYTE:
                     {
                        eeprom_value = (byte)(led_list[16] % 0x100);

#ifdef DEBUG_EEPROM_WRITE
                        show_byte_value(eeprom_value);

                        Serial.print(") Color 17 (LO BYTE)                       ");

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

                  case EEPROM_INDEX_LED_COLOR_18_HI_BYTE:
                     {
                        eeprom_value = (byte)(led_list[17] / 0x10000);

#ifdef DEBUG_EEPROM_WRITE
                        show_byte_value(eeprom_value);

                        Serial.print(") Color 18 (HI BYTE)                       ");

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

                  case EEPROM_INDEX_LED_COLOR_18_MID_BYTE:
                     {
                        eeprom_value = ((byte)(led_list[17] / 0x100) % 0x100);

#ifdef DEBUG_EEPROM_WRITE
                        show_byte_value(eeprom_value);

                        Serial.print(") Color 18 (MID BYTE)                      ");

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

                  case EEPROM_INDEX_LED_COLOR_18_LO_BYTE:
                     {
                        eeprom_value = (byte)(led_list[17] % 0x100);

#ifdef DEBUG_EEPROM_WRITE
                        show_byte_value(eeprom_value);

                        Serial.print(") Color 18 (LO BYTE)                       ");

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

                  case EEPROM_INDEX_LED_COLOR_19_HI_BYTE:
                     {
                        eeprom_value = (byte)(led_list[18] / 0x10000);

#ifdef DEBUG_EEPROM_WRITE
                        show_byte_value(eeprom_value);

                        Serial.print(") Color 19 (HI BYTE)                       ");

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

                  case EEPROM_INDEX_LED_COLOR_19_MID_BYTE:
                     {
                        eeprom_value = ((byte)(led_list[18] / 0x100) % 0x100);

#ifdef DEBUG_EEPROM_WRITE
                        show_byte_value(eeprom_value);

                        Serial.print(") Color 19 (MID BYTE)                      ");

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

                  case EEPROM_INDEX_LED_COLOR_19_LO_BYTE:
                     {
                        eeprom_value = (byte)(led_list[18] % 0x100);

#ifdef DEBUG_EEPROM_WRITE
                        show_byte_value(eeprom_value);

                        Serial.print(") Color 19 (LO BYTE)                       ");

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

                  case EEPROM_INDEX_LED_COLOR_20_HI_BYTE:
                     {
                        eeprom_value = (byte)(led_list[19] / 0x10000);

#ifdef DEBUG_EEPROM_WRITE
                        show_byte_value(eeprom_value);

                        Serial.print(") Color 20 (HI BYTE)                       ");

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

                  case EEPROM_INDEX_LED_COLOR_20_MID_BYTE:
                     {
                        eeprom_value = ((byte)(led_list[19] / 0x100) % 0x100);

#ifdef DEBUG_EEPROM_WRITE
                        show_byte_value(eeprom_value);

                        Serial.print(") Color 20 (MID BYTE)                      ");

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

                  case EEPROM_INDEX_LED_COLOR_20_LO_BYTE:
                     {
                        eeprom_value = (byte)(led_list[19] % 0x100);

#ifdef DEBUG_EEPROM_WRITE
                        show_byte_value(eeprom_value);

                        Serial.print(") Color 20 (LO BYTE)                       ");

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

                  case EEPROM_INDEX_LED_COLOR_21_HI_BYTE:
                     {
                        eeprom_value = (byte)(led_list[20] / 0x10000);

#ifdef DEBUG_EEPROM_WRITE
                        show_byte_value(eeprom_value);

                        Serial.print(") Color 21 (HI BYTE)                       ");

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

                  case EEPROM_INDEX_LED_COLOR_21_MID_BYTE:
                     {
                        eeprom_value = ((byte)(led_list[20] / 0x100) % 0x100);

#ifdef DEBUG_EEPROM_WRITE
                        show_byte_value(eeprom_value);

                        Serial.print(") Color 21 (MID BYTE)                      ");

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

                  case EEPROM_INDEX_LED_COLOR_21_LO_BYTE:
                     {
                        eeprom_value = (byte)(led_list[20] % 0x100);

#ifdef DEBUG_EEPROM_WRITE
                        show_byte_value(eeprom_value);

                        Serial.print(") Color 21 (LO BYTE)                       ");

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

                  case EEPROM_INDEX_LED_COLOR_22_HI_BYTE:
                     {
                        eeprom_value = (byte)(led_list[21] / 0x10000);

#ifdef DEBUG_EEPROM_WRITE
                        show_byte_value(eeprom_value);

                        Serial.print(") Color 22 (HI BYTE)                       ");

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

                  case EEPROM_INDEX_LED_COLOR_22_MID_BYTE:
                     {
                        eeprom_value = ((byte)(led_list[21] / 0x100) % 0x100);

#ifdef DEBUG_EEPROM_WRITE
                        show_byte_value(eeprom_value);

                        Serial.print(") Color 22 (MID BYTE)                      ");

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

                  case EEPROM_INDEX_LED_COLOR_22_LO_BYTE:
                     {
                        eeprom_value = (byte)(led_list[21] % 0x100);

#ifdef DEBUG_EEPROM_WRITE
                        show_byte_value(eeprom_value);

                        Serial.print(") Color 22 (LO BYTE)                       ");

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

                  case EEPROM_INDEX_LED_COLOR_23_HI_BYTE:
                     {
                        eeprom_value = (byte)(led_list[22] / 0x10000);

#ifdef DEBUG_EEPROM_WRITE
                        show_byte_value(eeprom_value);

                        Serial.print(") Color 23 (HI BYTE)                       ");

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

                  case EEPROM_INDEX_LED_COLOR_23_MID_BYTE:
                     {
                        eeprom_value = ((byte)(led_list[22] / 0x100) % 0x100);

#ifdef DEBUG_EEPROM_WRITE
                        show_byte_value(eeprom_value);

                        Serial.print(") Color 23 (MID BYTE)                      ");

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

                  case EEPROM_INDEX_LED_COLOR_23_LO_BYTE:
                     {
                        eeprom_value = (byte)(led_list[22] % 0x100);

#ifdef DEBUG_EEPROM_WRITE
                        show_byte_value(eeprom_value);

                        Serial.print(") Color 23 (LO BYTE)                       ");

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

                  case EEPROM_INDEX_LED_COLOR_24_HI_BYTE:
                     {
                        eeprom_value = (byte)(led_list[23] / 0x10000);

#ifdef DEBUG_EEPROM_WRITE
                        show_byte_value(eeprom_value);

                        Serial.print(") Color 24 (HI BYTE)                       ");

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

                  case EEPROM_INDEX_LED_COLOR_24_MID_BYTE:
                     {
                        eeprom_value = ((byte)(led_list[23] / 0x100) % 0x100);

#ifdef DEBUG_EEPROM_WRITE
                        show_byte_value(eeprom_value);

                        Serial.print(") Color 24 (MID BYTE)                      ");

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

                  case EEPROM_INDEX_LED_COLOR_24_LO_BYTE:
                     {
                        eeprom_value = (byte)(led_list[23] % 0x100);

#ifdef DEBUG_EEPROM_WRITE
                        show_byte_value(eeprom_value);

                        Serial.print(") Color 24 (LO BYTE)                       ");

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

                  case EEPROM_INDEX_LED_COLOR_25_HI_BYTE:
                     {
                        eeprom_value = (byte)(led_list[24] / 0x10000);

#ifdef DEBUG_EEPROM_WRITE
                        show_byte_value(eeprom_value);

                        Serial.print(") Color 25 (HI BYTE)                       ");

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

                  case EEPROM_INDEX_LED_COLOR_25_MID_BYTE:
                     {
                        eeprom_value = ((byte)(led_list[24] / 0x100) % 0x100);

#ifdef DEBUG_EEPROM_WRITE
                        show_byte_value(eeprom_value);

                        Serial.print(") Color 25 (MID BYTE)                      ");

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

                  case EEPROM_INDEX_LED_COLOR_25_LO_BYTE:
                     {
                        eeprom_value = (byte)(led_list[24] % 0x100);

#ifdef DEBUG_EEPROM_WRITE
                        show_byte_value(eeprom_value);

                        Serial.print(") Color 25 (LO BYTE)                       ");

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

                  case EEPROM_INDEX_LED_COLOR_26_HI_BYTE:
                     {
                        eeprom_value = (byte)(led_list[25] / 0x10000);

#ifdef DEBUG_EEPROM_WRITE
                        show_byte_value(eeprom_value);

                        Serial.print(") Color 26 (HI BYTE)                       ");

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

                  case EEPROM_INDEX_LED_COLOR_26_MID_BYTE:
                     {
                        eeprom_value = ((byte)(led_list[25] / 0x100) % 0x100);

#ifdef DEBUG_EEPROM_WRITE
                        show_byte_value(eeprom_value);

                        Serial.print(") Color 26 (MID BYTE)                      ");

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

                  case EEPROM_INDEX_LED_COLOR_26_LO_BYTE:
                     {
                        eeprom_value = (byte)(led_list[25] % 0x100);

#ifdef DEBUG_EEPROM_WRITE
                        show_byte_value(eeprom_value);

                        Serial.print(") Color 26 (LO BYTE)                       ");

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

                  case EEPROM_INDEX_LED_COLOR_27_HI_BYTE:
                     {
                        eeprom_value = (byte)(led_list[26] / 0x10000);

#ifdef DEBUG_EEPROM_WRITE
                        show_byte_value(eeprom_value);

                        Serial.print(") Color 27 (HI BYTE)                       ");

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

                  case EEPROM_INDEX_LED_COLOR_27_MID_BYTE:
                     {
                        eeprom_value = ((byte)(led_list[26] / 0x100) % 0x100);

#ifdef DEBUG_EEPROM_WRITE
                        show_byte_value(eeprom_value);

                        Serial.print(") Color 27 (MID BYTE)                      ");

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

                  case EEPROM_INDEX_LED_COLOR_27_LO_BYTE:
                     {
                        eeprom_value = (byte)(led_list[26] % 0x100);

#ifdef DEBUG_EEPROM_WRITE
                        show_byte_value(eeprom_value);

                        Serial.print(") Color 27 (LO BYTE)                       ");

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

                  case EEPROM_INDEX_LED_COLOR_28_HI_BYTE:
                     {
                        eeprom_value = (byte)(led_list[27] / 0x10000);

#ifdef DEBUG_EEPROM_WRITE
                        show_byte_value(eeprom_value);

                        Serial.print(") Color 28 (HI BYTE)                       ");

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

                  case EEPROM_INDEX_LED_COLOR_28_MID_BYTE:
                     {
                        eeprom_value = ((byte)(led_list[27] / 0x100) % 0x100);

#ifdef DEBUG_EEPROM_WRITE
                        show_byte_value(eeprom_value);

                        Serial.print(") Color 28 (MID BYTE)                      ");

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

                  case EEPROM_INDEX_LED_COLOR_28_LO_BYTE:
                     {
                        eeprom_value = (byte)(led_list[27] % 0x100);

#ifdef DEBUG_EEPROM_WRITE
                        show_byte_value(eeprom_value);

                        Serial.print(") Color 28 (LO BYTE)                       ");

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

                  case EEPROM_INDEX_LED_COLOR_29_HI_BYTE:
                     {
                        eeprom_value = (byte)(led_list[28] / 0x10000);

#ifdef DEBUG_EEPROM_WRITE
                        show_byte_value(eeprom_value);

                        Serial.print(") Color 29 (HI BYTE)                       ");

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

                  case EEPROM_INDEX_LED_COLOR_29_MID_BYTE:
                     {
                        eeprom_value = ((byte)(led_list[28] / 0x100) % 0x100);

#ifdef DEBUG_EEPROM_WRITE
                        show_byte_value(eeprom_value);

                        Serial.print(") Color 29 (MID BYTE)                      ");

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

                  case EEPROM_INDEX_LED_COLOR_29_LO_BYTE:
                     {
                        eeprom_value = (byte)(led_list[28] % 0x100);

#ifdef DEBUG_EEPROM_WRITE
                        show_byte_value(eeprom_value);

                        Serial.print(") Color 29 (LO BYTE)                       ");

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

                  case EEPROM_INDEX_LED_COLOR_30_HI_BYTE:
                     {
                        eeprom_value = (byte)(led_list[29] / 0x10000);

#ifdef DEBUG_EEPROM_WRITE
                        show_byte_value(eeprom_value);

                        Serial.print(") Color 30 (HI BYTE)                       ");

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

                  case EEPROM_INDEX_LED_COLOR_30_MID_BYTE:
                     {
                        eeprom_value = ((byte)(led_list[29] / 0x100) % 0x100);

#ifdef DEBUG_EEPROM_WRITE
                        show_byte_value(eeprom_value);

                        Serial.print(") Color 30 (MID BYTE)                      ");

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

                  case EEPROM_INDEX_LED_COLOR_30_LO_BYTE:
                     {
                        eeprom_value = (byte)(led_list[29] % 0x100);

#ifdef DEBUG_EEPROM_WRITE
                        show_byte_value(eeprom_value);

                        Serial.print(") Color 30 (LO BYTE)                       ");

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

                  case EEPROM_INDEX_LED_COLOR_31_HI_BYTE:
                     {
                        eeprom_value = (byte)(led_list[30] / 0x10000);

#ifdef DEBUG_EEPROM_WRITE
                        show_byte_value(eeprom_value);

                        Serial.print(") Color 31 (HI BYTE)                       ");

                        Serial.print(" 0x");
                        if (eeprom_value < 16)
                        {
                           Serial.print("0");
                        }
                        Serial.print(eeprom_value, HEX);

                        Serial.print(" = ");
                        if (eeprom_value < 100

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

            case EEPROM_INDEX_LED_COLOR_31_MID_BYTE:
               {
                  eeprom_value = ((byte)(led_list[30] / 0x100) % 0x100);

#ifdef DEBUG_EEPROM_WRITE
                  show_byte_value(eeprom_value);

                  Serial.print(") Color 31 (MID BYTE)                      ");

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

            case EEPROM_INDEX_LED_COLOR_31_LO_BYTE:
               {
                  eeprom_value = (byte)(led_list[30] % 0x100);

#ifdef DEBUG_EEPROM_WRITE
                  show_byte_value(eeprom_value);

                  Serial.print(") Color 31 (LO BYTE)                       ");

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

            case EEPROM_INDEX_LED_COLOR_32_HI_BYTE:
               {
                  eeprom_value = (byte)(led_list[31] / 0x10000);

#ifdef DEBUG_EEPROM_WRITE
                  show_byte_value(eeprom_value);

                  Serial.print(") Color 32 (HI BYTE)                       ");

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

            case EEPROM_INDEX_LED_COLOR_32_MID_BYTE:
               {
                  eeprom_value = ((byte)(led_list[31] / 0x100) % 0x100);

#ifdef DEBUG_EEPROM_WRITE
                  show_byte_value(eeprom_value);

                  Serial.print(") Color 32 (MID BYTE)                      ");

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

            case EEPROM_INDEX_LED_COLOR_32_LO_BYTE:
               {
                  eeprom_value = (byte)(led_list[31] % 0x100);

#ifdef DEBUG_EEPROM_WRITE
                  show_byte_value(eeprom_value);

                  Serial.print(") Color 32 (LO BYTE)                       ");

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


            case EEPROM_INDEX_LED_CHECKSUM:
               {
                  eeprom_value = (char)(xor_result);

#ifdef DEBUG_EEPROM_WRITE
                  show_byte_value(eeprom_value);
                  Serial.print(") Calculated CHECKSUM                            = ");
                  show_byte_value(xor_result);
                  Serial.println("");
#endif
               }
               break;

            case EEPROM_INDEX_LED_INV_CHECKSUM:
               {
                  eeprom_value = (char)(~xor_result);

#ifdef DEBUG_EEPROM_WRITE
                  show_byte_value(eeprom_value);
                  Serial.print(") Calculated INVERSE CHECKSUM                    = ");
                  show_byte_value((byte)~xor_result);
                  Serial.println("");
#endif
               }
               break;
            }

#ifndef DISABLE_EEPROM_WRITE_SETTINGS
            EEPROM.update((int)(eeprom_index + (pattern_number * EEPROM_INDEX_LED_VALUE_COUNT)), (byte)(eeprom_value));
#endif
            if (eeprom_index < EEPROM_INDEX_LED_CHECKSUM)
            {
               xor_result = (byte)(xor_result ^ eeprom_value);
            }
         }
   }
   break;
}
}  // save_settings()


// one-time setup
void setup(void)
{
   // Start serial
   Serial.begin(115200);

   delay(500);
   tft.begin();
   tft.setRotation(0);

   delay(100);
   ts.begin();
   ts.setRotation(2);

   pinMode(CHECK_BATTERY_PIN, INPUT);
   pinMode(FORCE_LED_COLOR_MAPPER_MODE, INPUT_PULLUP);

   delay(100);

   if (!(digitalRead(FORCE_LED_COLOR_MAPPER_MODE)))
   {
      ops_mode = OPS_MODE_TYPE_LED_COLOR_MAPPER;
   } else {
      ops_mode = OPS_MODE_TYPE_RV_LEVELING;
   }

   change_operating_mode();

   Serial.println("");
   Serial.print("EEPROM USED: ");
   Serial.println(EEPROM_INDEX_LED_INV_BRIGHT_LEVEL + 1);  // +1 to account for 0-indexed count
   Serial.print("EEPROM MAX ALLOWED: ");
   Serial.println(MAX_T4X_EEPROM_SIZE_ALLOWED);
   Serial.println("");
}  // setup()


// send the BLUE value to any paired units
void share_blue_value(void)
{
   char send_update_buffer[5];

   send_update_buffer[0] = 'B';
   send_update_buffer[1] = (blue_level / 100) + '0';
   send_update_buffer[2] = (blue_level % 100) / 10 + '0';
   send_update_buffer[3] = (blue_level % 10) + '0';
   send_update_buffer[4] = 0x00;

   radio_send(send_update_buffer);
}  // share_BLUE_value()


// send the GREEN value to any paired units
void share_green_value(void)
{
   char send_update_buffer[5];

   send_update_buffer[0] = 'G';
   send_update_buffer[1] = (green_level / 100) + '0';
   send_update_buffer[2] = (green_level % 100) / 10 + '0';
   send_update_buffer[3] = (green_level % 10) + '0';
   send_update_buffer[4] = 0x00;

   radio_send(send_update_buffer);
}  // share_green_value()


// send LED status to any paired units
void share_leds(void)
{
   char send_update_buffer[12];

   send_update_buffer[0] = 'L';
   send_update_buffer[3] = '0';
   send_update_buffer[4] = 'x';

   for (int i = 0; i < PATTERN_LENGTH_MAX; i++)
   {
      send_update_buffer[1] = ((i + 1) / 10) + '0';
      send_update_buffer[2] = ((i + 1) % 10) + '0';

      if ((led_list[i] / 0x100000) >= 10)
      {
         send_update_buffer[5] = ((led_list[i] / 0x100000) - 10 + 'A');
      } else {
         send_update_buffer[5] = ((led_list[i] / 0x100000) + '0');
      }

      if (((led_list[i] / 0x10000) % 0x10) >= 10)
      {
         send_update_buffer[6] = (((led_list[i] / 0x10000) % 0x10) - 10 + 'A');
      } else {
         send_update_buffer[6] = (((led_list[i] / 0x10000) % 0x10) + '0');
      }

      if (((led_list[i] / 0x1000) % 0x10) >= 10)
      {
         send_update_buffer[7] = (((led_list[i] / 0x1000) % 0x10) - 10 + 'A');
      } else {
         send_update_buffer[7] = (((led_list[i] / 0x1000) % 0x10) + '0');
      }

      if (((led_list[i] / 0x100) % 0x10) >= 10)
      {
         send_update_buffer[8] = (((led_list[i] / 0x100) % 0x10) - 10 + 'A');
      } else {
         send_update_buffer[8] = (((led_list[i] / 0x100) % 0x10) + '0');
      }

      if (((led_list[i] / 0x10) % 0x10) >= 10)
      {
         send_update_buffer[9] = (((led_list[i] / 0x10) % 0x10) - 10 + 'A');
      } else {
         send_update_buffer[9] = (((led_list[i] / 0x10) % 0x10) + '0');
      }

      if ((led_list[i] % 0x10) >= 10)
      {
         send_update_buffer[10] = ((led_list[i] % 0x10) - 10 + 'A');
      } else {
         send_update_buffer[10] = ((led_list[i] % 0x10) + '0');
      }

      send_update_buffer[11] = 0x00;

      radio_send(send_update_buffer);

      delay(20);
   }
}  // share_leds()


// send the led on button to any paired units
void share_leds_on_button(void)
{
   char send_update_buffer[4];

   send_update_buffer[0] = 'L';
   send_update_buffer[1] = 'O';

   send_update_buffer[2] = (int)(leds_on) + '0';

   send_update_buffer[3] = 0x00;

   radio_send(send_update_buffer);
}  // share_leds_on_button()


// send the pattern brightness to any paired units
void share_bright_level(void)
{
   char send_update_buffer[5];

   send_update_buffer[0] = 'B';
   send_update_buffer[1] = 'L';

   send_update_buffer[2] = (bright_level / 10) + '0';
   send_update_buffer[3] = (bright_level % 10) + '0';

   send_update_buffer[4] = 0x00;

   radio_send(send_update_buffer);
}  // share_bright_level()


// send the pattern length to any paired units
void share_pattern_length(void)
{
   char send_update_buffer[5];

   send_update_buffer[0] = 'P';
   send_update_buffer[1] = 'L';

   send_update_buffer[2] = (pattern_length / 10) + '0';
   send_update_buffer[3] = (pattern_length % 10) + '0';
   send_update_buffer[4] = 0x00;

   radio_send(send_update_buffer);
}  // share_pattern_length()


// send the pattern number to any paired units
void share_pattern_number(void)
{
   char send_update_buffer[5];

   send_update_buffer[0] = 'P';
   send_update_buffer[1] = 'N';

   send_update_buffer[2] = ((pattern_number + 1) / 10) + '0';
   send_update_buffer[3] = ((pattern_number + 1) % 10) + '0';
   send_update_buffer[4] = 0x00;

   radio_send(send_update_buffer);
}  // share_pattern_number()


// send the pattern speed to any paired units
void share_pattern_speed(void)
{
   char send_update_buffer[5];

   send_update_buffer[0] = 'P';
   send_update_buffer[1] = 'S';

   send_update_buffer[2] = (pattern_speed / 10) + '0';
   send_update_buffer[3] = (pattern_speed % 10) + '0';

   send_update_buffer[4] = 0x00;

   radio_send(send_update_buffer);
}  // share_pattern_speed()


// send the pattern type to any paired units
void share_pattern_type(void)
{
   char send_update_buffer[5];

   send_update_buffer[0] = 'P';
   send_update_buffer[1] = 'T';

   send_update_buffer[2] = ((int)pattern_type / 10) + '0';
   send_update_buffer[3] = ((int)pattern_type % 10) + '0';

   send_update_buffer[4] = 0x00;

   radio_send(send_update_buffer);
}  // share_pattern_type()


// send the RED value to any paired units
void share_red_value(void)
{
   char send_update_buffer[5];

   send_update_buffer[0] = 'R';
   send_update_buffer[1] = (red_level / 100) + '0';
   send_update_buffer[2] = (red_level % 100) / 10 + '0';
   send_update_buffer[3] = (red_level % 10) + '0';
   send_update_buffer[4] = 0x00;

   radio_send(send_update_buffer);
}  // share_red_value()


// display the charge status of the battery
void show_battery_status(void)
{
   if (digitalReadFast(CHECK_BATTERY_PIN))
   {
      //      if ((previous_battery_state) || (initial_battery_update_required))
      {
         initial_battery_update_required = false;

         previous_battery_state = false;

         switch (ops_mode)
         {
            case OPS_MODE_TYPE_RV_LEVELING:
               {
                  tft.fillRect(201, 0, 39, 35, ILI9341_GREEN);
                  tft.drawRect(203, 2, 35, 31, ILI9341_BLACK);

                  center_draw_text("BATT", 1, 221, 13, ILI9341_BLACK, ILI9341_GREEN);
                  center_draw_text("OK", 1, 221, 24, ILI9341_BLACK, ILI9341_GREEN);
               }
               break;

            case OPS_MODE_TYPE_LED_COLOR_MAPPER:
               {
                  tft.fillRect(251, 0, 39, 35, ILI9341_GREEN);
                  tft.drawRect(253, 2, 35, 31, ILI9341_BLACK);

                  center_draw_text("BATT", 1, 271, 13, ILI9341_BLACK, ILI9341_GREEN);
                  center_draw_text("OK", 1, 271, 24, ILI9341_BLACK, ILI9341_GREEN);
               }
               break;
         }
      }
   } else {
      initial_battery_update_required = false;

      previous_battery_state = true;

      battery_low_toggle = !battery_low_toggle;

      if (battery_low_toggle)
      {
         switch (ops_mode)
         {
            case OPS_MODE_TYPE_RV_LEVELING:
               {
                  tft.fillRect(201, 0, 39, 35, ILI9341_RED);
                  tft.drawRect(203, 2, 35, 31, ILI9341_BLACK);

                  center_draw_text("BATT", 1, 221, 13, ILI9341_BLACK, ILI9341_RED);
                  center_draw_text("LOW", 1, 221, 24, ILI9341_BLACK, ILI9341_RED);
               }
               break;

            case OPS_MODE_TYPE_LED_COLOR_MAPPER:
               {
                  tft.fillRect(251, 0, 39, 35, ILI9341_RED);
                  tft.drawRect(253, 2, 35, 31, ILI9341_BLACK);

                  center_draw_text("BATT", 1, 271, 13, ILI9341_BLACK, ILI9341_RED);
                  center_draw_text("LOW", 1, 271, 24, ILI9341_BLACK, ILI9341_RED);
               }
               break;
         }
      } else {
         switch (ops_mode)
         {
            case OPS_MODE_TYPE_RV_LEVELING:
               {
                  tft.fillRect(201, 0, 39, 35, ILI9341_BLACK);
                  tft.drawRect(203, 2, 35, 31, ILI9341_RED);

                  center_draw_text("BATT", 1, 221, 13, ILI9341_RED, ILI9341_BLACK);
                  center_draw_text("LOW", 1, 221, 24, ILI9341_RED, ILI9341_BLACK);
               }
               break;

            case OPS_MODE_TYPE_LED_COLOR_MAPPER:
               {
                  tft.fillRect(251, 0, 39, 35, ILI9341_BLACK);
                  tft.drawRect(253, 2, 35, 31, ILI9341_RED);

                  center_draw_text("BATT", 1, 271, 13, ILI9341_RED, ILI9341_BLACK);
                  center_draw_text("LOW", 1, 271, 24, ILI9341_RED, ILI9341_BLACK);
               }
               break;
         }
      }
   }
}  // show_battery_status();


// send the results to the serial console
void show_console_results()
{
#ifdef DEBUG_RESULTS
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
#endif
}  // show_console_results()


// show index of EEPROM reads/writes
void show_byte_value(unsigned char byte_value)
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
}  // show_byte_value()


// show index of EEPROM reads/writes
void show_index(int index)
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
}  // show_index()


// update the selected pattern brightness
void update_bright_level(void)
{
   tft.fillRect(102, 114, 18, 9, ILI9341_BLACK);

   tft.setTextSize(1);
   tft.setTextColor(ILI9341_WHITE, ILI9341_BLACK);
   tft.setCursor(108, 115);

   tft.print(bright_level);
}  // update_bright_level()


// update the radio channel number in LED channel mapper mode
void update_led_radio_channel(void)
{
   tft.fillRect(102, 99, 24, 9, ILI9341_BLACK);

   tft.setTextSize(1);
   tft.setTextColor(ILI9341_WHITE, ILI9341_BLACK);
   tft.setCursor(108, 100);

   if (!(radio_not_present))
   {
      tft.print(radio_channel);
   } else {
      tft.print("NOT PRESENT");
   }
}  // update_led_radio_channel()


// update the radio power level in LED channel mapper mode
void update_led_radio_power(void)
{
   tft.fillRect(102, 84, 30, 9, ILI9341_BLACK);

   tft.setTextSize(1);
   tft.setTextColor(ILI9341_WHITE, ILI9341_BLACK);
   tft.setCursor(108, 85);

   if (!(radio_not_present))
   {
      switch (radio_power_level)
      {
         case RADIO_POWER_LEVEL_LOW:
            {
               tft.print("LOW");
            }
            break;

         case RADIO_POWER_LEVEL_MID:
            {
               tft.print("MID");
            }
            break;

         case RADIO_POWER_LEVEL_HIGH:
            {
               tft.print("HIGH");
            }
            break;
      }
   } else {
      tft.print("NOT PRESENT");
   }
}  // update_led_radio_power()


// update LED changes
void update_leds(void)
{
   for (int i = 0; i < pattern_length; i++)
   {
      if (i < (PATTERN_LENGTH_MAX / 2))
      {
         tft.fillCircle(10 + (i * 20), 197, 8, ILI9341_WHITE);
         tft.fillCircle(10 + (i * 20), 197, 7, ILI9341_BLACK);
         tft.fillCircle(10 + (i * 20), 197, 6, ((map((led_list[i] / 0x10000), 0, 255, 0, 31)) * 2048) + ((map(((led_list[i] / 0x100) % 0x100), 0, 255, 0, 63)) * 32) + map((led_list[i] % 0x100), 0, 255, 0, 31));
      } else {
         tft.fillCircle(10 + ((i - (PATTERN_LENGTH_MAX / 2)) * 20), 217, 8, ILI9341_WHITE);
         tft.fillCircle(10 + ((i - (PATTERN_LENGTH_MAX / 2)) * 20), 217, 7, ILI9341_BLACK);
         tft.fillCircle(10 + ((i - (PATTERN_LENGTH_MAX / 2)) * 20), 217, 6, ((map((led_list[i] / 0x10000), 0, 255, 0, 31)) * 2048) + ((map(((led_list[i] / 0x100) % 0x100), 0, 255, 0, 63)) * 32) + map((led_list[i] % 0x100), 0, 255, 0, 31));
      }
   }

   if ((pattern_length < PATTERN_LENGTH_MAX) && (pattern_length >= 1))
   {
      for (int i = pattern_length; i < PATTERN_LENGTH_MAX; i++)
      {
         if (i < (PATTERN_LENGTH_MAX / 2))
         {
            tft.fillCircle(10 + (i * 20), 197, 8, ILI9341_BLACK);
            tft.drawCircle(10 + (i * 20), 197, 8, MY_ILI9341_GRAY);
         } else {
            tft.fillCircle(10 + ((i - (PATTERN_LENGTH_MAX / 2)) * 20), 217, 8, ILI9341_BLACK);
            tft.drawCircle(10 + ((i - (PATTERN_LENGTH_MAX / 2)) * 20), 217, 8, MY_ILI9341_GRAY);
         }
      }
   }
}  // update_leds()


// update the LEDs ON/OFF button
void update_leds_on_button(void)
{
   if (leds_on)
   {
      tft.fillRect(225, 40, 90, 25, ILI9341_GREEN);
      center_draw_text("LEDS ON", 1, 270, 53, ILI9341_BLACK, ILI9341_GREEN);
   } else {
      tft.fillRect(225, 40, 90, 25, ILI9341_RED);
      center_draw_text("LEDS OFF", 1, 270, 53, ILI9341_BLACK, ILI9341_RED);
   }
   tft.drawRect(227, 42, 86, 21, ILI9341_BLACK);
}  // update_leds_on_button()


// update the length of the pattern
void update_pattern_length(void)
{
   tft.fillRect(102, 174, 18, 9, ILI9341_BLACK);

   tft.setTextSize(1);
   tft.setTextColor(ILI9341_WHITE, ILI9341_BLACK);
   tft.setCursor(108, 175);

   tft.print(pattern_length);
}  // update_pattern_length()


// update the selected pattern number
void update_pattern_number(void)
{
   tft.fillRect(102, 129, 18, 9, ILI9341_BLACK);

   tft.setTextSize(1);
   tft.setTextColor(ILI9341_WHITE, ILI9341_BLACK);
   tft.setCursor(108, 130);

   tft.print(pattern_number + 1);
}  // update_pattern_number()


// update the speed of the pattern
void update_pattern_speed(void)
{
   tft.fillRect(102, 159, 18, 9, ILI9341_BLACK);

   tft.setTextSize(1);
   tft.setTextColor(ILI9341_WHITE, ILI9341_BLACK);
   tft.setCursor(108, 160);

   tft.print(pattern_speed);
}  // update_pattern_speed()


// update the selected pattern type
void update_pattern_type(void)
{
   tft.fillRect(100, 144, 80, 10, ILI9341_BLACK);

   tft.setTextSize(1);
   tft.setTextColor(ILI9341_WHITE, ILI9341_BLACK);
   tft.setCursor(108, 145);

   switch (pattern_type)
   {
      case PATTERN_TYPE_MARQUIS_FORWARD:
         {
            tft.print("MARQUIS-FWD");
         }
         break;

      case PATTERN_TYPE_MARQUIS_REVERSE:
         {
            tft.print("MARQUIS-REV");
         }
         break;

      case PATTERN_TYPE_CYCLE_FORWARD:
         {
            tft.print("CYCLE-FWD");
         }
         break;

      case PATTERN_TYPE_CYCLE_REVERSE:
         {
            tft.print("CYCLE-REV");
         }
         break;

      case PATTERN_TYPE_FADE_FORWARD:
         {
            tft.print("FADE-FWD");
         }
         break;

      case PATTERN_TYPE_FADE_REVERSE:
         {
            tft.print("FADE-REV");
         }
         break;

      case PATTERN_TYPE_CROSSFADE_FORWARD:
         {
            tft.print("XFADE-FWD");
         }
         break;

      case PATTERN_TYPE_CROSSFADE_REVERSE:
         {
            tft.print("XFADE-REV");
         }
         break;

      case PATTERN_TYPE_ZIPPER_FORWARD:
         {
            tft.print("ZIPPER-FWD");
         }
         break;

      case PATTERN_TYPE_ZIPPER_REVERSE:
         {
            tft.print("ZIPPER-REV");
         }
         break;

      case PATTERN_TYPE_RANDOM:
         {
            tft.print("RANDOM");
         }
         break;
   }
}  // update_pattern_type()


// update the radio channel number on the screen
void update_radio_channel(void)
{
   switch (ops_mode)
   {
      case OPS_MODE_TYPE_RV_LEVELING:
         {
            update_rv_radio_channel();
         }
         break;

      case OPS_MODE_TYPE_LED_COLOR_MAPPER:
         {
            update_led_radio_channel();
         }
         break;
   }
}  // update_radio_channel()


// update the radio power level on the screen
void update_radio_power(void)
{
   switch (ops_mode)
   {
      case OPS_MODE_TYPE_RV_LEVELING:
         {
            update_rv_radio_power();
         }
         break;

      case OPS_MODE_TYPE_LED_COLOR_MAPPER:
         {
            update_led_radio_power();
         }
         break;
   }
}  // update_radio_power()


// update the read/set LED button
void update_read_led_button(void)
{
   if (read_led)
   {
      tft.fillRect(225, 70, 90, 25, ILI9341_ORANGE);
      center_draw_text("READ LED", 1, 270, 83, ILI9341_BLACK, ILI9341_ORANGE);
   } else {
      tft.fillRect(225, 70, 90, 25, ILI9341_YELLOW);
      center_draw_text("SET LED", 1, 270, 83, ILI9341_BLACK, ILI9341_YELLOW);
   }
   tft.drawRect(227, 72, 86, 21, ILI9341_BLACK);
}  // update_read_led_button()


// update the radio channel number in RV leveling mode
void update_rv_radio_channel(void)
{
   String chan = "";

   if (radio_channel >= 100)
   {
      chan = "1";
   }

   if ((chan.length()) || ((radio_channel / 10) % 10))
   {
      chan += (char)(((radio_channel / 10) % 10) + '0');
   }

   chan += (char)((radio_channel % 10) + '0');

   tft.fillRect(75, 42, 50, 10, ILI9341_WHITE);

   if (!(radio_not_present))
   {
      tft.fillCircle(79, 48, 7, ILI9341_RED);
      tft.drawCircle(79, 48, 7, ILI9341_BLACK);
      center_draw_text("-", 1, 79, 50, ILI9341_BLACK, ILI9341_RED);

      tft.fillCircle(115, 48, 7, ILI9341_GREEN);
      tft.drawCircle(115, 48, 7, ILI9341_BLACK);
      center_draw_text("+", 1, 115, 49, ILI9341_BLACK, ILI9341_GREEN);

      center_draw_text(chan, 1, 97, 49, ILI9341_BLACK, ILI9341_WHITE);
   } else {
      center_draw_text("(NONE)", 1, 90, 49, ILI9341_BLACK, ILI9341_WHITE);
   }
}  // update_rv_radio_channel()


// update the radio power_level in RV leveling mode
void update_rv_radio_power(void)
{
   tft.fillRect(75, 57, 50, 10, ILI9341_WHITE);

   if (!(radio_not_present))
   {
      tft.fillCircle(79, 63, 7, ILI9341_RED);
      tft.drawCircle(79, 63, 7, ILI9341_BLACK);
      center_draw_text("-", 1, 79, 65, ILI9341_BLACK, ILI9341_RED);

      tft.fillCircle(115, 63, 7, ILI9341_GREEN);
      tft.drawCircle(115, 63, 7, ILI9341_BLACK);
      center_draw_text("+", 1, 115, 64, ILI9341_BLACK, ILI9341_GREEN);

      switch (radio_power_level)
      {
         case RADIO_POWER_LEVEL_LOW:
            {
               center_draw_text("LOW", 1, 97, 64, ILI9341_BLACK, ILI9341_WHITE);
            }
            break;

         case RADIO_POWER_LEVEL_MID:
            {
               center_draw_text("MID", 1, 97, 64, ILI9341_BLACK, ILI9341_WHITE);
            }
            break;

         case RADIO_POWER_LEVEL_HIGH:
            {
               center_draw_text("HI", 1, 97, 64, ILI9341_BLACK, ILI9341_WHITE);
            }
            break;
      }
   } else {
      center_draw_text("(NONE)", 1, 90, 64, ILI9341_BLACK, ILI9341_WHITE);
   }
}  // update_rv_radio_power()


// update settings changes
void update_settings(void)
{
   // display_color
   // RED: 0-31
   // GRN: 0-63
   // BLU: 0-31
   display_color = ((map(red_level, 0, 255, 0, 31)) * 2048) + ((map(green_level, 0, 255, 0, 63)) * 32) + map(blue_level, 0, 255, 0, 31);
   led_strip_color = (red_level * 0x10000) + (green_level * 0x100) + blue_level;

   tft.fillRect(140, 35, 60, 30, display_color);

   tft.setCursor(146, 47);
   tft.setTextSize(1);

   tft.setTextColor(0xFFFF - display_color);

   tft.print("0x");

   if (led_strip_color < 0x100000)
   {
      tft.print("0");
   }
   if (led_strip_color < 0x10000)
   {
      tft.print("0");
   }
   if (led_strip_color < 0x1000)
   {
      tft.print("0");
   }
   if (led_strip_color < 0x100)
   {
      tft.print("0");
   }
   if (led_strip_color < 0x10)
   {
      tft.print("0");
   }
   tft.print(led_strip_color, HEX);
}  // update_settings()


// update a color value
void update_value(unsigned int currentValue, unsigned int xCenter, unsigned int yCenter, unsigned int textColor)
{
   String valueText = "";

   if (currentValue >= 1000)
   {
      if (currentValue / 1000)
      {
         valueText += (currentValue / 1000);
      }
   }

   if (currentValue >= 100)
   {
      if ((currentValue % 1000) / 100)
      {
         valueText += ((currentValue % 1000) / 100);
      } else {
         if (valueText.length())
         {
            valueText += '0';
         }
      }
   }

   if (currentValue >= 10)
   {
      if ((currentValue % 100) / 10)
      {
         valueText += ((currentValue % 100) / 10);
      } else {
         if (valueText.length())
         {
            valueText += '0';
         }
      }
   }

   valueText += (currentValue % 10);

   tft.fillRect(xCenter - 20, yCenter - 8, 40, 16, ILI9341_BLACK);

   center_draw_text(valueText, 2, xCenter, yCenter, textColor, ILI9341_BLACK);

   update_settings();
}  // update_value()


// EOF PLACEHOLDER
