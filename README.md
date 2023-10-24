# Teensy-RV-Leveling-Helper
Teensy RV Leveling Helper using Teensy 4.0 + BMA400 Triple Axis Accelerometer Breakout + ILI9341 Color 320x240 TFT Touchscreen display

![TRLH - RV mode main screen](https://github.com/mjculross/Teensy-RV-Leveling-Helper/assets/4277910/6b16b0c0-ed02-4666-bd05-db4aa8ab182b)

This project came about after we acquired a used RV motorhome & I quickly grew tired of the following process of attempting to level the RV:
   1) park the RV on a particular site
   2) look at the bubble level on the kitchen counter to determine "level or not"
   3) make/revise my best guess as to how many blocks are required on each wheel to attain "level"
   4) drive the RV up on the best guess number of blocks
   5) if the RV is "level", then you're DONE !!
   6) if the RV is not yet "level", then drive the RV down off of the best guess number of blocks
   7) goto 3) above & repeat several times (until either we're close enough, or I get fed up, whichever comes first)

This project incorporates the following:

   HARDWARE:

      Teensy 4.0
         available from PJRC.com https://www.pjrc.com/teensy-4-0/

      ILI9341 Color 320x240 TFT Touchscreen display
         available from PJRC.com https://www.pjrc.com/store/display_ili9341_touch.html

      BMA400 Triple Axis Accelerometer Breakout
         available from Sparkfun https://www.sparkfun.com/products/21208

   This project uses touchscreen input & color display to create a visual (graphical) indication of how many inches of leveling blocks are required to be put under each wheel of an RV to level it
  
   This project uses the ILI9341_t3 library, which has been optimized for use with the Teensy TFT display, as well as the XPT2046_Touchscreen.h touchscreen library for that same display


   This project allows & stores configurable wheel base (front to back wheels distance) from 60" to 255".

   This project allows & stores configurable wheel distance (front wheels, left to right distance) from 60" to 84".


   Internet reference (with equation derivation) for transforming accelerometer x,y,z vector data into pitch & roll angles:

      https://mwrona.com/posts/accel-roll-pitch/

   The specific DPDT switch that I use for power control (& for which the 3D printed case is designed) is the Carling Technologies 62115929-0-0-V, available from digikey as part number 432-1252-ND (https://www.digikey.com/en/products/detail/carling-technologies/62115929-0-0-V/3025077).

   An Adafruit microUSB panel mount extension cable Product ID 3258 (https://www.adafruit.com/product/3258) is mounted on the side wall of the case.  The cable is split such that the USB 5VDC line connects to the USB input pin on the Adafruit PowerBoost 1000C to provide power & battery charging, while the remaining GROUND, D-, & D+ signals connect to the microUSB connector on the Teensy to allow (re)programming without opening the case.

   This project implements the following:

      1) read the XYZ vector from the BMA400 triple axis accelerometer breakout
      2) use the XYZ vector to calculate PITCH and ROLL values for the current plane of the RV/trailer orientation
      3) use the calculated PITCH and ROLL values to determine the highest wheel/point (which will not require any blocks)
      4) use the calculated PITCH and ROLL values to determine the number of inches of blocks required for the remaining wheels/points to achieve "level"
      5) update the display with the current results once per second (& average all values otherwise)
      6) repeat

OPERATIONAL MODES:

   This project can be switched between the following two primary modes of operation:
   - RV mode (four points of balance/leveling = four wheels)
   - Trailer mode (three points of balance/leveling = two wheels and the hitch)


MECHANICAL ASSEMBLY & CALIBRATION:

   For information on mechanical assembly and/or calibration, see the "readme" file in the "photos" folder.


CONTACT INFO:

If you have any questions, suggestions for improvement, and most especially any bug reports, please contact me at kd5rxt@arrl.net
