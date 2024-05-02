# LILYGO_TDISPLAY_TOUCH_DRIVER_MICROPYTHON
Micropython Touch driver library for Lilygo T-Display ESP32-S3 AMOLED


This library allow to use the Lilygo ESP32 S3 T-Display Amoled Touch capacities with Micropython


object CST8X can be called 

  cst = cst8x.CST8X(i2c=i2cbus) the easy way

  or

  cst = cst8x.CST8X(i2c=i2cbus, adress = Chip adress, int_pin=TS_INT, int_handler = TS_pressed, witdh, height, xmin, xmax, ymin, ymax, debug=False)


where

  i2cbus is mandatory !
  adress is optionnal, the library will look for the chip on the i2c bus
  int_pin to handle interruptions
  int_handler to handle a subroutine in your main program
  width = screen width
  height = screen height
  xmin/max and ymin/max are value you might adjust, however the library is autofocusing after every touch
  debug=True if you wan't console outputs.

  
