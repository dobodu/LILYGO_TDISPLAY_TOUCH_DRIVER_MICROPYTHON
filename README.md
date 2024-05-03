**LILYGO T-DISPLAY TOUCH DRIVER MICROPYTHON**

Micropython Touch driver library for Lilygo T-Display ESP32-S3 AMOLED


------------

Import the library and create the touch screen object

The easy way :

	import cst8x
	cst = cst8x.CST8X(i2c)

or the complete way

	import cst8x
	cst = cst8x.CST8X(i2c, adress, int_pin, int_handler, witdh, height, xmin, xmax, ymin, ymax, debug)

all parameter can be written like

	i2c = i2c bus you must set up before
	adress = adress of the chip on I2C
	int_pin = interruption GPIO pin
	int_handler = function handler when interruption pin raise up
	witdh, height = dimension of the display to get absolute values
	xmin, xmax, ymin, ymax = raw data min and max values for touch screen
	debug = if you want to have console outputs


------------

Mandatory parameters :

- i2cbus is the only mandatory parameter

Optional parameters

- adress : the library will look for the chip on the i2c bus
- int_pin : if no pin, no interruption
- int_handler : if no handler : no subroutine will be called on interruption
- width and height : defaut is 536x240
- xmin/max and ymin/max

Behaviour

- Library will autofocus min/max x and y values on every touch event.
