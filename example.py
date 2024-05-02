from machine import Pin, I2C,
from utime import sleep_ms

#I2C BUS
I2C0_SCL = Pin(02)
I2C0_SDA = Pin(03)
TS_INT = Pin(21, Pin.IN, Pin.PULL_UP)

TFT_WIDTH = 240
TFT_HEIGHT = 536

def TS_pressed(x,y):
    print("Touche ",x,y)

i2c = I2C(0, scl=I2C0_SCL, sda=I2C0_SDA, freq=100000, timeout=200000 )
    
cst = cst8x.CST8X(i2c=i2c, int_pin=TS_INT, int_handler = TS_pressed, debug=False)

while True:
    sleep_ms(10)
