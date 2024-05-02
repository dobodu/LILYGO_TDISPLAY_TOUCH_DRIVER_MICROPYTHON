"""Micropython Driver for LILYGO ESP32 S3 DISPLAY TOUCH DRIVER
CAN ALSO SUPPORT MANY CST8X CHIPSET but need to be tested

To do :

/!\ Functions are implemented but not tested

Check Motion Mask functionnality
Check Irq Control functionality
Return multiples fingers positions
Return motion gesture
"""

from utime import sleep_ms
from ustruct import unpack_from

#CST SETUP
CST_DEFAULT_ADDRESS = (0x15, 0x0D)
CST_GESTURE_ID = 0x01
CST_FINGER_NUM = 0x02	#0, 1 or 2 fingers
CST_F1_XPOS = 0x03	#Finger 1
CST_F1_YPOS = 0x05	#[11:0] : 11:8 High 7:0 Low
CST_F1_WEIGHT = 0x07
CST_F1_AREA = 0x08
CST_F2_XPOS = 0x09	#Finger 2 offset 6 bytes away from Finger 1
CST_F2_YPOS = 0x11
CST_F2_WEIGHT = 0x13
CST_F2_AREA = 0x014

CST_BPC0 = 0xB0	#[15:0]
CST_BPC1 = 0xB2

CST_CHIP_ID = 0xA7
CST_PROJ_ID = 0xA8
CST_FIRM_V = 0xA9

CST_MOTION_MSK = 0xEC		#[0]EnDClick [1]EnConUD [2] EnConLR
CST_IRQ_PULSE_WDTH = 0xED	#0.1ms to 200ms, default 10
CST_NOR_SCAN_PER = 0xEE		#0.1ms to 30ms, default 1
CST_MOTION_S1_ANGLE = 0xEF	#Angle = tan(c)*10

CST_LPSCAN_RAW_1 = 0xF0	#[15:0]
CST_LPSCAN_RAW_2 = 0xF2

CST_LP_AUTO_WAKETIME = 0xF4	#1s to 5 Default 5
CST_LP_SCAN_TH = 0xF5		#1 tà 255 default 48
CST_LP_SCAN_WIN = 0xF6		#0,1,2 or 3
CST_LP_SCAN_FREQ = 0xF7		#1 to 255 default 7
CST_LP_SCAN_IDAC = 0xF8		#1 to 255

CST_AUTO_SLEEPTIME = 0xF9	#1s to ? defaut 2s
CST_AUTO_IRQCTL = 0xFA		#[7]EnTest [6]EnTouch [5] EnChange [4]EnMotion [0] OnceWLP
CST_AUTO_RESET = 0xFB		#1s to 5 Default 5
CST_LONG_PRESSTIME = 0xFC	#1s to 5 Default 10

CST_IO_CTRL = 0xFD			#[0] 0=VDD/1=1.8V [1] 0=I2C_ADD/1=0D [2] 1=soft?/2=hard?
CST_DIS_AUTOSLEEP = 0xFE	#Defaut 0

CHIP_DICTIONARY = {
    0x11: "CST826",
    0xB4: "CST816S",
    0xB5: "CST816T",
    0xB6: "CST816D",
    0xB7: "CST820",
}

GESTURE_DICTIONARY = {
    0x00: "NO",
    0X01: "SWIPE_UP",
    0x02: "SWIPE_DOWN",
    0x03: "SWIPE_LEFT",
    0x04: "SWIPE_RIGHT",
    0x05: "SINGLE_CLICK",
    0x0B: "DOUBLE_CLICK",
    0x0C: "LONG_PRESS",
}

IRQ_CTRL_DICTIONARY = {
    0x01: "ONCEWLP",
    0x10: "MOTION",
    0x20: "CHANGE",
    0x40: "TOUCH",
    0X80: "TEST",
}

MOTION_MASK_DICTIONARY = {
    0x01: "DBLE CLICK",
    0x02: "CTRL UD",
    0x04: "CTRL LR",
}


class CST8X(object):

    def __init__(self, i2c, address=None, int_pin=None, int_handler=None, 
                 width=536, height=240, x_min=14, x_max=600,
                 y_min=14, y_max=228, debug=False):

        self._i2c = i2c
        self._debug = debug
        
        if address is None :
            devices = set(self._i2c.scan())
            mpus = devices.intersection(set(CST_DEFAULT_ADDRESS))
            nb_of_mpus = len(mpus)
            if nb_of_mpus == 0:
                self._ready = False
                return
                #raise ValueError("No CSTXXX detected")
            elif nb_of_mpus == 1:
                self._cst_add = mpus.pop()
                self._dbg("CST8X : DEVICE FOUND AT ADDRESS... ",hex(self._cst_add))
            else:
                raise ValueError("Two CST8x detected: must specify a device address")
        else :
            self._cst_add = address
            
        self.width = width
        self.height = height
        self.calibrate(x_min,x_max,y_min, y_max)

        chip_data = self.read(CST_CHIP_ID, 3)
        chip_id, proj_id, firm_id = unpack_from("<BBB", chip_data)
        chip_id &= 0xFF
        proj_id &= 0xFF
        firm_id &= 0xFF
        self._dbg("Chip {} Project {:02X} Firmware {:02X}".format(CHIP_DICTIONARY[chip_id], proj_id, firm_id))
            
        if int_pin is not None:
            self.int_pin = int_pin
            #self.int_pin.init(int_pin.IN)
            self.int_handler = int_handler
            self.int_locked = False
            int_pin.irq(trigger=int_pin.IRQ_FALLING | int_pin.IRQ_RISING,
                        handler=self.int_press)

    def int_press(self, pin):
        """Send X,Y values to passed interrupt handler."""
        if not pin.value() and not self.int_locked:
            self.int_locked = True  # Lock Interrupt
            res = self.raw_touch()
            if res is not None:
                self.int_handler(res[0], res[1])
            sleep_ms(100)  # Debounce falling edge
        elif pin.value() and self.int_locked:
            sleep_ms(100)  # Debounce rising edge
            self.int_locked = False  # Unlock interrupt

    def calibrate(self,xmin,xmax,ymin,ymax):
        self.x_min = xmin
        self.x_max = xmax
        self.y_min = ymin
        self.y_max = ymax
        self.x_multiplier = self.width / (xmax - xmin)
        self.x_add = xmin * self.x_multiplier		# f(xmin)=0
        self.y_multiplier = self.height / (ymax - ymin)
        self.y_add = ymin * self.y_multiplier		# g(ymin)=0
        
    def full_touch(self):
        val = self._i2c.readfrom_mem(self._cst_add, CST_GESTURE_ID, 14)
        g, f, f1x, f1y, f1w, f1a, f2x, f2y, f2w, f2a = unpack_from(">BBHHBBHHBB",val)
        f1x &= 0x0FFF
        f1y &= 0x0FFF
        f2x &= 0x0FFF
        f2y &= 0x0FFF
        self._dbg(g, f, f1x, f1y, f1w, f1a, f2x, f2y, f2w, f2a)

    def normalized_touch(self):
        #Normalize mean X,Y values to match LCD screen.
        res = self.raw_touch()
        if res == None :
            return None
        else :
            x = int(self.x_multiplier * (res[0]-self.x_min))
            y = int(self.y_multiplier * (res[1]-self.y_min))
        return (x, y)

    def raw_touch(self):
        val = self._i2c.readfrom_mem(self._cst_add, CST_FINGER_NUM, 5)
        f, x, y = unpack_from(">BHH",val)
        self._dbg(f,x,y)
        if f == 0 :
            return None
        else :
            x &= 0x0FFF
            y &= 0x0FFF
            if y < self.y_min : self.calibrate(self.x_min, self.x_max, y, self.y_max)
            if y > self.y_max : self.calibrate(self.x_min, self.x_max, self.y_min, y)
            if x < self.y_min : self.calibrate(x, self.x_max, self.y_min, self.y_max)
            if x > self.y_max : self.calibrate(self.x_min, x, self.y_min, self.y_max)
            return (x, y)
        
    def enable_motion(self, dblclck=False, contr_UD=False, contr_LR=False):
        mot_mask_data = self.read(CST_MOTION_MSK, 1)
        mot_mask = unpack_from("<B", mot_mask_data)[0]
        mot_mask &= 0x07 
        if dblclck : mot_mask |= 0x01
        if contr_UD : mot_mask |= 0x02
        if contr_LR : mot_mask |= 0x04
        self.write(CST_MOTION_MSK,mot_mask)

    def enable_irq_control(self, test=False, touch=False, change=False, motion=False, oncewlp=False):
        irq_ctrl_data = self.read(CST_AUTO_IRQCT, 1)
        irq_ctrl = unpack_from("<B", irq_ctrl_data)[0]
        irq_ctrl &= 0xF1
        if test : irq_ctrl &= 0x80
        if touch : irq_ctrl &= 0x40
        if change : irq_ctrl &= 0x20
        if motion : irq_ctrl &= 0x10
        if oncewlp : irq_ctrl &= 0x01
        self.write(CST_AUTO_IRQCT,irq_ctrl)        
    
    def write(self, register, values) :
        self.i2c.writeto_mem(self._cst_add, register, bytes(values))
        self._dbg("Writing $%02X <= %s" % (values[0], [hex(i) for i in values[1:]]))
        
    def read(self, register, length, irq_pin=None) :
        if irq_pin is not None:
            while irq_pin.value:
                pass
        result = self._i2c.readfrom_mem(self._cst_add, register, length)
        #self._dbg("Reading %02X => %s" % (register, [hex(i) for i in result]))
        return result
            
    def _dbg(self, *args, **kwargs):
        if self._debug:
            print("DBG::\t\t", *args, **kwargs)
