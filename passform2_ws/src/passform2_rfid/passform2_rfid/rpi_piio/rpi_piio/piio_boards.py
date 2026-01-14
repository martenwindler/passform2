#!/usr/bin/python
import enum
import time
from rpi_piio.PiIO import PiIO_Analog
from gpiozero import PWMLED, LED, Button
from rpi_piio.PiIO_ADS1x15 import ADS1015

class PiioPins(enum.Enum):
    O1 = 5
    O2 = 6
    O3 = 13
    O4 = 16
    O5 = 19
    O6 = 20
    O7 = 26
    O8 = 21
    RUN=25
    OE=12

    PWM1=17
    PWM2=27

    I1 = 22
    I2 = 23
    I3 = 24
    I4 = 8
    FAULT=7

    AI1 = 1
    AI2 = 2
    AI3 = 3
    AI4 = 0

class Adio(object):
    def __init__(self, gain=2):
        # TODO: try/except actually needed?
        self.pins = PiioPins()
        
        try:
            self.adc = PiIO_Analog(gain)
            self.gain = gain
            # csPin = 18
            # misoPin = 9
            # mosiPin = 10
            # clkPin = 11
            self.setup_pins()
        except:
            raise OSError('Check I2C and SPI enabled in raspi-config, disable Serial')

    def setup_pins(self):
        run = LED(self.adc.RUN)

        #Analog Out
        self.AO = dict()
        self.AO[1] = PWMLED(self.adc.PWM1)
        self.AO[2] = PWMLED(self.adc.PWM2)
        time.sleep(.1)

        # Analog IN
        self.AI = dict()
        self.AI[1] = 4
        time.sleep(.03)
        self.AI[2] = 2
        time.sleep(.03)
        self.AI[3] = 3
        time.sleep(.03)
        self.AI[4] = 1
        time.sleep(.03)

        #DigitalIn
        self.DI = dict()
        self.DI[1] = Button(self.adc.I1,pull_up=False)
        time.sleep(.03)
        self.DI[2] = Button(self.adc.I2,pull_up=False)
        time.sleep(.03)
        self.DI[3] = Button(self.adc.I3,pull_up=False)
        time.sleep(.03)
        self.DI[4] = Button(self.adc.I4,pull_up=False) # IXME: Doesnt work for some reason....
        time.sleep(.1)

        #DigitalOut
        self.DO = dict()
        self.DO[1] = LED(self.adc.O1)
        time.sleep(.03)
        self.DO[2] = LED(self.adc.O2)
        time.sleep(.03)
        self.DO[3] = LED(self.adc.O3)
        time.sleep(.03)
        self.DO[4] = LED(self.adc.O4)
        time.sleep(.03)
        self.DO[5] = LED(self.adc.O5)
        time.sleep(.03)
        self.DO[6] = LED(self.adc.O6)
        time.sleep(.03)
        self.DO[7] = LED(self.adc.O7)
        time.sleep(.03)
        self.DO[8] = LED(self.adc.O8)
        time.sleep(.03)
        self.DO[0] = LED(self.adc.OE)
        #self.DO['F'] = LED(self.adc.FAULT)    # on-board fault LED
        time.sleep(.1)

    def get_DI_pins(self):
        return list(self.DI.keys())
    def get_AI_pins(self):
        return list(self.AI.keys())
    def get_DO_pins(self):
        return list(self.DO.keys())
    def get_AO_pins(self):
        return list(self.AO.keys())

    def set_digital(self, pin, value):
        '''Sets the specified digital pin to value
        Throws, if pin is not part of the board or value is not in [0,1]
        '''
        self.DO[0].on()
        assert pin in list(self.DO.keys()), f'Invalid pin. {pin} not in {self.DO.keys()}'
        #assert value in [0,1], 'DO value must be in [0,1]'
        if value == 1:
            self.DO[pin].on()
        elif value == 0:
            self.DO[pin].off()

    def get_digital(self, pin):
        '''Returns the value of the specified digital pin
        Throws, if pin is not part of the board
        '''
        assert pin in self.DI.keys(), f'Invalid pin. {pin} not in {self.DI.keys()}'
        if self.DI[pin].is_pressed:
            return 1
        else:
            return 0

    def set_analog(self, pin, value):
        '''Sets the specified analog pin to value
        Throws, if pin is not part of the board or value is not in [0,1]
        '''
        assert pin in self.get_AO_pins(), f'Invalid pin. {pin} not in {self.AO.keys()}'
        #assert value in [0.0,1.0], f'AO value must be in [0,1]'
        self.AO[pin].value = value

    def get_analog(self, pin):
        '''Returns the value of the specified analog pin
        Throws, if pin is not part of the board
        '''
        assert pin in self.get_AI_pins(), f'Invalid pin. {pin} not in {self.AI.keys()}'
        return self.adc.get_scaled(self.AI[pin]-1)

if __name__ == '__main__':
    piio = Adio()
