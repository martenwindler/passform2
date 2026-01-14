from rpi_i2c.i2c_types import AnalogOut
from rpi_i2c.i2c_types import DigitalIn
import RPi.GPIO as GPIO
import time

class ActiveButton:

    def __init__(self, label, redLEDInput, greenLEDInput, buttonInput):

        #GPIO Modus (BOARD/BCM)
        GPIO.setmode(GPIO.BCM)
        self.init = 17

    def eventlistener():
        if GPIO.input(self.init) == 1:
            di.read

    class Color(enumerate):
        RED = [1,0]
        ORANGE = [0,0.105]
        YELLOW = [0,0.12]
        GREEN = [0,1]

    #colors_dict = {'red':[1,0] , 'orange': [0, 0.105], 'yellow': [0, 0.12], 'green': [0,1]}

    def setColor(color):
        print("LED set to" + color)

if __name__=="__main__":
    Button1 = ActiveButton(1, 2, 4)
