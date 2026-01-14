import RPi.GPIO as GPIO
import time

class UltrasonicSensor:

    def __init__(self,
        trigger_GPIO:int,
        echo_GPIO:int,
        label:str=None):

        #Label of US-Sensor(z.B. 1,2,rechts,links)
        self.label = label

        #GPIO Modus (BOARD/BCM)
        GPIO.setmode(GPIO.BCM)

        #GPIO Pins
        self.trigger = trigger_GPIO
        self.echo = echo_GPIO

        #GPIO IN/OUT
        GPIO.setup(self.trigger, GPIO.OUT)
        GPIO.setup(self.echo, GPIO.IN)

        self.speed_of_sound = 343 # speed of sound in m/s

    #eventuell nützliche Methoden
    def __repr__(self):
        return f"Ultrasonic_sensor({self.label!r}, {self.trigger}, {self.echo})"

    def __str__(self):
        return f"Ultrasonic_sensor: ({self.label!r}, {self.trigger}, {self.echo})"

    def __del__(self):
        GPIO.cleanup((self.trigger, self.echo))
        print("destroyed" + self.__str__())

    def get_distance(self):
        # Trigger auf HIGH
        GPIO.output(self.trigger, True)
        # Trigger nach 0,01ms auf LOW
        time.sleep(0.00001)
        GPIO.output(self.trigger, False)

        # start time in sec
        while GPIO.input(self.echo) == 0:
            time_start = time.time()
        # echo_pin to LOW gives stop time in sec
        while GPIO.input(self.echo) == 1:
            time_stop = time.time()
        #Zeit zwischen Start und Stop
        time_elapsed = time_stop - time_start
        distance = (time_elapsed*343)/2 # distance in m, divided by 2 since way back and forth
        return distance

if __name__ == '__main__':
    Sensor1 = UltrasonicSensor(19, 26,'rechts')
    Sensor2 = UltrasonicSensor(20,21,'l')

    abstand = Sensor1.get_distance()
    print(Sensor1)
    print(abstand)
    time.sleep(3)
    abstand = Sensor2.get_distance()
    print(Sensor2)
    print(abstand)
    #GPIO.cleanup()
