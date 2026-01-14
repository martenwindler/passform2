#Bibliotheken
import RPi.GPIO as GPIO
import time

#GPIO Modus (BOARD/BCM)
GPIO.setmode(GPIO.BCM)

#GPIO Pins
GPIO_TRIGGER_R = 19
GPIO_ECHO_R = 26
GPIO_TRIGGER_L = 20
GPIO_ECHO_L = 21

#GPIO IN/OUT
GPIO.setup(GPIO_TRIGGER_R, GPIO.OUT)
GPIO.setup(GPIO_ECHO_R, GPIO.IN)
GPIO.setup(GPIO_TRIGGER_L, GPIO.OUT)
GPIO.setup(GPIO_ECHO_L, GPIO.IN)

def distanz(sensor):

    if sensor=="l":
        trigger = GPIO_TRIGGER_L
        echo = GPIO_ECHO_L
    elif sensor=="r":
        trigger = GPIO_TRIGGER_R
        echo = GPIO_ECHO_R

    #Trigger auf HIGH
    GPIO.output(trigger, True)
    #Trigger nach 0,01ms auf LOW
    time.sleep(0.00001)
    GPIO.output(trigger, False)

    StartZeit = time.time()
    StopZeit = time.time()

    #speicher Startzeit
    while GPIO.input(echo) == 0:
        StartZeit = time.time()

    #speicher Ankunftzeit
    while GPIO.input(echo) == 1:
        StopZeit = time.time()

	#Zeit zwischen Start und Stop
    TimeElapsed = StopZeit -StartZeit
	#Multiplikation mit Schallgeschw und durch 2 da Hin und Zurueck
    distanz = (TimeElapsed*34300)/2

    return distanz

if __name__ == '__main__':
    try:
        while True:
            abstand = distanz("r")
            print ("Entfernung rechts = %.1f cm" % abstand)
            time.sleep(5)
            abstand = distanz("l")
            print ("Entfernung links = %.1f cm" % abstand)
            time.sleep(5)

    #Abbrechen und Resetten durch STRG+C
    except KeyboardInterrupt:
        print("Messung gestoppt")
        GPIO.cleanup()
