#Bibliotheken
import RPi.GPIO as GPIO
import time

#GPIO Modus (BOARD/BCM)
GPIO.setmode(GPIO.BCM)

#GPIO Pins
GPIO_TRIGGER = 20
GPIO_ECHO = 21

#GPIO IN/OUT
GPIO.setup(GPIO_TRIGGER, GPIO.OUT)
GPIO.setup(GPIO_ECHO, GPIO.IN)

def distanz():
	#Trigger auf HIGH
	GPIO.output(GPIO_TRIGGER, True)

	#Trigger nach 0,01ms auf LOW
	time.sleep(0.00001)
	GPIO.output(GPIO_TRIGGER, False)

	StartZeit = time.time()
	StopZeit = time.time()

	#speicher Startzeit
	while GPIO.input(GPIO_ECHO) == 0:
		StartZeit = time.time()

	#speicher Ankunftzeit
	while GPIO.input(GPIO_ECHO) == 1:
		StopZeit = time.time()

	#Zeit zwischen Start und Stop
	TimeElapsed = StopZeit -StartZeit
	#Multiplikation mit Schallgeschw und durch 2 da Hin und Zurueck
	distanz = (TimeElapsed*34300)/2

	return distanz

if __name__ == '__main__':
	try:
		while True:
			abstand = distanz()
			print ("Entfernung = %.1f cm" % abstand)
			time.sleep(1)

	#Abbrechen und Resetten durch STRG+C
	except KeyboardInterrupt:
		print("Messung gestoppt")
		GPIO.cleanup()
