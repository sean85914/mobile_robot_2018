import RPi.GPIO as GPIO
import time
GPIO.setmode(GPIO.BCM)
GPIO.setup(21, GPIO.IN)
ts = time.time()
while time.time() - ts < 5:
	print GPIO.input(21)
	time.sleep(0.1)
