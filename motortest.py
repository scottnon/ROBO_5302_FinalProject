import RPi.GPIO as GPIO

from time import sleep

GPIO.setmode(GPIO.BCM)

GPIO.setup(15, GPIO.OUT)
GPIO.setup(14, GPIO.OUT)

p=GPIO.PWM(14, 100)
p.start(0)
GPIO.output(15, True)
p.ChangeDutyCycle(25)
GPIO.output(14, True)

sleep(5)
p.stop()
GPIO.cleanup()
