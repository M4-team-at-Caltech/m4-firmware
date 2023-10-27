# GPIO library
import Jetson.GPIO as GPIO
 
# Handles time
import time 
 
# Pin Definition
led_pin = 7
 
# Set up the GPIO channel
GPIO.setmode(GPIO.BOARD) 
GPIO.setup(led_pin, GPIO.OUT, initial=GPIO.HIGH) 
 
GPIO.output(led_pin, GPIO.LOW) 
