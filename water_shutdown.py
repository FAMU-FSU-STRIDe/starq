import Jetson.GPIO as GPIO
import os
import time

# Pin Definitions
sensor_pin = 18  # Adjust this to the GPIO pin you're using

# Setup GPIO
GPIO.setmode(GPIO.BOARD)
GPIO.setup(sensor_pin, GPIO.IN)

def shutdown():
    print("Water detected! Shutting down...")
    os.system("sudo shutdown -h now")

try:
    while True:
        if GPIO.input(sensor_pin) == GPIO.HIGH:
            shutdown()
        time.sleep(1)  # Check every second
except KeyboardInterrupt:
    print("Script interrupted by user")
finally:
    GPIO.cleanup()