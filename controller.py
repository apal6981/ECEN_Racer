from Arduino import Arduino
import time

def start_driving(Car):
    Car.drive(float(1.5))
    time.sleep(1)
    Car.drive(float(0.5))

def steering(angle):
    Car.steer(float(angle))    