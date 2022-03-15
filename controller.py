from Arduino import Arduino
import time

def start_driving(Car):
    Car.drive(float(2.5))
    print("Drive forward 2")
    time.sleep(1)
    # for i in range(50):
        # print("-")

def go_forward(Car, val):
    pass
    # Car.drive(float(val))
    # print("Drive forward 0.5")
    # Car.drive(float(0.8))

def steering(Car, angle):
    Car.steer(float(angle))    