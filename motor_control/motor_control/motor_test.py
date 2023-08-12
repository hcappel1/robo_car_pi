import rclpy
from rclpy.node import Node
import RPi.GPIO as GPIO
import time
import math
import board
from adafruit_motorkit import MotorKit


if __name__ == '__main__':
	kit = MotorKit(i2c=board.I2C())
	while True:
		try:
			kit.motor1.throttle = 0.3
			kit.motor2.throttle = 0.3
			kit.motor3.throttle = 0.0
			kit.motor4.throttle = 0.0

			time.sleep(3)

			kit.motor1.throttle = 0.8
			kit.motor2.throttle = 0.8
			kit.motor3.throttle = 0.0
			kit.motor4.throttle = 0.0

			time.sleep(3)

			kit.motor1.throttle = 0.3
			kit.motor2.throttle = 0.3
			kit.motor3.throttle = 0.0
			kit.motor4.throttle = 0.0

			time.sleep(3)

			kit.motor1.throttle = 0.8
			kit.motor2.throttle = 0.8
			kit.motor3.throttle = 0.0
			kit.motor4.throttle = 0.0
		except KeyboardInterrupt:
			kit.motor1.throttle = 0.0
			kit.motor2.throttle = 0.0
			kit.motor3.throttle = 0.0
			kit.motor4.throttle = 0.0
			break