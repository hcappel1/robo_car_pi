import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
import RPi.GPIO as GPIO
import time
import board
from adafruit_motorkit import MotorKit
from os import geteuid

#kit = MotorKit(i2c=board.I2C())

class EncoderPublisher(Node):

	def __init__(self):
		super().__init__('encoder_count_node')
		self.publisher_m1 = self.create_publisher(Int32, 'encoder_count_m1', 100)
		self.publisher_m2 = self.create_publisher(Int32, 'encoder_count_m2', 100)
		self.publisher_m3 = self.create_publisher(Int32, 'encoder_count_m3', 100)
		self.publisher_m4 = self.create_publisher(Int32, 'encoder_count_m4', 100)

		self.stateLast_m1 = GPIO.input(17)
		self.stateLast_m2 = GPIO.input(18)
		self.stateLast_m3 = GPIO.input(27)
		self.stateLast_m4 = GPIO.input(22)

		self.stateCount_m1 = 0
		self.stateCount_m2 = 0
		self.stateCount_m3 = 0
		self.stateCount_m4 = 0

		self.rotationCount_m1 = 0
		self.rotationCount_m2 = 0
		self.rotationCount_m3 = 0
		self.rotationCount_m4 = 0
		
		self.statesPerRotation = 40		
		timer_period = 0.01
		self.timer = self.create_timer(timer_period, self.timer_callback)

	def timer_callback(self):
		self.stateCurrent_m1 = GPIO.input(17)
		self.stateCurrent_m2 = GPIO.input(18)
		self.stateCurrent_m3 = GPIO.input(27)
		self.stateCurrent_m4 = GPIO.input(22)

		#Run the routine for M1

		if self.stateCurrent_m1 != self.stateLast_m1:
			self.stateLast_m1 = self.stateCurrent_m1
			self.stateCount_m1 += 1
			msg_m1 = Int32()
			msg_m1.data = self.stateCount_m1
			self.publisher_m1.publish(msg_m1)
			#self.get_logger().info('%d' % self.stateCount)
		
		if self.stateCount_m1 == self.statesPerRotation:
			self.rotationCount_m1 += 1
			self.stateCount_m1 = 0

		#Run the routine for M2

		if self.stateCurrent_m2 != self.stateLast_m2:
			self.stateLast_m2 = self.stateCurrent_m2
			self.stateCount_m2 += 1
			msg_m2 = Int32()
			msg_m2.data = self.stateCount_m2
			self.publisher_m2.publish(msg_m2)
			#self.get_logger().info('%d' % self.stateCount)
		
		if self.stateCount_m2 == self.statesPerRotation:
			self.rotationCount_m2 += 1
			self.stateCount_m2 = 0

		#Run the routine for M3

		if self.stateCurrent_m3 != self.stateLast_m3:
			self.stateLast_m3 = self.stateCurrent_m3
			self.stateCount_m3 += 1
			msg_m3 = Int32()
			msg_m3.data = self.stateCount_m3
			self.publisher_m3.publish(msg_m3)
			#self.get_logger().info('%d' % self.stateCount)
		
		if self.stateCount_m3 == self.statesPerRotation:
			self.rotationCount_m3 += 1
			self.stateCount_m3 = 0

		#Run the routine for M4

		if self.stateCurrent_m4 != self.stateLast_m4:
			self.stateLast_m4 = self.stateCurrent_m4
			self.stateCount_m4 += 1
			msg_m4 = Int32()
			msg_m4.data = self.stateCount_m4
			self.publisher_m4.publish(msg_m4)
			#self.get_logger().info('%d' % self.stateCount)
		
		if self.stateCount_m4 == self.statesPerRotation:
			self.rotationCount_m4 += 1
			self.stateCount_m4 = 0

def main(args=None):
	rclpy.init(args=args)
	GPIO.setmode(GPIO.BCM)
	GPIO.setup(17,GPIO.IN, pull_up_down=GPIO.PUD_UP)
	GPIO.setup(18,GPIO.IN, pull_up_down=GPIO.PUD_UP)
	GPIO.setup(27,GPIO.IN, pull_up_down=GPIO.PUD_UP)
	GPIO.setup(22,GPIO.IN, pull_up_down=GPIO.PUD_UP)
	encoder_publisher = EncoderPublisher()
	rclpy.spin(encoder_publisher)
	encoder_publisher.destroy_node()
	rclpy.shutdown()
	
		
		


if __name__ == '__main__':
	main()