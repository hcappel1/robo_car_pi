import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import RPi.GPIO as GPIO
import time
import math
import board
from adafruit_motorkit import MotorKit
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor

kit = MotorKit(i2c=board.I2C())
kit.motor1.throttle = 0.0
kit.motor2.throttle = 0.0
kit.motor3.throttle = 0.0
kit.motor4.throttle = 0.0

class OmegaPID(Node):

	def __init__(self):
		super().__init__('motor_pid_node')

		self.m1_throttle = 0.2
		self.m2_throttle = 0.2

		#Callback groups
		self.m1_pid_group = MutuallyExclusiveCallbackGroup()
		self.m1_data_group = MutuallyExclusiveCallbackGroup()
		self.m2_pid_group = MutuallyExclusiveCallbackGroup()
		self.m2_data_group = MutuallyExclusiveCallbackGroup()

		#Controller params
		self.kp = 0.15
		self.ki = 0.1
		self.kd = 0.03

		#Motor 1 controller values
		self.m1_error = 0
		self.m1_cum_error = 0
		self.m1_cum_error_prev = 0
		self.m1_rate_error = 0
		self.m1_last_error = 0
		self.m1_output_voltage = 0
		self.m1_max_voltage = 11.1
		self.m1_input = 0
		self.dt = 0.01

		#Motor 2 controller values
		self.m2_error = 0
		self.m2_cum_error = 0
		self.m2_cum_error_prev = 0
		self.m2_rate_error = 0
		self.m2_last_error = 0
		self.m2_output_voltage = 0
		self.m2_max_voltage = 11.1
		self.m2_input = 0
		self.dt = 0.01

		#Feedback params
		self.omega_m1_meas = 0.0
		self.omega_m2_meas = 0.0

		#Reference values
		self.omega_m1_ref = 10.0
		self.omega_m2_ref = 10.0

		#Wheel velocity subscribers
		self.omega_subscription = self.create_subscription(
			Float32MultiArray,
			'wheel_omegas',
			self.omega_motors_cb,
			10,
			callback_group=self.m1_data_group
			)
		self.omega_subscription

		#Timers
		self.m1_pid_timer = self.create_timer(0.01, self.m1_pid_cb, callback_group=self.m1_pid_group)
		self.m2_pid_timer = self.create_timer(0.01, self.m2_pid_cb, callback_group=self.m2_pid_group)
		# self.omega_ref_timer = self.create_timer(8, self.omega_ref_callback)

	def m1_pid_cb(self):
		#Proportional
		self.m1_error = self.omega_m1_ref - self.omega_m1_meas
		#Integral
		self.m1_cum_error = self.m1_cum_error_prev + self.m1_error*self.dt
		#Derivative
		self.m1_rate_error = (self.m1_error - self.m1_last_error)/self.dt
		#PID
		self.m1_output_voltage = self.kp*self.m1_error + self.ki*self.m1_cum_error + self.kd*self.m1_rate_error

		#Prevent saturation
		if (self.m1_output_voltage > self.m1_max_voltage):
			self.m1_output_voltage = self.m1_max_voltage
			self.m1_cum_error = self.m1_cum_error_prev

		#Recursion
		self.m1_last_error = self.m1_error
		self.m1_cum_error_prev = self.m1_cum_error

		#Motor input is output voltage over max voltage as a ratio
		self.m1_input = self.m1_output_voltage/self.m1_max_voltage

		if (self.m1_input > 1):
			self.m1_input = 1

		if (self.m1_input < -1):
			self.m1_input = -1

		#Send command to motor
		kit.motor1.throttle = self.m1_input
		
		self.get_logger().info('wheel 1 throttle: %f' % self.m1_input)
		self.get_logger().info('wheel 1 vel: %f' % self.omega_m1_meas)

	def m2_pid_cb(self):
		#Proportional
		self.m2_error = self.omega_m2_ref - self.omega_m2_meas
		#Integral
		self.m2_cum_error = self.m2_cum_error_prev + self.m2_error*self.dt
		#Derivative
		self.m2_rate_error = (self.m2_error - self.m2_last_error)/self.dt
		#PID
		self.m2_output_voltage = self.kp*self.m2_error + self.ki*self.m2_cum_error + self.kd*self.m2_rate_error

		#Prevent saturation
		if (self.m2_output_voltage > self.m2_max_voltage):
			self.m2_output_voltage = self.m2_max_voltage
			self.m2_cum_error = self.m2_cum_error_prev

		#Recursion
		self.m2_last_error = self.m2_error
		self.m2_cum_error_prev = self.m2_cum_error

		#Motor input is output voltage over max voltage as a ratio
		self.m2_input = self.m2_output_voltage/self.m2_max_voltage

		if (self.m2_input > 1):
			self.m2_input = 1

		if (self.m2_input < -1):
			self.m2_input = -1

		#Send command to motor
		kit.motor2.throttle = self.m2_input
		
		#self.get_logger().info('wheel 1 throttle: %f' % self.m1_input)
		#self.get_logger().info('wheel 1 vel: %f' % self.omega_m1_meas)

	def omega_ref_callback(self):
		# if (self.omega_m1_ref < 40):
		# 	self.omega_m1_ref = self.omega_m1_ref + 10
		# else:
		# 	self.omega_m1_ref = 10

		# if (self.omega_m2_ref < 40):
		# 	self.omega_m2_ref = self.omega_m2_ref + 10
		# else:
		# 	self.omega_m2_ref = 10

		if (self.omega_m1_ref == 15):
			self.omega_m1_ref = 10
		else:
			self.omega_m1_ref = 15

	def omega_motors_cb(self, msg):
		self.omega_m1_meas = msg.data[0]
		self.omega_m2_meas = msg.data[1]

def main(args=None):
	rclpy.init(args=args)
	
	omega_pid = OmegaPID()

	executor = MultiThreadedExecutor()
	executor.add_node(omega_pid)
	try:
		executor.spin()
	except KeyboardInterrupt:
		kit.motor1.throttle = 0.0
		kit.motor2.throttle = 0.0
	finally:
		executor.shutdown()
		omega_pid.destroy_node()
	rclpy.shutdown()
	
		

if __name__ == '__main__':
	main()