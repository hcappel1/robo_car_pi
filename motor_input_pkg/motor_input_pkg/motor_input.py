import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import board
from adafruit_motorkit import MotorKit
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor

kit = MotorKit(i2c=board.I2C())
kit.motor1.throttle = 0.0
kit.motor2.throttle = 0.0
kit.motor3.throttle = 0.0
kit.motor4.throttle = 0.0


class MotorInput(Node):

	def __init__(self):
		super().__init__('motor_input_node')

		#Params
		self.m1_input = 0.0
		self.m2_input = 0.0
		self.m3_input = 0.0
		self.m4_input = 0.0

		self.m1_test_throttle = 0.5
		self.m2_test_throttle = 0.5
		self.m3_test_throttle = 0.5
		self.m4_test_throttle = 0.5

		self.testing = True

		#Callback groups
		self.motors_input_group = MutuallyExclusiveCallbackGroup()
		self.motor_run_group = MutuallyExclusiveCallbackGroup()

		#Subscribers
		self.motor_input_subscription = self.create_subscription(
			Float32MultiArray,
			'control_input_motors',
			self.control_input_motors_cb,
			10,
			callback_group=self.motors_input_group
			)
		self.motor_input_subscription

		#Timers
		self.motor_run_timer = self.create_timer(0.01, self.motor_test_cb, callback_group=self.motor_run_group)

	def control_input_motors_cb(self, msg):
		self.m1_input = msg.data[0]
		self.m2_input = msg.data[1]
		self.m3_input = msg.data[2]
		self.m4_input = msg.data[3]

		if (not self.testing):
			kit.motor1.throttle = self.m1_input
			kit.motor2.throttle = self.m2_input
			kit.motor3.throttle = self.m3_input
			kit.motor4.throttle = self.m4_input

	def motor_test_cb(self):
		if (self.testing):
			kit.motor1.throttle = self.m1_test_throttle
			kit.motor2.throttle = self.m2_test_throttle
			kit.motor3.throttle = self.m3_test_throttle
			kit.motor4.throttle = self.m4_test_throttle
		



def main(args=None):
	rclpy.init(args=args)
	
	motor_input_obj = MotorInput()

	executor = MultiThreadedExecutor()
	executor.add_node(motor_input_obj)
	try:
		executor.spin()
	except KeyboardInterrupt:
		kit.motor1.throttle = 0.0
		kit.motor2.throttle = 0.0
		kit.motor3.throttle = 0.0
		kit.motor4.throttle = 0.0
	finally:
		executor.shutdown()
		motor_input_obj.destroy_node()
	rclpy.shutdown()
	
		

if __name__ == '__main__':
	main()