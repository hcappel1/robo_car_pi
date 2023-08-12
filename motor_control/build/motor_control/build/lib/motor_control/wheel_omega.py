import rclpy
from rclpy.node import Node

from std_msgs.msg import Int32
from std_msgs.msg import Float32MultiArray

import math


class WheelAngVel(Node):

	def __init__(self):
		super().__init__('wheel_ang_vel_node')
		self.prev_encoder_count_m1 = 0
		self.prev_encoder_count_m2 = 0
		self.prev_encoder_count_m3 = 0
		self.prev_encoder_count_m4 = 0

		self.encoder_count_m1 = 0
		self.encoder_count_m2 = 0
		self.encoder_count_m3 = 0
		self.encoder_count_m4 = 0

		self.omega_m1 = 0
		self.omega_m2 = 0
		self.omega_m3 = 0
		self.omega_m4 = 0

		self.subscription1 = self.create_subscription(
			Int32,
			'encoder_count_m1',
			self.listener_cb_m1,
			10
			)
		self.subscription1

		self.subscription2 = self.create_subscription(
			Int32,
			'encoder_count_m2',
			self.listener_cb_m2,
			10
			)
		self.subscription2

		self.subscription3 = self.create_subscription(
			Int32,
			'encoder_count_m3',
			self.listener_cb_m3,
			10
			)
		self.subscription3

		self.subscription4 = self.create_subscription(
			Int32,
			'encoder_count_m4',
			self.listener_cb_m4,
			10
			)

		self.publisher_omega = self.create_publisher(Float32MultiArray, 'omega_motors', 100)

		self.timer = self.create_timer(0.2, self.timer_callback)

	def timer_callback(self):
		if (self.encoder_count_m1 >= self.prev_encoder_count_m1):
			encoder_diff_m1 = self.encoder_count_m1 - self.prev_encoder_count_m1
		else:
			encoder_diff_m1 = self.encoder_count_m1 + (40 - self.prev_encoder_count_m1)

		self.omega_m1 = (encoder_diff_m1*math.pi/20)/0.2 
		self.prev_encoder_count_m1 = self.encoder_count_m1

		####################################################

		if (self.encoder_count_m2 >= self.prev_encoder_count_m2):
			encoder_diff_m2 = self.encoder_count_m2 - self.prev_encoder_count_m2
		else:
			encoder_diff_m2 = self.encoder_count_m2 + (40 - self.prev_encoder_count_m2)

		self.omega_m2 = (encoder_diff_m2*math.pi/20)/0.2 
		self.prev_encoder_count_m2 = self.encoder_count_m2

		#####################################################

		if (self.encoder_count_m3 >= self.prev_encoder_count_m3):
			encoder_diff_m3 = self.encoder_count_m3 - self.prev_encoder_count_m3
		else:
			encoder_diff_m3 = self.encoder_count_m3 + (40 - self.prev_encoder_count_m3)

		self.omega_m3 = (encoder_diff_m3*math.pi/20)/0.2 
		self.prev_encoder_count_m3 = self.encoder_count_m3

		######################################################

		if (self.encoder_count_m4 >= self.prev_encoder_count_m4):
			encoder_diff_m4 = self.encoder_count_m4 - self.prev_encoder_count_m4
		else:
			encoder_diff_m4 = self.encoder_count_m4 + (40 - self.prev_encoder_count_m4)

		self.omega_m4 = (encoder_diff_m4*math.pi/20)/0.2 
		self.prev_encoder_count_m4 = self.encoder_count_m4

		#######################################################
		#self.get_logger().info('wheel 1 ang vel: %f' % self.omega_m1)
		#self.get_logger().info('wheel 2 ang vel: %f' % self.omega_m2)
		#self.get_logger().info('wheel 3 ang vel: %f' % self.omega_m3)
		#self.get_logger().info('wheel 4 ang vel: %f' % self.omega_m4)

		#######################################################

		omega_msg = Float32MultiArray()
		omega_msg.data = [self.omega_m1, self.omega_m2, self.omega_m3, self.omega_m4]
		self.publisher_omega.publish(omega_msg)

	def listener_cb_m1(self, msg):
		self.encoder_count_m1 = msg.data

	def listener_cb_m2(self, msg):
		self.encoder_count_m2 = msg.data

	def listener_cb_m3(self, msg):
		self.encoder_count_m3 = msg.data

	def listener_cb_m4(self, msg):
		self.encoder_count_m4 = msg.data
		

def main(args=None):
	rclpy.init(args=args)

	wheel_ang_vel = WheelAngVel()

	rclpy.spin(wheel_ang_vel)

	wheel_ang_vel.destroy_node()
	rclpy.shutdown()

if __name__ == '__main__':
	main()


