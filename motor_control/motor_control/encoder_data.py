import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Int32
from std_msgs.msg import Float32
import RPi.GPIO as GPIO
import time
import math

M1_PIN = 17
M2_PIN = 18
M3_PIN = 27
M4_PIN = 22

class EncoderData(Node):

	def __init__(self):
		super().__init__('encoder_data_node')

		self.wheel_publisher = self.create_publisher(Float32MultiArray, 'wheel_ang_vel', 100)
		self.wheel_time_publisher = self.create_publisher(Int32, 'wheel_time_diff', 100)
		self.m1_omega_publisher = self.create_publisher(Float32, 'm1_omega', 100)
		self.rate = self.create_rate(2)

		self.m1_encoder_t1 = 0
		self.m1_encoder_t2 = 0
		self.m1_freq = 0
		self.m1_time_diff = 1
		self.m1_omega = 0.0
		self.m1_encoder_last = 0

		self.m2_encoder_t1 = self.get_clock().now().to_msg().nanosec
		self.m2_encoder_t2 = self.get_clock().now().to_msg().nanosec
		self.m2_freq = 0
		self.m2_omega = 0.0

		self.m3_encoder_t1 = self.get_clock().now().to_msg().nanosec
		self.m3_encoder_t2 = self.get_clock().now().to_msg().nanosec
		self.m3_freq = 0
		self.m3_omega = 0.0

		self.m4_encoder_t1 = self.get_clock().now().to_msg().nanosec
		self.m4_encoder_t2 = self.get_clock().now().to_msg().nanosec
		self.m4_freq = 0
		self.m4_omega = 0.0

		GPIO.add_event_detect(M1_PIN, GPIO.RISING, callback=self.m1_encoder_cb)
		GPIO.add_event_detect(M2_PIN, GPIO.BOTH, callback=self.m2_encoder_cb, bouncetime=1)
		GPIO.add_event_detect(M3_PIN, GPIO.BOTH, callback=self.m3_encoder_cb, bouncetime=1)
		GPIO.add_event_detect(M4_PIN, GPIO.BOTH, callback=self.m4_encoder_cb, bouncetime=1)

		#while True:
		#	self.send_wheel_data()

##########################################################################################

	def m1_encoder_cb(self, channel):
		#if (GPIO.input(M1_PIN) == GPIO.HIGH):
			#self.get_logger().info('HIGH')
		self.m1_encoder_t1 = self.get_clock().now().to_msg().nanosec
		self.get_logger().info('t1: %f' % self.m1_encoder_t1)
		
		if (self.m1_encoder_t1 > self.m1_encoder_last):
			self.m1_time_diff = self.m1_encoder_t1 - self.m1_encoder_last




		#if (self.m1_encoder_t1 > self.m2_encoder_t2):
		#	self.m1_time_diff = self.m1_encoder_t1 - self.m1_encoder_t2
		#else:
		#	self.m1_time_diff = self.m1_encoder_t1 + (1*10**(9) - self.m1_encoder_t2)
		
		#self.m1_freq = 1/(self.m1_time_diff*10**(-9))
		time_diff_msg = Int32()
		time_diff_msg.data = self.m1_time_diff
		self.wheel_time_publisher.publish(time_diff_msg)
		#self.get_logger().info('%i' % self.m1_time_diff)
		self.m1_encoder_last = self.m1_encoder_t1

		# elif (GPIO.input(M1_PIN) == GPIO.LOW):
		# 	#self.get_logger().info('LOW')
		# 	self.m1_encoder_t2 = self.get_clock().now().to_msg().nanosec
		# 	self.get_logger().info('t2: %f' % self.m1_encoder_t2)
			




		# 	if (self.m1_encoder_t2 > self.m1_encoder_t1):
		# 		self.m1_time_diff = self.m1_encoder_t2 - self.m1_encoder_t1
		# 	#else:
		# 	#	self.m1_time_diff = self.m1_encoder_t2 + (1*10**(9) - self.m1_encoder_t1)

		# 	self.m1_freq = 1/(self.m1_time_diff*10**(-9))
		# 	time_diff_msg = Int32()
		# 	time_diff_msg.data = self.m1_time_diff
		# 	self.wheel_time_publisher.publish(time_diff_msg)
		# 	#self.get_logger().info('%i' % self.m1_time_diff)

		self.m1_omega = float(self.m1_freq*math.pi/20)
		m1_msg = Float32()
		m1_msg.data = self.m1_omega
		self.m1_omega_publisher.publish(m1_msg)

###########################################################################################

	def m2_encoder_cb(self, channel):
		if (GPIO.input(M2_PIN) == GPIO.HIGH):
			self.get_logger().info('HIGH')
			self.m2_encoder_t1 = self.get_clock().now().to_msg().nanosec
			self.m2_freq = 1/((self.m2_encoder_t1 - self.m2_encoder_t2)*10**(-9))

		elif (GPIO.input(M2_PIN) == GPIO.LOW):
			self.get_logger().info('LOW')
			self.m2_encoder_t2 = self.get_clock().now().to_msg().nanosec
			self.m2_freq = 1/((self.m2_encoder_t2 - self.m2_encoder_t1)*10**(-9))

		self.m2_omega = float(self.m2_freq*math.pi/20)

############################################################################################

	def m3_encoder_cb(self, channel):
		if (GPIO.input(M3_PIN) == GPIO.HIGH):
			self.m3_encoder_t1 = self.get_clock().now().to_msg().nanosec
			self.m3_freq = 1/((self.m3_encoder_t1 - self.m3_encoder_t2)*10**(-9))

		elif (GPIO.input(M3_PIN) == GPIO.LOW):
			self.m3_encoder_t2 = self.get_clock().now().to_msg().nanosec
			self.m3_freq = 1/((self.m3_encoder_t2 - self.m3_encoder_t1)*10**(-9))

		self.m3_omega = float(self.m3_freq*math.pi/20)

#############################################################################################

	def m4_encoder_cb(self, channel):
		if (GPIO.input(M4_PIN) == GPIO.HIGH):
			self.m4_encoder_t1 = self.get_clock().now().to_msg().nanosec
			self.m4_freq = 1/((self.m4_encoder_t1 - self.m4_encoder_t2)*10**(-9))

		elif (GPIO.input(M4_PIN) == GPIO.LOW):
			self.m4_encoder_t2 = self.get_clock().now().to_msg().nanosec
			self.m4_freq = 1/((self.m4_encoder_t2 - self.m4_encoder_t1)*10**(-9))

		self.m4_omega = float(self.m4_freq*math.pi/20)

#############################################################################################

	def send_wheel_data(self):
		try:
			#self.get_logger().info('%f' % self.m1_omega)
			ang_vel_msg = Float32MultiArray()
			ang_vel_msg.data = [self.m1_omega, self.m2_omega, self.m3_omega, self.m4_omega]
			self.wheel_publisher.publish(ang_vel_msg)

		except KeyboardInterrupt:
			self.get_logger().info('encoder wheel node cancelled')



def main(args=None):
	rclpy.init(args=args)
	GPIO.setmode(GPIO.BCM)
	GPIO.setup(17,GPIO.IN, pull_up_down=GPIO.PUD_UP)
	GPIO.setup(18,GPIO.IN, pull_up_down=GPIO.PUD_UP)
	GPIO.setup(27,GPIO.IN, pull_up_down=GPIO.PUD_UP)
	GPIO.setup(22,GPIO.IN, pull_up_down=GPIO.PUD_UP)
	encoder_data = EncoderData()
	rclpy.spin(encoder_data)
	encoder_publisher.destroy_node()
	rclpy.shutdown()
	
		
		


if __name__ == '__main__':
	main()