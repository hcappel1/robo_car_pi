import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
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
		self.rate = self.create_rate(2)

		self.m1_encoder_t1 = self.get_clock().now().to_msg().nanosec
		self.m1_encoder_t2 = self.get_clock().now().to_msg().nanosec
		self.m1_freq = 0
		self.m1_omega = 0.0

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

		GPIO.add_event_detect(M1_PIN, GPIO.BOTH, callback=self.m1_encoder_cb, bouncetime=1)
		GPIO.add_event_detect(M2_PIN, GPIO.BOTH, callback=self.m2_encoder_cb, bouncetime=1)
		GPIO.add_event_detect(M3_PIN, GPIO.BOTH, callback=self.m3_encoder_cb, bouncetime=1)
		GPIO.add_event_detect(M4_PIN, GPIO.BOTH, callback=self.m4_encoder_cb, bouncetime=1)

		while True:
			self.send_wheel_data()

##########################################################################################

	def m1_encoder_cb(self, channel):
		if (GPIO.input(M1_PIN) == GPIO.HIGH):
			self.m1_encoder_t1 = self.get_clock().now().to_msg().nanosec
			self.m1_freq = 1/((self.m1_encoder_t1 - self.m1_encoder_t2)*10**(-9))

		elif (GPIO.input(M1_PIN) == GPIO.LOW):
			self.m1_encoder_t2 = self.get_clock().now().to_msg().nanosec
			self.m1_freq = 1/((self.m1_encoder_t2 - self.m1_encoder_t1)*10**(-9))

		self.m1_omega = float(self.m1_freq*math.pi/20)

###########################################################################################

	def m2_encoder_cb(self, channel):
		if (GPIO.input(M2_PIN) == GPIO.HIGH):
			self.m2_encoder_t1 = self.get_clock().now().to_msg().nanosec
			self.m2_freq = 1/((self.m2_encoder_t1 - self.m2_encoder_t2)*10**(-9))

		elif (GPIO.input(M2_PIN) == GPIO.LOW):
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
			self.get_logger().info('%f' % self.m1_omega)
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