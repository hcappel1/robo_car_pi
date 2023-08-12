from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
	return LaunchDescription([
		Node(
			package='motor_input_pkg',
			executable='motor_input_node',
			output='screen',
		),
	])