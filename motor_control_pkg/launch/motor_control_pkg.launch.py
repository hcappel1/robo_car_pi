from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
	return LaunchDescription([
		Node(
			package='motor_control_pkg',
			executable='motor_pid_control_node',
			output='screen',
		),
	])