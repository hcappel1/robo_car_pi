from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
	return LaunchDescription([
		Node(
			package='motor_control',
			executable='encoder_count_node',
			output='screen',
			# prefix=["sudo -E env \"PYTHONPATH=$PYTHONPATH\" \"LD_LIBRARY_PATH=$LD_LIBRARY_PATH\" \"PATH=$PATH\" \"USER=$USER\" bash -c "]
		),
		Node(
			package='motor_control',
			executable='wheel_omega_node',
			output='screen',
			# prefix=["sudo -E env \"PYTHONPATH=$PYTHONPATH\" \"LD_LIBRARY_PATH=$LD_LIBRARY_PATH\" \"PATH=$PATH\" \"USER=$USER\" bash -c "]
		),
		Node(
			package='motor_control',
			executable='pid_omega_node',
			output='screen',
			# prefix=["sudo -E env \"PYTHONPATH=$PYTHONPATH\" \"LD_LIBRARY_PATH=$LD_LIBRARY_PATH\" \"PATH=$PATH\" \"USER=$USER\" bash -c "]
		),
		Node(
			package='motor_control',
			executable='encoder_data_node',
			output='screen',
			# prefix=["sudo -E env \"PYTHONPATH=$PYTHONPATH\" \"LD_LIBRARY_PATH=$LD_LIBRARY_PATH\" \"PATH=$PATH\" \"USER=$USER\" bash -c "]
		),

	])