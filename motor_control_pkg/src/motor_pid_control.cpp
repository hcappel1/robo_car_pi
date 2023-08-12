#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include <chrono>

using std::placeholders::_1;
using namespace std::chrono_literals;

class MotorPIDControl : public rclcpp::Node
{
public:

	float omega_m1 = 0.0;
	float omega_m2 = 0.0;
	float omega_m3 = 0.0;
	float omega_m4 = 0.0;
	
	MotorPIDControl() : Node("motor_pid_control")
	{
		//callback groups
		callback_group_1_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

		//publishers and subscribers
		wheel_omegas_sub = this->create_subscription<std_msgs::msg::Float32MultiArray>("wheel_omegas", 10, std::bind(&MotorPIDControl::WheelOmegasCB,this,_1));

		//timers
		m1_pid_timer = this->create_wall_timer(1ms, std::bind(&MotorPIDControl::m1_pid_execute, this), callback_group_1_);
	}

private:

	void WheelOmegasCB(const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
		omega_m1 = msg->data[0];
		omega_m2 = msg->data[1];
		omega_m3 = msg->data[2];
		omega_m4 = msg->data[3];

		//RCLCPP_INFO(this->get_logger(), "'%f'", omega_m1);
	}

	void m1_pid_execute(void) {
		RCLCPP_INFO(this->get_logger(), "execute m1 PID");
	}

	rclcpp::CallbackGroup::SharedPtr callback_group_1_;

	rclcpp::TimerBase::SharedPtr m1_pid_timer;

	rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr wheel_omegas_sub;
	
};



int main(int argc, char * argv[])
{

	//Initialize node
	rclcpp::init(argc,argv);
	std::shared_ptr<MotorPIDControl> motor_pid_control_node = std::make_shared<MotorPIDControl>();

	//Initialize one multithreaded executor object
	rclcpp::executors::MultiThreadedExecutor executor;
	executor.add_node(motor_pid_control_node);
	executor.spin();

	rclcpp::shutdown();
	return 0;
}