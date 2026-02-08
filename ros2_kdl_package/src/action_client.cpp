#include <functional>
#include <future>
#include <memory>
#include <string>
#include <sstream>

#include "ros2_kdl_package/action/ros2_kdl_traj.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

using Ros2KdlTraj = ros2_kdl_package::action::Ros2KdlTraj;
using GoalHandleRos2KdlTraj = rclcpp_action::ClientGoalHandle<Ros2KdlTraj>;
using namespace std::placeholders;

class KdlActionClient : public rclcpp::Node
{
    public:
		KdlActionClient() : KdlActionClient(rclcpp::NodeOptions()) {} 

		explicit KdlActionClient(const rclcpp::NodeOptions & options)
		: Node("kdl_action_client", options)
		{
			// dichiarazione del client nel nodo
			this->client_ptr_ = rclcpp_action::create_client<Ros2KdlTraj>(this, "ros2_kdl_traj");

		}

		void send_goal()
		{
			while (rclcpp::ok()) {
				double x, y, z, yaw, duration;
				std::cout << "Enter goal (x y z yaw duration): ";
				std::cin >> x >> y >> z >> yaw >> duration;

				if (!std::cin) {
					std::cout << "Input error. Exiting...\n";
					break;
				}

				Ros2KdlTraj::Goal goal_msg;
				goal_msg.x = x;
				goal_msg.y = y;
				goal_msg.z = z;
				goal_msg.yaw = yaw;
				goal_msg.traj_duration = duration;

				auto send_goal_options = rclcpp_action::Client<Ros2KdlTraj>::SendGoalOptions();
				send_goal_options.goal_response_callback = 
					std::bind(&KdlActionClient::goal_response_callback, this, _1);
				send_goal_options.feedback_callback = 
					std::bind(&KdlActionClient::feedback_callback, this, _1, _2);
				send_goal_options.result_callback = 
					std::bind(&KdlActionClient::result_callback, this, _1);

				client_ptr_->async_send_goal(goal_msg, send_goal_options);

				goal_done = false;
				while (!goal_done && rclcpp::ok()) {
					rclcpp::spin_some(this->get_node_base_interface());
				}
			}
      	}

    private:

		void goal_response_callback(const GoalHandleRos2KdlTraj::SharedPtr & goal_handle)
		{
			if (!goal_handle) {
				RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
			}
			else {
				RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
			}
		}

		void feedback_callback(
			GoalHandleRos2KdlTraj::SharedPtr,
			const std::shared_ptr<const Ros2KdlTraj::Feedback> feedback)
		{
			RCLCPP_INFO(this->get_logger(),
				"Action feedback: x=%.3f y=%.3f z=%.3f yaw=%.3f advancement=%.1f%%",
				feedback->current_x, feedback->current_y, feedback->current_z, feedback->current_yaw, feedback->advancement);
		}

		void result_callback(const GoalHandleRos2KdlTraj::WrappedResult & result_wrapper)
		{
			switch (result_wrapper.code) {
				case rclcpp_action::ResultCode::SUCCEEDED: {
					const auto & result = result_wrapper.result;
					RCLCPP_INFO(this->get_logger(),
						"Action result: x=%.3f y=%.3f z=%.3f yaw=%.3f",
						result->x_f, result->y_f, result->z_f, result->yaw_f);
					break;
				}
				case rclcpp_action::ResultCode::ABORTED:
					RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
					return;
				case rclcpp_action::ResultCode::CANCELED:
					RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
					return;
				default:
					RCLCPP_ERROR(this->get_logger(), "Unknown result code");
					return;
			}

			goal_done = true;
			
		}

		// dichiarazione variabili interne
		rclcpp_action::Client<Ros2KdlTraj>::SharedPtr client_ptr_;
		bool goal_done;
};  


int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto client_node = std::make_shared<KdlActionClient>();
    client_node->send_goal(); // ciclo interattivo
    rclcpp::shutdown();
    return 0;
} 