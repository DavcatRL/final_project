#include <functional>
#include <future>
#include <memory>
#include <string>
#include <sstream>
#include <thread>

#include "Eigen/Dense"
#include "std_msgs/msg/bool.hpp"
#include "ros2_package_interfaces/msg/marker_activate_command.hpp"

#include "ros2_kdl_package/action/ros2_kdl_traj.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

using Ros2KdlTraj = ros2_kdl_package::action::Ros2KdlTraj;
using NavigateToPose = nav2_msgs::action::NavigateToPose;
using GoalHandleRos2KdlTraj = rclcpp_action::ClientGoalHandle<Ros2KdlTraj>;
using GoalHandleNavigateToPose = rclcpp_action::ClientGoalHandle<NavigateToPose>;
using namespace std::placeholders;

class KdlBarClient : public rclcpp::Node
{
    public:
		KdlBarClient() : KdlBarClient(rclcpp::NodeOptions()) {} 

		explicit KdlBarClient(const rclcpp::NodeOptions & options)
		: Node("kdl_bar_client", options)
		{
			// dichiarazione del client iiwa nel nodo
			this->iiwa_client_ = rclcpp_action::create_client<Ros2KdlTraj>(this, "ros2_kdl_traj");

			// dichiarazione del client nav2 nel nodo
			this->nav2_client_ = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");

			// Create publisher to command the aruco movement
			markerGrabActivation_ = this->create_publisher<ros2_package_interfaces::msg::MarkerActivateCommand>("/marker/activate_grab", 10);

			// tavolo 1 initialization
			tavolo1.header.frame_id = "map";	// origin: (-6.44,-5.32)

			tavolo1.pose.position.x =  -2.2;	// in Gazebo
			tavolo1.pose.position.y =  0.25;	// in Gazebo
			tavolo1.pose.position.z =  0;

			// yaw = pi
			tavolo1.pose.orientation.x = 0.0;
			tavolo1.pose.orientation.y = 0.0;
			tavolo1.pose.orientation.z = 1.0;
			tavolo1.pose.orientation.w = 0.0;

			// tavolo 2 initialization
			tavolo2.header.frame_id = "map";	// origin: (-6.44,-5.32)

			tavolo2.pose.position.x =  -1.2;	// in Gazebo
			tavolo2.pose.position.y =  -1.25;	// in Gazebo
			tavolo2.pose.position.z =  0;

			// yaw = pi
			tavolo2.pose.orientation.x = 0.0;
			tavolo2.pose.orientation.y = 0.0;
			tavolo2.pose.orientation.z = 1.0;
			tavolo2.pose.orientation.w = 0.0;
			
			// bancone initialization
			bancone.header.frame_id = "map";	// origin: (-6.44,-5.32)

			bancone.pose.position.x =  0;	// in Gazebo
			bancone.pose.position.y =  0;	// in Gazebo
			bancone.pose.position.z =  0;

			// yaw = 0
			bancone.pose.orientation.x = 0.0;
			bancone.pose.orientation.y = 0.0;
			bancone.pose.orientation.z = 0.0;
			bancone.pose.orientation.w = 1.0;			
		}

		void keyboard_loop() {
			while (rclcpp::ok()) {
				int client_choice;
				int table_number;
				std::cout << "Ready to order? Enter your table number (1 or 2, 0 to exit): ";
				std::cin >> table_number;
				std::cout << "Enter your choice (1 or 2, 0 to exit): ";
				std::cin >> client_choice;

				if(!std::cin){
					std::cout << "Input error. Exiting...\n";
					break;
				}
				if((client_choice!=1 && client_choice!=2) || (table_number!=1 && table_number!=2)){
					std::cout << "Input error. Your choice must be 1 or 2. Exiting...\n";
					break;					
				}

				barman_send_goal(client_choice);

				if(table_number==1){
					waiter_send_goal(tavolo1, client_choice);
				}else{
					waiter_send_goal(tavolo2, client_choice);
				}

				// consegna
				ros2_package_interfaces::msg::MarkerActivateCommand msg;
				msg.active = false;
				if(client_choice == 1){
					msg.marker_id = "arucotag1";
				}else if(client_choice == 2){
					msg.marker_id = "arucotag2";
				}
				if(table_number == 1){
					msg.robot = "table1";
				}else if(table_number == 2){
					msg.robot = "table2";
				}
				markerGrabActivation_->publish(msg);

				rclcpp::sleep_for(std::chrono::seconds(5));

				waiter_send_goal(bancone, 0);
			}
		}

    private:

		void barman_send_goal(int client_choice)
		{
			if (!iiwa_client_->wait_for_action_server(std::chrono::seconds(15))) {
				RCLCPP_ERROR(this->get_logger(), "Action server not available");
				return;
			}

			Ros2KdlTraj::Goal goal_msg;
			ros2_package_interfaces::msg::MarkerActivateCommand msg;
			if(client_choice == 1){
				goal_msg.x = choice_one(0);
				goal_msg.y = choice_one(1);
				goal_msg.z = choice_one(2);
				goal_msg.yaw = choice_one(3);
				goal_msg.traj_duration = T;

				msg.active = true;
				msg.marker_id = "arucotag1";
				msg.robot = "iiwa";
			}else{
				goal_msg.x = choice_two(0);
				goal_msg.y = choice_two(1);
				goal_msg.z = choice_two(2);
				goal_msg.yaw = choice_two(3);
				goal_msg.traj_duration = T;

				msg.active = true;
				msg.marker_id = "arucotag2";
				msg.robot = "iiwa";
			}

			auto send_goal_options = rclcpp_action::Client<Ros2KdlTraj>::SendGoalOptions();
			send_goal_options.goal_response_callback = 
				std::bind(&KdlBarClient::iiwa_goal_response_callback, this, _1);
			send_goal_options.feedback_callback = 
				std::bind(&KdlBarClient::iiwa_feedback_callback, this, _1, _2);
			send_goal_options.result_callback = 
				std::bind(&KdlBarClient::iiwa_result_callback, this, _1);

			iiwa_client_->async_send_goal(goal_msg, send_goal_options);

			iiwa_goal_done = false;
			while (!iiwa_goal_done && rclcpp::ok()) {
				std::this_thread::sleep_for(std::chrono::milliseconds(100));
			}

			// ritorno alla posizione iniziale

			// avviso il marker che può iniziare a muoversi
			markerGrabActivation_->publish(msg);

			rclcpp::sleep_for(std::chrono::seconds(5));

			goal_msg.x = center(0);
			goal_msg.y = center(1);
			goal_msg.z = center(2);
			goal_msg.yaw = center(3);
			goal_msg.traj_duration = T;

			iiwa_client_->async_send_goal(goal_msg, send_goal_options);

			iiwa_goal_done = false;
			while (!iiwa_goal_done && rclcpp::ok()) {
				std::this_thread::sleep_for(std::chrono::milliseconds(100));
			}

			// qui fermo il marker
			msg.active = false;
			markerGrabActivation_->publish(msg);
      	}

		void waiter_send_goal(const geometry_msgs::msg::PoseStamped & goal_pose, int client_choice)
		{
			// Attendi che l'action server Nav2 sia pronto
			if (!nav2_client_->wait_for_action_server(std::chrono::seconds(10))) {
				RCLCPP_ERROR(this->get_logger(), "NavigateToPose action server non disponibile");
				return;
			}

			// Attivazione del grab
			ros2_package_interfaces::msg::MarkerActivateCommand msg;
			if(client_choice == 1){
				msg.active = true;
				msg.marker_id = "arucotag1";
				msg.robot = "fra2mo";
				markerGrabActivation_->publish(msg);
			}else if(client_choice == 2){
				msg.active = true;
				msg.marker_id = "arucotag2";
				msg.robot = "fra2mo";
				markerGrabActivation_->publish(msg);
			}

			// Costruisci il goal Nav2
			NavigateToPose::Goal goal_msg;
			goal_msg.pose = goal_pose;

			// Aggiorna il timestamp
			goal_msg.pose.header.stamp = this->get_clock()->now();

			auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
			send_goal_options.goal_response_callback = 
				std::bind(&KdlBarClient::nav_goal_response_callback, this, _1);
			send_goal_options.feedback_callback = 
				std::bind(&KdlBarClient::nav_feedback_callback, this, _1, _2);
			send_goal_options.result_callback = 
				std::bind(&KdlBarClient::nav_result_callback, this, _1);

			nav2_client_->async_send_goal(goal_msg, send_goal_options);

			// Attendi il completamento
			nav2_goal_done_=  false;
			while (!nav2_goal_done_ && rclcpp::ok()) {
				std::this_thread::sleep_for(std::chrono::milliseconds(100));
			}

			if(client_choice != 0){
				msg.active = false;
				markerGrabActivation_->publish(msg);
			}
		}

		// callbacks for the iiwa action
		void iiwa_goal_response_callback(const GoalHandleRos2KdlTraj::SharedPtr & goal_handle)
		{
			if (!goal_handle) {
				RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
			}
			else {
				RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
			}
		}

		void iiwa_feedback_callback(
			GoalHandleRos2KdlTraj::SharedPtr,
			const std::shared_ptr<const Ros2KdlTraj::Feedback> feedback)
		{
			RCLCPP_INFO(this->get_logger(),
				"Action feedback: x=%.3f y=%.3f z=%.3f yaw=%.3f advancement=%.1f%%",
				feedback->current_x, feedback->current_y, feedback->current_z, feedback->current_yaw, feedback->advancement);
		}

		void iiwa_result_callback(const GoalHandleRos2KdlTraj::WrappedResult & result_wrapper)
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

			iiwa_goal_done = true;
		}

		// callbacks for the nav2 action
		void nav_goal_response_callback(const GoalHandleNavigateToPose::SharedPtr & goal_handle)
		{
			if (!goal_handle) {
				RCLCPP_ERROR(this->get_logger(), "Nav2 goal was rejected by server");
			}
			else {
				RCLCPP_INFO(this->get_logger(), "Nav2 goal accepted by server, waiting for result");
			}
		}

		void nav_feedback_callback(
			GoalHandleNavigateToPose::SharedPtr,
			const std::shared_ptr<const NavigateToPose::Feedback> feedback)
		{
			RCLCPP_INFO(this->get_logger(),
				"Nav2 feedback: distance_remaining = %.3f m",
				feedback->distance_remaining);
		}

		void nav_result_callback(const GoalHandleNavigateToPose::WrappedResult & result)
		{
            switch (result.code) {
                case rclcpp_action::ResultCode::SUCCEEDED:
                    RCLCPP_INFO(this->get_logger(), "Nav2 goal completed!");
                    break;
                case rclcpp_action::ResultCode::ABORTED:
                    RCLCPP_ERROR(this->get_logger(), "Nav2 goal aborted");
                    break;
                case rclcpp_action::ResultCode::CANCELED:
                    RCLCPP_ERROR(this->get_logger(), "Nav2 goal canceled");
                    break;
                default:
                    RCLCPP_ERROR(this->get_logger(), "Unknown result code Nav2");
            }
			nav2_goal_done_ = true;
		}


		// dichiarazione variabili interne
		rclcpp_action::Client<Ros2KdlTraj>::SharedPtr iiwa_client_;
		rclcpp_action::Client<NavigateToPose>::SharedPtr nav2_client_;
		rclcpp::Publisher<ros2_package_interfaces::msg::MarkerActivateCommand>::SharedPtr markerGrabActivation_;

		// barman variables
		Eigen::Vector4d choice_one{0.6105, 0.6746, 1.038, -1.57};	// traiettoria non perfetta, non portare agli angoli del quadrato
		Eigen::Vector4d choice_two{0.6105, -0.6746, 1.038, 1.57};
		Eigen::Vector4d center{0.221, 0.0, 1.138, 0.0};
		double T = 20;
		bool iiwa_goal_done;

		// waiter variables
		geometry_msgs::msg::PoseStamped tavolo1;
		geometry_msgs::msg::PoseStamped tavolo2;
		geometry_msgs::msg::PoseStamped bancone;
		bool nav2_goal_done_;
};  


int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto client_node = std::make_shared<KdlBarClient>();

    // keyboard thread
    std::thread keyboard_thread(&KdlBarClient::keyboard_loop, client_node);

    // Spin the main node
    rclcpp::spin(client_node);

    keyboard_thread.join();
    rclcpp::shutdown();
    return 0;
} 