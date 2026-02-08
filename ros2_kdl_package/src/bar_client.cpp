#include <functional>
#include <future>
#include <memory>
#include <string>
#include <sstream>
#include <thread>
#include <cmath>

#include "Eigen/Dense"
#include "std_msgs/msg/bool.hpp"
#include "ros2_package_interfaces/msg/marker_activate_command.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

#include "ros2_kdl_package/action/ros2_kdl_traj.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Quaternion.h>

using Ros2KdlTraj = ros2_kdl_package::action::Ros2KdlTraj;
using NavigateToPose = nav2_msgs::action::NavigateToPose;
using GoalHandleRos2KdlTraj = rclcpp_action::ClientGoalHandle<Ros2KdlTraj>;
using GoalHandleNavigateToPose = rclcpp_action::ClientGoalHandle<NavigateToPose>;
using FloatArray = std_msgs::msg::Float64MultiArray;
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

			// subscriber to the aruco poses
			marker_available_ = false;
			marker1_pose_subscriber_ = this->create_subscription<geometry_msgs::msg::Pose>(
				"/aruco_double/pose",   // topic del marker
				10,                     // queue size
				std::bind(&KdlBarClient::marker1_pose_callback, this, _1)
			);

			marker2_pose_subscriber_ = this->create_subscription<geometry_msgs::msg::Pose>(
				"/aruco_double/pose2",  // topic del marker
				10,                     // queue size
				std::bind(&KdlBarClient::marker2_pose_callback, this, _1)
			);

			// Create publisher to command the aruco movement
			markerGrabActivation_ = this->create_publisher<ros2_package_interfaces::msg::MarkerActivateCommand>("/marker/activate_grab", 10);

			// Crate publisher to close the fingers of the gripper
			closeGripperPublisher_ = this->create_publisher<FloatArray>("/iiwa/gripper_controller/commands", 10);

			// tf variables to calculate the coordinates to grab the aruco
			tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
			tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

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

				ros2_package_interfaces::msg::MarkerActivateCommand msg;
				if(client_choice == 1){
					barman_send_goal(choice_one);
					msg.marker_id = "arucotag1";

				}else{
					barman_send_goal(choice_one);
					msg.marker_id = "arucotag2";
				}
				rclcpp::sleep_for(std::chrono::seconds(2));

				pose_to_grab = calculate_grab_pose();
				barman_send_goal(pose_to_grab);
				rclcpp::sleep_for(std::chrono::seconds(2));

				// now we can activate the grab procedure: create an apposita function
				msg.active = true;
				msg.reference_frame = "stereo_gazebo_left_camera_optical_frame";
				msg.world_frame = "world";
				msg.offset.position = current_marker_pose_.position;
				msg.offset.orientation = current_marker_pose_.orientation;
				markerGrabActivation_->publish(msg);
				rclcpp::sleep_for(std::chrono::seconds(2));

				// gripper closing
				std_msgs::msg::Float64MultiArray grip_msg;
				grip_msg.data = grip_;
				closeGripperPublisher_->publish(grip_msg);
				rclcpp::sleep_for(std::chrono::seconds(2));

				// ritorno alla posizione iniziale
				barman_send_goal(center);

				// qui fermo il marker
				msg.active = false;
				markerGrabActivation_->publish(msg);
				rclcpp::sleep_for(std::chrono::seconds(3));

				// gripper opening
				grip_ = {0.0, 0.0};
				grip_msg.data = grip_;
				closeGripperPublisher_->publish(grip_msg);
				rclcpp::sleep_for(std::chrono::seconds(2));

				// base link e world hanno la stessa orientazione: devo trasformare last orientation in world
				geometry_msgs::msg::PoseStamped obj;
				geometry_msgs::msg::PoseStamped pose_in_world_;
				obj.header.frame_id = "stereo_gazebo_left_camera_optical_frame";
				obj.pose.position = current_marker_pose_.position;
				obj.pose.orientation = current_marker_pose_.orientation;
				obj.header.stamp = rclcpp::Time(0);
				pose_in_world_ = tf_buffer_->transform(
					obj,
					"world",
					tf2::durationFromSec(1)
				);

				tf2::Quaternion q;
				tf2::fromMsg(pose_in_world_.pose.orientation, q);

				// riattivo il marker per fra2mo: definisco l'offset rispetto al base_link da trasformare in coordinate map
				msg.active = true;
				msg.reference_frame = "base_link";
				msg.world_frame = "map";
				msg.offset.position.x = 0.0;
				msg.offset.position.y = 0.0;
				msg.offset.position.z = pose_in_world_.pose.position.z - 0.059;

				msg.offset.orientation = tf2::toMsg(q);
				markerGrabActivation_->publish(msg);

				if(table_number==1){
					waiter_send_goal(tavolo1);
				}else{
					waiter_send_goal(tavolo2);
				}

				// consegna
				msg.reference_frame = "world";
				msg.world_frame = "world";
				if(table_number == 1){
					msg.offset.position.x = -2;
					msg.offset.position.y =  1;
					msg.offset.position.z =  1;

					msg.offset.orientation = tf2::toMsg(q);
				}else if(table_number == 2){
					msg.offset.position.x = -1;
					msg.offset.position.y = -2;
					msg.offset.position.z =  1;

					msg.offset.orientation = tf2::toMsg(q);
				}
				markerGrabActivation_->publish(msg);

				rclcpp::sleep_for(std::chrono::seconds(3));

				msg.active = false;
				markerGrabActivation_->publish(msg);
				waiter_send_goal(bancone);
			}
		}

    private:

		void marker1_pose_callback(geometry_msgs::msg::Pose::ConstSharedPtr msg)
		{           
			// Salva la posa
			if(client_choice == 1){
				previous_marker_pose_ = current_marker_pose_;
				current_marker_pose_ = *msg; 
				marker_available_ = true;

				RCLCPP_INFO(this->get_logger(),
					"Marker local -> x: %.3f, y: %.3f, z: %.3f",
					current_marker_pose_.position.x,
					current_marker_pose_.position.y,
					current_marker_pose_.position.z
				);
			} 			
		}

		void marker2_pose_callback(geometry_msgs::msg::Pose::ConstSharedPtr msg)
		{           
			// Salva la posa
			if(client_choice == 2){
				previous_marker_pose_ = current_marker_pose_;
				current_marker_pose_ = *msg; 
				marker_available_ = true;

				RCLCPP_INFO(this->get_logger(),
					"Marker local -> x: %.3f, y: %.3f, z: %.3f",
					current_marker_pose_.position.x,
					current_marker_pose_.position.y,
					current_marker_pose_.position.z
				);
			} 
		}


		void barman_send_goal(const Eigen::Vector4d& choice)
		{
			if (!iiwa_client_->wait_for_action_server(std::chrono::seconds(15))) {
				RCLCPP_ERROR(this->get_logger(), "Action server not available");
				return;
			}

			Ros2KdlTraj::Goal goal_msg;
			goal_msg.x = choice(0);
			goal_msg.y = choice(1);
			goal_msg.z = choice(2);
			goal_msg.yaw = choice(3);
			goal_msg.traj_duration = T;

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

      	}

		void waiter_send_goal(const geometry_msgs::msg::PoseStamped & goal_pose)
		{
			// Attendi che l'action server Nav2 sia pronto
			if (!nav2_client_->wait_for_action_server(std::chrono::seconds(10))) {
				RCLCPP_ERROR(this->get_logger(), "NavigateToPose action server non disponibile");
				return;
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
		}

		// function that calculates the coordinates of the object to grab
		Eigen::Vector4d calculate_grab_pose(){
			geometry_msgs::msg::PoseStamped obj_to_grab;
			geometry_msgs::msg::PoseStamped pose_in_world_;
			double r = 0.21;
			double xm = current_marker_pose_.position.x;
			double ym = current_marker_pose_.position.y;
			double zm = current_marker_pose_.position.z;

			// voglio che iiwa stia:
			// - alla stessa altezza del marker;
			// - a distanza fissata r (disegna una circonferenza attorno al centro del marker)
			// - sul segmento che congiunge il frame camera e il centro del marker
			// - yaw : orientazione della direzione della retta che connette i centri, nel frame del controllore (speculare al world)

			// distanza camera-marker nel piano x-z
			double norm = std::sqrt(xm*xm + zm*zm);
			if(norm < 1e-6){
				RCLCPP_ERROR(this->get_logger(), "Marker troppo vicino alla camera");
				return Eigen::Vector4d::Zero();
			}
			
			double x_ = (1.0 - r / norm) * xm;	// uso il  meno perché devo prendere l'intersezione più vicina
			double z_ = (1.0 - r / norm) * zm;
			double y_ = ym;  					// stessa altezza del marker

			// definisco la posa nel frame camera
			obj_to_grab.header.frame_id = "stereo_gazebo_left_camera_optical_frame";
			obj_to_grab.header.stamp = rclcpp::Time(0);

			obj_to_grab.pose.position.x = x_;
			obj_to_grab.pose.position.y = y_;
			obj_to_grab.pose.position.z = z_;

			// yaw = direzione della retta camera → marker
			double angle = std::atan2(xm, zm);  				// è rispetto all'asse z del piano xz dello stereog. 
			tf2::Quaternion q_m;
			q_m.setRPY(0.0, -1.57 + angle, 0.0);				// correggo il pitch in stereog. (la sua origine è lungo x)
			obj_to_grab.pose.orientation = tf2::toMsg(q_m);		// orientazione desiderata nel frame stereo gazebo

			// stampa della trasformazione world → stereo gazebo
			geometry_msgs::msg::TransformStamped tf_cam_world;
			tf_cam_world = tf_buffer_->lookupTransform(
				"world",
				"stereo_gazebo_left_camera_optical_frame",
				rclcpp::Time(0)
			);
			RCLCPP_INFO(this->get_logger(),
				"TF camera->world: trans = [%.3f, %.3f, %.3f]",
				tf_cam_world.transform.translation.x,
				tf_cam_world.transform.translation.y,
				tf_cam_world.transform.translation.z
			);

			tf2::Quaternion q;
			tf2::fromMsg(tf_cam_world.transform.rotation, q);

			double ro, p, y;
			tf2::Matrix3x3(q).getRPY(ro, p, y);

			RCLCPP_INFO(this->get_logger(),
				"TF camera->world RPY = [roll=%.3f, pitch=%.3f, yaw=%.3f]",
				ro, p, y
			);

			// ora calcolo posizione e orientazione in world
			obj_to_grab.header.stamp = rclcpp::Time(0);
            pose_in_world_ = tf_buffer_->transform(
                obj_to_grab,
                "world",
                tf2::durationFromSec(1)
            );

			tf2::Quaternion q_world;
			tf2::fromMsg(pose_in_world_.pose.orientation, q_world);

			double roll_w, pitch_w, yaw_w;
			tf2::Matrix3x3(q_world).getRPY(roll_w, pitch_w, yaw_w);

			// calcolo dello yaw nel frame del controller, speculare al world, mi serve per il comando
			tf2::Transform controller_frame;
			controller_frame.setOrigin(tf2::Vector3(1.0, 0.0, 0.0)); // posizione
			tf2::Quaternion q_c;
			q_c.setRPY(0.0, 0.0, M_PI);                   		 	 // orientazione
			controller_frame.setRotation(q_c);

			tf2::Quaternion q_controller_local = controller_frame.getRotation().inverse() * q_world;
			double roll_c, pitch_c, yaw_c;
			tf2::Matrix3x3(q_controller_local).getRPY(roll_c, pitch_c, yaw_c);

			RCLCPP_INFO(this->get_logger(),
				"Orientation in controller frame RPY = [roll=%.3f, pitch=%.3f, yaw=%.3f]",
				roll_c, pitch_c, yaw_c);

			Eigen::Vector4d result;
			result << pose_in_world_.pose.position.x,
					pose_in_world_.pose.position.y,
					pose_in_world_.pose.position.z,
					yaw_c;

			RCLCPP_INFO(this->get_logger(),
				"Grab pose -> x: %.3f, y: %.3f, z: %.3f, angle: %.3f",
				result(0), result(1), result(2), result(3)
			);
			return result;
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
		rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr marker1_pose_subscriber_;
		rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr marker2_pose_subscriber_;
		rclcpp::Publisher<ros2_package_interfaces::msg::MarkerActivateCommand>::SharedPtr markerGrabActivation_;
		rclcpp::Publisher<FloatArray>::SharedPtr closeGripperPublisher_;

		// barman variables
		int client_choice;
		Eigen::Vector4d choice_one{0.6105, 0.6746, 1.038, -1.57};	// traiettoria non perfetta, non portare agli angoli del quadrato
		Eigen::Vector4d choice_two{0.6105, -0.6746, 1.038, 1.57};
		Eigen::Vector4d center{0.221, 0.0, 1.138, 0.0};
		Eigen::Vector4d pose_to_grab;
		std::vector<double> grip_ = {0.03, 0.03};
		double T = 20;
		bool iiwa_goal_done;

		// tf for the targeted pose
		std::shared_ptr<tf2_ros::Buffer> tf_buffer_;                // memoria delle trasformazioni
		std::shared_ptr<tf2_ros::TransformListener> tf_listener_;   // sottoscrive ai topic tf

		// waiter variables
		geometry_msgs::msg::PoseStamped tavolo1;
		geometry_msgs::msg::PoseStamped tavolo2;
		geometry_msgs::msg::PoseStamped bancone;
		bool nav2_goal_done_;

		// aruco marker subscription 
		bool marker_available_;
		geometry_msgs::msg::Pose current_marker_pose_;
		geometry_msgs::msg::Pose previous_marker_pose_; // for debug
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