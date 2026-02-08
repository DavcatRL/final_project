#include <functional>
#include <memory>
#include <string> 

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/bool.hpp"
#include "ros2_package_interfaces/msg/marker_activate_command.hpp"
#include "ros_gz_interfaces/srv/set_entity_pose.hpp"

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>


using namespace std::placeholders;


class MarkerHandler : public rclcpp::Node
{
public:
    MarkerHandler()
    : Node("grab_simulator")
    {
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // subscriber to the aruco poses
        marker_available_ = false;
        robotid = "none";
        marker1_pose_subscriber_ = this->create_subscription<geometry_msgs::msg::Pose>(
            "/aruco_double/pose",   // topic del marker
            10,                     // queue size
            std::bind(&MarkerHandler::marker1_pose_callback, this, _1)
        );

        marker2_pose_subscriber_ = this->create_subscription<geometry_msgs::msg::Pose>(
            "/aruco_double/pose2",  // topic del marker
            10,                     // queue size
            std::bind(&MarkerHandler::marker2_pose_callback, this, _1)
        );

        // subscriber to command the aruco movement
        grab_subscriber_ = this->create_subscription<ros2_package_interfaces::msg::MarkerActivateCommand>(
            "/marker/activate_grab",    // topic del marker
            10,                         // queue size
            std::bind(&MarkerHandler::grab_callback, this, _1)
        );

        // service client to update the aruco pose
        set_pose_client_ = this->create_client<ros_gz_interfaces::srv::SetEntityPose>(
            "/world/default/set_pose"
        );

        // periodic pose update request
        executing = false;
        fra2mo_executing = false;
        timer_ = this->create_wall_timer(std::chrono::milliseconds(100), 
            std::bind(&MarkerHandler::timer_callback, this));

        // callback to handle the service response
        response_callback_ = std::bind(&MarkerHandler::handle_service_response, this, std::placeholders::_1);

        tf2::Quaternion q;
        q.setRPY(M_PI, 0.0, 0.0);
        geometry_msgs::msg::Quaternion table_orientation_ = tf2::toMsg(q);

        table1.position.x = -2;
        table1.position.y =  1;
        table1.position.z =  1;
        table1.orientation = table_orientation_;

        table2.position.x = -1;
        table2.position.y = -2;
        table2.position.z =  1;
        table2.orientation = table_orientation_;

  }

private:
    bool poses_equal_operator(const geometry_msgs::msg::Pose& a, const geometry_msgs::msg::Pose& b, double tol = 1e-3)
    {
        return  std::fabs(a.position.x - b.position.x) < tol &&
                std::fabs(a.position.y - b.position.y) < tol &&
                std::fabs(a.position.z - b.position.z) < tol &&
                std::fabs(a.orientation.x - b.orientation.x) < tol &&
                std::fabs(a.orientation.y - b.orientation.y) < tol &&
                std::fabs(a.orientation.z - b.orientation.z) < tol &&
                std::fabs(a.orientation.w - b.orientation.w) < tol;
    }

    void marker1_pose_callback(geometry_msgs::msg::Pose::ConstSharedPtr msg)
    {           
        // Salva la posa per eventuale debug
        previous_marker_pose_ = current_marker_pose_;
        current_marker_pose_ = *msg; 
        marker_available_ = true;

        // stampa della pose del marker
        geometry_msgs::msg::PoseStamped camera_pose; 
        camera_pose.header.frame_id = "stereo_gazebo_left_camera_optical_frame";
        camera_pose.header.stamp = rclcpp::Time(0);
        camera_pose.pose.position.x = current_marker_pose_.position.x;
        camera_pose.pose.position.y = current_marker_pose_.position.y;
        camera_pose.pose.position.z = current_marker_pose_.position.z;
        camera_pose.pose.orientation = current_marker_pose_.orientation;

        geometry_msgs::msg::PoseStamped world_pose;
        world_pose = tf_buffer_->transform(
            camera_pose,
            "world",
            tf2::durationFromSec(0.1)
        );

        RCLCPP_INFO(this->get_logger(),
            "Marker WORLD -> x: %.3f, y: %.3f, z: %.3f",
            world_pose.pose.position.x,
            world_pose.pose.position.y,
            world_pose.pose.position.z
        );

    }

    void marker2_pose_callback(geometry_msgs::msg::Pose::ConstSharedPtr msg)
    {           
        // Salva la posa per eventuale debug
        previous_marker_pose_ = current_marker_pose_;
        current_marker_pose_ = *msg; 
        marker_available_ = true;

        // stampa della pose del marker
        geometry_msgs::msg::PoseStamped camera_pose; 
        camera_pose.header.frame_id = "stereo_gazebo_left_camera_optical_frame";
        camera_pose.header.stamp = rclcpp::Time(0);
        camera_pose.pose.position.x = current_marker_pose_.position.x;
        camera_pose.pose.position.y = current_marker_pose_.position.y;
        camera_pose.pose.position.z = current_marker_pose_.position.z;
        camera_pose.pose.orientation = current_marker_pose_.orientation;

        geometry_msgs::msg::PoseStamped world_pose;
        world_pose = tf_buffer_->transform(
            camera_pose,
            "world",
            tf2::durationFromSec(0.1)
        );

        RCLCPP_INFO(this->get_logger(),
            "Marker WORLD -> x: %.3f, y: %.3f, z: %.3f",
            world_pose.pose.position.x,
            world_pose.pose.position.y,
            world_pose.pose.position.z
        );
    }

    void grab_callback(const ros2_package_interfaces::msg::MarkerActivateCommand::SharedPtr msg)
    {
        executing = msg->active;
        markerid = msg->marker_id;
        robotid = msg->robot;

        RCLCPP_INFO(this->get_logger(), "Marker id %s", markerid.c_str());
        RCLCPP_INFO(this->get_logger(), "executing %s", executing ? "true" : "false");
        RCLCPP_INFO(this->get_logger(), "Robot id %s", robotid.c_str());

        if(robotid == "iiwa"){
            // target marker relative pose definition if iiwa is moving the aruco
            reference_frame_ = "stereo_gazebo_left_camera_optical_frame";
            world_frame_ = "world";

            marker_offset_.x = 0.0;
            marker_offset_.y = 0.0;
            marker_offset_.z = 0.5;

            tf2::Quaternion q;
            q.setRPY(M_PI, 0.0, 0.0);
            marker_orientation_ = tf2::toMsg(q);

            marker_pose_local_.header.frame_id = reference_frame_;
            marker_pose_local_.pose.position.x = marker_offset_.x;
            marker_pose_local_.pose.position.y = marker_offset_.y;
            marker_pose_local_.pose.position.z = marker_offset_.z;
            marker_pose_local_.pose.orientation = marker_orientation_;

            fra2mo_executing = false;
        }
        else if(robotid == "fra2mo"){
            // target marker relative pose definition if fra2mo is moving the aruco
            reference_frame_ = "base_link";
            world_frame_ = "map";

            marker_offset_.x = 0.0;
            marker_offset_.y = 0.0;
            marker_offset_.z = 1.078;

            tf2::Quaternion q;
            q.setRPY(M_PI/2, 0.0, 0.0); // q.setRPY(M_PI, 0.0, 0.0);
            marker_orientation_ = tf2::toMsg(q);

            marker_pose_local_.header.frame_id = reference_frame_;
            marker_pose_local_.pose.position.x = marker_offset_.x;
            marker_pose_local_.pose.position.y = marker_offset_.y;
            marker_pose_local_.pose.position.z = marker_offset_.z;
            marker_pose_local_.pose.orientation = marker_orientation_;

            fra2mo_executing = true;
        }
        else if (robotid == "table1"){
            // Aggiorna la pose del marker solo se il service è pronto
            if (set_pose_client_ && set_pose_client_->service_is_ready()) {
                auto request = std::make_shared<ros_gz_interfaces::srv::SetEntityPose::Request>();
                request->entity.name = markerid; 
                marker_pose_local_.header.stamp = rclcpp::Time(0);
                request->pose = table1;
                auto result_future = set_pose_client_->async_send_request(request, response_callback_);
            }
        }
        else if (robotid == "table2"){
            // Aggiorna la pose del marker solo se il service è pronto
            if (set_pose_client_ && set_pose_client_->service_is_ready()) {
                auto request = std::make_shared<ros_gz_interfaces::srv::SetEntityPose::Request>();
                request->entity.name = markerid; 
                marker_pose_local_.header.stamp = rclcpp::Time(0);
                request->pose = table2;
                auto result_future = set_pose_client_->async_send_request(request, response_callback_);
            }
        }
    }

    void timer_callback(){
        if(!marker_available_ || !executing){
            return;
        }
        
        // Aggiorna la pose del marker solo se il service è pronto
        if (set_pose_client_ && set_pose_client_->service_is_ready()) {

            auto request = std::make_shared<ros_gz_interfaces::srv::SetEntityPose::Request>();
            request->entity.name = markerid; 

            marker_pose_local_.header.stamp = rclcpp::Time(0);

            marker_pose_world_ = tf_buffer_->transform(
                marker_pose_local_,
                world_frame_,
                tf2::durationFromSec(0.1)
            );

            request->pose = marker_pose_world_.pose;

            auto result_future = set_pose_client_->async_send_request(request, response_callback_);

            if(!fra2mo_executing){
                marker_available_ = false;
            }
        }
    }

    void handle_service_response(rclcpp::Client<ros_gz_interfaces::srv::SetEntityPose>::SharedFuture future)
    {
        auto result = future.get();
        if(result->success)
            RCLCPP_INFO(this->get_logger(), "Marker aggiornato correttamente");
        else
            RCLCPP_WARN(this->get_logger(), "Aggiornamento marker fallito");
    }

    // class variables definition

    // functions
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr marker1_pose_subscriber_;
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr marker2_pose_subscriber_;
    rclcpp::Subscription<ros2_package_interfaces::msg::MarkerActivateCommand>::SharedPtr grab_subscriber_;
    rclcpp::Client<ros_gz_interfaces::srv::SetEntityPose>::SharedPtr set_pose_client_;
    rclcpp::TimerBase::SharedPtr timer_; 
    std::function<void(rclcpp::Client<ros_gz_interfaces::srv::SetEntityPose>::SharedFuture)> response_callback_;

    // tf for the targeted pose
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;                // memoria delle trasformazioni
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;   // sottoscrive ai topic tf

    // definition of the local and global target pose
    std::string reference_frame_; 
    std::string world_frame_;
    geometry_msgs::msg::Vector3 marker_offset_;
    geometry_msgs::msg::Quaternion marker_orientation_;
    geometry_msgs::msg::PoseStamped marker_pose_local_;
    geometry_msgs::msg::PoseStamped marker_pose_world_;

    // order characteristics
    std::string markerid;               // chosen product to move
    std::string robotid;                // waiter or barman: who is moving?
    geometry_msgs::msg::Pose table1;    // table 1 coordinates
    geometry_msgs::msg::Pose table2;    // table 2 coordinates

    // received aruco pose and control variables
    bool marker_available_;
    bool executing;
    bool fra2mo_executing;
    geometry_msgs::msg::Pose current_marker_pose_;  // for debug
    geometry_msgs::msg::Pose previous_marker_pose_; // for debug
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MarkerHandler>());
  rclcpp::shutdown();
  return 0;
}
