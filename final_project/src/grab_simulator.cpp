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
        timer_ = this->create_wall_timer(std::chrono::milliseconds(100), 
            std::bind(&MarkerHandler::timer_callback, this));

        // callback to handle the service response
        response_callback_ = std::bind(&MarkerHandler::handle_service_response, this, std::placeholders::_1);

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

    void grab_callback(const ros2_package_interfaces::msg::MarkerActivateCommand::SharedPtr msg)
    {
        executing = msg->active;
        markerid = msg->marker_id;
        reference_frame_ = msg->reference_frame;
        world_frame_ = msg->world_frame;
        marker_offset_.x = msg->offset.position.x;
        marker_offset_.y = msg->offset.position.y;
        marker_offset_.z = msg->offset.position.z;
        marker_orientation_ = msg->offset.orientation;

        RCLCPP_INFO(this->get_logger(), "Marker id %s", markerid.c_str());
        RCLCPP_INFO(this->get_logger(), "executing %s", executing ? "true" : "false");
        RCLCPP_INFO(this->get_logger(),
            "Marker offset position -> x: %.3f, y: %.3f, z: %.3f",
            marker_offset_.x,
            marker_offset_.y,
            marker_offset_.z
        );


        marker_pose_local_.header.frame_id = reference_frame_;
        marker_pose_local_.pose.position.x = marker_offset_.x;
        marker_pose_local_.pose.position.y = marker_offset_.y;
        marker_pose_local_.pose.position.z = marker_offset_.z;
        marker_pose_local_.pose.orientation = marker_orientation_;

    }

    void timer_callback(){
        if(!executing){
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
    rclcpp::Subscription<ros2_package_interfaces::msg::MarkerActivateCommand>::SharedPtr grab_subscriber_;
    rclcpp::Client<ros_gz_interfaces::srv::SetEntityPose>::SharedPtr set_pose_client_;
    rclcpp::TimerBase::SharedPtr timer_; 
    std::function<void(rclcpp::Client<ros_gz_interfaces::srv::SetEntityPose>::SharedFuture)> response_callback_;

    // tf for the targeted pose
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;                // memoria delle trasformazioni
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;   // sottoscrive ai topic tf

    // target position where to update the marker
    std::string reference_frame_; 
    std::string world_frame_;
    geometry_msgs::msg::Vector3 marker_offset_;
    geometry_msgs::msg::Quaternion marker_orientation_;
    std::string markerid;
    bool executing;  

    geometry_msgs::msg::PoseStamped marker_pose_local_;
    geometry_msgs::msg::PoseStamped marker_pose_world_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MarkerHandler>());
  rclcpp::shutdown();
  return 0;
}
