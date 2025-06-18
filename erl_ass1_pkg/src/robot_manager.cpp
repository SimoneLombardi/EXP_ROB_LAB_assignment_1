#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/int16.hpp>
#include <memory>
#include <chrono>
#include <string>
#include <rclcpp/timer.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/service.hpp>
#include <rclcpp/node.hpp>
#include <geometry_msgs/msg/twist.hpp>

#include "erl_ass1_pkg_interfaces/srv/robot_activation.hpp"
#include "ros2_aruco_interfaces/msg/aruco_markers.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;

class robot_manager : public rclcpp::Node
{
    public:
        robot_manager()
        : Node("robot_manager"), count_(0)
        {
            camera_joint_publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/controller_camera_joint/commands", 10);
            diff_drive_publisher_   = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

            stop_routine_sub_ = this->create_subscription<std_msgs::msg::Int16>(
                "/stop_routine", 10, std::bind(&robot_manager::stop_routine_callback, this, _1));

            timer_ = this->create_wall_timer(
                500ms, std::bind(&robot_manager::timer_callback, this));

            rbh_servServ_           = this->create_service<erl_ass1_pkg_interfaces::srv::RobotActivation>("rbh_change", std::bind(&robot_manager::rbh_callback, this, _1, _2));
            RCLCPP_INFO(this->get_logger(), "[ROBOT MANAGER] -- publisher and service online, start publish on camera joint");
        }

    private:
        void timer_callback(){
            auto dif_drive_msg = geometry_msgs::msg::Twist();
            auto cmr_joint_msg = std_msgs::msg::Float64MultiArray();

            if(robot_bh == 1){ // camera joint
                dif_drive_msg.angular.z = {0.0};
                cmr_joint_msg.data = {0.6};

                diff_drive_publisher_->publish(dif_drive_msg);
                camera_joint_publisher_->publish(cmr_joint_msg);
            }else if(robot_bh == 2){ // diff drive pub
                dif_drive_msg.angular.z = {-0.6};
                cmr_joint_msg.data = {0.0};

                diff_drive_publisher_->publish(dif_drive_msg);
                camera_joint_publisher_->publish(cmr_joint_msg);
            }else{
                //RCLCPP_INFO(this->get_logger(), "robot idle");
                dif_drive_msg.angular.z = {0.0};
                cmr_joint_msg.data = {0.0};

                diff_drive_publisher_->publish(dif_drive_msg);
                camera_joint_publisher_->publish(cmr_joint_msg);
            }
        }

        void rbh_callback(const std::shared_ptr<erl_ass1_pkg_interfaces::srv::RobotActivation::Request> request, 
                          const std::shared_ptr<erl_ass1_pkg_interfaces::srv::RobotActivation::Response> responce){

            if(request->activation_mode != 1 && request->activation_mode != 2){
                responce->message = "invalid activation mode, must be 0:stand by, 1:move camera or 2:move robot, continuing with the current state";
                RCLCPP_ERROR(this->get_logger(),"RECIVED INVALID REQUEST: %ld", request->activation_mode);
            }else{
                if(robot_bh == 0){
                    robot_bh = request->activation_mode;
                    responce->message = "[ROBOT MANAGER]Robot state changed to " + std::to_string(robot_bh);
                }else{
                    responce->message = "[ROBOT MANAGER]routine already started, wait completion" + std::to_string(robot_bh);
                }
            }
        }

        void stop_routine_callback(const std_msgs::msg::Int16::SharedPtr msg){
            RCLCPP_INFO(this->get_logger(),"Received routine end message, shutting down node: ");
            rclcpp::sleep_for(2s); // wait for a second to ensure the message is processed
            // Shut down the node
            rclcpp::shutdown();
        }

        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr camera_joint_publisher_;
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr diff_drive_publisher_;
        rclcpp::Service<erl_ass1_pkg_interfaces::srv::RobotActivation>::SharedPtr rbh_servServ_;
        rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr stop_routine_sub_;

        int robot_bh = 0; // questo sceglie se idle=0, camera=1 o diff drive=2
        size_t count_; // just a counter for the node
};

int main(int argc, char * argv[]){
    // init rclcpp
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<robot_manager>());
    rclcpp::shutdown();

    return 0;
}





