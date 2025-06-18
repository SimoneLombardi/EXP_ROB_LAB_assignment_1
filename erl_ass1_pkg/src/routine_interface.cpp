#include <rclcpp/rclcpp.hpp>
#include <cstring>
#include <boost/algorithm/string.hpp>
#include "erl_ass1_pkg_interfaces/msg/marker_info.hpp"
#include "erl_ass1_pkg_interfaces/srv/robot_activation.hpp"
#include <ros2_aruco_interfaces/msg/aruco_markers.hpp>

using namespace std::chrono_literals;
using std::placeholders::_1;


int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "insert type of routine: 'camera' or 'robot'");
    std::string temp;
    std::getline(std::cin, temp);
    
    const char* command = temp.c_str();

    std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("routine_interface");
    rclcpp::Client<erl_ass1_pkg_interfaces::srv::RobotActivation>::SharedPtr client = 
        node->create_client<erl_ass1_pkg_interfaces::srv::RobotActivation>("/rbh_change");

    auto request = std::make_shared<erl_ass1_pkg_interfaces::srv::RobotActivation::Request>();

    const char* camera = "camera";
    const char* robot = "robot";
    
    if(strcmp(command, camera) == 0){
        request->activation_mode = 1;
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "sending request for camera routine");
    }else if(strcmp(command, robot) == 0){
        request->activation_mode = 2;
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "sending request for robot routine");
    }else{
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "errore comando");
    }

    while(!client->wait_for_service(1s)){
        if(!rclcpp::ok()){
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
            return 0;  
        }
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
    }

    auto result = client->async_send_request(request);
    //wait for result
    if (rclcpp::spin_until_future_complete(node, result) == rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "responce: %s", result.get()->message.c_str());
    } else {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "routine error");
    }

    rclcpp::shutdown();
    return 0;
}

