#ifndef BDX_JOINT_STATE_PUBLISHER_HPP
#define BDX_JOINT_STATE_PUBLISHER_HPP

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>
#include <map>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

class BDXJointStatePublisher : public rclcpp::Node
{
public:
    BDXJointStatePublisher();

private:
    void publish_joint_states();
    void joint_callback(const sensor_msgs::msg::JointState::SharedPtr msg, const std::string& joint_name);

    std::map<std::string, double> joint_positions_;
    std::map<std::string, double> joint_velocities_;
    std::map<std::string, double> joint_efforts_;
    std::vector<std::string> joint_names_;

    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher_;
    std::map<std::string, rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr> joint_subscribers_;
    
    rclcpp::TimerBase::SharedPtr timer_;
};

#endif  // BDX_JOINT_STATE_PUBLISHER_HPP
