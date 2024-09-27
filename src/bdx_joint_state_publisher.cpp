#include "bdx_joint_state_publisher/bdx_joint_state_publisher.hpp"

using namespace std::chrono_literals;

BDXJointStatePublisher::BDXJointStatePublisher() : Node("bdx_joint_state_publisher")
{
    // Initialize joint names
    joint_names_ = {
        "head_neck_pitch", "head_pitch", "head_roll", "head_yaw",
        "hip_abduction_left", "hip_abduction_right", "hip_flexion_left", 
        "hip_flexion_right", "hip_rotation_left", "hip_rotation_right",
        "thigh_joint_left", "thigh_joint_right", "toe_joint_left", "toe_joint_right"
    };

    // Set initial positions, velocities, and efforts to zero
    for (const auto &name : joint_names_)
    {
        joint_positions_[name] = 0.0;
        joint_velocities_[name] = 0.0;
        joint_efforts_[name] = 0.0;
    }

    // Create subscribers for each joint topic dynamically
    for (const auto &name : joint_names_) {
        std::string topic_name = "/" + name + "_status";
        joint_subscribers_[name] = this->create_subscription<sensor_msgs::msg::JointState>(
            topic_name, 10, [this, name](const sensor_msgs::msg::JointState::SharedPtr msg) {
                this->joint_callback(msg, name);
            }
        );
    }

    // Publisher for the /joint_states topic
    publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);

    // Timer to publish joint states periodically (every 100ms)
    timer_ = this->create_wall_timer(100ms, std::bind(&BDXJointStatePublisher::publish_joint_states, this));
}

void BDXJointStatePublisher::joint_callback(const sensor_msgs::msg::JointState::SharedPtr msg, const std::string& joint_name)
{
    if (!msg->name.empty()) {
        for (size_t i = 0; i < msg->name.size(); ++i) {
            if (msg->name[i] == joint_name) {
                joint_positions_[joint_name] = msg->position.empty() ? 0.0 : msg->position[i];
                joint_velocities_[joint_name] = msg->velocity.empty() ? 0.0 : msg->velocity[i];
                joint_efforts_[joint_name] = msg->effort.empty() ? 0.0 : msg->effort[i];
            }
        }
    }
}

void BDXJointStatePublisher::publish_joint_states()
{
    auto message = sensor_msgs::msg::JointState();

    // Set the current time in the header
    message.header.stamp = this->get_clock()->now();

    // Populate joint names, positions, velocities, and efforts
    message.name = joint_names_;
    for (const auto &name : joint_names_) {
        message.position.push_back(joint_positions_[name]);
        message.velocity.push_back(joint_velocities_[name]);
        message.effort.push_back(joint_efforts_[name]);
    }

    // Publish the message
    publisher_->publish(message);

    RCLCPP_INFO(this->get_logger(), "Published /joint_states message");
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<BDXJointStatePublisher>());
    rclcpp::shutdown();
    return 0;
}
