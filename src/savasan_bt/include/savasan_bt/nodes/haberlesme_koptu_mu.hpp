#pragma once
#include <behaviortree_cpp/condition_node.h>
#include <rclcpp/rclcpp.hpp>
#include <mavros_msgs/msg/state.hpp>

class HaberlesmeKoptuMu : public BT::ConditionNode
{
public:
    HaberlesmeKoptuMu(const std::string &name, const BT::NodeConfig &config);
    static BT::PortsList providedPorts();
    BT::NodeStatus tick() override;

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr sub_;
    bool connected_;
    rclcpp::Time last_msg_time_;
    bool msg_received_;
};