#include "savasan_bt/nodes/haberlesme_koptu_mu.hpp"
#include <iostream>

HaberlesmeKoptuMu::HaberlesmeKoptuMu(const std::string &name, const BT::NodeConfig &config)
    : BT::ConditionNode(name, config), connected_(true), msg_received_(false)
{
    node_ = rclcpp::Node::make_shared("bt_haberlesme_kontrol");
    sub_ = node_->create_subscription<mavros_msgs::msg::State>(
        "/mavros/state", 10,
        [this](const mavros_msgs::msg::State::SharedPtr msg) {
            connected_ = msg->connected;
            last_msg_time_ = node_->now();
            msg_received_ = true;
        });
}

BT::PortsList HaberlesmeKoptuMu::providedPorts() { return {}; }

BT::NodeStatus HaberlesmeKoptuMu::tick()
{
    rclcpp::spin_some(node_);

    // Hiç mesaj gelmediyse veya 3 saniyedir mesaj gelmiyorsa → koptu
    if (!msg_received_ || (node_->now() - last_msg_time_).seconds() > 3.0) {
        std::cout << "[KURTARMA] Haberlesme KOPTU!" << std::endl;
        return BT::NodeStatus::SUCCESS;
    }

    if (!connected_) {
        std::cout << "[KURTARMA] Haberlesme KOPTU!" << std::endl;
        return BT::NodeStatus::SUCCESS;
    }

    return BT::NodeStatus::FAILURE;
}