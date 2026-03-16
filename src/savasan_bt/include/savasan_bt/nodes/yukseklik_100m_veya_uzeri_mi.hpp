#pragma once
#include <behaviortree_cpp/condition_node.h>
#include <iostream>

class Yukseklik100mVeyaUzeriMi : public BT::ConditionNode
{
public:
    Yukseklik100mVeyaUzeriMi(const std::string &name, const BT::NodeConfig &config)
        : BT::ConditionNode(name, config) {}

    static BT::PortsList providedPorts() { return {}; }

    BT::NodeStatus tick() override
    {
        std::cout << "[CONDITION] Yukseklik100mVeyaUzeriMi -> FAILURE (dummy)" << std::endl;
        return BT::NodeStatus::FAILURE;
    }
};
