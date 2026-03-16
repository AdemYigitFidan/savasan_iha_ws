#pragma once
#include <behaviortree_cpp/condition_node.h>
#include <iostream>

class HedefeXKadarYaklastinMi : public BT::ConditionNode
{
public:
    HedefeXKadarYaklastinMi(const std::string &name, const BT::NodeConfig &config)
        : BT::ConditionNode(name, config) {}

    static BT::PortsList providedPorts() { return {}; }

    BT::NodeStatus tick() override
    {
        std::cout << "[CONDITION] HedefeXKadarYaklastinMi -> FAILURE (dummy)" << std::endl;
        return BT::NodeStatus::FAILURE;
    }
};
