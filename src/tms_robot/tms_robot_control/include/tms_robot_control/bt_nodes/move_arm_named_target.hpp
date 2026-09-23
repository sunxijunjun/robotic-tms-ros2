#pragma once

#include <string>
#include <behaviortree_cpp/action_node.h>

class MoveArmNamedTargetNode : public BT::StatefulActionNode {
public:
  MoveArmNamedTargetNode(const std::string & name, const BT::NodeConfig & config);
  static BT::PortsList providedPorts();
  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  std::string target_name_;
  std::string planning_group_;
};