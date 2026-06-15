#include "increase_clearance/plugins/increase_clearance_action.hpp"

namespace increase_clearance
{

IncreaseClearanceAction::IncreaseClearanceAction(
  const std::string & xml_tag_name,
  const std::string & action_name,
  const BT::NodeConfiguration & conf)
: BtActionNode<Action>(xml_tag_name, action_name, conf)
{
}

void IncreaseClearanceAction::on_tick()
{
  increment_recovery_count();
}

BT::NodeStatus IncreaseClearanceAction::on_success()
{
  setOutput("error_code_id", ActionResult::NONE);
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus IncreaseClearanceAction::on_aborted()
{
  if (result_.result) {
    setOutput("error_code_id", result_.result->error_code);
  } else {
    setOutput("error_code_id", ActionResult::UNKNOWN);
  }
  return BT::NodeStatus::FAILURE;
}

BT::NodeStatus IncreaseClearanceAction::on_cancelled()
{
  setOutput("error_code_id", ActionResult::NONE);
  return BT::NodeStatus::SUCCESS;
}

}  // namespace increase_clearance

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  BT::NodeBuilder builder =
    [](const std::string & name, const BT::NodeConfiguration & config)
    {
      return std::make_unique<increase_clearance::IncreaseClearanceAction>(
        name, "increase_clearance", config);
    };

  factory.registerBuilder<increase_clearance::IncreaseClearanceAction>(
    "IncreaseClearance", builder);
}
