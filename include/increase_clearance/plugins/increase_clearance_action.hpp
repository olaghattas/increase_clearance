#ifndef INCREASE_CLEARANCE__PLUGINS__INCREASE_CLEARANCE_ACTION_HPP_
#define INCREASE_CLEARANCE__PLUGINS__INCREASE_CLEARANCE_ACTION_HPP_

#include <string>

#include "increase_clearance/action/increase_clearance.hpp"
#include "nav2_behavior_tree/bt_action_node.hpp"

namespace increase_clearance
{

class IncreaseClearanceAction
  : public nav2_behavior_tree::BtActionNode<increase_clearance::action::IncreaseClearance>
{
  using Action = increase_clearance::action::IncreaseClearance;
  using ActionResult = Action::Result;

public:
  IncreaseClearanceAction(
    const std::string & xml_tag_name,
    const std::string & action_name,
    const BT::NodeConfiguration & conf);

  void on_tick() override;

  BT::NodeStatus on_success() override;
  BT::NodeStatus on_aborted() override;
  BT::NodeStatus on_cancelled() override;

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts(
      {
        BT::OutputPort<ActionResult::_error_code_type>(
          "error_code_id", "The increase_clearance behavior server error code"),
      });
  }
};

}  // namespace increase_clearance

#endif  // INCREASE_CLEARANCE__PLUGINS__INCREASE_CLEARANCE_ACTION_HPP_
