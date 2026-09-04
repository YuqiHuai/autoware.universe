// Copyright 2025 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "manager.hpp"

#include <memory>
#include <string>

namespace autoware::behavior_velocity_planner
{
NoDrivableLaneModuleManager::NoDrivableLaneModuleManager(rclcpp::Node & node)
: SceneModuleManagerInterface(node, getModuleName())
{
  const std::string ns(NoDrivableLaneModuleManager::getModuleName());
  planner_param_.stop_margin =
    experimental::get_or_declare_parameter<double>(node, ns + ".stop_margin");
  planner_param_.print_debug_info =
    experimental::get_or_declare_parameter<bool>(node, ns + ".print_debug_info");

  module_activation_pub_ =
    node.create_publisher<autoware_internal_debug_msgs::msg::StringStamped>(
      "/planning/module_activation", rclcpp::QoS{10});
}

void NoDrivableLaneModuleManager::plan(
  experimental::Trajectory & path, const std_msgs::msg::Header & header,
  const std::vector<geometry_msgs::msg::Point> & left_bound,
  const std::vector<geometry_msgs::msg::Point> & right_bound,
  const PlannerData & planner_data)
{
  experimental::SceneModuleManagerInterface<>::plan(path, header, left_bound, right_bound, planner_data);
  if (!module_activation_pub_) {
    return;
  }
  // One beacon per registered scene module; the id is the map element it
  // attached to, which is what makes the signal attributable.
  for (const auto & scene_module : scene_modules_) {
    autoware_internal_debug_msgs::msg::StringStamped msg;
    msg.stamp = header.stamp;
    msg.data = "bvp::no_drivable_lane|run|id=" + std::to_string(scene_module->getModuleId());
    module_activation_pub_->publish(msg);
  }
}

void NoDrivableLaneModuleManager::launchNewModules(
  const Trajectory & path, [[maybe_unused]] const rclcpp::Time & stamp,
  const PlannerData & planner_data)
{
  PathWithLaneId path_msg;
  path_msg.points = path.restore();

  for (const auto & ll : planning_utils::getLaneletsOnPath(
         path_msg, planner_data.route_handler_->getLaneletMapPtr(),
         planner_data.current_odometry->pose)) {
    const auto lane_id = ll.id();
    const auto module_id = lane_id;

    if (isModuleRegistered(module_id)) {
      continue;
    }

    const std::string no_drivable_lane_attribute = ll.attributeOr("no_drivable_lane", "no");
    if (no_drivable_lane_attribute != "yes") {
      continue;
    }

    registerModule(
      std::make_shared<NoDrivableLaneModule>(
        module_id, lane_id, planner_param_, logger_.get_child("no_drivable_lane_module"), clock_,
        time_keeper_, planning_factor_interface_),
      planner_data);
  }
}

std::function<bool(const std::shared_ptr<experimental::SceneModuleInterface> &)>
NoDrivableLaneModuleManager::getModuleExpiredFunction(
  const Trajectory & path, const PlannerData & planner_data)
{
  PathWithLaneId path_msg;
  path_msg.points = path.restore();

  const auto lane_id_set = planning_utils::getLaneIdSetOnPath(
    path_msg, planner_data.route_handler_->getLaneletMapPtr(), planner_data.current_odometry->pose);

  return [lane_id_set](const std::shared_ptr<experimental::SceneModuleInterface> & scene_module) {
    return lane_id_set.count(scene_module->getModuleId()) == 0;
  };
}

}  // namespace autoware::behavior_velocity_planner

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
  autoware::behavior_velocity_planner::NoDrivableLaneModulePlugin,
  autoware::behavior_velocity_planner::experimental::PluginInterface)
