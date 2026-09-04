// Copyright 2023 TIER IV, Inc.
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

#include "autoware/behavior_velocity_rtc_interface/experimental/scene_module_interface_with_rtc.hpp"

#include <autoware_utils/ros/uuid_helper.hpp>

#include <algorithm>
#include <limits>
#include <memory>
#include <vector>

namespace autoware::behavior_velocity_planner::experimental
{

SceneModuleInterfaceWithRTC::SceneModuleInterfaceWithRTC(
  const lanelet::Id module_id, rclcpp::Logger logger, rclcpp::Clock::SharedPtr clock,
  const std::shared_ptr<autoware_utils::TimeKeeper> time_keeper,
  const std::shared_ptr<planning_factor_interface::PlanningFactorInterface>
    planning_factor_interface)
: SceneModuleInterface(module_id, logger, clock, time_keeper, planning_factor_interface),
  activated_(false),
  safe_(false),
  distance_(std::numeric_limits<double>::lowest())
{
}

SceneModuleManagerInterfaceWithRTC::SceneModuleManagerInterfaceWithRTC(
  rclcpp::Node & node, const char * module_name, const bool enable_rtc)
: SceneModuleManagerInterface<SceneModuleInterfaceWithRTC>(node, module_name),
  rtc_interface_(&node, module_name, enable_rtc),
  objects_of_interest_marker_interface_(&node, module_name)
{
  module_activation_pub_ = node.create_publisher<autoware_internal_debug_msgs::msg::StringStamped>(
    "/planning/module_activation", rclcpp::QoS{10});
}

void SceneModuleManagerInterfaceWithRTC::publishModuleActivation(
  const std::string & event, const std::string & detail)
{
  if (!module_activation_pub_) {
    return;
  }
  autoware_internal_debug_msgs::msg::StringStamped msg;
  msg.stamp = clock_->now();
  msg.data = "bvp::" + std::string{getModuleName()} + "|" + event + "|" + detail;
  module_activation_pub_->publish(msg);
}

void SceneModuleManagerInterfaceWithRTC::plan(
  Trajectory & path, const std_msgs::msg::Header & header,
  const std::vector<geometry_msgs::msg::Point> & left_bound,
  const std::vector<geometry_msgs::msg::Point> & right_bound, const PlannerData & planner_data)
{
  setActivation();
  modifyPathVelocity(path, header, left_bound, right_bound, planner_data);
  // One beacon per registered scene module. The module id is the map element the
  // module attached to, which is what makes the signal attributable; safe and
  // activated separate "declined" from "acted".
  for (const auto & scene_module : scene_modules_) {
    publishModuleActivation(
      "run", "id=" + std::to_string(scene_module->getModuleId()) +
               ",safe=" + (scene_module->isSafe() ? "1" : "0") +
               ",activated=" + (scene_module->isActivated() ? "1" : "0"));
  }
  sendRTC(header.stamp);
  publishObjectsOfInterestMarker();
}

void SceneModuleManagerInterfaceWithRTC::sendRTC(const Time & stamp)
{
  for (const auto & scene_module : scene_modules_) {
    const UUID uuid = getUUID(scene_module->getModuleId());
    const auto state = !scene_module->isActivated() && scene_module->isSafe()
                         ? State::WAITING_FOR_EXECUTION
                         : State::RUNNING;
    updateRTCStatus(uuid, scene_module->isSafe(), state, scene_module->getDistance(), stamp);
  }
  publishRTCStatus(stamp);
}

void SceneModuleManagerInterfaceWithRTC::setActivation()
{
  for (const auto & scene_module : scene_modules_) {
    const UUID uuid = getUUID(scene_module->getModuleId());
    scene_module->setActivation(rtc_interface_.isActivated(uuid));
    scene_module->setRTCEnabled(rtc_interface_.isRTCEnabled(uuid));
  }
}

UUID SceneModuleManagerInterfaceWithRTC::getUUID(const lanelet::Id & module_id) const
{
  if (map_uuid_.count(module_id) == 0) {
    const UUID uuid;
    return uuid;
  }
  return map_uuid_.at(module_id);
}

void SceneModuleManagerInterfaceWithRTC::generate_uuid(const lanelet::Id & module_id)
{
  map_uuid_.emplace(module_id, autoware_utils::generate_uuid());
}

void SceneModuleManagerInterfaceWithRTC::removeUUID(const lanelet::Id & module_id)
{
  const auto result = map_uuid_.erase(module_id);
  if (result == 0) {
    RCLCPP_WARN_STREAM(logger_, "[removeUUID] module_id = " << module_id << " is not registered.");
  }
}

void SceneModuleManagerInterfaceWithRTC::publishObjectsOfInterestMarker()
{
  for (const auto & scene_module : scene_modules_) {
    const auto objects = scene_module->getObjectsOfInterestData();
    for (const auto & obj : objects) {
      objects_of_interest_marker_interface_.insertObjectData(obj.pose, obj.shape, obj.color);
    }
    scene_module->clearObjectsOfInterestData();
  }

  objects_of_interest_marker_interface_.publishMarkerArray();
}

void SceneModuleManagerInterfaceWithRTC::deleteExpiredModules(
  const Trajectory & path, const PlannerData & planner_data)
{
  const auto isModuleExpired = getModuleExpiredFunction(path, planner_data);

  auto itr = scene_modules_.begin();
  while (itr != scene_modules_.end()) {
    if (isModuleExpired(*itr)) {
      const UUID uuid = getUUID((*itr)->getModuleId());
      updateRTCStatus(
        uuid, (*itr)->isSafe(), State::SUCCEEDED, std::numeric_limits<double>::lowest(),
        clock_->now());
      removeUUID((*itr)->getModuleId());
      registered_module_id_set_.erase((*itr)->getModuleId());
      itr = scene_modules_.erase(itr);
    } else {
      itr++;
    }
  }
}

template void SceneModuleManagerInterface<SceneModuleInterfaceWithRTC>::updateSceneModuleInstances(
  const Trajectory & path, const rclcpp::Time & stamp, const PlannerData & planner_data);
template void SceneModuleManagerInterface<SceneModuleInterfaceWithRTC>::modifyPathVelocity(
  Trajectory & path, const std_msgs::msg::Header & header,
  const std::vector<geometry_msgs::msg::Point> & left_bound,
  const std::vector<geometry_msgs::msg::Point> & right_bound, const PlannerData & planner_data);
template void SceneModuleManagerInterface<SceneModuleInterfaceWithRTC>::registerModule(
  const std::shared_ptr<SceneModuleInterfaceWithRTC> & scene_module,
  const PlannerData & planner_data);
}  // namespace autoware::behavior_velocity_planner::experimental
