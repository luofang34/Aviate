// SPDX-License-Identifier: MIT
// Copyright (c) 2024 Aviate Project
//
// MotorFailurePlugin - Gazebo plugin for simulating motor failures

#include "MotorFailurePlugin.hh"

#include <gz/plugin/Register.hh>
#include <gz/sim/components/JointVelocityCmd.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/Util.hh>

#include <algorithm>
#include <regex>

namespace aviate
{
namespace gz_plugin
{

void MotorFailurePlugin::Configure(
    const gz::sim::Entity &entity,
    const std::shared_ptr<const sdf::Element> &sdf,
    gz::sim::EntityComponentManager &ecm,
    gz::sim::EventManager & /*eventMgr*/)
{
    model_ = gz::sim::Model(entity);

    if (!model_.Valid(ecm)) {
        gzerr << "[MotorFailurePlugin] Must be attached to a model entity\n";
        return;
    }

    // Get model name for default topic
    std::string model_name = model_.Name(ecm);

    // Check for custom topic in SDF
    if (sdf->HasElement("topic")) {
        topic_ = sdf->Get<std::string>("topic");
    } else {
        // Default PX4-compatible topic
        topic_ = "/model/" + model_name + "/motor_failure/motor_number";
    }

    // Subscribe to motor failure commands
    if (!node_.Subscribe(topic_, &MotorFailurePlugin::OnMotorFailure, this)) {
        gzerr << "[MotorFailurePlugin] Failed to subscribe to " << topic_ << "\n";
        return;
    }

    gzmsg << "[MotorFailurePlugin] Listening on: " << topic_ << "\n";
}

void MotorFailurePlugin::PreUpdate(
    const gz::sim::UpdateInfo &info,
    gz::sim::EntityComponentManager &ecm)
{
    if (info.paused) {
        return;
    }

    // Discover joints on first update (after model is fully loaded)
    if (!joints_discovered_) {
        DiscoverMotorJoints(ecm);
    }

    // Apply any active motor failure
    if (joints_discovered_) {
        ApplyFailure(ecm);
    }
}

void MotorFailurePlugin::OnMotorFailure(const gz::msgs::Int32 &msg)
{
    int32_t motor = msg.data();

    // Store atomically for thread-safe access from PreUpdate
    failed_motor_.store(motor, std::memory_order_relaxed);

    gzdbg << "[MotorFailurePlugin] Received command: " << motor << "\n";
}

void MotorFailurePlugin::DiscoverMotorJoints(gz::sim::EntityComponentManager &ecm)
{
    motor_joints_.clear();

    // Get all joints in the model
    auto joints = model_.Joints(ecm);

    // Pattern: rotor_0_joint, rotor_1_joint, etc.
    std::regex pattern(R"(rotor_(\d+)_joint)");
    std::smatch match;

    // Collect joints with their indices
    std::vector<std::pair<int, gz::sim::Entity>> indexed_joints;

    for (const auto &joint : joints) {
        auto name_comp = ecm.Component<gz::sim::components::Name>(joint);
        if (!name_comp) {
            continue;
        }

        std::string name = name_comp->Data();
        if (std::regex_match(name, match, pattern)) {
            int idx = std::stoi(match[1].str());
            indexed_joints.emplace_back(idx, joint);
            gzdbg << "[MotorFailurePlugin] Found motor joint: " << name << " (index " << idx << ")\n";
        }
    }

    if (indexed_joints.empty()) {
        gzwarn << "[MotorFailurePlugin] No rotor joints found\n";
        joints_discovered_ = true;
        return;
    }

    // Sort by index and populate vector
    std::sort(indexed_joints.begin(), indexed_joints.end());

    int max_idx = indexed_joints.back().first;
    motor_joints_.resize(max_idx + 1, gz::sim::kNullEntity);

    for (const auto &[idx, joint] : indexed_joints) {
        motor_joints_[idx] = joint;
    }

    gzmsg << "[MotorFailurePlugin] Discovered " << indexed_joints.size() << " motor joints\n";
    joints_discovered_ = true;
}

void MotorFailurePlugin::ApplyFailure(gz::sim::EntityComponentManager &ecm)
{
    int32_t motor = failed_motor_.load(std::memory_order_relaxed);

    // Log state changes
    if (motor != last_logged_failure_) {
        if (motor > 0) {
            gzerr << "[MotorFailurePlugin] Motor " << motor << " FAILED\n";
        } else if (last_logged_failure_ > 0) {
            gzmsg << "[MotorFailurePlugin] Motor " << last_logged_failure_ << " recovered\n";
        }
        last_logged_failure_ = motor;
    }

    // No failure active
    if (motor <= 0) {
        return;
    }

    // Convert 1-indexed motor number to 0-indexed
    int idx = motor - 1;

    if (idx < 0 || idx >= static_cast<int>(motor_joints_.size())) {
        return;
    }

    gz::sim::Entity joint = motor_joints_[idx];
    if (joint == gz::sim::kNullEntity) {
        return;
    }

    // Get or create velocity command component
    auto vel_cmd = ecm.Component<gz::sim::components::JointVelocityCmd>(joint);
    if (vel_cmd) {
        // Override with zero velocity
        *vel_cmd = gz::sim::components::JointVelocityCmd({0.0});
    }
}

}  // namespace gz_plugin
}  // namespace aviate

// Register the plugin with Gazebo
GZ_ADD_PLUGIN(
    aviate::gz_plugin::MotorFailurePlugin,
    gz::sim::System,
    aviate::gz_plugin::MotorFailurePlugin::ISystemConfigure,
    aviate::gz_plugin::MotorFailurePlugin::ISystemPreUpdate)

// Register alias for PX4-compatible name
GZ_ADD_PLUGIN_ALIAS(
    aviate::gz_plugin::MotorFailurePlugin,
    "gz::sim::systems::MotorFailureSystem")

// Register under the filename used in PX4 models
GZ_ADD_PLUGIN_ALIAS(
    aviate::gz_plugin::MotorFailurePlugin,
    "MotorFailurePlugin")
