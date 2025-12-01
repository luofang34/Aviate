// SPDX-License-Identifier: MIT
// Copyright (c) 2024 Aviate Project
//
// MotorFailurePlugin - Gazebo plugin for simulating motor failures


#ifndef AVIATE_GZ_MOTOR_FAILURE_PLUGIN_HH_
#define AVIATE_GZ_MOTOR_FAILURE_PLUGIN_HH_

#include <gz/sim/System.hh>
#include <gz/sim/Model.hh>
#include <gz/transport/Node.hh>
#include <gz/msgs/int32.pb.h>

#include <atomic>
#include <string>
#include <vector>

namespace aviate
{
namespace gz_plugin
{

/// @brief Gazebo system plugin that simulates motor failures.
///
/// Subscribes to a gz-transport topic to receive motor failure commands.
/// When a motor failure is triggered, the plugin sets the joint velocity
/// command to zero, simulating a stopped motor.
///
/// Interface (PX4-compatible):
/// - Topic: /model/<model_name>/motor_failure/motor_number
/// - Message: gz.msgs.Int32
/// - Values: 0 or -1 = no failure, 1-N = fail motor N (1-indexed)
///
/// SDF configuration:
/// <plugin filename="MotorFailurePlugin" name="aviate::gz_plugin::MotorFailurePlugin">
///   <topic>/custom/topic</topic>  <!-- optional, overrides default -->
/// </plugin>
class MotorFailurePlugin :
    public gz::sim::System,
    public gz::sim::ISystemConfigure,
    public gz::sim::ISystemPreUpdate
{
public:
    MotorFailurePlugin() = default;
    ~MotorFailurePlugin() override = default;

    /// @brief Configure the plugin from SDF
    void Configure(
        const gz::sim::Entity &entity,
        const std::shared_ptr<const sdf::Element> &sdf,
        gz::sim::EntityComponentManager &ecm,
        gz::sim::EventManager &eventMgr) override;

    /// @brief Called before each simulation step
    void PreUpdate(
        const gz::sim::UpdateInfo &info,
        gz::sim::EntityComponentManager &ecm) override;

private:
    /// @brief Callback for motor failure commands
    void OnMotorFailure(const gz::msgs::Int32 &msg);

    /// @brief Discover motor joints in the model
    void DiscoverMotorJoints(gz::sim::EntityComponentManager &ecm);

    /// @brief Apply motor failure by zeroing velocity command
    void ApplyFailure(gz::sim::EntityComponentManager &ecm);

    gz::transport::Node node_;
    gz::sim::Model model_{gz::sim::kNullEntity};

    std::vector<gz::sim::Entity> motor_joints_;
    std::atomic<int32_t> failed_motor_{0};  // 0 = no failure, 1-N = motor index
    int32_t last_logged_failure_{0};

    std::string topic_;
    bool joints_discovered_{false};
};

}  // namespace gz_plugin
}  // namespace aviate

#endif  // AVIATE_GZ_MOTOR_FAILURE_PLUGIN_HH_
