// Aviate Gazebo Plugin - gz-sim System Plugin
//
// This plugin runs inside gz-sim and provides zero-copy access to the
// EntityComponentManager (ECM) for reading physics state and commanding actuators.
//
// The plugin writes state to shared memory that can be read by Rust via FFI.

#ifndef AVIATE_GZ_PLUGIN_HH
#define AVIATE_GZ_PLUGIN_HH

#include <gz/sim/System.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/transport/Node.hh>
#include <memory>
#include <string>

#include "shared_state.h"

namespace aviate {

/// Aviate gz-sim System Plugin
///
/// This plugin:
/// 1. Reads model pose/velocity from ECM each simulation step
/// 2. Writes state to shared memory for Rust FFI access
/// 3. Reads motor commands from shared memory
/// 4. Applies motor velocities to the model
class AviateGzPlugin
    : public gz::sim::System,
      public gz::sim::ISystemConfigure,
      public gz::sim::ISystemPreUpdate,
      public gz::sim::ISystemPostUpdate
{
public:
    AviateGzPlugin();
    ~AviateGzPlugin() override;

    // ISystemConfigure - called once when plugin is loaded
    void Configure(
        const gz::sim::Entity& entity,
        const std::shared_ptr<const sdf::Element>& sdf,
        gz::sim::EntityComponentManager& ecm,
        gz::sim::EventManager& eventMgr) override;

    // ISystemPreUpdate - called before physics step (for applying commands)
    void PreUpdate(
        const gz::sim::UpdateInfo& info,
        gz::sim::EntityComponentManager& ecm) override;

    // ISystemPostUpdate - called after physics step (for reading state)
    void PostUpdate(
        const gz::sim::UpdateInfo& info,
        const gz::sim::EntityComponentManager& ecm) override;

private:
    /// Initialize shared memory
    bool InitSharedMemory();

    /// Clean up shared memory
    void CleanupSharedMemory();

    /// Instance ID for multi-vehicle support
    int instance_{0};

    /// Model name to track
    std::string modelName_;

    /// Model entity
    gz::sim::Entity modelEntity_{gz::sim::kNullEntity};

    /// Shared memory pointer
    AviateSharedState* sharedState_{nullptr};

    /// Shared memory file descriptor
    int shmFd_{-1};

    /// Shared memory name (instance-specific)
    std::string shmName_;

    /// Last motor command sequence (to detect new commands)
    uint32_t lastMotorSeq_{0};

    /// Motor topic name (instance-specific)
    std::string motorTopic_;

    /// gz-transport node for publishing motor commands
    gz::transport::Node node_;

    /// Motor command publisher
    gz::transport::Node::Publisher motorPub_;

    /// Lockstep mode enabled
    bool lockstep_{false};

    /// Timeout for lockstep wait (microseconds)
    uint64_t lockstepTimeoutUs_{10000};  // 10ms default
};

}  // namespace aviate

#endif  // AVIATE_GZ_PLUGIN_HH
