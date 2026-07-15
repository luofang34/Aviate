// Aviate Gazebo Plugin - gz-sim System Plugin
//
// Runs inside gz-sim: publishes ground-truth model state into the
// aviate-xil-contract shared-memory block and forwards motor
// commands from the block to the rotor model.
//
// POLICY-FREE BY DESIGN: this C++ stays a gz API adapter. The layout
// is the cbindgen-generated aviate_xil_contract.h (Rust-owned, #262);
// unit conversion, actuator curves, and configuration interpretation
// all live on the Rust side.

#ifndef AVIATE_GZ_PLUGIN_HH
#define AVIATE_GZ_PLUGIN_HH

#include <gz/sim/System.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/transport/Node.hh>
#include <memory>
#include <string>

#include "aviate_xil_contract.h"

namespace aviate {

class AviateGzPlugin
    : public gz::sim::System,
      public gz::sim::ISystemConfigure,
      public gz::sim::ISystemPreUpdate,
      public gz::sim::ISystemPostUpdate
{
public:
    AviateGzPlugin();
    ~AviateGzPlugin() override;

    void Configure(
        const gz::sim::Entity& entity,
        const std::shared_ptr<const sdf::Element>& sdf,
        gz::sim::EntityComponentManager& ecm,
        gz::sim::EventManager& eventMgr) override;

    void PreUpdate(
        const gz::sim::UpdateInfo& info,
        gz::sim::EntityComponentManager& ecm) override;

    void PostUpdate(
        const gz::sim::UpdateInfo& info,
        const gz::sim::EntityComponentManager& ecm) override;

private:
    bool InitSharedMemory();
    void CleanupSharedMemory();

    /// Instance ID for multi-vehicle support
    int instance_{0};

    /// Model name to track
    std::string modelName_;

    /// Model entity (re-resolved after a world reset)
    gz::sim::Entity modelEntity_{gz::sim::kNullEntity};

    /// Shared block (aviate-xil-contract layout v2)
    AviateSharedStateV2* shm_{nullptr};

    /// Shared memory name (instance-specific)
    std::string shmName_;

    /// Motor topic name (instance-specific)
    std::string motorTopic_;

    /// gz-transport node for publishing motor commands
    gz::transport::Node node_;

    /// Motor command publisher
    gz::transport::Node::Publisher motorPub_;

    /// Lockstep gate armed from SDF at load time
    bool lockstep_{false};

    /// Timeout for the lockstep wait (microseconds)
    uint64_t lockstepTimeoutUs_{10000};

    /// Monotonic physics-step counter (published under the seqlock)
    uint64_t simStep_{0};

    /// Simulation-world epoch mirrored into every snapshot
    uint32_t resetGeneration_{1};

    /// Last published sim time, for world-reset (rewind) detection
    uint64_t lastTimeUs_{0};

    /// Whether at least one state snapshot has been published
    bool timePublished_{false};

    /// Last coherent motor command, republished when a seqlock read
    /// loses the retry budget — NEVER a zero fallback: publishing
    /// zeros on read contention would command a mid-air motor cut.
    double lastMotorLanes_[8] = {0};

    /// Lane count paired with lastMotorLanes_
    uint32_t lastMotorCount_{0};
};

}  // namespace aviate

#endif  // AVIATE_GZ_PLUGIN_HH
