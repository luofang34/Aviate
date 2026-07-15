// Aviate Gazebo Plugin Implementation
//
// gz API adapter over the Rust-owned shared-memory contract
// (aviate_xil_contract.h). See AviateGzPlugin.hh for the policy
// boundary.

#include "AviateGzPlugin.hh"

#include <gz/sim/components/AngularVelocity.hh>
#include <gz/sim/components/LinearVelocity.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/sim/Util.hh>
#include <gz/plugin/Register.hh>
#include <gz/msgs/actuators.pb.h>

#include <sys/mman.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <chrono>
#include <cstring>
#include <iostream>
#include <thread>

// The generated header carries the Rust layout; drift between the
// two languages must be a build failure, not a runtime mystery.
static_assert(sizeof(AviateSharedStateV2) == 448,
              "aviate_xil_contract layout drifted from the pinned size");
static_assert(offsetof(AviateSharedStateV2, state) == 64,
              "model-state block offset drifted");
static_assert(offsetof(AviateSharedStateV2, command) == 256,
              "motor-command block offset drifted");
static_assert(offsetof(AviateSharedStateV2, control) == 384,
              "control block offset drifted");
static_assert(offsetof(AviateModelStateBlock, reset_generation) == 4,
              "snapshot generation offset drifted");

namespace aviate {

AviateGzPlugin::AviateGzPlugin() = default;

AviateGzPlugin::~AviateGzPlugin()
{
    CleanupSharedMemory();
}

void AviateGzPlugin::Configure(
    const gz::sim::Entity& /*entity*/,
    const std::shared_ptr<const sdf::Element>& sdf,
    gz::sim::EntityComponentManager& /*ecm*/,
    gz::sim::EventManager& /*eventMgr*/)
{
    modelName_ = "x500";
    if (sdf->HasElement("model_name")) {
        modelName_ = sdf->Get<std::string>("model_name");
    }

    instance_ = 0;
    if (sdf->HasElement("instance")) {
        instance_ = sdf->Get<int>("instance");
    }

    motorTopic_ = "/" + modelName_ + "/command/motor_speed";
    if (sdf->HasElement("motor_topic")) {
        motorTopic_ = sdf->Get<std::string>("motor_topic");
    }

    shmName_ = "/aviate_gz_bridge";
    if (instance_ != 0) {
        shmName_ += "_" + std::to_string(instance_);
    }

    lockstep_ = false;
    if (sdf->HasElement("lockstep")) {
        lockstep_ = sdf->Get<bool>("lockstep");
    }

    lockstepTimeoutUs_ = 10000;
    if (sdf->HasElement("lockstep_timeout_us")) {
        lockstepTimeoutUs_ = sdf->Get<uint64_t>("lockstep_timeout_us");
    }

    std::cout << "[AviateGzPlugin] model: " << modelName_
              << " instance: " << instance_
              << " shm: " << shmName_
              << " lockstep: " << (lockstep_ ? "enabled" : "disabled") << std::endl;

    if (!InitSharedMemory()) {
        std::cerr << "[AviateGzPlugin] Failed to initialize shared memory!" << std::endl;
        return;
    }
}

void AviateGzPlugin::PreUpdate(
    const gz::sim::UpdateInfo& /*info*/,
    gz::sim::EntityComponentManager& ecm)
{
    if (!shm_) {
        return;
    }

    // Lockstep gate: wait for the FC to acknowledge the previous
    // step, with a timeout so an absent or restarting FC cannot
    // deadlock physics. Armed from SDF; the shared word lets the FC
    // side release the gate.
    if (lockstep_
        && __atomic_load_n(&shm_->control.lockstep_enabled, __ATOMIC_ACQUIRE) != 0
        && simStep_ > 0) {
        auto startWait = std::chrono::steady_clock::now();
        while (true) {
            uint64_t fcAck = __atomic_load_n(&shm_->command.fc_step_ack, __ATOMIC_ACQUIRE);
            if (fcAck >= simStep_) {
                break;
            }
            auto elapsed = std::chrono::steady_clock::now() - startWait;
            if (std::chrono::duration_cast<std::chrono::microseconds>(elapsed).count()
                >= static_cast<int64_t>(lockstepTimeoutUs_)) {
                break;
            }
            std::this_thread::sleep_for(std::chrono::microseconds(10));
        }
    }

    // Deferred model lookup — included models load after Configure,
    // and a world reset may invalidate the cached entity.
    if (modelEntity_ == gz::sim::kNullEntity) {
        ecm.Each<gz::sim::components::Name>(
            [&](const gz::sim::Entity& ent,
                const gz::sim::components::Name* name) -> bool
            {
                if (name->Data() == modelName_) {
                    modelEntity_ = ent;
                    return false;
                }
                return true;
            });

        if (modelEntity_ != gz::sim::kNullEntity) {
            std::cout << "[AviateGzPlugin] Found model '" << modelName_ << "'" << std::endl;
            gz::sim::enableComponent<gz::sim::components::WorldLinearVelocity>(ecm, modelEntity_);
            gz::sim::enableComponent<gz::sim::components::WorldAngularVelocity>(ecm, modelEntity_);
            if (!motorPub_.Valid()) {
                motorPub_ = node_.Advertise<gz::msgs::Actuators>(motorTopic_);
            }
        }
        return;
    }

    // Republish the FC's latest motor command every PreUpdate —
    // gz-transport does not replay history for late subscribers.
    // Consistent snapshot via the command seqlock; if the retry
    // budget is exhausted mid-write, keep the LAST GOOD command
    // rather than fabricating zeros (a zero fallback would command
    // a mid-air motor cut on read contention).
    for (uint32_t attempt = 0; attempt < AviateSEQLOCK_MAX_RETRIES; ++attempt) {
        uint32_t s1 = __atomic_load_n(&shm_->command.seq, __ATOMIC_ACQUIRE);
        if (s1 & 1u) {
            continue;
        }
        double lanes[8];
        std::memcpy(lanes, shm_->command.motor_vel, sizeof(lanes));
        uint32_t n = shm_->command.num_motors;
        uint32_t s2 = __atomic_load_n(&shm_->command.seq, __ATOMIC_ACQUIRE);
        if (s1 == s2) {
            if (n > 8) n = 8;
            std::memcpy(lastMotorLanes_, lanes, sizeof(lanes));
            lastMotorCount_ = n;
            break;
        }
    }

    uint32_t numMotors = lastMotorCount_;
    if (numMotors < 1) numMotors = 4;  // Nothing received: idle lanes

    gz::msgs::Actuators msg;
    for (uint32_t i = 0; i < numMotors; i++) {
        msg.add_velocity(lastMotorLanes_[i]);
    }
    motorPub_.Publish(msg);
}

void AviateGzPlugin::PostUpdate(
    const gz::sim::UpdateInfo& info,
    const gz::sim::EntityComponentManager& ecm)
{
    if (!shm_) {
        return;
    }

    uint64_t timeUs = static_cast<uint64_t>(
        std::chrono::duration_cast<std::chrono::microseconds>(info.simTime).count());

    // World-reset detection: sim time rewound. Bump the epoch so
    // every consumer re-establishes freshness instead of
    // quarantining, and drop the cached model entity — a reset may
    // have recreated it.
    if (timePublished_ && timeUs < lastTimeUs_) {
        resetGeneration_ = __atomic_add_fetch(
            &shm_->header.reset_generation, 1, __ATOMIC_ACQ_REL);
        modelEntity_ = gz::sim::kNullEntity;
        std::cout << "[AviateGzPlugin] world reset detected (time rewound); generation -> "
                  << resetGeneration_ << std::endl;
    }
    lastTimeUs_ = timeUs;
    timePublished_ = true;

    if (modelEntity_ == gz::sim::kNullEntity) {
        return;
    }

    double pos[3] = {0}, quat[4] = {1, 0, 0, 0}, vel[3] = {0}, angVel[3] = {0};
    auto poseComp = ecm.Component<gz::sim::components::Pose>(modelEntity_);
    if (poseComp) {
        const auto& pose = poseComp->Data();
        pos[0] = pose.Pos().X();
        pos[1] = pose.Pos().Y();
        pos[2] = pose.Pos().Z();
        quat[0] = pose.Rot().W();
        quat[1] = pose.Rot().X();
        quat[2] = pose.Rot().Y();
        quat[3] = pose.Rot().Z();
    }
    auto linVelComp = ecm.Component<gz::sim::components::WorldLinearVelocity>(modelEntity_);
    if (linVelComp) {
        vel[0] = linVelComp->Data().X();
        vel[1] = linVelComp->Data().Y();
        vel[2] = linVelComp->Data().Z();
    }
    auto angVelComp = ecm.Component<gz::sim::components::WorldAngularVelocity>(modelEntity_);
    if (angVelComp) {
        angVel[0] = angVelComp->Data().X();
        angVel[1] = angVelComp->Data().Y();
        angVel[2] = angVelComp->Data().Z();
    }

    // Publish one coherent {generation, step, time, state} snapshot
    // under the model seqlock: odd while writing, even after (#262).
    // sim_step stays monotonic across resets — epochs are told apart
    // by reset_generation, which rides inside the same payload.
    simStep_ += 1;
    __atomic_add_fetch(&shm_->state.seq, 1, __ATOMIC_ACQ_REL);
    shm_->state.reset_generation = resetGeneration_;
    shm_->state.sim_step = simStep_;
    shm_->state.time_us = timeUs;
    std::memcpy(shm_->state.pos, pos, sizeof(pos));
    std::memcpy(shm_->state.quat, quat, sizeof(quat));
    std::memcpy(shm_->state.vel, vel, sizeof(vel));
    std::memcpy(shm_->state.ang_vel, angVel, sizeof(angVel));
    shm_->state.valid = 1;
    __atomic_add_fetch(&shm_->state.seq, 1, __ATOMIC_RELEASE);
}

bool AviateGzPlugin::InitSharedMemory()
{
    // Always unlink any prior segment first: macOS disallows
    // ftruncate on an existing POSIX shm object.
    (void) shm_unlink(shmName_.c_str());

    int fd = shm_open(shmName_.c_str(), O_CREAT | O_RDWR, 0666);
    if (fd == -1) {
        std::cerr << "[AviateGzPlugin] shm_open failed for " << shmName_
                  << ": " << strerror(errno) << std::endl;
        return false;
    }

    if (ftruncate(fd, sizeof(AviateSharedStateV2)) == -1) {
        std::cerr << "[AviateGzPlugin] ftruncate failed: " << strerror(errno) << std::endl;
        close(fd);
        return false;
    }

    void* ptr = mmap(nullptr, sizeof(AviateSharedStateV2),
                     PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
    close(fd);
    if (ptr == MAP_FAILED) {
        std::cerr << "[AviateGzPlugin] mmap failed: " << strerror(errno) << std::endl;
        return false;
    }

    shm_ = static_cast<AviateSharedStateV2*>(ptr);
    std::memset(shm_, 0, sizeof(AviateSharedStateV2));

    // Fingerprint before the ready flag: an attacher that sees
    // plugin_ready has a fully self-described block (#262). The
    // SDF lockstep element seeds the shared gate word.
    shm_->header.magic = AviateMAGIC;
    shm_->header.layout_version = AviateLAYOUT_VERSION;
    shm_->header.declared_size = static_cast<uint32_t>(sizeof(AviateSharedStateV2));
    shm_->header.reset_generation = 1;
    resetGeneration_ = 1;
    shm_->control.lockstep_enabled = lockstep_ ? 1u : 0u;
    __atomic_store_n(&shm_->header.plugin_ready, 1u, __ATOMIC_RELEASE);

    std::cout << "[AviateGzPlugin] Shared memory initialized: " << shmName_
              << " (layout v" << AviateLAYOUT_VERSION
              << ", " << sizeof(AviateSharedStateV2) << " B)" << std::endl;
    return true;
}

void AviateGzPlugin::CleanupSharedMemory()
{
    if (shm_) {
        __atomic_store_n(&shm_->header.plugin_ready, 0u, __ATOMIC_RELEASE);
        munmap(shm_, sizeof(AviateSharedStateV2));
        shm_ = nullptr;
        shm_unlink(shmName_.c_str());
    }
}

}  // namespace aviate

GZ_ADD_PLUGIN(
    aviate::AviateGzPlugin,
    gz::sim::System,
    aviate::AviateGzPlugin::ISystemConfigure,
    aviate::AviateGzPlugin::ISystemPreUpdate,
    aviate::AviateGzPlugin::ISystemPostUpdate)

GZ_ADD_PLUGIN_ALIAS(aviate::AviateGzPlugin, "AviateGzPlugin")
