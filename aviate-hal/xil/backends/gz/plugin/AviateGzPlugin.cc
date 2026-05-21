// Aviate Gazebo Plugin Implementation
//
// This plugin provides zero-copy physics data access via shared memory.

#include "AviateGzPlugin.hh"

#include <gz/sim/components/AngularVelocity.hh>
#include <gz/sim/components/LinearVelocity.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/sim/components/JointVelocityCmd.hh>
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

namespace aviate {

AviateGzPlugin::AviateGzPlugin() = default;

AviateGzPlugin::~AviateGzPlugin()
{
    CleanupSharedMemory();
}

void AviateGzPlugin::Configure(
    const gz::sim::Entity& entity,
    const std::shared_ptr<const sdf::Element>& sdf,
    gz::sim::EntityComponentManager& ecm,
    gz::sim::EventManager& /*eventMgr*/)
{
    // Get model name from SDF parameter (default: "x500")
    modelName_ = "x500";
    if (sdf->HasElement("model_name")) {
        modelName_ = sdf->Get<std::string>("model_name");
    }

    // Get instance ID for multi-vehicle support (default: 0)
    instance_ = 0;
    if (sdf->HasElement("instance")) {
        instance_ = sdf->Get<int>("instance");
    }

    // Build motor topic: /<model_name>/command/motor_speed
    motorTopic_ = "/" + modelName_ + "/command/motor_speed";
    if (sdf->HasElement("motor_topic")) {
        motorTopic_ = sdf->Get<std::string>("motor_topic");
    }

    // Build shared memory name: /aviate_gz_bridge or /aviate_gz_bridge_<instance>
    if (instance_ == 0) {
        shmName_ = AVIATE_SHM_NAME;
    } else {
        shmName_ = std::string(AVIATE_SHM_NAME_BASE) + "_" + std::to_string(instance_);
    }

    // Check for lockstep mode (default: disabled for real-time simulation)
    lockstep_ = false;
    if (sdf->HasElement("lockstep")) {
        lockstep_ = sdf->Get<bool>("lockstep");
    }

    // Lockstep timeout in microseconds (default: 10ms)
    lockstepTimeoutUs_ = 10000;
    if (sdf->HasElement("lockstep_timeout_us")) {
        lockstepTimeoutUs_ = sdf->Get<uint64_t>("lockstep_timeout_us");
    }

    std::cout << "[AviateGzPlugin] Configuring for model: " << modelName_ << std::endl;
    std::cout << "[AviateGzPlugin] Instance: " << instance_ << std::endl;
    std::cout << "[AviateGzPlugin] Motor topic: " << motorTopic_ << std::endl;
    std::cout << "[AviateGzPlugin] Shared memory: " << shmName_ << std::endl;
    std::cout << "[AviateGzPlugin] Lockstep: " << (lockstep_ ? "enabled" : "disabled") << std::endl;
    std::cout << "[AviateGzPlugin] Model lookup deferred to first update (included models load later)" << std::endl;

    // Initialize shared memory early
    if (!InitSharedMemory()) {
        std::cerr << "[AviateGzPlugin] Failed to initialize shared memory!" << std::endl;
        return;
    }
}

void AviateGzPlugin::PreUpdate(
    const gz::sim::UpdateInfo& /*info*/,
    gz::sim::EntityComponentManager& ecm)
{
    if (!sharedState_) {
        return;
    }

    // Lockstep synchronization: wait for flight controller to acknowledge previous step
    if (lockstep_ && __atomic_load_n(&sharedState_->lockstep_enabled, __ATOMIC_ACQUIRE)) {
        uint64_t simStep = __atomic_load_n(&sharedState_->sim_step, __ATOMIC_ACQUIRE);
        if (simStep > 0) {
            // Wait for FC to acknowledge the previous step (with timeout)
            auto startWait = std::chrono::steady_clock::now();
            while (true) {
                uint64_t fcAck = __atomic_load_n(&sharedState_->fc_step_ack, __ATOMIC_ACQUIRE);
                if (fcAck >= simStep) {
                    break;  // FC has caught up
                }

                // Check timeout
                auto elapsed = std::chrono::steady_clock::now() - startWait;
                if (std::chrono::duration_cast<std::chrono::microseconds>(elapsed).count()
                    >= static_cast<int64_t>(lockstepTimeoutUs_)) {
                    // Timeout - continue anyway to prevent deadlock
                    break;
                }

                // Yield CPU briefly
                std::this_thread::sleep_for(std::chrono::microseconds(10));
            }
        }
    }

    // Deferred model lookup - included models are loaded after Configure()
    if (modelEntity_ == gz::sim::kNullEntity) {
        ecm.Each<gz::sim::components::Name>(
            [&](const gz::sim::Entity& ent,
                const gz::sim::components::Name* name) -> bool
            {
                if (name->Data() == modelName_) {
                    modelEntity_ = ent;
                    return false;  // Stop iteration
                }
                return true;
            });

        if (modelEntity_ != gz::sim::kNullEntity) {
            std::cout << "[AviateGzPlugin] Found model '" << modelName_ << "'" << std::endl;
            // Enable velocity components for the model
            gz::sim::enableComponent<gz::sim::components::WorldLinearVelocity>(ecm, modelEntity_);
            gz::sim::enableComponent<gz::sim::components::WorldAngularVelocity>(ecm, modelEntity_);

            // Create motor command publisher
            motorPub_ = node_.Advertise<gz::msgs::Actuators>(motorTopic_);
            std::cout << "[AviateGzPlugin] Motor publisher created for topic: " << motorTopic_ << std::endl;
        }
        return;  // Wait for model to be found
    }

    // Publish the latest motor command every PreUpdate, not only
    // when motor_seq changes. gz-transport doesn't replay history,
    // so if MulticopterMotorModel's subscription lands after the
    // kernel's first non-zero motor message that message is lost
    // and the vehicle never lifts. Republishing the current values
    // each tick is cheap (one Actuators message) and removes the
    // race.
    __atomic_load_n(&sharedState_->motor_seq, __ATOMIC_ACQUIRE);
    {
        gz::msgs::Actuators msg;
        int numMotors = sharedState_->num_motors;
        if (numMotors > 8) numMotors = 8;
        if (numMotors < 1) numMotors = 4;  // Default to 4 motors

        for (int i = 0; i < numMotors; i++) {
            msg.add_velocity(sharedState_->motor_vel[i]);
        }

        motorPub_.Publish(msg);
    }
}

void AviateGzPlugin::PostUpdate(
    const gz::sim::UpdateInfo& info,
    const gz::sim::EntityComponentManager& ecm)
{
    if (!sharedState_ || modelEntity_ == gz::sim::kNullEntity) {
        return;
    }

    // Get world pose
    auto poseComp = ecm.Component<gz::sim::components::Pose>(modelEntity_);
    if (poseComp) {
        const auto& pose = poseComp->Data();
        const auto& pos = pose.Pos();
        const auto& rot = pose.Rot();

        sharedState_->pos[0] = pos.X();
        sharedState_->pos[1] = pos.Y();
        sharedState_->pos[2] = pos.Z();

        sharedState_->quat[0] = rot.W();
        sharedState_->quat[1] = rot.X();
        sharedState_->quat[2] = rot.Y();
        sharedState_->quat[3] = rot.Z();
    }

    // Get world linear velocity
    auto linVelComp = ecm.Component<gz::sim::components::WorldLinearVelocity>(modelEntity_);
    if (linVelComp) {
        const auto& vel = linVelComp->Data();
        sharedState_->vel[0] = vel.X();
        sharedState_->vel[1] = vel.Y();
        sharedState_->vel[2] = vel.Z();
    }

    // Get world angular velocity
    auto angVelComp = ecm.Component<gz::sim::components::WorldAngularVelocity>(modelEntity_);
    if (angVelComp) {
        const auto& angVel = angVelComp->Data();
        sharedState_->ang_vel[0] = angVel.X();
        sharedState_->ang_vel[1] = angVel.Y();
        sharedState_->ang_vel[2] = angVel.Z();
    }

    // Set timestamp (simulation time in microseconds)
    auto simTimeNs = std::chrono::duration_cast<std::chrono::microseconds>(info.simTime).count();
    sharedState_->time_us = static_cast<uint64_t>(simTimeNs);

    // Increment sequence and mark valid
    __atomic_fetch_add(&sharedState_->seq, 1, __ATOMIC_RELEASE);
    __atomic_store_n(&sharedState_->valid, 1, __ATOMIC_RELEASE);

    // Increment sim_step so consumers (test runner, FC) can detect
    // new state regardless of whether lockstep gating is active. The
    // lockstep_ flag governs only the *blocking* gate in PreUpdate,
    // not the publication of the step counter.
    __atomic_fetch_add(&sharedState_->sim_step, 1, __ATOMIC_RELEASE);
}

bool AviateGzPlugin::InitSharedMemory()
{
    // Always unlink any prior shm segment first. macOS disallows
    // ftruncate on an existing POSIX shm object — stale state from a
    // previous run fails the resize below with EINVAL. ENOENT is
    // benign (no prior segment).
    (void) shm_unlink(shmName_.c_str());

    // Create shared memory object using instance-specific name
    shmFd_ = shm_open(shmName_.c_str(), O_CREAT | O_RDWR, 0666);
    if (shmFd_ == -1) {
        std::cerr << "[AviateGzPlugin] shm_open failed for " << shmName_ << ": " << strerror(errno) << std::endl;
        return false;
    }

    // Set size
    if (ftruncate(shmFd_, sizeof(AviateSharedState)) == -1) {
        std::cerr << "[AviateGzPlugin] ftruncate failed: " << strerror(errno) << std::endl;
        close(shmFd_);
        shmFd_ = -1;
        return false;
    }

    // Map into memory
    void* ptr = mmap(nullptr, sizeof(AviateSharedState), PROT_READ | PROT_WRITE, MAP_SHARED, shmFd_, 0);
    if (ptr == MAP_FAILED) {
        std::cerr << "[AviateGzPlugin] mmap failed: " << strerror(errno) << std::endl;
        close(shmFd_);
        shmFd_ = -1;
        return false;
    }

    sharedState_ = static_cast<AviateSharedState*>(ptr);

    // Initialize state
    std::memset(sharedState_, 0, sizeof(AviateSharedState));
    __atomic_store_n(&sharedState_->plugin_ready, 1, __ATOMIC_RELEASE);

    std::cout << "[AviateGzPlugin] Shared memory initialized: " << shmName_ << std::endl;
    return true;
}

void AviateGzPlugin::CleanupSharedMemory()
{
    if (sharedState_) {
        __atomic_store_n(&sharedState_->plugin_ready, 0, __ATOMIC_RELEASE);
        munmap(sharedState_, sizeof(AviateSharedState));
        sharedState_ = nullptr;
    }

    if (shmFd_ != -1) {
        close(shmFd_);
        if (!shmName_.empty()) {
            shm_unlink(shmName_.c_str());
        }
        shmFd_ = -1;
    }
}

}  // namespace aviate

// Register plugin with gz-sim
GZ_ADD_PLUGIN(
    aviate::AviateGzPlugin,
    gz::sim::System,
    aviate::AviateGzPlugin::ISystemConfigure,
    aviate::AviateGzPlugin::ISystemPreUpdate,
    aviate::AviateGzPlugin::ISystemPostUpdate)

GZ_ADD_PLUGIN_ALIAS(aviate::AviateGzPlugin, "AviateGzPlugin")
