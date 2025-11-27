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
#include <cstring>
#include <iostream>

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

    std::cout << "[AviateGzPlugin] Configuring for model: " << modelName_ << std::endl;
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

    // Check for new motor commands from shared memory
    uint32_t motorSeq = __atomic_load_n(&sharedState_->motor_seq, __ATOMIC_ACQUIRE);
    if (motorSeq != lastMotorSeq_) {
        lastMotorSeq_ = motorSeq;

        // Publish motor velocities via gz-transport
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
}

bool AviateGzPlugin::InitSharedMemory()
{
    // Create shared memory object
    shmFd_ = shm_open(AVIATE_SHM_NAME, O_CREAT | O_RDWR, 0666);
    if (shmFd_ == -1) {
        std::cerr << "[AviateGzPlugin] shm_open failed: " << strerror(errno) << std::endl;
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

    std::cout << "[AviateGzPlugin] Shared memory initialized: " << AVIATE_SHM_NAME << std::endl;
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
        shm_unlink(AVIATE_SHM_NAME);
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
