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

#include <sys/file.h>
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

// The protocol assumes the 8- and 4-byte lane accesses are real atomic
// instructions. If a target lowered them to a lock, two PROCESSES
// would take locks in different address spaces — no mutual exclusion
// at all, and a silently torn block.
static_assert(__atomic_always_lock_free(8, nullptr),
              "8-byte atomics are not lock-free on this target");
static_assert(__atomic_always_lock_free(4, nullptr),
              "4-byte atomics are not lock-free on this target");

namespace {

// Atomic payload lanes — the C++ half of the rule aviate-xil-shm's
// mapping.rs states for Rust: EVERY payload lane is read and written
// atomically (relaxed) by BOTH sides. The seqlock supplies ordering
// and the all-or-nothing snapshot; the atomics supply a defined
// memory model. A plain load/store or memcpy racing the peer
// process's access is a data race — undefined behaviour in C++ and
// Rust alike — however well the seqlock protocol behaves in
// practice.
//
// The wire lanes are `uint64_t` bit patterns (contract v3), so the
// atomic builtins operate on the lane's real type. v2 typed them
// `double` and both sides had to reinterpret_cast a `double*` into a
// `uint64_t*` to access them atomically — an object-access /
// strict-aliasing hazard. Conversion now happens at the boundary,
// through memcpy, which is the only well-defined way to reinterpret
// an object's bytes in C++.

inline double LoadLaneRelaxed(const uint64_t* p)
{
    uint64_t bits = __atomic_load_n(p, __ATOMIC_RELAXED);
    double out;
    std::memcpy(&out, &bits, sizeof(out));
    return out;
}

inline void StoreLaneRelaxed(uint64_t* p, double v)
{
    uint64_t bits;
    std::memcpy(&bits, &v, sizeof(bits));
    __atomic_store_n(p, bits, __ATOMIC_RELAXED);
}

// The C++ half of the writer lease. The Rust side
// (aviate-xil-shm's mapping/lease.rs) owns the protocol; both sides
// must derive the same path and semantics without sharing code:
// an exclusive flock on /tmp/<name>.lease, held for the writer's
// whole life, released by the kernel on any exit including a crash.
// The lease file is never unlinked — removing a lock file races its
// next locker (two processes lock two different inodes behind one
// path).
std::string LeasePath(const std::string& shmName)
{
    std::string base = shmName;
    if (!base.empty() && base.front() == '/') {
        base.erase(0, 1);
    }
    return "/tmp/" + base + ".lease";
}

// Take the writer lease, or return -1 if a live writer holds it.
//
// O_CLOEXEC is load-bearing: flock locks ride the open file
// description, which survives fork+exec, so without it any child
// this process spawns keeps the lease alive after the writer exits.
// The bounded retry sees through the same mechanism in the other
// direction: a fork() anywhere in a lease-holding process briefly
// pins a just-released lock (the child's duplicated fd holds it
// until exec), so a held verdict only counts once it has persisted
// past that window. A live writer holds its lease for its whole
// life; 100 ms separates the two cleanly.
int AcquireWriterLease(const std::string& shmName)
{
    const std::string path = LeasePath(shmName);
    int fd = open(path.c_str(), O_CREAT | O_RDWR | O_CLOEXEC, 0666);
    if (fd == -1) {
        std::cerr << "[AviateGzPlugin] cannot open lease file " << path
                  << ": " << strerror(errno) << std::endl;
        return -1;
    }
    constexpr int kAttempts = 25;
    for (int attempt = 0; attempt < kAttempts; ++attempt) {
        if (flock(fd, LOCK_EX | LOCK_NB) == 0) {
            return fd;
        }
        if (attempt + 1 < kAttempts) {
            std::this_thread::sleep_for(std::chrono::milliseconds(4));
        }
    }
    close(fd);
    return -1;
}

// Whether the name currently resolves to the object that stamped
// `incarnation`. Asking our own mapping would be circular — it
// always answers with the value we stamped — so resolve the NAME.
bool NameResolvesToIncarnation(const std::string& shmName, uint64_t incarnation)
{
    int fd = shm_open(shmName.c_str(), O_RDONLY, 0);
    if (fd == -1) {
        return false;
    }
    struct stat st = {};
    if (fstat(fd, &st) == -1 ||
        st.st_size < static_cast<off_t>(sizeof(AviateSharedStateV2))) {
        close(fd);
        return false;
    }
    void* ptr = mmap(nullptr, sizeof(AviateSharedStateV2), PROT_READ, MAP_SHARED, fd, 0);
    close(fd);
    if (ptr == MAP_FAILED) {
        return false;
    }
    const auto* blk = static_cast<const AviateSharedStateV2*>(ptr);
    const uint64_t live = __atomic_load_n(&blk->header.writer_incarnation, __ATOMIC_ACQUIRE);
    munmap(ptr, sizeof(AviateSharedStateV2));
    return live == incarnation;
}

}  // namespace

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
        for (size_t i = 0; i < 8; ++i) {
            lanes[i] = LoadLaneRelaxed(&shm_->command.motor_vel_bits[i]);
        }
        uint32_t n = __atomic_load_n(&shm_->command.num_motors, __ATOMIC_RELAXED);
        // LoadLoad barrier before the re-read: an acquire LOAD only
        // stops later accesses from moving up, it does NOT stop the
        // lane copy above from sinking BELOW the re-read — a
        // weakly-ordered CPU will do exactly that, and a torn
        // snapshot then escapes with both sequence reads agreeing.
        // Same barrier Linux's read_seqretry() issues via smp_rmb(),
        // and the same one aviate-xil-contract's Rust reader uses.
        __atomic_thread_fence(__ATOMIC_ACQUIRE);
        uint32_t s2 = __atomic_load_n(&shm_->command.seq, __ATOMIC_RELAXED);
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
        // Retire the outgoing snapshot in the same act. Until the new
        // world publishes its first step the block still holds the
        // PREVIOUS epoch's pose — valid, coherent, and from a world
        // that no longer exists. Publishing valid = 0 through the
        // state seqlock stops a reader consuming it; the reader's
        // generation double-check is the second line of defence.
        __atomic_add_fetch(&shm_->state.seq, 1, __ATOMIC_ACQ_REL);
        __atomic_store_n(&shm_->state.valid, 0u, __ATOMIC_RELAXED);
        __atomic_store_n(&shm_->state.reset_generation, resetGeneration_, __ATOMIC_RELAXED);
        __atomic_add_fetch(&shm_->state.seq, 1, __ATOMIC_RELEASE);
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
    // under the model seqlock: odd while writing, even after.
    // sim_step stays monotonic across resets — epochs are told apart
    // by reset_generation, which rides inside the same payload.
    simStep_ += 1;
    __atomic_add_fetch(&shm_->state.seq, 1, __ATOMIC_ACQ_REL);
    __atomic_store_n(&shm_->state.reset_generation, resetGeneration_, __ATOMIC_RELAXED);
    __atomic_store_n(&shm_->state.sim_step, simStep_, __ATOMIC_RELAXED);
    __atomic_store_n(&shm_->state.time_us, timeUs, __ATOMIC_RELAXED);
    for (size_t i = 0; i < 3; ++i) {
        StoreLaneRelaxed(&shm_->state.pos_bits[i], pos[i]);
        StoreLaneRelaxed(&shm_->state.vel_bits[i], vel[i]);
        StoreLaneRelaxed(&shm_->state.ang_vel_bits[i], angVel[i]);
    }
    for (size_t i = 0; i < 4; ++i) {
        StoreLaneRelaxed(&shm_->state.quat_bits[i], quat[i]);
    }
    __atomic_store_n(&shm_->state.valid, 1u, __ATOMIC_RELAXED);
    __atomic_add_fetch(&shm_->state.seq, 1, __ATOMIC_RELEASE);
}

bool AviateGzPlugin::InitSharedMemory()
{
    // The lease comes FIRST. Unlinking before owning the lease is
    // how one writer destroys another's live object: an
    // unconditional unlink here would pull a LIVE peer's block out
    // from under every consumer. Refuse loudly instead of taking
    // over.
    leaseFd_ = AcquireWriterLease(shmName_);
    if (leaseFd_ == -1) {
        std::cerr << "[AviateGzPlugin] another live writer holds "
                  << LeasePath(shmName_)
                  << "; refusing to take over " << shmName_ << std::endl;
        return false;
    }

    // With the lease held, this can only remove a DEAD writer's
    // leftover (macOS also disallows ftruncate on an existing POSIX
    // shm object, so a fresh object is required either way).
    (void) shm_unlink(shmName_.c_str());

    int fd = shm_open(shmName_.c_str(), O_CREAT | O_RDWR, 0666);
    if (fd == -1) {
        std::cerr << "[AviateGzPlugin] shm_open failed for " << shmName_
                  << ": " << strerror(errno) << std::endl;
        close(leaseFd_);
        leaseFd_ = -1;
        return false;
    }

    if (ftruncate(fd, sizeof(AviateSharedStateV2)) == -1) {
        std::cerr << "[AviateGzPlugin] ftruncate failed: " << strerror(errno) << std::endl;
        close(fd);
        // A failed creation must not leave a half-made object
        // behind the published name.
        shm_unlink(shmName_.c_str());
        close(leaseFd_);
        leaseFd_ = -1;
        return false;
    }

    void* ptr = mmap(nullptr, sizeof(AviateSharedStateV2),
                     PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
    close(fd);
    if (ptr == MAP_FAILED) {
        std::cerr << "[AviateGzPlugin] mmap failed: " << strerror(errno) << std::endl;
        shm_unlink(shmName_.c_str());
        close(leaseFd_);
        leaseFd_ = -1;
        return false;
    }

    shm_ = static_cast<AviateSharedStateV2*>(ptr);

    // The block is NOT cleared here. shm_open(O_CREAT) publishes the
    // NAME before ftruncate sizes it, so from that instant an
    // attacher may be mapping this object and atomically loading
    // plugin_ready — a bulk memset would race those loads, which is
    // a data race by definition. It is also unnecessary: a freshly
    // created POSIX shm object is zero-filled and ftruncate
    // zero-extends. Attachers see the pre-ftruncate window as a
    // zero-sized object and retry.
    //
    // Fingerprint before the ready flag, every store atomic: an
    // attacher that sees plugin_ready has a fully self-described
    // block. The SDF lockstep element seeds the shared gate word.
    __atomic_store_n(&shm_->header.magic, AviateMAGIC, __ATOMIC_RELAXED);
    __atomic_store_n(&shm_->header.layout_version, AviateLAYOUT_VERSION, __ATOMIC_RELAXED);
    __atomic_store_n(&shm_->header.declared_size,
                     static_cast<uint32_t>(sizeof(AviateSharedStateV2)), __ATOMIC_RELAXED);
    __atomic_store_n(&shm_->header.reset_generation, 1u, __ATOMIC_RELAXED);
    resetGeneration_ = 1;
    // Non-repeating per created object: a consumer whose writer
    // CRASHED keeps mapping the orphaned block (plugin_ready never
    // cleared, memory alive while mapped) and would serve the dead
    // world's last snapshot forever. Comparing this against the
    // incarnation of whatever the NAME resolves to now is the only
    // way to tell the objects apart — macOS reports st_dev = st_ino
    // = 0 for every POSIX shm object. Zero is reserved for
    // "not stamped".
    {
        uint64_t nanos = static_cast<uint64_t>(
            std::chrono::duration_cast<std::chrono::nanoseconds>(
                std::chrono::system_clock::now().time_since_epoch())
                .count());
        uint64_t pid = static_cast<uint64_t>(getpid());
        incarnation_ = (((nanos << 16) | (nanos >> 48)) ^ pid) | 1ull;
        __atomic_store_n(&shm_->header.writer_incarnation, incarnation_,
                         __ATOMIC_RELAXED);
    }
    __atomic_store_n(&shm_->control.lockstep_enabled, lockstep_ ? 1u : 0u, __ATOMIC_RELAXED);
    __atomic_store_n(&shm_->header.plugin_ready, 1u, __ATOMIC_RELEASE);

    std::cout << "[AviateGzPlugin] Shared memory initialized: " << shmName_
              << " (layout v" << AviateLAYOUT_VERSION
              << ", " << sizeof(AviateSharedStateV2) << " B)" << std::endl;
    return true;
}

void AviateGzPlugin::CleanupSharedMemory()
{
    if (shm_) {
        // Unlink only if the name still resolves to OUR object.
        // While the lease is held no successor can exist, so this is
        // belt-and-braces — but it must resolve the NAME: our own
        // mapping always answers with the incarnation we stamped,
        // replaced or not.
        const bool ours = NameResolvesToIncarnation(shmName_, incarnation_);
        __atomic_store_n(&shm_->header.plugin_ready, 0u, __ATOMIC_RELEASE);
        munmap(shm_, sizeof(AviateSharedStateV2));
        shm_ = nullptr;
        if (ours) {
            shm_unlink(shmName_.c_str());
        }
    }
    // Release the lease only after the unlink: freeing it earlier
    // would let a new writer win the lease while the name still
    // resolves to our dying object.
    if (leaseFd_ != -1) {
        close(leaseFd_);
        leaseFd_ = -1;
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
