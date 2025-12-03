// Aviate Gazebo Bridge - C Interface Implementation
//
// This file implements the C FFI interface for Rust to access gz-sim data
// via shared memory created by the AviateGzPlugin.
//
// Multi-instance support: Each vehicle instance has its own bridge state.

#include "aviate_gz_bridge.h"
#include "shared_state.h"

#include <sys/mman.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <cstdio>
#include <cstring>
#include <atomic>

// Bridge state for a single instance
struct BridgeState {
    AviateSharedState* shm;
    int fd;
    uint32_t last_seq;
    bool initialized;
};

// Array of bridge states (one per instance)
static BridgeState g_bridges[AVIATE_MAX_INSTANCES] = {};

// Helper to validate instance
static inline bool valid_instance(int instance) {
    return instance >= 0 && instance < AVIATE_MAX_INSTANCES;
}

extern "C" {

int aviate_gz_init_instance(int instance)
{
    if (!valid_instance(instance)) {
        return -3;  // Invalid instance
    }

    BridgeState& bridge = g_bridges[instance];

    if (bridge.initialized) {
        return 0;  // Already initialized for this instance
    }

    // Build shared memory name based on instance
    char shm_name[64];
    if (instance == 0) {
        snprintf(shm_name, sizeof(shm_name), "%s", AVIATE_SHM_NAME);
    } else {
        snprintf(shm_name, sizeof(shm_name), "%s_%d", AVIATE_SHM_NAME_BASE, instance);
    }

    // Open existing shared memory (created by plugin)
    bridge.fd = shm_open(shm_name, O_RDWR, 0666);
    if (bridge.fd == -1) {
        return -1;  // Plugin not running yet
    }

    // Map into memory
    void* ptr = mmap(nullptr, sizeof(AviateSharedState),
                     PROT_READ | PROT_WRITE, MAP_SHARED, bridge.fd, 0);
    if (ptr == MAP_FAILED) {
        close(bridge.fd);
        bridge.fd = -1;
        return -2;
    }

    bridge.shm = static_cast<AviateSharedState*>(ptr);
    bridge.last_seq = 0;
    bridge.initialized = true;

    return 0;
}

void aviate_gz_shutdown_instance(int instance)
{
    if (!valid_instance(instance)) {
        return;
    }

    BridgeState& bridge = g_bridges[instance];

    if (!bridge.initialized) {
        return;
    }

    if (bridge.shm) {
        munmap(bridge.shm, sizeof(AviateSharedState));
        bridge.shm = nullptr;
    }

    if (bridge.fd != -1) {
        close(bridge.fd);
        bridge.fd = -1;
    }

    bridge.initialized = false;
}

int aviate_gz_get_model_state_instance(int instance, AviateModelState* out)
{
    if (!valid_instance(instance) || !out) {
        return -1;
    }

    BridgeState& bridge = g_bridges[instance];

    if (!bridge.initialized || !bridge.shm) {
        return -1;
    }

    // Check if plugin has marked data as valid
    if (!__atomic_load_n(&bridge.shm->valid, __ATOMIC_ACQUIRE)) {
        out->valid = 0;
        return -2;
    }

    // Read current sequence
    uint32_t seq = __atomic_load_n(&bridge.shm->seq, __ATOMIC_ACQUIRE);

    // Copy data
    out->pos[0] = bridge.shm->pos[0];
    out->pos[1] = bridge.shm->pos[1];
    out->pos[2] = bridge.shm->pos[2];

    out->quat[0] = bridge.shm->quat[0];
    out->quat[1] = bridge.shm->quat[1];
    out->quat[2] = bridge.shm->quat[2];
    out->quat[3] = bridge.shm->quat[3];

    out->vel[0] = bridge.shm->vel[0];
    out->vel[1] = bridge.shm->vel[1];
    out->vel[2] = bridge.shm->vel[2];

    out->ang_vel[0] = bridge.shm->ang_vel[0];
    out->ang_vel[1] = bridge.shm->ang_vel[1];
    out->ang_vel[2] = bridge.shm->ang_vel[2];

    out->time_us = bridge.shm->time_us;
    out->valid = 1;

    bridge.last_seq = seq;
    return 0;
}

int aviate_gz_set_motor_speeds_instance(int instance, const AviateMotorCommand* cmd)
{
    if (!valid_instance(instance) || !cmd) {
        return -1;
    }

    BridgeState& bridge = g_bridges[instance];

    if (!bridge.initialized || !bridge.shm) {
        return -1;
    }

    // Copy motor velocities
    int n = cmd->num_motors;
    if (n > 8) n = 8;

    for (int i = 0; i < n; i++) {
        bridge.shm->motor_vel[i] = cmd->velocities[i];
    }
    bridge.shm->num_motors = n;

    // Increment sequence to signal new command
    __atomic_fetch_add(&bridge.shm->motor_seq, 1, __ATOMIC_RELEASE);

    return 0;
}

uint64_t aviate_gz_get_sim_time_us_instance(int instance)
{
    if (!valid_instance(instance)) {
        return 0;
    }

    BridgeState& bridge = g_bridges[instance];

    if (!bridge.initialized || !bridge.shm) {
        return 0;
    }
    return bridge.shm->time_us;
}

int aviate_gz_is_connected_instance(int instance)
{
    if (!valid_instance(instance)) {
        return 0;
    }

    BridgeState& bridge = g_bridges[instance];

    if (!bridge.initialized || !bridge.shm) {
        return 0;
    }
    return __atomic_load_n(&bridge.shm->plugin_ready, __ATOMIC_ACQUIRE) != 0;
}

void aviate_gz_set_lockstep_instance(int instance, int enabled)
{
    if (!valid_instance(instance)) {
        return;
    }

    BridgeState& bridge = g_bridges[instance];

    if (!bridge.initialized || !bridge.shm) {
        return;
    }
    __atomic_store_n(&bridge.shm->lockstep_enabled, enabled ? 1 : 0, __ATOMIC_RELEASE);
}

uint64_t aviate_gz_get_sim_step_instance(int instance)
{
    if (!valid_instance(instance)) {
        return 0;
    }

    BridgeState& bridge = g_bridges[instance];

    if (!bridge.initialized || !bridge.shm) {
        return 0;
    }
    return __atomic_load_n(&bridge.shm->sim_step, __ATOMIC_ACQUIRE);
}

void aviate_gz_ack_step_instance(int instance, uint64_t step)
{
    if (!valid_instance(instance)) {
        return;
    }

    BridgeState& bridge = g_bridges[instance];

    if (!bridge.initialized || !bridge.shm) {
        return;
    }
    __atomic_store_n(&bridge.shm->fc_step_ack, step, __ATOMIC_RELEASE);
}

// ============================================================================
// Legacy API (instance 0 only, for backwards compatibility)
// ============================================================================

int aviate_gz_init(void)
{
    return aviate_gz_init_instance(0);
}

void aviate_gz_shutdown(void)
{
    aviate_gz_shutdown_instance(0);
}

int aviate_gz_get_model_state(AviateModelState* out)
{
    return aviate_gz_get_model_state_instance(0, out);
}

int aviate_gz_set_motor_speeds(const AviateMotorCommand* cmd)
{
    return aviate_gz_set_motor_speeds_instance(0, cmd);
}

uint64_t aviate_gz_get_sim_time_us(void)
{
    return aviate_gz_get_sim_time_us_instance(0);
}

int aviate_gz_is_connected(void)
{
    return aviate_gz_is_connected_instance(0);
}

void aviate_gz_set_lockstep(int enabled)
{
    aviate_gz_set_lockstep_instance(0, enabled);
}

uint64_t aviate_gz_get_sim_step(void)
{
    return aviate_gz_get_sim_step_instance(0);
}

void aviate_gz_ack_step(uint64_t step)
{
    aviate_gz_ack_step_instance(0, step);
}

}  // extern "C"
