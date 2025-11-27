// Aviate Gazebo Bridge - C Interface Implementation
//
// This file implements the C FFI interface for Rust to access gz-sim data
// via shared memory created by the AviateGzPlugin.

#include "aviate_gz_bridge.h"
#include "shared_state.h"

#include <sys/mman.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <cstdio>
#include <cstring>
#include <atomic>

// Global state for the C bridge
static struct {
    AviateSharedState* shm;
    int fd;
    uint32_t last_seq;
    bool initialized;
} g_bridge = {nullptr, -1, 0, false};

extern "C" {

int aviate_gz_init_instance(int instance)
{
    if (g_bridge.initialized) {
        return 0;  // Already initialized
    }

    // Build shared memory name based on instance
    char shm_name[64];
    if (instance == 0) {
        snprintf(shm_name, sizeof(shm_name), "%s", AVIATE_SHM_NAME);
    } else {
        snprintf(shm_name, sizeof(shm_name), "%s_%d", AVIATE_SHM_NAME_BASE, instance);
    }

    // Open existing shared memory (created by plugin)
    g_bridge.fd = shm_open(shm_name, O_RDWR, 0666);
    if (g_bridge.fd == -1) {
        return -1;  // Plugin not running yet
    }

    // Map into memory
    void* ptr = mmap(nullptr, sizeof(AviateSharedState),
                     PROT_READ | PROT_WRITE, MAP_SHARED, g_bridge.fd, 0);
    if (ptr == MAP_FAILED) {
        close(g_bridge.fd);
        g_bridge.fd = -1;
        return -2;
    }

    g_bridge.shm = static_cast<AviateSharedState*>(ptr);
    g_bridge.last_seq = 0;
    g_bridge.initialized = true;

    return 0;
}

int aviate_gz_init(void)
{
    return aviate_gz_init_instance(0);
}

void aviate_gz_shutdown(void)
{
    if (!g_bridge.initialized) {
        return;
    }

    if (g_bridge.shm) {
        munmap(g_bridge.shm, sizeof(AviateSharedState));
        g_bridge.shm = nullptr;
    }

    if (g_bridge.fd != -1) {
        close(g_bridge.fd);
        g_bridge.fd = -1;
    }

    g_bridge.initialized = false;
}

int aviate_gz_get_model_state(AviateModelState* out)
{
    if (!g_bridge.initialized || !g_bridge.shm || !out) {
        return -1;
    }

    // Check if plugin has marked data as valid
    if (!__atomic_load_n(&g_bridge.shm->valid, __ATOMIC_ACQUIRE)) {
        out->valid = 0;
        return -2;
    }

    // Read current sequence
    uint32_t seq = __atomic_load_n(&g_bridge.shm->seq, __ATOMIC_ACQUIRE);

    // Copy data (simple memcpy is safe due to atomic seq)
    out->pos[0] = g_bridge.shm->pos[0];
    out->pos[1] = g_bridge.shm->pos[1];
    out->pos[2] = g_bridge.shm->pos[2];

    out->quat[0] = g_bridge.shm->quat[0];
    out->quat[1] = g_bridge.shm->quat[1];
    out->quat[2] = g_bridge.shm->quat[2];
    out->quat[3] = g_bridge.shm->quat[3];

    out->vel[0] = g_bridge.shm->vel[0];
    out->vel[1] = g_bridge.shm->vel[1];
    out->vel[2] = g_bridge.shm->vel[2];

    out->ang_vel[0] = g_bridge.shm->ang_vel[0];
    out->ang_vel[1] = g_bridge.shm->ang_vel[1];
    out->ang_vel[2] = g_bridge.shm->ang_vel[2];

    out->time_us = g_bridge.shm->time_us;
    out->valid = 1;

    g_bridge.last_seq = seq;
    return 0;
}

int aviate_gz_set_motor_speeds(const AviateMotorCommand* cmd)
{
    if (!g_bridge.initialized || !g_bridge.shm || !cmd) {
        return -1;
    }

    // Copy motor velocities
    int n = cmd->num_motors;
    if (n > 8) n = 8;

    for (int i = 0; i < n; i++) {
        g_bridge.shm->motor_vel[i] = cmd->velocities[i];
    }
    g_bridge.shm->num_motors = n;

    // Increment sequence to signal new command
    __atomic_fetch_add(&g_bridge.shm->motor_seq, 1, __ATOMIC_RELEASE);

    return 0;
}

uint64_t aviate_gz_get_sim_time_us(void)
{
    if (!g_bridge.initialized || !g_bridge.shm) {
        return 0;
    }
    return g_bridge.shm->time_us;
}

int aviate_gz_is_connected(void)
{
    if (!g_bridge.initialized || !g_bridge.shm) {
        return 0;
    }
    return __atomic_load_n(&g_bridge.shm->plugin_ready, __ATOMIC_ACQUIRE) != 0;
}

void aviate_gz_set_lockstep(int enabled)
{
    if (!g_bridge.initialized || !g_bridge.shm) {
        return;
    }
    __atomic_store_n(&g_bridge.shm->lockstep_enabled, enabled ? 1 : 0, __ATOMIC_RELEASE);
}

uint64_t aviate_gz_get_sim_step(void)
{
    if (!g_bridge.initialized || !g_bridge.shm) {
        return 0;
    }
    return __atomic_load_n(&g_bridge.shm->sim_step, __ATOMIC_ACQUIRE);
}

void aviate_gz_ack_step(uint64_t step)
{
    if (!g_bridge.initialized || !g_bridge.shm) {
        return;
    }
    __atomic_store_n(&g_bridge.shm->fc_step_ack, step, __ATOMIC_RELEASE);
}

}  // extern "C"
