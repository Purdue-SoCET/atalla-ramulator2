#ifndef RAMULATOR_DPI_WRAPPER_H
#define RAMULATOR_DPI_WRAPPER_H

#ifdef __cplusplus
extern "C" {
#endif

// Opaque handle to Ramulator instance
typedef void* ramulator_handle_t;

// Initialize Ramulator with YAML config file
ramulator_handle_t ramulator_init(const char* config_file);

// Send a memory request (returns 1 if accepted, 0 if rejected)
int ramulator_send_request(
    ramulator_handle_t handle,
    unsigned long long addr,
    int req_type,        // 0 = Read, 1 = Write
    int source_id
);

// Advance memory system clock by one cycle
void ramulator_tick(ramulator_handle_t handle);

// Check if request completed (returns addr if completed, -1 if none)
long long ramulator_check_response(ramulator_handle_t handle);

// Cleanup
void ramulator_finalize(ramulator_handle_t handle);

#ifdef __cplusplus
}
#endif

#endif

