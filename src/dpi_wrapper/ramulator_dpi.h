#ifndef RAMULATOR_DPI_WRAPPER_H
#define RAMULATOR_DPI_WRAPPER_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// Opaque handle to Ramulator instance
typedef void* ramulator_handle_t;

// Initialize Ramulator with YAML config file
ramulator_handle_t ramulator_init(const char* config_file);

// Send a memory request (returns 1 if accepted, 0 if rejected)
// For writes: data is stored in the functional model.
// For reads:  data is ignored.
int ramulator_send_request(
    ramulator_handle_t handle,
    unsigned long long addr,
    int req_type,        // 0 = Read, 1 = Write
    int source_id,
    uint64_t data        // write data (functional model); ignored for reads
);

// Advance memory system clock by one cycle
void ramulator_tick(ramulator_handle_t handle);

// Check if a read request completed.
// Returns the address of the completed request, or -1 if none pending.
// If data_out is non-NULL, the functional model value for that address is
// written to *data_out.
long long ramulator_check_response(ramulator_handle_t handle, uint64_t* data_out);

// Cleanup
void ramulator_finalize(ramulator_handle_t handle);

#ifdef __cplusplus
}
#endif

#endif
