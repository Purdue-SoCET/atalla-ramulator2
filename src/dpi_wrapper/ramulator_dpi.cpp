#include "ramulator_dpi.h"
#include "base/config.h"
#include "frontend/frontend.h"
#include "memory_system/memory_system.h"
#include <queue>
#include <memory>
#include <unordered_map>
#include <stdexcept>
#include <cstdio>
#include <cstdlib>

using namespace Ramulator;

// Pairs a completed address with the functional data value to return.
struct CompletedReq {
    uint64_t addr;
    uint64_t data;
};

static CompletedReq g_resp_fifo[4096];
static size_t g_resp_head = 0;
static size_t g_resp_tail = 0;
static size_t g_resp_count = 0;

static std::unordered_map<uint64_t, uint64_t> g_functional_mem;

static inline void resp_fifo_reset() {
    g_resp_head = 0;
    g_resp_tail = 0;
    g_resp_count = 0;
}

static inline bool resp_fifo_push(uint64_t addr, uint64_t data) {
    if (g_resp_count >= 4096) return false;
    g_resp_fifo[g_resp_tail].addr = addr;
    g_resp_fifo[g_resp_tail].data = data;
    g_resp_tail = (g_resp_tail + 1) % 4096;
    g_resp_count++;
    return true;
}

static inline bool resp_fifo_pop(CompletedReq& out) {
    if (g_resp_count == 0) return false;
    out = g_resp_fifo[g_resp_head];
    g_resp_head = (g_resp_head + 1) % 4096;
    g_resp_count--;
    return true;
}

struct RamulatorWrapper {
    std::unique_ptr<IFrontEnd>    frontend;
    std::unique_ptr<IMemorySystem> memory_system;

    std::queue<CompletedReq> completed_requests;

    // Functional model: shadow memory storing the last value written to each address.
    // Reads to addresses that have never been written return the address itself
    // as a deterministic default.
    std::unordered_map<Addr_t, uint64_t> functional_mem;

    int      mem_tick_ratio;
    int      frontend_tick_ratio;
    uint64_t cycle_count;
};

extern "C" {

ramulator_handle_t ramulator_init(const char* config_file) {
    try {
        YAML::Node config = Config::parse_config_file(config_file, {});

        auto* wrapper = new RamulatorWrapper();

        wrapper->frontend.reset(Factory::create_frontend(config));
        wrapper->memory_system.reset(Factory::create_memory_system(config));

        if (!wrapper->frontend || !wrapper->memory_system) {
            fprintf(stderr, "[ramulator_init] ERROR: failed to create frontend or memory system\n");
            delete wrapper;
            return nullptr;
        }

        wrapper->frontend->connect_memory_system(wrapper->memory_system.get());
        wrapper->memory_system->connect_frontend(wrapper->frontend.get());

        wrapper->frontend_tick_ratio = wrapper->frontend->get_clock_ratio();
        wrapper->mem_tick_ratio      = wrapper->memory_system->get_clock_ratio();
        wrapper->cycle_count         = 0;

        if (wrapper->frontend_tick_ratio <= 0) wrapper->frontend_tick_ratio = 1;
        if (wrapper->mem_tick_ratio <= 0)      wrapper->mem_tick_ratio = 1;

        resp_fifo_reset();
        g_functional_mem.clear();

        return static_cast<ramulator_handle_t>(wrapper);
    } catch (const std::exception& e) {
        fprintf(stderr, "[ramulator_init] ERROR: %s\n", e.what());
        return nullptr;
    } catch (...) {
        fprintf(stderr, "[ramulator_init] ERROR: unknown exception\n");
        return nullptr;
    }
}

int ramulator_send_request(
    ramulator_handle_t handle,
    unsigned long long addr,
    int req_type,
    int source_id,
    uint64_t data
) {
    auto* wrapper = static_cast<RamulatorWrapper*>(handle);
    if (!wrapper) return 0;

    try {
        const uint64_t uaddr = static_cast<uint64_t>(addr);

        if (req_type == 1) {
            g_functional_mem[uaddr] = data;
        }

        auto callback = [req_type](Request& req) {
            if (req_type != 0) return;

            const uint64_t raddr = static_cast<uint64_t>(req.addr);

            uint64_t val = 0;
            auto it = g_functional_mem.find(raddr);
            if (it != g_functional_mem.end()) {
                val = it->second;
            } else {
                val = raddr;
            }

            if (!resp_fifo_push(raddr, val)) {
                fprintf(stderr, "[callback] ERROR: completion FIFO overflow addr=0x%llx\n",
                        (unsigned long long)raddr);
            }
        };

        bool accepted = wrapper->frontend->receive_external_requests(
            req_type, uaddr, source_id, callback
        );

        return accepted ? 1 : 0;
    } catch (const std::exception& e) {
        fprintf(stderr, "[ramulator_send_request] ERROR: %s\n", e.what());
        return 0;
    } catch (...) {
        fprintf(stderr, "[ramulator_send_request] ERROR: unknown exception\n");
        return 0;
    }
}

void ramulator_tick(ramulator_handle_t handle) {
    auto* wrapper = static_cast<RamulatorWrapper*>(handle);
    if (!wrapper) return;

    wrapper->cycle_count++;

    const int tick_mult = wrapper->frontend_tick_ratio * wrapper->mem_tick_ratio;
    if (tick_mult <= 0) return;

    if (((wrapper->cycle_count % tick_mult) % wrapper->mem_tick_ratio) == 0) {
        wrapper->frontend->tick();
    }

    if (((wrapper->cycle_count % tick_mult) % wrapper->frontend_tick_ratio) == 0) {
        wrapper->memory_system->tick();
    }
}

long long ramulator_check_response(ramulator_handle_t handle, long long* data_out) {
    (void)handle;

    CompletedReq cr;
    if (!resp_fifo_pop(cr)) {
        return -1;
    }

    if (data_out) {
        *data_out = static_cast<long long>(cr.data);
    }

    return static_cast<long long>(cr.addr);
}

void ramulator_finalize(ramulator_handle_t handle) {
    auto* wrapper = static_cast<RamulatorWrapper*>(handle);
    delete wrapper;
}

// Called from SV instead of $finish to bypass QuestaSim's post-simulation
// cleanup, which hits the heap corruption left by Ramulator2.
// _Exit() terminates the process immediately without running destructors,
// atexit handlers, or any simulator teardown code.
void ramulator_exit(int code) {
    std::_Exit(code);
}

} // extern "C"
