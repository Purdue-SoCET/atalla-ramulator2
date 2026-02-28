#include "ramulator_dpi.h"
#include "base/config.h"
#include "frontend/frontend.h"
#include "memory_system/memory_system.h"
#include <queue>
#include <memory>
#include <unordered_map>
#include <tuple>
#include <stdexcept>
#include <cstdio>

using namespace Ramulator;
using RequestInfo = std::tuple<long long, int>;

// Pairs a completed address with the functional data value to return.
struct CompletedReq {
    Addr_t   addr;
    uint64_t data;
};

struct RamulatorWrapper {
    std::unique_ptr<IFrontEnd>    frontend;
    std::unique_ptr<IMemorySystem> memory_system;

    // Timing model bookkeeping
    std::unordered_map<Addr_t, RequestInfo> req_times;
    std::queue<CompletedReq>                completed_requests;

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

        wrapper->frontend->connect_memory_system(wrapper->memory_system.get());
        wrapper->memory_system->connect_frontend(wrapper->frontend.get());

        wrapper->frontend_tick_ratio = wrapper->frontend->get_clock_ratio();
        wrapper->mem_tick_ratio      = wrapper->memory_system->get_clock_ratio();
        wrapper->cycle_count         = 0;

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

    // Functional model: record write data immediately (no timing needed).
    if (req_type == 1 /* Write */) {
        wrapper->functional_mem[addr] = data;
    }

    // Timing model callback: fires when the DRAM read pipeline completes.
    auto callback = [wrapper](Request& req) {
        uint64_t val = 0;
        auto it = wrapper->functional_mem.find(req.addr);
        if (it != wrapper->functional_mem.end()) {
            val = it->second;
        } else {
            // Address never written — return address as default.
            val = static_cast<uint64_t>(req.addr);
        }
        wrapper->completed_requests.push({req.addr, val});
    };

    bool accepted = wrapper->frontend->receive_external_requests(
        req_type, addr, source_id, callback
    );

    if (accepted) {
        wrapper->req_times[addr] = std::make_tuple(
            static_cast<long long>(wrapper->cycle_count), req_type
        );
        printf("Request type %d at address %llx is accepted\n", req_type, addr);
        return 1;
    }
    return 0;
}

void ramulator_tick(ramulator_handle_t handle) {
    auto* wrapper = static_cast<RamulatorWrapper*>(handle);

    wrapper->cycle_count++;

    int tick_mult = wrapper->frontend_tick_ratio * wrapper->mem_tick_ratio;
    if (((wrapper->cycle_count % tick_mult) % wrapper->mem_tick_ratio) == 0) {
        wrapper->frontend->tick();
    }
    if ((wrapper->cycle_count % tick_mult) % wrapper->frontend_tick_ratio == 0) {
        wrapper->memory_system->tick();
    }
}

long long ramulator_check_response(ramulator_handle_t handle, uint64_t* data_out) {
    auto* wrapper = static_cast<RamulatorWrapper*>(handle);

    if (wrapper->completed_requests.empty()) {
        return -1;
    }

    CompletedReq cr = wrapper->completed_requests.front();
    wrapper->completed_requests.pop();

    if (data_out) {
        *data_out = cr.data;
    }

    auto it = wrapper->req_times.find(cr.addr);
    if (it != wrapper->req_times.end()) {
        auto [timestamp, req_type] = it->second;
        printf("Tick difference for address %llx, req_type %d: %lld\n",
               (unsigned long long)cr.addr, req_type,
               (long long)wrapper->cycle_count - timestamp);
    }

    return static_cast<long long>(cr.addr);
}

void ramulator_finalize(ramulator_handle_t handle) {
    auto* wrapper = static_cast<RamulatorWrapper*>(handle);
    wrapper->frontend->finalize();
    wrapper->memory_system->finalize();
    delete wrapper;
}

} // extern "C"
