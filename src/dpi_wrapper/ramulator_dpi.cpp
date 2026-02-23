#include "ramulator_dpi.h"
#include "base/config.h"
#include "frontend/frontend.h"
#include "memory_system/memory_system.h"
#include <queue>
#include <memory>
#include <unordered_map>
#include <tuple>

using namespace Ramulator;
using RequestInfo = std::tuple<long long, int>;

struct RamulatorWrapper {
    std::unique_ptr<IFrontEnd> frontend;
    std::unique_ptr<IMemorySystem> memory_system;
    std::unordered_map<Addr_t, RequestInfo> req_times;
    std::queue<Addr_t> completed_requests;
    int mem_tick_ratio;
    int frontend_tick_ratio;
    uint64_t cycle_count;
};

extern "C" {

ramulator_handle_t ramulator_init(const char* config_file) {
    try {
        // Parse config
        YAML::Node config = Config::parse_config_file(config_file, {});
        
        // Create wrapper
        auto* wrapper = new RamulatorWrapper();
        
        // Create frontend and memory system
        wrapper->frontend.reset(Factory::create_frontend(config));
        wrapper->memory_system.reset(Factory::create_memory_system(config));
        
        // Connect them
        wrapper->frontend->connect_memory_system(wrapper->memory_system.get());
        wrapper->memory_system->connect_frontend(wrapper->frontend.get());
        
        // Get clock ratios
        wrapper->frontend_tick_ratio = wrapper->frontend->get_clock_ratio();
        wrapper->mem_tick_ratio = wrapper->memory_system->get_clock_ratio();
        wrapper->cycle_count = 0;
        
        return static_cast<ramulator_handle_t>(wrapper);
    } catch (...) {
        return nullptr;
    }
}

int ramulator_send_request(
    ramulator_handle_t handle,
    unsigned long long addr,
    int req_type,
    int source_id
) {
    auto* wrapper = static_cast<RamulatorWrapper*>(handle);
    
    // Callback to track completed requests
    auto callback = [wrapper](Request& req) {
        wrapper->completed_requests.push(req.addr);
    };
    
    // Use receive_external_requests like GEM5 frontend
    bool accepted = wrapper->frontend->receive_external_requests(
        req_type, addr, source_id, callback
    );
    
    if (accepted) {
        wrapper->req_times[addr] = std::make_tuple(wrapper->cycle_count, req_type);
        printf("Request type %d at address %x is accepted\n", req_type, addr);
        return 1;
    } else {
        return 0;
    }
    // return accepted ? 1 : 0;
}

void ramulator_tick(ramulator_handle_t handle) {
    auto* wrapper = static_cast<RamulatorWrapper*>(handle);
    
    wrapper->cycle_count++;
    
    // Tick frontend if needed
    int tick_mult = wrapper->frontend_tick_ratio * wrapper->mem_tick_ratio;
    if (((wrapper->cycle_count % tick_mult) % wrapper->mem_tick_ratio) == 0) {
        wrapper->frontend->tick();
    }
    
    // Tick memory system if needed
    if ((wrapper->cycle_count % tick_mult) % wrapper->frontend_tick_ratio == 0) {
        wrapper->memory_system->tick();
    }
}

long long ramulator_check_response(ramulator_handle_t handle) {
    auto* wrapper = static_cast<RamulatorWrapper*>(handle);
    
    if (wrapper->completed_requests.empty()) {
        return -1;
    }
    
    Addr_t addr = wrapper->completed_requests.front();
    wrapper->completed_requests.pop();
    auto [timestamp, req_type] = wrapper->req_times[addr];
    printf("Tick difference for address %x, req_type %d: %ld\n", addr, req_type, wrapper->cycle_count - timestamp);

    return addr;
}

void ramulator_finalize(ramulator_handle_t handle) {
    auto* wrapper = static_cast<RamulatorWrapper*>(handle);
    wrapper->frontend->finalize();
    wrapper->memory_system->finalize();
    delete wrapper;
}

} // extern "C"
