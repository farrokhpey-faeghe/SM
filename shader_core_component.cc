#include <sst/core/sst_config.h>
#include "shader_core_component.hpp"
#include "gpu_types.hpp"
#include "l1_arch.hpp"
#include "ldst_unit.hpp"
#include "warp_scheduler.hpp"

static void load_cache_config_params(CacheConfig &cfg, SST::Params &params)
{
// Helper macro to simplify loading
#define LOAD_PARAM(T, VAR, DEFAULT_VAL) \
    cfg.VAR = (T)params.find<T>(#VAR, DEFAULT_VAL)
    // Geometry
    LOAD_PARAM(uint32_t, nsets, 64);
    LOAD_PARAM(uint32_t, line_size, 128);
    LOAD_PARAM(uint32_t, assoc, 4);
    // Ports & Banking
    LOAD_PARAM(uint32_t, return_bytes_per_cycle, 8);
    LOAD_PARAM(uint32_t, num_banks, 32);
    LOAD_PARAM(uint32_t, ports_per_bank, 1);
    LOAD_PARAM(uint32_t, tag_ports_per_cycle, 2);
    LOAD_PARAM(uint32_t, load_ports_per_cycle, 1);
    LOAD_PARAM(uint32_t, store_ports_per_cycle, 1);
    // Downstream (NOTE: These are just for L1's internal stats, SST handles the real latency)
    LOAD_PARAM(uint32_t, miss_queue_entries, 64);
    LOAD_PARAM(uint32_t, icnt_req_lat, 20);
    LOAD_PARAM(uint32_t, icnt_resp_lat, 20);
    LOAD_PARAM(bool, l2_hit, true);
    LOAD_PARAM(uint32_t, l2_hit_lat, 60);
    LOAD_PARAM(uint32_t, l2_miss_lat, 200);
    // Return bandwidth
    LOAD_PARAM(uint32_t, sectors_per_cycle_return, 2);
    // MSHR
    LOAD_PARAM(uint32_t, mshr_entries, 64);
    LOAD_PARAM(uint32_t, mshr_max_merge, 16);
    LOAD_PARAM(uint32_t, mshr_per_set, 4);
    // Timing
    LOAD_PARAM(uint32_t, set_install_lock_cycles, 1);
    LOAD_PARAM(uint32_t, tag_access_cycles, 1);
    LOAD_PARAM(uint32_t, data_access_cycles, 1);
    // Policies (Note: Enums need casting from int)
    cfg.repl = (ReplPolicy)params.find<int>("repl", (int)ReplPolicy::LRU);
    cfg.write = (WritePolicy)params.find<int>("write", (int)WritePolicy::WRITE_BACK);
    cfg.alloc = (AllocPolicy)params.find<int>("alloc", (int)AllocPolicy::WRITE_ALLOCATE);
    // Hashing
    LOAD_PARAM(uint32_t, bank_xor_shift, 0);
    LOAD_PARAM(uint32_t, set_xor_shift, 0);

#undef LOAD_PARAM
}

ShaderCoreComponent::ShaderCoreComponent(SST::ComponentId_t id, SST::Params &params)
    : SST::Component(id),
      output_("ShaderCore", 0, 0, SST::Output::STDOUT)
{
    output_.verbose(CALL_INFO, 1, 0, "--- Initializing ShaderCore Component ---\n");

    //  1.Load Parameters
    std::string clock_freq = params.find<std::string>("clock", "1.5GHz");
    uint32_t num_warps = params.find<uint32_t>("num_warps", 32);

    //  2Instantiate the CacheConfig
    cfg_ = std::make_unique<CacheConfig>();
    load_cache_config_params(*cfg_, params);
    // 3. Instantiate the "Engine" Models
    scoreboard_ = std::make_unique<RegisterScoreboard>(num_warps, 128); // 128 regs per warp

    // Use the class name from your file: SimpleFUResources
    fu_model_ = std::make_unique<SimpleFUResources>(2, 1, 1); // Example: 2 SP, 1 SFU, 1 MEM
    // The main engines
    SchedConfig sched_cfg; // Use defaults for now
    scheduler_ = std::make_unique<WarpScheduler>(sched_cfg, *scoreboard_, *fu_model_);
    ldst_unit_ = std::make_unique<LDSTUnit>(*cfg_);
    l1_cache_ = std::make_unique<L1SectorCache>(*cfg_);
    // 4. Configure SST Links
    link_mem_ = configureLink("port_mem", new SST::Event::Handler<ShaderCoreComponent>(this, &ShaderCoreComponent::handle_mem_resp));
    if (!link_mem_)
    {
        output_.fatal(CALL_INFO, -1, "Failed to configure memory link 'port_mem'!\n");
    }
    // 5. Register SST Clock
    registerClock(clock_freq, new SST::Clock::Handler<ShaderCoreComponent>(this, &ShaderCoreComponent::clock_core));
    output_.verbose(CALL_INFO, 1, 0, "--- ShaderCore Initialization Complete ---\n");
}
// --- Destructor Definition ---
ShaderCoreComponent::~ShaderCoreComponent()
{
    // unique_ptrs will handle cleanup automatically
    output_.verbose(CALL_INFO, 1, 0, "--- Destroying ShaderCore Component ---\n");
}

// --- SST setup() ---
void ShaderCoreComponent::setup()
{
    output_.verbose(CALL_INFO, 1, 0, "ShaderCore setup() - Injecting test workload\n");

    // Create a simple test workload: 2 warps in CTA 0
    // Warp 0: MEM op
    // Warp 1: MEM op
    std::vector<InstMeta> warp0_insts;
    warp0_insts.push_back({OpClass::MEM, 0, 0, 0, true, {}, {1}, 20});   // MEM op
    warp0_insts.push_back({OpClass::BARRIER, 0, 0, 1, true, {}, {}, 1}); // Barrier

    std::vector<InstMeta> warp1_insts;
    warp1_insts.push_back({OpClass::MEM, 1, 0, 0, true, {}, {1}, 20});   // MEM op
    warp1_insts.push_back({OpClass::BARRIER, 1, 0, 1, true, {}, {}, 1}); // Barrier

    scheduler_->add_warp(0, warp0_insts, 0);
    scheduler_->add_warp(1, warp1_insts, 0);
    scheduler_->set_cta_size(0, 2); // Tell barrier manager 2 warps are in CTA 0

    scheduler_->build_pools();
}

// --- Event Handler Definition ---
// Handles responses from the L2/Memory
void ShaderCoreComponent::handle_mem_resp(SST::Event *ev)
{
    // Try to cast the generic event to our specific L1TxnResp event
    auto *resp_evt = dynamic_cast<L1TxnResp *>(ev);
    if (resp_evt)
    {
        // This is the CRITICAL FIX from my previous message
        // (This assumes you made the changes to l1_arch.hpp)
        l1_cache_->handleMemoryResponse(resp_evt->resp);
    }
    delete ev; // Always delete the event after handling
}

// Clock Handler Definition
// This is the "heartbeat" that connects all your C++ models
bool ShaderCoreComponent::clock_core(SST::Cycle_t cycle)
{

    // 1. Tick all the engines
    scheduler_->tick();
    ldst_unit_->tick();
    l1_cache_->tick();

    // 2. Connect Scheduler -> LDST Unit
    if (cycle == 10)
    {
        WarpMemReq req{};
        req.warp_id = 0;
        req.is_write = false;
        req.active[0] = true;
        req.addrs[0] = 1024;
        req.user_tag = 1000;
        ldst_unit_->enqueue(req);
        output_.verbose(CALL_INFO, 2, 0, "Cycle %llu: Manually enqueued read for warp 0\n", cycle);
    }
    if (cycle == 12)
    {
        WarpMemReq req{};
        req.warp_id = 1;
        req.is_write = true;
        req.active[0] = true;
        req.addrs[0] = 4096;
        req.user_tag = 1001;
        ldst_unit_->enqueue(req);
        output_.verbose(CALL_INFO, 2, 0, "Cycle %llu: Manually enqueued write for warp 1\n", cycle);
    }

    // 3. Connect LDST Unit -> L1 Cache
    // Get all new coalesced transactions from the LD/ST unit
    auto l1_txns = ldst_unit_->drainL1Transactions();
    for (const auto &txn : l1_txns)
    {
        l1_cache_->pushTxn(txn); // Push them into the L1
    }
    // 4. Connect L1 Cache -> LDST Unit (for completed hits/misses)
    // Get all responses (from hits or completed MSHRs)
    auto l1_resps = l1_cache_->drainResponses();
    for (const auto &resp : l1_resps)
    {
        ldst_unit_->onResp(resp); // Tell the LD/ST unit
    }

    // 5. Connect LDST Unit -> Scheduler (for warp completion)
    // Get all warps that are now finished
    auto completed_warps = ldst_unit_->drainCompletedWarps();
    for (const auto &done : completed_warps)
    {
        // TODO: Tell scheduler/scoreboard that warp 'done.warp_id' is un-stalled
        // and its destination register is ready.
        output_.verbose(CALL_INFO, 2, 0, "Cycle %llu: Warp %u completed mem request\n", cycle, done.warp_id);
    }

    // 6. Connect L1 Cache -> SST Memory Link
    // Check if the L1 has any misses it needs to send to L2
    CoalescedTxn mem_req;
    int safety_counter = 0;
    const int MAX_REQUESTS = 100;

    while (l1_cache_->getNextMemoryRequest(mem_req) && safety_counter < MAX_REQUESTS)
    {
        link_mem_->send(new L1TxnReq(mem_req));
        safety_counter++;
    }

    if (safety_counter >= MAX_REQUESTS)
    {
        std::cerr << "[ERROR] Hit max memory request limit at cycle "
                  << cycle << std::endl;
        return true; // Stop simulation
    }
    // Return false to keep simulating
    // (Return true to end simulation)
    return false;
}