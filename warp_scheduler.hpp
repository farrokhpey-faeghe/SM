#pragma once
#include <vector>
#include <deque>
#include <cstdint>
#include <limits>
#include <optional>
#include <cassert>
#include <iostream>
#include <string>
#include <algorithm>
#include <map>
#include <queue>
#pragma once
#include <vector>
#include <queue>
#include "gpu_interfaces.hpp"
#include "gpu_types.hpp"

/*
enum class SchedPolicy
{
    LRR,
    GTO,
    TWO_LEVEL
};
enum class OpClass
{
    SP,
    SFU,
    MEM,
    BARRIER,
    NOP
};

struct InstMeta
{
    OpClass op = OpClass::SP;
    ...
    uint32_t latency = 4;
};
*/
// --- END REMOVAL ---

class RegisterScoreboard : public ScoreboardIfc
{
public:
    RegisterScoreboard(uint32_t num_warps, uint32_t num_regs)
        : reg_ready_cycle_(num_warps, std::vector<uint64_t>(num_regs, 0)), cur_cycle_(0), num_regs_(num_regs) {}

    void set_cycle(uint64_t cyc) { cur_cycle_ = cyc; }

    // REPLACE THIS ENTIRE FUNCTION:
    bool is_ready(uint32_t warp_id, const InstMeta &imeta) const override
    {
        if (warp_id >= reg_ready_cycle_.size())
            return false;

        const auto &rf = reg_ready_cycle_[warp_id]; // FIXED: Only check THIS warp

        // Check source registers
        for (uint32_t r : imeta.src_regs)
        {
            if (r >= num_regs_ || rf[r] > cur_cycle_)
                return false;
        }

        // Check destination register
        if (imeta.dst_reg)
        {
            uint32_t d = *imeta.dst_reg;
            if (d >= num_regs_ || rf[d] > cur_cycle_)
                return false;
        }

        return true;
    }

    void on_issue(uint32_t warp_id, const InstMeta &imeta, uint64_t issue_cycle) override
    {
        if (imeta.dst_reg && warp_id < reg_ready_cycle_.size())
        {
            reg_ready_cycle_[warp_id][*imeta.dst_reg] = issue_cycle + imeta.latency;
        }
    }

private:
    std::vector<std::vector<uint64_t>> reg_ready_cycle_;
    uint64_t cur_cycle_;
    uint32_t num_regs_;
};

// FU Resource (simple model)
struct SimpleFU : FUResourcesIfc
{
    uint32_t sp_per_cyc = 1, sfu_per_cyc = 1, mem_per_cyc = 1;
    mutable uint32_t sp_left = 0, sfu_left = 0, mem_left = 0;

    void new_cycle() override
    {
        sp_left = sp_per_cyc;
        sfu_left = sfu_per_cyc;
        mem_left = mem_per_cyc;
    }
    bool can_issue(OpClass opc) const override
    {
        switch (opc)
        {
        case OpClass::SP:
            return sp_left > 0;
        case OpClass::SFU:
            return sfu_left > 0;
        case OpClass::MEM:
            return mem_left > 0;
        case OpClass::BARRIER:
        case OpClass::NOP:
            return true;
        default:
            return true;
        }
    }
    void reserve(OpClass opc) override
    {
        switch (opc)
        {
        case OpClass::SP:
            assert(sp_left > 0);
            --sp_left;
            break;
        case OpClass::SFU:
            assert(sfu_left > 0);
            --sfu_left;
            break;
        case OpClass::MEM:
            assert(mem_left > 0);
            --mem_left;
            break;
        case OpClass::BARRIER:
        case OpClass::NOP:
        default:
            break;
        }
    }
};

struct WarpState
{
    uint32_t warp_id = 0;
    uint32_t cta_id = 0;
    std::deque<InstMeta> iq;
    bool valid = true;
    bool at_barrier = false;
    bool finished = false;
    uint64_t age = 0;
};

struct SchedConfig
{
    SchedPolicy policy = SchedPolicy::GTO;
    uint32_t issue_width = 2;
    uint32_t num_pools = 2;
    bool verbose = false;
};

struct Pool
{
    std::vector<uint32_t> warps;
    uint32_t rr_next = 0;
};

class CTABarrier
{
public:
    void set_cta_size(uint32_t cta_id, uint32_t num_warps)
    {
        cta_total_[cta_id] = num_warps;
        cta_arrived_[cta_id] = 0;
        cta_released_[cta_id] = false;
    }
    void arrive(uint32_t cta_id)
    {
        auto &arrived_count = cta_arrived_[cta_id];
        if (!cta_released_[cta_id])
        {
            arrived_count++;
            if (arrived_count >= cta_total_[cta_id])
            {
                cta_released_[cta_id] = true;
            }
        }
    }
    bool is_released(uint32_t cta_id) const
    {
        auto it = cta_released_.find(cta_id);
        return it != cta_released_.end() && it->second;
    }

private:
    std::map<uint32_t, uint32_t> cta_total_;
    std::map<uint32_t, uint32_t> cta_arrived_;
    std::map<uint32_t, bool> cta_released_;
};

class WarpScheduler
{
public:
    WarpScheduler(const SchedConfig &cfg, ScoreboardIfc &sb, FUResourcesIfc &fu)
        : cfg_(cfg), sb_(sb), fu_(fu) {}

    void add_warp(uint32_t warp_id, const std::vector<InstMeta> &instructions, uint32_t cta_id = 0)
    {
        if (warp_id >= warps_.size())
            warps_.resize(warp_id + 1);
        auto &w = warps_[warp_id];
        w.warp_id = warp_id;
        w.cta_id = cta_id;
        w.valid = true;
        w.finished = false;
        w.at_barrier = false;
        w.age = std::numeric_limits<uint64_t>::max() - warp_id;
        w.iq.clear();
        for (const auto &inst : instructions)
        {
            w.iq.push_back(inst);
        }
    }

    void set_cta_size(uint32_t cta_id, uint32_t num_warps)
    {
        barrier_manager_.set_cta_size(cta_id, num_warps);
    }

    bool all_warps_finished() const
    {
        for (const auto &w : warps_)
        {
            if (w.valid && !w.finished)
            {
                return false;
            }
        }
        return true;
    }
    void build_pools()
    {
        pools_.assign(std::max<uint32_t>(1, cfg_.num_pools), {});
        uint32_t p_idx = 0;
        for (uint32_t w_idx = 0; w_idx < warps_.size(); ++w_idx)
        {
            if (warps_[w_idx].valid)
            {
                pools_[p_idx].warps.push_back(w_idx);
                p_idx = (p_idx + 1) % pools_.size();
            }
        }
    }

    void tick()
    {
        fu_.new_cycle();
        if (auto *rsb = dynamic_cast<RegisterScoreboard *>(&sb_))
            rsb->set_cycle(sched_cycle_);
        for (const auto &ws : warps_)
        {
            if (ws.valid && !ws.finished)
            {
                bool cta_has_barrier = std::any_of(warps_.begin(), warps_.end(),
                                                   [&](const auto &other_warp)
                                                   {
                                                       return other_warp.valid &&
                                                              other_warp.cta_id == ws.cta_id &&
                                                              other_warp.at_barrier &&
                                                              !barrier_manager_.is_released(ws.cta_id);
                                                   });
                if (cta_has_barrier)
                {
                    stats.stall_barrier++;
                }
            }
        }

        issued_this_cycle_ = 0;
        switch (cfg_.policy)
        {
        case SchedPolicy::LRR:
            schedule_lrr();
            break;
        case SchedPolicy::GTO:
            schedule_gto();
            break;
        case SchedPolicy::TWO_LEVEL:
            schedule_twolevel();
            break;
        }
        sched_cycle_++;
    }

    struct Stats
    {
        uint64_t issued_total = 0;
        uint64_t stall_scoreboard = 0;
        uint64_t stall_fu = 0;
        uint64_t stall_barrier = 0;
    } stats;

private:
    void schedule_lrr()
    {
        if (warps_.empty())
            return;
        //
        for (uint32_t tried = 0; tried < warps_.size() && issued_this_cycle_ < cfg_.issue_width; ++tried)
        {
            uint32_t w = (lrr_next_ + tried) % warps_.size();
            if (try_issue_from_warp(w))
            {
                //
                lrr_next_ = (w + 1) % warps_.size();
            }
        }
    }
    void schedule_gto()
    {
        if (issued_this_cycle_ >= cfg_.issue_width)
            return;

        for (uint32_t w = 0; w < warps_.size(); ++w)
        {
            if (!is_warp_schedulable(w))
                continue;
            if (warps_[w].iq.empty())
                continue;

            const auto &im = warps_[w].iq.front();
            if (!sb_.is_ready(w, im))
            {
                if (cfg_.verbose)
                {
                    std::cout << "[cyc " << sched_cycle_ << "] Warp " << w
                              << " stalled on scoreboard\n";
                }
                stats.stall_scoreboard++;
                continue;
            }
            if (!fu_.can_issue(im.op) && im.op != OpClass::BARRIER)
            {
                stats.stall_fu++;
                continue;
            }

            // Found a ready warp, try to issue
            if (try_issue_from_warp(w))
            {
                last_winner_ = w;
                break; // Only issue one instruction per cycle in GTO mode
            }
        }
    }

    void schedule_twolevel()
    {
        if (pools_.empty())
            build_pools();
        if (pools_.empty())
            return;

        for (uint32_t pt = 0; pt < pools_.size() && issued_this_cycle_ < cfg_.issue_width; ++pt)
        {
            uint32_t pidx = (pool_rr_next_ + pt) % pools_.size();
            auto &P = pools_[pidx];
            if (P.warps.empty())
                continue;

            uint32_t tried = 0;
            while (tried < P.warps.size() && issued_this_cycle_ < cfg_.issue_width)
            {
                uint32_t idx = (P.rr_next + tried) % P.warps.size();
                uint32_t w = P.warps[idx];
                if (try_issue_from_warp(w))
                {
                    P.rr_next = (idx + 1) % P.warps.size();
                    break;
                }
                tried++;
            }
        }
        // pool round-robin pointer
        pool_rr_next_ = (pool_rr_next_ + 1) % pools_.size();
    }

    // Helper to check if a warp is eligible to be considered for scheduling
    bool is_warp_schedulable(uint32_t w_idx) const
    {
        if (w_idx >= warps_.size())
            return false;
        const auto &ws = warps_[w_idx];
        if (!ws.valid || ws.finished || ws.iq.empty())
            return false;
        // If at barrier, check if the barrier has been released
        if (ws.at_barrier && !barrier_manager_.is_released(ws.cta_id))
        {
            if (cfg_.verbose)
            {
                std::cout << "[cyc " << sched_cycle_ << "] Warp " << w_idx
                          << " blocked at barrier\n";
            }
            return false;
        }

        // Check if any warp in the same CTA is at a barrier
        if (!ws.at_barrier)
        {
            for (const auto &other_warp : warps_)
            {
                if (other_warp.valid && other_warp.cta_id == ws.cta_id &&
                    other_warp.at_barrier && !barrier_manager_.is_released(ws.cta_id))
                {
                    if (cfg_.verbose)
                    {
                        std::cout << "[cyc " << sched_cycle_ << "] Warp " << w_idx
                                  << " waiting for CTA barrier\n";
                    }
                    return false;
                }
            }
        }
        return true;
    }

    // Main issue logic for a single warp
    bool try_issue_from_warp(uint32_t w_idx)
    {
        if (!is_warp_schedulable(w_idx))
        {
            if (w_idx < warps_.size())
            {
                const auto &ws = warps_[w_idx];
                if (ws.at_barrier || (ws.valid &&
                                      std::any_of(warps_.begin(), warps_.end(),
                                                  [&](const auto &other_warp)
                                                  {
                                                      return other_warp.valid &&
                                                             other_warp.cta_id == ws.cta_id &&
                                                             other_warp.at_barrier &&
                                                             !barrier_manager_.is_released(ws.cta_id);
                                                  })))
                {
                    stats.stall_barrier++;
                }
            }
            return false;
        }

        auto &ws = warps_[w_idx];
        auto &im = ws.iq.front();
        if (im.op == OpClass::BARRIER)
        {
            ws.iq.pop_front();
            ws.at_barrier = true;
            barrier_manager_.arrive(ws.cta_id);
            stats.issued_total++;
            issued_this_cycle_++;

            if (barrier_manager_.is_released(ws.cta_id))
            {
                for (auto &warp_state : warps_)
                {
                    if (warp_state.valid && warp_state.cta_id == ws.cta_id)
                    {
                        warp_state.at_barrier = false;
                    }
                }
            }
            if (ws.iq.empty())
                ws.finished = true;

            if (cfg_.verbose)
            {
                std::cout << "[cyc " << sched_cycle_ << "] Warp " << w_idx
                          << " reached barrier (total: " << stats.issued_total << ")\n";
            }
            return true;
        }

        if (!sb_.is_ready(w_idx, im))
        {
            stats.stall_scoreboard++;
            return false;
        }
        if (!fu_.can_issue(im.op))
        {
            stats.stall_fu++;
            return false;
        }

        fu_.reserve(im.op);
        sb_.on_issue(w_idx, im, sched_cycle_);
        ws.iq.pop_front();
        ws.age = global_age_++;
        issued_this_cycle_++;
        stats.issued_total++;
        if (cfg_.verbose)
        {
            std::cout << "[cyc " << sched_cycle_ << "] Issued warp " << w_idx
                      << " op " << static_cast<int>(im.op)
                      << " (total: " << stats.issued_total << ")\n";
        }
        if (ws.iq.empty())
            ws.finished = true;
        return true;
    }

private:
    SchedConfig cfg_;
    ScoreboardIfc &sb_;
    FUResourcesIfc &fu_;
    CTABarrier barrier_manager_;

    std::vector<WarpState> warps_;
    std::vector<Pool> pools_;

    uint32_t lrr_next_ = 0;
    std::optional<uint32_t> last_winner_;
    uint32_t pool_rr_next_ = 0;

    uint64_t sched_cycle_ = 0;
    uint64_t global_age_ = 1;
    uint32_t issued_this_cycle_ = 0;
};