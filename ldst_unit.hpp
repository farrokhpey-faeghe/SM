#pragma once
#include <array>
#include <vector>
#include <deque>
#include <unordered_map>
#include <cstdint>
#include <cassert>
#include <utility>
#include "l1_arch.hpp"
#include <sst/core/serialization/serializable.h>

// Warp-level Request (from core)

struct WarpMemReq
{
    uint32_t warp_id = 0;
    bool is_write = false;
    bool is_atomic = false;
    bool is_volatile = false;
    uint32_t bytes_per_lane = 4;
    std::array<Addr, 32> addrs{};
    std::array<bool, 32> active{};
    uint64_t user_tag = 0;
    CacheOp cop = CacheOp::DEFAULT;

    void serialize_order(SST::Core::Serialization::serializer &ser)
    {
        ser & warp_id;
        ser & is_write;
        ser & is_atomic;
        ser & is_volatile;
        ser & bytes_per_lane;

        // --- Manually serialize the arrays ---
        for (int i = 0; i < 32; ++i)
        {
            ser &addrs[i];
        }
        for (int i = 0; i < 32; ++i)
        {
            ser &active[i];
        }
        // --- End of manual serialization ---

        ser & user_tag;
        ser & cop;
    }
};
// Warp Completion Report
struct WarpDone
{
    uint32_t warp_id = 0;
    uint64_t user_tag = 0;
    uint32_t txns_issued = 0;
    uint32_t lanes_touched = 0;
    uint32_t cycles_in_flight = 0;

    // Add serialization
    void serialize_order(SST::Core::Serialization::serializer &ser)
    {
        ser & warp_id;
        ser & user_tag;
        ser & txns_issued;
        ser & lanes_touched;
        ser & cycles_in_flight;
    }
};

// Coalescer (warp → cache txns)

namespace Coalescer
{

    static std::vector<CoalescedTxn> coalesce(
        uint64_t ldst_req_id,
        const WarpMemReq &wreq,
        const CacheConfig &cfg)
    {
        struct Bucket
        {
            Addr line_base = 0;
            uint32_t mask = 0;
        };
        std::unordered_map<uint64_t, Bucket> buckets;
        buckets.reserve(8);

        for (uint32_t lane = 0; lane < 32; ++lane)
        {
            if (!wreq.active[lane])
                continue;
            Addr a = wreq.addrs[lane];
            uint64_t la = cfg.line(a); // line address
            auto it = buckets.find(la);
            if (it == buckets.end())
            {
                buckets.emplace(la, Bucket{la * cfg.line_size, (1u << lane)});
            }
            else
            {
                it->second.mask |= (1u << lane);
            }
        }

        std::vector<CoalescedTxn> out;
        out.reserve(buckets.size());
        for (auto &kv : buckets)
        {
            CoalescedTxn t{};
            t.line_base = kv.second.line_base;
            t.lane_mask = kv.second.mask;
            t.is_write = wreq.is_write;
            t.is_atomic = wreq.is_atomic;
            t.is_volatile = wreq.is_volatile;
            t.cop = wreq.cop;
            t.warp_id = wreq.warp_id;
            t.bytes_per_lane = wreq.bytes_per_lane;
            t.req_tag = ldst_req_id;
            out.push_back(t);
        }
        return out;
    }
}

// LD/ST Unit (front-end to L1)
class LDSTUnit
{
public:
    struct Stats
    {
        uint64_t warps_enqueued = 0;
        // --- ADDITIONS START ---
        uint64_t read_warps_enqueued = 0;
        uint64_t write_warps_enqueued = 0;
        // --- ADDITIONS END ---
        uint64_t warps_completed = 0;
        uint64_t txns_issued_total = 0;
        uint64_t lanes_total = 0;
        uint64_t outstanding_max = 0;
        uint64_t bytes_completed = 0;
        double avg_txn_per_warp = 0.0;
        double avg_lanes_per_txn = 0.0;
        // --- ADDITION START ---
        double write_percent = 0.0;
        // --- ADDITION END ---
    };
    LDSTUnit(const CacheConfig &cfg, uint32_t queue_depth = 64)
        : cfg_(cfg), queue_depth_(queue_depth), current_cycle_(0) {}

    void set_issue_per_cycle(uint32_t n) { issue_per_cycle_ = (n == 0 ? 1 : n); }

    uint64_t get_cycle() const { return current_cycle_; }

    bool enqueue(const WarpMemReq &wreq)
    {
        if (request_queue_.size() >= queue_depth_)
            return false;

        // --- ADDITIONS START ---
        // Count reads vs. writes
        if (wreq.is_write)
        {
            stats_.write_warps_enqueued++;
        }
        else
        {
            stats_.read_warps_enqueued++;
        }
        // --- ADDITIONS END ---

        request_queue_.push_back(wreq);
        stats_.warps_enqueued++; // This still counts the total
        return true;
    }

    // --- THIS TICK IS MODIFIED ---
    void tick()
    {
        current_cycle_++;

        // 1) Issue new warps and COALESCE them
        uint32_t issued_warps = 0;
        while (!request_queue_.empty() && issued_warps < issue_per_cycle_)
        {
            const auto &wreq = request_queue_.front();
            const uint64_t req_id = next_req_id_++;

            auto txns = Coalescer::coalesce(req_id, wreq, cfg_);
            if (txns.empty())
            {
                request_queue_.pop_front();
                continue;
            }

            InFlightReq &trk = in_flight_[req_id];
            trk.warp_id = wreq.warp_id;
            trk.user_tag = wreq.user_tag;
            trk.txn_total = static_cast<uint32_t>(txns.size());
            trk.txn_left = static_cast<uint32_t>(txns.size());
            trk.active_lanes = 0;
            trk.enqueue_cycle = get_cycle(); // Use its own cycle

            for (const auto &t : txns)
            {
                trk.active_lanes += popcount32(t.lane_mask);
                // l1_.pushTxn(t); // <-- DELETED
                txns_to_l1_.push_back(t); // <-- ADDED: Save to output queue
            }

            stats_.txns_issued_total += txns.size();
            stats_.lanes_total += trk.active_lanes;
            if (in_flight_.size() > stats_.outstanding_max)
                stats_.outstanding_max = in_flight_.size();

            request_queue_.pop_front();
            issued_warps++;
        }

        // 2) --- DELETED ---
        // l1_.tick();
        // auto resps = l1_.drainResponses();
        // for (auto &r : resps)
        //     onResp(r);

        // 3) ... (stats calculation from your original file) ...
        if (stats_.warps_completed > 0)
            stats_.avg_txn_per_warp = double(stats_.txns_issued_total) / double(stats_.warps_completed);
        if (stats_.txns_issued_total > 0)
            stats_.avg_lanes_per_txn = double(stats_.lanes_total) / double(stats_.txns_issued_total);

        // --- ADDITIONS START ---
        // Calculate write percent
        if (stats_.warps_enqueued > 0)
        {
            stats_.write_percent = (double(stats_.write_warps_enqueued) / double(stats_.warps_enqueued)) * 100.0;
        }
        // --- ADDITIONS END ---
    }

    // --- THIS IS A NEW PUBLIC FUNCTION ---
    // The "boss" component calls this
    void onResp(const MemResp &r)
    {
        auto it = in_flight_.find(r.req_tag);
        if (it == in_flight_.end())
            return;

        auto &trk = it->second;
        stats_.bytes_completed += r.bytes_in_txn;
        assert(trk.txn_left > 0 && "More responses than issued!");
        trk.txn_left--;

        if (trk.txn_left == 0)
        {
            WarpDone wd{};
            wd.warp_id = trk.warp_id;
            wd.user_tag = trk.user_tag;
            wd.txns_issued = trk.txn_total;
            wd.lanes_touched = trk.active_lanes;
            wd.cycles_in_flight = get_cycle() - trk.enqueue_cycle;
            done_.push_back(wd);

            stats_.warps_completed++;
            in_flight_.erase(it);
        }
    }

    // The "boss" calls these to get the output queues
    std::vector<WarpDone> drainCompletedWarps()
    {
        std::vector<WarpDone> out;
        out.swap(done_);
        return out;
    }

    std::vector<CoalescedTxn> drainL1Transactions()
    {
        std::vector<CoalescedTxn> out;
        out.swap(txns_to_l1_);
        return out;
    }

    const Stats &getStats() const { return stats_; }

private:
    struct InFlightReq
    {
        uint32_t warp_id = 0;
        uint64_t user_tag = 0;
        uint32_t txn_total = 0;
        uint32_t txn_left = 0;
        uint32_t active_lanes = 0;
        uint32_t enqueue_cycle = 0;
    };

private:
    const CacheConfig &cfg_;
    // L1SectorCache &l1_; // <-- DELETED
    const uint32_t queue_depth_;
    uint32_t issue_per_cycle_ = 4;
    uint64_t current_cycle_; // <-- ADDED

    std::deque<WarpMemReq> request_queue_;
    std::unordered_map<uint64_t, InFlightReq> in_flight_;
    uint64_t next_req_id_ = 1;

    std::vector<WarpDone> done_;
    Stats stats_;

    // --- ADDED: OUTPUT QUEUE ---
    std::vector<CoalescedTxn> txns_to_l1_;
};