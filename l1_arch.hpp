#pragma once
#include <cstdint>
#include <vector>
#include <deque>
#include <unordered_map>
#include <optional>
#include <array>
#include <random>
#include <limits>
#include <cassert>
#include <algorithm>
#include <iostream>
#include <iomanip> // For std::setw
#include <sst/core/serialization/serializable.h>

using Addr = uint64_t;

// Utils
static inline uint32_t popcount32(uint32_t x)
{
#if defined(__GNUG__) || defined(__clang__)
    return __builtin_popcount(x);
#else
    uint32_t c = 0;
    while (x)
    {
        x &= (x - 1);
        ++c;
    }
    return c;
#endif
}

// Policies/Enums
enum class ReplPolicy
{
    LRU,
    FIFO,
    RAND
};
enum class WritePolicy
{
    WRITE_BACK,
    WRITE_THROUGH
};
enum class AllocPolicy
{
    WRITE_ALLOCATE,
    NO_WRITE_ALLOCATE
};

// Cache operators
enum class CacheOp
{
    DEFAULT,
    CA,
    CG,
    CS,
    LU,
    READ_ONLY
};

struct CacheConfig
{
    // Geometry
    uint32_t nsets = 64;      // 64 * 4 * 128 = 32KB
    uint32_t line_size = 128; // 4 x 32B sectors
    uint32_t assoc = 4;

    // Per-SM DRAM return cap (bytes/cycle) ~ V100: 900/80/1.53 ≈ 7.35 → 8
    uint32_t return_bytes_per_cycle = 8;

    // Banking & ports
    uint32_t num_banks = 32;
    uint32_t ports_per_bank = 1;
    // tag/data ports (load/store جدا)
    uint32_t tag_ports_per_cycle = 2;
    uint32_t load_ports_per_cycle = 1;
    uint32_t store_ports_per_cycle = 1;

    // Downstream (ICNT/L2/DRAM)
    uint32_t miss_queue_entries = 64;
    uint32_t icnt_req_lat = 20;
    uint32_t icnt_resp_lat = 20;
    bool l2_hit = true;
    uint32_t l2_hit_lat = 60;
    uint32_t l2_miss_lat = 200;

    // Return bandwidth (beat=32B)
    uint32_t sectors_per_cycle_return = 2;

    // MSHR
    uint32_t mshr_entries = 64;
    uint32_t mshr_max_merge = 16;
    uint32_t mshr_per_set = 4;
    uint32_t set_install_lock_cycles = 1;
    uint32_t tag_access_cycles = 1;
    uint32_t data_access_cycles = 1;
    ReplPolicy repl = ReplPolicy::LRU;
    WritePolicy write = WritePolicy::WRITE_BACK;
    AllocPolicy alloc = AllocPolicy::WRITE_ALLOCATE;

    uint32_t bank_xor_shift = 0;
    uint32_t set_xor_shift = 0;

    inline uint64_t line(Addr a) const { return a / line_size; }
    inline uint64_t tagOf(Addr a) const { return a & ~(Addr)(line_size - 1); }

    inline uint32_t setIdx(Addr a) const
    {
        uint64_t la = line(a);
        if (set_xor_shift)
            la ^= (la >> set_xor_shift);
        return (uint32_t)(la % nsets);
    }
    inline uint32_t bankOf(Addr a) const
    {
        uint64_t la = line(a);
        if (bank_xor_shift)
            la ^= (la >> bank_xor_shift);
        return (uint32_t)(la % num_banks);
    }
};

struct SectorState
{
    bool valid = false, dirty = false, pending = false;
};

struct SectorLine
{
    uint64_t tag = 0;
    bool line_valid = false;
    bool evict_first = false; // for CS/LU
    uint64_t age = 0;
    std::array<SectorState, 4> sec{};
};

struct SectorSet
{
    std::vector<SectorLine> ways;
};

class SectorTagArray
{
public:
    SectorTagArray() = default;
    explicit SectorTagArray(const CacheConfig &cfg) { reset(cfg); }

    void reset(const CacheConfig &cfg)
    {
        cfg_ = cfg;
        sets_.assign(cfg_.nsets, {});
        for (auto &s : sets_)
            s.ways.assign(cfg_.assoc, {});
        age_ = 1;
        sector_size_ = cfg_.line_size / 4; // 32B
    }

    struct Probe
    {
        bool tag_match = false;
        bool full_hit = false;
        uint32_t set = 0, way = 0;
        uint8_t need_mask = 0;
        uint8_t have_mask = 0;
        bool eviction = false;
        uint64_t evict_tag = 0;
        uint8_t evict_dirty_mask = 0;
    };

    uint32_t sectorIdx(Addr a) const
    {
        uint64_t off = a % cfg_.line_size;
        return uint32_t(off / sector_size_);
    }

    uint8_t sectorMask(Addr line_base, uint32_t lane_mask, uint32_t bytes_per_lane) const
    {
        uint8_t m = 0;
        for (uint32_t lane = 0; lane < 32; ++lane)
        {
            if ((lane_mask >> lane) & 1u)
            {
                Addr a = line_base + (Addr)lane * bytes_per_lane;
                m |= (1u << sectorIdx(a));
            }
        }
        if (m == 0)
            m = 0xF;
        return m;
    }

    Probe probe(Addr line_base, uint32_t lane_mask, uint32_t bytes_per_lane, bool is_write, CacheOp cop)
    {
        Probe r{};
        r.set = cfg_.setIdx(line_base);
        auto &set = sets_[r.set];
        const auto tag = cfg_.tagOf(line_base);
        r.need_mask = sectorMask(line_base, lane_mask, bytes_per_lane);

        for (uint32_t w = 0; w < set.ways.size(); ++w)
        {
            auto &ln = set.ways[w];
            if (ln.line_valid && ln.tag == tag)
            {
                r.tag_match = true;
                r.way = w;
                ln.age = ++age_;
                uint8_t have = 0;
                for (int s = 0; s < 4; ++s)
                    if (ln.sec[s].valid && !ln.sec[s].pending)
                        have |= (1u << s);
                r.have_mask = have;
                r.full_hit = ((have & r.need_mask) == r.need_mask);

                if (is_write && cop != CacheOp::READ_ONLY)
                {
                    if (cfg_.write == WritePolicy::WRITE_BACK)
                    {
                        for (int s = 0; s < 4; ++s)
                            if ((r.need_mask >> s) & 1u && ((have >> s) & 1u))
                                ln.sec[s].dirty = true;
                    }
                }
                if (cop == CacheOp::LU)
                    ln.evict_first = true;
                return r;
            }
        }

        r.tag_match = false;
        r.way = selectVictim(set);
        const auto &vic = set.ways[r.way];
        if (vic.line_valid)
        {
            r.eviction = true;
            r.evict_tag = vic.tag;
            uint8_t dm = 0;
            for (int s = 0; s < 4; ++s)
                if (vic.sec[s].dirty)
                    dm |= (1u << s);
            r.evict_dirty_mask = dm;
        }
        return r;
    }

    void markPending(uint32_t set, uint32_t way, uint8_t pend_mask)
    {
        auto &ln = sets_[set].ways[way];
        ln.line_valid = true;
        for (int s = 0; s < 4; ++s)
            if ((pend_mask >> s) & 1u)
                ln.sec[s].pending = true;
    }

    void install(uint32_t set, uint32_t way, Addr line_base, uint8_t filled_mask, bool make_dirty, CacheOp cop)
    {
        auto &ln = sets_[set].ways[way];
        ln.tag = cfg_.tagOf(line_base);
        ln.line_valid = true;
        ln.age = ++age_;
        ln.evict_first = (cop == CacheOp::CS);
        for (int s = 0; s < 4; ++s)
        {
            if ((filled_mask >> s) & 1u)
            {
                ln.sec[s].valid = true;
                ln.sec[s].pending = false;
                if (make_dirty && cfg_.write == WritePolicy::WRITE_BACK)
                    ln.sec[s].dirty = true;
            }
        }
    }

private:
    uint32_t selectVictim(SectorSet &set)
    {
        for (uint32_t w = 0; w < set.ways.size(); ++w)
            if (!set.ways[w].line_valid)
                return w;
        for (uint32_t w = 0; w < set.ways.size(); ++w)
            if (set.ways[w].evict_first)
                return w;
        uint32_t v = 0;
        uint64_t best = std::numeric_limits<uint64_t>::max();
        for (uint32_t w = 0; w < set.ways.size(); ++w)
            if (set.ways[w].age < best)
            {
                best = set.ways[w].age;
                v = w;
            }
        return v;
    }

    CacheConfig cfg_{};
    std::vector<SectorSet> sets_;
    uint64_t age_ = 1;
    uint32_t sector_size_ = 32;
};

// Txn & Resp
struct CoalescedTxn : public SST::Core::Serialization::serializable
{
    Addr line_base = 0;     // 128B aligned
    uint32_t lane_mask = 0; // 32 lanes
    bool is_write = false;
    bool is_atomic = false;
    bool is_volatile = false;
    CacheOp cop = CacheOp::DEFAULT;

    uint32_t warp_id = 0;
    uint32_t bytes_per_lane = 4;
    uint64_t req_tag = 0; // upper-level request id

    void serialize_order(SST::Core::Serialization::serializer &ser) override
    {
        ser & line_base;
        ser & lane_mask;
        ser & is_write;
        ser & is_atomic;
        ser & is_volatile;
        ser & cop;
        ser & warp_id;
        ser & bytes_per_lane;
        ser & req_tag;
    }
    ImplementSerializable(CoalescedTxn);
};

struct MemResp : public SST::Core::Serialization::serializable
{
    uint64_t req_tag = 0;
    uint32_t bytes_in_txn = 0;

    MemResp() {}

    MemResp(uint64_t tag, uint32_t bytes) : req_tag(tag), bytes_in_txn(bytes) {}
    void serialize_order(SST::Core::Serialization::serializer &ser) override
    {
        ser & req_tag;
        ser & bytes_in_txn;
    }
    ImplementSerializable(MemResp);
};

struct HitPipe
{
    uint64_t req_tag = 0;
    uint32_t bytes = 0;
    uint32_t ready_cycle = 0;
};

struct MSHRRow
{
    Addr line_addr = 0;
    uint8_t total_mask = 0;
    uint8_t pending_mask = 0;
    uint8_t returned_mask = 0;
    uint8_t install_dirty_mask = 0;
    uint32_t ready_cycle = 0;
    bool inflight_down = false;
    uint32_t set_id = 0;

    struct Merge
    {
        uint64_t req_tag = 0;
        uint8_t need_mask = 0;
        uint32_t bytes = 0;
        bool is_write = false;
    };
    std::vector<Merge> merged;
};

struct MSHRReady
{
    Addr line_addr = 0;
    uint8_t arrived_mask = 0;
    uint8_t install_dirty_mask = 0;
    std::vector<MSHRRow::Merge> satisfied;
    uint32_t set_id = 0;
};

class SectorMSHR
{
public:
    explicit SectorMSHR(const CacheConfig &cfg) : cfg_(cfg) {}
    bool has(Addr la) const { return rows_.count(la); }
    size_t size() const { return rows_.size(); }
    bool inflight(Addr la) const
    {
        auto it = rows_.find(la);
        return it != rows_.end() && it->second.inflight_down;
    }
    uint8_t pendingMask(Addr la) const
    {
        auto it = rows_.find(la);
        return (it == rows_.end() ? 0 : it->second.pending_mask);
    }

    bool alloc_or_extend(Addr la, uint8_t need_mask, uint32_t now, uint32_t set_id, bool &is_new)
    {
        auto it = rows_.find(la);
        if (it == rows_.end())
        {
            if (rows_.size() >= cfg_.mshr_entries)
                return false;
            MSHRRow r;
            r.line_addr = la;
            r.total_mask = need_mask;
            r.pending_mask = need_mask;
            r.returned_mask = 0;
            r.ready_cycle = now;
            r.inflight_down = false;
            r.set_id = set_id;
            rows_.emplace(la, std::move(r));
            is_new = true;
        }
        else
        {
            it->second.total_mask |= need_mask;
            it->second.pending_mask |= need_mask;
            is_new = false;
        }
        return true;
    }

    bool merge(Addr la, uint64_t req_tag, uint8_t need_mask, bool is_write, uint32_t bytes)
    {
        auto it = rows_.find(la);
        if (it == rows_.end())
            return false;
        auto &row = it->second;
        if (row.merged.size() >= cfg_.mshr_max_merge)
            return false;
        row.merged.push_back({req_tag, need_mask, bytes, is_write});
        if (is_write)
            row.install_dirty_mask |= need_mask;
        return true;
    }

    void setInflight(Addr la, bool v)
    {
        auto it = rows_.find(la);
        if (it != rows_.end())
            it->second.inflight_down = v;
    }
    void onDownstreamReady(Addr la, uint32_t now)
    {
        auto it = rows_.find(la);
        if (it != rows_.end())
            it->second.ready_cycle = now;
    }
    uint32_t setOfUnsafe(Addr la) const
    {
        auto it = rows_.find(la);
        return (it == rows_.end() ? 0u : it->second.set_id);
    }

    void retire(uint32_t now, std::vector<MSHRReady> &out, std::vector<Addr> &freed_rows,
                uint32_t sectors_per_cycle)
    {
        for (auto it = rows_.begin(); it != rows_.end();)
        {
            auto &r = it->second;
            if (r.ready_cycle <= now && r.pending_mask)
            {
                uint8_t chunk = 0;
                int cnt = 0;
                for (int s = 0; s < 4; ++s)
                {
                    if ((r.pending_mask >> s) & 1u)
                    {
                        chunk |= (1u << s);
                        if (++cnt == (int)sectors_per_cycle)
                            break;
                    }
                }
                r.pending_mask &= ~chunk;
                r.returned_mask |= chunk;

                std::vector<MSHRRow::Merge> satisfied;
                {
                    auto &vec = r.merged;
                    for (int i = (int)vec.size() - 1; i >= 0; --i)
                    {
                        if ((r.returned_mask & vec[i].need_mask) == vec[i].need_mask)
                        {
                            satisfied.push_back(vec[i]);
                            vec.erase(vec.begin() + i);
                        }
                    }
                }

                out.push_back({r.line_addr, chunk, r.install_dirty_mask, std::move(satisfied), r.set_id});
                r.ready_cycle = now + 1;
                if (r.pending_mask == 0 && r.merged.empty())
                {
                    freed_rows.push_back(r.line_addr);
                    it = rows_.erase(it);
                    continue;
                }
            }
            ++it;
        }
    }

private:
    const CacheConfig &cfg_;
    std::unordered_map<Addr, MSHRRow> rows_;
};

// Miss Queue & WB Queue
struct DownRet
{
    Addr line_addr = 0;
    uint8_t req_mask = 0;
    uint32_t ready_cycle = 0;
};

class MissQueue
{
public:
    explicit MissQueue(const CacheConfig &cfg) : cfg_(cfg) {}
    bool canPush() const { return q_.size() < cfg_.miss_queue_entries; }
    size_t occupancy() const { return q_.size(); }

    void push(Addr la, uint8_t smask, uint32_t now)
    {
        uint32_t l2 = cfg_.l2_hit ? cfg_.l2_hit_lat : cfg_.l2_miss_lat;
        uint32_t rt = cfg_.icnt_req_lat + l2 + cfg_.icnt_resp_lat;
        q_.push_back({la, smask, now + rt});
    }

    void tick(uint32_t now, std::vector<DownRet> &out)
    {
        while (!q_.empty() && q_.front().ready_cycle <= now)
        {
            out.push_back(q_.front());
            q_.pop_front();
        }
    }

private:
    const CacheConfig &cfg_;
    std::deque<DownRet> q_;
};

struct WBPkt
{
    Addr line_addr = 0;
    uint8_t sector_mask = 0;
    uint32_t ready_cycle = 0;
};

class WBQueue
{
public:
    explicit WBQueue(const CacheConfig &cfg) : cfg_(cfg) {}
    bool canPush() const { return q_.size() < cfg_.miss_queue_entries; }
    size_t occupancy() const { return q_.size(); }

    void push(Addr la, uint8_t dmask, uint32_t now)
    {
        uint32_t l2 = cfg_.l2_hit ? cfg_.l2_hit_lat : cfg_.l2_miss_lat;
        uint32_t rt = cfg_.icnt_req_lat + l2 + cfg_.icnt_resp_lat;
        q_.push_back({la, dmask, now + rt});
    }

    void tick(uint32_t now, std::vector<WBPkt> &out)
    {
        while (!q_.empty() && q_.front().ready_cycle <= now)
        {
            out.push_back(q_.front());
            q_.pop_front();
        }
    }

private:
    const CacheConfig &cfg_;
    std::deque<WBPkt> q_;
};

class L1SectorCache
{
public:
    bool enable_debug_ = false;

    void enable_debug(bool v) { enable_debug_ = v; }
    explicit L1SectorCache(const CacheConfig &cfg)
        : cfg_(cfg), tags_(cfg), mshr_(cfg), missq_(cfg), wbq_(cfg)
    {
        bank_q_.assign(cfg_.num_banks, {});
        issue_count_.assign(cfg_.num_banks, 0);
        set_mshr_occ_.assign(cfg_.nsets, 0);
        set_lock_until_.assign(cfg_.nsets, 0);
    }

    uint32_t get_cycle() const { return cycle_; }
    uint32_t dram_budget_bytes_ = 0;
    std::deque<HitPipe> deferred_mem_;

    struct Stats
    {
        uint64_t line_hits = 0, partial_hits = 0, misses = 0;
        uint64_t mshr_allocs = 0, mshr_merges = 0, mshr_set_block = 0;
        uint64_t writebacks = 0, writeback_bytes = 0;
        uint64_t bank_conflicts = 0, responses = 0;
        uint64_t txns_tagged = 0, txns_sent_down = 0, txns_bypassed = 0;
        uint64_t missq_occupancy_max = 0, wbq_occupancy_max = 0;
    } stats;

    void pushTxn(const CoalescedTxn &t)
    {
        PipeReq pr{t, cycle_, cfg_.bankOf(t.line_base)};
        auto &q = bank_q_[pr.bank];
        if (q.size() >= cfg_.ports_per_bank)
            stats.bank_conflicts++; //
        q.push_back(pr);
    }

    void tick()
    {
        cycle_++;
        if (enable_debug_ && cycle_ % 100 == 0)
        {
            std::cout << "[L1 DEBUG cyc=" << cycle_ << "] "
                      << "MSHR entries=" << mshr_.size()
                      << " pending_installs=" << pending_installs_.size()
                      << " bank_q_total=" << get_total_bank_queue_size()
                      << " hitQ=" << hitQ_.size()
                      << std::endl;
        }

        // hit
        uint32_t load_budget = cfg_.load_ports_per_cycle;
        uint32_t store_budget = cfg_.store_ports_per_cycle;

        // hits
        for (int i = (int)deferred_hits_.size() - 1; i >= 0 && (load_budget || store_budget); --i)
        {
            auto pr = deferred_hits_[i];
            uint32_t bytes = popcount32(pr.t.lane_mask) * pr.t.bytes_per_lane;
            bool is_store = pr.t.is_write;
            if ((is_store && store_budget == 0) || (!is_store && load_budget == 0))
                continue;
            uint32_t ready = cycle_ + cfg_.data_access_cycles;
            hitQ_.push_back({pr.t.req_tag, bytes, ready});
            (is_store ? store_budget : load_budget)--;
            stats.line_hits++;
            deferred_hits_.erase(deferred_hits_.begin() + i);
        }

        // decoupled miss sender
        while (!pending_send_down_.empty() && missq_.canPush())
        {
            Addr la = pending_send_down_.front();
            pending_send_down_.pop_front();
            if (!mshr_.has(la) || mshr_.inflight(la))
                continue;
            uint8_t mask = mshr_.pendingMask(la);
            if (mask == 0)
                continue;
            missq_.push(la, mask, cycle_);
            stats.txns_sent_down++;
            mshr_.setInflight(la, true);
        }
        stats.missq_occupancy_max = std::max<uint64_t>(stats.missq_occupancy_max, missq_.occupancy());
        stats.wbq_occupancy_max = std::max<uint64_t>(stats.wbq_occupancy_max, wbq_.occupancy());

        // A) TAG issue
        for (auto &c : issue_count_)
            c = 0;
        uint32_t tag_budget = cfg_.tag_ports_per_cycle;
        for (uint32_t b = 0; b < cfg_.num_banks && tag_budget; ++b)
        {
            uint32_t bank_budget = cfg_.ports_per_bank;
            while (bank_budget && tag_budget && !bank_q_[b].empty())
            {
                tagQ_.push_back(bank_q_[b].front());
                bank_q_[b].pop_front();
                issue_count_[b]++;
                bank_budget--;
                tag_budget--;
                stats.txns_tagged++;
            }
        }

        // B) Lookup
        for (int i = (int)tagQ_.size() - 1; i >= 0; --i)
        {
            auto pr = tagQ_[i];
            tagQ_.erase(tagQ_.begin() + i);
            auto &t = pr.t;

            // bypass
            if (t.cop == CacheOp::CG || t.is_atomic)
            {
                if (missq_.canPush())
                {
                    uint8_t need_mask = 0xF;
                    missq_.push(cfg_.line(t.line_base), need_mask, cycle_);
                    stats.txns_bypassed++;
                    stats.txns_sent_down++;
                    bypass_waiters_[cfg_.line(t.line_base)]
                        .push_back({t.req_tag, popcount32(t.lane_mask) * t.bytes_per_lane});
                }
                else
                {
                    bank_q_[pr.bank].push_front(pr);
                }
                continue;
            }

            bool ro = (t.cop == CacheOp::READ_ONLY);
            auto p = tags_.probe(t.line_base, t.lane_mask, t.bytes_per_lane, t.is_write && !ro, t.cop);

            if (p.full_hit)
            {
                uint32_t bytes = popcount32(t.lane_mask) * t.bytes_per_lane;
                bool is_store = t.is_write;
                if ((is_store && store_budget) || (!is_store && load_budget))
                {
                    uint32_t ready = cycle_ + cfg_.tag_access_cycles + cfg_.data_access_cycles;
                    hitQ_.push_back({t.req_tag, bytes, ready});
                    (is_store ? store_budget : load_budget)--;
                    stats.line_hits++;
                }
                else
                {
                    deferred_hits_.push_back(pr);
                }
                continue;
            }

            if (p.tag_match)
                stats.partial_hits++;
            uint8_t miss_mask = p.tag_match ? (uint8_t)(p.need_mask & ~p.have_mask) : p.need_mask;

            if (t.is_write && cfg_.alloc == AllocPolicy::NO_WRITE_ALLOCATE)
            {
                stats.misses++;
                if (missq_.canPush())
                {
                    missq_.push(cfg_.line(t.line_base), miss_mask, cycle_);
                    stats.txns_sent_down++;
                    bypass_waiters_[cfg_.line(t.line_base)]
                        .push_back({t.req_tag, popcount32(t.lane_mask) * t.bytes_per_lane});
                }
                else
                {
                    bank_q_[pr.bank].push_front(pr);
                }
                continue;
            }

            stats.misses++;
            Addr la = cfg_.line(t.line_base);
            uint32_t set_id = p.set;

            if (!mshr_.has(la))
            {
                if (set_mshr_occ_[set_id] >= cfg_.mshr_per_set || set_lock_until_[set_id] > cycle_)
                {
                    stats.mshr_set_block++;
                    bank_q_[pr.bank].push_front(pr);
                    continue;
                }
            }

            bool is_new = false;
            if (!mshr_.alloc_or_extend(la, miss_mask, cycle_, set_id, is_new))
            {
                bank_q_[pr.bank].push_front(pr);
                continue;
            }
            if (is_new)
            {
                stats.mshr_allocs++;
                set_mshr_occ_[set_id]++;
            }
            else
            {
                stats.mshr_merges++;
            }

            (void)mshr_.merge(la, t.req_tag, miss_mask, t.is_write && !ro,
                              popcount32(t.lane_mask) * t.bytes_per_lane);

            if (!mshr_.inflight(la))
            {
                if (missq_.canPush())
                {
                    missq_.push(la, miss_mask, cycle_);
                    stats.txns_sent_down++;
                    mshr_.setInflight(la, true);
                }
                else
                {
                    pending_send_down_.push_back(la);
                }
            }

            // MSHR logic
            if (!p.tag_match)
            {
                if (p.eviction && p.evict_dirty_mask)
                {
                    stats.writebacks++;
                    stats.writeback_bytes += 32u * popcount32(p.evict_dirty_mask);
                    if (wbq_.canPush())
                        wbq_.push(cfg_.line(t.line_base), p.evict_dirty_mask, cycle_);
                }
                tags_.markPending(p.set, p.way, miss_mask);
                set_lock_until_[p.set] = std::max<uint32_t>(set_lock_until_[p.set], cycle_ + cfg_.set_install_lock_cycles);
                pending_installs_[la] = {p.set, p.way, t.line_base, t.cop};
            }
            else
            {

                tags_.markPending(p.set, p.way, miss_mask);
                set_lock_until_[p.set] = std::max<uint32_t>(set_lock_until_[p.set], cycle_ + cfg_.set_install_lock_cycles);
                pending_installs_[la] = {p.set, p.way, t.line_base, t.cop};
            }
        }
        std::vector<DownRet> dres;
        missq_.tick(cycle_, dres);
        for (auto &r : dres)
        {
            Addr la = r.line_addr;
            auto itbw = bypass_waiters_.find(la);
            if (itbw != bypass_waiters_.end())
            {
                for (auto &bw : itbw->second)
                {
                    emit_mem_resp(bw.req_tag, bw.bytes);
                }
                bypass_waiters_.erase(itbw);
                continue;
            }
            if (mshr_.has(la))
            {
                mshr_.setInflight(la, false);
                mshr_.onDownstreamReady(la, cycle_);
            }
        }

        std::vector<WBPkt> wbret;
        wbq_.tick(cycle_, wbret);
        (void)wbret;
        std::vector<MSHRReady> ready_rows;
        std::vector<Addr> freed_rows;
        mshr_.retire(cycle_, ready_rows, freed_rows, cfg_.sectors_per_cycle_return);

        for (auto &row : ready_rows)
        {
            auto it = pending_installs_.find(row.line_addr);
            if (it == pending_installs_.end())
            {
                std::cerr << "[L1 ERROR cyc=" << cycle_
                          << "] MSHR retired line_addr=" << row.line_addr
                          << " but pending_installs_ not found!" << std::endl;

                uint32_t set_id = row.set_id;
                uint32_t way = 0;
                Addr line_base = row.line_addr * cfg_.line_size;
                pending_installs_[row.line_addr] = {set_id, way, line_base, CacheOp::DEFAULT};
                it = pending_installs_.find(row.line_addr);
            }
            auto &pin = it->second;

            bool make_dirty = (row.install_dirty_mask != 0) && (cfg_.write == WritePolicy::WRITE_BACK);
            tags_.install(pin.set, pin.way, pin.line_base, row.arrived_mask, make_dirty, pin.cop);

            for (auto &m : row.satisfied)
            {

                emit_mem_resp(m.req_tag, m.bytes);
            }
        }

        for (auto la : freed_rows)
        {
            auto itpin = pending_installs_.find(la);
            if (itpin != pending_installs_.end())
            {
                uint32_t s = itpin->second.set;
                if (s < set_mshr_occ_.size() && set_mshr_occ_[s] > 0)
                    set_mshr_occ_[s]--;
                pending_installs_.erase(itpin);
            }
        }

        size_t done_before = done_.size();
        for (int i = (int)hitQ_.size() - 1; i >= 0; --i)
        {
            if (hitQ_[i].ready_cycle <= cycle_)
            {
                done_.push_back(MemResp{hitQ_[i].req_tag, hitQ_[i].bytes});
                hitQ_.erase(hitQ_.begin() + i);
            }
        }
        size_t done_after = done_.size();
        if (done_after > done_before)
            stats.responses += (done_after - done_before);
    }

    std::vector<MemResp> drainResponses()
    {
        std::vector<MemResp> out;
        out.swap(done_);
        return out;
    }
    bool getNextMemoryRequest(CoalescedTxn &outTxn)
    {
        if (pending_send_down_.empty())
            return false;

        Addr la = pending_send_down_.front();
        pending_send_down_.pop_front();
        if (!mshr_.has(la))
        {
            return false;
        }

        if (mshr_.inflight(la))
        {
            return false;
        }

        uint8_t mask = mshr_.pendingMask(la);
        if (mask == 0)
        {
            return false;
        }
        mshr_.setInflight(la, true);

        CoalescedTxn tx{};
        tx.line_base = (Addr)la * (Addr)cfg_.line_size;
        tx.lane_mask = mask;
        tx.is_write = false;
        tx.req_tag = la;

        outTxn = tx;
        return true;
    }

    void handleMemoryResponse(const MemResp &resp)
    {
        done_.push_back(resp);
        stats.responses++;
    }

private:
    struct PipeReq
    {
        CoalescedTxn t;
        uint32_t enq_cycle;
        uint32_t bank;
    };
    struct PendingInstall
    {
        uint32_t set, way;
        Addr line_base;
        CacheOp cop;
    };
    struct BypassWait
    {
        uint64_t req_tag;
        uint32_t bytes;
    };
    size_t get_total_bank_queue_size() const
    {
        size_t total = 0;
        for (const auto &q : bank_q_)
        {
            total += q.size();
        }
        return total;
    }
    void emit_mem_resp(uint64_t req_tag, uint32_t bytes)
    {
        uint32_t rate = std::max<uint32_t>(1, cfg_.return_bytes_per_cycle);
        uint32_t extra = ((bytes + rate - 1) / rate) - 1; // ceil(bytes/rate)-1
        uint32_t ready = cycle_ + extra;
        hitQ_.push_back({req_tag, bytes, ready});
    }

    CacheConfig cfg_;
    SectorTagArray tags_;
    SectorMSHR mshr_;
    MissQueue missq_;
    WBQueue wbq_;

    uint32_t cycle_ = 0;

    std::vector<std::deque<PipeReq>> bank_q_;
    std::vector<uint32_t> issue_count_;
    std::vector<PipeReq> tagQ_;
    std::vector<PipeReq> deferred_hits_;
    std::vector<HitPipe> hitQ_;
    std::unordered_map<Addr, PendingInstall> pending_installs_;

    std::deque<Addr> pending_send_down_;
    std::unordered_map<Addr, std::vector<BypassWait>> bypass_waiters_;

    std::vector<uint32_t> set_mshr_occ_;
    std::vector<uint32_t> set_lock_until_;

    std::vector<MemResp> done_;
};

// Demo main
#ifdef L1_ARCH
#include <iostream>
#include <unordered_map>
int main()
{
    CacheConfig cfg;
    cfg.nsets = 64;
    cfg.assoc = 4;
    cfg.line_size = 128;
    cfg.num_banks = 32;
    cfg.ports_per_bank = 1;
    cfg.tag_ports_per_cycle = 2;
    cfg.load_ports_per_cycle = 1;
    cfg.store_ports_per_cycle = 1;
    cfg.icnt_req_lat = 20;
    cfg.icnt_resp_lat = 20;
    cfg.l2_hit = true;
    cfg.l2_hit_lat = 60;
    cfg.l2_miss_lat = 200;
    cfg.sectors_per_cycle_return = 2;
    cfg.return_bytes_per_cycle = 8;
    cfg.tag_access_cycles = 1;
    cfg.data_access_cycles = 1;
    cfg.set_xor_shift = 2;
    cfg.bank_xor_shift = 2;

    L1SectorCache l1(cfg);

    auto issue = [&](uint32_t warp, Addr base, Addr stride, bool isW, CacheOp cop = CacheOp::DEFAULT)
    {
        std::unordered_map<uint64_t, uint32_t> buckets;
        for (uint32_t lane = 0; lane < 32; ++lane)
        {
            Addr a = base + (Addr)stride * lane;
            buckets[cfg.line(a)] |= (1u << lane);
        }
        for (auto &kv : buckets)
        {
            CoalescedTxn tx;
            tx.line_base = kv.first * cfg.line_size;
            tx.lane_mask = kv.second;
            tx.is_write = isW;
            tx.cop = cop;
            tx.warp_id = warp;
            tx.bytes_per_lane = 4;
            tx.req_tag = ((uint64_t)warp << 32) | (kv.first & 0xffffffffull);
            l1.pushTxn(tx);
        }
    };

    // misses
    issue(0, 1024, 4, true, CacheOp::DEFAULT);
    issue(1, 4096, 4, false, CacheOp::DEFAULT);

    for (int c = 0; c < 200; ++c)
    {
        l1.tick();
        (void)l1.drainResponses();
    }

    // hits
    issue(2, 1024, 4, false, CacheOp::DEFAULT);
    issue(3, 4096 + 16, 0, false, CacheOp::DEFAULT);

    // bypass CG
    issue(4, 8192, 4, false, CacheOp::CG);

    for (int c = 0; c < 300; ++c)
    {
        l1.tick();
        (void)l1.drainResponses();
    }

    std::cout << "\n--- L1 Stats ---\n";
    std::cout << "Hits=" << l1.stats.line_hits
              << "  PartialHits=" << l1.stats.partial_hits
              << "  Misses=" << l1.stats.misses
              << "  MSHR_allocs=" << l1.stats.mshr_allocs
              << "  MSHR_merges=" << l1.stats.mshr_merges
              << "  MSHR_set_blocks=" << l1.stats.mshr_set_block
              << "  WB=" << l1.stats.writebacks << " (" << l1.stats.writeback_bytes << "B)"
              << "  BankConflicts=" << l1.stats.bank_conflicts
              << "  TxnsTagged=" << l1.stats.txns_tagged
              << "  Down=" << l1.stats.txns_sent_down
              << "  Bypassed=" << l1.stats.txns_bypassed
              << "  MissQmax=" << l1.stats.missq_occupancy_max
              << "  WBQmax=" << l1.stats.wbq_occupancy_max
              << "  Resp=" << l1.stats.responses
              << "\n";
    return 0;
}
#endif // L1_ARCH

// g++ -std=c++17 -O2 -Wall -DL1_ARCH demo2.cpp -o l1_demo2
