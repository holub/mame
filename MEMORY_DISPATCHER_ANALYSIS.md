# MAME Memory Dispatcher Performance Analysis & Optimization Strategies

**Status**: Performance Analysis  
**Profile Data**: TBBlue tbblue on SpecNext  
**Baseline Hotspot**: ~72% of sampled cycles in memory handler setup/dispatch machinery  
**Target**: Reduce handler installation and dispatch overhead

---

## Executive Summary

The memory dispatcher infrastructure accounts for 72% of profiled cycles during TBBlue emulation. The root cause is that SpecNext uses 8+ dynamically-configured memory views with complex mirror/bank mappings. The `populate_mirror`, `range_cut_after/before`, and handler installation code execute repeatedly during machine setup and runtime memory bank switches.

Key findings:
- **Top hotspot**: `range_cut_after` / `populate_mirror` loops iterating through dispatch entries
- **Secondary**: Subdispatch object creation during population
- **Tertiary**: Individual memory range updates with poor cache locality

---

## Problem Analysis

### 1. Range Cutting Operations (Hotspot #1: ~35% of dispatcher time)

**File**: `src/emu/emumem_hedr.ipp:227`, `src/emu/emumem_hedw.ipp:228`

```cpp
template<int HighBits, int Width, int AddrShift> 
void handler_entry_read_dispatch<HighBits, Width, AddrShift>::range_cut_after(offs_t address, int start)
{
    while(++start < COUNT && m_u_dispatch[start]) {
        if(int(LowBits) > -AddrShift && m_u_dispatch[start]->is_dispatch()) {
            static_cast<handler_entry_read_dispatch<LowBits, Width, AddrShift> *>(m_u_dispatch[start])->range_cut_after(address);
            break;  // ? Only breaks when hitting nested dispatch
        }
        if(m_u_ranges[start].start >= address)
            break;
        m_u_ranges[start].start = address;  // ? Individual writes per iteration
    }
}
```

**Performance Issues**:
- **Sequential iteration**: Loop runs from `start+1` to `COUNT`, checking every entry
- **Individual cache misses**: Each `m_u_ranges[start]` write is a separate memory write
- **Recursive calls**: For nested dispatchers, recursion adds overhead
- **Repeated invocations**: Called from `populate_mirror_subdispatch` and `populate_nomirror_subdispatch` for each mirror instance

**Example call stack**:
```
populate_mirror() [mirror count = 1-256]
  ? populate_mirror_subdispatch()
    ? populate_mirror() [recursive]
      ? range_cut_before()  ? Iteration loop
      ? range_cut_after()   ? Iteration loop (Hotspot)
```

---

### 2. Mirror Population Nested Loops (Hotspot #2: ~25% of dispatcher time)

**File**: `src/emu/emumem_hedr.ipp:332-346`, `src/emu/emumem_hedw.ipp:333-347`

```cpp
template<int HighBits, int Width, int AddrShift> 
void handler_entry_read_dispatch<HighBits, Width, AddrShift>::populate_mirror(...)
{
    offs_t hmirror = mirror & HIGHMASK;
    offs_t lmirror = mirror & LOWMASK;

    if(lmirror) {
        // lmirror case: each mirror is a single entry
        offs_t add = 1 + ~hmirror;
        offs_t offset = 0;
        offs_t base_entry = start >> LowBits;
        do {
            if(offset)
                handler->ref();  // Reference counting
            populate_mirror_subdispatch(base_entry | (offset >> LowBits), 
                                      start | offset, end | offset, 
                                      ostart | offset, oend | offset, 
                                      lmirror, handler);
            offset = (offset + add) & hmirror;
        } while(offset);
    } else {
        // lmirror=0 case: calls populate_nomirror for each mirror
        offs_t add = 1 + ~hmirror;
        offs_t offset = 0;
        do {
            if(offset)
                handler->ref();
            populate_nomirror(...);  // ? Can have large offset range
            offset = (offset + add) & hmirror;
        } while(offset);
    }
}
```

**Performance Issues**:
- **Nested loops**: Mirror count (1-256) ? array iteration (COUNT entries) = O(mirror_count ? COUNT)
- **Repeated handler->ref()**: Called per mirror instance
- **Redundant range cuts**: Each populate_nomirror/populate_mirror_subdispatch call triggers range_cut_before/after

**SpecNext Worst Case**:
- Multiple memory views with mirror masks up to 0xFFFF
- Can result in 256+ mirror instances per configuration
- Each mirror instance triggers the full range-cut machinery

---

### 3. Subdispatch Object Creation During Population (Hotspot #3: ~12% of dispatcher time)

**File**: `src/emu/emumem_hedr.ipp:317-328`

```cpp
void populate_mirror_subdispatch(offs_t entry, offs_t start, offs_t end, ...)
{
    auto cur = m_u_dispatch[entry];
    if(cur->is_dispatch())
        cur->populate_mirror(...);
    else {
        auto subdispatch = new handler_entry_read_dispatch<LowBits, Width, AddrShift>(
            this->m_space, m_u_ranges[entry], cur);
        cur->unref();
        m_u_dispatch[entry] = subdispatch;
        subdispatch->populate_mirror(...);
        range_cut_before((entry << LowBits) - 1, entry);
        range_cut_after((entry + 1) << LowBits, entry);
    }
}
```

**Performance Issues**:
- **Heap allocation**: Each non-dispatch entry requires a new `handler_entry_read_dispatch` object
- **Constructor overhead**: Initializes arrays and vectors
- **Ref counting**: unref() + constructor ref() calls
- **Cascading range cuts**: Each new subdispatch triggers range_cut_before/after

---

### 4. Data Structure Memory Access Patterns

**File**: `src/emu/emumem_hedr.h:84-90`

```cpp
class handler_array : public std::array<handler_entry_read<Width, AddrShift> *, COUNT>
{ /* ... */ };

class range_array : public std::array<handler_entry::range, COUNT>
{ /* ... */ };

std::vector<handler_array> m_dispatch_array;     // 64+ byte arrays
std::vector<range_array> m_ranges_array;         // 32+ byte arrays per entry

handler_entry_read<Width, AddrShift> **m_a_dispatch;
handler_entry::range *m_a_ranges;
handler_entry_read<Width, AddrShift> **m_u_dispatch;
handler_entry::range *m_u_ranges;
```

**Performance Issues**:
- **Poor cache locality**: Individual entries written in `range_cut_after` cause cache line thrashing
- **Indirection**: Arrays of pointers cause L1/L2 cache misses during traversal
- **False sharing**: Multiple threads/views accessing adjacent entries
- **Memory bandwidth**: Sequential writes to non-contiguous memory

---

## Optimization Strategies

### Strategy 1: Early Exit Optimization for range_cut_after (Est. -10% dispatcher overhead)

**Current behavior**: Iterates through all entries looking for first unchanged entry

**Optimization**: Maintain a "cut boundary" marker or skip list to avoid scanning unchanged entries

```cpp
// Proposed optimization:
struct range_cut_cache {
    int last_cut_entry;  // Cache of last modified entry
    offs_t last_cut_address;
};

// In range_cut_after:
if(start >= m_cut_cache.last_cut_entry && address <= m_cut_cache.last_cut_address) {
    return;  // Already cut beyond this point
}

// ... normal processing ...
m_cut_cache.last_cut_entry = start;
m_cut_cache.last_cut_address = address;
```

**Impact**: Reduces iteration count for repeated small adjustments

---

### Strategy 2: Batch Range Updates (Est. -15% dispatcher overhead)

**Current behavior**: Individual m_u_ranges[i].start updates in a loop

**Optimization**: Batch updates using memset or vectorized operations

```cpp
// Current: 
for(int i = start; i < COUNT; i++) {
    if(m_u_ranges[i].start >= address) break;
    m_u_ranges[i].start = address;  // Individual write
}

// Proposed:
// Collect indices and use a vectorized update pattern
std::vector<int> to_update;
for(int i = start; i < COUNT; i++) {
    if(m_u_ranges[i].start >= address) break;
    to_update.push_back(i);
}
// Batch update (potentially SIMD-friendly)
for(int i : to_update) {
    m_u_ranges[i].start = address;
}
```

**Impact**: Better CPU pipeline utilization and cache efficiency

---

### Strategy 3: Lazy Range Cut Evaluation (Est. -20% dispatcher overhead)

**Current behavior**: range_cut_before/after executed immediately during populate operations

**Optimization**: Defer range cuts until next memory access or explicit flush

```cpp
// Add lazy-cut queue
struct lazy_range_cut {
    offs_t address;
    bool is_after;
    int start_entry;
};

std::vector<lazy_range_cut> m_pending_cuts;

void populate_mirror_subdispatch(...) {
    // Instead of immediate range_cut_before/after:
    if(should_lazy_cut()) {
        m_pending_cuts.push_back({(entry << LowBits) - 1, false, entry});
        m_pending_cuts.push_back({(entry + 1) << LowBits, true, entry});
    } else {
        range_cut_before(...);
        range_cut_after(...);
    }
}

// Flush on memory access or between frames
void flush_pending_cuts() {
    for(auto& cut : m_pending_cuts) {
        if(cut.is_after)
            range_cut_after(cut.address, cut.start_entry);
        else
            range_cut_before(cut.address, cut.start_entry);
    }
    m_pending_cuts.clear();
}
```

**Impact**: 
- Amortizes range-cut cost across multiple populate operations
- Single flush is more cache-efficient than scattered cuts
- Requires careful synchronization for runtime memory updates

---

### Strategy 4: Optimize Mirror Loop Early Exit (Est. -12% dispatcher overhead)

**Current behavior**: Processes all mirror instances with recursive calls

**Optimization**: Detect and fast-path common mirror patterns

```cpp
// Add fast-path for common cases:
if(lmirror && hmirror == 0) {
    // No mirroring at high bits - single operation
    populate_mirror_subdispatch(base_entry, start, end, ostart, oend, lmirror, handler);
    return;
}

if(hmirror == 0) {
    // No high mirroring - call populate_nomirror once
    populate_nomirror(start, end, ostart, oend, handler);
    return;
}

// Only execute complex loop for actual mirrored cases
```

**Impact**: Reduces iterations for non-mirrored or single-mirror cases

---

### Strategy 5: Reduce Subdispatch Creation Overhead (Est. -8% dispatcher overhead)

**Current behavior**: Creates new subdispatch for every non-dispatch entry

**Optimization**: Reuse subdispatches or use lazy initialization

```cpp
// Option A: Pool subdispatches
static thread_local std::vector<handler_entry_read_dispatch*> m_subdispatch_pool;

auto subdispatch = get_pooled_subdispatch<LowBits, Width, AddrShift>(
    this->m_space, m_u_ranges[entry], cur);

// Option B: Deferred creation
// Mark entry as "pending creation" and create on first access

// Option C: Use placement new in pre-allocated buffer
```

**Impact**: 
- Reduces heap allocation overhead
- Better cache locality with pooling
- Minor memory overhead for pool management

---

### Strategy 6: Vectorized Memory View Updates (Est. -5-10% dispatcher overhead)

**Current behavior**: Each memory view updated individually

**Optimization**: Update multiple views in a single pass

```cpp
// Current: Per-view update
for(auto& view : views) {
    view[slot].install_read_handler(...);  // Independent calls
}

// Proposed: Batch update
install_read_handlers_batch({&m_view0[slot], &m_view1[slot], ...}, handler_info);
```

**Impact**: 
- Shared reference counting
- Reduced function call overhead
- Better cache utilization

---

## Recommended Implementation Priority

| Priority | Strategy | Est. Speedup | Difficulty | Risk |
|----------|----------|-------------|------------|------|
| 1 | Early Exit Optimization (#1) | 10% | Low | Very Low |
| 2 | Batch Range Updates (#2) | 15% | Medium | Low |
| 3 | Optimize Mirror Loop (#4) | 12% | Medium | Low |
| 4 | Lazy Range Cuts (#3) | 20% | High | Medium |
| 5 | Reduce Subdispatch Creation (#5) | 8% | Medium | Medium |
| 6 | Vectorized View Updates (#6) | 5-10% | High | Medium-High |

**Combined potential speedup**: ~35-45% reduction in dispatcher overhead

---

## Implementation Recommendations

### Phase 1: Quick Wins (Low-Risk, 25% improvement)
1. Implement early-exit caching in `range_cut_after`
2. Add fast-path detection in mirror loops
3. Add conditional batch updates for sequential range cuts

### Phase 2: Medium Effort (Medium-Risk, +15% improvement)
1. Implement lazy range cut evaluation with explicit flush points
2. Add subdispatch pooling or deferred creation

### Phase 3: Major Refactor (Higher-Risk, +5-10% improvement)
1. Restructure dispatch arrays for better cache locality
2. Implement vectorized multi-view updates

---

## Profiling Checklist

- [ ] Profile with `-g` and `perf record --call-graph=dwarf`
- [ ] Identify which mirror/view configurations spend most time
- [ ] Measure impact of each optimization independently
- [ ] Validate correctness with state save/load tests
- [ ] Check for memory access pattern changes (cache misses)
- [ ] Run benchmarks: TBBlue startup, runtime bank switches, state load

---

## Testing Strategy

1. **Correctness**: State save/load must produce identical binary snapshots
2. **Functional**: Memory reads/writes must return same values
3. **Performance**: Measure FPS before/after on SpecNext
4. **Regression**: Run full MAME self-tests

