# Performance Investigation - Action Items & Quick Start

## Quick Summary

**Problem**: TBBlue emulation spending 72% of CPU in memory dispatcher  
**Root Cause**: 8+ memory views with complex banked mappings exercising range-cut and mirror-population machinery  
**Solution**: Two targeted optimizations implemented  
**Expected Speedup**: 15-25% reduction in dispatcher overhead = 3-5% overall improvement  

---

## What Was Done

### ? Optimization #1: Range Cut Cache
- **Files**: emumem_hedr.{h,ipp}, emumem_hedw.{h,ipp}
- **What**: Added cache to skip redundant range-cut iterations
- **Lines Added**: ~40 (including comments)
- **Memory Overhead**: 32 bytes per dispatcher
- **Expected Impact**: 8-12% dispatcher speedup

### ? Optimization #2: Mirror Fast-Paths  
- **Files**: emumem_hedr.ipp, emumem_hedw.ipp
- **What**: Added fast-path detection for non-mirrored cases
- **Lines Added**: ~12 per function
- **Memory Overhead**: 0 (pure algorithmic)
- **Expected Impact**: 10-15% dispatcher speedup

---

## Next Steps (In Order)

### 1. Compile & Test (30 minutes)
```bash
cd /home/andrei/workspace/mame
make REGENIE=1 DEBUG=1 OPTIMIZE=0 -j6 TOOLS=0 OSD=sdl3 \
    SOURCES=sinclair/spectrum.cpp,sinclair/spec128.cpp,sinclair/specpls3.cpp,\
    sinclair/next/specnext.cpp,galaxian/galaxian.cpp,williams/williams.cpp 2>&1 | tee build.log
```

**Check for**:
- No compilation errors
- No new warnings
- Successful build completion

### 2. Run Validation (10 minutes)
```bash
mame -validate
```

**Expected**: No new errors or warnings

### 3. Test SpecNext Functionality (5 minutes)
```bash
mame tbblue -autoboot -seconds 10
# Watch for:
# - No crashes
# - Normal boot sequence
# - No visual artifacts
```

### 4. Profile & Measure (20 minutes)
```bash
# Baseline (if not already done)
perf record -g -F 99 -- mame tbblue -autoboot -seconds 30
perf report > baseline_profile.txt

# Key metrics to check:
# - % time in range_cut_after/range_cut_before
# - % time in populate_mirror
# - Overall FPS/CPU usage
```

### 5. Commit & Document
```bash
git add src/emu/emumem_hedr.* src/emu/emumem_hedw.*
git commit -m "perf: Memory dispatcher range-cut cache and mirror fast-paths

- Add m_cut_cache to track last range-cut operations
- Skip redundant range-cut iterations for cached cases  
- Add fast-path for non-mirrored memory regions

Performance impact (estimated):
- 8-12% reduction in range_cut overhead
- 10-15% reduction in mirror population overhead
- ~18-27% total reduction in dispatcher overhead
- ~3-5% overall emulation speedup on SpecNext

This addresses the issue where TBBlue profiling showed 72% of
cycles in memory dispatcher machinery. The optimizations target
the two hottest functions that are repeatedly exercised during
complex memory view configuration."
```

---

## Files Changed Summary

| File | Changes | Lines | Impact |
|------|---------|-------|--------|
| emumem_hedr.h | Add m_cut_cache struct | +8 | 0 (zero-cost abstraction) |
| emumem_hedr.ipp | Optimize range_cut_* | +60 | Algorithm optimization |
| emumem_hedr.ipp | Add fast-paths to populate_mirror | +12 | Early return optimization |
| emumem_hedw.h | Add m_cut_cache struct | +8 | 0 (zero-cost abstraction) |
| emumem_hedw.ipp | Optimize range_cut_* | +60 | Algorithm optimization |
| emumem_hedw.ipp | Add fast-paths to populate_mirror | +12 | Early return optimization |

---

## Documentation Generated

For reference and future maintenance:

1. **README_PERFORMANCE_INVESTIGATION.md** (This Folder)
   - High-level overview of investigation
   - Performance analysis summary
   - Technical explanation of optimizations

2. **MEMORY_DISPATCHER_ANALYSIS.md** (Detailed Technical)
   - Deep dive into each hotspot
   - Call stack analysis
   - 6 potential optimizations with pros/cons
   - Prioritization matrix

3. **MEMORY_DISPATCHER_IMPLEMENTATION.md** (Implementation Guide)
   - Concrete code changes for each optimization
   - Before/after code examples
   - Testing strategy
   - Profiling methodology

4. **MEMORY_DISPATCHER_OPTIMIZATION_SUMMARY.md** (Status Tracking)
   - Phase 1 implementation details
   - Phase 2 proposals
   - Performance measurement expectations
   - Validation checklist

---

## Expected Results

### Before Optimizations
```
TBBlue emulation performance:
- ~70% CPU in memory dispatcher machinery
- X FPS on standard hardware
- 72% of profiled samples in range_cut/populate_mirror
```

### After Phase 1 Optimizations
```
TBBlue emulation performance:
- ~45-50% CPU in memory dispatcher machinery
- X * 1.03-1.05 FPS (estimated 3-5% improvement)
- 50-60% of profiled samples in range_cut/populate_mirror
```

### After Full Optimization (Phase 1 + Phase 2)
```
TBBlue emulation performance:
- ~30-35% CPU in memory dispatcher machinery
- X * 1.25-1.40 FPS (estimated 25-40% improvement)
- Depends on other bottlenecks being addressed
```

---

## Troubleshooting

### Build Fails
**Problem**: Compilation errors in emumem_hedr*.ipp  
**Solution**: Ensure you're using the exact modified files from this repository  
**Check**: `git diff src/emu/emumem_hedr.ipp` should show only the expected changes

### SpecNext Doesn't Boot
**Problem**: TBBlue crashes or doesn't boot after optimization  
**Solution**: Verify range_cut_cache initialization is correct  
**Debug**: Add VERBOSE logging to see range_cut calls

### No Performance Improvement
**Problem**: Profiling shows no improvement  
**Possible Causes**:
1. Optimizations not actually being used (check compiler flags)
2. Other bottlenecks dominate (profile to verify)
3. Cache ineffective due to usage pattern (analyze call patterns)

**Debug Steps**:
```cpp
// Add instrumentation
static int cache_hits = 0, cache_misses = 0;

// In range_cut_after fast-path:
if(cache_hit) cache_hits++;
else cache_misses++;

// Print stats periodically
LOGMASKED(LOG_MEM, "Cache: %d hits, %d misses, ratio: %.1f%%",
    cache_hits, cache_misses, 100.0*cache_hits/(cache_hits+cache_misses));
```

---

## Follow-up Optimizations (Future)

### Optimization #3: Lazy Range Cuts (Phase 2)
- Defer range cuts until next memory access
- Amortize cost across multiple operations
- Est. 15-20% additional speedup
- Status: Proposed, not yet implemented

### Optimization #4: Subdispatch Pooling (Phase 2)
- Reuse subdispatch objects instead of allocating
- Better memory locality
- Est. 5-10% additional speedup
- Status: Proposed, not yet implemented

---

## Performance Testing Workflow

```bash
# 1. Create baseline
perf record -g -o baseline.perf -- mame tbblue -autoboot -seconds 30
perf report -i baseline.perf > before_profile.txt

# 2. Apply optimizations
git checkout optimized-branch

# 3. Rebuild
make clean
make REGENIE=1 OPTIMIZE=0 -j6 SOURCES=...

# 4. Test with optimizations
perf record -g -o optimized.perf -- mame tbblue -autoboot -seconds 30
perf report -i optimized.perf > after_profile.txt

# 5. Compare results
diff before_profile.txt after_profile.txt
perf diff -i baseline.perf optimized.perf

# 6. Analyze key metrics
grep -E "range_cut|populate_mirror" before_profile.txt after_profile.txt
```

---

## Questions for Analysis

After implementing and testing, investigate:

1. **Are range_cut_before and range_cut_after actually called repeatedly?**
   - Add counters to track call frequency
   - Log entry/address pairs to detect patterns

2. **What percentage of mirror configurations use fast-paths?**
   - Instrument populate_mirror to count fast-path vs general case hits
   - Should be 60%+ for SpecNext

3. **Are there other memory-related bottlenecks?**
   - The 2.9% in inflate_fast suggests state loading overhead
   - Worth investigating if dispatcher optimization isn't sufficient

4. **Cache effectiveness on different memory patterns?**
   - Profile with different ROM/RAM configurations
   - Test with different SpecNext snapshot files

---

## Maintenance Notes

### For Future Developers

The two optimizations are minimal and defensive:

1. **Range Cut Cache**: Only skips iterations, fallback to full scan always works
2. **Mirror Fast-Paths**: Only changes algorithm for zero-mirror and single-entry cases

Both can be:
- Disabled by removing the fast-path conditions
- Verified by comparing results with cache disabled
- Extended with additional patterns (e.g., single-mirror case)

### Code Comments

The changes include inline comments explaining the optimization rationale. The m_cut_cache structure is self-documenting.

---

## Summary Checklist

- [x] Analyzed performance issue
- [x] Identified root causes
- [x] Designed solutions
- [x] Implemented Phase 1 optimizations
- [x] Created documentation
- [ ] Build and test
- [ ] Profile and measure
- [ ] Validate correctness
- [ ] Commit changes
- [ ] Plan Phase 2

---

## Contact Points

For questions about these optimizations:

1. **Analysis Documents**: See MEMORY_DISPATCHER_ANALYSIS.md for detailed breakdown
2. **Implementation Details**: See MEMORY_DISPATCHER_IMPLEMENTATION.md for code changes
3. **Status Tracking**: See MEMORY_DISPATCHER_OPTIMIZATION_SUMMARY.md for progress

