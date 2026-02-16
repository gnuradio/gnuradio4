# Finding 031: invokeProcessOneNonConst reports full batch as processedIn on early exit — silent sample loss

## Invariant
For processOne blocks, the number of input samples consumed must equal the number actually processed (1:1 ratio).

## Code Locus
`Block.hpp:1926` — `invokeProcessOneNonConst()` return statement.

## Possible Violation
```cpp
for (std::size_t i = 0UZ; i < nSamplesToProcess; ++i) {
    // ... process inputSpans[i] → outputSpans[i] ...
    nOutSamplesBeforeRequestedStop++;
    if (_outputTagsChanged || lifecycle::isShuttingDown(this->state())) [[unlikely]] {
        break;   // loop exits early — e.g., at iteration 5 of 10
    }
}
_outputTagsChanged = false;
return ProcessOneResult{..., nSamplesToProcess,                                    // ← full batch (10)
                             std::min(nSamplesToProcess, nOutSamplesBeforeRequestedStop)  // ← actual (5)
                       };
```

When `_outputTagsChanged` breaks the loop early (e.g., after 5 of 10 iterations):
- `processedIn = nSamplesToProcess` = 10 (the full batch, not adjusted)
- `processedOut = nOutSamplesBeforeRequestedStop` = 5 (actual iterations)

Back in `workInternal()` (line 2159–2204):
1. `publishSamples(5, outputSpans)` — publishes 5 output samples (correct)
2. `consumeReaders(processedIn=10, inputSpans)` — consumes 10 input samples (WRONG)

Only 5 input samples were read (the loop accessed `inputs[0..4]`), but 10 are consumed. Samples 5–9 are silently dropped.

The `isShuttingDown()` path is safe because lines 2183–2188 zero `processedIn`. But the `_outputTagsChanged` path leaves `userReturnStatus = OK` and `processedIn = 10`, causing the overconsumption.

### Trigger path
1. Block implements non-const `processOne()` (has side effects, uses sample-by-sample mode)
2. Block sets `_outputTagsChanged = true` inside `processOne()` at iteration 5 to emit a mid-stream tag
3. Loop breaks at iteration 5 with `nOutSamplesBeforeRequestedStop = 5`
4. Return: `processedIn = 10, processedOut = 5`
5. `workInternal()` publishes 5 samples but consumes 10 → 5 input samples lost

## Fix
Use `nOutSamplesBeforeRequestedStop` for both `processedIn` and `processedOut`. For processOne blocks, input and output counts are always 1:1 per iteration:
```cpp
return ProcessOneResult{lifecycle::isShuttingDown(this->state()) ? DONE : OK,
                        nOutSamplesBeforeRequestedStop,
                        nOutSamplesBeforeRequestedStop};
```

## CI Impact
All tests pass: qa_Scheduler, qa_Graph, qa_Block, qa_LifeCycle.
