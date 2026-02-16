# Finding 029: OutputSpan::publishTag() off-by-one — writes past tag buffer end

## Invariant
Buffer bounds checks must prevent out-of-bounds writes.

## Code Locus
`Port.hpp:699` — `OutputSpan::publishTag()`.

## Possible Violation
```cpp
if (tagsPublished > tags.size()) {    // ← off-by-one: should be >=
    return;
}
// ...
tags[tagsPublished++] = {index, std::forward<TPropertyMap>(tagData)};   // line 717
```

When `tagsPublished == tags.size()`, the guard (`>`) evaluates to false and the write at line 717 accesses `tags[tags.size()]` — one past the end of the span. The `tags` span is a `WriterSpan` reserved from the tag circular buffer (line 672):
```cpp
tags(tagsWriter.template reserve<SpanReleasePolicy::ProcessNone>(tagsWriter.available()))
```

The out-of-bounds write corrupts adjacent memory in the circular buffer.

### Trigger path
1. Block has an output port with a small tag buffer (e.g., 4 slots available)
2. Block publishes 4 tags in one work cycle (e.g., via `publishTag()` in `processBulk`)
3. `tagsPublished` reaches 4 == `tags.size()` → guard `4 > 4` is false
4. Fifth `publishTag()` call writes to `tags[4]` — out of bounds

## Fix
Change `>` to `>=`:
```cpp
if (tagsPublished >= tags.size()) {
```

## CI Impact
All tests pass: qa_Scheduler, qa_Graph, qa_Block, qa_LifeCycle.
