using Xunit;

// Allocation and deterministic replay guardrails should not run concurrently
// with other tests that exercise shared runtime pools or static caches.
[assembly: CollectionBehavior(DisableTestParallelization = true)]
