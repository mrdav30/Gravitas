# Benchmark Publishing And CCD Diagnostics Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Turn CCD benchmark evidence and solver status into repeatable release signals through benchmark artifact publishing, baseline comparison, and deterministic host-visible CCD diagnostics where justified.

**Architecture:** Keep benchmark evidence separate from runtime diagnostics. Benchmark publishing owns historical performance comparison; runtime diagnostics expose deterministic counters only when they help users tune or debug CCD behavior.

**Tech Stack:** .NET 8, BenchmarkDotNet, xUnit v3, GitHub Actions or external benchmark tooling, Gravitas diagnostics.

---

**Date:** 2026-06-21
**Status:** Post-alpha / evidence-gated
**Owner:** Gravitas benchmark/diagnostics hardening

## Purpose

The CCD depth plan produced useful manual benchmark rows, but publishing,
external baseline storage/comparison, CI gating, and host-visible CCD counters
were intentionally deferred while the multi-repository workflow was still being
evaluated. Those are release-readiness concerns, not runtime CCD algorithm work,
so they belong in a separate plan.

This plan is distinct from
`2026-06-21-benchmark-signal-hardening-backlog-plan.md`: that document tracks
measured runtime signals to investigate. This one tracks the evidence pipeline
that makes future performance and CCD diagnostics trustworthy.

## Guiding Rules

- Do not gate on raw wall-clock thresholds tied to one machine.
- Prefer baseline comparison artifacts, statistical comparison, or an external
  tool such as Bencher over brittle CI timing assertions.
- Keep runtime diagnostics deterministic and disabled-path allocation-free.
- Benchmark-only counters do not automatically become public runtime APIs.
- Cross-repo workflow choices must respect the LSF release order and package
  publishing model.

## Workstream 1: Benchmark Artifact Model

**Tasks**

- [ ] Decide which benchmark selections are release evidence versus local smoke.
- [ ] Define artifact naming, retention, and comparison metadata.
- [ ] Document commands for CCD evidence, substep evidence, and runtime
  allocation evidence.
- [ ] Ensure benchmark output can identify package version, commit SHA,
  configuration, and runtime mode.

## Workstream 2: Baseline Comparison Tooling

**Tasks**

- [ ] Evaluate BenchmarkDotNet comparison output, stored JSON artifacts, and
  external services such as Bencher.
- [ ] Pick a comparison approach that works across FixedMathSharp,
  SwiftCollections, GridForge, Chronicler, and Gravitas.
- [ ] Add a local comparison workflow before any CI gate.
- [ ] Document how to interpret noisy rows and when to rerun.

## Workstream 3: CI Or Release Workflow Integration

**Tasks**

- [ ] Start with benchmark project build validation in CI.
- [ ] Add manual benchmark artifact publishing before automatic failure gates.
- [ ] Gate only stable selections with enough signal-to-noise.
- [ ] Keep expensive benchmark suites out of ordinary PR validation unless the
  repository owner explicitly opts in.

## Workstream 4: Host-Visible CCD Diagnostics

**Problem**

Current bodies expose last-step CCD substep count and cap status, but richer
service-level CCD work may need host-visible counters for tuning and debugging.

**Tasks**

- [ ] Identify counters users actually need: candidate count, accepted hits,
  substep cap hits, island count, exact reducer attempts, false-positive
  rejections, and mixed CCD handoffs.
- [ ] Decide which counters are benchmark-only and which belong in
  `GravitasDiagnosticSink`.
- [ ] Add diagnostics tests proving disabled paths allocate `0` bytes and
  enabled paths preserve deterministic event order.
- [ ] Update `docs/wiki/DIAGNOSTICS.md` and `docs/wiki/COLLISION_PIPELINE.md`.

## Done Criteria

- CCD benchmark evidence can be published and compared repeatably without
  relying on one developer's local console output.
- Any CI/performance gate uses a defensible comparison model.
- Host-visible CCD diagnostics are deterministic, documented, and
  allocation-conscious.
- The measured runtime signal backlog remains focused on runtime RCA rather
  than benchmark platform setup.
