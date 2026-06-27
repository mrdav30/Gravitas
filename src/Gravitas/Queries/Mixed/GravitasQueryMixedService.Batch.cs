//=======================================================================
// GravitasQueryMixedService.Batch.cs
//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp;
using SwiftCollections;
using System;

namespace Gravitas.Queries;

/// <summary>
/// Owns batched mixed 3D/2D query behavior for one <see cref="GravitasWorldContext"/>.
/// </summary>
public sealed partial class GravitasQueryMixedService
{
    private readonly SwiftList<PhysicsMixedHit> _batchMixedHits = new();

    /// <summary>
    /// Gets the number of requests processed by the last mixed batch query call.
    /// </summary>
    public int LastBatchRequestCount { get; private set; }

    /// <summary>
    /// Gets the number of hits accepted by the last mixed batch query call.
    /// </summary>
    public int LastBatchHitCount { get; private set; }

    /// <summary>
    /// Gets the summed top-level candidate count reported by the last mixed batch query call.
    /// </summary>
    public int LastBatchCandidateCount { get; private set; }

    /// <summary>
    /// Gets the summed mesh-triangle candidate count reported by the last mixed batch query call.
    /// </summary>
    public int LastBatchMeshTriangleCandidateCount { get; private set; }

    /// <summary>
    /// Executes multiple mixed swept-sphere-against-2D queries and writes one closest-hit slot per request.
    /// </summary>
    public int SweepSphereAgainst2DBatch(
        ReadOnlySpan<PhysicsSweepSphereAgainst2DRequest> requests,
        Span<PhysicsMixedHit> closestHits)
    {
        ValidateClosestBatchOutput(requests.Length, closestHits);
        ValidateSweepSphereAgainst2DRequests(requests);
        ResetBatchCounters(requests.Length);

        int hitCount = 0;
        for (int i = 0; i < requests.Length; i++)
        {
            PhysicsSweepSphereAgainst2DRequest request = requests[i];
            bool found = SweepSphereAgainst2D(
                request.Start,
                request.End,
                request.Radius,
                request.LayerMask,
                out PhysicsMixedHit hit,
                request.ExcludedCollider,
                request.IncludeTriggers);
            WriteClosestHit(closestHits, i, hit, found, ref hitCount);
        }

        LastBatchHitCount = hitCount;
        return hitCount;
    }

    /// <summary>
    /// Executes multiple mixed swept-sphere-against-2D queries and appends all hits into one caller-owned hit buffer.
    /// </summary>
    public int SweepSphereAgainst2DAllBatch(
        ReadOnlySpan<PhysicsSweepSphereAgainst2DRequest> requests,
        SwiftList<PhysicsMixedHit> hits,
        Span<PhysicsQueryHitRange> ranges)
    {
        SwiftThrowHelper.ThrowIfNull(hits, nameof(hits));
        ValidateRangeBatchOutput(requests.Length, ranges);
        ValidateSweepSphereAgainst2DRequests(requests);
        ResetBatchCounters(requests.Length);
        hits.FastClear();

        for (int i = 0; i < requests.Length; i++)
        {
            PhysicsSweepSphereAgainst2DRequest request = requests[i];
            int count = SweepSphereAgainst2DAll(
                request.Start,
                request.End,
                request.Radius,
                request.LayerMask,
                _batchMixedHits,
                request.ExcludedCollider,
                request.IncludeTriggers);
            AppendRange(hits, ranges, i, count);
        }

        LastBatchHitCount = hits.Count;
        return hits.Count;
    }

    /// <summary>
    /// Executes multiple mixed swept-circle-against-3D queries and writes one closest-hit slot per request.
    /// </summary>
    public int SweepCircleAgainst3DBatch(
        ReadOnlySpan<PhysicsSweepCircleAgainst3DRequest> requests,
        Span<PhysicsMixedHit> closestHits)
    {
        ValidateClosestBatchOutput(requests.Length, closestHits);
        ValidateSweepCircleAgainst3DRequests(requests);
        ResetBatchCounters(requests.Length);

        int hitCount = 0;
        for (int i = 0; i < requests.Length; i++)
        {
            PhysicsSweepCircleAgainst3DRequest request = requests[i];
            bool found = SweepCircleAgainst3D(
                request.Start,
                request.End,
                request.Radius,
                request.SlabCenterY,
                request.HalfThickness,
                request.LayerMask,
                out PhysicsMixedHit hit,
                request.ExcludedCollider,
                request.IncludeTriggers);
            WriteClosestHit(closestHits, i, hit, found, ref hitCount);
        }

        LastBatchHitCount = hitCount;
        return hitCount;
    }

    /// <summary>
    /// Executes multiple mixed swept-circle-against-3D queries and appends all hits into one caller-owned hit buffer.
    /// </summary>
    public int SweepCircleAgainst3DAllBatch(
        ReadOnlySpan<PhysicsSweepCircleAgainst3DRequest> requests,
        SwiftList<PhysicsMixedHit> hits,
        Span<PhysicsQueryHitRange> ranges)
    {
        SwiftThrowHelper.ThrowIfNull(hits, nameof(hits));
        ValidateRangeBatchOutput(requests.Length, ranges);
        ValidateSweepCircleAgainst3DRequests(requests);
        ResetBatchCounters(requests.Length);
        hits.FastClear();

        for (int i = 0; i < requests.Length; i++)
        {
            PhysicsSweepCircleAgainst3DRequest request = requests[i];
            int count = SweepCircleAgainst3DAll(
                request.Start,
                request.End,
                request.Radius,
                request.SlabCenterY,
                request.HalfThickness,
                request.LayerMask,
                _batchMixedHits,
                request.ExcludedCollider,
                request.IncludeTriggers);
            AppendRange(hits, ranges, i, count);
        }

        LastBatchHitCount = hits.Count;
        return hits.Count;
    }

    private void WriteClosestHit(Span<PhysicsMixedHit> closestHits, int index, PhysicsMixedHit hit, bool found, ref int hitCount)
    {
        if (found)
        {
            closestHits[index] = hit;
            hitCount++;
            AccumulateBatchCounters(1);
        }
        else
        {
            closestHits[index] = default;
            AccumulateBatchCounters(0);
        }
    }

    private void AppendRange(
        SwiftList<PhysicsMixedHit> hits,
        Span<PhysicsQueryHitRange> ranges,
        int index,
        int count)
    {
        int start = hits.Count;
        if (count > 0)
            hits.AddRange(_batchMixedHits.AsReadOnlySpan());

        ranges[index] = new PhysicsQueryHitRange(start, count);
        AccumulateBatchCounters(count);
    }

    private void ResetBatchCounters(int requestCount)
    {
        LastBatchRequestCount = requestCount;
        LastBatchHitCount = 0;
        LastBatchCandidateCount = 0;
        LastBatchMeshTriangleCandidateCount = 0;
    }

    private void AccumulateBatchCounters(int hitCount)
    {
        LastBatchHitCount += hitCount;
        LastBatchCandidateCount += LastQueryCandidateCount;
        LastBatchMeshTriangleCandidateCount += LastMeshTriangleCandidateCount;
    }

    private static void ValidateClosestBatchOutput(int requestCount, Span<PhysicsMixedHit> closestHits)
    {
        SwiftThrowHelper.ThrowIfArgument(
            closestHits.Length < requestCount,
            nameof(closestHits),
            "Closest-hit output span must contain at least one slot per request.");
    }

    private static void ValidateRangeBatchOutput(int requestCount, Span<PhysicsQueryHitRange> ranges)
    {
        SwiftThrowHelper.ThrowIfArgument(
            ranges.Length < requestCount,
            nameof(ranges),
            "Range output span must contain at least one slot per request.");
    }

    private static void ValidateSweepSphereAgainst2DRequests(ReadOnlySpan<PhysicsSweepSphereAgainst2DRequest> requests)
    {
        for (int i = 0; i < requests.Length; i++)
        {
            SwiftThrowHelper.ThrowIfArgument(
                requests[i].Radius <= Fixed64.Zero,
                nameof(requests),
                "Mixed swept sphere radius must be greater than zero.");
        }
    }

    private static void ValidateSweepCircleAgainst3DRequests(ReadOnlySpan<PhysicsSweepCircleAgainst3DRequest> requests)
    {
        for (int i = 0; i < requests.Length; i++)
        {
            SwiftThrowHelper.ThrowIfArgument(
                requests[i].Radius <= Fixed64.Zero,
                nameof(requests),
                "Mixed swept circle radius must be greater than zero.");
            SwiftThrowHelper.ThrowIfArgument(
                requests[i].HalfThickness <= Fixed64.Zero,
                nameof(requests),
                "Mixed swept circle half-thickness must be greater than zero.");
        }
    }
}
