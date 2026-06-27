//=======================================================================
// GravitasQuery2DService.Batch.cs
//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp;
using SwiftCollections;
using System;

namespace Gravitas.Queries;

/// <summary>
/// Owns batched pure 2D query behavior for one <see cref="GravitasWorldContext"/>.
/// </summary>
public sealed partial class GravitasQuery2DService
{
    private readonly SwiftList<Physics2DHit> _batch2DHits = new();

    /// <summary>
    /// Gets the number of requests processed by the last pure 2D batch query call.
    /// </summary>
    public int LastBatchRequestCount { get; private set; }

    /// <summary>
    /// Gets the number of hits accepted by the last pure 2D batch query call.
    /// </summary>
    public int LastBatchHitCount { get; private set; }

    /// <summary>
    /// Gets the summed candidate count reported by the last pure 2D batch query call.
    /// </summary>
    public int LastBatchCandidateCount { get; private set; }

    /// <summary>
    /// Executes multiple pure 2D segment raycasts and writes one closest-hit slot per request.
    /// </summary>
    public int RaycastBatch(ReadOnlySpan<PhysicsRaycast2DRequest> requests, Span<Physics2DHit> closestHits)
    {
        ValidateClosestBatchOutput(requests.Length, closestHits);
        ResetBatchCounters(requests.Length);

        int hitCount = 0;
        for (int i = 0; i < requests.Length; i++)
        {
            PhysicsRaycast2DRequest request = requests[i];
            bool found = Raycast(request.Start, request.End, request.LayerMask, out Physics2DHit hit);
            WriteClosestHit(closestHits, i, hit, found, ref hitCount);
        }

        LastBatchHitCount = hitCount;
        return hitCount;
    }

    /// <summary>
    /// Executes multiple pure 2D segment raycasts and appends all hits into one caller-owned hit buffer.
    /// </summary>
    public int RaycastAllBatch(
        ReadOnlySpan<PhysicsRaycast2DRequest> requests,
        SwiftList<Physics2DHit> hits,
        Span<PhysicsQueryHitRange> ranges)
    {
        SwiftThrowHelper.ThrowIfNull(hits, nameof(hits));
        ValidateRangeBatchOutput(requests.Length, ranges);
        ResetBatchCounters(requests.Length);
        hits.FastClear();

        for (int i = 0; i < requests.Length; i++)
        {
            PhysicsRaycast2DRequest request = requests[i];
            int count = RaycastAll(request.Start, request.End, request.LayerMask, _batch2DHits);
            AppendRange(hits, ranges, i, count);
        }

        LastBatchHitCount = hits.Count;
        return hits.Count;
    }

    /// <summary>
    /// Executes multiple pure 2D circle overlaps and writes one closest-hit slot per request.
    /// </summary>
    public int OverlapCircleBatch(ReadOnlySpan<PhysicsOverlapCircle2DRequest> requests, Span<Physics2DHit> closestHits)
    {
        ValidateClosestBatchOutput(requests.Length, closestHits);
        ValidateOverlapCircleRequests(requests);
        ResetBatchCounters(requests.Length);

        int hitCount = 0;
        for (int i = 0; i < requests.Length; i++)
        {
            PhysicsOverlapCircle2DRequest request = requests[i];
            bool found = OverlapCircle(request.Center, request.Radius, request.LayerMask, out Physics2DHit hit);
            WriteClosestHit(closestHits, i, hit, found, ref hitCount);
        }

        LastBatchHitCount = hitCount;
        return hitCount;
    }

    /// <summary>
    /// Executes multiple pure 2D circle overlaps and appends all hits into one caller-owned hit buffer.
    /// </summary>
    public int OverlapCircleAllBatch(
        ReadOnlySpan<PhysicsOverlapCircle2DRequest> requests,
        SwiftList<Physics2DHit> hits,
        Span<PhysicsQueryHitRange> ranges)
    {
        SwiftThrowHelper.ThrowIfNull(hits, nameof(hits));
        ValidateRangeBatchOutput(requests.Length, ranges);
        ValidateOverlapCircleRequests(requests);
        ResetBatchCounters(requests.Length);
        hits.FastClear();

        for (int i = 0; i < requests.Length; i++)
        {
            PhysicsOverlapCircle2DRequest request = requests[i];
            int count = OverlapCircleAll(request.Center, request.Radius, request.LayerMask, _batch2DHits);
            AppendRange(hits, ranges, i, count);
        }

        LastBatchHitCount = hits.Count;
        return hits.Count;
    }

    /// <summary>
    /// Executes multiple pure 2D AABB overlaps and writes one closest-hit slot per request.
    /// </summary>
    public int OverlapAabbBatch(ReadOnlySpan<PhysicsOverlapAabb2DRequest> requests, Span<Physics2DHit> closestHits)
    {
        ValidateClosestBatchOutput(requests.Length, closestHits);
        ValidateOverlapAabbRequests(requests);
        ResetBatchCounters(requests.Length);

        int hitCount = 0;
        for (int i = 0; i < requests.Length; i++)
        {
            PhysicsOverlapAabb2DRequest request = requests[i];
            bool found = OverlapAabb(request.Center, request.Size, request.LayerMask, out Physics2DHit hit);
            WriteClosestHit(closestHits, i, hit, found, ref hitCount);
        }

        LastBatchHitCount = hitCount;
        return hitCount;
    }

    /// <summary>
    /// Executes multiple pure 2D AABB overlaps and appends all hits into one caller-owned hit buffer.
    /// </summary>
    public int OverlapAabbAllBatch(
        ReadOnlySpan<PhysicsOverlapAabb2DRequest> requests,
        SwiftList<Physics2DHit> hits,
        Span<PhysicsQueryHitRange> ranges)
    {
        SwiftThrowHelper.ThrowIfNull(hits, nameof(hits));
        ValidateRangeBatchOutput(requests.Length, ranges);
        ValidateOverlapAabbRequests(requests);
        ResetBatchCounters(requests.Length);
        hits.FastClear();

        for (int i = 0; i < requests.Length; i++)
        {
            PhysicsOverlapAabb2DRequest request = requests[i];
            int count = OverlapAabbAll(request.Center, request.Size, request.LayerMask, _batch2DHits);
            AppendRange(hits, ranges, i, count);
        }

        LastBatchHitCount = hits.Count;
        return hits.Count;
    }

    /// <summary>
    /// Executes multiple pure 2D convex polygon overlaps and writes one closest-hit slot per request.
    /// </summary>
    public int OverlapPolygonBatch(
        ReadOnlySpan<PhysicsOverlapPolygon2DRequest> requests,
        ReadOnlySpan<Vector2d> vertices,
        Span<Physics2DHit> closestHits)
    {
        ValidateClosestBatchOutput(requests.Length, closestHits);
        ValidatePolygonRequestRanges(requests, vertices);
        ResetBatchCounters(requests.Length);

        int hitCount = 0;
        for (int i = 0; i < requests.Length; i++)
        {
            PhysicsOverlapPolygon2DRequest request = requests[i];
            ReadOnlySpan<Vector2d> polygon = vertices.Slice(request.VertexStart, request.VertexCount);
            bool found = OverlapPolygon(polygon, request.LayerMask, out Physics2DHit hit);
            WriteClosestHit(closestHits, i, hit, found, ref hitCount);
        }

        LastBatchHitCount = hitCount;
        return hitCount;
    }

    /// <summary>
    /// Executes multiple pure 2D convex polygon overlaps and appends all hits into one caller-owned hit buffer.
    /// </summary>
    public int OverlapPolygonAllBatch(
        ReadOnlySpan<PhysicsOverlapPolygon2DRequest> requests,
        ReadOnlySpan<Vector2d> vertices,
        SwiftList<Physics2DHit> hits,
        Span<PhysicsQueryHitRange> ranges)
    {
        SwiftThrowHelper.ThrowIfNull(hits, nameof(hits));
        ValidateRangeBatchOutput(requests.Length, ranges);
        ValidatePolygonRequestRanges(requests, vertices);
        ResetBatchCounters(requests.Length);
        hits.FastClear();

        for (int i = 0; i < requests.Length; i++)
        {
            PhysicsOverlapPolygon2DRequest request = requests[i];
            ReadOnlySpan<Vector2d> polygon = vertices.Slice(request.VertexStart, request.VertexCount);
            int count = OverlapPolygonAll(polygon, request.LayerMask, _batch2DHits);
            AppendRange(hits, ranges, i, count);
        }

        LastBatchHitCount = hits.Count;
        return hits.Count;
    }

    /// <summary>
    /// Executes multiple pure 2D swept-circle queries and writes one closest-hit slot per request.
    /// </summary>
    public int SweepCircleBatch(ReadOnlySpan<PhysicsSweepCircle2DRequest> requests, Span<Physics2DHit> closestHits)
    {
        ValidateClosestBatchOutput(requests.Length, closestHits);
        ValidateSweepCircleRequests(requests);
        ResetBatchCounters(requests.Length);

        int hitCount = 0;
        for (int i = 0; i < requests.Length; i++)
        {
            PhysicsSweepCircle2DRequest request = requests[i];
            bool found = SweepCircle(
                request.Start,
                request.End,
                request.Radius,
                request.LayerMask,
                out Physics2DHit hit,
                request.ExcludedCollider,
                request.IncludeTriggers);
            WriteClosestHit(closestHits, i, hit, found, ref hitCount);
        }

        LastBatchHitCount = hitCount;
        return hitCount;
    }

    /// <summary>
    /// Executes multiple pure 2D swept-circle queries and appends all hits into one caller-owned hit buffer.
    /// </summary>
    public int SweepCircleAllBatch(
        ReadOnlySpan<PhysicsSweepCircle2DRequest> requests,
        SwiftList<Physics2DHit> hits,
        Span<PhysicsQueryHitRange> ranges)
    {
        SwiftThrowHelper.ThrowIfNull(hits, nameof(hits));
        ValidateRangeBatchOutput(requests.Length, ranges);
        ValidateSweepCircleRequests(requests);
        ResetBatchCounters(requests.Length);
        hits.FastClear();

        for (int i = 0; i < requests.Length; i++)
        {
            PhysicsSweepCircle2DRequest request = requests[i];
            int count = SweepCircleAll(
                request.Start,
                request.End,
                request.Radius,
                request.LayerMask,
                _batch2DHits,
                request.ExcludedCollider,
                request.IncludeTriggers);
            AppendRange(hits, ranges, i, count);
        }

        LastBatchHitCount = hits.Count;
        return hits.Count;
    }

    private void WriteClosestHit(Span<Physics2DHit> closestHits, int index, Physics2DHit hit, bool found, ref int hitCount)
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
        SwiftList<Physics2DHit> hits,
        Span<PhysicsQueryHitRange> ranges,
        int index,
        int count)
    {
        int start = hits.Count;
        if (count > 0)
            hits.AddRange(_batch2DHits.AsReadOnlySpan());

        ranges[index] = new PhysicsQueryHitRange(start, count);
        AccumulateBatchCounters(count);
    }

    private void ResetBatchCounters(int requestCount)
    {
        LastBatchRequestCount = requestCount;
        LastBatchHitCount = 0;
        LastBatchCandidateCount = 0;
    }

    private void AccumulateBatchCounters(int hitCount)
    {
        LastBatchHitCount += hitCount;
        LastBatchCandidateCount += LastQueryCandidateCount;
    }

    private static void ValidateClosestBatchOutput(int requestCount, Span<Physics2DHit> closestHits)
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

    private static void ValidatePolygonRequestRanges(
        ReadOnlySpan<PhysicsOverlapPolygon2DRequest> requests,
        ReadOnlySpan<Vector2d> vertices)
    {
        for (int i = 0; i < requests.Length; i++)
        {
            PhysicsOverlapPolygon2DRequest request = requests[i];
            SwiftThrowHelper.ThrowIfArgument(
                request.VertexStart < 0
                || request.VertexCount < 0
                || request.VertexStart > vertices.Length
                || request.VertexCount > vertices.Length - request.VertexStart,
                nameof(requests),
                "Polygon batch request vertex range must be contained by the supplied vertex span.");

            QueryDetection2D.ValidateConvexQueryPolygon(vertices.Slice(request.VertexStart, request.VertexCount));
        }
    }

    private static void ValidateOverlapCircleRequests(ReadOnlySpan<PhysicsOverlapCircle2DRequest> requests)
    {
        for (int i = 0; i < requests.Length; i++)
        {
            SwiftThrowHelper.ThrowIfArgument(
                requests[i].Radius < Fixed64.Zero,
                nameof(requests),
                "2D query radius cannot be negative.");
        }
    }

    private static void ValidateOverlapAabbRequests(ReadOnlySpan<PhysicsOverlapAabb2DRequest> requests)
    {
        for (int i = 0; i < requests.Length; i++)
            QueryDetection2D.ValidateAabbSize(requests[i].Size);
    }

    private static void ValidateSweepCircleRequests(ReadOnlySpan<PhysicsSweepCircle2DRequest> requests)
    {
        for (int i = 0; i < requests.Length; i++)
        {
            SwiftThrowHelper.ThrowIfArgument(
                requests[i].Radius <= Fixed64.Zero,
                nameof(requests),
                "2D sweep radius must be greater than zero.");
        }
    }
}
