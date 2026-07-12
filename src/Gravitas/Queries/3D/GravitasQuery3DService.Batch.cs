//=======================================================================
// GravitasQuery3DService.Batch.cs
//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp;
using Gravitas.Colliders;
using Gravitas.Support;
using SwiftCollections;
using System;

namespace Gravitas.Queries;

/// <summary>
/// Owns batched 3D query behavior for one <see cref="GravitasWorldContext"/>.
/// </summary>
public sealed partial class GravitasQuery3DService
{
    private readonly SwiftList<Physics3DHit> _batch3DHits = new();

    /// <summary>
    /// Gets the number of requests processed by the last 3D batch query call.
    /// </summary>
    public int LastBatchRequestCount { get; private set; }

    /// <summary>
    /// Gets the number of hits accepted by the last 3D batch query call.
    /// </summary>
    public int LastBatchHitCount { get; private set; }

    /// <summary>
    /// Gets the summed candidate count reported by the last 3D batch query call.
    /// </summary>
    public int LastBatchCandidateCount { get; private set; }

    /// <summary>
    /// Executes multiple 3D segment raycasts and writes one closest-hit slot per request.
    /// </summary>
    /// <returns>The number of requests that produced a hit.</returns>
    public int RaycastBatch(ReadOnlySpan<PhysicsRaycast3DRequest> requests, Span<Physics3DHit> closestHits)
    {
        ValidateClosestBatchOutput(requests.Length, closestHits);
        ResetBatchCounters(requests.Length);

        int hitCount = 0;
        for (int i = 0; i < requests.Length; i++)
        {
            PhysicsRaycast3DRequest request = requests[i];
            bool found = TryRaycastBatchRequest(request, out Physics3DHit hit);
            if (found)
            {
                closestHits[i] = hit;
                hitCount++;
            }
            else
            {
                closestHits[i] = default;
            }

            AccumulateBatchCounters(found ? 1 : 0);
        }

        LastBatchHitCount = hitCount;
        return hitCount;
    }

    /// <summary>
    /// Executes multiple 3D segment raycasts and appends all hits into one caller-owned hit buffer.
    /// </summary>
    /// <returns>The number of hits written to <paramref name="hits"/>.</returns>
    public int RaycastAllBatch(
        ReadOnlySpan<PhysicsRaycast3DRequest> requests,
        SwiftList<Physics3DHit> hits,
        Span<PhysicsQueryHitRange> ranges)
    {
        SwiftThrowHelper.ThrowIfNull(hits, nameof(hits));
        ValidateRangeBatchOutput(requests.Length, ranges);
        ResetBatchCounters(requests.Length);
        hits.FastClear();

        for (int i = 0; i < requests.Length; i++)
        {
            int start = hits.Count;
            PhysicsRaycast3DRequest request = requests[i];
            int count = RaycastAllBatchRequest(request, _batch3DHits);
            if (count > 0)
                hits.AddRange(_batch3DHits.AsReadOnlySpan());

            ranges[i] = new PhysicsQueryHitRange(start, count);
            AccumulateBatchCounters(count);
        }

        LastBatchHitCount = hits.Count;
        return hits.Count;
    }

    /// <summary>
    /// Executes multiple 3D swept-sphere queries and writes one closest-hit slot per request.
    /// </summary>
    public int SweepSphereBatch(ReadOnlySpan<PhysicsSweepSphere3DRequest> requests, Span<Physics3DHit> closestHits)
    {
        ValidateClosestBatchOutput(requests.Length, closestHits);
        ResetBatchCounters(requests.Length);

        int hitCount = 0;
        for (int i = 0; i < requests.Length; i++)
        {
            PhysicsSweepSphere3DRequest request = requests[i];
            bool found = SweepSphere(
                request.Start,
                request.End,
                request.Radius,
                request.LayerMask,
                request.ExcludedCollider,
                out Physics3DHit hit);
            WriteClosestHit(closestHits, i, hit, found, ref hitCount);
        }

        LastBatchHitCount = hitCount;
        return hitCount;
    }

    /// <summary>
    /// Executes multiple 3D swept-sphere queries and appends all hits into one caller-owned hit buffer.
    /// </summary>
    public int SweepSphereAllBatch(
        ReadOnlySpan<PhysicsSweepSphere3DRequest> requests,
        SwiftList<Physics3DHit> hits,
        Span<PhysicsQueryHitRange> ranges)
    {
        SwiftThrowHelper.ThrowIfNull(hits, nameof(hits));
        ValidateRangeBatchOutput(requests.Length, ranges);
        ResetBatchCounters(requests.Length);
        hits.FastClear();

        for (int i = 0; i < requests.Length; i++)
        {
            PhysicsSweepSphere3DRequest request = requests[i];
            int count = SweepSphereAllBatchRequest(request, _batch3DHits);
            AppendRange(hits, ranges, i, count);
        }

        LastBatchHitCount = hits.Count;
        return hits.Count;
    }

    /// <summary>
    /// Executes multiple registered capsule-source sweeps and writes one closest-hit slot per request.
    /// </summary>
    public int SweepCapsuleBatch(ReadOnlySpan<PhysicsSweepCapsule3DRequest> requests, Span<Physics3DHit> closestHits)
    {
        ValidateClosestBatchOutput(requests.Length, closestHits);
        ResetBatchCounters(requests.Length);

        int hitCount = 0;
        LSCollider? preparedSource = null;
        Vector3d preparedDisplacement = default;
        for (int i = 0; i < requests.Length; i++)
        {
            PhysicsSweepCapsule3DRequest request = requests[i];
            bool found = TrySweepSourceBatchRequest(
                request.Source,
                request.Displacement,
                request.LayerMask,
                request.ExcludedCollider,
                request.IncludeTriggers,
                ref preparedSource,
                ref preparedDisplacement,
                out Physics3DHit hit);
            WriteClosestHit(closestHits, i, hit, found, ref hitCount);
        }

        LastBatchHitCount = hitCount;
        return hitCount;
    }

    /// <summary>
    /// Executes multiple registered capsule-source sweeps and appends all hits into one caller-owned hit buffer.
    /// </summary>
    public int SweepCapsuleAllBatch(
        ReadOnlySpan<PhysicsSweepCapsule3DRequest> requests,
        SwiftList<Physics3DHit> hits,
        Span<PhysicsQueryHitRange> ranges)
    {
        SwiftThrowHelper.ThrowIfNull(hits, nameof(hits));
        ValidateRangeBatchOutput(requests.Length, ranges);
        ResetBatchCounters(requests.Length);
        hits.FastClear();

        LSCollider? preparedSource = null;
        Vector3d preparedDisplacement = default;
        for (int i = 0; i < requests.Length; i++)
        {
            PhysicsSweepCapsule3DRequest request = requests[i];
            int count = SweepSourceAllBatchRequest(
                request.Source,
                request.Displacement,
                request.LayerMask,
                request.ExcludedCollider,
                request.IncludeTriggers,
                ref preparedSource,
                ref preparedDisplacement,
                _batch3DHits);
            AppendRange(hits, ranges, i, count);
        }

        LastBatchHitCount = hits.Count;
        return hits.Count;
    }

    /// <summary>
    /// Executes multiple registered cuboid-source sweeps and writes one closest-hit slot per request.
    /// </summary>
    public int SweepCuboidBatch(ReadOnlySpan<PhysicsSweepCuboid3DRequest> requests, Span<Physics3DHit> closestHits)
    {
        ValidateClosestBatchOutput(requests.Length, closestHits);
        ResetBatchCounters(requests.Length);

        int hitCount = 0;
        LSCollider? preparedSource = null;
        Vector3d preparedDisplacement = default;
        for (int i = 0; i < requests.Length; i++)
        {
            PhysicsSweepCuboid3DRequest request = requests[i];
            bool found = TrySweepSourceBatchRequest(
                request.Source,
                request.Displacement,
                request.LayerMask,
                request.ExcludedCollider,
                request.IncludeTriggers,
                ref preparedSource,
                ref preparedDisplacement,
                out Physics3DHit hit);
            WriteClosestHit(closestHits, i, hit, found, ref hitCount);
        }

        LastBatchHitCount = hitCount;
        return hitCount;
    }

    /// <summary>
    /// Executes multiple registered cuboid-source sweeps and appends all hits into one caller-owned hit buffer.
    /// </summary>
    public int SweepCuboidAllBatch(
        ReadOnlySpan<PhysicsSweepCuboid3DRequest> requests,
        SwiftList<Physics3DHit> hits,
        Span<PhysicsQueryHitRange> ranges)
    {
        SwiftThrowHelper.ThrowIfNull(hits, nameof(hits));
        ValidateRangeBatchOutput(requests.Length, ranges);
        ResetBatchCounters(requests.Length);
        hits.FastClear();

        LSCollider? preparedSource = null;
        Vector3d preparedDisplacement = default;
        for (int i = 0; i < requests.Length; i++)
        {
            PhysicsSweepCuboid3DRequest request = requests[i];
            int count = SweepSourceAllBatchRequest(
                request.Source,
                request.Displacement,
                request.LayerMask,
                request.ExcludedCollider,
                request.IncludeTriggers,
                ref preparedSource,
                ref preparedDisplacement,
                _batch3DHits);
            AppendRange(hits, ranges, i, count);
        }

        LastBatchHitCount = hits.Count;
        return hits.Count;
    }

    /// <summary>
    /// Executes multiple registered cylinder-source sweeps and writes one closest-hit slot per request.
    /// </summary>
    public int SweepCylinderBatch(ReadOnlySpan<PhysicsSweepCylinder3DRequest> requests, Span<Physics3DHit> closestHits)
    {
        ValidateClosestBatchOutput(requests.Length, closestHits);
        ResetBatchCounters(requests.Length);

        int hitCount = 0;
        LSCollider? preparedSource = null;
        Vector3d preparedDisplacement = default;
        for (int i = 0; i < requests.Length; i++)
        {
            PhysicsSweepCylinder3DRequest request = requests[i];
            bool found = TrySweepSourceBatchRequest(
                request.Source,
                request.Displacement,
                request.LayerMask,
                request.ExcludedCollider,
                request.IncludeTriggers,
                ref preparedSource,
                ref preparedDisplacement,
                out Physics3DHit hit);
            WriteClosestHit(closestHits, i, hit, found, ref hitCount);
        }

        LastBatchHitCount = hitCount;
        return hitCount;
    }

    /// <summary>
    /// Executes multiple registered cylinder-source sweeps and appends all hits into one caller-owned hit buffer.
    /// </summary>
    public int SweepCylinderAllBatch(
        ReadOnlySpan<PhysicsSweepCylinder3DRequest> requests,
        SwiftList<Physics3DHit> hits,
        Span<PhysicsQueryHitRange> ranges)
    {
        SwiftThrowHelper.ThrowIfNull(hits, nameof(hits));
        ValidateRangeBatchOutput(requests.Length, ranges);
        ResetBatchCounters(requests.Length);
        hits.FastClear();

        LSCollider? preparedSource = null;
        Vector3d preparedDisplacement = default;
        for (int i = 0; i < requests.Length; i++)
        {
            PhysicsSweepCylinder3DRequest request = requests[i];
            int count = SweepSourceAllBatchRequest(
                request.Source,
                request.Displacement,
                request.LayerMask,
                request.ExcludedCollider,
                request.IncludeTriggers,
                ref preparedSource,
                ref preparedDisplacement,
                _batch3DHits);
            AppendRange(hits, ranges, i, count);
        }

        LastBatchHitCount = hits.Count;
        return hits.Count;
    }

    /// <summary>
    /// Executes multiple registered cone-source sweeps and writes one closest-hit slot per request.
    /// </summary>
    public int SweepConeBatch(ReadOnlySpan<PhysicsSweepCone3DRequest> requests, Span<Physics3DHit> closestHits)
    {
        ValidateClosestBatchOutput(requests.Length, closestHits);
        ResetBatchCounters(requests.Length);

        int hitCount = 0;
        LSCollider? preparedSource = null;
        Vector3d preparedDisplacement = default;
        for (int i = 0; i < requests.Length; i++)
        {
            PhysicsSweepCone3DRequest request = requests[i];
            bool found = TrySweepSourceBatchRequest(
                request.Source,
                request.Displacement,
                request.LayerMask,
                request.ExcludedCollider,
                request.IncludeTriggers,
                ref preparedSource,
                ref preparedDisplacement,
                out Physics3DHit hit);
            WriteClosestHit(closestHits, i, hit, found, ref hitCount);
        }

        LastBatchHitCount = hitCount;
        return hitCount;
    }

    /// <summary>
    /// Executes multiple registered cone-source sweeps and appends all hits into one caller-owned hit buffer.
    /// </summary>
    public int SweepConeAllBatch(
        ReadOnlySpan<PhysicsSweepCone3DRequest> requests,
        SwiftList<Physics3DHit> hits,
        Span<PhysicsQueryHitRange> ranges)
    {
        SwiftThrowHelper.ThrowIfNull(hits, nameof(hits));
        ValidateRangeBatchOutput(requests.Length, ranges);
        ResetBatchCounters(requests.Length);
        hits.FastClear();

        LSCollider? preparedSource = null;
        Vector3d preparedDisplacement = default;
        for (int i = 0; i < requests.Length; i++)
        {
            PhysicsSweepCone3DRequest request = requests[i];
            int count = SweepSourceAllBatchRequest(
                request.Source,
                request.Displacement,
                request.LayerMask,
                request.ExcludedCollider,
                request.IncludeTriggers,
                ref preparedSource,
                ref preparedDisplacement,
                _batch3DHits);
            AppendRange(hits, ranges, i, count);
        }

        LastBatchHitCount = hits.Count;
        return hits.Count;
    }

    /// <summary>
    /// Executes multiple registered convex mesh-source sweeps and writes one closest-hit slot per request.
    /// </summary>
    public int SweepConvexMeshBatch(ReadOnlySpan<PhysicsSweepConvexMesh3DRequest> requests, Span<Physics3DHit> closestHits)
    {
        ValidateClosestBatchOutput(requests.Length, closestHits);
        ResetBatchCounters(requests.Length);

        int hitCount = 0;
        LSCollider? preparedSource = null;
        Vector3d preparedDisplacement = default;
        for (int i = 0; i < requests.Length; i++)
        {
            PhysicsSweepConvexMesh3DRequest request = requests[i];
            bool found = TrySweepSourceBatchRequest(
                request.Source,
                request.Displacement,
                request.LayerMask,
                request.ExcludedCollider,
                request.IncludeTriggers,
                ref preparedSource,
                ref preparedDisplacement,
                out Physics3DHit hit);
            WriteClosestHit(closestHits, i, hit, found, ref hitCount);
        }

        LastBatchHitCount = hitCount;
        return hitCount;
    }

    /// <summary>
    /// Executes multiple registered convex mesh-source sweeps and appends all hits into one caller-owned hit buffer.
    /// </summary>
    public int SweepConvexMeshAllBatch(
        ReadOnlySpan<PhysicsSweepConvexMesh3DRequest> requests,
        SwiftList<Physics3DHit> hits,
        Span<PhysicsQueryHitRange> ranges)
    {
        SwiftThrowHelper.ThrowIfNull(hits, nameof(hits));
        ValidateRangeBatchOutput(requests.Length, ranges);
        ResetBatchCounters(requests.Length);
        hits.FastClear();

        LSCollider? preparedSource = null;
        Vector3d preparedDisplacement = default;
        for (int i = 0; i < requests.Length; i++)
        {
            PhysicsSweepConvexMesh3DRequest request = requests[i];
            int count = SweepSourceAllBatchRequest(
                request.Source,
                request.Displacement,
                request.LayerMask,
                request.ExcludedCollider,
                request.IncludeTriggers,
                ref preparedSource,
                ref preparedDisplacement,
                _batch3DHits);
            AppendRange(hits, ranges, i, count);
        }

        LastBatchHitCount = hits.Count;
        return hits.Count;
    }

    /// <summary>
    /// Executes multiple registered compound-source sweeps and writes one closest-hit slot per request.
    /// </summary>
    public int SweepCompoundBatch(ReadOnlySpan<PhysicsSweepCompound3DRequest> requests, Span<Physics3DHit> closestHits)
    {
        ValidateClosestBatchOutput(requests.Length, closestHits);
        ResetBatchCounters(requests.Length);

        int hitCount = 0;
        LSCollider? preparedSource = null;
        Vector3d preparedDisplacement = default;
        for (int i = 0; i < requests.Length; i++)
        {
            PhysicsSweepCompound3DRequest request = requests[i];
            bool found = TrySweepSourceBatchRequest(
                request.Source,
                request.Displacement,
                request.LayerMask,
                request.ExcludedCollider,
                request.IncludeTriggers,
                ref preparedSource,
                ref preparedDisplacement,
                out Physics3DHit hit);
            WriteClosestHit(closestHits, i, hit, found, ref hitCount);
        }

        LastBatchHitCount = hitCount;
        return hitCount;
    }

    /// <summary>
    /// Executes multiple registered compound-source sweeps and appends all hits into one caller-owned hit buffer.
    /// </summary>
    public int SweepCompoundAllBatch(
        ReadOnlySpan<PhysicsSweepCompound3DRequest> requests,
        SwiftList<Physics3DHit> hits,
        Span<PhysicsQueryHitRange> ranges)
    {
        SwiftThrowHelper.ThrowIfNull(hits, nameof(hits));
        ValidateRangeBatchOutput(requests.Length, ranges);
        ResetBatchCounters(requests.Length);
        hits.FastClear();

        LSCollider? preparedSource = null;
        Vector3d preparedDisplacement = default;
        for (int i = 0; i < requests.Length; i++)
        {
            PhysicsSweepCompound3DRequest request = requests[i];
            int count = SweepSourceAllBatchRequest(
                request.Source,
                request.Displacement,
                request.LayerMask,
                request.ExcludedCollider,
                request.IncludeTriggers,
                ref preparedSource,
                ref preparedDisplacement,
                _batch3DHits);
            AppendRange(hits, ranges, i, count);
        }

        LastBatchHitCount = hits.Count;
        return hits.Count;
    }

    /// <summary>
    /// Executes multiple 3D X/Z circle overlaps and writes one closest-hit slot per request.
    /// </summary>
    public int OverlapCircleBatch(ReadOnlySpan<PhysicsOverlapCircle3DRequest> requests, Span<Physics3DHit> closestHits)
    {
        ValidateClosestBatchOutput(requests.Length, closestHits);
        ResetBatchCounters(requests.Length);

        int hitCount = 0;
        for (int i = 0; i < requests.Length; i++)
        {
            PhysicsOverlapCircle3DRequest request = requests[i];
            bool found = TryOverlapCircleBatchRequest(request, out Physics3DHit hit);
            WriteClosestHit(closestHits, i, hit, found, ref hitCount);
        }

        LastBatchHitCount = hitCount;
        return hitCount;
    }

    /// <summary>
    /// Executes multiple 3D X/Z circle overlaps and appends all hits into one caller-owned hit buffer.
    /// </summary>
    public int OverlapCircleAllBatch(
        ReadOnlySpan<PhysicsOverlapCircle3DRequest> requests,
        SwiftList<Physics3DHit> hits,
        Span<PhysicsQueryHitRange> ranges)
    {
        SwiftThrowHelper.ThrowIfNull(hits, nameof(hits));
        ValidateRangeBatchOutput(requests.Length, ranges);
        ResetBatchCounters(requests.Length);
        hits.FastClear();

        for (int i = 0; i < requests.Length; i++)
        {
            PhysicsOverlapCircle3DRequest request = requests[i];
            int count = OverlapCircleAllBatchRequest(request, _batch3DHits);
            AppendRange(hits, ranges, i, count);
        }

        LastBatchHitCount = hits.Count;
        return hits.Count;
    }

    /// <summary>
    /// Executes multiple directional cone-volume overlaps and writes one closest-hit slot per request.
    /// </summary>
    public int OverlapConeBatch(ReadOnlySpan<PhysicsOverlapCone3DRequest> requests, Span<Physics3DHit> closestHits)
    {
        ValidateClosestBatchOutput(requests.Length, closestHits);
        ResetBatchCounters(requests.Length);

        int hitCount = 0;
        for (int i = 0; i < requests.Length; i++)
        {
            PhysicsOverlapCone3DRequest request = requests[i];
            bool found = TryOverlapConeBatchRequest(request, out Physics3DHit hit);
            WriteClosestHit(closestHits, i, hit, found, ref hitCount);
        }

        LastBatchHitCount = hitCount;
        return hitCount;
    }

    /// <summary>
    /// Executes multiple directional cone-volume overlaps and appends all hits into one caller-owned hit buffer.
    /// </summary>
    public int OverlapConeAllBatch(
        ReadOnlySpan<PhysicsOverlapCone3DRequest> requests,
        SwiftList<Physics3DHit> hits,
        Span<PhysicsQueryHitRange> ranges)
    {
        SwiftThrowHelper.ThrowIfNull(hits, nameof(hits));
        ValidateRangeBatchOutput(requests.Length, ranges);
        ResetBatchCounters(requests.Length);
        hits.FastClear();

        for (int i = 0; i < requests.Length; i++)
        {
            PhysicsOverlapCone3DRequest request = requests[i];
            int count = OverlapConeAllBatchRequest(request, _batch3DHits);
            AppendRange(hits, ranges, i, count);
        }

        LastBatchHitCount = hits.Count;
        return hits.Count;
    }

    /// <summary>
    /// Executes multiple directional 3D X/Z circle proximity queries and writes one closest-hit slot per request.
    /// </summary>
    public int OverlapCircleInDirectionBatch(
        ReadOnlySpan<PhysicsOverlapCircleInDirection3DRequest> requests,
        Span<Physics3DHit> closestHits)
    {
        ValidateClosestBatchOutput(requests.Length, closestHits);
        ResetBatchCounters(requests.Length);

        int hitCount = 0;
        for (int i = 0; i < requests.Length; i++)
        {
            PhysicsOverlapCircleInDirection3DRequest request = requests[i];
            bool found = TryOverlapCircleInDirectionBatchRequest(request, out Physics3DHit hit);
            WriteClosestHit(closestHits, i, hit, found, ref hitCount);
        }

        LastBatchHitCount = hitCount;
        return hitCount;
    }

    private bool TryRaycastBatchRequest(PhysicsRaycast3DRequest request, out Physics3DHit hit)
    {
        _currentLayerMask = request.LayerMask;
        Vector3d segment = request.End - request.Start;
        if (segment.MagnitudeSquared == Fixed64.Zero)
        {
            ResetLastQueryCounters();
            hit = default;
            return false;
        }

        Vector3d direction = segment.Normalized;
        BeginRaycastTrace(request.Start, request.End);
        return TryFindClosestHit(request.Start, request.End, direction, out hit);
    }

    private int RaycastAllBatchRequest(PhysicsRaycast3DRequest request, SwiftList<Physics3DHit> results)
    {
        _currentLayerMask = request.LayerMask;
        results.FastClear();
        Vector3d segment = request.End - request.Start;
        if (segment.MagnitudeSquared == Fixed64.Zero)
        {
            ResetLastQueryCounters();
            return 0;
        }

        BeginRaycastTrace(request.Start, request.End);
        AddAllHits(request.Start, request.End, segment.Normalized, results);
        Physics3DHitSorter.SortByDistance(results);
        return results.Count;
    }

    private int SweepSphereAllBatchRequest(PhysicsSweepSphere3DRequest request, SwiftList<Physics3DHit> results)
    {
        results.FastClear();
        Vector3d segment = request.End - request.Start;
        if (segment.MagnitudeSquared == Fixed64.Zero || request.Radius <= Fixed64.Zero)
        {
            ResetLastQueryCounters();
            return 0;
        }

        BeginSweepTrace(
            request.Start,
            request.End,
            request.Radius,
            request.LayerMask,
            request.ExcludedCollider);
        AddAllSweepHits(request.Start, request.End, segment.Normalized, request.Radius, results);
        Physics3DHitSorter.SortByDistance(results);
        return results.Count;
    }

    private bool TrySweepSourceBatchRequest(
        LSCollider? source,
        Vector3d displacement,
        PhysicsLayerMask layerMask,
        LSCollider? excludedCollider,
        bool includeTriggers,
        ref LSCollider? preparedSource,
        ref Vector3d preparedDisplacement,
        out Physics3DHit hit)
    {
        PrepareBatchConvexSweepSource(source, displacement, ref preparedSource, ref preparedDisplacement);
        LSCollider prepared = preparedSource!;
        if (displacement.MagnitudeSquared <= Fixed64.Epsilon)
        {
            ResetLastQueryCounters();
            hit = default;
            return false;
        }

        BeginConvexSweepTrace(prepared, layerMask, excludedCollider, includeTriggers);
        return TryFindClosestConvexSweepHit(prepared, displacement, out hit);
    }

    private int SweepSourceAllBatchRequest(
        LSCollider? source,
        Vector3d displacement,
        PhysicsLayerMask layerMask,
        LSCollider? excludedCollider,
        bool includeTriggers,
        ref LSCollider? preparedSource,
        ref Vector3d preparedDisplacement,
        SwiftList<Physics3DHit> results)
    {
        results.FastClear();
        PrepareBatchConvexSweepSource(source, displacement, ref preparedSource, ref preparedDisplacement);
        LSCollider prepared = preparedSource!;
        if (displacement.MagnitudeSquared <= Fixed64.Epsilon)
        {
            ResetLastQueryCounters();
            return 0;
        }

        BeginConvexSweepTrace(prepared, layerMask, excludedCollider, includeTriggers);
        AddAllConvexSweepHits(prepared, displacement, results);
        Physics3DHitSorter.SortByDistance(results);
        return results.Count;
    }

    private void PrepareBatchConvexSweepSource(
        LSCollider? source,
        Vector3d displacement,
        ref LSCollider? preparedSource,
        ref Vector3d preparedDisplacement)
    {
        SwiftThrowHelper.ThrowIfNull(source, nameof(source));
        EnsureSourceBelongsToContext(source!);

        if (ReferenceEquals(source, preparedSource) && displacement.Equals(preparedDisplacement))
            return;

        PrepareConvexSweepSource(source!, displacement);
        preparedSource = source;
        preparedDisplacement = displacement;
    }

    private bool TryOverlapCircleBatchRequest(PhysicsOverlapCircle3DRequest request, out Physics3DHit hit)
    {
        _currentLayerMask = request.LayerMask;
        NextCircleVersion();
        ResetLastQueryCounters();
        _redundantColliderCheck.Clear();
        _redundantVoxelCheck.Clear();

        bool found = false;
        Physics3DHit closest = default;
        Fixed64 closestDistance = Fixed64.MaxValue;
        TraceCircleForClosestHit(
            request.Position,
            request.Radius,
            ref found,
            ref closest,
            ref closestDistance);

        hit = closest;
        return found;
    }

    private int OverlapCircleAllBatchRequest(PhysicsOverlapCircle3DRequest request, SwiftList<Physics3DHit> results)
    {
        _currentLayerMask = request.LayerMask;
        NextCircleVersion();
        ResetLastQueryCounters();
        results.FastClear();
        _redundantColliderCheck.Clear();
        _redundantVoxelCheck.Clear();

        TraceCircleForAllHits(request.Position, request.Radius, results);
        Physics3DHitSorter.SortByDistance(results);
        return results.Count;
    }

    private bool TryOverlapCircleInDirectionBatchRequest(
        PhysicsOverlapCircleInDirection3DRequest request,
        out Physics3DHit hit)
    {
        _currentLayerMask = request.LayerMask;
        NextCircleVersion();
        ResetLastQueryCounters();
        _redundantColliderCheck.Clear();
        _redundantVoxelCheck.Clear();

        Vector3d direction = request.Direction.MagnitudeSquared == Fixed64.Zero
            ? Vector3d.Zero
            : request.Direction.Normalized;
        Fixed64 maxDistanceSqr = request.MaxDistance * request.MaxDistance;
        bool found = false;
        Physics3DHit closest = default;
        Fixed64 closestDistance = Fixed64.MaxValue;
        TraceCircleForDirectionalHit(
            request.Position,
            request.Radius,
            direction,
            maxDistanceSqr,
            ref found,
            ref closest,
            ref closestDistance);

        hit = found ? closest : default;
        return found;
    }

    private bool TryOverlapConeBatchRequest(PhysicsOverlapCone3DRequest request, out Physics3DHit hit) =>
        OverlapCone(
            request.Origin,
            request.Direction,
            request.Length,
            request.EndRadius,
            out hit,
            request.LayerMask);

    private int OverlapConeAllBatchRequest(PhysicsOverlapCone3DRequest request, SwiftList<Physics3DHit> results) =>
        OverlapConeAll(
            request.Origin,
            request.Direction,
            request.Length,
            request.EndRadius,
            request.LayerMask,
            results);

    private void WriteClosestHit(Span<Physics3DHit> closestHits, int index, Physics3DHit hit, bool found, ref int hitCount)
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
        SwiftList<Physics3DHit> hits,
        Span<PhysicsQueryHitRange> ranges,
        int index,
        int count)
    {
        int start = hits.Count;
        if (count > 0)
            hits.AddRange(_batch3DHits.AsReadOnlySpan());

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

    private static void ValidateClosestBatchOutput(int requestCount, Span<Physics3DHit> closestHits)
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
}
