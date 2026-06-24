//=======================================================================
// GravitasQuery3DService.Raycast.cs
//=======================================================================
// MIT License, Copyright (c) 2026–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp;
using Gravitas.Colliders;
using Gravitas.Support;
using GridForge;
using GridForge.Grids;
using GridForge.Utility;
using SwiftCollections;
using System.Runtime.CompilerServices;

namespace Gravitas.Queries;

/// <summary>
/// Owns 3D raycast, swept-sphere, and X/Z circle query buffers for one <see cref="GravitasWorldContext"/>.
/// </summary>
public sealed partial class GravitasQuery3DService
{
    private readonly GravitasWorldContext _context;
    private readonly RaycastSegmentWorker _worker = new();
    private readonly SweptSphereQueryWorker _sweepWorker = new();
    private readonly ConvexSweepQueryWorker _convexSweepWorker = new();
    private SwiftList<Vector3d> _bufferIntersectionPoints = new();
    private readonly SwiftHashSet<int> _redundantColliderCheck = new();
    private readonly SwiftHashSet<int> _redundantVoxelCheck = new();
    private readonly SwiftList<Voxel> _coveredVoxels = new();
    private readonly GridTraceScratch _traceScratch = new();

    private PhysicsLayerMask _currentLayerMask;
    private LSCollider? _currentExcludedCollider;
    private LSCollider? _currentSweepSourceCollider;
    private bool _currentStaticSweepTargetsOnly;
    private bool _currentIncludeTriggers;

    /// <summary>
    /// Initializes a new raycast service for the supplied context.
    /// </summary>
    /// <param name="context">The owning world context.</param>
    public GravitasQuery3DService(GravitasWorldContext context)
    {
        SwiftThrowHelper.ThrowIfNull(context, nameof(context));
        _context = context;
    }

    /// <summary>
    /// Gets the owning world context.
    /// </summary>
    public GravitasWorldContext Context => _context;

    /// <summary>
    /// Gets the context-local raycast/sweep query version.
    /// </summary>
    public uint RaycastVersion { get; private set; }

    /// <summary>
    /// Gets the context-local 3D X/Z circle query version.
    /// </summary>
    public uint CircleVersion { get; private set; }

    internal int LastQueryCandidateCount { get; private set; }

    /// <summary>
    /// Resets context-local raycast query buffers.
    /// </summary>
    public void Reset()
    {
        RaycastVersion = 0;
        CircleVersion = 0;
        LastQueryCandidateCount = 0;
        _bufferIntersectionPoints.FastClear();
        _redundantColliderCheck.Clear();
        _redundantVoxelCheck.Clear();
        _coveredVoxels.FastClear();
        _traceScratch.Clear();
    }

    /// <summary>
    /// Performs a raycast from an origin in a direction up to a maximum distance.
    /// </summary>
    public bool Raycast(
        Vector3d origin,
        Vector3d direction,
        Fixed64 maxDistance,
        out Physics3DHit raycastHit,
        PhysicsLayerMask layerMask)
    {
        _currentLayerMask = layerMask;
        if (direction.MagnitudeSquared == Fixed64.Zero || maxDistance <= Fixed64.Zero)
        {
            LastQueryCandidateCount = 0;
            raycastHit = default;
            return false;
        }

        Vector3d rayDirection = direction.Normalized;
        Vector3d end = origin + rayDirection * maxDistance;

        BeginRaycastTrace(origin, end);
        bool hit = TryFindClosestHit(origin, end, rayDirection, out raycastHit);
        _context.Diagnostics.EmitRayQuery(
            origin,
            end,
            Fixed64.Zero,
            layerMask.Bits,
            hit,
            hit ? 1 : 0,
            raycastHit);
        return hit;
    }

    /// <summary>
    /// Executes a raycast between two points and writes hits from closest to farthest into caller-owned storage.
    /// </summary>
    public int RaycastAll(
        Vector3d start3d,
        Vector3d end3d,
        PhysicsLayerMask layerMask,
        SwiftList<Physics3DHit> results)
    {
        SwiftThrowHelper.ThrowIfNull(results, nameof(results));

        _currentLayerMask = layerMask;
        results.FastClear();

        Vector3d segment = end3d - start3d;
        if (segment.MagnitudeSquared == Fixed64.Zero)
        {
            LastQueryCandidateCount = 0;
            return 0;
        }

        BeginRaycastTrace(start3d, end3d);
        AddAllHits(start3d, end3d, segment.Normalized, results);
        Physics3DHitSorter.SortByDistance(results);
        _context.Diagnostics.EmitRayQuery(
            start3d,
            end3d,
            Fixed64.Zero,
            layerMask.Bits,
            results.Count > 0,
            results.Count,
            results.Count > 0 ? results[0] : default);
        return results.Count;
    }

    /// <summary>
    /// Sweeps a sphere from an origin in a direction up to a maximum distance and returns the closest hit.
    /// </summary>
    public bool SweepSphere(
        Vector3d origin,
        Fixed64 radius,
        Vector3d direction,
        Fixed64 maxDistance,
        out Physics3DHit sweepHit,
        PhysicsLayerMask layerMask,
        LSCollider? excludedCollider = null)
    {
        if (direction.MagnitudeSquared == Fixed64.Zero || maxDistance <= Fixed64.Zero)
        {
            LastQueryCandidateCount = 0;
            sweepHit = default;
            return false;
        }

        Vector3d sweepDirection = direction.Normalized;
        Vector3d end = origin + sweepDirection * maxDistance;
        bool hit = SweepSphere(origin, end, radius, layerMask, excludedCollider, out sweepHit);
        _context.Diagnostics.EmitRayQuery(
            origin,
            end,
            radius,
            layerMask.Bits,
            hit,
            hit ? 1 : 0,
            sweepHit);
        return hit;
    }

    /// <summary>
    /// Sweeps a sphere between two points and writes hits from closest to farthest into caller-owned storage.
    /// </summary>
    public int SweepSphereAll(
        Vector3d start3d,
        Vector3d end3d,
        Fixed64 radius,
        PhysicsLayerMask layerMask,
        SwiftList<Physics3DHit> results,
        LSCollider? excludedCollider = null)
    {
        return SweepSphereAllCore(
            start3d,
            end3d,
            radius,
            layerMask,
            results,
            excludedCollider,
            includeTriggers: true,
            staticTargetsOnly: false);
    }

    internal int SweepSphereAgainstStaticAll(
        Vector3d start3d,
        Vector3d end3d,
        Fixed64 radius,
        PhysicsLayerMask layerMask,
        SwiftList<Physics3DHit> results,
        LSCollider? excludedCollider = null,
        bool includeTriggers = true)
    {
        return SweepSphereAllCore(
            start3d,
            end3d,
            radius,
            layerMask,
            results,
            excludedCollider,
            includeTriggers,
            staticTargetsOnly: true);
    }

    /// <summary>
    /// Sweeps a capsule source by <paramref name="displacement"/> and returns the closest 3D target hit.
    /// </summary>
    public bool SweepCapsule(
        LSCapsuleCollider source,
        Vector3d displacement,
        PhysicsLayerMask layerMask,
        out Physics3DHit sweepHit,
        LSCollider? excludedCollider = null,
        bool includeTriggers = true)
    {
        SwiftThrowHelper.ThrowIfNull(source, nameof(source));
        return SweepPrimitiveSource(source, displacement, layerMask, out sweepHit, excludedCollider, includeTriggers);
    }

    /// <summary>
    /// Writes all 3D targets hit by sweeping a capsule source into caller-owned storage.
    /// </summary>
    public int SweepCapsuleAll(
        LSCapsuleCollider source,
        Vector3d displacement,
        PhysicsLayerMask layerMask,
        SwiftList<Physics3DHit> results,
        LSCollider? excludedCollider = null,
        bool includeTriggers = true)
    {
        SwiftThrowHelper.ThrowIfNull(source, nameof(source));
        SwiftThrowHelper.ThrowIfNull(results, nameof(results));
        return SweepPrimitiveSourceAll(source, displacement, layerMask, results, excludedCollider, includeTriggers);
    }

    /// <summary>
    /// Sweeps a cuboid source by <paramref name="displacement"/> and returns the closest 3D target hit.
    /// </summary>
    public bool SweepCuboid(
        LSCuboidCollider source,
        Vector3d displacement,
        PhysicsLayerMask layerMask,
        out Physics3DHit sweepHit,
        LSCollider? excludedCollider = null,
        bool includeTriggers = true)
    {
        SwiftThrowHelper.ThrowIfNull(source, nameof(source));
        return SweepPrimitiveSource(source, displacement, layerMask, out sweepHit, excludedCollider, includeTriggers);
    }

    /// <summary>
    /// Writes all 3D targets hit by sweeping a cuboid source into caller-owned storage.
    /// </summary>
    public int SweepCuboidAll(
        LSCuboidCollider source,
        Vector3d displacement,
        PhysicsLayerMask layerMask,
        SwiftList<Physics3DHit> results,
        LSCollider? excludedCollider = null,
        bool includeTriggers = true)
    {
        SwiftThrowHelper.ThrowIfNull(source, nameof(source));
        SwiftThrowHelper.ThrowIfNull(results, nameof(results));
        return SweepPrimitiveSourceAll(source, displacement, layerMask, results, excludedCollider, includeTriggers);
    }

    /// <summary>
    /// Sweeps a finite-cylinder source by <paramref name="displacement"/> and returns the closest 3D target hit.
    /// </summary>
    public bool SweepCylinder(
        LSCylinderCollider source,
        Vector3d displacement,
        PhysicsLayerMask layerMask,
        out Physics3DHit sweepHit,
        LSCollider? excludedCollider = null,
        bool includeTriggers = true)
    {
        SwiftThrowHelper.ThrowIfNull(source, nameof(source));
        return SweepPrimitiveSource(source, displacement, layerMask, out sweepHit, excludedCollider, includeTriggers);
    }

    /// <summary>
    /// Writes all 3D targets hit by sweeping a finite-cylinder source into caller-owned storage.
    /// </summary>
    public int SweepCylinderAll(
        LSCylinderCollider source,
        Vector3d displacement,
        PhysicsLayerMask layerMask,
        SwiftList<Physics3DHit> results,
        LSCollider? excludedCollider = null,
        bool includeTriggers = true)
    {
        SwiftThrowHelper.ThrowIfNull(source, nameof(source));
        SwiftThrowHelper.ThrowIfNull(results, nameof(results));
        return SweepPrimitiveSourceAll(source, displacement, layerMask, results, excludedCollider, includeTriggers);
    }

    /// <summary>
    /// Sweeps a convex mesh source by <paramref name="displacement"/> and returns the closest 3D target hit.
    /// </summary>
    public bool SweepConvexMesh(
        LSMeshCollider source,
        Vector3d displacement,
        PhysicsLayerMask layerMask,
        out Physics3DHit sweepHit,
        LSCollider? excludedCollider = null,
        bool includeTriggers = true)
    {
        SwiftThrowHelper.ThrowIfNull(source, nameof(source));
        return SweepConvexMeshSource(source, displacement, layerMask, out sweepHit, excludedCollider, includeTriggers);
    }

    /// <summary>
    /// Writes all 3D targets hit by sweeping a convex mesh source into caller-owned storage.
    /// </summary>
    public int SweepConvexMeshAll(
        LSMeshCollider source,
        Vector3d displacement,
        PhysicsLayerMask layerMask,
        SwiftList<Physics3DHit> results,
        LSCollider? excludedCollider = null,
        bool includeTriggers = true)
    {
        SwiftThrowHelper.ThrowIfNull(source, nameof(source));
        SwiftThrowHelper.ThrowIfNull(results, nameof(results));
        return SweepConvexMeshSourceAll(source, displacement, layerMask, results, excludedCollider, includeTriggers);
    }

    /// <summary>
    /// Sweeps an authored compound source by <paramref name="displacement"/> and returns the closest 3D target hit.
    /// </summary>
    public bool SweepCompound(
        LSCompoundCollider source,
        Vector3d displacement,
        PhysicsLayerMask layerMask,
        out Physics3DHit sweepHit,
        LSCollider? excludedCollider = null,
        bool includeTriggers = true)
    {
        SwiftThrowHelper.ThrowIfNull(source, nameof(source));
        return SweepCompoundSource(source, displacement, layerMask, out sweepHit, excludedCollider, includeTriggers);
    }

    /// <summary>
    /// Writes all 3D targets hit by sweeping an authored compound source into caller-owned storage.
    /// </summary>
    public int SweepCompoundAll(
        LSCompoundCollider source,
        Vector3d displacement,
        PhysicsLayerMask layerMask,
        SwiftList<Physics3DHit> results,
        LSCollider? excludedCollider = null,
        bool includeTriggers = true)
    {
        SwiftThrowHelper.ThrowIfNull(source, nameof(source));
        SwiftThrowHelper.ThrowIfNull(results, nameof(results));
        return SweepCompoundSourceAll(source, displacement, layerMask, results, excludedCollider, includeTriggers);
    }

    private bool SweepPrimitiveSource(
        LSCollider source,
        Vector3d displacement,
        PhysicsLayerMask layerMask,
        out Physics3DHit sweepHit,
        LSCollider? excludedCollider,
        bool includeTriggers)
    {
        EnsureSourceBelongsToContext(source);
        _convexSweepWorker.PreparePrimitiveSource(source, displacement);
        return SweepPreparedConvexSource(source, displacement, layerMask, out sweepHit, excludedCollider, includeTriggers);
    }

    private int SweepPrimitiveSourceAll(
        LSCollider source,
        Vector3d displacement,
        PhysicsLayerMask layerMask,
        SwiftList<Physics3DHit> results,
        LSCollider? excludedCollider,
        bool includeTriggers)
    {
        EnsureSourceBelongsToContext(source);
        _convexSweepWorker.PreparePrimitiveSource(source, displacement);
        return SweepPreparedConvexSourceAll(source, displacement, layerMask, results, excludedCollider, includeTriggers);
    }

    private bool SweepConvexMeshSource(
        LSMeshCollider source,
        Vector3d displacement,
        PhysicsLayerMask layerMask,
        out Physics3DHit sweepHit,
        LSCollider? excludedCollider,
        bool includeTriggers)
    {
        EnsureSourceBelongsToContext(source);
        _convexSweepWorker.PrepareConvexMeshSource(source, displacement);
        return SweepPreparedConvexSource(source, displacement, layerMask, out sweepHit, excludedCollider, includeTriggers);
    }

    private int SweepConvexMeshSourceAll(
        LSMeshCollider source,
        Vector3d displacement,
        PhysicsLayerMask layerMask,
        SwiftList<Physics3DHit> results,
        LSCollider? excludedCollider,
        bool includeTriggers)
    {
        EnsureSourceBelongsToContext(source);
        _convexSweepWorker.PrepareConvexMeshSource(source, displacement);
        return SweepPreparedConvexSourceAll(source, displacement, layerMask, results, excludedCollider, includeTriggers);
    }

    private bool SweepCompoundSource(
        LSCompoundCollider source,
        Vector3d displacement,
        PhysicsLayerMask layerMask,
        out Physics3DHit sweepHit,
        LSCollider? excludedCollider,
        bool includeTriggers)
    {
        EnsureSourceBelongsToContext(source);
        _convexSweepWorker.PrepareCompoundSource(source, displacement);
        return SweepPreparedConvexSource(source, displacement, layerMask, out sweepHit, excludedCollider, includeTriggers);
    }

    private int SweepCompoundSourceAll(
        LSCompoundCollider source,
        Vector3d displacement,
        PhysicsLayerMask layerMask,
        SwiftList<Physics3DHit> results,
        LSCollider? excludedCollider,
        bool includeTriggers)
    {
        EnsureSourceBelongsToContext(source);
        _convexSweepWorker.PrepareCompoundSource(source, displacement);
        return SweepPreparedConvexSourceAll(source, displacement, layerMask, results, excludedCollider, includeTriggers);
    }

    private int SweepSphereAllCore(
        Vector3d start3d,
        Vector3d end3d,
        Fixed64 radius,
        PhysicsLayerMask layerMask,
        SwiftList<Physics3DHit> results,
        LSCollider? excludedCollider,
        bool includeTriggers,
        bool staticTargetsOnly)
    {
        SwiftThrowHelper.ThrowIfNull(results, nameof(results));

        results.FastClear();
        Vector3d segment = end3d - start3d;
        if (segment.MagnitudeSquared == Fixed64.Zero || radius <= Fixed64.Zero)
        {
            LastQueryCandidateCount = 0;
            return 0;
        }

        BeginSweepTrace(start3d, end3d, radius, layerMask, excludedCollider, includeTriggers, staticTargetsOnly);
        AddAllSweepHits(start3d, end3d, segment.Normalized, radius, results);
        Physics3DHitSorter.SortByDistance(results);
        _context.Diagnostics.EmitRayQuery(
            start3d,
            end3d,
            radius,
            layerMask.Bits,
            results.Count > 0,
            results.Count,
            results.Count > 0 ? results[0] : default);
        return results.Count;
    }

    private bool SweepPreparedConvexSource(
        LSCollider source,
        Vector3d displacement,
        PhysicsLayerMask layerMask,
        out Physics3DHit sweepHit,
        LSCollider? excludedCollider,
        bool includeTriggers)
    {
        sweepHit = default;
        if (displacement.MagnitudeSquared <= Fixed64.Epsilon)
        {
            LastQueryCandidateCount = 0;
            return false;
        }

        BeginConvexSweepTrace(source, layerMask, excludedCollider, includeTriggers);
        bool hit = TryFindClosestConvexSweepHit(source, displacement, out sweepHit);
        _context.Diagnostics.EmitRayQuery(
            source.Center,
            source.Center + displacement,
            source.ScaledRadius,
            layerMask.Bits,
            hit,
            hit ? 1 : 0,
            sweepHit);
        return hit;
    }

    private int SweepPreparedConvexSourceAll(
        LSCollider source,
        Vector3d displacement,
        PhysicsLayerMask layerMask,
        SwiftList<Physics3DHit> results,
        LSCollider? excludedCollider,
        bool includeTriggers)
    {
        results.FastClear();
        if (displacement.MagnitudeSquared <= Fixed64.Epsilon)
        {
            LastQueryCandidateCount = 0;
            return 0;
        }

        BeginConvexSweepTrace(source, layerMask, excludedCollider, includeTriggers);
        AddAllConvexSweepHits(source, displacement, results);
        Physics3DHitSorter.SortByDistance(results);
        _context.Diagnostics.EmitRayQuery(
            source.Center,
            source.Center + displacement,
            source.ScaledRadius,
            layerMask.Bits,
            results.Count > 0,
            results.Count,
            results.Count > 0 ? results[0] : default);
        return results.Count;
    }

    private void BeginConvexSweepTrace(
        LSCollider source,
        PhysicsLayerMask layerMask,
        LSCollider? excludedCollider,
        bool includeTriggers)
    {
        _currentLayerMask = layerMask;
        _currentExcludedCollider = excludedCollider;
        _currentSweepSourceCollider = source;
        _currentIncludeTriggers = includeTriggers;
        _currentStaticSweepTargetsOnly = false;
        _redundantColliderCheck.Clear();
        _redundantVoxelCheck.Clear();
        LastQueryCandidateCount = 0;
        RaycastVersion++;
    }

    private void BeginRaycastTrace(Vector3d start, Vector3d end)
    {
        _redundantColliderCheck.Clear();
        _redundantVoxelCheck.Clear();
        _bufferIntersectionPoints.FastClear();
        _currentExcludedCollider = null;
        _currentSweepSourceCollider = null;
        _currentStaticSweepTargetsOnly = false;
        _currentIncludeTriggers = true;
        LastQueryCandidateCount = 0;
        RaycastVersion++;
        _worker.PrepareSegmentCheck(start, end);
    }

    private void BeginSweepTrace(
        Vector3d start,
        Vector3d end,
        Fixed64 radius,
        PhysicsLayerMask layerMask,
        LSCollider? excludedCollider,
        bool includeTriggers = true,
        bool staticTargetsOnly = false)
    {
        _currentLayerMask = layerMask;
        _currentExcludedCollider = excludedCollider;
        _currentSweepSourceCollider = null;
        _currentIncludeTriggers = includeTriggers;
        _currentStaticSweepTargetsOnly = staticTargetsOnly;
        _redundantColliderCheck.Clear();
        _redundantVoxelCheck.Clear();
        LastQueryCandidateCount = 0;
        RaycastVersion++;
        _sweepWorker.Prepare(start, end, radius);
    }

    private bool SweepSphere(
        Vector3d start,
        Vector3d end,
        Fixed64 radius,
        PhysicsLayerMask layerMask,
        LSCollider? excludedCollider,
        out Physics3DHit sweepHit)
    {
        sweepHit = default;
        if (radius <= Fixed64.Zero)
        {
            LastQueryCandidateCount = 0;
            return false;
        }

        Vector3d segment = end - start;
        if (segment.MagnitudeSquared == Fixed64.Zero)
        {
            LastQueryCandidateCount = 0;
            return false;
        }

        Vector3d direction = segment.Normalized;
        BeginSweepTrace(start, end, radius, layerMask, excludedCollider);
        return TryFindClosestSweepHit(start, end, radius, direction, out sweepHit);
    }

    private bool TryFindClosestSweepHit(
        Vector3d start,
        Vector3d end,
        Fixed64 radius,
        Vector3d direction,
        out Physics3DHit sweepHit)
    {
        bool found = false;
        Fixed64 closestDistance = Fixed64.MaxValue;
        Physics3DHit closestHit = default;

        TraceSweepForClosestHit(start, end, radius, direction, ref found, ref closestDistance, ref closestHit);

        sweepHit = closestHit;
        return found;
    }

    private bool TryFindClosestHit(Vector3d start, Vector3d end, Vector3d direction, out Physics3DHit raycastHit)
    {
        bool found = false;
        Fixed64 closestDistance = Fixed64.MaxValue;
        Physics3DHit closestHit = default;

        TraceLineForClosestHit(start, end, direction, ref found, ref closestDistance, ref closestHit);

        raycastHit = closestHit;
        return found;
    }

    private void AddAllHits(Vector3d start, Vector3d end, Vector3d direction, SwiftList<Physics3DHit> results)
    {
        TraceLineForAllHits(start, end, direction, results);
    }

    private void AddAllSweepHits(
        Vector3d start,
        Vector3d end,
        Vector3d direction,
        Fixed64 radius,
        SwiftList<Physics3DHit> results)
    {
        TraceSweepForAllHits(start, end, radius, direction, results);
    }

    private bool TryFindClosestConvexSweepHit(
        LSCollider source,
        Vector3d displacement,
        out Physics3DHit sweepHit)
    {
        bool found = false;
        Fixed64 closestDistance = Fixed64.MaxValue;
        Physics3DHit closestHit = default;

        TraceConvexSweepForClosestHit(source, displacement, ref found, ref closestDistance, ref closestHit);

        sweepHit = closestHit;
        return found;
    }

    private void AddAllConvexSweepHits(
        LSCollider source,
        Vector3d displacement,
        SwiftList<Physics3DHit> results)
    {
        TraceConvexSweepForAllHits(source, displacement, results);
    }

    private void TraceLineForClosestHit(
        Vector3d start,
        Vector3d end,
        Vector3d direction,
        ref bool found,
        ref Fixed64 closestDistance,
        ref Physics3DHit closestHit)
    {
        GridTracer.TraceLineInto(_context.World, start, end, _coveredVoxels, _traceScratch);
        for (int i = 0; i < _coveredVoxels.Count; i++)
            ProcessTraceVoxelForClosestHit(
                _coveredVoxels[i],
                start,
                direction,
                ref found,
                ref closestDistance,
                ref closestHit);

        ProcessTraceEndVoxelForClosestHit(end, start, direction, ref found, ref closestDistance, ref closestHit);
    }

    private void TraceLineForAllHits(
        Vector3d start,
        Vector3d end,
        Vector3d direction,
        SwiftList<Physics3DHit> results)
    {
        GridTracer.TraceLineInto(_context.World, start, end, _coveredVoxels, _traceScratch);
        for (int i = 0; i < _coveredVoxels.Count; i++)
            ProcessTraceVoxelForAllHits(_coveredVoxels[i], start, direction, results);

        ProcessTraceEndVoxelForAllHits(end, start, direction, results);
    }

    private void TraceSweepForClosestHit(
        Vector3d start,
        Vector3d end,
        Fixed64 radius,
        Vector3d direction,
        ref bool found,
        ref Fixed64 closestDistance,
        ref Physics3DHit closestHit)
    {
        PrepareSweepBounds(start, end, radius, out Vector3d coverageMin, out Vector3d coverageMax);
        GridTracer.GetCoveredVoxelsInto(_context.World, coverageMin, coverageMax, _coveredVoxels, _traceScratch);
        for (int i = 0; i < _coveredVoxels.Count; i++)
            ProcessSweepVoxelForClosestHit(
                _coveredVoxels[i],
                start,
                direction,
                ref found,
                ref closestDistance,
                ref closestHit);
    }

    private void TraceSweepForAllHits(
        Vector3d start,
        Vector3d end,
        Fixed64 radius,
        Vector3d direction,
        SwiftList<Physics3DHit> results)
    {
        PrepareSweepBounds(start, end, radius, out Vector3d coverageMin, out Vector3d coverageMax);
        GridTracer.GetCoveredVoxelsInto(_context.World, coverageMin, coverageMax, _coveredVoxels, _traceScratch);
        for (int i = 0; i < _coveredVoxels.Count; i++)
            ProcessSweepVoxelForAllHits(_coveredVoxels[i], start, direction, results);
    }

    private void TraceConvexSweepForClosestHit(
        LSCollider source,
        Vector3d displacement,
        ref bool found,
        ref Fixed64 closestDistance,
        ref Physics3DHit closestHit)
    {
        PrepareConvexSweepBounds(source, displacement, out Vector3d coverageMin, out Vector3d coverageMax);
        GridTracer.GetCoveredVoxelsInto(_context.World, coverageMin, coverageMax, _coveredVoxels, _traceScratch);
        for (int i = 0; i < _coveredVoxels.Count; i++)
            ProcessConvexSweepVoxelForClosestHit(
                _coveredVoxels[i],
                ref found,
                ref closestDistance,
                ref closestHit);
    }

    private void TraceConvexSweepForAllHits(
        LSCollider source,
        Vector3d displacement,
        SwiftList<Physics3DHit> results)
    {
        PrepareConvexSweepBounds(source, displacement, out Vector3d coverageMin, out Vector3d coverageMax);
        GridTracer.GetCoveredVoxelsInto(_context.World, coverageMin, coverageMax, _coveredVoxels, _traceScratch);
        for (int i = 0; i < _coveredVoxels.Count; i++)
            ProcessConvexSweepVoxelForAllHits(_coveredVoxels[i], results);
    }

    private void PrepareSweepBounds(
        Vector3d start,
        Vector3d end,
        Fixed64 radius,
        out Vector3d coverageMin,
        out Vector3d coverageMax)
    {
        Vector3d radiusExtents = Vector3d.One * radius;
        coverageMin = Vector3d.Min(start, end) - radiusExtents;
        coverageMax = Vector3d.Max(start, end) + radiusExtents;
    }

    private static void PrepareConvexSweepBounds(
        LSCollider source,
        Vector3d displacement,
        out Vector3d coverageMin,
        out Vector3d coverageMax)
    {
        coverageMin = Vector3d.Min(source.BoundsMin, source.BoundsMin + displacement);
        coverageMax = Vector3d.Max(source.BoundsMax, source.BoundsMax + displacement);
    }

    private void ProcessTraceEndVoxelForClosestHit(
        Vector3d end,
        Vector3d origin,
        Vector3d direction,
        ref bool found,
        ref Fixed64 closestDistance,
        ref Physics3DHit closestHit)
    {
        if (_context.World.TryGetVoxel(end, out Voxel? voxel))
            ProcessTraceVoxelForClosestHit(voxel!, origin, direction, ref found, ref closestDistance, ref closestHit);
    }

    private void ProcessTraceEndVoxelForAllHits(
        Vector3d end,
        Vector3d origin,
        Vector3d direction,
        SwiftList<Physics3DHit> results)
    {
        if (_context.World.TryGetVoxel(end, out Voxel? voxel))
            ProcessTraceVoxelForAllHits(voxel!, origin, direction, results);
    }

    private void ProcessTraceVoxelForClosestHit(
        Voxel voxel,
        Vector3d origin,
        Vector3d direction,
        ref bool found,
        ref Fixed64 closestDistance,
        ref Physics3DHit closestHit)
    {
        if (!GridTraversal.TryGetUniquePartition(voxel, _redundantVoxelCheck, out PhysicsPartition? partition))
        {
            return;
        }

        ProcessPartitionForClosestHit(
            partition!,
            origin,
            direction,
            ref found,
            ref closestDistance,
            ref closestHit);
    }

    private void ProcessTraceVoxelForAllHits(
        Voxel voxel,
        Vector3d origin,
        Vector3d direction,
        SwiftList<Physics3DHit> results)
    {
        if (!GridTraversal.TryGetUniquePartition(voxel, _redundantVoxelCheck, out PhysicsPartition? partition))
        {
            return;
        }

        ProcessPartitionForAllHits(partition!, origin, direction, results);
    }

    private void ProcessSweepVoxelForClosestHit(
        Voxel voxel,
        Vector3d origin,
        Vector3d direction,
        ref bool found,
        ref Fixed64 closestDistance,
        ref Physics3DHit closestHit)
    {
        if (!GridTraversal.TryGetUniquePartition(voxel, _redundantVoxelCheck, out PhysicsPartition? partition))
        {
            return;
        }

        ProcessPartitionForClosestSweepHit(
            partition!,
            origin,
            direction,
            ref found,
            ref closestDistance,
            ref closestHit);
    }

    private void ProcessSweepVoxelForAllHits(
        Voxel voxel,
        Vector3d origin,
        Vector3d direction,
        SwiftList<Physics3DHit> results)
    {
        if (!GridTraversal.TryGetUniquePartition(voxel, _redundantVoxelCheck, out PhysicsPartition? partition))
        {
            return;
        }

        ProcessPartitionForAllSweepHits(partition!, origin, direction, results);
    }

    private void ProcessConvexSweepVoxelForClosestHit(
        Voxel voxel,
        ref bool found,
        ref Fixed64 closestDistance,
        ref Physics3DHit closestHit)
    {
        if (!GridTraversal.TryGetUniquePartition(voxel, _redundantVoxelCheck, out PhysicsPartition? partition))
        {
            return;
        }

        ProcessPartitionForClosestConvexSweepHit(
            partition!,
            ref found,
            ref closestDistance,
            ref closestHit);
    }

    private void ProcessConvexSweepVoxelForAllHits(
        Voxel voxel,
        SwiftList<Physics3DHit> results)
    {
        if (!GridTraversal.TryGetUniquePartition(voxel, _redundantVoxelCheck, out PhysicsPartition? partition))
        {
            return;
        }

        ProcessPartitionForAllConvexSweepHits(partition!, results);
    }

    private void ProcessPartitionForClosestHit(
        PhysicsPartition partition,
        Vector3d origin,
        Vector3d direction,
        ref bool found,
        ref Fixed64 closestDistance,
        ref Physics3DHit closestHit)
    {
        ProcessColliderListForClosestHit(
            partition.ContainedDynamicObjects,
            origin,
            direction,
            ref found,
            ref closestDistance,
            ref closestHit);

        ProcessColliderListForClosestHit(
            partition.ContainedKinematicObjects,
            origin,
            direction,
            ref found,
            ref closestDistance,
            ref closestHit);

        ProcessColliderListForClosestHit(
            partition.ContainedStaticObjects,
            origin,
            direction,
            ref found,
            ref closestDistance,
            ref closestHit);
    }

    private void ProcessColliderListForClosestHit(
        SwiftSparseSet? colliderIds,
        Vector3d origin,
        Vector3d direction,
        ref bool found,
        ref Fixed64 closestDistance,
        ref Physics3DHit closestHit)
    {
        if (colliderIds == null)
            return;

        for (int i = colliderIds.Count - 1; i >= 0; i--)
        {
            if (!TryBuildHitForCollider(colliderIds.DenseKeys[i], origin, direction, out Physics3DHit hit)
                || !ShouldReplaceClosestHit(hit, found, closestHit))
            {
                continue;
            }

            found = true;
            closestDistance = hit.Distance;
            closestHit = hit;
        }
    }

    private void ProcessPartitionForAllHits(
        PhysicsPartition partition,
        Vector3d origin,
        Vector3d direction,
        SwiftList<Physics3DHit> results)
    {
        ProcessColliderListForAllHits(partition.ContainedDynamicObjects, origin, direction, results);
        ProcessColliderListForAllHits(partition.ContainedKinematicObjects, origin, direction, results);
        ProcessColliderListForAllHits(partition.ContainedStaticObjects, origin, direction, results);
    }

    private void ProcessColliderListForAllHits(
        SwiftSparseSet? colliderIds,
        Vector3d origin,
        Vector3d direction,
        SwiftList<Physics3DHit> results)
    {
        if (colliderIds == null)
            return;

        for (int i = colliderIds.Count - 1; i >= 0; i--)
        {
            if (TryBuildHitForCollider(colliderIds.DenseKeys[i], origin, direction, out Physics3DHit hit))
                results.Add(hit);
        }
    }

    private void ProcessPartitionForClosestSweepHit(
        PhysicsPartition partition,
        Vector3d origin,
        Vector3d direction,
        ref bool found,
        ref Fixed64 closestDistance,
        ref Physics3DHit closestHit)
    {
        if (!_currentStaticSweepTargetsOnly)
        {
            ProcessColliderListForClosestSweepHit(
                partition.ContainedDynamicObjects,
                origin,
                direction,
                ref found,
                ref closestDistance,
                ref closestHit);
        }

        ProcessColliderListForClosestSweepHit(
            partition.ContainedKinematicObjects,
            origin,
            direction,
            ref found,
            ref closestDistance,
            ref closestHit);

        ProcessColliderListForClosestSweepHit(
            partition.ContainedStaticObjects,
            origin,
            direction,
            ref found,
            ref closestDistance,
            ref closestHit);
    }

    private void ProcessColliderListForClosestSweepHit(
        SwiftSparseSet? colliderIds,
        Vector3d origin,
        Vector3d direction,
        ref bool found,
        ref Fixed64 closestDistance,
        ref Physics3DHit closestHit)
    {
        if (colliderIds == null)
            return;

        for (int i = colliderIds.Count - 1; i >= 0; i--)
        {
            if (!TryBuildSweepHitForCollider(colliderIds.DenseKeys[i], origin, direction, out Physics3DHit hit)
                || !ShouldReplaceClosestHit(hit, found, closestHit))
            {
                continue;
            }

            found = true;
            closestDistance = hit.Distance;
            closestHit = hit;
        }
    }

    private static bool ShouldReplaceClosestHit(Physics3DHit hit, bool found, Physics3DHit closestHit) =>
        !found || Physics3DHitSorter.ComesBefore(hit, closestHit);

    private void ProcessPartitionForAllSweepHits(
        PhysicsPartition partition,
        Vector3d origin,
        Vector3d direction,
        SwiftList<Physics3DHit> results)
    {
        if (!_currentStaticSweepTargetsOnly)
            ProcessColliderListForAllSweepHits(partition.ContainedDynamicObjects, origin, direction, results);

        ProcessColliderListForAllSweepHits(partition.ContainedKinematicObjects, origin, direction, results);
        ProcessColliderListForAllSweepHits(partition.ContainedStaticObjects, origin, direction, results);
    }

    private void ProcessPartitionForClosestConvexSweepHit(
        PhysicsPartition partition,
        ref bool found,
        ref Fixed64 closestDistance,
        ref Physics3DHit closestHit)
    {
        ProcessColliderListForClosestConvexSweepHit(
            partition.ContainedDynamicObjects,
            ref found,
            ref closestDistance,
            ref closestHit);

        ProcessColliderListForClosestConvexSweepHit(
            partition.ContainedKinematicObjects,
            ref found,
            ref closestDistance,
            ref closestHit);

        ProcessColliderListForClosestConvexSweepHit(
            partition.ContainedStaticObjects,
            ref found,
            ref closestDistance,
            ref closestHit);
    }

    private void ProcessPartitionForAllConvexSweepHits(
        PhysicsPartition partition,
        SwiftList<Physics3DHit> results)
    {
        ProcessColliderListForAllConvexSweepHits(partition.ContainedDynamicObjects, results);
        ProcessColliderListForAllConvexSweepHits(partition.ContainedKinematicObjects, results);
        ProcessColliderListForAllConvexSweepHits(partition.ContainedStaticObjects, results);
    }

    private void ProcessColliderListForAllSweepHits(
        SwiftSparseSet? colliderIds,
        Vector3d origin,
        Vector3d direction,
        SwiftList<Physics3DHit> results)
    {
        if (colliderIds == null)
            return;

        for (int i = colliderIds.Count - 1; i >= 0; i--)
        {
            if (TryBuildSweepHitForCollider(colliderIds.DenseKeys[i], origin, direction, out Physics3DHit hit))
                results.Add(hit);
        }
    }

    private void ProcessColliderListForClosestConvexSweepHit(
        SwiftSparseSet? colliderIds,
        ref bool found,
        ref Fixed64 closestDistance,
        ref Physics3DHit closestHit)
    {
        if (colliderIds == null)
            return;

        for (int i = colliderIds.Count - 1; i >= 0; i--)
        {
            if (!TryBuildConvexSweepHitForCollider(colliderIds.DenseKeys[i], out Physics3DHit hit)
                || !ShouldReplaceClosestHit(hit, found, closestHit))
            {
                continue;
            }

            found = true;
            closestDistance = hit.Distance;
            closestHit = hit;
        }
    }

    private void ProcessColliderListForAllConvexSweepHits(
        SwiftSparseSet? colliderIds,
        SwiftList<Physics3DHit> results)
    {
        if (colliderIds == null)
            return;

        for (int i = colliderIds.Count - 1; i >= 0; i--)
        {
            if (TryBuildConvexSweepHitForCollider(colliderIds.DenseKeys[i], out Physics3DHit hit))
                results.Add(hit);
        }
    }

    private bool TryBuildHitForCollider(
        int colliderId,
        Vector3d origin,
        Vector3d direction,
        out Physics3DHit hit)
    {
        hit = default;
        return _context.Physics.TryGetColliderById(colliderId, out LSCollider? current)
            && DoesCurrentColliderIntersectRay(current)
            && TryBuildHit(current!, origin, direction, out hit);
    }

    private bool TryBuildSweepHitForCollider(
        int colliderId,
        Vector3d origin,
        Vector3d direction,
        out Physics3DHit hit)
    {
        hit = default;
        return _context.Physics.TryGetColliderById(colliderId, out LSCollider? current)
            && IsSweepCandidate(current)
            && TryBuildSweepHit(current!, origin, direction, out hit);
    }

    private bool TryBuildConvexSweepHitForCollider(int colliderId, out Physics3DHit hit)
    {
        hit = default;
        return _context.Physics.TryGetColliderById(colliderId, out LSCollider? current)
            && IsSweepCandidate(current)
            && _convexSweepWorker.TrySweepPreparedSource(current!, out hit);
    }

    private bool TryBuildHit(
        LSCollider collider,
        Vector3d origin,
        Vector3d direction,
        out Physics3DHit raycastHit)
    {
        Fixed64 closestDistance = Fixed64.MaxValue;
        Vector3d closestIntersection = Vector3d.Zero;

        for (int i = _bufferIntersectionPoints.Count - 1; i >= 0; i--)
        {
            Fixed64 dist = Vector3d.Distance(_bufferIntersectionPoints[i], origin);
            if (dist < closestDistance)
            {
                closestDistance = dist;
                closestIntersection = _bufferIntersectionPoints[i];
            }
        }

        if (closestDistance == Fixed64.MaxValue)
        {
            raycastHit = default;
            return false;
        }

        Vector3d normal = collider.GetNormalAtPoint(closestIntersection);
        raycastHit = new Physics3DHit(collider, closestIntersection, normal, closestDistance, direction);
        return true;
    }

    private bool TryBuildSweepHit(
        LSCollider collider,
        Vector3d origin,
        Vector3d direction,
        out Physics3DHit sweepHit)
    {
        sweepHit = default;
        if (!_sweepWorker.TrySweep(collider, out Vector3d sweepCenter, out Fixed64 distance))
            return false;

        Vector3d point = GetSweepSurfacePoint(collider, sweepCenter, direction);
        Vector3d normal = ResolveSweepNormal(collider, point, sweepCenter, direction);
        sweepHit = new Physics3DHit(collider, point, normal, distance, direction);
        return true;
    }

    private bool DoesCurrentColliderIntersectRay(LSCollider? current)
    {
        if (current == null)
            return false;

        if (!_currentLayerMask.Includes(current.Layer)
            || current.RaycastVersion == RaycastVersion
            || !_redundantColliderCheck.Add(current.Id))
        {
            return false;
        }

        current.RaycastVersion = RaycastVersion;
        _bufferIntersectionPoints.FastClear();
        return current.ColliderOverlapsRay(_worker, ref _bufferIntersectionPoints);
    }

    private bool IsSweepCandidate(LSCollider? current)
    {
        if (current == null)
            return false;

        if (ReferenceEquals(current, _currentExcludedCollider)
            || ReferenceEquals(current, _currentSweepSourceCollider)
            || !_currentLayerMask.Includes(current.Layer)
            || current.RaycastVersion == RaycastVersion
            || !_redundantColliderCheck.Add(current.Id))
        {
            return false;
        }

        if (!_currentIncludeTriggers && current.IsTrigger)
            return false;

        if (_currentStaticSweepTargetsOnly && !IsStaticStyleSweepTarget(current))
            return false;

        current.RaycastVersion = RaycastVersion;
        LastQueryCandidateCount++;
        return true;
    }

    private void EnsureSourceBelongsToContext(LSCollider source)
    {
        SwiftThrowHelper.ThrowIfArgument(
            !ReferenceEquals(source.Context, _context),
            nameof(source),
            "Sweep source collider must belong to this query service context.");
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static bool IsStaticStyleSweepTarget(LSCollider collider)
    {
        StiffBody? body = collider.Body;
        return body == null || body.Immovable || body.IsKinematic;
    }

    private static Vector3d GetSweepSurfacePoint(LSCollider collider, Vector3d sweepCenter, Vector3d direction)
    {
        Vector3d centerDelta = sweepCenter - collider.Center;
        if (centerDelta.MagnitudeSquared <= Fixed64.Epsilon)
            return collider.Center - direction * collider.ScaledRadius;

        return collider.ClosestPointOnSurface(sweepCenter);
    }

    private static Vector3d ResolveSweepNormal(
        LSCollider collider,
        Vector3d point,
        Vector3d sweepCenter,
        Vector3d direction)
    {
        Vector3d fromPointToSweepCenter = sweepCenter - point;
        if ((collider is LSCuboidCollider || collider is LSCylinderCollider)
            && fromPointToSweepCenter.MagnitudeSquared > Fixed64.Epsilon)
        {
            return fromPointToSweepCenter.Normalized;
        }

        Vector3d normal = collider.GetNormalAtPoint(point);
        if (normal.MagnitudeSquared > Fixed64.Epsilon)
        {
            normal = normal.Normalized;
            if (collider is LSMeshCollider && Vector3d.Dot(normal, direction) > Fixed64.Zero)
                return -normal;

            return normal;
        }

        if (fromPointToSweepCenter.MagnitudeSquared > Fixed64.Epsilon)
            return -fromPointToSweepCenter.Normalized;

        return direction.MagnitudeSquared > Fixed64.Epsilon ? -direction.Normalized : Vector3d.Zero;
    }

}
