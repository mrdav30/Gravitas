using FixedMathSharp;
using Gravitas.Colliders;
using Gravitas.Support;
using GridForge.Grids;
using SwiftCollections;

namespace Gravitas.Raycasting;

/// <summary>
/// Owns raycast query buffers and worker state for one <see cref="GravitasWorldContext"/>.
/// </summary>
public sealed class GravitasRaycastService
{
    private readonly GravitasWorldContext _context;
    private readonly RaycastSegmentWorker _worker = new();
    private readonly SweptSphereQueryWorker _sweepWorker = new();
    private SwiftList<Vector3d> _bufferIntersectionPoints = new();
    private readonly SwiftHashSet<int> _redundantColliderCheck = new();
    private readonly SwiftHashSet<int> _redundantVoxelCheck = new();

    private PhysicsLayerMask _currentLayerMask;
    private LSCollider? _currentExcludedCollider;

    /// <summary>
    /// Initializes a new raycast service for the supplied context.
    /// </summary>
    /// <param name="context">The owning world context.</param>
    public GravitasRaycastService(GravitasWorldContext context)
    {
        SwiftThrowHelper.ThrowIfNull(context, nameof(context));
        _context = context;
    }

    /// <summary>
    /// Gets the owning world context.
    /// </summary>
    public GravitasWorldContext Context => _context;

    /// <summary>
    /// Gets the context-local raycast query version.
    /// </summary>
    public uint Version { get; private set; }

    /// <summary>
    /// Resets context-local raycast query buffers.
    /// </summary>
    public void Reset()
    {
        Version = 0;
        _bufferIntersectionPoints.FastClear();
        _redundantColliderCheck.Clear();
        _redundantVoxelCheck.Clear();
    }

    /// <summary>
    /// Performs a raycast from an origin in a direction up to a maximum distance.
    /// </summary>
    public bool Raycast(
        Vector3d origin,
        Vector3d direction,
        Fixed64 maxDistance,
        out LSRaycastHit raycastHit,
        PhysicsLayerMask layerMask)
    {
        _currentLayerMask = layerMask;
        if (direction.SqrMagnitude == Fixed64.Zero || maxDistance <= Fixed64.Zero)
        {
            raycastHit = default;
            return false;
        }

        Vector3d rayDirection = direction.Normal;
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
        SwiftList<LSRaycastHit> results)
    {
        SwiftThrowHelper.ThrowIfNull(results, nameof(results));

        _currentLayerMask = layerMask;
        results.FastClear();

        Vector3d segment = end3d - start3d;
        if (segment.SqrMagnitude == Fixed64.Zero)
            return 0;

        BeginRaycastTrace(start3d, end3d);
        AddAllHits(start3d, end3d, segment.Normal, results);
        RaycastHitSorter.SortByDistance(results);
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
        out LSRaycastHit sweepHit,
        PhysicsLayerMask layerMask,
        LSCollider? excludedCollider = null)
    {
        if (direction.SqrMagnitude == Fixed64.Zero || maxDistance <= Fixed64.Zero)
        {
            sweepHit = default;
            return false;
        }

        Vector3d sweepDirection = direction.Normal;
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
        SwiftList<LSRaycastHit> results,
        LSCollider? excludedCollider = null)
    {
        SwiftThrowHelper.ThrowIfNull(results, nameof(results));

        results.FastClear();
        Vector3d segment = end3d - start3d;
        if (segment.SqrMagnitude == Fixed64.Zero || radius <= Fixed64.Zero)
            return 0;

        BeginSweepTrace(start3d, end3d, radius, layerMask, excludedCollider);
        AddAllSweepHits(start3d, end3d, segment.Normal, radius, results);
        RaycastHitSorter.SortByDistance(results);
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

    private void BeginRaycastTrace(Vector3d start, Vector3d end)
    {
        _redundantColliderCheck.Clear();
        _redundantVoxelCheck.Clear();
        _bufferIntersectionPoints.FastClear();
        _currentExcludedCollider = null;
        Version++;
        _worker.PrepareSegmentCheck(start, end);
    }

    private void BeginSweepTrace(
        Vector3d start,
        Vector3d end,
        Fixed64 radius,
        PhysicsLayerMask layerMask,
        LSCollider? excludedCollider)
    {
        _currentLayerMask = layerMask;
        _currentExcludedCollider = excludedCollider;
        _redundantColliderCheck.Clear();
        _redundantVoxelCheck.Clear();
        Version++;
        _sweepWorker.Prepare(start, end, radius);
    }

    private bool SweepSphere(
        Vector3d start,
        Vector3d end,
        Fixed64 radius,
        PhysicsLayerMask layerMask,
        LSCollider? excludedCollider,
        out LSRaycastHit sweepHit)
    {
        sweepHit = default;
        if (radius <= Fixed64.Zero)
            return false;

        Vector3d segment = end - start;
        if (segment.SqrMagnitude == Fixed64.Zero)
            return false;

        Vector3d direction = segment.Normal;
        BeginSweepTrace(start, end, radius, layerMask, excludedCollider);
        return TryFindClosestSweepHit(start, end, radius, direction, out sweepHit);
    }

    private bool TryFindClosestSweepHit(
        Vector3d start,
        Vector3d end,
        Fixed64 radius,
        Vector3d direction,
        out LSRaycastHit sweepHit)
    {
        bool found = false;
        Fixed64 closestDistance = Fixed64.MAX_VALUE;
        LSRaycastHit closestHit = default;

        TraceSweepForClosestHit(start, end, radius, direction, ref found, ref closestDistance, ref closestHit);

        sweepHit = closestHit;
        return found;
    }

    private bool TryFindClosestHit(Vector3d start, Vector3d end, Vector3d direction, out LSRaycastHit raycastHit)
    {
        bool found = false;
        Fixed64 closestDistance = Fixed64.MAX_VALUE;
        LSRaycastHit closestHit = default;

        TraceLineForClosestHit(start, end, direction, ref found, ref closestDistance, ref closestHit);

        raycastHit = closestHit;
        return found;
    }

    private void AddAllHits(Vector3d start, Vector3d end, Vector3d direction, SwiftList<LSRaycastHit> results)
    {
        TraceLineForAllHits(start, end, direction, results);
    }

    private void AddAllSweepHits(
        Vector3d start,
        Vector3d end,
        Vector3d direction,
        Fixed64 radius,
        SwiftList<LSRaycastHit> results)
    {
        TraceSweepForAllHits(start, end, radius, direction, results);
    }

    private void TraceLineForClosestHit(
        Vector3d start,
        Vector3d end,
        Vector3d direction,
        ref bool found,
        ref Fixed64 closestDistance,
        ref LSRaycastHit closestHit)
    {
        PrepareTraceLine(start, end, out Vector3d traceStart, out Vector3d step, out Fixed64 steps);

        for (Fixed64 i = Fixed64.Zero; i <= steps; i += Fixed64.One)
        {
            ProcessTracePositionForClosestHit(
                _context.World.FloorToVoxelSize(traceStart + step * i),
                start,
                direction,
                ref found,
                ref closestDistance,
                ref closestHit);
        }

        ProcessTracePositionForClosestHit(end, start, direction, ref found, ref closestDistance, ref closestHit);
    }

    private void TraceLineForAllHits(
        Vector3d start,
        Vector3d end,
        Vector3d direction,
        SwiftList<LSRaycastHit> results)
    {
        PrepareTraceLine(start, end, out Vector3d traceStart, out Vector3d step, out Fixed64 steps);

        for (Fixed64 i = Fixed64.Zero; i <= steps; i += Fixed64.One)
        {
            ProcessTracePositionForAllHits(
                _context.World.FloorToVoxelSize(traceStart + step * i),
                start,
                direction,
                results);
        }

        ProcessTracePositionForAllHits(end, start, direction, results);
    }

    private void TraceSweepForClosestHit(
        Vector3d start,
        Vector3d end,
        Fixed64 radius,
        Vector3d direction,
        ref bool found,
        ref Fixed64 closestDistance,
        ref LSRaycastHit closestHit)
    {
        PrepareSweepBounds(start, end, radius, out Vector3d snappedMin, out Vector3d snappedMax);
        GridWorld world = _context.World;
        Fixed64 step = world.VoxelSize;
        for (Fixed64 x = snappedMin.x; x <= snappedMax.x; x += step)
            for (Fixed64 y = snappedMin.y; y <= snappedMax.y; y += step)
                for (Fixed64 z = snappedMin.z; z <= snappedMax.z; z += step)
                    ProcessSweepPositionForClosestHit(
                        new Vector3d(x, y, z),
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
        SwiftList<LSRaycastHit> results)
    {
        PrepareSweepBounds(start, end, radius, out Vector3d snappedMin, out Vector3d snappedMax);
        GridWorld world = _context.World;
        Fixed64 step = world.VoxelSize;
        for (Fixed64 x = snappedMin.x; x <= snappedMax.x; x += step)
            for (Fixed64 y = snappedMin.y; y <= snappedMax.y; y += step)
                for (Fixed64 z = snappedMin.z; z <= snappedMax.z; z += step)
                    ProcessSweepPositionForAllHits(
                        new Vector3d(x, y, z),
                        start,
                        direction,
                        results);
    }

    private void PrepareSweepBounds(
        Vector3d start,
        Vector3d end,
        Fixed64 radius,
        out Vector3d snappedMin,
        out Vector3d snappedMax)
    {
        Vector3d radiusExtents = Vector3d.One * radius;
        Vector3d min = Vector3d.Min(start, end) - radiusExtents;
        Vector3d max = Vector3d.Max(start, end) + radiusExtents;
        (snappedMin, snappedMax) = _context.World.SnapBoundsToVoxelSize(min, max);
    }

    private void PrepareTraceLine(Vector3d start, Vector3d end, out Vector3d traceStart, out Vector3d step, out Fixed64 steps)
    {
        GridWorld world = _context.World;
        (Vector3d snappedMin, Vector3d snappedMax) = world.SnapBoundsToVoxelSize(start, end);
        traceStart = CreateTraceEndpoint(start, end, snappedMin, snappedMax, useMinWhenIncreasing: true);
        Vector3d traceEnd = CreateTraceEndpoint(start, end, snappedMin, snappedMax, useMinWhenIncreasing: false);

        Vector3d diff = traceEnd - traceStart;
        Vector3d delta = Vector3d.Abs(diff);
        Fixed64 maxDelta = FixedMath.Max(FixedMath.Max(delta.x, delta.y), delta.z);
        steps = FixedMath.Ceiling(maxDelta / world.VoxelSize);
        step = diff / (steps + Fixed64.One);
    }

    private void ProcessTracePositionForClosestHit(
        Vector3d tracePosition,
        Vector3d origin,
        Vector3d direction,
        ref bool found,
        ref Fixed64 closestDistance,
        ref LSRaycastHit closestHit)
    {
        GridWorld world = _context.World;
        int cellIndex = world.GetSpatialGridKey(tracePosition);
        if (!world.SpatialGridHash.TryGetValue(cellIndex, out SwiftHashSet<ushort> gridList))
            return;

        foreach (ushort gridIndex in gridList)
        {
            if (!world.ActiveGrids.IsAllocated(gridIndex))
                continue;

            VoxelGrid currentGrid = world.ActiveGrids[gridIndex];
            if (!currentGrid.TryGetVoxel(tracePosition, out Voxel? voxel)
                || !_redundantVoxelCheck.Add(voxel!.SpawnToken)
                || !voxel.TryGetPartition(out PhysicsPartition? partition))
            {
                continue;
            }

            ProcessPartitionForClosestHit(
                partition!,
                origin,
                direction,
                ref found,
                ref closestDistance,
                ref closestHit);
        }
    }

    private void ProcessTracePositionForAllHits(
        Vector3d tracePosition,
        Vector3d origin,
        Vector3d direction,
        SwiftList<LSRaycastHit> results)
    {
        GridWorld world = _context.World;
        int cellIndex = world.GetSpatialGridKey(tracePosition);
        if (!world.SpatialGridHash.TryGetValue(cellIndex, out SwiftHashSet<ushort> gridList))
            return;

        foreach (ushort gridIndex in gridList)
        {
            if (!world.ActiveGrids.IsAllocated(gridIndex))
                continue;

            VoxelGrid currentGrid = world.ActiveGrids[gridIndex];
            if (!currentGrid.TryGetVoxel(tracePosition, out Voxel? voxel)
                || !_redundantVoxelCheck.Add(voxel!.SpawnToken)
                || !voxel.TryGetPartition(out PhysicsPartition? partition))
            {
                continue;
            }

            ProcessPartitionForAllHits(partition!, origin, direction, results);
        }
    }

    private void ProcessSweepPositionForClosestHit(
        Vector3d tracePosition,
        Vector3d origin,
        Vector3d direction,
        ref bool found,
        ref Fixed64 closestDistance,
        ref LSRaycastHit closestHit)
    {
        GridWorld world = _context.World;
        int cellIndex = world.GetSpatialGridKey(tracePosition);
        if (!world.SpatialGridHash.TryGetValue(cellIndex, out SwiftHashSet<ushort> gridList))
            return;

        foreach (ushort gridIndex in gridList)
        {
            if (!world.ActiveGrids.IsAllocated(gridIndex))
                continue;

            VoxelGrid currentGrid = world.ActiveGrids[gridIndex];
            if (!currentGrid.TryGetVoxel(tracePosition, out Voxel? voxel)
                || !_redundantVoxelCheck.Add(voxel!.SpawnToken)
                || !voxel.TryGetPartition(out PhysicsPartition? partition))
            {
                continue;
            }

            ProcessPartitionForClosestSweepHit(
                partition!,
                origin,
                direction,
                ref found,
                ref closestDistance,
                ref closestHit);
        }
    }

    private void ProcessSweepPositionForAllHits(
        Vector3d tracePosition,
        Vector3d origin,
        Vector3d direction,
        SwiftList<LSRaycastHit> results)
    {
        GridWorld world = _context.World;
        int cellIndex = world.GetSpatialGridKey(tracePosition);
        if (!world.SpatialGridHash.TryGetValue(cellIndex, out SwiftHashSet<ushort> gridList))
            return;

        foreach (ushort gridIndex in gridList)
        {
            if (!world.ActiveGrids.IsAllocated(gridIndex))
                continue;

            VoxelGrid currentGrid = world.ActiveGrids[gridIndex];
            if (!currentGrid.TryGetVoxel(tracePosition, out Voxel? voxel)
                || !_redundantVoxelCheck.Add(voxel!.SpawnToken)
                || !voxel.TryGetPartition(out PhysicsPartition? partition))
            {
                continue;
            }

            ProcessPartitionForAllSweepHits(partition!, origin, direction, results);
        }
    }

    private void ProcessPartitionForClosestHit(
        PhysicsPartition partition,
        Vector3d origin,
        Vector3d direction,
        ref bool found,
        ref Fixed64 closestDistance,
        ref LSRaycastHit closestHit)
    {
        ProcessColliderListForClosestHit(
            partition.ContainedDynamicObjects,
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
        SwiftSparseMap<byte>? colliderIds,
        Vector3d origin,
        Vector3d direction,
        ref bool found,
        ref Fixed64 closestDistance,
        ref LSRaycastHit closestHit)
    {
        if (colliderIds == null)
            return;

        for (int i = colliderIds.Count - 1; i >= 0; i--)
        {
            if (!TryBuildHitForCollider(colliderIds.DenseKeys[i], origin, direction, out LSRaycastHit hit)
                || hit.Distance >= closestDistance)
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
        SwiftList<LSRaycastHit> results)
    {
        ProcessColliderListForAllHits(partition.ContainedDynamicObjects, origin, direction, results);
        ProcessColliderListForAllHits(partition.ContainedStaticObjects, origin, direction, results);
    }

    private void ProcessColliderListForAllHits(
        SwiftSparseMap<byte>? colliderIds,
        Vector3d origin,
        Vector3d direction,
        SwiftList<LSRaycastHit> results)
    {
        if (colliderIds == null)
            return;

        for (int i = colliderIds.Count - 1; i >= 0; i--)
        {
            if (TryBuildHitForCollider(colliderIds.DenseKeys[i], origin, direction, out LSRaycastHit hit))
                results.Add(hit);
        }
    }

    private void ProcessPartitionForClosestSweepHit(
        PhysicsPartition partition,
        Vector3d origin,
        Vector3d direction,
        ref bool found,
        ref Fixed64 closestDistance,
        ref LSRaycastHit closestHit)
    {
        ProcessColliderListForClosestSweepHit(
            partition.ContainedDynamicObjects,
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
        SwiftSparseMap<byte>? colliderIds,
        Vector3d origin,
        Vector3d direction,
        ref bool found,
        ref Fixed64 closestDistance,
        ref LSRaycastHit closestHit)
    {
        if (colliderIds == null)
            return;

        for (int i = colliderIds.Count - 1; i >= 0; i--)
        {
            if (!TryBuildSweepHitForCollider(colliderIds.DenseKeys[i], origin, direction, out LSRaycastHit hit)
                || !ShouldReplaceClosestSweepHit(hit, found, closestDistance, closestHit))
            {
                continue;
            }

            found = true;
            closestDistance = hit.Distance;
            closestHit = hit;
        }
    }

    private static bool ShouldReplaceClosestSweepHit(
        LSRaycastHit hit,
        bool found,
        Fixed64 closestDistance,
        LSRaycastHit closestHit)
    {
        if (!found)
            return true;

        int distanceCompare = hit.Distance.CompareTo(closestDistance);
        if (distanceCompare != 0)
            return distanceCompare < 0;

        int hitId = hit.Collider?.Id ?? -1;
        int closestId = closestHit.Collider?.Id ?? -1;
        return hitId < closestId;
    }

    private void ProcessPartitionForAllSweepHits(
        PhysicsPartition partition,
        Vector3d origin,
        Vector3d direction,
        SwiftList<LSRaycastHit> results)
    {
        ProcessColliderListForAllSweepHits(partition.ContainedDynamicObjects, origin, direction, results);
        ProcessColliderListForAllSweepHits(partition.ContainedStaticObjects, origin, direction, results);
    }

    private void ProcessColliderListForAllSweepHits(
        SwiftSparseMap<byte>? colliderIds,
        Vector3d origin,
        Vector3d direction,
        SwiftList<LSRaycastHit> results)
    {
        if (colliderIds == null)
            return;

        for (int i = colliderIds.Count - 1; i >= 0; i--)
        {
            if (TryBuildSweepHitForCollider(colliderIds.DenseKeys[i], origin, direction, out LSRaycastHit hit))
                results.Add(hit);
        }
    }

    private bool TryBuildHitForCollider(
        int colliderId,
        Vector3d origin,
        Vector3d direction,
        out LSRaycastHit hit)
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
        out LSRaycastHit hit)
    {
        hit = default;
        return _context.Physics.TryGetColliderById(colliderId, out LSCollider? current)
            && IsSweepCandidate(current)
            && TryBuildSweepHit(current!, origin, direction, out hit);
    }

    private bool TryBuildHit(
        LSCollider collider,
        Vector3d origin,
        Vector3d direction,
        out LSRaycastHit raycastHit)
    {
        Fixed64 closestDistance = Fixed64.MAX_VALUE;
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

        if (closestDistance == Fixed64.MAX_VALUE)
        {
            raycastHit = default;
            return false;
        }

        Vector3d normal = collider.GetNormalAtPoint(closestIntersection);
        raycastHit = new LSRaycastHit(collider, closestIntersection, normal, closestDistance, direction);
        return true;
    }

    private bool TryBuildSweepHit(
        LSCollider collider,
        Vector3d origin,
        Vector3d direction,
        out LSRaycastHit sweepHit)
    {
        sweepHit = default;
        if (!_sweepWorker.TrySweep(collider, out Vector3d sweepCenter, out Fixed64 distance))
            return false;

        Vector3d point = GetSweepSurfacePoint(collider, sweepCenter, direction);
        Vector3d normal = ResolveSweepNormal(collider, point, sweepCenter, direction);
        sweepHit = new LSRaycastHit(collider, point, normal, distance, direction);
        return true;
    }

    private bool DoesCurrentColliderIntersectRay(LSCollider? current)
    {
        if (current == null)
            return false;

        if (!_currentLayerMask.Includes(current.Layer)
            || current.RaycastVersion == Version
            || !_redundantColliderCheck.Add(current.Id))
        {
            return false;
        }

        current.RaycastVersion = Version;
        _bufferIntersectionPoints.FastClear();
        return current.ColliderOverlapsRay(_worker, ref _bufferIntersectionPoints);
    }

    private bool IsSweepCandidate(LSCollider? current)
    {
        if (current == null)
            return false;

        if (ReferenceEquals(current, _currentExcludedCollider)
            || !_currentLayerMask.Includes(current.Layer)
            || current.RaycastVersion == Version
            || !_redundantColliderCheck.Add(current.Id))
        {
            return false;
        }

        current.RaycastVersion = Version;
        return true;
    }

    private static Vector3d GetSweepSurfacePoint(LSCollider collider, Vector3d sweepCenter, Vector3d direction)
    {
        Vector3d centerDelta = sweepCenter - collider.Center;
        if (centerDelta.SqrMagnitude <= Fixed64.Epsilon)
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
            && fromPointToSweepCenter.SqrMagnitude > Fixed64.Epsilon)
        {
            return fromPointToSweepCenter.Normal;
        }

        Vector3d normal = collider.GetNormalAtPoint(point);
        if (normal.SqrMagnitude > Fixed64.Epsilon)
            return normal.Normal;

        if (fromPointToSweepCenter.SqrMagnitude > Fixed64.Epsilon)
            return -fromPointToSweepCenter.Normal;

        return direction.SqrMagnitude > Fixed64.Epsilon ? -direction.Normal : Vector3d.Zero;
    }

    private static Vector3d CreateTraceEndpoint(
        Vector3d start,
        Vector3d end,
        Vector3d snappedMin,
        Vector3d snappedMax,
        bool useMinWhenIncreasing)
    {
        return new Vector3d(
            SelectTraceCoordinate(start.x, end.x, snappedMin.x, snappedMax.x, useMinWhenIncreasing),
            SelectTraceCoordinate(start.y, end.y, snappedMin.y, snappedMax.y, useMinWhenIncreasing),
            SelectTraceCoordinate(start.z, end.z, snappedMin.z, snappedMax.z, useMinWhenIncreasing));
    }

    private static Fixed64 SelectTraceCoordinate(
        Fixed64 start,
        Fixed64 end,
        Fixed64 snappedMin,
        Fixed64 snappedMax,
        bool useMinWhenIncreasing)
    {
        return (start <= end) == useMinWhenIncreasing ? snappedMin : snappedMax;
    }
}
