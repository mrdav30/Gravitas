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
    private SwiftList<Vector3d> _bufferIntersectionPoints = new();
    private readonly SwiftHashSet<int> _redundantColliderCheck = new();
    private readonly SwiftHashSet<int> _redundantVoxelCheck = new();

    private PhysicsLayerMask _currentLayerMask;

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
        return TryFindClosestHit(origin, end, rayDirection, out raycastHit);
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
        return results.Count;
    }

    private void BeginRaycastTrace(Vector3d start, Vector3d end)
    {
        _redundantColliderCheck.Clear();
        _redundantVoxelCheck.Clear();
        _bufferIntersectionPoints.FastClear();
        Version++;
        _worker.PrepareSegmentCheck(start, end);
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
        SwiftList<int>? colliderIds,
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
            if (!TryBuildHitForCollider(colliderIds[i], origin, direction, out LSRaycastHit hit)
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
        SwiftList<int>? colliderIds,
        Vector3d origin,
        Vector3d direction,
        SwiftList<LSRaycastHit> results)
    {
        if (colliderIds == null)
            return;

        for (int i = colliderIds.Count - 1; i >= 0; i--)
        {
            if (TryBuildHitForCollider(colliderIds[i], origin, direction, out LSRaycastHit hit))
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
