using FixedMathSharp;
using Gravitas.Colliders;
using Gravitas.Support;
using GridForge.Grids;
using SwiftCollections;

namespace Gravitas.Raycasting;

/// <summary>
/// Owns X/Z circle overlap query buffers and duplicate suppression for one <see cref="GravitasWorldContext"/>.
/// </summary>
public sealed class GravitasCircleQueryService
{
    private readonly GravitasWorldContext _context;
    private readonly SwiftHashSet<int> _redundantColliderCheck = new();

    private PhysicsLayerMask _currentLayerMask;

    /// <summary>
    /// Initializes a new circle query service for the supplied context.
    /// </summary>
    /// <param name="context">The owning world context.</param>
    public GravitasCircleQueryService(GravitasWorldContext context)
    {
        SwiftThrowHelper.ThrowIfNull(context, nameof(context));
        _context = context;
    }

    /// <summary>
    /// Gets the owning world context.
    /// </summary>
    public GravitasWorldContext Context => _context;

    /// <summary>
    /// Gets the context-local circle query version.
    /// </summary>
    public uint Version { get; private set; }

    /// <summary>
    /// Resets context-local circle query duplicate state.
    /// </summary>
    public void Reset()
    {
        Version = 0;
        _redundantColliderCheck.Clear();
    }

    /// <summary>
    /// Finds the closest collider whose surface is inside the supplied X/Z circle radius.
    /// </summary>
    public bool OverlapCircle(
        Vector3d position,
        Fixed64 radius,
        out LSRaycastHit raycastHit,
        PhysicsLayerMask layerMask)
    {
        _currentLayerMask = layerMask;
        Version++;
        _redundantColliderCheck.Clear();

        LSRaycastHit closestHit = default;
        Fixed64 closestDist = Fixed64.MAX_VALUE;
        bool found = false;

        TraceCircleForClosestHit(position, radius, ref found, ref closestHit, ref closestDist);

        raycastHit = closestHit;
        _context.Diagnostics.EmitCircleQuery(
            position,
            radius,
            Vector3d.Zero,
            Fixed64.Zero,
            layerMask.Bits,
            found,
            found ? 1 : 0,
            closestHit);

        return found;
    }

    /// <summary>
    /// Finds the closest circle-overlap hit whose hit point lies in the supplied direction and distance.
    /// </summary>
    public bool OverlapCircleInDirection(
        Vector3d position,
        Fixed64 radius,
        Vector3d direction,
        out LSRaycastHit raycastHit,
        Fixed64 maxDistance,
        PhysicsLayerMask layerMask)
    {
        _currentLayerMask = layerMask;
        Version++;
        _redundantColliderCheck.Clear();

        Vector3d normalizedDirection = direction.SqrMagnitude == Fixed64.Zero ? Vector3d.Zero : direction.Normal;
        Fixed64 maxDistanceSqr = maxDistance * maxDistance;
        LSRaycastHit closestHit = default;
        Fixed64 closestDist = Fixed64.MAX_VALUE;
        bool found = false;

        TraceCircleForDirectionalHit(
            position,
            radius,
            normalizedDirection,
            maxDistanceSqr,
            ref found,
            ref closestHit,
            ref closestDist);

        raycastHit = closestHit;
        _context.Diagnostics.EmitCircleQuery(
            position,
            radius,
            normalizedDirection,
            maxDistance,
            layerMask.Bits,
            found,
            found ? 1 : 0,
            closestHit);

        return found;
    }

    /// <summary>
    /// Finds all colliders whose surfaces are inside the supplied X/Z circle radius.
    /// </summary>
    public int OverlapCircleAll(
        Vector3d position,
        Fixed64 radius,
        PhysicsLayerMask layerMask,
        SwiftList<LSRaycastHit> results)
    {
        SwiftThrowHelper.ThrowIfNull(results, nameof(results));

        _currentLayerMask = layerMask;
        Version++;

        results.FastClear();
        _redundantColliderCheck.Clear();

        TraceCircleForAllHits(position, radius, results);

        RaycastHitSorter.SortByDistance(results);
        LSRaycastHit firstHit = results.Count > 0 ? results[0] : default;
        _context.Diagnostics.EmitCircleQuery(
            position,
            radius,
            Vector3d.Zero,
            Fixed64.Zero,
            layerMask.Bits,
            results.Count > 0,
            results.Count,
            firstHit);

        return results.Count;
    }

    private void TraceCircleForClosestHit(
        Vector3d position,
        Fixed64 radius,
        ref bool found,
        ref LSRaycastHit closestHit,
        ref Fixed64 closestDist)
    {
        Fixed64 xMin = position.x - radius;
        Fixed64 xMax = position.x + radius;
        Fixed64 y = position.y;
        Fixed64 zMin = position.z - radius;
        Fixed64 zMax = position.z + radius;

        for (Fixed64 x = xMin; x <= xMax; x += _context.World.VoxelSize)
        {
            for (Fixed64 z = zMin; z <= zMax; z += _context.World.VoxelSize)
            {
                Vector3d castPosition = new(x, y, z);
                if (!_context.World.TryGetVoxel(castPosition, out Voxel? voxel)
                    || voxel!.TryGetPartition(out PhysicsPartition? partition) == false)
                {
                    continue;
                }

                ProcessPartitionForClosestHit(partition!, position, radius, ref found, ref closestHit, ref closestDist);
            }
        }
    }

    private void TraceCircleForDirectionalHit(
        Vector3d position,
        Fixed64 radius,
        Vector3d direction,
        Fixed64 maxDistanceSqr,
        ref bool found,
        ref LSRaycastHit closestHit,
        ref Fixed64 closestDist)
    {
        Fixed64 xMin = position.x - radius;
        Fixed64 xMax = position.x + radius;
        Fixed64 y = position.y;
        Fixed64 zMin = position.z - radius;
        Fixed64 zMax = position.z + radius;

        for (Fixed64 x = xMin; x <= xMax; x += _context.World.VoxelSize)
        {
            for (Fixed64 z = zMin; z <= zMax; z += _context.World.VoxelSize)
            {
                Vector3d castPosition = new(x, y, z);
                if (!_context.World.TryGetVoxel(castPosition, out Voxel? voxel)
                    || voxel!.TryGetPartition(out PhysicsPartition? partition) == false)
                {
                    continue;
                }

                ProcessPartitionForDirectionalHit(
                    partition!,
                    position,
                    radius,
                    direction,
                    maxDistanceSqr,
                    ref found,
                    ref closestHit,
                    ref closestDist);
            }
        }
    }

    private void TraceCircleForAllHits(Vector3d position, Fixed64 radius, SwiftList<LSRaycastHit> results)
    {
        Fixed64 xMin = position.x - radius;
        Fixed64 xMax = position.x + radius;
        Fixed64 y = position.y;
        Fixed64 zMin = position.z - radius;
        Fixed64 zMax = position.z + radius;

        for (Fixed64 x = xMin; x <= xMax; x += _context.World.VoxelSize)
        {
            for (Fixed64 z = zMin; z <= zMax; z += _context.World.VoxelSize)
            {
                Vector3d castPosition = new(x, y, z);
                if (!_context.World.TryGetVoxel(castPosition, out Voxel? voxel)
                    || voxel!.TryGetPartition(out PhysicsPartition? partition) == false)
                {
                    continue;
                }

                ProcessPartitionForAllHits(partition!, position, radius, results);
            }
        }
    }

    private void ProcessPartitionForClosestHit(
        PhysicsPartition partition,
        Vector3d position,
        Fixed64 radius,
        ref bool found,
        ref LSRaycastHit closestHit,
        ref Fixed64 closestDist)
    {
        ProcessColliderListForClosestHit(
            partition.ContainedDynamicObjects,
            position,
            radius,
            ref found,
            ref closestHit,
            ref closestDist);

        ProcessColliderListForClosestHit(
            partition.ContainedStaticObjects,
            position,
            radius,
            ref found,
            ref closestHit,
            ref closestDist);
    }

    private void ProcessColliderListForClosestHit(
        SwiftSparseMap<byte>? colliderIds,
        Vector3d position,
        Fixed64 radius,
        ref bool found,
        ref LSRaycastHit closestHit,
        ref Fixed64 closestDist)
    {
        if (colliderIds == null)
            return;

        for (int i = colliderIds.Count - 1; i >= 0; i--)
        {
            if (!TryBuildOverlapHit(colliderIds.DenseKeys[i], position, radius, out LSRaycastHit hitInfo)
                || hitInfo.Distance >= closestDist)
            {
                continue;
            }

            found = true;
            closestHit = hitInfo;
            closestDist = hitInfo.Distance;
        }
    }

    private void ProcessPartitionForDirectionalHit(
        PhysicsPartition partition,
        Vector3d position,
        Fixed64 radius,
        Vector3d direction,
        Fixed64 maxDistanceSqr,
        ref bool found,
        ref LSRaycastHit closestHit,
        ref Fixed64 closestDist)
    {
        ProcessColliderListForDirectionalHit(partition.ContainedDynamicObjects, position, radius, direction, maxDistanceSqr, ref found, ref closestHit, ref closestDist);
        ProcessColliderListForDirectionalHit(partition.ContainedStaticObjects, position, radius, direction, maxDistanceSqr, ref found, ref closestHit, ref closestDist);
    }

    private void ProcessColliderListForDirectionalHit(
        SwiftSparseMap<byte>? colliderIds,
        Vector3d position,
        Fixed64 radius,
        Vector3d direction,
        Fixed64 maxDistanceSqr,
        ref bool found,
        ref LSRaycastHit closestHit,
        ref Fixed64 closestDist)
    {
        if (colliderIds == null || direction.SqrMagnitude == Fixed64.Zero)
            return;

        for (int i = colliderIds.Count - 1; i >= 0; i--)
        {
            if (!TryBuildOverlapHit(colliderIds.DenseKeys[i], position, radius, out LSRaycastHit hitInfo))
                continue;

            Vector3d toHit = hitInfo.Point - position;
            if (toHit.SqrMagnitude > maxDistanceSqr
                || Vector3d.Dot(toHit.Normal, direction) <= Fixed64.Zero
                || hitInfo.Distance >= closestDist)
            {
                continue;
            }

            found = true;
            closestHit = hitInfo;
            closestDist = hitInfo.Distance;
        }
    }

    private void ProcessPartitionForAllHits(
        PhysicsPartition partition,
        Vector3d position,
        Fixed64 radius,
        SwiftList<LSRaycastHit> results)
    {
        ProcessColliderListForAllHits(partition.ContainedDynamicObjects, position, radius, results);
        ProcessColliderListForAllHits(partition.ContainedStaticObjects, position, radius, results);
    }

    private void ProcessColliderListForAllHits(
        SwiftSparseMap<byte>? colliderIds,
        Vector3d position,
        Fixed64 radius,
        SwiftList<LSRaycastHit> results)
    {
        if (colliderIds == null)
            return;

        for (int i = colliderIds.Count - 1; i >= 0; i--)
        {
            if (TryBuildOverlapHit(colliderIds.DenseKeys[i], position, radius, out LSRaycastHit hitInfo))
                results.Add(hitInfo);
        }
    }

    private bool TryBuildOverlapHit(int colliderId, Vector3d position, Fixed64 radius, out LSRaycastHit raycastHit)
    {
        raycastHit = default;
        if (!_context.Physics.TryGetColliderById(colliderId, out LSCollider? current))
            return false;

        LSCollider collider = current!;
        if (!_currentLayerMask.Includes(collider.Layer)
            || collider.CircleQueryVersion == Version
            || !_redundantColliderCheck.Add(collider.Id))
        {
            return false;
        }

        collider.CircleQueryVersion = Version;
        Fixed64 broadDistance = collider.ScaledRadius + radius;
        if ((collider.Center - position).SqrMagnitude > broadDistance * broadDistance)
            return false;

        Vector3d point = GetClosestSurfacePoint(collider, position);
        Vector3d toPoint = point - position;
        Fixed64 distance = toPoint.Magnitude;
        if (distance > radius)
            return false;

        Vector3d normal = collider.GetNormalAtPoint(point);
        raycastHit = new LSRaycastHit(collider, point, normal, distance, toPoint);
        return true;
    }

    private static Vector3d GetClosestSurfacePoint(LSCollider collider, Vector3d position)
    {
        if ((position - collider.Center).SqrMagnitude == Fixed64.Zero)
            return collider.Center + Vector3d.Right * collider.ScaledRadius;

        return collider.ClosestPointOnSurface(position);
    }
}
