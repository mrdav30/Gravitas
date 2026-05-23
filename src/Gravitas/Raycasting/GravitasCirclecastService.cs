using FixedMathSharp;
using Gravitas.Colliders;
using Gravitas.Support;
using GridForge.Grids;
using SwiftCollections;

namespace Gravitas.Raycasting;

/// <summary>
/// Owns circlecast query buffers and duplicate suppression for one <see cref="GravitasWorldContext"/>.
/// </summary>
public sealed class GravitasCirclecastService
{
    private readonly GravitasWorldContext _context;
    private readonly SwiftHashSet<int> _redundantColliderCheck = new();

    private PhysicsLayerMask _currentLayerMask;

    /// <summary>
    /// Initializes a new circlecast service for the supplied context.
    /// </summary>
    /// <param name="context">The owning world context.</param>
    public GravitasCirclecastService(GravitasWorldContext context)
    {
        SwiftThrowHelper.ThrowIfNull(context, nameof(context));
        _context = context;
    }

    /// <summary>
    /// Gets the owning world context.
    /// </summary>
    public GravitasWorldContext Context => _context;

    /// <summary>
    /// Gets the context-local circlecast query version.
    /// </summary>
    public uint Version { get; private set; }

    /// <summary>
    /// Resets context-local circlecast query buffers.
    /// </summary>
    public void Reset()
    {
        Version = 0;
        _redundantColliderCheck.Clear();
    }

    /// <summary>
    /// Performs a circle cast and returns the closest hit.
    /// </summary>
    public bool CircleCast(
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

        raycastHit = closestHit;
        return found;
    }

    /// <summary>
    /// Performs a directional circle cast and returns the closest hit within the supplied distance.
    /// </summary>
    public bool CircleCast(
        Vector3d position,
        Fixed64 radius,
        Vector3d direction,
        out LSRaycastHit raycastHit,
        Fixed64 maxDistance,
        PhysicsLayerMask layerMask)
    {
        if (CircleCast(position, radius, out LSRaycastHit hitInfo, layerMask))
        {
            Vector3d toHit = hitInfo.Point - position;
            if (toHit.SqrMagnitude <= maxDistance * maxDistance && Vector3d.Dot(toHit.Normal, direction) > Fixed64.Zero)
            {
                raycastHit = hitInfo;
                return true;
            }
        }

        raycastHit = default;
        return false;
    }

    /// <summary>
    /// Finds all colliders touching a circle and writes them from closest to farthest into caller-owned storage.
    /// </summary>
    public int CircleCastAll(
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

        RaycastHitSorter.SortByDistance(results);
        return results.Count;
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
        SwiftList<int>? colliderIds,
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
            if (!CheckCollider(colliderIds[i], position, radius, out LSRaycastHit hitInfo)
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
        SwiftList<int>? colliderIds,
        Vector3d position,
        Fixed64 radius,
        SwiftList<LSRaycastHit> results)
    {
        if (colliderIds == null)
            return;

        for (int i = colliderIds.Count - 1; i >= 0; i--)
        {
            if (CheckCollider(colliderIds[i], position, radius, out LSRaycastHit hitInfo))
                results.Add(hitInfo);
        }
    }

    private bool CheckCollider(int colliderId, Vector3d position, Fixed64 radius, out LSRaycastHit raycastHit)
    {
        raycastHit = default;
        if (!_context.Physics.TryGetColliderById(colliderId, out LSCollider? current))
            return false;

        LSCollider collider = current!;
        if (!_currentLayerMask.Includes(collider.Layer)
            || collider.SpherecastVersion == Version
            || !_redundantColliderCheck.Add(collider.Id))
        {
            return false;
        }

        collider.SpherecastVersion = Version;
        Fixed64 minFastDist = collider.ScaledRadius + radius;
        minFastDist *= minFastDist;

        Vector3d direction = collider.Position - position;
        if (direction.SqrMagnitude > minFastDist)
            return false;

        Vector3d normal = direction.Normal;
        Vector3d point = position + normal * radius;
        Fixed64 distance = direction.Magnitude;

        raycastHit = new LSRaycastHit(collider, point, normal, distance, direction);
        return true;
    }
}
