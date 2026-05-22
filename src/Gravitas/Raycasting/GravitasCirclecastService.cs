using FixedMathSharp;
using Gravitas.Colliders;
using Gravitas.Support;
using GridForge.Grids;
using SwiftCollections;
using System;
using System.Collections.Generic;

namespace Gravitas.Raycasting;

/// <summary>
/// Owns circlecast query buffers and duplicate suppression for one <see cref="GravitasWorldContext"/>.
/// </summary>
public sealed class GravitasCirclecastService
{
    private static readonly Comparison<LSRaycastHit> RaycastComparer = (a, b) => a.Distance.CompareTo(b.Distance);

    private readonly GravitasWorldContext _context;
    private readonly SwiftList<LSRaycastHit> _hitColliders = new();
    private readonly SwiftHashSet<int> _redundantColliderCheck = new();

    private SingleLayer _currentIgnoreLayer;

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
        _hitColliders.FastClear();
        _redundantColliderCheck.Clear();
    }

    /// <summary>
    /// Performs a circle cast and returns the closest hit.
    /// </summary>
    public bool CircleCast(
        Vector3d position,
        Fixed64 radius,
        out LSRaycastHit raycastHit,
        SingleLayer ignoreLayers)
    {
        _currentIgnoreLayer = ignoreLayers;
        Version++;

        _hitColliders.FastClear();
        _redundantColliderCheck.Clear();

        Fixed64 xMin = position.x - radius;
        Fixed64 xMax = position.x + radius;
        Fixed64 y = position.y;
        Fixed64 zMin = position.z - radius;
        Fixed64 zMax = position.z + radius;

        LSRaycastHit? closestHit = null;
        Fixed64 closestDist = Fixed64.MAX_VALUE;

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

                ProcessPartitionForClosestHit(partition!, position, radius, ref closestHit, ref closestDist);
            }
        }

        if (closestHit.HasValue)
        {
            raycastHit = closestHit.Value;
            return true;
        }

        raycastHit = default;
        return false;
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
        SingleLayer ignoreLayers)
    {
        if (CircleCast(position, radius, out LSRaycastHit hitInfo, ignoreLayers))
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
    /// Finds all colliders touching a circle and returns them from closest to farthest away.
    /// </summary>
    public IEnumerable<LSRaycastHit> CircleCastAll(
        Vector3d position,
        Fixed64 radius,
        SingleLayer ignoreLayers)
    {
        _currentIgnoreLayer = ignoreLayers;
        Version++;

        _hitColliders.FastClear();
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

                ProcessPartitionForAllHits(partition!, position, radius);
            }
        }

        _hitColliders.Sort(Comparer<LSRaycastHit>.Create(RaycastComparer));

        for (int i = 0; i < _hitColliders.Count; i++)
            yield return _hitColliders[i];
    }

    private void ProcessPartitionForClosestHit(
        PhysicsPartition partition,
        Vector3d position,
        Fixed64 radius,
        ref LSRaycastHit? closestHit,
        ref Fixed64 closestDist)
    {
        int dynamicCount = partition.ContainedDynamicObjects?.Count ?? 0;
        for (int i = dynamicCount - 1; i >= 0; i--)
        {
            int colliderId = partition.ContainedDynamicObjects?[i] ?? -1;
            if (colliderId == -1)
                continue;

            if (_context.Physics.TryGetColliderById(colliderId, out LSCollider? current)
                && CheckCollider(current, position, radius, out LSRaycastHit? hitInfo)
                && hitInfo!.Value.Distance < closestDist)
            {
                closestHit = hitInfo;
                closestDist = hitInfo.Value.Distance;
            }
        }

        int staticCount = partition.ContainedStaticObjects?.Count ?? 0;
        for (int i = staticCount - 1; i >= 0; i--)
        {
            int colliderId = partition.ContainedStaticObjects?[i] ?? -1;
            if (colliderId == -1)
                continue;

            if (_context.Physics.TryGetColliderById(colliderId, out LSCollider? current)
                && CheckCollider(current, position, radius, out LSRaycastHit? hitInfo)
                && hitInfo!.Value.Distance < closestDist)
            {
                closestHit = hitInfo;
                closestDist = hitInfo.Value.Distance;
            }
        }
    }

    private void ProcessPartitionForAllHits(PhysicsPartition partition, Vector3d position, Fixed64 radius)
    {
        int dynamicCount = partition.ContainedDynamicObjects?.Count ?? 0;
        for (int i = dynamicCount - 1; i >= 0; i--)
        {
            int colliderId = partition.ContainedDynamicObjects?[i] ?? -1;
            if (colliderId == -1)
                continue;

            if (_context.Physics.TryGetColliderById(colliderId, out LSCollider? current)
                && CheckCollider(current, position, radius, out LSRaycastHit? raycastHit))
            {
                _hitColliders.Add(raycastHit!.Value);
            }
        }

        int staticCount = partition.ContainedStaticObjects?.Count ?? 0;
        for (int i = staticCount - 1; i >= 0; i--)
        {
            int colliderId = partition.ContainedStaticObjects?[i] ?? -1;
            if (colliderId == -1)
                continue;

            if (_context.Physics.TryGetColliderById(colliderId, out LSCollider? current)
                && CheckCollider(current, position, radius, out LSRaycastHit? raycastHit))
            {
                _hitColliders.Add(raycastHit!.Value);
            }
        }
    }

    private bool CheckCollider(LSCollider? current, Vector3d position, Fixed64 radius, out LSRaycastHit? raycastHit)
    {
        raycastHit = null;

        if (current == null)
            return false;

        bool layerMaskExclude = _currentIgnoreLayer >= -1 && (_currentIgnoreLayer & (1 << current.Layer)) == 0;
        if (layerMaskExclude)
            return false;

        bool layerMaskIncludes = _currentIgnoreLayer == -1 || (_currentIgnoreLayer & (1 << current.Layer)) != 0;
        if (layerMaskIncludes
            && current.SpherecastVersion != Version
            && _redundantColliderCheck.Add(current.Id))
        {
            current.SpherecastVersion = Version;
            Fixed64 minFastDist = current.ScaledRadius + radius;
            minFastDist *= minFastDist;

            Vector3d direction = current.Position - position;

            if (direction.SqrMagnitude <= minFastDist)
            {
                Vector3d normal = direction.Normal;
                Vector3d point = position + normal * radius;
                Fixed64 distance = radius;

                raycastHit = new LSRaycastHit(current, point, normal, distance, direction);
                return true;
            }
        }

        return false;
    }
}
