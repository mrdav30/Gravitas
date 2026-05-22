using FixedMathSharp;
using Gravitas.Colliders;
using Gravitas.Support;
using GridForge.Grids;
using SwiftCollections;
using System;
using System.Collections.Generic;

namespace Gravitas.Raycasting;

/// <summary>
/// A utility class for performing a circle cast within a 3D space, primarily used in collision detection in a physics simulation.
/// The circle cast can identify all colliders (dynamic and static) that intersect with a defined circle.
/// </summary>
public static class Circlecaster
{
    private static readonly SwiftList<LSRaycastHit> _hitColliders = new();

    // stores previously checked Ids so we don't add them again if found on another node
    private static readonly SwiftHashSet<int> _redundantColliderCheck = new();

    private static SingleLayer _currentIgnoreLayer;
    private static uint _version;

    public static void Reset()
    {
        _version = 0;
        _hitColliders.FastClear();
        _redundantColliderCheck.Clear();
    }

    /// <summary>
    /// Performs a circle cast in the 3D space and returns the first collider that intersects with the circle.
    /// </summary>
    /// <param name="gridWorld">The GridWorld instance to perform the circle cast on.</param>
    /// <param name="position">The center point of the circle in the 3D space.</param>
    /// <param name="radius">The radius of the circle.</param>
    /// <param name="raycastHit">Out parameter that will contain information about the closest hit, if any.</param>
    /// <param name="ignoreLayers">Optional parameter to ignore certain layers during the cast (default is 0).</param>
    /// <returns>True if any collider intersects with the circle, false otherwise.</returns>
    /// <remarks>Returns a new instance of LSRayCastHit</remarks>
    public static bool CircleCast(
        GridWorld gridWorld,
        Vector3d position,
        Fixed64 radius,
        out LSRaycastHit raycastHit,
        SingleLayer ignoreLayers)
    {
        _currentIgnoreLayer = ignoreLayers;
        _version++;

        _hitColliders.FastClear();
        _redundantColliderCheck.Clear();

        Fixed64 xMin = position.x - radius;
        Fixed64 xMax = position.x + radius;
        Fixed64 y = position.y;
        Fixed64 zMin = position.z - radius;
        Fixed64 zMax = position.z + radius;

        LSCollider? current;
        LSRaycastHit? closestHit = null;
        Fixed64 closestDist = Fixed64.MAX_VALUE;

        for (Fixed64 x = xMin; x <= xMax; x += gridWorld.VoxelSize)
        {
            for (Fixed64 z = zMin; z <= zMax; z += gridWorld.VoxelSize)
            {
                Vector3d castPosition = new(x, y, z);
                if (!gridWorld.TryGetVoxel(castPosition, out Voxel? voxel)
                    || voxel!.TryGetPartition(out PhysicsPartition? partition) == false)
                {
                    continue;
                }

                int dynamicCount = partition!.ContainedDynamicObjects?.Count ?? 0;

                for (int k = dynamicCount - 1; k >= 0; k--)
                {
                    int colliderId = partition.ContainedDynamicObjects?[k] ?? -1;
                    if (colliderId == -1) continue;
                    if (PhysicsManager.TryGetColliderById(colliderId, out current)
                        && CheckCollider(current, position, radius, out LSRaycastHit? hitInfo)
                        && hitInfo!.Value.Distance < closestDist)
                    {
                        closestHit = hitInfo;
                        closestDist = hitInfo.Value.Distance;
                    }
                }

                int staticCount = partition.ContainedStaticObjects?.Count ?? 0;
                for (int l = staticCount - 1; l >= 0; l--)
                {
                    int colliderId = partition.ContainedStaticObjects?[l] ?? -1;
                    if (colliderId == -1) continue;
                    if (PhysicsManager.TryGetColliderById(colliderId, out current)
                        && CheckCollider(current, position, radius, out LSRaycastHit? hitInfo)
                        && hitInfo!.Value.Distance < closestDist)
                    {
                        closestHit = hitInfo;
                        closestDist = hitInfo.Value.Distance;
                    }
                }
            }
        }

        if (closestHit.HasValue)
        {
            raycastHit = closestHit.Value;
            return true;
        }
        else
        {
            raycastHit = default;
            return false;
        }
    }

    public static bool CircleCast(
        GridWorld gridWorld,
        Vector3d position,
        Fixed64 radius,
        Vector3d direction,
        out LSRaycastHit raycastHit,
        Fixed64 maxDistance,
        SingleLayer ignoreLayers)
    {
        if (CircleCast(gridWorld, position, radius, out LSRaycastHit hitInfo, ignoreLayers))
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

    private static readonly Comparison<LSRaycastHit> _rayCastComparer = (a, b) => a.Distance.CompareTo(b.Distance);

    /// <summary>
    /// Finds all dynamic bodies touching a defined circle and enumerates over them from closest to farthest away.
    /// </summary>
    /// <param name="gridWorld">The GridWorld instance to perform the circle cast on.</param>
    /// <param name="position">Starting position.</param>
    /// <param name="radius">Radius from position to search.</param>
    /// <param name="ignoreLayers">Ignore bodies with layer Id (default is 0).</param>
    public static IEnumerable<LSRaycastHit> CircleCastAll(
        GridWorld gridWorld,
        Vector3d position,
        Fixed64 radius,
        SingleLayer ignoreLayers)
    {
        _currentIgnoreLayer = ignoreLayers;
        _version++;

        _hitColliders.FastClear();
        _redundantColliderCheck.Clear();

        Fixed64 xMin = position.x - radius;
        Fixed64 xMax = position.x + radius;
        Fixed64 y = position.y;
        Fixed64 zMin = position.z - radius;
        Fixed64 zMax = position.z + radius;

        LSCollider? current = null;

        for (Fixed64 x = xMin; x <= xMax; x += gridWorld.VoxelSize)
        {
            for (Fixed64 z = zMin; z <= zMax; z += gridWorld.VoxelSize)
            {
                Vector3d castPosition = new(x, y, z);
                if (!gridWorld.TryGetVoxel(castPosition, out Voxel? voxel)
                    || voxel!.TryGetPartition(out PhysicsPartition? partition) == false)
                {
                    continue;
                }

                int dynamicCount = partition!.ContainedDynamicObjects?.Count ?? 0;
                for (int k = dynamicCount - 1; k >= 0; k--)
                {
                    int colliderId = partition.ContainedDynamicObjects?[k] ?? -1;
                    if (colliderId == -1) continue;
                    if (PhysicsManager.TryGetColliderById(colliderId, out current)
                        && CheckCollider(current, position, radius, out LSRaycastHit? raycastHit))
                    {
                        _hitColliders.Add(raycastHit!.Value);
                    }
                }

                int staticCount = partition.ContainedStaticObjects?.Count ?? 0;
                for (int l = staticCount - 1; l >= 0; l--)
                {
                    int colliderId = partition.ContainedStaticObjects?[l] ?? -1;
                    if (colliderId == -1) continue;
                    if (PhysicsManager.TryGetColliderById(colliderId, out current)
                        && CheckCollider(current, position, radius, out LSRaycastHit? raycastHit))
                    {
                        _hitColliders.Add(raycastHit!.Value);
                    }
                }
            }
        }

        _hitColliders.Sort(Comparer<LSRaycastHit>.Create(_rayCastComparer));

        for (int i = 0; i < _hitColliders.Count; i++)
            yield return _hitColliders[i];

        yield break;
    }

    /// <summary>
    /// Checks if the current collider is intersecting the circle.
    /// </summary>
    /// <param name="current">The collider to check for intersection.</param>
    /// <param name="position">The center of the circle.</param>
    /// <param name="radius">The radius of the circle.</param>
    /// <param name="raycastHit">Out parameter that will contain hit information if there is an intersection.</param>
    /// <returns>True if the collider intersects, false otherwise.</returns>
    private static bool CheckCollider(LSCollider? current, Vector3d position, Fixed64 radius, out LSRaycastHit? raycastHit)
    {
        raycastHit = null;

        if (current == null) return false;

        bool layerMaskExclude = _currentIgnoreLayer >= -1 && (_currentIgnoreLayer & (1 << current.Layer)) == 0;
        if (layerMaskExclude) return false;

        bool layerMaskIncludes = _currentIgnoreLayer == -1 || (_currentIgnoreLayer & (1 << current.Layer)) != 0;
        if (layerMaskIncludes
            && current.SpherecastVersion != _version
            && _redundantColliderCheck.Add(current.Id))
        {
            current.SpherecastVersion = _version;
            Fixed64 minFastDist = current.ScaledRadius + radius;
            //unnormalized distance value for comparison
            minFastDist *= minFastDist;

            Vector3d direction = current.Position - position;

            if (direction.SqrMagnitude <= minFastDist)//  Collider touches circle!
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
