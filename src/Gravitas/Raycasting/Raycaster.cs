using FixedMathSharp;
using Gravitas.Colliders;
using Gravitas.Support;
using GridForge;
using GridForge.Grids;
using GridForge.Utility;
using SwiftCollections;
using System;
using System.Collections.Generic;

namespace Gravitas.Raycasting;

/// <summary>
/// Provides functionality to perform raycasting operations on LSColliders in the lockstep framework.
/// </summary>
public static class Raycaster
{
    private static SwiftList<Vector3d> _bufferIntersectionPoints = new();
    private static readonly SwiftList<LSRaycastHit> _hitColliders = new();
    private static readonly SwiftHashSet<int> _redundantColliderCheck = new();

    private static SingleLayer _currentIgnoreLayer;
    private static uint _version;

    /// <summary>
    /// Resets the internal state of the Raycaster, clearing all data and returning all LSRaycastHits to the object pool.
    /// </summary>
    public static void Reset()
    {
        _version = 0;
        _bufferIntersectionPoints.FastClear();
        _redundantColliderCheck.Clear();
    }

    /// <summary>
    /// Performs a raycast from an origin in a specified direction up to a maximum distance, returning the first hit LSCollider.
    /// </summary>
    /// <param name="gridWorld">The GridWorld instance to perform the raycast on.</param>
    /// <param name="origin">The origin point of the raycast.</param>
    /// <param name="direction">The direction of the raycast.</param>
    /// <param name="maxDistance">The maximum distance the raycast should check for colliders.</param>
    /// <param name="raycastHit">Out parameter containing hit information if a collider is hit. If no collider is hit, this will be set to default.</param>
    /// <param name="ignoreLayers">Optional parameter specifying which layers to ignore during the raycast. Default is 0.</param>
    /// <returns>True if a collider is hit, false otherwise.</returns>
    public static bool Raycast(
        GridWorld gridWorld,
        Vector3d origin,
        Vector3d direction,
        Fixed64 maxDistance,
        out LSRaycastHit raycastHit,
        SingleLayer ignoreLayers)
    {
        _currentIgnoreLayer = ignoreLayers;

        Vector3d end = origin + direction * maxDistance;

        Fixed64 startHeight = origin.y;
        Fixed64 dist2d = (end.ToVector2d() - origin.ToVector2d()).Magnitude;

        if (dist2d != Fixed64.Zero)
        {
            Fixed64 heightSlope = (end.y - origin.y) / dist2d;
            if (heightSlope == Fixed64.Zero)
            {
                raycastHit = default;
                return false;
            }

            foreach (LSCollider collider in RaycastLine(gridWorld, origin, end))
            {
                bool heightIntersects = false;
                bool mined = false;
                bool maxed = false;
                Fixed64 closestDistance = Fixed64.MAX_VALUE;
                Vector3d closestIntersection = Vector3d.Zero; // Initialize to Zero

                for (int i = _bufferIntersectionPoints.Count - 1; i >= 0; i--)
                {
                    Fixed64 dist = Vector3d.Distance(_bufferIntersectionPoints[i], origin);
                    Fixed64 heightAtPosition = startHeight + (dist * heightSlope);

                    if (heightAtPosition < collider.BoundsMin.y)
                        mined = true;
                    else if (heightAtPosition > collider.BoundsMax.y)
                        maxed = true;
                    else
                        heightIntersects = true;

                    if (mined && maxed)
                        heightIntersects = true;

                    if (dist < closestDistance)
                    {
                        closestDistance = dist;
                        closestIntersection = _bufferIntersectionPoints[i];
                    }

                    if (heightIntersects)
                    {
                        Vector3d normal = collider.GetNormalAtPoint(closestIntersection);
                        raycastHit = new LSRaycastHit(collider, closestIntersection, Vector3d.Zero, closestDistance, direction);
                        return true;
                    }
                }
            }
        }

        raycastHit = default;
        return false;
    }

    private static readonly Comparison<LSRaycastHit> _rayCastComparison = (a, b) => a.Distance.CompareTo(b.Distance);

    /// <summary>
    /// Executes a raycast between two points, returning all hit LSColliders.
    /// This method takes into account the height of colliders, checking if the ray intersects the collider's height range.
    /// </summary>
    /// <param name="gridWorld">The GridWorld instance to perform the raycast on.</param>
    /// <param name="start3d">The starting point of the raycast.</param>
    /// <param name="end3d">The ending point of the raycast.</param>
    /// <param name="ignoreLayers">Optional parameter specifying which layers to ignore during the raycast. Default is 0.</param>
    /// <returns>An IEnumerable of LSRaycastHit information for each hit collider.</returns>
    public static IEnumerable<LSRaycastHit> RaycastAll(
        GridWorld gridWorld,
        Vector3d start3d,
        Vector3d end3d,
        SingleLayer ignoreLayers)
    {
        _currentIgnoreLayer = ignoreLayers;
        _hitColliders.FastClear();

        Fixed64 startHeight = start3d.y;
        Fixed64 dist2d = (end3d.ToVector2d() - start3d.ToVector2d()).Magnitude;
        Vector3d direction = (end3d - start3d).Normal;

        if (dist2d != Fixed64.Zero)
        {
            Fixed64 heightSlope = (end3d.y - start3d.y) / dist2d;
            if (heightSlope == Fixed64.Zero)
                yield break;

            foreach (LSCollider collider in RaycastLine(gridWorld, start3d, end3d))
            {
                bool heightIntersects = false;
                bool mined = false;
                bool maxed = false;
                Fixed64 closestDistance = Fixed64.MAX_VALUE;
                Vector3d closestIntersection = Vector3d.Zero; // Initialize to Zero
                for (int i = _bufferIntersectionPoints.Count - 1; i >= 0; i--)
                {
                    Fixed64 dist = Vector3d.Distance(_bufferIntersectionPoints[i], start3d);
                    Fixed64 heightAtPosition = startHeight + (dist * heightSlope);

                    if (heightAtPosition < collider.BoundsMin.y)
                        mined = true;
                    else if (heightAtPosition > collider.BoundsMax.y)
                        maxed = true;
                    else
                        heightIntersects = true;

                    if (mined && maxed)
                        heightIntersects = true;

                    if (mined && maxed)
                        heightIntersects = true;

                    if (dist < closestDistance)
                    {
                        closestDistance = dist;
                        closestIntersection = _bufferIntersectionPoints[i];
                    }

                    if (heightIntersects)
                        break;
                }

                if (heightIntersects)
                {
                    Vector3d normal = collider.GetNormalAtPoint(closestIntersection);
                    LSRaycastHit result = new(collider, closestIntersection, normal, closestDistance, direction);
                    _hitColliders.Add(result);
                }
            }
        }

        // TODO: _hitColliders should be a SwiftSortedList
        _hitColliders.Sort(Comparer<LSRaycastHit>.Create(_rayCastComparison));

        foreach (LSRaycastHit hitInfo in _hitColliders)
            yield return hitInfo;

        yield break;
    }

    /// <summary>
    /// Performs a raycast line between two points, returning all LSColliders hit by the ray.
    /// </summary>
    /// <param name="gridWorld">The GridWorld instance to perform the raycast on.</param>
    /// <param name="start">The starting point of the raycast.</param>
    /// <param name="end">The ending point of the raycast.</param>
    /// <returns>An IEnumerable of LSColliders hit by the raycast.</returns>
    private static IEnumerable<LSCollider> RaycastLine(GridWorld gridWorld, Vector3d start, Vector3d end)
    {
        _redundantColliderCheck.Clear();
        _bufferIntersectionPoints.FastClear();
        _version++;

        // Debug.DrawRay(start.ToVector3(), (end-start).ToVector3(), Color.red);

        RayCasterWorker.PrepareAxisCheck(start, end);
        foreach (GridVoxelSet gridVoxelSet in GridTracer.TraceLine(gridWorld, start, end))
        {
            foreach (Voxel voxel in gridVoxelSet.Voxels)
            {
                if (!voxel.TryGetPartition(out PhysicsPartition? partition))
                    continue;

                int dynamicCount = partition!.ContainedDynamicObjects?.Count ?? 0;
                for (int i = dynamicCount - 1; i >= 0; i--)
                {
                    int colliderId = partition.ContainedDynamicObjects?[i] ?? -1;
                    if (colliderId == -1) continue;
                    if (PhysicsManager.TryGetColliderById(colliderId, out LSCollider? current) && DoesCurrentColliderIntersectRay(current))
                        yield return current!;
                }

                int staticCount = partition.ContainedStaticObjects?.Count ?? 0;
                for (int l = staticCount - 1; l >= 0; l--)
                {
                    int colliderId = partition.ContainedStaticObjects?[l] ?? -1;
                    if (colliderId == -1) continue;
                    if (PhysicsManager.TryGetColliderById(colliderId, out LSCollider? current) && DoesCurrentColliderIntersectRay(current))
                        yield return current!;
                }
            }
        }

        yield break;
    }

    /// <summary>
    /// Checks if the current collider in the raycast path intersects with the ray.
    /// This method ensures that redundant checks are avoided by maintaining an internal version system.
    /// </summary>
    /// <returns>True if the collider intersects with the ray, false otherwise.</returns>
    private static bool DoesCurrentColliderIntersectRay(LSCollider? current)
    {
        if (current == null) return false;

        bool layerMaskExclude = _currentIgnoreLayer >= -1 && (_currentIgnoreLayer & (1 << current.Layer)) == 0;
        if (layerMaskExclude) return false;

        bool layerMaskIncludes = _currentIgnoreLayer == -1 || (_currentIgnoreLayer & (1 << current.Layer)) != 0;
        if (layerMaskIncludes
            && current.RaycastVersion != _version
            && _redundantColliderCheck.Add(current.Id))
        {
            current.RaycastVersion = _version;
            _bufferIntersectionPoints.FastClear();
            if (current.ColliderOverlapsRay(ref _bufferIntersectionPoints))
                return true;
        }

        return false;
    }
}
