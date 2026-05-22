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
/// Owns raycast query buffers and worker state for one <see cref="GravitasWorldContext"/>.
/// </summary>
public sealed class GravitasRaycastService
{
    private static readonly Comparison<LSRaycastHit> RaycastComparison = (a, b) => a.Distance.CompareTo(b.Distance);

    private readonly GravitasWorldContext _context;
    private readonly RaycastAxisWorker _worker = new();
    private SwiftList<Vector3d> _bufferIntersectionPoints = new();
    private readonly SwiftList<LSRaycastHit> _hitColliders = new();
    private readonly SwiftHashSet<int> _redundantColliderCheck = new();

    private SingleLayer _currentIgnoreLayer;

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
        _hitColliders.FastClear();
        _redundantColliderCheck.Clear();
    }

    /// <summary>
    /// Performs a raycast from an origin in a direction up to a maximum distance.
    /// </summary>
    public bool Raycast(
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

            foreach (LSCollider collider in RaycastLine(origin, end))
            {
                if (TryBuildHit(collider, origin, direction, startHeight, heightSlope, out raycastHit))
                    return true;
            }
        }

        raycastHit = default;
        return false;
    }

    /// <summary>
    /// Executes a raycast between two points and returns all hits from closest to farthest.
    /// </summary>
    public IEnumerable<LSRaycastHit> RaycastAll(
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
            if (heightSlope != Fixed64.Zero)
            {
                foreach (LSCollider collider in RaycastLine(start3d, end3d))
                {
                    if (TryBuildHit(collider, start3d, direction, startHeight, heightSlope, out LSRaycastHit hit))
                        _hitColliders.Add(hit);
                }
            }
        }

        _hitColliders.Sort(Comparer<LSRaycastHit>.Create(RaycastComparison));

        foreach (LSRaycastHit hitInfo in _hitColliders)
            yield return hitInfo;
    }

    private IEnumerable<LSCollider> RaycastLine(Vector3d start, Vector3d end)
    {
        _redundantColliderCheck.Clear();
        _bufferIntersectionPoints.FastClear();
        Version++;

        _worker.PrepareAxisCheck(start, end);
        foreach (GridVoxelSet gridVoxelSet in GridTracer.TraceLine(_context.World, start, end))
        {
            foreach (Voxel voxel in gridVoxelSet.Voxels)
            {
                if (!voxel.TryGetPartition(out PhysicsPartition? partition))
                    continue;

                int dynamicCount = partition!.ContainedDynamicObjects?.Count ?? 0;
                for (int i = dynamicCount - 1; i >= 0; i--)
                {
                    int colliderId = partition.ContainedDynamicObjects?[i] ?? -1;
                    if (colliderId == -1)
                        continue;

                    if (_context.Physics.TryGetColliderById(colliderId, out LSCollider? current)
                        && DoesCurrentColliderIntersectRay(current))
                    {
                        yield return current!;
                    }
                }

                int staticCount = partition.ContainedStaticObjects?.Count ?? 0;
                for (int i = staticCount - 1; i >= 0; i--)
                {
                    int colliderId = partition.ContainedStaticObjects?[i] ?? -1;
                    if (colliderId == -1)
                        continue;

                    if (_context.Physics.TryGetColliderById(colliderId, out LSCollider? current)
                        && DoesCurrentColliderIntersectRay(current))
                    {
                        yield return current!;
                    }
                }
            }
        }
    }

    private bool TryBuildHit(
        LSCollider collider,
        Vector3d origin,
        Vector3d direction,
        Fixed64 startHeight,
        Fixed64 heightSlope,
        out LSRaycastHit raycastHit)
    {
        bool heightIntersects = false;
        bool mined = false;
        bool maxed = false;
        Fixed64 closestDistance = Fixed64.MAX_VALUE;
        Vector3d closestIntersection = Vector3d.Zero;

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
                break;
        }

        if (!heightIntersects)
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

        bool layerMaskExclude = _currentIgnoreLayer >= -1 && (_currentIgnoreLayer & (1 << current.Layer)) == 0;
        if (layerMaskExclude)
            return false;

        bool layerMaskIncludes = _currentIgnoreLayer == -1 || (_currentIgnoreLayer & (1 << current.Layer)) != 0;
        if (layerMaskIncludes
            && current.RaycastVersion != Version
            && _redundantColliderCheck.Add(current.Id))
        {
            current.RaycastVersion = Version;
            _bufferIntersectionPoints.FastClear();
            if (current.ColliderOverlapsRay(_worker, ref _bufferIntersectionPoints))
                return true;
        }

        return false;
    }
}
