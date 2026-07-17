//=======================================================================
// GravitasQuery2DService.Raycast.cs
//=======================================================================
// MIT License, Copyright (c) 2026–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp;
using Gravitas.Support;
using SwiftCollections;

namespace Gravitas.Queries;

/// <summary>
/// Owns pure 2D segment raycast query behavior for one <see cref="GravitasWorldContext"/>.
/// </summary>
public sealed partial class GravitasQuery2DService
{
    /// <summary>
    /// Finds the closest pure 2D collider hit by the segment from <paramref name="start"/> to <paramref name="end"/>.
    /// </summary>
    public bool Raycast(Vector2d start, Vector2d end, out Physics2DHit hit)
    {
        return Raycast(start, end, PhysicsLayerMask.All, out hit);
    }

    /// <summary>
    /// Finds the closest pure 2D collider on an included layer hit by the segment.
    /// </summary>
    public bool Raycast(Vector2d start, Vector2d end, PhysicsLayerMask layerMask, out Physics2DHit hit)
    {
        if (!Vector2d.TrySubtract(end, start, out Vector2d segment)
            || !Vector2d.TryGetMagnitude(segment, out Fixed64 segmentLength)
            || segmentLength == Fixed64.Zero)
        {
            LastQueryCandidateCount = 0;
            hit = default;
            return false;
        }

        EnsureCandidateCapacity();
        uint queryVersion = NextRaycastVersion();
        _context.Collisions2D.CollectBoundsCandidates(
            CreateMin(start, end),
            CreateMax(start, end),
            layerMask,
            queryVersion,
            raycastQuery: true,
            _queryCandidates);

        LastQueryCandidateCount = _queryCandidates.Count;
        bool found = false;
        Physics2DHit closest = default;
        for (int i = 0; i < _queryCandidates.Count; i++)
        {
            if (!QueryDetection2D.TryRaycast(start, end, _queryCandidates[i], out Physics2DHit candidate)
                || !PhysicsHitSelectionPolicy.ShouldReplace(candidate, found, closest))
            {
                continue;
            }

            closest = candidate;
            found = true;
        }

        hit = closest;
        return found;
    }

    /// <summary>
    /// Writes all pure 2D colliders hit by the segment into <paramref name="results"/>.
    /// </summary>
    public int RaycastAll(Vector2d start, Vector2d end, SwiftList<Physics2DHit> results)
    {
        return RaycastAll(start, end, PhysicsLayerMask.All, results);
    }

    /// <summary>
    /// Writes all pure 2D colliders on included layers hit by the segment into <paramref name="results"/>.
    /// </summary>
    public int RaycastAll(Vector2d start, Vector2d end, PhysicsLayerMask layerMask, SwiftList<Physics2DHit> results)
    {
        SwiftThrowHelper.ThrowIfNull(results, nameof(results));

        results.FastClear();
        if (!Vector2d.TrySubtract(end, start, out Vector2d segment)
            || !Vector2d.TryGetMagnitude(segment, out Fixed64 segmentLength)
            || segmentLength == Fixed64.Zero)
        {
            LastQueryCandidateCount = 0;
            return 0;
        }

        EnsureCandidateCapacity();
        uint queryVersion = NextRaycastVersion();
        _context.Collisions2D.CollectBoundsCandidates(
            CreateMin(start, end),
            CreateMax(start, end),
            layerMask,
            queryVersion,
            raycastQuery: true,
            _queryCandidates);

        LastQueryCandidateCount = _queryCandidates.Count;
        for (int i = 0; i < _queryCandidates.Count; i++)
            if (QueryDetection2D.TryRaycast(start, end, _queryCandidates[i], out Physics2DHit hit))
                results.Add(hit);

        Physics2DHitSorter.SortByDistance(results);
        return results.Count;
    }

}
