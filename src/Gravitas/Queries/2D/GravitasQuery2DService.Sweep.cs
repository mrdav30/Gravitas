//=======================================================================
// GravitasQuery2DService.Sweep.cs
//=======================================================================
// MIT License, Copyright (c) 2026–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp;
using Gravitas.Colliders;
using Gravitas.Support;
using SwiftCollections;

namespace Gravitas.Queries;

/// <summary>
/// Owns pure 2D swept-circle query behavior for one <see cref="GravitasWorldContext"/>.
/// </summary>
public sealed partial class GravitasQuery2DService
{
    /// <summary>
    /// Finds the closest pure 2D collider hit by sweeping a circle from <paramref name="start"/> to <paramref name="end"/>.
    /// </summary>
    public bool SweepCircle(Vector2d start, Vector2d end, Fixed64 radius, out Physics2DHit hit)
    {
        return SweepCircle(start, end, radius, PhysicsLayerMask.All, out hit);
    }

    /// <summary>
    /// Finds the closest pure 2D collider on an included layer hit by sweeping a circle.
    /// </summary>
    public bool SweepCircle(
        Vector2d start,
        Vector2d end,
        Fixed64 radius,
        PhysicsLayerMask layerMask,
        out Physics2DHit hit,
        LSCollider2D? excludedCollider = null,
        bool includeTriggers = true)
    {
        SwiftThrowHelper.ThrowIfArgument(radius <= Fixed64.Zero, nameof(radius), "2D sweep radius must be greater than zero.");

        if (!FixedVectorDifference.TryCreate(start, end, out Vector2d segment)
            || !Vector2d.TryGetMagnitude(segment, out Fixed64 segmentLength)
            || segmentLength <= Fixed64.Epsilon)
        {
            LastQueryCandidateCount = 0;
            hit = default;
            return false;
        }

        EnsureCandidateCapacity();
        uint queryVersion = NextRaycastVersion();
        _context.Collisions2D.CollectBoundsCandidates(
            CreateSweepMin(start, end, radius),
            CreateSweepMax(start, end, radius),
            layerMask,
            queryVersion,
            raycastQuery: true,
            _queryCandidates);

        LastQueryCandidateCount = _queryCandidates.Count;
        bool found = false;
        Physics2DHit closest = default;
        for (int i = 0; i < _queryCandidates.Count; i++)
        {
            LSCollider2D collider = _queryCandidates[i];
            if (!IsEligibleSweepCandidate(collider, excludedCollider, includeTriggers)
                || !QueryDetection2D.TrySweepCircle(start, end, radius, collider, out Physics2DHit candidate)
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
    /// Writes all pure 2D colliders hit by sweeping a circle into <paramref name="results"/>.
    /// </summary>
    public int SweepCircleAll(Vector2d start, Vector2d end, Fixed64 radius, SwiftList<Physics2DHit> results)
    {
        return SweepCircleAll(start, end, radius, PhysicsLayerMask.All, results);
    }

    /// <summary>
    /// Writes all pure 2D colliders on included layers hit by sweeping a circle into <paramref name="results"/>.
    /// </summary>
    public int SweepCircleAll(
        Vector2d start,
        Vector2d end,
        Fixed64 radius,
        PhysicsLayerMask layerMask,
        SwiftList<Physics2DHit> results,
        LSCollider2D? excludedCollider = null,
        bool includeTriggers = true)
    {
        return SweepCircleAllCore(
            start,
            end,
            radius,
            layerMask,
            results,
            excludedCollider,
            includeTriggers,
            staticTargetsOnly: false);
    }

    internal int SweepCircleAgainstStaticAll(
        Vector2d start,
        Vector2d end,
        Fixed64 radius,
        PhysicsLayerMask layerMask,
        SwiftList<Physics2DHit> results,
        LSCollider2D? excludedCollider = null,
        bool includeTriggers = true)
    {
        return SweepCircleAllCore(
            start,
            end,
            radius,
            layerMask,
            results,
            excludedCollider,
            includeTriggers,
            staticTargetsOnly: true);
    }

    private int SweepCircleAllCore(
        Vector2d start,
        Vector2d end,
        Fixed64 radius,
        PhysicsLayerMask layerMask,
        SwiftList<Physics2DHit> results,
        LSCollider2D? excludedCollider,
        bool includeTriggers,
        bool staticTargetsOnly)
    {
        SwiftThrowHelper.ThrowIfNull(results, nameof(results));
        SwiftThrowHelper.ThrowIfArgument(radius <= Fixed64.Zero, nameof(radius), "2D sweep radius must be greater than zero.");

        results.FastClear();
        if (!FixedVectorDifference.TryCreate(start, end, out Vector2d segment)
            || !Vector2d.TryGetMagnitude(segment, out Fixed64 segmentLength)
            || segmentLength <= Fixed64.Epsilon)
        {
            LastQueryCandidateCount = 0;
            return 0;
        }

        EnsureCandidateCapacity();
        uint queryVersion = NextRaycastVersion();
        _context.Collisions2D.CollectBoundsCandidates(
            CreateSweepMin(start, end, radius),
            CreateSweepMax(start, end, radius),
            layerMask,
            queryVersion,
            raycastQuery: true,
            _queryCandidates,
            staticStyleOnly: staticTargetsOnly);

        LastQueryCandidateCount = _queryCandidates.Count;
        for (int i = 0; i < _queryCandidates.Count; i++)
        {
            LSCollider2D collider = _queryCandidates[i];
            if (IsEligibleSweepCandidate(collider, excludedCollider, includeTriggers)
                && QueryDetection2D.TrySweepCircle(start, end, radius, collider, out Physics2DHit hit))
            {
                results.Add(hit);
            }
        }

        Physics2DHitSorter.SortByDistance(results);
        return results.Count;
    }

}
