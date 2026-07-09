//=======================================================================
// GravitasQuery2DService.Overlap.cs
//=======================================================================
// MIT License, Copyright (c) 2026–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp;
using Gravitas.Colliders;
using Gravitas.Support;
using SwiftCollections;
using System;

namespace Gravitas.Queries;

/// <summary>
/// Owns pure 2D overlap query behavior for one <see cref="GravitasWorldContext"/>.
/// </summary>
public sealed partial class GravitasQuery2DService
{
    /// <summary>
    /// Writes all active pure 2D colliders overlapping the query circle into <paramref name="results"/>.
    /// </summary>
    /// <returns>The number of hits written to <paramref name="results"/>.</returns>
    public int OverlapCircleAll(Vector2d center, Fixed64 radius, SwiftList<Physics2DHit> results)
    {
        return OverlapCircleAll(center, radius, PhysicsLayerMask.All, results);
    }

    /// <summary>
    /// Writes all active pure 2D colliders on included layers that overlap the query circle.
    /// </summary>
    /// <returns>The number of hits written to <paramref name="results"/>.</returns>
    public int OverlapCircleAll(
        Vector2d center,
        Fixed64 radius,
        PhysicsLayerMask layerMask,
        SwiftList<Physics2DHit> results)
    {
        return OverlapCircleAllCore(center, radius, layerMask, results, staticTargetsOnly: false);
    }

    internal int OverlapCircleAgainstStaticAll(
        Vector2d center,
        Fixed64 radius,
        PhysicsLayerMask layerMask,
        SwiftList<Physics2DHit> results,
        LSCollider2D? excludedCollider = null,
        bool includeTriggers = true)
    {
        return OverlapCircleAllCore(
            center,
            radius,
            layerMask,
            results,
            staticTargetsOnly: true,
            excludedCollider: excludedCollider,
            includeTriggers: includeTriggers);
    }

    /// <summary>
    /// Finds the closest active pure 2D collider overlapping the supplied query circle.
    /// </summary>
    public bool OverlapCircle(Vector2d center, Fixed64 radius, out Physics2DHit hit)
    {
        return OverlapCircle(center, radius, PhysicsLayerMask.All, out hit);
    }

    /// <summary>
    /// Finds the closest active pure 2D collider on an included layer overlapping the supplied query circle.
    /// </summary>
    public bool OverlapCircle(Vector2d center, Fixed64 radius, PhysicsLayerMask layerMask, out Physics2DHit hit)
    {
        SwiftThrowHelper.ThrowIfArgument(radius < Fixed64.Zero, nameof(radius), "2D query radius cannot be negative.");

        EnsureCandidateCapacity();
        uint queryVersion = NextOverlapQueryVersion();
        _context.Collisions2D.CollectBoundsCandidates(
            new Vector2d(center.X - radius, center.Y - radius),
            new Vector2d(center.X + radius, center.Y + radius),
            layerMask,
            queryVersion,
            raycastQuery: false,
            _queryCandidates);

        LastQueryCandidateCount = _queryCandidates.Count;
        bool found = false;
        Physics2DHit closest = default;
        for (int i = 0; i < _queryCandidates.Count; i++)
        {
            if (!QueryDetection2D.TryOverlapCircle(center, radius, _queryCandidates[i], out Physics2DHit candidate)
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
    /// Finds the closest active pure 2D collider overlapping the supplied axis-aligned box.
    /// </summary>
    public bool OverlapAabb(Vector2d center, Vector2d size, out Physics2DHit hit)
    {
        return OverlapAabb(center, size, PhysicsLayerMask.All, out hit);
    }

    /// <summary>
    /// Finds the closest active pure 2D collider on an included layer overlapping the supplied axis-aligned box.
    /// </summary>
    public bool OverlapAabb(Vector2d center, Vector2d size, PhysicsLayerMask layerMask, out Physics2DHit hit)
    {
        QueryDetection2D.ValidateAabbSize(size);
        Vector2d halfExtents = size * Fixed64.Half;
        Span<Vector2d> vertices = stackalloc Vector2d[4];
        CreateAabbVertices(center, halfExtents, vertices);
        return OverlapAreaCore(
            vertices,
            center,
            center - halfExtents,
            center + halfExtents,
            layerMask,
            out hit);
    }

    /// <summary>
    /// Writes all active pure 2D colliders overlapping the supplied axis-aligned box into <paramref name="results"/>.
    /// </summary>
    /// <returns>The number of hits written to <paramref name="results"/>.</returns>
    public int OverlapAabbAll(Vector2d center, Vector2d size, SwiftList<Physics2DHit> results)
    {
        return OverlapAabbAll(center, size, PhysicsLayerMask.All, results);
    }

    /// <summary>
    /// Writes all active pure 2D colliders on included layers that overlap the supplied axis-aligned box.
    /// </summary>
    /// <returns>The number of hits written to <paramref name="results"/>.</returns>
    public int OverlapAabbAll(
        Vector2d center,
        Vector2d size,
        PhysicsLayerMask layerMask,
        SwiftList<Physics2DHit> results)
    {
        SwiftThrowHelper.ThrowIfNull(results, nameof(results));
        QueryDetection2D.ValidateAabbSize(size);
        Vector2d halfExtents = size * Fixed64.Half;
        Span<Vector2d> vertices = stackalloc Vector2d[4];
        CreateAabbVertices(center, halfExtents, vertices);
        return OverlapAreaAllCore(
            vertices,
            center,
            center - halfExtents,
            center + halfExtents,
            layerMask,
            results);
    }

    /// <summary>
    /// Finds the closest active pure 2D collider overlapping the supplied convex polygon.
    /// </summary>
    public bool OverlapPolygon(ReadOnlySpan<Vector2d> vertices, out Physics2DHit hit)
    {
        return OverlapPolygon(vertices, PhysicsLayerMask.All, out hit);
    }

    /// <summary>
    /// Finds the closest active pure 2D collider on an included layer overlapping the supplied convex polygon.
    /// </summary>
    public bool OverlapPolygon(ReadOnlySpan<Vector2d> vertices, PhysicsLayerMask layerMask, out Physics2DHit hit)
    {
        QueryDetection2D.ValidateConvexQueryPolygon(vertices);
        Vector2d center = QueryDetection2D.CalculateAverageCenter(vertices);
        CalculateAreaBounds(vertices, out Vector2d min, out Vector2d max);
        return OverlapAreaCore(vertices, center, min, max, layerMask, out hit);
    }

    /// <summary>
    /// Writes all active pure 2D colliders overlapping the supplied convex polygon into <paramref name="results"/>.
    /// </summary>
    /// <returns>The number of hits written to <paramref name="results"/>.</returns>
    public int OverlapPolygonAll(ReadOnlySpan<Vector2d> vertices, SwiftList<Physics2DHit> results)
    {
        return OverlapPolygonAll(vertices, PhysicsLayerMask.All, results);
    }

    /// <summary>
    /// Writes all active pure 2D colliders on included layers that overlap the supplied convex polygon.
    /// </summary>
    /// <returns>The number of hits written to <paramref name="results"/>.</returns>
    public int OverlapPolygonAll(
        ReadOnlySpan<Vector2d> vertices,
        PhysicsLayerMask layerMask,
        SwiftList<Physics2DHit> results)
    {
        SwiftThrowHelper.ThrowIfNull(results, nameof(results));
        QueryDetection2D.ValidateConvexQueryPolygon(vertices);
        Vector2d center = QueryDetection2D.CalculateAverageCenter(vertices);
        CalculateAreaBounds(vertices, out Vector2d min, out Vector2d max);
        return OverlapAreaAllCore(vertices, center, min, max, layerMask, results);
    }

    private int OverlapCircleAllCore(
        Vector2d center,
        Fixed64 radius,
        PhysicsLayerMask layerMask,
        SwiftList<Physics2DHit> results,
        bool staticTargetsOnly,
        LSCollider2D? excludedCollider = null,
        bool includeTriggers = true)
    {
        SwiftThrowHelper.ThrowIfNull(results, nameof(results));
        SwiftThrowHelper.ThrowIfArgument(radius < Fixed64.Zero, nameof(radius), "2D query radius cannot be negative.");

        results.FastClear();
        EnsureCandidateCapacity();
        uint queryVersion = NextOverlapQueryVersion();
        _context.Collisions2D.CollectBoundsCandidates(
            new Vector2d(center.X - radius, center.Y - radius),
            new Vector2d(center.X + radius, center.Y + radius),
            layerMask,
            queryVersion,
            raycastQuery: false,
            _queryCandidates,
            staticStyleOnly: staticTargetsOnly);

        LastQueryCandidateCount = _queryCandidates.Count;
        for (int i = 0; i < _queryCandidates.Count; i++)
        {
            LSCollider2D collider = _queryCandidates[i];
            if (IsEligibleSweepCandidate(collider, excludedCollider, includeTriggers)
                && QueryDetection2D.TryOverlapCircle(center, radius, collider, out Physics2DHit hit))
            {
                results.Add(hit);
            }
        }

        Physics2DHitSorter.SortByDistance(results);
        return results.Count;
    }

    private bool OverlapAreaCore(
        ReadOnlySpan<Vector2d> vertices,
        Vector2d center,
        Vector2d min,
        Vector2d max,
        PhysicsLayerMask layerMask,
        out Physics2DHit hit)
    {
        EnsureCandidateCapacity();
        uint queryVersion = NextOverlapQueryVersion();
        _context.Collisions2D.CollectBoundsCandidates(
            min,
            max,
            layerMask,
            queryVersion,
            raycastQuery: false,
            _queryCandidates);

        LastQueryCandidateCount = _queryCandidates.Count;
        bool found = false;
        Physics2DHit closest = default;
        for (int i = 0; i < _queryCandidates.Count; i++)
        {
            if (!QueryDetection2D.TryOverlapPolygon(vertices, center, _queryCandidates[i], out Physics2DHit candidate)
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

    private int OverlapAreaAllCore(
        ReadOnlySpan<Vector2d> vertices,
        Vector2d center,
        Vector2d min,
        Vector2d max,
        PhysicsLayerMask layerMask,
        SwiftList<Physics2DHit> results)
    {
        results.FastClear();
        EnsureCandidateCapacity();
        uint queryVersion = NextOverlapQueryVersion();
        _context.Collisions2D.CollectBoundsCandidates(
            min,
            max,
            layerMask,
            queryVersion,
            raycastQuery: false,
            _queryCandidates);

        LastQueryCandidateCount = _queryCandidates.Count;
        for (int i = 0; i < _queryCandidates.Count; i++)
            if (QueryDetection2D.TryOverlapPolygon(vertices, center, _queryCandidates[i], out Physics2DHit hit))
                results.Add(hit);

        Physics2DHitSorter.SortByDistance(results);
        return results.Count;
    }

}
