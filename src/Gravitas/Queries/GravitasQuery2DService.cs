//=======================================================================
// GravitasQuery2DService.cs
//=======================================================================
// MIT License, Copyright (c) 2026–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp;
using Gravitas.Colliders;
using Gravitas.Support;
using SwiftCollections;
using System.Runtime.CompilerServices;

namespace Gravitas.Queries;

/// <summary>
/// Owns pure 2D query buffers and query dispatch for one <see cref="GravitasWorldContext"/>.
/// </summary>
public sealed class GravitasQuery2DService
{
    private readonly GravitasWorldContext _context;
    private readonly SwiftList<LSCollider2D> _queryCandidates = new();
    private uint _circleQueryVersion;
    private uint _raycastVersion;

    /// <summary>
    /// Initializes a pure 2D query service for the supplied context.
    /// </summary>
    /// <param name="context">The owning world context.</param>
    public GravitasQuery2DService(GravitasWorldContext context)
    {
        SwiftThrowHelper.ThrowIfNull(context, nameof(context));
        _context = context;
    }

    /// <summary>
    /// Gets the owning world context.
    /// </summary>
    public GravitasWorldContext Context => _context;

    internal int LastQueryCandidateCount { get; private set; }

    /// <summary>
    /// Resets context-local pure 2D query buffers.
    /// </summary>
    public void Reset()
    {
        _queryCandidates.FastClear();
        LastQueryCandidateCount = 0;
        _circleQueryVersion = 0;
        _raycastVersion = 0;
    }

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
        uint queryVersion = NextCircleQueryVersion();
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
        Vector2d segment = end - start;
        if (segment.MagnitudeSquared == Fixed64.Zero)
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
                || (found && !Physics2DHitSorter.ComesBefore(candidate, closest)))
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
        Vector2d segment = end - start;
        if (segment.MagnitudeSquared == Fixed64.Zero)
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

        Vector2d segment = end - start;
        if (segment.MagnitudeSquared <= Fixed64.Epsilon)
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
                || (found && !Physics2DHitSorter.ComesBefore(candidate, closest)))
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
        Vector2d segment = end - start;
        if (segment.MagnitudeSquared <= Fixed64.Epsilon)
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

    private void EnsureCandidateCapacity()
    {
        int colliderCount = _context.Physics2D.ColliderCount;
        if (colliderCount > 0)
            _queryCandidates.EnsureCapacity(colliderCount);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private uint NextCircleQueryVersion()
    {
        _circleQueryVersion++;
        if (_circleQueryVersion == 0)
            _circleQueryVersion = 1;
        return _circleQueryVersion;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private uint NextRaycastVersion()
    {
        _raycastVersion++;
        if (_raycastVersion == 0)
            _raycastVersion = 1;
        return _raycastVersion;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static Vector2d CreateMin(Vector2d first, Vector2d second) =>
        new(FixedMath.Min(first.X, second.X), FixedMath.Min(first.Y, second.Y));

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static Vector2d CreateMax(Vector2d first, Vector2d second) =>
        new(FixedMath.Max(first.X, second.X), FixedMath.Max(first.Y, second.Y));

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static Vector2d CreateSweepMin(Vector2d first, Vector2d second, Fixed64 radius) =>
        new(FixedMath.Min(first.X, second.X) - radius, FixedMath.Min(first.Y, second.Y) - radius);

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static Vector2d CreateSweepMax(Vector2d first, Vector2d second, Fixed64 radius) =>
        new(FixedMath.Max(first.X, second.X) + radius, FixedMath.Max(first.Y, second.Y) + radius);

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static bool IsEligibleSweepCandidate(LSCollider2D collider, LSCollider2D? excludedCollider, bool includeTriggers)
    {
        if (!includeTriggers && collider.IsTrigger)
            return false;

        return excludedCollider == null
            || (!ReferenceEquals(collider, excludedCollider)
                && !ReferenceEquals(collider.AgentOrNull, excludedCollider.AgentOrNull)
                && !collider.IsSibling(excludedCollider));
    }
}
