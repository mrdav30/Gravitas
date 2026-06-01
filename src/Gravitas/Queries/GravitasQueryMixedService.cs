using FixedMathSharp;
using Gravitas.Colliders;
using Gravitas.Support;
using SwiftCollections;
using System.Runtime.CompilerServices;

namespace Gravitas.Queries;

/// <summary>
/// Owns explicit mixed 3D/2D query buffers for one <see cref="GravitasWorldContext"/>.
/// </summary>
public sealed class GravitasQueryMixedService
{
    private readonly GravitasWorldContext _context;
    private readonly SwiftList<LSCollider2D> _candidates2D = new();
    private readonly SwiftList<LSCollider> _candidates3D = new();
    private readonly SwiftList<PhysicsMixedHit> _singleHitResults = new();
    private readonly SweptSphereQueryWorker _sweepWorker = new();

    public GravitasQueryMixedService(GravitasWorldContext context)
    {
        SwiftThrowHelper.ThrowIfNull(context, nameof(context));
        _context = context;
    }

    public GravitasWorldContext Context => _context;

    internal int LastQueryCandidateCount { get; private set; }

    public void Reset()
    {
        _candidates2D.FastClear();
        _candidates3D.FastClear();
        _singleHitResults.FastClear();
        LastQueryCandidateCount = 0;
    }

    /// <summary>
    /// Sweeps a 3D sphere against embedded 2D mixed slabs and returns the closest hit.
    /// </summary>
    public bool SweepSphereAgainst2D(
        Vector3d start,
        Vector3d end,
        Fixed64 radius,
        PhysicsLayerMask layerMask,
        out PhysicsMixedHit hit,
        LSCollider? excludedCollider = null,
        bool includeTriggers = true)
    {
        int count = SweepSphereAgainst2DAll(start, end, radius, layerMask, _singleHitResults, excludedCollider, includeTriggers);
        hit = count > 0 ? _singleHitResults[0] : default;
        return count > 0;
    }

    /// <summary>
    /// Sweeps a 3D sphere against embedded 2D mixed slabs and writes hits from closest to farthest.
    /// </summary>
    public int SweepSphereAgainst2DAll(
        Vector3d start,
        Vector3d end,
        Fixed64 radius,
        PhysicsLayerMask layerMask,
        SwiftList<PhysicsMixedHit> results,
        LSCollider? excludedCollider = null,
        bool includeTriggers = true)
    {
        SwiftThrowHelper.ThrowIfNull(results, nameof(results));
        SwiftThrowHelper.ThrowIfArgument(radius <= Fixed64.Zero, nameof(radius), "Mixed swept sphere radius must be greater than zero.");

        results.FastClear();
        Vector3d segment = end - start;
        if (segment.SqrMagnitude <= Fixed64.Epsilon)
        {
            LastQueryCandidateCount = 0;
            return 0;
        }

        Vector3d direction = segment.Normal;
        Fixed64 length = segment.Magnitude;
        CreateSweepBounds(start, end, radius, out Vector3d min, out Vector3d max);
        _context.MixedCollisions.Collect2DCandidatesInMixedBounds(min, max, layerMask, _candidates2D);
        LastQueryCandidateCount = _candidates2D.Count;

        for (int i = 0; i < _candidates2D.Count; i++)
        {
            LSCollider2D collider = _candidates2D[i];
            if (IsEligible2DTarget(collider, excludedCollider, includeTriggers)
                && TrySweepSphereAgainst2D(start, direction, length, radius, collider, out PhysicsMixedHit candidate))
            {
                results.Add(candidate);
            }
        }

        PhysicsMixedHitSorter.SortByDistance(results);
        _context.Diagnostics.EmitMixedQuery(
            start,
            end,
            radius,
            layerMask.Bits,
            results.Count > 0,
            results.Count,
            results.Count > 0 ? results[0] : default);
        return results.Count;
    }

    /// <summary>
    /// Sweeps a 2D circle embedded in a finite mixed slab against 3D colliders and returns the closest hit.
    /// </summary>
    public bool SweepCircleAgainst3D(
        Vector2d start,
        Vector2d end,
        Fixed64 radius,
        Fixed64 slabCenterY,
        Fixed64 halfThickness,
        PhysicsLayerMask layerMask,
        out PhysicsMixedHit hit,
        LSCollider2D? excludedCollider = null,
        bool includeTriggers = true)
    {
        int count = SweepCircleAgainst3DAll(
            start,
            end,
            radius,
            slabCenterY,
            halfThickness,
            layerMask,
            _singleHitResults,
            excludedCollider,
            includeTriggers);
        hit = count > 0 ? _singleHitResults[0] : default;
        return count > 0;
    }

    /// <summary>
    /// Sweeps a 2D circle embedded in a finite mixed slab against 3D colliders and writes hits from closest to farthest.
    /// </summary>
    public int SweepCircleAgainst3DAll(
        Vector2d start,
        Vector2d end,
        Fixed64 radius,
        Fixed64 slabCenterY,
        Fixed64 halfThickness,
        PhysicsLayerMask layerMask,
        SwiftList<PhysicsMixedHit> results,
        LSCollider2D? excludedCollider = null,
        bool includeTriggers = true)
    {
        SwiftThrowHelper.ThrowIfNull(results, nameof(results));
        SwiftThrowHelper.ThrowIfArgument(radius <= Fixed64.Zero, nameof(radius), "Mixed swept circle radius must be greater than zero.");
        SwiftThrowHelper.ThrowIfArgument(halfThickness <= Fixed64.Zero, nameof(halfThickness), "Mixed swept circle half-thickness must be greater than zero.");

        results.FastClear();
        Vector2d segment = end - start;
        if (segment.SqrMagnitude <= Fixed64.Epsilon)
        {
            LastQueryCandidateCount = 0;
            return 0;
        }

        Vector3d start3D = new(start.x, slabCenterY, start.y);
        Vector3d end3D = new(end.x, slabCenterY, end.y);
        Vector3d direction = (end3D - start3D).Normal;
        Fixed64 proxyRadius = FixedMath.Max(radius, halfThickness);
        CreateSweepBounds(start3D, end3D, proxyRadius, out Vector3d min, out Vector3d max);
        _context.MixedCollisions.Collect3DCandidatesInMixedBounds(min, max, layerMask, _candidates3D);
        LastQueryCandidateCount = _candidates3D.Count;
        _sweepWorker.Prepare(start3D, end3D, proxyRadius);

        for (int i = 0; i < _candidates3D.Count; i++)
        {
            LSCollider collider = _candidates3D[i];
            if (!IsEligible3DTarget(collider, excludedCollider, includeTriggers)
                || !_sweepWorker.TrySweep(collider, out Vector3d centerAtImpact, out Fixed64 distance))
            {
                continue;
            }

            results.Add(BuildCircleAgainst3DHit(
                collider,
                centerAtImpact,
                direction,
                radius,
                slabCenterY,
                halfThickness,
                distance,
                excludedCollider));
        }

        PhysicsMixedHitSorter.SortByDistance(results);
        _context.Diagnostics.EmitMixedQuery(
            start3D,
            end3D,
            radius,
            layerMask.Bits,
            results.Count > 0,
            results.Count,
            results.Count > 0 ? results[0] : default);
        return results.Count;
    }

    private static bool TrySweepSphereAgainst2D(
        Vector3d start,
        Vector3d direction,
        Fixed64 length,
        Fixed64 radius,
        LSCollider2D collider,
        out PhysicsMixedHit hit)
    {
        if (collider is LSCircleCollider2D circle)
            return TrySweepSphereAgainstCircleSlab(start, direction, length, radius, circle, out hit);

        return TrySweepSphereAgainstPrismBounds(start, direction, length, radius, collider, out hit);
    }

    private static bool TrySweepSphereAgainstCircleSlab(
        Vector3d start,
        Vector3d direction,
        Fixed64 length,
        Fixed64 radius,
        LSCircleCollider2D circle,
        out PhysicsMixedHit hit)
    {
        Vector3d center = new(circle.Center.x, circle.MixedSlabCenterY, circle.Center.y);
        Fixed64 combinedRadius = circle.Radius + radius;
        Fixed64 expandedHalfHeight = circle.MixedHalfThickness + radius;
        Vector3d localStart = start - center;

        if (IsInsideCircleSlab(localStart, combinedRadius, expandedHalfHeight))
        {
            hit = BuildSphereAgainst2DHit(circle, start, radius, Fixed64.Zero, direction);
            return true;
        }

        bool found = false;
        Fixed64 bestDistance = Fixed64.MAX_VALUE;
        TryKeepEarlierSweep(
            TrySweepCircleSlabSide(localStart, direction, length, combinedRadius, expandedHalfHeight, out Fixed64 sideDistance),
            sideDistance,
            ref found,
            ref bestDistance);
        TryKeepEarlierSweep(
            TrySweepCircleSlabCap(localStart, direction, length, combinedRadius, expandedHalfHeight, out Fixed64 topDistance),
            topDistance,
            ref found,
            ref bestDistance);
        TryKeepEarlierSweep(
            TrySweepCircleSlabCap(localStart, direction, length, combinedRadius, -expandedHalfHeight, out Fixed64 bottomDistance),
            bottomDistance,
            ref found,
            ref bestDistance);

        if (!found)
        {
            hit = default;
            return false;
        }

        Vector3d sweepCenter = start + direction * bestDistance;
        hit = BuildSphereAgainst2DHit(circle, sweepCenter, radius, bestDistance, direction);
        return true;
    }

    private static bool TrySweepSphereAgainstPrismBounds(
        Vector3d start,
        Vector3d direction,
        Fixed64 length,
        Fixed64 radius,
        LSCollider2D collider,
        out PhysicsMixedHit hit)
    {
        BoundingBox bounds = collider.MixedBounds3D;
        Vector3d radiusExtents = Vector3d.One * radius;
        Vector3d min = bounds.Min - radiusExtents;
        Vector3d max = bounds.Max + radiusExtents;
        if (!TrySweepBox(start, direction, length, min, max, out Fixed64 distance))
        {
            hit = default;
            return false;
        }

        Vector3d sweepCenter = start + direction * distance;
        hit = BuildSphereAgainst2DHit(collider, sweepCenter, radius, distance, direction);
        return true;
    }

    private static PhysicsMixedHit BuildSphereAgainst2DHit(
        LSCollider2D collider,
        Vector3d sweepCenter,
        Fixed64 radius,
        Fixed64 distance,
        Vector3d direction)
    {
        Vector3d point2D = GetClosestEmbeddedPoint(collider, sweepCenter);
        Vector3d to2D = point2D - sweepCenter;
        Vector3d normal3DTo2D = to2D.SqrMagnitude > Fixed64.Epsilon
            ? to2D.Normal
            : Resolve3DTo2DFallback(collider, sweepCenter, direction);
        Vector3d point3D = sweepCenter + normal3DTo2D * radius;
        return new PhysicsMixedHit(
            null,
            collider,
            point3D,
            point2D,
            normal3DTo2D,
            distance,
            direction);
    }

    private static PhysicsMixedHit BuildCircleAgainst3DHit(
        LSCollider collider,
        Vector3d sweepCenter,
        Vector3d direction,
        Fixed64 radius,
        Fixed64 slabCenterY,
        Fixed64 halfThickness,
        Fixed64 distance,
        LSCollider2D? sourceCollider)
    {
        Vector3d point3D = GetSweepSurfacePoint(collider, sweepCenter, direction);
        Vector3d to2D = sweepCenter - point3D;
        Vector3d normal3DTo2D = to2D.SqrMagnitude > Fixed64.Epsilon
            ? to2D.Normal
            : direction.SqrMagnitude > Fixed64.Epsilon ? -direction.Normal : Vector3d.Right;
        Vector2d planarNormal = new(normal3DTo2D.x, normal3DTo2D.z);
        Vector2d planarPoint = new(sweepCenter.x, sweepCenter.z);
        if (planarNormal.SqrMagnitude > Fixed64.Epsilon)
            planarPoint -= planarNormal.Normal * radius;

        Vector3d point2D = new(
            planarPoint.x,
            ClampAxis(point3D.y, slabCenterY - halfThickness, slabCenterY + halfThickness),
            planarPoint.y);
        return new PhysicsMixedHit(
            collider,
            sourceCollider,
            point3D,
            point2D,
            normal3DTo2D,
            distance,
            direction);
    }

    private static Vector3d GetClosestEmbeddedPoint(LSCollider2D collider, Vector3d sweepCenter)
    {
        Vector2d closest2D = collider.GetClosestPoint(new Vector2d(sweepCenter.x, sweepCenter.z));
        return new Vector3d(
            closest2D.x,
            ClampAxis(
                sweepCenter.y,
                collider.MixedSlabCenterY - collider.MixedHalfThickness,
                collider.MixedSlabCenterY + collider.MixedHalfThickness),
            closest2D.y);
    }

    private static Vector3d GetSweepSurfacePoint(LSCollider collider, Vector3d sweepCenter, Vector3d direction)
    {
        Vector3d centerDelta = sweepCenter - collider.Center;
        if (centerDelta.SqrMagnitude <= Fixed64.Epsilon)
            return collider.Center - direction * collider.ScaledRadius;

        return collider.ClosestPointOnSurface(sweepCenter);
    }

    private static Vector3d Resolve3DTo2DFallback(LSCollider2D collider, Vector3d sweepCenter, Vector3d direction)
    {
        Vector3d embeddedCenter = new(collider.Center.x, collider.MixedSlabCenterY, collider.Center.y);
        Vector3d to2D = embeddedCenter - sweepCenter;
        if (to2D.SqrMagnitude > Fixed64.Epsilon)
            return to2D.Normal;

        return direction.SqrMagnitude > Fixed64.Epsilon ? direction.Normal : Vector3d.Down;
    }

    private static bool TrySweepBox(
        Vector3d start,
        Vector3d direction,
        Fixed64 length,
        Vector3d min,
        Vector3d max,
        out Fixed64 distance)
    {
        distance = Fixed64.Zero;
        if (IsInsideBox(start, min, max))
            return true;

        Fixed64 entry = Fixed64.Zero;
        Fixed64 exit = length;
        if (!ClipSegmentAxis(start.x, direction.x, min.x, max.x, ref entry, ref exit)
            || !ClipSegmentAxis(start.y, direction.y, min.y, max.y, ref entry, ref exit)
            || !ClipSegmentAxis(start.z, direction.z, min.z, max.z, ref entry, ref exit))
        {
            return false;
        }

        distance = entry;
        return true;
    }

    private static bool TrySweepCircleSlabSide(
        Vector3d localStart,
        Vector3d direction,
        Fixed64 length,
        Fixed64 radius,
        Fixed64 halfHeight,
        out Fixed64 distance)
    {
        distance = Fixed64.Zero;
        Fixed64 a = direction.x * direction.x + direction.z * direction.z;
        if (a <= Fixed64.Epsilon)
            return false;

        Fixed64 b = 2 * (localStart.x * direction.x + localStart.z * direction.z);
        Fixed64 c = localStart.x * localStart.x + localStart.z * localStart.z - radius * radius;
        Fixed64 discriminant = b * b - 4 * a * c;
        if (discriminant < Fixed64.Zero)
            return false;

        Fixed64 root = FixedMath.Sqrt(discriminant);
        Fixed64 denominator = 2 * a;
        Fixed64 first = (-b - root) / denominator;
        Fixed64 second = (-b + root) / denominator;
        bool found = false;
        Fixed64 best = Fixed64.MAX_VALUE;
        TryKeepEarlierSweep(
            IsCircleSlabSideHit(localStart, direction, length, halfHeight, first),
            first,
            ref found,
            ref best);
        TryKeepEarlierSweep(
            IsCircleSlabSideHit(localStart, direction, length, halfHeight, second),
            second,
            ref found,
            ref best);

        distance = best;
        return found;
    }

    private static bool TrySweepCircleSlabCap(
        Vector3d localStart,
        Vector3d direction,
        Fixed64 length,
        Fixed64 radius,
        Fixed64 capY,
        out Fixed64 distance)
    {
        distance = Fixed64.Zero;
        if (direction.y.Abs() <= Fixed64.Epsilon)
            return false;

        Fixed64 candidate = (capY - localStart.y) / direction.y;
        if (candidate < Fixed64.Zero || candidate > length)
            return false;

        Vector3d localPoint = localStart + direction * candidate;
        Fixed64 radialSqr = localPoint.x * localPoint.x + localPoint.z * localPoint.z;
        if (radialSqr > radius * radius + Fixed64.Epsilon)
            return false;

        distance = candidate;
        return true;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static bool IsCircleSlabSideHit(
        Vector3d localStart,
        Vector3d direction,
        Fixed64 length,
        Fixed64 halfHeight,
        Fixed64 distance)
    {
        if (distance < Fixed64.Zero || distance > length)
            return false;

        Fixed64 y = localStart.y + direction.y * distance;
        return y >= -halfHeight && y <= halfHeight;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static bool IsInsideCircleSlab(Vector3d localPoint, Fixed64 radius, Fixed64 halfHeight) =>
        localPoint.y >= -halfHeight
        && localPoint.y <= halfHeight
        && localPoint.x * localPoint.x + localPoint.z * localPoint.z <= radius * radius;

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static bool IsInsideBox(Vector3d point, Vector3d min, Vector3d max) =>
        point.x >= min.x && point.x <= max.x
        && point.y >= min.y && point.y <= max.y
        && point.z >= min.z && point.z <= max.z;

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static bool ClipSegmentAxis(
        Fixed64 position,
        Fixed64 direction,
        Fixed64 min,
        Fixed64 max,
        ref Fixed64 entry,
        ref Fixed64 exit)
    {
        if (direction.Abs() <= Fixed64.Epsilon)
            return position >= min && position <= max;

        Fixed64 first = (min - position) / direction;
        Fixed64 second = (max - position) / direction;
        if (first > second)
            (first, second) = (second, first);

        if (first > entry)
            entry = first;
        if (second < exit)
            exit = second;
        return entry <= exit;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static void TryKeepEarlierSweep(
        bool candidateFound,
        Fixed64 candidateDistance,
        ref bool found,
        ref Fixed64 bestDistance)
    {
        if (!candidateFound || candidateDistance >= bestDistance)
            return;

        found = true;
        bestDistance = candidateDistance;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static bool IsEligible2DTarget(LSCollider2D collider, LSCollider? excludedCollider, bool includeTriggers)
    {
        if (!collider.IsActive || (!includeTriggers && collider.IsTrigger))
            return false;

        return excludedCollider == null
            || (!ReferenceEquals(collider.AgentOrNull, excludedCollider.AgentOrNull)
                && !excludedCollider.ExcludesMixedCollisionWith(collider));
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static bool IsEligible3DTarget(LSCollider collider, LSCollider2D? excludedCollider, bool includeTriggers)
    {
        if (!collider.IsActive || (!includeTriggers && collider.IsTrigger))
            return false;

        return excludedCollider == null
            || (!ReferenceEquals(collider.AgentOrNull, excludedCollider.AgentOrNull)
                && !collider.ExcludesMixedCollisionWith(excludedCollider));
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static void CreateSweepBounds(Vector3d start, Vector3d end, Fixed64 radius, out Vector3d min, out Vector3d max)
    {
        Vector3d radiusExtents = Vector3d.One * radius;
        min = Vector3d.Min(start, end) - radiusExtents;
        max = Vector3d.Max(start, end) + radiusExtents;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static Fixed64 ClampAxis(Fixed64 value, Fixed64 min, Fixed64 max) =>
        value < min ? min : value > max ? max : value;
}
