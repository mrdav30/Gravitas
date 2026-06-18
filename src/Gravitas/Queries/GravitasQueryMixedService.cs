using FixedMathSharp;
using FixedMathSharp.Bounds;
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
        return SweepSphereAgainst2DAllCore(
            start,
            end,
            radius,
            layerMask,
            results,
            excludedCollider,
            includeTriggers,
            staticTargetsOnly: false);
    }

    internal int SweepSphereAgainstStatic2DAll(
        Vector3d start,
        Vector3d end,
        Fixed64 radius,
        PhysicsLayerMask layerMask,
        SwiftList<PhysicsMixedHit> results,
        LSCollider? excludedCollider = null,
        bool includeTriggers = true)
    {
        return SweepSphereAgainst2DAllCore(
            start,
            end,
            radius,
            layerMask,
            results,
            excludedCollider,
            includeTriggers,
            staticTargetsOnly: true);
    }

    private int SweepSphereAgainst2DAllCore(
        Vector3d start,
        Vector3d end,
        Fixed64 radius,
        PhysicsLayerMask layerMask,
        SwiftList<PhysicsMixedHit> results,
        LSCollider? excludedCollider,
        bool includeTriggers,
        bool staticTargetsOnly)
    {
        SwiftThrowHelper.ThrowIfNull(results, nameof(results));
        SwiftThrowHelper.ThrowIfArgument(radius <= Fixed64.Zero, nameof(radius), "Mixed swept sphere radius must be greater than zero.");

        results.FastClear();
        Vector3d segment = end - start;
        if (segment.MagnitudeSquared <= Fixed64.Epsilon)
        {
            LastQueryCandidateCount = 0;
            return 0;
        }

        Vector3d direction = segment.Normalized;
        Fixed64 length = segment.Magnitude;
        CreateSweepBounds(start, end, radius, out Vector3d min, out Vector3d max);
        _context.MixedCollisions.Collect2DCandidatesInMixedBounds(min, max, layerMask, _candidates2D);
        LastQueryCandidateCount = _candidates2D.Count;

        for (int i = 0; i < _candidates2D.Count; i++)
        {
            LSCollider2D collider = _candidates2D[i];
            if (IsEligible2DTarget(collider, excludedCollider, includeTriggers, staticTargetsOnly)
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
        return SweepCircleAgainst3DAllCore(
            start,
            end,
            radius,
            slabCenterY,
            halfThickness,
            layerMask,
            results,
            excludedCollider,
            includeTriggers,
            staticTargetsOnly: false);
    }

    internal int SweepCircleAgainstStatic3DAll(
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
        return SweepCircleAgainst3DAllCore(
            start,
            end,
            radius,
            slabCenterY,
            halfThickness,
            layerMask,
            results,
            excludedCollider,
            includeTriggers,
            staticTargetsOnly: true);
    }

    private int SweepCircleAgainst3DAllCore(
        Vector2d start,
        Vector2d end,
        Fixed64 radius,
        Fixed64 slabCenterY,
        Fixed64 halfThickness,
        PhysicsLayerMask layerMask,
        SwiftList<PhysicsMixedHit> results,
        LSCollider2D? excludedCollider,
        bool includeTriggers,
        bool staticTargetsOnly)
    {
        SwiftThrowHelper.ThrowIfNull(results, nameof(results));
        SwiftThrowHelper.ThrowIfArgument(radius <= Fixed64.Zero, nameof(radius), "Mixed swept circle radius must be greater than zero.");
        SwiftThrowHelper.ThrowIfArgument(halfThickness <= Fixed64.Zero, nameof(halfThickness), "Mixed swept circle half-thickness must be greater than zero.");

        results.FastClear();
        Vector2d segment = end - start;
        if (segment.MagnitudeSquared <= Fixed64.Epsilon)
        {
            LastQueryCandidateCount = 0;
            return 0;
        }

        Vector3d start3D = new(start.X, slabCenterY, start.Y);
        Vector3d end3D = new(end.X, slabCenterY, end.Y);
        Fixed64 length = segment.Magnitude;
        Vector2d direction2D = segment / length;
        Vector3d direction = new(direction2D.X, Fixed64.Zero, direction2D.Y);
        Fixed64 proxyRadius = FixedMath.Max(radius, halfThickness);
        CreateCircleSlabSweepBounds(start, end, radius, slabCenterY, halfThickness, out Vector3d min, out Vector3d max);
        _context.MixedCollisions.Collect3DCandidatesInMixedBounds(min, max, layerMask, _candidates3D);
        LastQueryCandidateCount = _candidates3D.Count;
        _sweepWorker.Prepare(start3D, end3D, proxyRadius);

        for (int i = 0; i < _candidates3D.Count; i++)
        {
            LSCollider collider = _candidates3D[i];
            if (!IsEligible3DTarget(collider, excludedCollider, includeTriggers, staticTargetsOnly)
                || !TrySweepCircleAgainst3DCollider(
                    collider,
                    start,
                    direction2D,
                    length,
                    radius,
                    slabCenterY,
                    halfThickness,
                    direction,
                    excludedCollider,
                    out PhysicsMixedHit candidate))
            {
                continue;
            }

            results.Add(candidate);
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

    private bool TrySweepCircleAgainst3DCollider(
        LSCollider collider,
        Vector2d start,
        Vector2d direction2D,
        Fixed64 length,
        Fixed64 radius,
        Fixed64 slabCenterY,
        Fixed64 halfThickness,
        Vector3d direction3D,
        LSCollider2D? sourceCollider,
        out PhysicsMixedHit hit)
    {
        if (collider is LSSphereCollider sphere)
        {
            return TrySweepCircleAgainstSphere(
                start,
                direction2D,
                length,
                radius,
                slabCenterY,
                halfThickness,
                direction3D,
                sphere,
                sourceCollider,
                out hit);
        }

        if (!_sweepWorker.TrySweep(collider, out Vector3d centerAtImpact, out Fixed64 distance))
        {
            hit = default;
            return false;
        }

        hit = BuildCircleAgainst3DHit(
            collider,
            centerAtImpact,
            direction3D,
            radius,
            slabCenterY,
            halfThickness,
            distance,
            sourceCollider);
        return true;
    }

    private static bool TrySweepSphereAgainst2D(
        Vector3d start,
        Vector3d direction,
        Fixed64 length,
        Fixed64 radius,
        LSCollider2D collider,
        out PhysicsMixedHit hit)
    {
        if (collider is LSCompoundCollider2D compound)
            return TrySweepSphereAgainstCompound2D(start, direction, length, radius, compound, out hit);

        if (collider is LSCircleCollider2D circle)
            return TrySweepSphereAgainstCircleSlab(start, direction, length, radius, circle, out hit);

        return TrySweepSphereAgainstPrismBounds(start, direction, length, radius, collider, out hit);
    }

    private static bool TrySweepSphereAgainstCompound2D(
        Vector3d start,
        Vector3d direction,
        Fixed64 length,
        Fixed64 radius,
        LSCompoundCollider2D compound,
        out PhysicsMixedHit hit)
    {
        bool found = false;
        PhysicsMixedHit best = default;

        for (int i = 0; i < compound.PartCount; i++)
        {
            LSCollider2D part = compound.GetPartCollider(i);
            if (!TrySweepSphereAgainst2D(start, direction, length, radius, part, out PhysicsMixedHit candidate))
                continue;

            if (!found || PhysicsMixedHitSorter.ComesBefore(candidate, best))
            {
                best = candidate;
                found = true;
            }
        }

        if (!found)
        {
            hit = default;
            return false;
        }

        hit = new PhysicsMixedHit(
            null,
            compound,
            best.Point3D,
            best.Point2D,
            best.Normal3DTo2D,
            best.Distance,
            best.Direction3D);
        return true;
    }

    private static bool TrySweepCircleAgainstSphere(
        Vector2d start,
        Vector2d direction,
        Fixed64 length,
        Fixed64 radius,
        Fixed64 slabCenterY,
        Fixed64 halfThickness,
        Vector3d direction3D,
        LSSphereCollider sphere,
        LSCollider2D? sourceCollider,
        out PhysicsMixedHit hit)
    {
        Fixed64 sphereRadius = sphere.ScaledRadius;
        Fixed64 verticalExcess = (sphere.Center.Y - slabCenterY).Abs() - halfThickness;
        if (verticalExcess < Fixed64.Zero)
            verticalExcess = Fixed64.Zero;
        if (verticalExcess > sphereRadius)
        {
            hit = default;
            return false;
        }

        Fixed64 planarSphereRadiusSqr = sphereRadius * sphereRadius - verticalExcess * verticalExcess;
        if (planarSphereRadiusSqr < Fixed64.Zero)
            planarSphereRadiusSqr = Fixed64.Zero;

        Fixed64 combinedPlanarRadius = radius + FixedMath.Sqrt(planarSphereRadiusSqr);
        Vector2d sphereCenter = new(sphere.Center.X, sphere.Center.Z);
        if (!TrySweepPointInPlane(start, direction, length, sphereCenter, combinedPlanarRadius, out Fixed64 distance))
        {
            hit = default;
            return false;
        }

        Vector2d center2D = start + direction * distance;
        Vector3d sweepCenter = new(center2D.X, slabCenterY, center2D.Y);
        hit = BuildCircleAgainst3DHit(
            sphere,
            sweepCenter,
            direction3D,
            radius,
            slabCenterY,
            halfThickness,
            distance,
            sourceCollider);
        return true;
    }

    private static bool TrySweepPointInPlane(
        Vector2d start,
        Vector2d direction,
        Fixed64 length,
        Vector2d point,
        Fixed64 radius,
        out Fixed64 distance)
    {
        Fixed64 radiusSqr = radius * radius;
        Vector2d startToPoint = start - point;
        if (startToPoint.MagnitudeSquared <= radiusSqr)
        {
            distance = Fixed64.Zero;
            return true;
        }

        Fixed64 b = Vector2d.Dot(startToPoint, direction);
        Fixed64 c = startToPoint.MagnitudeSquared - radiusSqr;
        if (c > Fixed64.Zero && b > Fixed64.Zero)
        {
            distance = default;
            return false;
        }

        Fixed64 discriminant = b * b - c;
        if (discriminant < Fixed64.Zero)
        {
            distance = default;
            return false;
        }

        distance = -b - FixedMath.Sqrt(discriminant);
        if (distance < Fixed64.Zero)
            distance = Fixed64.Zero;
        return distance <= length;
    }

    private static bool TrySweepSphereAgainstCircleSlab(
        Vector3d start,
        Vector3d direction,
        Fixed64 length,
        Fixed64 radius,
        LSCircleCollider2D circle,
        out PhysicsMixedHit hit)
    {
        Vector3d center = new(circle.Center.X, circle.MixedSlabCenterY, circle.Center.Y);
        Fixed64 combinedRadius = circle.ScaledRadius + radius;
        Fixed64 expandedHalfHeight = circle.MixedHalfThickness + radius;
        Vector3d localStart = start - center;

        if (IsInsideCircleSlab(localStart, combinedRadius, expandedHalfHeight))
        {
            hit = BuildSphereAgainst2DHit(circle, start, radius, Fixed64.Zero, direction);
            return true;
        }

        bool found = false;
        Fixed64 bestDistance = Fixed64.MaxValue;
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
        FixedBoundBox bounds = collider.MixedBounds3D;
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
        Vector3d normal3DTo2D = to2D.MagnitudeSquared > Fixed64.Epsilon
            ? to2D.Normalized
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
        Vector3d normal3DTo2D = to2D.MagnitudeSquared > Fixed64.Epsilon
            ? to2D.Normalized
            : direction.MagnitudeSquared > Fixed64.Epsilon ? -direction.Normalized : Vector3d.Right;
        Vector2d planarNormal = new(normal3DTo2D.X, normal3DTo2D.Z);
        Vector2d planarPoint = new(sweepCenter.X, sweepCenter.Z);
        if (planarNormal.MagnitudeSquared > Fixed64.Epsilon)
            planarPoint -= planarNormal.Normalized * radius;

        Vector3d point2D = new(
            planarPoint.X,
            ClampAxis(point3D.Y, slabCenterY - halfThickness, slabCenterY + halfThickness),
            planarPoint.Y);
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
        Vector2d closest2D = collider.GetClosestPoint(new Vector2d(sweepCenter.X, sweepCenter.Z));
        return new Vector3d(
            closest2D.X,
            ClampAxis(
                sweepCenter.Y,
                collider.MixedSlabCenterY - collider.MixedHalfThickness,
                collider.MixedSlabCenterY + collider.MixedHalfThickness),
            closest2D.Y);
    }

    private static Vector3d GetSweepSurfacePoint(LSCollider collider, Vector3d sweepCenter, Vector3d direction)
    {
        Vector3d centerDelta = sweepCenter - collider.Center;
        if (centerDelta.MagnitudeSquared <= Fixed64.Epsilon)
            return collider.Center - direction * collider.ScaledRadius;

        return collider.ClosestPointOnSurface(sweepCenter);
    }

    private static Vector3d Resolve3DTo2DFallback(LSCollider2D collider, Vector3d sweepCenter, Vector3d direction)
    {
        Vector3d embeddedCenter = new(collider.Center.X, collider.MixedSlabCenterY, collider.Center.Y);
        Vector3d to2D = embeddedCenter - sweepCenter;
        if (to2D.MagnitudeSquared > Fixed64.Epsilon)
            return to2D.Normalized;

        return direction.MagnitudeSquared > Fixed64.Epsilon ? direction.Normalized : Vector3d.Down;
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
        if (!ClipSegmentAxis(start.X, direction.X, min.X, max.X, ref entry, ref exit)
            || !ClipSegmentAxis(start.Y, direction.Y, min.Y, max.Y, ref entry, ref exit)
            || !ClipSegmentAxis(start.Z, direction.Z, min.Z, max.Z, ref entry, ref exit))
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
        Fixed64 a = direction.X * direction.X + direction.Z * direction.Z;
        if (a <= Fixed64.Epsilon)
            return false;

        Fixed64 b = 2 * (localStart.X * direction.X + localStart.Z * direction.Z);
        Fixed64 c = localStart.X * localStart.X + localStart.Z * localStart.Z - radius * radius;
        Fixed64 discriminant = b * b - 4 * a * c;
        if (discriminant < Fixed64.Zero)
            return false;

        Fixed64 root = FixedMath.Sqrt(discriminant);
        Fixed64 denominator = 2 * a;
        Fixed64 first = (-b - root) / denominator;
        Fixed64 second = (-b + root) / denominator;
        bool found = false;
        Fixed64 best = Fixed64.MaxValue;
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
        if (direction.Y.Abs() <= Fixed64.Epsilon)
            return false;

        Fixed64 candidate = (capY - localStart.Y) / direction.Y;
        if (candidate < Fixed64.Zero || candidate > length)
            return false;

        Vector3d localPoint = localStart + direction * candidate;
        Fixed64 radialSqr = localPoint.X * localPoint.X + localPoint.Z * localPoint.Z;
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

        Fixed64 y = localStart.Y + direction.Y * distance;
        return y >= -halfHeight && y <= halfHeight;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static bool IsInsideCircleSlab(Vector3d localPoint, Fixed64 radius, Fixed64 halfHeight) =>
        localPoint.Y >= -halfHeight
        && localPoint.Y <= halfHeight
        && localPoint.X * localPoint.X + localPoint.Z * localPoint.Z <= radius * radius;

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static bool IsInsideBox(Vector3d point, Vector3d min, Vector3d max) =>
        point.X >= min.X && point.X <= max.X
        && point.Y >= min.Y && point.Y <= max.Y
        && point.Z >= min.Z && point.Z <= max.Z;

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
    private static bool IsEligible2DTarget(
        LSCollider2D collider,
        LSCollider? excludedCollider,
        bool includeTriggers,
        bool staticTargetsOnly)
    {
        if (!collider.IsActive || (!includeTriggers && collider.IsTrigger))
            return false;

        if (staticTargetsOnly)
        {
            StiffBody2D? body = collider.Body;
            if (body != null && !body.Immovable && !body.IsKinematic)
                return false;
        }

        return excludedCollider == null
            || (!ReferenceEquals(collider.AgentOrNull, excludedCollider.AgentOrNull)
                && !excludedCollider.ExcludesMixedCollisionWith(collider));
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static bool IsEligible3DTarget(
        LSCollider collider,
        LSCollider2D? excludedCollider,
        bool includeTriggers,
        bool staticTargetsOnly)
    {
        if (!collider.IsActive || (!includeTriggers && collider.IsTrigger))
            return false;

        if (staticTargetsOnly)
        {
            StiffBody? body = collider.Body;
            if (body != null && !body.Immovable && !body.IsKinematic)
                return false;
        }

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
    private static void CreateCircleSlabSweepBounds(
        Vector2d start,
        Vector2d end,
        Fixed64 radius,
        Fixed64 slabCenterY,
        Fixed64 halfThickness,
        out Vector3d min,
        out Vector3d max)
    {
        min = new Vector3d(
            FixedMath.Min(start.X, end.X) - radius,
            slabCenterY - halfThickness,
            FixedMath.Min(start.Y, end.Y) - radius);
        max = new Vector3d(
            FixedMath.Max(start.X, end.X) + radius,
            slabCenterY + halfThickness,
            FixedMath.Max(start.Y, end.Y) + radius);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static Fixed64 ClampAxis(Fixed64 value, Fixed64 min, Fixed64 max) =>
        value < min ? min : value > max ? max : value;
}
