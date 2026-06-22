//=======================================================================
// GravitasQueryMixedService.cs
//=======================================================================
// MIT License, Copyright (c) 2026–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp;
using FixedMathSharp.Bounds;
using Gravitas.Colliders;
using Gravitas.Support;
using SwiftCollections;
using System;
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
        bool includeTriggers = true,
        bool cacheTargetPartitions = false)
    {
        return SweepSphereAgainst2DAllCore(
            start,
            end,
            radius,
            layerMask,
            results,
            excludedCollider,
            includeTriggers,
            staticTargetsOnly: true,
            cacheTargetPartitions);
    }

    private int SweepSphereAgainst2DAllCore(
        Vector3d start,
        Vector3d end,
        Fixed64 radius,
        PhysicsLayerMask layerMask,
        SwiftList<PhysicsMixedHit> results,
        LSCollider? excludedCollider,
        bool includeTriggers,
        bool staticTargetsOnly,
        bool cacheTargetPartitions = false)
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
        _context.MixedCollisions.Collect2DCandidatesInMixedBounds(
            min,
            max,
            layerMask,
            _candidates2D,
            staticStyleOnly: staticTargetsOnly,
            cachePartitionRefresh: cacheTargetPartitions);
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
        bool includeTriggers = true,
        bool cacheTargetPartitions = false)
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
            staticTargetsOnly: true,
            cacheTargetPartitions);
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
        bool staticTargetsOnly,
        bool cacheTargetPartitions = false)
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
        Fixed64 proxyRadius = FixedMath.Sqrt(radius * radius + halfThickness * halfThickness);
        CreateCircleSlabSweepBounds(start, end, radius, slabCenterY, halfThickness, out Vector3d min, out Vector3d max);
        _context.MixedCollisions.Collect3DCandidatesInMixedBounds(
            min,
            max,
            layerMask,
            _candidates3D,
            staticStyleOnly: staticTargetsOnly,
            cachePartitionRefresh: cacheTargetPartitions);
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

        if (collider is LSCuboidCollider cuboid)
        {
            return TrySweepCircleAgainstCuboid(
                start,
                direction2D,
                length,
                radius,
                slabCenterY,
                halfThickness,
                direction3D,
                cuboid,
                sourceCollider,
                out hit);
        }

        if (collider is LSCapsuleCollider capsule)
        {
            bool found = TrySweepCircleAgainstCapsule(
                start,
                direction2D,
                length,
                radius,
                slabCenterY,
                halfThickness,
                direction3D,
                capsule,
                sourceCollider,
                out hit,
                out bool handled);
            if (handled)
                return found;
        }

        if (collider is LSCylinderCollider cylinder)
        {
            bool found = TrySweepCircleAgainstCylinder(
                start,
                direction2D,
                length,
                radius,
                slabCenterY,
                halfThickness,
                direction3D,
                cylinder,
                sourceCollider,
                out hit,
                out bool handled);
            if (handled)
                return found;
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
            PhysicsQueryReducerKind.ConservativeFallback,
            distance,
            sourceCollider);
        return true;
    }

    private static bool TrySweepCircleAgainstCuboid(
        Vector2d start,
        Vector2d direction,
        Fixed64 length,
        Fixed64 radius,
        Fixed64 slabCenterY,
        Fixed64 halfThickness,
        Vector3d direction3D,
        LSCuboidCollider cuboid,
        LSCollider2D? sourceCollider,
        out PhysicsMixedHit hit)
    {
        Span<Vector2d> projection = stackalloc Vector2d[32];
        if (!TryBuildCuboidSlabProjection(cuboid, slabCenterY, halfThickness, projection, out int projectionCount)
            || !TrySweepCircleAgainstConvexProjection(
                start,
                direction,
                length,
                radius,
                projection.Slice(0, projectionCount),
                out Fixed64 distance))
        {
            hit = default;
            return false;
        }

        Vector2d center2D = start + direction * distance;
        Vector3d sweepCenter = new(center2D.X, slabCenterY, center2D.Y);
        hit = BuildCircleAgainst3DHit(
            cuboid,
            sweepCenter,
            direction3D,
            radius,
            slabCenterY,
            halfThickness,
            PhysicsQueryReducerKind.Exact,
            distance,
            sourceCollider);
        return true;
    }

    private static bool TrySweepCircleAgainstCapsule(
        Vector2d start,
        Vector2d direction,
        Fixed64 length,
        Fixed64 radius,
        Fixed64 slabCenterY,
        Fixed64 halfThickness,
        Vector3d direction3D,
        LSCapsuleCollider capsule,
        LSCollider2D? sourceCollider,
        out PhysicsMixedHit hit,
        out bool handled)
    {
        handled = false;
        if (!TryGetVerticalSegmentInterval(capsule.LineSegmentStart, capsule.LineSegmentEnd, out Fixed64 segmentMinY, out Fixed64 segmentMaxY))
        {
            hit = default;
            return false;
        }

        handled = true;
        Fixed64 verticalExcess = GetIntervalDistance(segmentMinY, segmentMaxY, slabCenterY - halfThickness, slabCenterY + halfThickness);
        Fixed64 capsuleRadius = capsule.ScaledRadius;
        if (verticalExcess > capsuleRadius)
        {
            hit = default;
            return false;
        }

        Fixed64 planarRadiusSqr = capsuleRadius * capsuleRadius - verticalExcess * verticalExcess;
        if (planarRadiusSqr < Fixed64.Zero)
            planarRadiusSqr = Fixed64.Zero;

        return TryBuildCircleAgainstPlanarCircleTargetHit(
            start,
            direction,
            length,
            radius,
            slabCenterY,
            halfThickness,
            direction3D,
            capsule,
            new Vector2d(capsule.Center.X, capsule.Center.Z),
            FixedMath.Sqrt(planarRadiusSqr),
            sourceCollider,
            out hit);
    }

    private static bool TrySweepCircleAgainstCylinder(
        Vector2d start,
        Vector2d direction,
        Fixed64 length,
        Fixed64 radius,
        Fixed64 slabCenterY,
        Fixed64 halfThickness,
        Vector3d direction3D,
        LSCylinderCollider cylinder,
        LSCollider2D? sourceCollider,
        out PhysicsMixedHit hit,
        out bool handled)
    {
        handled = false;
        if (!TryGetVerticalSegmentInterval(cylinder.LineSegmentStart, cylinder.LineSegmentEnd, out Fixed64 segmentMinY, out Fixed64 segmentMaxY))
        {
            hit = default;
            return false;
        }

        handled = true;
        if (!IntervalsOverlap(segmentMinY, segmentMaxY, slabCenterY - halfThickness, slabCenterY + halfThickness))
        {
            hit = default;
            return false;
        }

        return TryBuildCircleAgainstPlanarCircleTargetHit(
            start,
            direction,
            length,
            radius,
            slabCenterY,
            halfThickness,
            direction3D,
            cylinder,
            new Vector2d(cylinder.Center.X, cylinder.Center.Z),
            cylinder.ScaledRadius,
            sourceCollider,
            out hit);
    }

    private static bool TryBuildCircleAgainstPlanarCircleTargetHit(
        Vector2d start,
        Vector2d direction,
        Fixed64 length,
        Fixed64 sourceRadius,
        Fixed64 slabCenterY,
        Fixed64 halfThickness,
        Vector3d direction3D,
        LSCollider collider,
        Vector2d targetCenter,
        Fixed64 targetPlanarRadius,
        LSCollider2D? sourceCollider,
        out PhysicsMixedHit hit)
    {
        if (!TrySweepPointInPlane(start, direction, length, targetCenter, sourceRadius + targetPlanarRadius, out Fixed64 distance))
        {
            hit = default;
            return false;
        }

        Vector2d center2D = start + direction * distance;
        Vector3d sweepCenter = new(center2D.X, slabCenterY, center2D.Y);
        hit = BuildCircleAgainst3DHit(
            collider,
            sweepCenter,
            direction3D,
            sourceRadius,
            slabCenterY,
            halfThickness,
            PhysicsQueryReducerKind.Exact,
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
            best.ReducerKind,
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
            PhysicsQueryReducerKind.Exact,
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

    private static bool TryBuildCuboidSlabProjection(
        LSCuboidCollider cuboid,
        Fixed64 slabCenterY,
        Fixed64 halfThickness,
        Span<Vector2d> projection,
        out int projectionCount)
    {
        projectionCount = 0;
        Fixed64 slabMinY = slabCenterY - halfThickness;
        Fixed64 slabMaxY = slabCenterY + halfThickness;
        Vector3d[] vertices = cuboid.Vertices;

        for (int i = 0; i < vertices.Length; i++)
        {
            Vector3d vertex = vertices[i];
            if (vertex.Y >= slabMinY && vertex.Y <= slabMaxY)
                TryAddUniqueProjectionPoint(projection, ref projectionCount, new Vector2d(vertex.X, vertex.Z));
        }

        for (int i = 0; i < LSCuboidCollider.EdgeDefinitions.Length; i++)
        {
            int[] edge = LSCuboidCollider.EdgeDefinitions[i];
            Vector3d first = vertices[edge[0]];
            Vector3d second = vertices[edge[1]];
            TryAddSlabPlaneIntersection(first, second, slabMinY, projection, ref projectionCount);
            if (slabMaxY != slabMinY)
                TryAddSlabPlaneIntersection(first, second, slabMaxY, projection, ref projectionCount);
        }

        if (projectionCount == 0)
            return false;

        BuildConvexHullInPlace(projection, ref projectionCount);
        return projectionCount > 0;
    }

    private static void TryAddSlabPlaneIntersection(
        Vector3d first,
        Vector3d second,
        Fixed64 planeY,
        Span<Vector2d> projection,
        ref int projectionCount)
    {
        Fixed64 deltaY = second.Y - first.Y;
        if (deltaY.Abs() <= Fixed64.Epsilon)
            return;

        Fixed64 t = (planeY - first.Y) / deltaY;
        if (t < Fixed64.Zero || t > Fixed64.One)
            return;

        Vector3d point = first + (second - first) * t;
        TryAddUniqueProjectionPoint(projection, ref projectionCount, new Vector2d(point.X, point.Z));
    }

    private static void TryAddUniqueProjectionPoint(Span<Vector2d> projection, ref int projectionCount, Vector2d point)
    {
        for (int i = 0; i < projectionCount; i++)
        {
            Vector2d delta = projection[i] - point;
            if (delta.MagnitudeSquared <= Fixed64.Epsilon)
                return;
        }

        if (projectionCount < projection.Length)
            projection[projectionCount++] = point;
    }

    private static void BuildConvexHullInPlace(Span<Vector2d> projection, ref int projectionCount)
    {
        if (projectionCount <= 2)
            return;

        SortProjectionPoints(projection.Slice(0, projectionCount));
        Span<Vector2d> hull = stackalloc Vector2d[64];
        int hullCount = 0;

        for (int i = 0; i < projectionCount; i++)
        {
            while (hullCount >= 2 && Cross(hull[hullCount - 2], hull[hullCount - 1], projection[i]) <= Fixed64.Zero)
                hullCount--;

            hull[hullCount++] = projection[i];
        }

        int lowerCount = hullCount;
        for (int i = projectionCount - 2; i >= 0; i--)
        {
            while (hullCount > lowerCount && Cross(hull[hullCount - 2], hull[hullCount - 1], projection[i]) <= Fixed64.Zero)
                hullCount--;

            hull[hullCount++] = projection[i];
        }

        if (hullCount > 1)
            hullCount--;

        for (int i = 0; i < hullCount; i++)
            projection[i] = hull[i];

        projectionCount = hullCount;
    }

    private static void SortProjectionPoints(Span<Vector2d> points)
    {
        for (int i = 1; i < points.Length; i++)
        {
            Vector2d candidate = points[i];
            int j = i - 1;
            while (j >= 0 && ComesAfter(points[j], candidate))
            {
                points[j + 1] = points[j];
                j--;
            }

            points[j + 1] = candidate;
        }
    }

    private static bool TrySweepCircleAgainstConvexProjection(
        Vector2d start,
        Vector2d direction,
        Fixed64 length,
        Fixed64 radius,
        ReadOnlySpan<Vector2d> projection,
        out Fixed64 distance)
    {
        distance = Fixed64.Zero;
        if (projection.Length == 1)
            return TrySweepPointInPlane(start, direction, length, projection[0], radius, out distance);

        if (projection.Length == 2)
            return TrySweepPointAgainstSegmentCapsule(start, direction, length, projection[0], projection[1], radius, out distance);

        Fixed64 radiusSqr = radius * radius;
        if (IsPointInsideConvexProjection(start, projection)
            || DistanceSquaredToConvexProjection(start, projection) <= radiusSqr)
        {
            return true;
        }

        bool found = false;
        Fixed64 best = Fixed64.MaxValue;
        for (int i = 0; i < projection.Length; i++)
        {
            Vector2d first = projection[i];
            Vector2d second = projection[(i + 1) % projection.Length];
            TryKeepEarlierSweep(
                TrySweepPointAgainstSegmentCapsule(start, direction, length, first, second, radius, out Fixed64 candidate),
                candidate,
                ref found,
                ref best);
        }

        distance = best;
        return found;
    }

    private static bool TrySweepPointAgainstSegmentCapsule(
        Vector2d start,
        Vector2d direction,
        Fixed64 length,
        Vector2d segmentStart,
        Vector2d segmentEnd,
        Fixed64 radius,
        out Fixed64 distance)
    {
        distance = Fixed64.Zero;
        Fixed64 radiusSqr = radius * radius;
        if (DistanceSquaredToSegment(start, segmentStart, segmentEnd) <= radiusSqr)
            return true;

        bool found = false;
        Fixed64 best = Fixed64.MaxValue;
        TryKeepEarlierSweep(
            TrySweepPointInPlane(start, direction, length, segmentStart, radius, out Fixed64 startDistance),
            startDistance,
            ref found,
            ref best);
        TryKeepEarlierSweep(
            TrySweepPointInPlane(start, direction, length, segmentEnd, radius, out Fixed64 endDistance),
            endDistance,
            ref found,
            ref best);

        Vector2d edge = segmentEnd - segmentStart;
        Fixed64 edgeLengthSqr = edge.MagnitudeSquared;
        if (edgeLengthSqr > Fixed64.Epsilon)
        {
            Fixed64 edgeLength = FixedMath.Sqrt(edgeLengthSqr);
            Vector2d edgeDirection = edge / edgeLength;
            Vector2d normal = new(-edgeDirection.Y, edgeDirection.X);
            Fixed64 signedStart = Vector2d.Dot(start - segmentStart, normal);
            Fixed64 signedDirection = Vector2d.Dot(direction, normal);
            if (signedDirection.Abs() > Fixed64.Epsilon)
            {
                TryKeepEarlierSweep(
                    TrySweepPointAgainstSegmentOffsetLine(
                        start,
                        direction,
                        length,
                        segmentStart,
                        edgeDirection,
                        edgeLength,
                        signedStart,
                        signedDirection,
                        radius,
                        out Fixed64 positiveDistance),
                    positiveDistance,
                    ref found,
                    ref best);
                TryKeepEarlierSweep(
                    TrySweepPointAgainstSegmentOffsetLine(
                        start,
                        direction,
                        length,
                        segmentStart,
                        edgeDirection,
                        edgeLength,
                        signedStart,
                        signedDirection,
                        -radius,
                        out Fixed64 negativeDistance),
                    negativeDistance,
                    ref found,
                    ref best);
            }
        }

        distance = best;
        return found;
    }

    private static bool TrySweepPointAgainstSegmentOffsetLine(
        Vector2d start,
        Vector2d direction,
        Fixed64 length,
        Vector2d segmentStart,
        Vector2d edgeDirection,
        Fixed64 edgeLength,
        Fixed64 signedStart,
        Fixed64 signedDirection,
        Fixed64 signedRadius,
        out Fixed64 distance)
    {
        distance = (signedRadius - signedStart) / signedDirection;
        if (distance < Fixed64.Zero || distance > length)
            return false;

        Vector2d point = start + direction * distance;
        Fixed64 projection = Vector2d.Dot(point - segmentStart, edgeDirection);
        return projection >= Fixed64.Zero && projection <= edgeLength;
    }

    private static bool IsPointInsideConvexProjection(Vector2d point, ReadOnlySpan<Vector2d> projection)
    {
        bool hasPositive = false;
        bool hasNegative = false;
        for (int i = 0; i < projection.Length; i++)
        {
            Fixed64 cross = Cross(projection[i], projection[(i + 1) % projection.Length], point);
            hasPositive |= cross > Fixed64.Epsilon;
            hasNegative |= cross < -Fixed64.Epsilon;
            if (hasPositive && hasNegative)
                return false;
        }

        return true;
    }

    private static Fixed64 DistanceSquaredToConvexProjection(Vector2d point, ReadOnlySpan<Vector2d> projection)
    {
        Fixed64 best = Fixed64.MaxValue;
        for (int i = 0; i < projection.Length; i++)
        {
            Fixed64 distanceSqr = DistanceSquaredToSegment(point, projection[i], projection[(i + 1) % projection.Length]);
            if (distanceSqr < best)
                best = distanceSqr;
        }

        return best;
    }

    private static Fixed64 DistanceSquaredToSegment(Vector2d point, Vector2d segmentStart, Vector2d segmentEnd)
    {
        Vector2d edge = segmentEnd - segmentStart;
        Fixed64 edgeLengthSqr = edge.MagnitudeSquared;
        if (edgeLengthSqr <= Fixed64.Epsilon)
            return (point - segmentStart).MagnitudeSquared;

        Fixed64 t = Vector2d.Dot(point - segmentStart, edge) / edgeLengthSqr;
        t = FixedMath.Clamp01(t);
        Vector2d closest = segmentStart + edge * t;
        return (point - closest).MagnitudeSquared;
    }

    private static bool TryGetVerticalSegmentInterval(Vector3d start, Vector3d end, out Fixed64 minY, out Fixed64 maxY)
    {
        Vector3d segment = end - start;
        if (segment.X * segment.X + segment.Z * segment.Z > Fixed64.Epsilon)
        {
            minY = Fixed64.Zero;
            maxY = Fixed64.Zero;
            return false;
        }

        minY = FixedMath.Min(start.Y, end.Y);
        maxY = FixedMath.Max(start.Y, end.Y);
        return true;
    }

    private static Fixed64 GetIntervalDistance(Fixed64 firstMin, Fixed64 firstMax, Fixed64 secondMin, Fixed64 secondMax)
    {
        if (firstMax < secondMin)
            return secondMin - firstMax;

        if (secondMax < firstMin)
            return firstMin - secondMax;

        return Fixed64.Zero;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static bool IntervalsOverlap(Fixed64 firstMin, Fixed64 firstMax, Fixed64 secondMin, Fixed64 secondMax) =>
        firstMin <= secondMax && secondMin <= firstMax;

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static bool ComesAfter(Vector2d first, Vector2d second) =>
        first.X > second.X || (first.X == second.X && first.Y > second.Y);

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static Fixed64 Cross(Vector2d origin, Vector2d first, Vector2d second) =>
        (first.X - origin.X) * (second.Y - origin.Y) - (first.Y - origin.Y) * (second.X - origin.X);

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
            hit = BuildSphereAgainst2DHit(
                circle,
                start,
                radius,
                PhysicsQueryReducerKind.Exact,
                Fixed64.Zero,
                direction);
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
        hit = BuildSphereAgainst2DHit(
            circle,
            sweepCenter,
            radius,
            PhysicsQueryReducerKind.Exact,
            bestDistance,
            direction);
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
        hit = BuildSphereAgainst2DHit(
            collider,
            sweepCenter,
            radius,
            PhysicsQueryReducerKind.ConservativeFallback,
            distance,
            direction);
        return true;
    }

    private static PhysicsMixedHit BuildSphereAgainst2DHit(
        LSCollider2D collider,
        Vector3d sweepCenter,
        Fixed64 radius,
        PhysicsQueryReducerKind reducerKind,
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
            reducerKind,
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
        PhysicsQueryReducerKind reducerKind,
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
            reducerKind,
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
