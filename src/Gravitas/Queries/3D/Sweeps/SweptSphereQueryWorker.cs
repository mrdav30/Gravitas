//=======================================================================
// SweptSphereQueryWorker.cs
//=======================================================================
// MIT License, Copyright (c) 2026–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp;
using FixedMathSharp.Bounds;
using Gravitas.Colliders;
using Gravitas.CollisionHandling;
using SwiftCollections;
using SwiftCollections.Query;

namespace Gravitas.Queries;

/// <summary>
/// Performs deterministic swept-sphere checks for one prepared segment.
/// </summary>
public sealed class SweptSphereQueryWorker
{
    private static readonly Fixed64 BoundsTolerance = Fixed64.FromFraction(1, 4096);

    private readonly SwiftList<int> _meshTriangleBuffer = new();

    private Vector3d _start;
    private Vector3d _end;
    private Vector3d _direction;
    private Vector3d _sweptBoundsMin;
    private Vector3d _sweptBoundsMax;
    private Fixed64 _length;
    private Fixed64 _lengthSqr;
    private Fixed64 _radius;
    private int _sweepDepth;

    internal int LastMeshTriangleCandidateCount { get; private set; }

    internal Fixed64 Radius => _radius;

    /// <summary>
    /// Prepares this worker for a swept sphere from <paramref name="start"/> to <paramref name="end"/>.
    /// </summary>
    public void Prepare(Vector3d start, Vector3d end, Fixed64 radius)
    {
        _start = start;
        _end = end;
        _radius = radius;

        if (!Vector3d.TrySubtract(end, start, out Vector3d segment)
            || !Vector3d.TryGetMagnitude(segment, out _length))
        {
            _length = Fixed64.Zero;
            _lengthSqr = Fixed64.Zero;
            _direction = Vector3d.Zero;
            return;
        }

        _lengthSqr = segment.MagnitudeSquared;
        _direction = _length <= Fixed64.Epsilon ? Vector3d.Zero : segment.Normalized;
        SweepBoundsUtility.CreateSweptSphereBounds(
            start,
            end,
            radius,
            BoundsTolerance,
            out _sweptBoundsMin,
            out _sweptBoundsMax);
    }

    /// <summary>
    /// Checks a collider against the prepared swept sphere.
    /// </summary>
    public bool TrySweep(
        LSCollider collider,
        out Vector3d sphereCenterAtImpact,
        out Fixed64 impactDistance)
    {
        if (_sweepDepth == 0)
            LastMeshTriangleCandidateCount = 0;

        _sweepDepth++;
        sphereCenterAtImpact = Vector3d.Zero;
        impactDistance = Fixed64.Zero;

        try
        {
            if (_length <= Fixed64.Epsilon
                || !SweepBoundsUtility.OverlapsInclusive(_sweptBoundsMin, _sweptBoundsMax, collider.BoundsMin, collider.BoundsMax))
            {
                return false;
            }

            bool found = collider switch
            {
                LSSphereCollider sphere => TrySweepSphere(sphere.Center, sphere.ScaledRadius, _radius, out sphereCenterAtImpact, out impactDistance),
                LSCapsuleCollider capsule => TrySweepCapsule(capsule, out sphereCenterAtImpact, out impactDistance),
                LSCuboidCollider cuboid => TrySweepCuboid(cuboid, out sphereCenterAtImpact, out impactDistance),
                LSCylinderCollider cylinder => TrySweepCylinder(cylinder, out sphereCenterAtImpact, out impactDistance),
                LSConeCollider cone => TrySweepCone(cone, out sphereCenterAtImpact, out impactDistance),
                LSMeshCollider mesh => TrySweepMesh(mesh, out sphereCenterAtImpact, out impactDistance),
                LSCompoundCollider compound => TrySweepCompound(compound, out sphereCenterAtImpact, out impactDistance),
                _ => false
            };

            if (!found)
            {
                sphereCenterAtImpact = Vector3d.Zero;
                impactDistance = Fixed64.Zero;
            }

            return found;
        }
        finally
        {
            _sweepDepth--;
        }
    }

    private bool TrySweepSphere(
        Vector3d center,
        Fixed64 radius,
        Fixed64 radiusExpansion,
        out Vector3d sphereCenterAtImpact,
        out Fixed64 impactDistance)
    {
        sphereCenterAtImpact = Vector3d.Zero;
        impactDistance = Fixed64.Zero;

        if (!RadialSweepAdmission.TryIntersect(
                _start,
                _direction,
                _length,
                center,
                radius,
                radiusExpansion,
                _end,
                center,
                out Fixed64 parameter))
            return false;

        impactDistance = parameter;
        sphereCenterAtImpact = parameter == _length
            ? _end
            : _start + _direction * parameter;
        return true;
    }

    private bool TrySweepCapsule(
        LSCapsuleCollider capsule,
        out Vector3d sphereCenterAtImpact,
        out Fixed64 impactDistance)
    {
        sphereCenterAtImpact = Vector3d.Zero;
        impactDistance = Fixed64.Zero;

        var query = new FixedSegment(_start, _end);
        if (!query.TryGetCapsuleIntersectionDistanceInterval(
                capsule.Center,
                capsule.WorldAxis,
                capsule.AxisHalfLength,
                capsule.ScaledRadius,
                _radius,
                _length,
                out Fixed64 entry,
                out _,
                out _,
                out _))
        {
            return false;
        }

        BuildFiniteAxisSweepHit(entry, out sphereCenterAtImpact, out impactDistance);
        return true;
    }

    private bool TrySweepCylinder(
        LSCylinderCollider cylinder,
        out Vector3d sphereCenterAtImpact,
        out Fixed64 impactDistance)
    {
        sphereCenterAtImpact = Vector3d.Zero;
        impactDistance = Fixed64.Zero;

        var query = new FixedSegment(_start, _end);
        if (!query.TryGetFiniteCylinderIntersectionDistanceInterval(
                cylinder.Center,
                cylinder.WorldAxis,
                cylinder.HalfHeight,
                cylinder.ScaledRadius,
                _radius,
                _radius,
                _length,
                out Fixed64 entry,
                out _,
                out _,
                out _))
        {
            return false;
        }

        BuildFiniteAxisSweepHit(entry, out sphereCenterAtImpact, out impactDistance);
        return true;
    }

    private bool TrySweepCone(
        LSConeCollider cone,
        out Vector3d sphereCenterAtImpact,
        out Fixed64 impactDistance)
    {
        sphereCenterAtImpact = Vector3d.Zero;
        impactDistance = Fixed64.Zero;

        if (GetSphereConeSeparation(cone, _start) <= Fixed64.Zero)
        {
            sphereCenterAtImpact = _start;
            return true;
        }

        Fixed64 travel = Fixed64.Zero;
        Fixed64 previousTravel = Fixed64.Zero;

        for (int i = 0; i < 32; i++)
        {
            Vector3d center = _start + _direction * travel;
            Fixed64 separation = GetSphereConeSeparation(cone, center);
            if (separation <= BoundsTolerance)
            {
                impactDistance = RefineSphereConeImpact(cone, previousTravel, travel);
                sphereCenterAtImpact = _start + _direction * impactDistance;
                return true;
            }

            Vector3d closest = cone.ClosestPointOnSurface(center);
            Vector3d toCenter = center - closest;
            Fixed64 distance = toCenter.Magnitude;
            // Positive separation implies a non-zero closest-surface delta for
            // a valid non-negative swept radius.
            Vector3d normal = toCenter / distance;
            Fixed64 closingSpeed = -Vector3d.Dot(_direction, normal);
            Fixed64 step = closingSpeed > Fixed64.Epsilon
                ? separation / closingSpeed
                : _length * Fixed64.FromFraction(1, 32);

            if (step <= BoundsTolerance)
                step = BoundsTolerance;

            previousTravel = travel;
            travel += step;
            if (travel > _length)
                break;
        }

        return false;
    }

    private bool TrySweepCuboid(
        LSCuboidCollider cuboid,
        out Vector3d sphereCenterAtImpact,
        out Fixed64 impactDistance)
    {
        FixedQuaternion inverseRotation = cuboid.Rotation.Inverse();
        Vector3d localStart = (_start - cuboid.Center) * inverseRotation;
        Vector3d localDirection = _direction * inverseRotation;
        Vector3d halfExtents = cuboid.ScaledSize * Fixed64.Half;

        Vector3d min = -halfExtents - Vector3d.One * _radius;
        Vector3d max = halfExtents + Vector3d.One * _radius;
        return TrySweepLocalBox(
            cuboid.Center,
            cuboid.Rotation,
            localStart,
            localDirection,
            min,
            max,
            out sphereCenterAtImpact,
            out impactDistance);
    }

    private bool TrySweepMesh(
        LSMeshCollider mesh,
        out Vector3d sphereCenterAtImpact,
        out Fixed64 impactDistance)
    {
        sphereCenterAtImpact = Vector3d.Zero;
        impactDistance = Fixed64.MaxValue;

        CreateSweepBounds(_start, _end, _radius, out Vector3d min, out Vector3d max);
        mesh.GetTrianglesInBounds(new FixedBoundVolume(min, max), _meshTriangleBuffer);
        LastMeshTriangleCandidateCount += _meshTriangleBuffer.Count;

        bool found = false;
        for (int i = 0; i < _meshTriangleBuffer.Count; i++)
        {
            int triangleIndex = _meshTriangleBuffer[i];
            mesh.Mesh.GetTriangleVertices(triangleIndex, out Vector3d first, out Vector3d second, out Vector3d third);
            Vector3d normal = mesh.Mesh.GetFaceNormalWorld(triangleIndex);
            found |= TryKeepEarlierSweep(
                TrySweepTriangle(first, second, third, normal.Normalized, out Vector3d triangleHit, out Fixed64 triangleDistance),
                triangleHit,
                triangleDistance,
                ref sphereCenterAtImpact,
                ref impactDistance);
        }

        return found;
    }

    private bool TrySweepCompound(
        LSCompoundCollider compound,
        out Vector3d sphereCenterAtImpact,
        out Fixed64 impactDistance)
    {
        sphereCenterAtImpact = Vector3d.Zero;
        impactDistance = Fixed64.MaxValue;

        bool found = false;
        for (int i = 0; i < compound.PartCount; i++)
        {
            found |= TryKeepEarlierSweep(
                TrySweep(compound.GetPartCollider(i), out Vector3d partHit, out Fixed64 partDistance),
                partHit,
                partDistance,
                ref sphereCenterAtImpact,
                ref impactDistance);
        }

        return found;
    }

    private bool TrySweepTriangle(
        Vector3d first,
        Vector3d second,
        Vector3d third,
        Vector3d normal,
        out Vector3d sphereCenterAtImpact,
        out Fixed64 impactDistance)
    {
        sphereCenterAtImpact = Vector3d.Zero;
        impactDistance = Fixed64.MaxValue;

        var triangle = new FixedTriangle(first, second, third);
        Vector3d closestAtStart = triangle.ClosestPoint(_start);
        if (Vector3d.DistanceSquared(_start, closestAtStart) <= _radius * _radius)
        {
            sphereCenterAtImpact = _start;
            impactDistance = Fixed64.Zero;
            return true;
        }

        bool found = false;
        found |= TryKeepEarlierSweep(
            TrySweepTriangleFace(triangle, normal, _radius, out Vector3d frontHit, out Fixed64 frontDistance),
            frontHit,
            frontDistance,
            ref sphereCenterAtImpact,
            ref impactDistance);
        found |= TryKeepEarlierSweep(
            TrySweepTriangleFace(triangle, normal, -_radius, out Vector3d backHit, out Fixed64 backDistance),
            backHit,
            backDistance,
            ref sphereCenterAtImpact,
            ref impactDistance);
        found |= TryKeepEarlierSweep(
            TrySweepTriangleEdge(first, second, out Vector3d firstEdgeHit, out Fixed64 firstEdgeDistance),
            firstEdgeHit,
            firstEdgeDistance,
            ref sphereCenterAtImpact,
            ref impactDistance);
        found |= TryKeepEarlierSweep(
            TrySweepTriangleEdge(second, third, out Vector3d secondEdgeHit, out Fixed64 secondEdgeDistance),
            secondEdgeHit,
            secondEdgeDistance,
            ref sphereCenterAtImpact,
            ref impactDistance);
        found |= TryKeepEarlierSweep(
            TrySweepTriangleEdge(third, first, out Vector3d thirdEdgeHit, out Fixed64 thirdEdgeDistance),
            thirdEdgeHit,
            thirdEdgeDistance,
            ref sphereCenterAtImpact,
            ref impactDistance);
        found |= TryKeepEarlierSweep(
            TrySweepSphere(first, Fixed64.Zero, _radius, out Vector3d firstVertexHit, out Fixed64 firstVertexDistance),
            firstVertexHit,
            firstVertexDistance,
            ref sphereCenterAtImpact,
            ref impactDistance);
        found |= TryKeepEarlierSweep(
            TrySweepSphere(second, Fixed64.Zero, _radius, out Vector3d secondVertexHit, out Fixed64 secondVertexDistance),
            secondVertexHit,
            secondVertexDistance,
            ref sphereCenterAtImpact,
            ref impactDistance);
        found |= TryKeepEarlierSweep(
            TrySweepSphere(third, Fixed64.Zero, _radius, out Vector3d thirdVertexHit, out Fixed64 thirdVertexDistance),
            thirdVertexHit,
            thirdVertexDistance,
            ref sphereCenterAtImpact,
            ref impactDistance);

        return found;
    }

    private bool TrySweepTriangleFace(
        FixedTriangle triangle,
        Vector3d normal,
        Fixed64 signedRadius,
        out Vector3d sphereCenterAtImpact,
        out Fixed64 impactDistance)
    {
        sphereCenterAtImpact = Vector3d.Zero;
        impactDistance = Fixed64.Zero;

        Fixed64 denominator = Vector3d.Dot(_direction, normal);
        if (denominator.Abs() <= Fixed64.Epsilon)
            return false;

        Fixed64 signedStartDistance = Vector3d.Dot(_start - triangle.A, normal);
        Fixed64 distance = (signedRadius - signedStartDistance) / denominator;
        if (distance < Fixed64.Zero || distance > _length)
            return false;

        Vector3d center = _start + _direction * distance;
        Vector3d pointOnPlane = center - normal * signedRadius;
        if (!triangle.ContainsProjection(pointOnPlane))
            return false;

        sphereCenterAtImpact = center;
        impactDistance = distance;
        return true;
    }

    private bool TrySweepTriangleEdge(
        Vector3d edgeStart,
        Vector3d edgeEnd,
        out Vector3d sphereCenterAtImpact,
        out Fixed64 impactDistance)
    {
        sphereCenterAtImpact = Vector3d.Zero;
        impactDistance = Fixed64.MaxValue;

        var query = new FixedSegment(_start, _end);
        var edgeAxis = new FixedSegment(edgeStart, edgeEnd);
        if (!query.TryGetFiniteCylinderIntersectionDistanceInterval(
                edgeAxis,
                Fixed64.Zero,
                _radius,
                _length,
                out Fixed64 entry,
                out _,
                out _,
                out _))
        {
            return false;
        }

        BuildFiniteAxisSweepHit(entry, out sphereCenterAtImpact, out impactDistance);
        return true;
    }

    private bool TrySweepLocalBox(
        Vector3d center,
        FixedQuaternion rotation,
        Vector3d localStart,
        Vector3d localDirection,
        Vector3d min,
        Vector3d max,
        out Vector3d sphereCenterAtImpact,
        out Fixed64 impactDistance)
    {
        sphereCenterAtImpact = Vector3d.Zero;
        impactDistance = Fixed64.Zero;

        if (SweepBoundsUtility.OverlapsInclusive(localStart, localStart, min, max))
        {
            sphereCenterAtImpact = _start;
            return true;
        }

        if (!SweepBoundsUtility.TryClipSegment(
            localStart,
            localDirection,
            _length,
            min,
            max,
            out Fixed64 entry,
            out _))
        {
            return false;
        }

        impactDistance = entry;
        sphereCenterAtImpact = center + rotation * (localStart + localDirection * entry);
        return true;
    }

    private void BuildFiniteAxisSweepHit(
        Fixed64 distance,
        out Vector3d sphereCenterAtImpact,
        out Fixed64 impactDistance)
    {
        impactDistance = distance;
        sphereCenterAtImpact = new FixedSegment(_start, _end)
            .GetPointAtDistance(distance, _length);
    }

    private static bool TryKeepEarlierSweep(
        bool candidateFound,
        Vector3d candidateCenter,
        Fixed64 candidateDistance,
        ref Vector3d sphereCenterAtImpact,
        ref Fixed64 impactDistance)
    {
        if (!candidateFound || candidateDistance >= impactDistance)
            return false;

        sphereCenterAtImpact = candidateCenter;
        impactDistance = candidateDistance;
        return true;
    }

    private Fixed64 GetSphereConeSeparation(LSConeCollider cone, Vector3d center)
    {
        Vector3d closest = cone.ClosestPointOnSurface(center);
        Fixed64 distance = Vector3d.Distance(center, closest);
        return distance - _radius;
    }

    private Fixed64 RefineSphereConeImpact(LSConeCollider cone, Fixed64 lower, Fixed64 upper)
    {
        Fixed64 low = FixedMath.Clamp(lower, Fixed64.Zero, _length);
        Fixed64 high = FixedMath.Clamp(upper, Fixed64.Zero, _length);

        for (int i = 0; i < 16; i++)
        {
            Fixed64 mid = (low + high) * Fixed64.Half;
            Vector3d center = _start + _direction * mid;
            if (GetSphereConeSeparation(cone, center) <= BoundsTolerance)
                high = mid;
            else
                low = mid;
        }

        return high;
    }

    private static void CreateSweepBounds(Vector3d start, Vector3d end, Fixed64 radius, out Vector3d min, out Vector3d max)
    {
        Vector3d radiusExtents = Vector3d.One * radius;
        min = Vector3d.Min(start, end) - radiusExtents;
        max = Vector3d.Max(start, end) + radiusExtents;
    }
}
