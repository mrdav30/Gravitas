//=======================================================================
// SweptSphereQueryWorker.cs
//=======================================================================
// MIT License, Copyright (c) 2026–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp;
using FixedMathSharp.Geometry;
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
    private readonly ConvexSweepQueryWorker _finiteAxisConvexSweep = new();

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
        _finiteAxisConvexSweep.PrepareSphereSource(
            start,
            radius,
            segment);
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
        out Fixed64 impactDistance) =>
        TrySweepSphere(
            _start,
            _end,
            _length,
            center,
            radius,
            radiusExpansion,
            out sphereCenterAtImpact,
            out impactDistance);

    private static bool TrySweepSphere(
        Vector3d start,
        Vector3d end,
        Fixed64 length,
        Vector3d center,
        Fixed64 radius,
        Fixed64 radiusExpansion,
        out Vector3d sphereCenterAtImpact,
        out Fixed64 impactDistance)
    {
        sphereCenterAtImpact = Vector3d.Zero;
        impactDistance = Fixed64.Zero;

        var query = new FixedSegment(start, end);
        if (!query.TryGetSphereIntersectionDistanceInterval(
                new FixedBoundSphere(center, radius),
                radiusExpansion,
                length,
                out Fixed64 distance,
                out _,
                out _,
                out _))
            return false;

        impactDistance = distance;
        sphereCenterAtImpact = query.GetPointAtDistance(distance, length);
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
                capsule.Rotation,
                capsule.AxisLength,
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
        out Fixed64 impactDistance) =>
        TrySweepCanonicalFiniteAxis(
            cylinder,
            out sphereCenterAtImpact,
            out impactDistance);

    private bool TrySweepCone(
        LSConeCollider cone,
        out Vector3d sphereCenterAtImpact,
        out Fixed64 impactDistance) =>
        TrySweepCanonicalFiniteAxis(
            cone,
            out sphereCenterAtImpact,
            out impactDistance);

    private bool TrySweepCanonicalFiniteAxis(
        LSCollider target,
        out Vector3d sphereCenterAtImpact,
        out Fixed64 impactDistance)
    {
        sphereCenterAtImpact = Vector3d.Zero;
        impactDistance = Fixed64.Zero;
        if (!_finiteAxisConvexSweep.TrySweepPreparedSource(
                target,
                out Physics3DHit hit))
        {
            return false;
        }

        Fixed64 distance = _length - hit.Distance <= BoundsTolerance
            ? _length
            : hit.Distance;
        BuildFiniteAxisSweepHit(
            distance,
            out sphereCenterAtImpact,
            out impactDistance);
        return true;
    }

    private bool TrySweepCuboid(
        LSCuboidCollider cuboid,
        out Vector3d sphereCenterAtImpact,
        out Fixed64 impactDistance)
    {
        sphereCenterAtImpact = Vector3d.Zero;
        impactDistance = Fixed64.Zero;

        var query = new FixedSegment(_start, _end);
        if (!query.TryGetSweptSphereOrientedBoxIntersectionDistance(
                cuboid.OrientedBox,
                _radius,
                _length,
                out Fixed64 entry))
        {
            return false;
        }

        BuildFiniteAxisSweepHit(entry, out sphereCenterAtImpact, out impactDistance);
        return true;
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

        var startAnchor = new FixedPointAnchor(
            _start,
            FixedQuaternion.Identity,
            Vector3d.Zero);
        var endAnchor = new FixedPointAnchor(
            _end,
            FixedQuaternion.Identity,
            Vector3d.Zero);
        if (!startAnchor.TryGetLocalPointIn(
                mesh.Mesh.Origin,
                mesh.Mesh.Rotation,
                out Vector3d localStart)
            || !endAnchor.TryGetLocalPointIn(
                mesh.Mesh.Origin,
                mesh.Mesh.Rotation,
                out Vector3d localEnd))
        {
            return false;
        }

        Vector3d localDirection =
            mesh.Mesh.Rotation.Inverse().Rotate(_direction);
        bool found = false;
        for (int i = 0; i < _meshTriangleBuffer.Count; i++)
        {
            int triangleIndex = _meshTriangleBuffer[i];
            mesh.Mesh.GetLocalTriangleVertices(
                triangleIndex,
                out Vector3d first,
                out Vector3d second,
                out Vector3d third);
            Vector3d normal = mesh.Mesh.FaceNormals[triangleIndex];
            found |= TryKeepEarlierSweep(
                TrySweepTriangle(
                    first,
                    second,
                    third,
                    normal,
                    localStart,
                    localEnd,
                    localDirection,
                    out Vector3d triangleHit,
                    out Fixed64 triangleDistance),
                triangleHit,
                triangleDistance,
                ref sphereCenterAtImpact,
                ref impactDistance);
        }

        if (found)
        {
            sphereCenterAtImpact = new FixedSegment(
                _start,
                _end).GetPointAtDistance(
                    impactDistance,
                    _length);
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
        Vector3d start,
        Vector3d end,
        Vector3d direction,
        out Vector3d sphereCenterAtImpact,
        out Fixed64 impactDistance)
    {
        sphereCenterAtImpact = Vector3d.Zero;
        impactDistance = Fixed64.MaxValue;

        var triangle = new FixedTriangle(first, second, third);
        Vector3d closestAtStart = triangle.ClosestPoint(start);
        if (Vector3d.DistanceSquared(start, closestAtStart) <= _radius * _radius)
        {
            sphereCenterAtImpact = start;
            impactDistance = Fixed64.Zero;
            return true;
        }

        bool found = false;
        found |= TryKeepEarlierSweep(
            TrySweepTriangleFace(triangle, normal, _radius, start, direction, out Vector3d frontHit, out Fixed64 frontDistance),
            frontHit,
            frontDistance,
            ref sphereCenterAtImpact,
            ref impactDistance);
        found |= TryKeepEarlierSweep(
            TrySweepTriangleFace(triangle, normal, -_radius, start, direction, out Vector3d backHit, out Fixed64 backDistance),
            backHit,
            backDistance,
            ref sphereCenterAtImpact,
            ref impactDistance);
        found |= TryKeepEarlierSweep(
            TrySweepTriangleEdge(first, second, start, end, out Vector3d firstEdgeHit, out Fixed64 firstEdgeDistance),
            firstEdgeHit,
            firstEdgeDistance,
            ref sphereCenterAtImpact,
            ref impactDistance);
        found |= TryKeepEarlierSweep(
            TrySweepTriangleEdge(second, third, start, end, out Vector3d secondEdgeHit, out Fixed64 secondEdgeDistance),
            secondEdgeHit,
            secondEdgeDistance,
            ref sphereCenterAtImpact,
            ref impactDistance);
        found |= TryKeepEarlierSweep(
            TrySweepTriangleEdge(third, first, start, end, out Vector3d thirdEdgeHit, out Fixed64 thirdEdgeDistance),
            thirdEdgeHit,
            thirdEdgeDistance,
            ref sphereCenterAtImpact,
            ref impactDistance);
        found |= TryKeepEarlierSweep(
            TrySweepSphere(start, end, _length, first, Fixed64.Zero, _radius, out Vector3d firstVertexHit, out Fixed64 firstVertexDistance),
            firstVertexHit,
            firstVertexDistance,
            ref sphereCenterAtImpact,
            ref impactDistance);
        found |= TryKeepEarlierSweep(
            TrySweepSphere(start, end, _length, second, Fixed64.Zero, _radius, out Vector3d secondVertexHit, out Fixed64 secondVertexDistance),
            secondVertexHit,
            secondVertexDistance,
            ref sphereCenterAtImpact,
            ref impactDistance);
        found |= TryKeepEarlierSweep(
            TrySweepSphere(start, end, _length, third, Fixed64.Zero, _radius, out Vector3d thirdVertexHit, out Fixed64 thirdVertexDistance),
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
        Vector3d start,
        Vector3d direction,
        out Vector3d sphereCenterAtImpact,
        out Fixed64 impactDistance)
    {
        sphereCenterAtImpact = Vector3d.Zero;
        impactDistance = Fixed64.Zero;

        Fixed64 denominator = Vector3d.Dot(direction, normal);
        if (denominator.Abs() <= Fixed64.Epsilon)
            return false;

        Fixed64 signedStartDistance = Vector3d.Dot(start - triangle.A, normal);
        Fixed64 distance = (signedRadius - signedStartDistance) / denominator;
        if (distance < Fixed64.Zero || distance > _length)
            return false;

        Vector3d center = start + direction * distance;
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
        Vector3d start,
        Vector3d end,
        out Vector3d sphereCenterAtImpact,
        out Fixed64 impactDistance)
    {
        sphereCenterAtImpact = Vector3d.Zero;
        impactDistance = Fixed64.MaxValue;

        var query = new FixedSegment(start, end);
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

        impactDistance = entry;
        sphereCenterAtImpact = query.GetPointAtDistance(entry, _length);
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

    private static void CreateSweepBounds(Vector3d start, Vector3d end, Fixed64 radius, out Vector3d min, out Vector3d max)
    {
        Vector3d radiusExtents = Vector3d.One * radius;
        min = Vector3d.Min(start, end) - radiusExtents;
        max = Vector3d.Max(start, end) + radiusExtents;
    }
}
