//=======================================================================
// SweptSphereQueryWorker.cs
//=======================================================================
// MIT License, Copyright (c) 2026–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp;
using Gravitas.Colliders;
using SwiftCollections;
using SwiftCollections.Query;

namespace Gravitas.Queries;

/// <summary>
/// Performs deterministic swept-sphere checks for one prepared segment.
/// </summary>
public sealed class SweptSphereQueryWorker
{
    private readonly SwiftList<int> _meshTriangleBuffer = new();

    private Vector3d _start;
    private Vector3d _end;
    private Vector3d _direction;
    private Fixed64 _length;
    private Fixed64 _lengthSqr;
    private Fixed64 _radius;

    /// <summary>
    /// Prepares this worker for a swept sphere from <paramref name="start"/> to <paramref name="end"/>.
    /// </summary>
    public void Prepare(Vector3d start, Vector3d end, Fixed64 radius)
    {
        _start = start;
        _end = end;
        _radius = radius;

        Vector3d segment = end - start;
        _lengthSqr = segment.MagnitudeSquared;
        _length = _lengthSqr <= Fixed64.Epsilon ? Fixed64.Zero : segment.Magnitude;
        _direction = _length <= Fixed64.Epsilon ? Vector3d.Zero : segment / _length;
    }

    /// <summary>
    /// Checks a collider against the prepared swept sphere.
    /// </summary>
    public bool TrySweep(
        LSCollider collider,
        out Vector3d sphereCenterAtImpact,
        out Fixed64 impactDistance)
    {
        sphereCenterAtImpact = Vector3d.Zero;
        impactDistance = Fixed64.Zero;

        if (_length <= Fixed64.Epsilon)
            return false;

        return collider switch
        {
            LSSphereCollider sphere => TrySweepSphere(sphere.Center, sphere.ScaledRadius + _radius, out sphereCenterAtImpact, out impactDistance),
            LSCapsuleCollider capsule => TrySweepCapsule(capsule, out sphereCenterAtImpact, out impactDistance),
            LSCuboidCollider cuboid => TrySweepCuboid(cuboid, out sphereCenterAtImpact, out impactDistance),
            LSCylinderCollider cylinder => TrySweepCylinder(cylinder, out sphereCenterAtImpact, out impactDistance),
            LSMeshCollider mesh => TrySweepMesh(mesh, out sphereCenterAtImpact, out impactDistance),
            LSCompoundCollider compound => TrySweepCompound(compound, out sphereCenterAtImpact, out impactDistance),
            _ => false
        };
    }

    private bool TrySweepSphere(
        Vector3d center,
        Fixed64 radius,
        out Vector3d sphereCenterAtImpact,
        out Fixed64 impactDistance)
    {
        sphereCenterAtImpact = Vector3d.Zero;
        impactDistance = Fixed64.Zero;

        Fixed64 radiusSqr = radius * radius;
        Vector3d startToCenter = _start - center;
        if (startToCenter.MagnitudeSquared <= radiusSqr)
        {
            sphereCenterAtImpact = _start;
            return true;
        }

        Fixed64 b = Vector3d.Dot(startToCenter, _direction);
        Fixed64 c = startToCenter.MagnitudeSquared - radiusSqr;
        Fixed64 discriminant = b * b - c;
        if (discriminant < Fixed64.Zero)
            return false;

        Fixed64 root = FixedMath.Sqrt(discriminant);
        Fixed64 distance = -b - root;
        if (distance < Fixed64.Zero)
            distance = -b + root;

        if (distance < Fixed64.Zero || distance > _length)
            return false;

        impactDistance = distance;
        sphereCenterAtImpact = _start + _direction * distance;
        return true;
    }

    private bool TrySweepCapsule(
        LSCapsuleCollider capsule,
        out Vector3d sphereCenterAtImpact,
        out Fixed64 impactDistance)
    {
        sphereCenterAtImpact = Vector3d.Zero;
        impactDistance = Fixed64.MaxValue;

        Fixed64 radius = capsule.ScaledRadius + _radius;
        bool found = false;

        found |= TryKeepEarlierSweep(
            TrySweepSphere(capsule.LineSegmentStart, radius, out Vector3d startHit, out Fixed64 startDistance),
            startHit,
            startDistance,
            ref sphereCenterAtImpact,
            ref impactDistance);
        found |= TryKeepEarlierSweep(
            TrySweepSphere(capsule.LineSegmentEnd, radius, out Vector3d endHit, out Fixed64 endDistance),
            endHit,
            endDistance,
            ref sphereCenterAtImpact,
            ref impactDistance);

        Fixed64 halfHeight = capsule.CylinderHeight * Fixed64.Half;
        if (halfHeight > Fixed64.Epsilon)
        {
            found |= TryKeepEarlierSweep(
                TrySweepFiniteCylinder(
                    capsule.Center,
                    capsule.Rotation,
                    radius,
                    halfHeight,
                    includeCaps: false,
                    out Vector3d cylinderHit,
                    out Fixed64 cylinderDistance),
                cylinderHit,
                cylinderDistance,
                ref sphereCenterAtImpact,
                ref impactDistance);
        }

        return found;
    }

    private bool TrySweepCylinder(
        LSCylinderCollider cylinder,
        out Vector3d sphereCenterAtImpact,
        out Fixed64 impactDistance)
    {
        return TrySweepFiniteCylinder(
            cylinder.Center,
            cylinder.Rotation,
            cylinder.ScaledRadius + _radius,
            cylinder.HalfHeight + _radius,
            includeCaps: true,
            out sphereCenterAtImpact,
            out impactDistance);
    }

    private bool TrySweepFiniteCylinder(
        Vector3d center,
        FixedQuaternion rotation,
        Fixed64 radius,
        Fixed64 halfHeight,
        bool includeCaps,
        out Vector3d sphereCenterAtImpact,
        out Fixed64 impactDistance)
    {
        sphereCenterAtImpact = Vector3d.Zero;
        impactDistance = Fixed64.MaxValue;

        FixedQuaternion inverseRotation = rotation.Inverse();
        Vector3d localStart = (_start - center) * inverseRotation;
        Vector3d localDirection = _direction * inverseRotation;
        Fixed64 radiusSqr = radius * radius;

        if (IsPointInsideFiniteCylinder(localStart, radiusSqr, halfHeight))
        {
            sphereCenterAtImpact = _start;
            impactDistance = Fixed64.Zero;
            return true;
        }

        bool found = false;
        found |= TryKeepEarlierSweep(
            TrySweepCylinderSide(center, rotation, localStart, localDirection, radiusSqr, halfHeight, out Vector3d sideHit, out Fixed64 sideDistance),
            sideHit,
            sideDistance,
            ref sphereCenterAtImpact,
            ref impactDistance);

        if (includeCaps)
        {
            found |= TryKeepEarlierSweep(
                TrySweepCylinderCap(center, rotation, localStart, localDirection, radiusSqr, halfHeight, out Vector3d topHit, out Fixed64 topDistance),
                topHit,
                topDistance,
                ref sphereCenterAtImpact,
                ref impactDistance);
            found |= TryKeepEarlierSweep(
                TrySweepCylinderCap(center, rotation, localStart, localDirection, radiusSqr, -halfHeight, out Vector3d bottomHit, out Fixed64 bottomDistance),
                bottomHit,
                bottomDistance,
                ref sphereCenterAtImpact,
                ref impactDistance);
        }

        return found;
    }

    private bool TrySweepCylinderSide(
        Vector3d center,
        FixedQuaternion rotation,
        Vector3d localStart,
        Vector3d localDirection,
        Fixed64 radiusSqr,
        Fixed64 halfHeight,
        out Vector3d sphereCenterAtImpact,
        out Fixed64 impactDistance)
    {
        sphereCenterAtImpact = Vector3d.Zero;
        impactDistance = Fixed64.MaxValue;

        Fixed64 a = localDirection.X * localDirection.X + localDirection.Z * localDirection.Z;
        if (a <= Fixed64.Epsilon)
            return false;

        Fixed64 b = 2 * (localStart.X * localDirection.X + localStart.Z * localDirection.Z);
        Fixed64 c = localStart.X * localStart.X + localStart.Z * localStart.Z - radiusSqr;
        Fixed64 discriminant = b * b - 4 * a * c;
        if (discriminant < Fixed64.Zero)
            return false;

        Fixed64 root = FixedMath.Sqrt(discriminant);
        Fixed64 denominator = 2 * a;
        Fixed64 first = (-b - root) / denominator;
        Fixed64 second = (-b + root) / denominator;

        bool found = false;
        found |= TryKeepEarlierSweep(
            TryBuildCylinderLocalHit(center, rotation, localStart, localDirection, first, halfHeight, out Vector3d firstHit, out Fixed64 firstDistance),
            firstHit,
            firstDistance,
            ref sphereCenterAtImpact,
            ref impactDistance);
        found |= TryKeepEarlierSweep(
            TryBuildCylinderLocalHit(center, rotation, localStart, localDirection, second, halfHeight, out Vector3d secondHit, out Fixed64 secondDistance),
            secondHit,
            secondDistance,
            ref sphereCenterAtImpact,
            ref impactDistance);

        return found;
    }

    private bool TrySweepCylinderCap(
        Vector3d center,
        FixedQuaternion rotation,
        Vector3d localStart,
        Vector3d localDirection,
        Fixed64 radiusSqr,
        Fixed64 capY,
        out Vector3d sphereCenterAtImpact,
        out Fixed64 impactDistance)
    {
        sphereCenterAtImpact = Vector3d.Zero;
        impactDistance = Fixed64.Zero;

        if (localDirection.Y.Abs() <= Fixed64.Epsilon)
            return false;

        Fixed64 distance = (capY - localStart.Y) / localDirection.Y;
        if (distance < Fixed64.Zero || distance > _length)
            return false;

        Vector3d localPoint = localStart + localDirection * distance;
        Fixed64 radialSqr = localPoint.X * localPoint.X + localPoint.Z * localPoint.Z;
        if (radialSqr > radiusSqr + Fixed64.Epsilon)
            return false;

        sphereCenterAtImpact = center + rotation * localPoint;
        impactDistance = distance;
        return true;
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

        bool found = false;
        for (int i = 0; i < _meshTriangleBuffer.Count; i++)
        {
            int triangleIndex = _meshTriangleBuffer[i];
            mesh.Mesh.GetTriangleVertices(triangleIndex, out Vector3d first, out Vector3d second, out Vector3d third);
            Vector3d normal = mesh.Mesh.GetFaceNormalWorld(triangleIndex);
            if (normal.MagnitudeSquared <= Fixed64.Epsilon)
                continue;

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

        Vector3d closestAtStart = MeshUtils.ClosestPointOnTriangle(first, second, third, normal, _start);
        if (Vector3d.DistanceSquared(_start, closestAtStart) <= _radius * _radius)
        {
            sphereCenterAtImpact = _start;
            impactDistance = Fixed64.Zero;
            return true;
        }

        bool found = false;
        found |= TryKeepEarlierSweep(
            TrySweepTriangleFace(first, second, third, normal, _radius, out Vector3d frontHit, out Fixed64 frontDistance),
            frontHit,
            frontDistance,
            ref sphereCenterAtImpact,
            ref impactDistance);
        found |= TryKeepEarlierSweep(
            TrySweepTriangleFace(first, second, third, normal, -_radius, out Vector3d backHit, out Fixed64 backDistance),
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
            TrySweepSphere(first, _radius, out Vector3d firstVertexHit, out Fixed64 firstVertexDistance),
            firstVertexHit,
            firstVertexDistance,
            ref sphereCenterAtImpact,
            ref impactDistance);
        found |= TryKeepEarlierSweep(
            TrySweepSphere(second, _radius, out Vector3d secondVertexHit, out Fixed64 secondVertexDistance),
            secondVertexHit,
            secondVertexDistance,
            ref sphereCenterAtImpact,
            ref impactDistance);
        found |= TryKeepEarlierSweep(
            TrySweepSphere(third, _radius, out Vector3d thirdVertexHit, out Fixed64 thirdVertexDistance),
            thirdVertexHit,
            thirdVertexDistance,
            ref sphereCenterAtImpact,
            ref impactDistance);

        return found;
    }

    private bool TrySweepTriangleFace(
        Vector3d first,
        Vector3d second,
        Vector3d third,
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

        Fixed64 signedStartDistance = Vector3d.Dot(_start - first, normal);
        Fixed64 distance = (signedRadius - signedStartDistance) / denominator;
        if (distance < Fixed64.Zero || distance > _length)
            return false;

        Vector3d center = _start + _direction * distance;
        Vector3d pointOnPlane = center - normal * signedRadius;
        if (!MeshUtils.IsPointInTrianglePlane(first, second, third, normal, pointOnPlane))
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

        Vector3d edge = edgeEnd - edgeStart;
        Fixed64 edgeLengthSqr = edge.MagnitudeSquared;
        if (edgeLengthSqr <= Fixed64.Epsilon)
            return false;

        Fixed64 edgeLength = FixedMath.Sqrt(edgeLengthSqr);
        Vector3d edgeDirection = edge / edgeLength;
        Vector3d startToEdge = _start - edgeStart;
        Vector3d radialStart = startToEdge - edgeDirection * Vector3d.Dot(startToEdge, edgeDirection);
        Vector3d radialDirection = _direction - edgeDirection * Vector3d.Dot(_direction, edgeDirection);
        Fixed64 a = radialDirection.MagnitudeSquared;
        if (a <= Fixed64.Epsilon)
            return false;

        Fixed64 b = 2 * Vector3d.Dot(radialStart, radialDirection);
        Fixed64 c = radialStart.MagnitudeSquared - _radius * _radius;
        Fixed64 discriminant = b * b - 4 * a * c;
        if (discriminant < Fixed64.Zero)
            return false;

        Fixed64 root = FixedMath.Sqrt(discriminant);
        Fixed64 denominator = 2 * a;
        Fixed64 first = (-b - root) / denominator;
        Fixed64 second = (-b + root) / denominator;

        bool found = false;
        found |= TryKeepEarlierSweep(
            TryBuildTriangleEdgeHit(edgeStart, edgeDirection, edgeLength, first, out Vector3d firstHit, out Fixed64 firstDistance),
            firstHit,
            firstDistance,
            ref sphereCenterAtImpact,
            ref impactDistance);
        found |= TryKeepEarlierSweep(
            TryBuildTriangleEdgeHit(edgeStart, edgeDirection, edgeLength, second, out Vector3d secondHit, out Fixed64 secondDistance),
            secondHit,
            secondDistance,
            ref sphereCenterAtImpact,
            ref impactDistance);

        return found;
    }

    private bool TryBuildTriangleEdgeHit(
        Vector3d edgeStart,
        Vector3d edgeDirection,
        Fixed64 edgeLength,
        Fixed64 distance,
        out Vector3d sphereCenterAtImpact,
        out Fixed64 impactDistance)
    {
        sphereCenterAtImpact = Vector3d.Zero;
        impactDistance = Fixed64.Zero;

        if (distance < Fixed64.Zero || distance > _length)
            return false;

        Vector3d center = _start + _direction * distance;
        Fixed64 edgeProjection = Vector3d.Dot(center - edgeStart, edgeDirection);
        if (edgeProjection < Fixed64.Zero || edgeProjection > edgeLength)
            return false;

        sphereCenterAtImpact = center;
        impactDistance = distance;
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

        if (IsPointInsideBox(localStart, min, max))
        {
            sphereCenterAtImpact = _start;
            return true;
        }

        Fixed64 entry = Fixed64.Zero;
        Fixed64 exit = _length;
        if (!ClipSegmentAxis(localStart.X, localDirection.X, min.X, max.X, ref entry, ref exit)
            || !ClipSegmentAxis(localStart.Y, localDirection.Y, min.Y, max.Y, ref entry, ref exit)
            || !ClipSegmentAxis(localStart.Z, localDirection.Z, min.Z, max.Z, ref entry, ref exit))
        {
            return false;
        }

        impactDistance = entry;
        sphereCenterAtImpact = center + rotation * (localStart + localDirection * entry);
        return true;
    }

    private bool TryBuildCylinderLocalHit(
        Vector3d center,
        FixedQuaternion rotation,
        Vector3d localStart,
        Vector3d localDirection,
        Fixed64 distance,
        Fixed64 halfHeight,
        out Vector3d sphereCenterAtImpact,
        out Fixed64 impactDistance)
    {
        sphereCenterAtImpact = Vector3d.Zero;
        impactDistance = Fixed64.Zero;

        if (distance < Fixed64.Zero || distance > _length)
            return false;

        Vector3d localPoint = localStart + localDirection * distance;
        if (localPoint.Y < -halfHeight || localPoint.Y > halfHeight)
            return false;

        sphereCenterAtImpact = center + rotation * localPoint;
        impactDistance = distance;
        return true;
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

    private static bool IsPointInsideFiniteCylinder(Vector3d localPoint, Fixed64 radiusSqr, Fixed64 halfHeight) =>
        localPoint.Y >= -halfHeight
        && localPoint.Y <= halfHeight
        && localPoint.X * localPoint.X + localPoint.Z * localPoint.Z <= radiusSqr;

    private static bool IsPointInsideBox(Vector3d localPoint, Vector3d min, Vector3d max) =>
        localPoint.X >= min.X && localPoint.X <= max.X
        && localPoint.Y >= min.Y && localPoint.Y <= max.Y
        && localPoint.Z >= min.Z && localPoint.Z <= max.Z;

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

    private static void CreateSweepBounds(Vector3d start, Vector3d end, Fixed64 radius, out Vector3d min, out Vector3d max)
    {
        Vector3d radiusExtents = Vector3d.One * radius;
        min = Vector3d.Min(start, end) - radiusExtents;
        max = Vector3d.Max(start, end) + radiusExtents;
    }
}
