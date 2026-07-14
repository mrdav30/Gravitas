//=======================================================================
// RaycastSegmentWorker.cs
//=======================================================================
// MIT License, Copyright (c) 2026–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp;
using Gravitas.Colliders;
using Gravitas.Support;
using SwiftCollections;
using SwiftCollections.Query;

namespace Gravitas.Queries;

/// <summary>
/// Stores segment data used by one context-owned raycast service while checking collider overlaps.
/// </summary>
public sealed class RaycastSegmentWorker
{
    private Vector3d _cachedOrigin;
    private Vector3d _cachedEnd;
    private Vector3d _segmentDirection;
    private Fixed64 _segmentLength;
    private Fixed64 _segmentLengthSqr;
    private bool _segmentIsValid;
    private bool _calculateIntersections;
    private readonly SwiftList<int> _meshTriangleBuffer = new();

    internal Vector3d SegmentDirection => _segmentDirection;

    /// <summary>
    /// Prepares this worker for overlap checks against the line segment between two points.
    /// </summary>
    public void PrepareSegmentCheck(Vector3d p1, Vector3d p2, bool calculateIntersectionPoints = true)
    {
        _cachedOrigin = p1;
        _cachedEnd = p2;

        if (!FixedVectorDifference.TryCreate(p1, p2, out Vector3d segment)
            || !Vector3d.TryGetMagnitude(segment, out _segmentLength))
        {
            _segmentLengthSqr = Fixed64.Zero;
            _segmentLength = Fixed64.Zero;
            _segmentDirection = Vector3d.Zero;
            _segmentIsValid = false;
            _calculateIntersections = calculateIntersectionPoints;
            return;
        }

        _segmentLengthSqr = segment.MagnitudeSquared;
        _segmentDirection = _segmentLength == Fixed64.Zero ? Vector3d.Zero : segment.Normalized;
        _segmentIsValid = true;
        _calculateIntersections = calculateIntersectionPoints;
    }

    public bool CheckSphereOverlaps(LSSphereCollider sphereCollider, ref SwiftList<Vector3d> outputIntersectionPoints) =>
        CheckSphereOverlaps(sphereCollider.Center, sphereCollider.ScaledRadiusSqr, ref outputIntersectionPoints);

    /// <summary>
    /// Checks whether a sphere overlaps this worker's prepared ray segment.
    /// </summary>
    public bool CheckSphereOverlaps(
        Vector3d position,
        Fixed64 sqrRadius,
        ref SwiftList<Vector3d> outputIntersectionPoints)
    {
        if (!_segmentIsValid)
            return false;

        if (_segmentLengthSqr == Fixed64.Zero)
            return CheckPointInsideSphere(position, sqrRadius, ref outputIntersectionPoints);

        Vector3d originToCenter = position - _cachedOrigin;
        Fixed64 closestParameter = Vector3d.Dot(originToCenter, _segmentDirection);
        closestParameter = FixedMath.Clamp(closestParameter, Fixed64.Zero, _segmentLength);
        Vector3d closestPoint = _cachedOrigin + _segmentDirection * closestParameter;
        if ((closestPoint - position).MagnitudeSquared > sqrRadius)
            return false;

        if (!_calculateIntersections)
            return true;

        Vector3d originFromCenter = _cachedOrigin - position;
        Fixed64 c = originFromCenter.MagnitudeSquared - sqrRadius;
        if (c <= Fixed64.Zero)
        {
            outputIntersectionPoints.Add(_cachedOrigin);
            return true;
        }

        Fixed64 b = Vector3d.Dot(originFromCenter, _segmentDirection);
        // In exact arithmetic the closest-point overlap proves a non-negative
        // discriminant; clamp fixed-point normalization residue for tangencies.
        Fixed64 discriminant = FixedMath.Max(b * b - c, Fixed64.Zero);

        Fixed64 root = FixedMath.Sqrt(discriminant);
        AddIntersectionPointIfOnSegment(-b - root, ref outputIntersectionPoints);
        if (root != Fixed64.Zero)
            AddIntersectionPointIfOnSegment(-b + root, ref outputIntersectionPoints);

        return outputIntersectionPoints.Count > 0;
    }

    public bool CheckCapsuleOverlaps(LSCapsuleCollider capsuleCollider, ref SwiftList<Vector3d> outputIntersectionPoints)
    {
        bool intersects = CheckCapsuleCylinderOverlaps(capsuleCollider, ref outputIntersectionPoints);

        if (!intersects)
        {
            intersects = CheckSphereOverlaps(capsuleCollider.LineSegmentEnd, capsuleCollider.ScaledRadiusSqr, ref outputIntersectionPoints)
                         || CheckSphereOverlaps(capsuleCollider.LineSegmentStart, capsuleCollider.ScaledRadiusSqr, ref outputIntersectionPoints);
        }

        return intersects;
    }

    public bool CheckCylinderOverlaps(LSCylinderCollider cylinderCollider, ref SwiftList<Vector3d> outputIntersectionPoints) =>
        CheckFiniteCylinderOverlaps(
            cylinderCollider.Center,
            cylinderCollider.Rotation,
            cylinderCollider.ScaledRadius,
            cylinderCollider.ScaledRadiusSqr,
            cylinderCollider.HalfHeight,
            includeCaps: true,
            ref outputIntersectionPoints);

    public bool CheckConeOverlaps(LSConeCollider coneCollider, ref SwiftList<Vector3d> outputIntersectionPoints)
    {
        if (!_segmentIsValid)
            return false;

        FixedQuaternion inverseRotation = coneCollider.Rotation.Inverse();
        Vector3d localOrigin = (_cachedOrigin - coneCollider.Center) * inverseRotation;

        if (_segmentLengthSqr == Fixed64.Zero)
        {
            if (!coneCollider.ContainsWorldPoint(_cachedOrigin))
                return false;

            if (_calculateIntersections)
                outputIntersectionPoints.Add(_cachedOrigin);
            return true;
        }

        if (coneCollider.ContainsWorldPoint(_cachedOrigin))
        {
            if (_calculateIntersections)
                outputIntersectionPoints.Add(_cachedOrigin);
            return true;
        }

        Vector3d localDirection = _segmentDirection * inverseRotation;
        bool intersects = CheckConeSide(
            coneCollider,
            localOrigin,
            localDirection,
            ref outputIntersectionPoints);
        intersects |= CheckConeBase(
            coneCollider,
            localOrigin,
            localDirection,
            ref outputIntersectionPoints);
        return intersects;
    }

    public bool CheckMeshOverlaps(LSMeshCollider meshCollider, ref SwiftList<Vector3d> outputIntersectionPoints)
    {
        if (!_segmentIsValid)
            return false;

        Vector3d localOrigin = meshCollider.Mesh.ConvertWorldToLocal(_cachedOrigin);
        Vector3d localEnd = meshCollider.Mesh.ConvertWorldToLocal(_cachedEnd);
        Vector3d localSegment = localEnd - localOrigin;
        Fixed64 localSegmentLengthSqr = localSegment.MagnitudeSquared;
        Fixed64 localSegmentLength = localSegmentLengthSqr == Fixed64.Zero ? Fixed64.Zero : localSegment.Magnitude;
        Vector3d localSegmentDirection = localSegmentLength == Fixed64.Zero ? Vector3d.Zero : localSegment.Normalized;

        _meshTriangleBuffer.FastClear();
        meshCollider.Mesh.GetTrianglesInLocalBounds(CreateSegmentBounds(localOrigin, localEnd), _meshTriangleBuffer);
        bool intersects = false;
        for (int i = 0; i < _meshTriangleBuffer.Count; i++)
        {
            int triangleIndex = _meshTriangleBuffer[i];
            meshCollider.Mesh.GetLocalTriangleVertices(triangleIndex, out Vector3d first, out Vector3d second, out Vector3d third);
            if (!TryAddLocalTriangleIntersection(
                    meshCollider.Mesh,
                    first,
                    second,
                    third,
                    meshCollider.Mesh.FaceNormals[triangleIndex],
                    localOrigin,
                    localEnd,
                    localSegmentDirection,
                    localSegmentLength,
                    localSegmentLengthSqr,
                    ref outputIntersectionPoints))
            {
                continue;
            }

            intersects = true;
            if (!_calculateIntersections)
                return true;
        }

        return intersects;
    }

    private bool CheckCapsuleCylinderOverlaps(LSCapsuleCollider capsuleCollider, ref SwiftList<Vector3d> outputIntersectionPoints) =>
        CheckFiniteCylinderOverlaps(
            capsuleCollider.Center,
            capsuleCollider.Rotation,
            capsuleCollider.ScaledRadius,
            capsuleCollider.ScaledRadiusSqr,
            capsuleCollider.CylinderHeight * Fixed64.Half,
            includeCaps: false,
            ref outputIntersectionPoints);

    private bool CheckFiniteCylinderOverlaps(
        Vector3d center,
        FixedQuaternion rotation,
        Fixed64 radius,
        Fixed64 radiusSqr,
        Fixed64 halfHeight,
        bool includeCaps,
        ref SwiftList<Vector3d> outputIntersectionPoints)
    {
        if (!_segmentIsValid)
            return false;

        FixedQuaternion inverseRotation = rotation.Inverse();
        Vector3d localOrigin = (_cachedOrigin - center) * inverseRotation;

        if (_segmentLengthSqr == Fixed64.Zero)
            return CheckPointInsideFiniteCylinder(center, rotation, localOrigin, radiusSqr, halfHeight, ref outputIntersectionPoints);

        Vector3d localDirection = _segmentDirection * inverseRotation;
        if (PointInsideFiniteCylinder(localOrigin, radiusSqr, halfHeight))
        {
            if (_calculateIntersections)
                outputIntersectionPoints.Add(_cachedOrigin);
            return true;
        }

        bool intersects = CheckFiniteCylinderSide(
            center,
            rotation,
            localOrigin,
            localDirection,
            radiusSqr,
            halfHeight,
            ref outputIntersectionPoints);

        if (includeCaps)
        {
            intersects |= CheckFiniteCylinderCap(center, rotation, localOrigin, localDirection, radiusSqr, halfHeight, ref outputIntersectionPoints);
            intersects |= CheckFiniteCylinderCap(center, rotation, localOrigin, localDirection, radiusSqr, -halfHeight, ref outputIntersectionPoints);
        }

        return intersects;
    }

    private bool CheckFiniteCylinderSide(
        Vector3d center,
        FixedQuaternion rotation,
        Vector3d localOrigin,
        Vector3d localDirection,
        Fixed64 radiusSqr,
        Fixed64 halfHeight,
        ref SwiftList<Vector3d> outputIntersectionPoints)
    {
        Fixed64 a = localDirection.X * localDirection.X + localDirection.Z * localDirection.Z;
        if (a <= Fixed64.Epsilon)
            return false;

        Fixed64 b = 2 * (localOrigin.X * localDirection.X + localOrigin.Z * localDirection.Z);
        Fixed64 c = localOrigin.X * localOrigin.X + localOrigin.Z * localOrigin.Z - radiusSqr;
        Fixed64 discriminant = b * b - 4 * a * c;
        if (discriminant < Fixed64.Zero)
            return false;

        Fixed64 root = FixedMath.Sqrt(discriminant);
        Fixed64 denominator = 2 * a;
        Fixed64 t1 = (-b - root) / denominator;
        Fixed64 t2 = (-b + root) / denominator;

        bool intersects = TryAddFiniteCylinderPoint(center, rotation, localOrigin, localDirection, t1, halfHeight, ref outputIntersectionPoints);
        if (t2 != t1)
            intersects |= TryAddFiniteCylinderPoint(center, rotation, localOrigin, localDirection, t2, halfHeight, ref outputIntersectionPoints);

        return intersects;
    }

    private bool CheckFiniteCylinderCap(
        Vector3d center,
        FixedQuaternion rotation,
        Vector3d localOrigin,
        Vector3d localDirection,
        Fixed64 radiusSqr,
        Fixed64 capY,
        ref SwiftList<Vector3d> outputIntersectionPoints)
    {
        if (localDirection.Y.Abs() <= Fixed64.Epsilon)
            return false;

        Fixed64 distance = (capY - localOrigin.Y) / localDirection.Y;
        if (distance < Fixed64.Zero || distance > _segmentLength)
            return false;

        Vector3d localPoint = localOrigin + localDirection * distance;
        Fixed64 radialSqr = localPoint.X * localPoint.X + localPoint.Z * localPoint.Z;
        if (radialSqr > radiusSqr + Fixed64.Epsilon)
            return false;

        AddLocalIntersectionPoint(center, rotation, localPoint, ref outputIntersectionPoints);
        return true;
    }

    private bool CheckConeSide(
        LSConeCollider cone,
        Vector3d localOrigin,
        Vector3d localDirection,
        ref SwiftList<Vector3d> outputIntersectionPoints)
    {
        Fixed64 slope = cone.ScaledRadius / cone.Height;
        Fixed64 slopeSqr = slope * slope;
        Fixed64 q = cone.HalfHeight - localOrigin.Y;
        Fixed64 a = localDirection.X * localDirection.X
            + localDirection.Z * localDirection.Z
            - slopeSqr * localDirection.Y * localDirection.Y;
        Fixed64 b = 2 * (localOrigin.X * localDirection.X
            + localOrigin.Z * localDirection.Z
            + slopeSqr * q * localDirection.Y);
        Fixed64 c = localOrigin.X * localOrigin.X
            + localOrigin.Z * localOrigin.Z
            - slopeSqr * q * q;

        if (a.Abs() <= Fixed64.Epsilon)
        {
            if (b.Abs() <= Fixed64.Epsilon)
                return false;

            return TryAddConeSidePoint(
                cone,
                localOrigin,
                localDirection,
                -c / b,
                ref outputIntersectionPoints);
        }

        Fixed64 discriminant = b * b - 4 * a * c;
        if (discriminant < Fixed64.Zero)
            return false;

        Fixed64 root = FixedMath.Sqrt(discriminant);
        Fixed64 denominator = 2 * a;
        Fixed64 first = (-b - root) / denominator;
        Fixed64 second = (-b + root) / denominator;

        bool intersects = TryAddConeSidePoint(cone, localOrigin, localDirection, first, ref outputIntersectionPoints);
        if (second != first)
            intersects |= TryAddConeSidePoint(cone, localOrigin, localDirection, second, ref outputIntersectionPoints);

        return intersects;
    }

    private bool CheckConeBase(
        LSConeCollider cone,
        Vector3d localOrigin,
        Vector3d localDirection,
        ref SwiftList<Vector3d> outputIntersectionPoints)
    {
        if (localDirection.Y.Abs() <= Fixed64.Epsilon)
            return false;

        Fixed64 distance = (-cone.HalfHeight - localOrigin.Y) / localDirection.Y;
        if (distance < Fixed64.Zero || distance > _segmentLength)
            return false;

        Vector3d localPoint = localOrigin + localDirection * distance;
        Fixed64 radialSqr = localPoint.X * localPoint.X + localPoint.Z * localPoint.Z;
        if (radialSqr > cone.ScaledRadiusSqr + Fixed64.Epsilon)
            return false;

        AddLocalIntersectionPoint(cone.Center, cone.Rotation, localPoint, ref outputIntersectionPoints);
        return true;
    }

    private bool TryAddConeSidePoint(
        LSConeCollider cone,
        Vector3d localOrigin,
        Vector3d localDirection,
        Fixed64 distance,
        ref SwiftList<Vector3d> outputIntersectionPoints)
    {
        if (distance < Fixed64.Zero || distance > _segmentLength)
            return false;

        Vector3d localPoint = localOrigin + localDirection * distance;
        if (localPoint.Y < -cone.HalfHeight - Fixed64.Epsilon
            || localPoint.Y > cone.HalfHeight + Fixed64.Epsilon)
        {
            return false;
        }

        AddLocalIntersectionPoint(cone.Center, cone.Rotation, localPoint, ref outputIntersectionPoints);
        return true;
    }

    public bool CheckAABBoxOverlaps(LSCuboidCollider aabox, ref SwiftList<Vector3d> outputIntersectionPoints) =>
         CheckAABBoxOverlaps(aabox.BoundsMin, aabox.BoundsMax, ref outputIntersectionPoints);

    /// <summary>
    /// Checks whether an axis-aligned bounding box overlaps this worker's prepared ray segment.
    /// </summary>
    public bool CheckAABBoxOverlaps(Vector3d min, Vector3d max, ref SwiftList<Vector3d> outputIntersectionPoints)
    {
        if (!_segmentIsValid)
            return false;

        if (_segmentLengthSqr == Fixed64.Zero)
            return CheckPointInsideBox(min, max, ref outputIntersectionPoints);

        if (!SweepBoundsUtility.TryClipSegment(
            _cachedOrigin,
            _segmentDirection,
            _segmentLength,
            min,
            max,
            out Fixed64 entry,
            out Fixed64 exit))
        {
            return false;
        }

        if (_calculateIntersections)
        {
            outputIntersectionPoints.Add(_cachedOrigin + _segmentDirection * entry);
            if (exit != entry)
                outputIntersectionPoints.Add(_cachedOrigin + _segmentDirection * exit);
        }

        return true;
    }

    /// <summary>
    /// Checks whether an oriented bounding box overlaps this worker's prepared ray segment.
    /// </summary>
    public bool CheckOBBoxOverlaps(LSCuboidCollider oobox, ref SwiftList<Vector3d> outputIntersectionPoints)
    {
        if (!_segmentIsValid)
            return false;

        FixedQuaternion inverseRotation = oobox.Rotation.Inverse();
        Vector3d localOrigin = (_cachedOrigin - oobox.Center) * inverseRotation;
        Vector3d halfExtents = oobox.ScaledSize * Fixed64.Half;
        Vector3d min = -halfExtents;
        Vector3d max = halfExtents;

        if (_segmentLengthSqr == Fixed64.Zero)
        {
            if (localOrigin.X < min.X || localOrigin.X > max.X
                || localOrigin.Y < min.Y || localOrigin.Y > max.Y
                || localOrigin.Z < min.Z || localOrigin.Z > max.Z)
            {
                return false;
            }

            AddLocalIntersectionPoint(oobox.Center, oobox.Rotation, localOrigin, ref outputIntersectionPoints);
            return true;
        }

        Vector3d localDirection = _segmentDirection * inverseRotation;
        if (!SweepBoundsUtility.TryClipSegment(
            localOrigin,
            localDirection,
            _segmentLength,
            min,
            max,
            out Fixed64 entry,
            out Fixed64 exit))
        {
            return false;
        }

        if (_calculateIntersections)
        {
            AddLocalIntersectionPoint(oobox.Center, oobox.Rotation, localOrigin + localDirection * entry, ref outputIntersectionPoints);
            if (exit != entry)
                AddLocalIntersectionPoint(oobox.Center, oobox.Rotation, localOrigin + localDirection * exit, ref outputIntersectionPoints);
        }

        return true;
    }

    private bool TryAddFiniteCylinderPoint(
        Vector3d center,
        FixedQuaternion rotation,
        Vector3d localOrigin,
        Vector3d localDirection,
        Fixed64 distance,
        Fixed64 halfHeight,
        ref SwiftList<Vector3d> outputIntersectionPoints)
    {
        if (distance < Fixed64.Zero || distance > _segmentLength)
            return false;

        Vector3d localPoint = localOrigin + localDirection * distance;
        if (localPoint.Y < -halfHeight || localPoint.Y > halfHeight)
            return false;

        AddLocalIntersectionPoint(center, rotation, localPoint, ref outputIntersectionPoints);
        return true;
    }

    private bool TryAddLocalTriangleIntersection(
        PhysicsMesh mesh,
        Vector3d first,
        Vector3d second,
        Vector3d third,
        Vector3d normal,
        Vector3d localOrigin,
        Vector3d localEnd,
        Vector3d localSegmentDirection,
        Fixed64 localSegmentLength,
        Fixed64 localSegmentLengthSqr,
        ref SwiftList<Vector3d> outputIntersectionPoints)
    {
        if (localSegmentLengthSqr == Fixed64.Zero)
        {
            if (Vector3d.Dot(localOrigin - first, normal).Abs() > Fixed64.Epsilon
                || !MeshUtils.IsPointInTrianglePlane(first, second, third, normal, localOrigin))
            {
                return false;
            }

            AddTriangleIntersectionPoint(mesh.ConvertLocalToWorld(localOrigin), ref outputIntersectionPoints);
            return true;
        }

        Fixed64 denominator = Vector3d.Dot(normal, localSegmentDirection);
        if (denominator.Abs() <= Fixed64.Epsilon)
        {
            if (Vector3d.Dot(localOrigin - first, normal).Abs() > Fixed64.Epsilon)
                return false;

            bool found = false;
            if (MeshUtils.IsPointInTrianglePlane(first, second, third, normal, localOrigin))
            {
                AddTriangleIntersectionPoint(mesh.ConvertLocalToWorld(localOrigin), ref outputIntersectionPoints);
                found = true;
            }

            if (MeshUtils.IsPointInTrianglePlane(first, second, third, normal, localEnd))
            {
                AddTriangleIntersectionPoint(mesh.ConvertLocalToWorld(localEnd), ref outputIntersectionPoints);
                found = true;
            }

            return found;
        }

        Fixed64 distance = Vector3d.Dot(first - localOrigin, normal) / denominator;
        if (distance < Fixed64.Zero || distance > localSegmentLength)
            return false;

        Vector3d localPoint = localOrigin + localSegmentDirection * distance;
        if (!MeshUtils.IsPointInTrianglePlane(first, second, third, normal, localPoint))
            return false;

        AddTriangleIntersectionPoint(mesh.ConvertLocalToWorld(localPoint), ref outputIntersectionPoints);
        return true;
    }

    private void AddTriangleIntersectionPoint(Vector3d point, ref SwiftList<Vector3d> outputIntersectionPoints)
    {
        if (!_calculateIntersections)
            return;

        for (int i = 0; i < outputIntersectionPoints.Count; i++)
        {
            if (Vector3d.DistanceSquared(outputIntersectionPoints[i], point) <= Fixed64.Epsilon)
                return;
        }

        outputIntersectionPoints.Add(point);
    }

    private static FixedBoundVolume CreateSegmentBounds(Vector3d origin, Vector3d end)
    {
        Vector3d min = Vector3d.Min(origin, end);
        Vector3d max = Vector3d.Max(origin, end);
        return new FixedBoundVolume(min, max);
    }

    private bool CheckPointInsideFiniteCylinder(
        Vector3d center,
        FixedQuaternion rotation,
        Vector3d localPoint,
        Fixed64 radiusSqr,
        Fixed64 halfHeight,
        ref SwiftList<Vector3d> outputIntersectionPoints)
    {
        if (!PointInsideFiniteCylinder(localPoint, radiusSqr, halfHeight))
            return false;

        AddLocalIntersectionPoint(center, rotation, localPoint, ref outputIntersectionPoints);
        return true;
    }

    private static bool PointInsideFiniteCylinder(Vector3d localPoint, Fixed64 radiusSqr, Fixed64 halfHeight) =>
        localPoint.Y >= -halfHeight
        && localPoint.Y <= halfHeight
        && localPoint.X * localPoint.X + localPoint.Z * localPoint.Z <= radiusSqr;

    private void AddLocalIntersectionPoint(
        Vector3d center,
        FixedQuaternion rotation,
        Vector3d localPoint,
        ref SwiftList<Vector3d> outputIntersectionPoints)
    {
        if (!_calculateIntersections)
            return;

        Vector3d worldPoint = center + rotation * localPoint;
        for (int i = 0; i < outputIntersectionPoints.Count; i++)
        {
            if (Vector3d.DistanceSquared(outputIntersectionPoints[i], worldPoint) <= Fixed64.Epsilon)
                return;
        }

        outputIntersectionPoints.Add(worldPoint);
    }

    private bool CheckPointInsideSphere(
        Vector3d position,
        Fixed64 sqrRadius,
        ref SwiftList<Vector3d> outputIntersectionPoints)
    {
        if ((_cachedOrigin - position).MagnitudeSquared > sqrRadius)
            return false;

        if (_calculateIntersections)
            outputIntersectionPoints.Add(_cachedOrigin);

        return true;
    }

    private bool CheckPointInsideBox(
        Vector3d min,
        Vector3d max,
        ref SwiftList<Vector3d> outputIntersectionPoints)
    {
        if (_cachedOrigin.X < min.X || _cachedOrigin.X > max.X
            || _cachedOrigin.Y < min.Y || _cachedOrigin.Y > max.Y
            || _cachedOrigin.Z < min.Z || _cachedOrigin.Z > max.Z)
        {
            return false;
        }

        if (_calculateIntersections)
            outputIntersectionPoints.Add(_cachedOrigin);

        return true;
    }

    private void AddIntersectionPointIfOnSegment(Fixed64 distance, ref SwiftList<Vector3d> outputIntersectionPoints)
    {
        if (distance > _segmentLength)
            return;

        outputIntersectionPoints.Add(_cachedOrigin + _segmentDirection * distance);
    }

}
