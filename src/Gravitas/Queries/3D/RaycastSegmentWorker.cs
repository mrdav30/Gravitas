//=======================================================================
// RaycastSegmentWorker.cs
//=======================================================================
// MIT License, Copyright (c) 2026–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp;
using FixedMathSharp.Bounds;
using Gravitas.Colliders;
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
    private Vector3d _cachedSegment;
    private Vector3d _segmentDirection;
    private Fixed64 _segmentLength;
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

        if (!Vector3d.TrySubtract(p2, p1, out Vector3d segment)
            || !Vector3d.TryGetMagnitude(segment, out _segmentLength))
        {
            _segmentLength = Fixed64.Zero;
            _cachedSegment = Vector3d.Zero;
            _segmentDirection = Vector3d.Zero;
            _segmentIsValid = false;
            _calculateIntersections = calculateIntersectionPoints;
            return;
        }

        _cachedSegment = segment;
        _segmentDirection = _segmentLength == Fixed64.Zero ? Vector3d.Zero : segment.Normalized;
        _segmentIsValid = true;
        _calculateIntersections = calculateIntersectionPoints;
    }

    public bool CheckSphereOverlaps(LSSphereCollider sphereCollider, ref SwiftList<Vector3d> outputIntersectionPoints) =>
        CheckSphereOverlaps(
            new FixedBoundSphere(sphereCollider.Center, sphereCollider.ScaledRadius),
            ref outputIntersectionPoints);

    /// <summary>
    /// Checks whether a sphere overlaps this worker's prepared ray segment.
    /// </summary>
    /// <param name="sphere">The sphere bound.</param>
    /// <param name="outputIntersectionPoints">
    /// Receives segment points reconstructed from the nearest representable bounded intersection parameters.
    /// </param>
    /// <returns><see langword="true"/> when the prepared segment overlaps the sphere.</returns>
    public bool CheckSphereOverlaps(
        FixedBoundSphere sphere,
        ref SwiftList<Vector3d> outputIntersectionPoints)
    {
        if (!_segmentIsValid)
            return false;

        if (_cachedSegment.IsZero)
            return CheckPointInsideSphere(sphere, ref outputIntersectionPoints);

        bool startsInside = sphere.Contains(_cachedOrigin);
        var ray = new FixedRay(_cachedOrigin, _cachedSegment);
        if (!ray.TryGetIntersectionInterval(
                sphere,
                Fixed64.One,
                out Fixed64 entry,
                out Fixed64 exit))
        {
            return false;
        }

        if (!_calculateIntersections)
            return true;

        if (startsInside)
        {
            outputIntersectionPoints.Add(_cachedOrigin);
            return true;
        }

        AddSegmentParameterIntersectionPoint(entry, ref outputIntersectionPoints);
        if (exit != entry
            && (exit < Fixed64.One || !sphere.ContainsStrict(_cachedEnd)))
        {
            AddSegmentParameterIntersectionPoint(exit, ref outputIntersectionPoints);
        }

        return true;
    }

    public bool CheckCapsuleOverlaps(LSCapsuleCollider capsuleCollider, ref SwiftList<Vector3d> outputIntersectionPoints)
    {
        if (!_segmentIsValid)
            return false;

        var query = new FixedSegment(_cachedOrigin, _cachedEnd);
        if (!query.TryGetCapsuleIntersectionDistanceInterval(
                capsuleCollider.Center,
                capsuleCollider.WorldAxis,
                capsuleCollider.AxisHalfLength,
                capsuleCollider.ScaledRadius,
                Fixed64.Zero,
                _segmentLength,
                out Fixed64 entry,
                out Fixed64 exit,
                out bool startContained,
                out bool endContainedStrict))
        {
            return false;
        }

        if (!_calculateIntersections)
            return true;

        AddFiniteAxisIntersectionInterval(
            entry,
            exit,
            startContained,
            endContainedStrict,
            ref outputIntersectionPoints);
        return true;
    }

    public bool CheckCylinderOverlaps(LSCylinderCollider cylinderCollider, ref SwiftList<Vector3d> outputIntersectionPoints)
    {
        if (!_segmentIsValid)
            return false;

        var query = new FixedSegment(_cachedOrigin, _cachedEnd);
        if (!query.TryGetFiniteCylinderIntersectionDistanceInterval(
                cylinderCollider.Center,
                cylinderCollider.WorldAxis,
                cylinderCollider.HalfHeight,
                cylinderCollider.ScaledRadius,
                Fixed64.Zero,
                Fixed64.Zero,
                _segmentLength,
                out Fixed64 entry,
                out Fixed64 exit,
                out bool startContained,
                out bool endContainedStrict))
        {
            return false;
        }

        if (!_calculateIntersections)
            return true;

        AddFiniteAxisIntersectionInterval(
            entry,
            exit,
            startContained,
            endContainedStrict,
            ref outputIntersectionPoints);
        return true;
    }

    public bool CheckConeOverlaps(LSConeCollider coneCollider, ref SwiftList<Vector3d> outputIntersectionPoints)
    {
        if (!_segmentIsValid)
            return false;

        var query = new FixedSegment(_cachedOrigin, _cachedEnd);
        if (!query.TryGetCenteredFiniteConeIntersectionDistanceInterval(
                coneCollider.Center,
                coneCollider.Axis,
                coneCollider.Height,
                coneCollider.ScaledRadius,
                _segmentLength,
                out Fixed64 entry,
                out Fixed64 exit,
                out bool startContained,
                out bool endContainedStrict))
        {
            return false;
        }

        if (!_calculateIntersections)
            return true;

        AddFiniteAxisIntersectionInterval(
            entry,
            exit,
            startContained,
            endContainedStrict,
            ref outputIntersectionPoints);

        return true;
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

    public bool CheckAABBoxOverlaps(LSCuboidCollider aabox, ref SwiftList<Vector3d> outputIntersectionPoints) =>
         CheckAABBoxOverlaps(aabox.BoundsMin, aabox.BoundsMax, ref outputIntersectionPoints);

    /// <summary>
    /// Checks whether an axis-aligned bounding box overlaps this worker's prepared ray segment.
    /// </summary>
    public bool CheckAABBoxOverlaps(Vector3d min, Vector3d max, ref SwiftList<Vector3d> outputIntersectionPoints)
    {
        if (!_segmentIsValid)
            return false;

        if (_cachedSegment.IsZero)
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
        Vector3d halfExtents = oobox.ScaledSize * Fixed64.Half;
        Vector3d min = -halfExtents;
        Vector3d max = halfExtents;
        if (!Vector3d.TrySubtract(_cachedOrigin, oobox.Center, out Vector3d originOffset)
            || !Vector3d.TrySubtract(_cachedEnd, oobox.Center, out Vector3d endOffset)
            || !TryTransformObbOffsetToLocal(originOffset, oobox.Rotation, inverseRotation, out Vector3d transformedOrigin)
            || !TryTransformObbOffsetToLocal(endOffset, oobox.Rotation, inverseRotation, out Vector3d transformedEnd))
        {
            return false;
        }

        Vector3d localOrigin = SnapLocalPointToBounds(
            transformedOrigin,
            min,
            max);
        Vector3d localEnd = SnapLocalPointToBounds(
            transformedEnd,
            min,
            max);

        if (!Vector3d.TrySubtract(localEnd, localOrigin, out Vector3d localSegment)
            || !Vector3d.TryGetMagnitude(localSegment, out Fixed64 localLength))
        {
            return false;
        }

        if (localLength == Fixed64.Zero)
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

        Vector3d localDirection = localSegment / localLength;
        if (!SweepBoundsUtility.TryClipSegment(
            localOrigin,
            localDirection,
            localLength,
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

    private static bool TryTransformObbOffsetToLocal(
        Vector3d worldOffset,
        FixedQuaternion rotation,
        FixedQuaternion inverseRotation,
        out Vector3d localOffset)
    {
        localOffset = inverseRotation * worldOffset;

        // Quaternion-vector multiplication is a rounded Q32.32 matrix
        // transform. One deterministic iterative-refinement step solves the
        // represented rotation matrix more accurately at large coordinates,
        // keeping tangent classification scale independent without widening
        // the geometric box.
        // The collider rotation is normalized, so rotating the inverse result
        // back cannot cross to the opposite Fixed64 domain extreme.
        Vector3d residual = worldOffset - rotation * localOffset;
        if (residual == Vector3d.Zero)
            return true;

        return Vector3d.TryAdd(localOffset, inverseRotation * residual, out localOffset);
    }

    private static Vector3d SnapLocalPointToBounds(Vector3d point, Vector3d min, Vector3d max) =>
        new(
            SnapLocalCoordinateToBounds(point.X, min.X, max.X),
            SnapLocalCoordinateToBounds(point.Y, min.Y, max.Y),
            SnapLocalCoordinateToBounds(point.Z, min.Z, max.Z));

    private static Fixed64 SnapLocalCoordinateToBounds(Fixed64 value, Fixed64 min, Fixed64 max)
    {
        if ((value - min).Abs() <= Fixed64.Epsilon)
            return min;

        return (value - max).Abs() <= Fixed64.Epsilon ? max : value;
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
        var triangle = new FixedTriangle(first, second, third);
        if (localSegmentLengthSqr == Fixed64.Zero)
        {
            if (Vector3d.Dot(localOrigin - first, normal).Abs() > Fixed64.Epsilon
                || !triangle.ContainsProjection(localOrigin))
            {
                return false;
            }

            AddIntersectionPoint(mesh.ConvertLocalToWorld(localOrigin), ref outputIntersectionPoints);
            return true;
        }

        Fixed64 denominator = Vector3d.Dot(normal, localSegmentDirection);
        if (denominator.Abs() <= Fixed64.Epsilon)
        {
            if (Vector3d.Dot(localOrigin - first, normal).Abs() > Fixed64.Epsilon)
                return false;

            bool found = false;
            if (triangle.ContainsProjection(localOrigin))
            {
                AddIntersectionPoint(mesh.ConvertLocalToWorld(localOrigin), ref outputIntersectionPoints);
                found = true;
            }

            if (triangle.ContainsProjection(localEnd))
            {
                AddIntersectionPoint(mesh.ConvertLocalToWorld(localEnd), ref outputIntersectionPoints);
                found = true;
            }

            return found;
        }

        Fixed64 distance = Vector3d.Dot(first - localOrigin, normal) / denominator;
        if (distance < Fixed64.Zero || distance > localSegmentLength)
            return false;

        Vector3d localPoint = localOrigin + localSegmentDirection * distance;
        if (!triangle.ContainsProjection(localPoint))
            return false;

        AddIntersectionPoint(mesh.ConvertLocalToWorld(localPoint), ref outputIntersectionPoints);
        return true;
    }

    private void AddIntersectionPoint(Vector3d point, ref SwiftList<Vector3d> outputIntersectionPoints)
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

    private void AddLocalIntersectionPoint(
        Vector3d center,
        FixedQuaternion rotation,
        Vector3d localPoint,
        ref SwiftList<Vector3d> outputIntersectionPoints)
    {
        Vector3d worldPoint = center + rotation * localPoint;
        AddIntersectionPoint(worldPoint, ref outputIntersectionPoints);
    }

    private bool CheckPointInsideSphere(
        FixedBoundSphere sphere,
        ref SwiftList<Vector3d> outputIntersectionPoints)
    {
        if (!sphere.Contains(_cachedOrigin))
            return false;

        if (_calculateIntersections)
            outputIntersectionPoints.Add(_cachedOrigin);

        return true;
    }

    private void AddFiniteAxisIntersectionInterval(
        Fixed64 entry,
        Fixed64 exit,
        bool startContained,
        bool endInsideStrict,
        ref SwiftList<Vector3d> outputIntersectionPoints)
    {
        AddSegmentIntersectionPoint(entry, ref outputIntersectionPoints);
        if (!startContained
            && exit != entry
            && (exit < _segmentLength || !endInsideStrict))
        {
            AddSegmentIntersectionPoint(exit, ref outputIntersectionPoints);
        }

    }

    private void AddSegmentIntersectionPoint(
        Fixed64 distance,
        ref SwiftList<Vector3d> outputIntersectionPoints) =>
        outputIntersectionPoints.Add(new FixedSegment(_cachedOrigin, _cachedEnd)
            .GetPointAtDistance(distance, _segmentLength));

    private void AddSegmentParameterIntersectionPoint(
        Fixed64 parameter,
        ref SwiftList<Vector3d> outputIntersectionPoints) =>
        outputIntersectionPoints.Add(
            parameter == Fixed64.One
                ? _cachedEnd
                : Vector3d.Lerp(_cachedOrigin, _cachedEnd, parameter));

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

}
