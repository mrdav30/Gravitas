//=======================================================================
// RaycastSegmentWorker.cs
//=======================================================================
// MIT License, Copyright (c) 2026–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp;
using FixedMathSharp.Geometry;
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
    /// Receives segment points reconstructed from exact physical intersection distances.
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

        var query = new FixedSegment(_cachedOrigin, _cachedEnd);
        if (!query.TryGetSphereIntersectionDistanceInterval(
                sphere,
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

    public bool CheckCapsuleOverlaps(LSCapsuleCollider capsuleCollider, ref SwiftList<Vector3d> outputIntersectionPoints)
    {
        if (!_segmentIsValid)
            return false;

        var query = new FixedSegment(_cachedOrigin, _cachedEnd);
        if (!query.TryGetCapsuleIntersectionDistanceInterval(
                capsuleCollider.Center,
                capsuleCollider.Rotation,
                capsuleCollider.AxisLength,
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
                cylinderCollider.Rotation,
                cylinderCollider.Height,
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
                coneCollider.Rotation,
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

        if (!meshCollider.Mesh.TryConvertWorldToScaledLocal(
                _cachedOrigin,
                out Vector3d localOrigin)
            || !meshCollider.Mesh.TryConvertWorldToScaledLocal(
                _cachedEnd,
                out Vector3d localEnd))
        {
            return false;
        }
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

        var ray = new FixedRay(_cachedOrigin, _cachedSegment);
        if (!oobox.OrientedBox.TryGetRayIntersectionInterval(
                ray,
                Fixed64.One,
                out Fixed64 entry,
                out Fixed64 exit))
            return false;

        if (_calculateIntersections)
        {
            // The interval is clipped to this segment, so both parameters are
            // convex combinations of its representable endpoints.
            _ = ray.TryGetPoint(entry, out Vector3d entryPoint);
            Vector3d exitPoint = default;
            if (exit != entry)
                _ = ray.TryGetPoint(exit, out exitPoint);

            AddIntersectionPoint(entryPoint, ref outputIntersectionPoints);
            if (exit != entry)
                AddIntersectionPoint(exitPoint, ref outputIntersectionPoints);
        }

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
        var triangle = new FixedTriangle(first, second, third);
        if (localSegmentLengthSqr == Fixed64.Zero)
        {
            if (Vector3d.Dot(localOrigin - first, normal).Abs() > Fixed64.Epsilon
                || !triangle.ContainsProjection(localOrigin))
            {
                return false;
            }

            // localOrigin was obtained from this representable world endpoint.
            _ = mesh.TryConvertScaledLocalToWorld(
                localOrigin,
                out Vector3d worldOrigin);

            AddIntersectionPoint(worldOrigin, ref outputIntersectionPoints);
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
                _ = mesh.TryConvertScaledLocalToWorld(
                    localOrigin,
                    out Vector3d worldOrigin);

                AddIntersectionPoint(worldOrigin, ref outputIntersectionPoints);
                found = true;
            }

            if (triangle.ContainsProjection(localEnd))
            {
                _ = mesh.TryConvertScaledLocalToWorld(
                    localEnd,
                    out Vector3d worldEnd);

                AddIntersectionPoint(worldEnd, ref outputIntersectionPoints);
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

        // A point on the local segment maps to the same convex combination of
        // the already-representable world endpoints.
        _ = mesh.TryConvertScaledLocalToWorld(
            localPoint,
            out Vector3d worldPoint);

        AddIntersectionPoint(worldPoint, ref outputIntersectionPoints);
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
