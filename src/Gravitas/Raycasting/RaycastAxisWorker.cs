using FixedMathSharp;
using Gravitas.Colliders;
using SwiftCollections;

namespace Gravitas.Raycasting;

/// <summary>
/// Stores the axis data used by one context-owned raycast service while checking collider overlaps.
/// </summary>
public sealed class RaycastAxisWorker
{
    private Vector3d _cachedOrigin;
    private Vector3d _cachedEnd;

    private Fixed64 _axisMin;
    private Fixed64 _axisMax;
    private Vector3d _cacheAxisDirection;
    private Vector3d _cacheAxisNormal;
    private Vector3d _perpVector;
    private Fixed64 _cacheProjPerp;

    private bool _calculateIntersections;

    /// <summary>
    /// Prepares this worker for overlap checks against the line segment between two points.
    /// </summary>
    public void PrepareAxisCheck(Vector3d p1, Vector3d p2, bool calculateIntersectionPoints = true)
    {
        _cachedOrigin = p1;
        _cachedEnd = p2;
        _cacheAxisDirection = (p2 - p1).Normal;
        _cacheAxisNormal = _cacheAxisDirection.LeftHandNormal;

        _axisMin = p1.Dot(_cacheAxisDirection.x, _cacheAxisDirection.y, _cacheAxisDirection.z);
        _axisMax = p2.Dot(_cacheAxisDirection.x, _cacheAxisDirection.y, _cacheAxisDirection.z);
        _cacheProjPerp = _cachedOrigin.Dot(_cacheAxisNormal.x, _cacheAxisNormal.y, _cacheAxisNormal.z);
        _perpVector = _cacheAxisNormal * _cacheProjPerp;

        _calculateIntersections = calculateIntersectionPoints;
    }

    public bool CheckSphereOverlaps(LSSphereCollider sphereCollider, ref SwiftList<Vector3d> outputIntersectionPoints) =>
        CheckSphereOverlaps(sphereCollider.Center, sphereCollider.ScaledRadius, sphereCollider.ScaledRadiusSqr, ref outputIntersectionPoints);

    /// <summary>
    /// Checks whether a sphere overlaps this worker's prepared ray segment.
    /// </summary>
    public bool CheckSphereOverlaps(
        Vector3d position,
        Fixed64 radius,
        Fixed64 sqrRadius,
        ref SwiftList<Vector3d> outputIntersectionPoints)
    {
        Fixed64 projPos = position.Dot(_cacheAxisDirection.x, _cacheAxisDirection.y, _cacheAxisDirection.z);
        if (projPos >= _axisMin && projPos <= _axisMax)
        {
            Fixed64 projPerp = position.Dot(_cacheAxisNormal.x, _cacheAxisNormal.y, _cacheAxisNormal.z);
            Fixed64 perpDif = _cacheProjPerp - projPerp;
            Fixed64 perpDist = perpDif.Abs();

            if (perpDist <= radius)
            {
                if (!_calculateIntersections)
                    return true;

                Fixed64 sin = perpDif;
                Fixed64 cos = FixedMath.Sqrt(sqrRadius - sin * sin);
                if (cos == Fixed64.Zero)
                {
                    outputIntersectionPoints.Add((_cacheAxisDirection * projPos) + _perpVector);
                }
                else
                {
                    outputIntersectionPoints.Add(_cacheAxisDirection * (projPos - cos) + _perpVector);
                    outputIntersectionPoints.Add(_cacheAxisDirection * (projPos + cos) + _perpVector);
                }

                return true;
            }
        }

        Fixed64 p1Dist = position.SqrDistance(_cachedOrigin.x, _cachedOrigin.y, _cachedOrigin.z);
        if (p1Dist <= sqrRadius)
        {
            if (_calculateIntersections)
                outputIntersectionPoints.Add(_cachedOrigin);
            return true;
        }

        Fixed64 p2Dist = position.SqrDistance(_cachedEnd.x, _cachedEnd.y, _cachedEnd.z);
        if (p2Dist <= sqrRadius)
        {
            if (_calculateIntersections)
                outputIntersectionPoints.Add(_cachedEnd);
            return true;
        }

        return false;
    }

    public bool CheckCapsuleOverlaps(LSCapsuleCollider capsuleCollider, ref SwiftList<Vector3d> outputIntersectionPoints)
    {
        bool intersects = CheckCylinderOverlaps(capsuleCollider, ref outputIntersectionPoints);

        if (!intersects)
        {
            intersects = CheckSphereOverlaps(capsuleCollider.LineSegmentEnd, capsuleCollider.ScaledRadius, capsuleCollider.ScaledRadiusSqr, ref outputIntersectionPoints)
                         || CheckSphereOverlaps(capsuleCollider.LineSegmentStart, capsuleCollider.ScaledRadius, capsuleCollider.ScaledRadiusSqr, ref outputIntersectionPoints);
        }

        return intersects;
    }

    private bool CheckCylinderOverlaps(LSCapsuleCollider capsuleCollider, ref SwiftList<Vector3d> outputIntersectionPoints)
    {
        Vector3d transformedRayOrigin = _cachedOrigin - capsuleCollider.Center;
        transformedRayOrigin = transformedRayOrigin * capsuleCollider.Rotation.Inverse();
        Vector3d transformedRayDirection = _cacheAxisDirection * capsuleCollider.Rotation.Inverse();

        Fixed64 a = transformedRayDirection.x * transformedRayDirection.x + transformedRayDirection.z * transformedRayDirection.z;
        Fixed64 b = 2 * (transformedRayOrigin.x * transformedRayDirection.x + transformedRayOrigin.z * transformedRayDirection.z);
        Fixed64 c = transformedRayOrigin.x * transformedRayOrigin.x + transformedRayOrigin.z * transformedRayOrigin.z - capsuleCollider.ScaledRadiusSqr;

        Fixed64 discriminant = b * b - 4 * a * c;
        if (discriminant < Fixed64.Zero)
            return false;

        Fixed64 t1 = (-b - FixedMath.Sqrt(discriminant)) / (2 * a);
        Fixed64 t2 = (-b + FixedMath.Sqrt(discriminant)) / (2 * a);

        Vector3d p1 = transformedRayOrigin + _cacheAxisDirection * t1;
        Vector3d p2 = transformedRayOrigin + _cacheAxisDirection * t2;

        bool intersects = false;

        if (p1.y >= -capsuleCollider.Bounds.Scope.y && p1.y <= capsuleCollider.Bounds.Scope.y)
        {
            if (_calculateIntersections)
                outputIntersectionPoints.Add(capsuleCollider.Center + p1);
            intersects = true;
        }

        if (p2.y >= -capsuleCollider.Bounds.Scope.y && p2.y <= capsuleCollider.Bounds.Scope.y)
        {
            if (_calculateIntersections)
                outputIntersectionPoints.Add(capsuleCollider.Center + p2);
            intersects = true;
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
        Fixed64 minDistance = Fixed64.MAX_VALUE;
        bool intersects = false;

        CheckAabbAxis(min.x, max.x, ref minDistance, ref intersects, ref outputIntersectionPoints);
        CheckAabbAxis(min.y, max.y, ref minDistance, ref intersects, ref outputIntersectionPoints);
        CheckAabbAxis(min.z, max.z, ref minDistance, ref intersects, ref outputIntersectionPoints);

        return intersects;
    }

    private void CheckAabbAxis(
        Fixed64 min,
        Fixed64 max,
        ref Fixed64 minDistance,
        ref bool intersects,
        ref SwiftList<Vector3d> outputIntersectionPoints)
    {
        FixedRange boxRange = new(min, max);
        FixedRange lineRange = new(_axisMin, _axisMax);
        if (FixedRange.ComputeOverlapDepth(boxRange, lineRange) <= Fixed64.Zero)
            return;

        Fixed64 t1 = (boxRange.Min - _axisMin) / (_axisMax - _axisMin);
        Fixed64 t2 = (boxRange.Max - _axisMin) / (_axisMax - _axisMin);

        Vector3d intersection1 = Vector3d.UnclampedLerp(_cachedOrigin, _cachedEnd, t1);
        Vector3d intersection2 = Vector3d.UnclampedLerp(_cachedOrigin, _cachedEnd, t2);

        Fixed64 distance1 = (intersection1 - _cachedOrigin).Magnitude;
        Fixed64 distance2 = (intersection2 - _cachedOrigin).Magnitude;

        if (distance1 < minDistance)
        {
            if (_calculateIntersections)
                outputIntersectionPoints.Add(intersection1);
            minDistance = distance1;
            intersects = true;
        }

        if (distance2 < minDistance)
        {
            if (_calculateIntersections)
                outputIntersectionPoints.Add(intersection2);
            minDistance = distance2;
            intersects = true;
        }
    }

    /// <summary>
    /// Checks whether an oriented bounding box overlaps this worker's prepared ray segment.
    /// </summary>
    public bool CheckOBBoxOverlaps(LSCuboidCollider oobox, ref SwiftList<Vector3d> outputIntersectionPoints)
    {
        if (!CheckAABBoxOverlaps(oobox.BoundsMin, oobox.BoundsMax, ref outputIntersectionPoints))
            return false;

        for (int i = 0; i < outputIntersectionPoints.Count; i++)
        {
            Vector3d worldSpaceIntersection = outputIntersectionPoints[i].Rotate(oobox.Position, oobox.Rotation);
            outputIntersectionPoints[i] = worldSpaceIntersection;
        }

        return true;
    }

    public bool CheckMeshOverlaps(LSMeshCollider collider, ref SwiftList<Vector3d> outputIntersectionPoints)
    {
        for (int i = 0; i < collider.Mesh.Vertices.Length; i += 3)
        {
            Vector3d[] triangle = new Vector3d[3]
            {
                    collider.Mesh.Vertices[i],
                    collider.Mesh.Vertices[i + 1],
                    collider.Mesh.Vertices[i + 2]
            };

            if (RayTriangleIntersection(triangle, out Fixed64 t))
            {
                if (_calculateIntersections)
                    outputIntersectionPoints.Add(_cachedOrigin + (_cacheAxisDirection * t));
                return true;
            }
        }

        return false;
    }

    public bool RayTriangleIntersection(Vector3d[] triangle, out Fixed64 t)
    {
        t = Fixed64.Zero;

        Vector3d edge1Displacement = triangle[1] - triangle[0];
        Vector3d edge2Displacement = triangle[2] - triangle[1];

        Vector3d directionCrossEdge2 = Vector3d.Cross(_cacheAxisDirection, edge2Displacement);
        Fixed64 determinant = Vector3d.Dot(edge1Displacement, directionCrossEdge2);

        if (determinant > -Fixed64.Epsilon && determinant < Fixed64.Epsilon)
            return false;

        Fixed64 invDeterminant = Fixed64.One / determinant;

        Vector3d originDifference = _cachedOrigin - triangle[0];
        Fixed64 u = Vector3d.Dot(originDifference, directionCrossEdge2) * invDeterminant;

        if (u < Fixed64.Zero || u > Fixed64.One)
            return false;

        Vector3d originDifferenceCrossEdge1 = Vector3d.Cross(originDifference, edge1Displacement);
        Fixed64 v = Vector3d.Dot(_cacheAxisDirection, originDifferenceCrossEdge1) * invDeterminant;

        if (v < Fixed64.Zero || u + v > Fixed64.One)
            return false;

        t = Vector3d.Dot(edge2Displacement, originDifferenceCrossEdge1) * invDeterminant;

        return t > Fixed64.Epsilon;
    }
}
