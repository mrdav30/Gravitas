using FixedMathSharp;
using Gravitas.Colliders;
using SwiftCollections;

namespace Gravitas.Raycasting;

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
    private bool _calculateIntersections;

    /// <summary>
    /// Prepares this worker for overlap checks against the line segment between two points.
    /// </summary>
    public void PrepareSegmentCheck(Vector3d p1, Vector3d p2, bool calculateIntersectionPoints = true)
    {
        _cachedOrigin = p1;
        _cachedEnd = p2;

        Vector3d segment = p2 - p1;
        _segmentLengthSqr = segment.SqrMagnitude;
        _segmentLength = _segmentLengthSqr == Fixed64.Zero ? Fixed64.Zero : segment.Magnitude;
        _segmentDirection = _segmentLength == Fixed64.Zero ? Vector3d.Zero : segment / _segmentLength;
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
        if (_segmentLengthSqr == Fixed64.Zero)
            return CheckPointInsideSphere(position, sqrRadius, ref outputIntersectionPoints);

        Vector3d originToCenter = position - _cachedOrigin;
        Fixed64 closestParameter = Vector3d.Dot(originToCenter, _segmentDirection);
        closestParameter = FixedMath.Clamp(closestParameter, Fixed64.Zero, _segmentLength);
        Vector3d closestPoint = _cachedOrigin + _segmentDirection * closestParameter;
        if ((closestPoint - position).SqrMagnitude > sqrRadius)
            return false;

        if (!_calculateIntersections)
            return true;

        Vector3d originFromCenter = _cachedOrigin - position;
        Fixed64 c = originFromCenter.SqrMagnitude - sqrRadius;
        if (c <= Fixed64.Zero)
        {
            outputIntersectionPoints.Add(_cachedOrigin);
            return true;
        }

        Fixed64 b = Vector3d.Dot(originFromCenter, _segmentDirection);
        Fixed64 discriminant = b * b - c;
        if (discriminant < Fixed64.Zero)
            return false;

        Fixed64 root = FixedMath.Sqrt(discriminant);
        AddIntersectionPointIfOnSegment(-b - root, ref outputIntersectionPoints);
        if (root != Fixed64.Zero)
            AddIntersectionPointIfOnSegment(-b + root, ref outputIntersectionPoints);

        return outputIntersectionPoints.Count > 0;
    }

    public bool CheckCapsuleOverlaps(LSCapsuleCollider capsuleCollider, ref SwiftList<Vector3d> outputIntersectionPoints)
    {
        bool intersects = CheckCylinderOverlaps(capsuleCollider, ref outputIntersectionPoints);

        if (!intersects)
        {
            intersects = CheckSphereOverlaps(capsuleCollider.LineSegmentEnd, capsuleCollider.ScaledRadiusSqr, ref outputIntersectionPoints)
                         || CheckSphereOverlaps(capsuleCollider.LineSegmentStart, capsuleCollider.ScaledRadiusSqr, ref outputIntersectionPoints);
        }

        return intersects;
    }

    private bool CheckCylinderOverlaps(LSCapsuleCollider capsuleCollider, ref SwiftList<Vector3d> outputIntersectionPoints)
    {
        Vector3d transformedRayOrigin = (_cachedOrigin - capsuleCollider.Center) * capsuleCollider.Rotation.Inverse();
        Vector3d transformedRayDirection = _segmentDirection * capsuleCollider.Rotation.Inverse();

        Fixed64 a = transformedRayDirection.x * transformedRayDirection.x + transformedRayDirection.z * transformedRayDirection.z;
        if (a == Fixed64.Zero)
            return false;

        Fixed64 b = 2 * (transformedRayOrigin.x * transformedRayDirection.x + transformedRayOrigin.z * transformedRayDirection.z);
        Fixed64 c = transformedRayOrigin.x * transformedRayOrigin.x + transformedRayOrigin.z * transformedRayOrigin.z - capsuleCollider.ScaledRadiusSqr;

        Fixed64 discriminant = b * b - 4 * a * c;
        if (discriminant < Fixed64.Zero)
            return false;

        Fixed64 root = FixedMath.Sqrt(discriminant);
        Fixed64 t1 = (-b - root) / (2 * a);
        Fixed64 t2 = (-b + root) / (2 * a);

        bool intersects = false;
        intersects |= TryAddCapsuleCylinderPoint(capsuleCollider, transformedRayOrigin, transformedRayDirection, t1, ref outputIntersectionPoints);
        if (t2 != t1)
            intersects |= TryAddCapsuleCylinderPoint(capsuleCollider, transformedRayOrigin, transformedRayDirection, t2, ref outputIntersectionPoints);

        return intersects;
    }

    public bool CheckAABBoxOverlaps(LSCuboidCollider aabox, ref SwiftList<Vector3d> outputIntersectionPoints) =>
         CheckAABBoxOverlaps(aabox.BoundsMin, aabox.BoundsMax, ref outputIntersectionPoints);

    /// <summary>
    /// Checks whether an axis-aligned bounding box overlaps this worker's prepared ray segment.
    /// </summary>
    public bool CheckAABBoxOverlaps(Vector3d min, Vector3d max, ref SwiftList<Vector3d> outputIntersectionPoints)
    {
        if (_segmentLengthSqr == Fixed64.Zero)
            return CheckPointInsideBox(min, max, ref outputIntersectionPoints);

        Fixed64 entry = Fixed64.Zero;
        Fixed64 exit = _segmentLength;

        if (!ClipSegmentAxis(_cachedOrigin.x, _segmentDirection.x, min.x, max.x, ref entry, ref exit)
            || !ClipSegmentAxis(_cachedOrigin.y, _segmentDirection.y, min.y, max.y, ref entry, ref exit)
            || !ClipSegmentAxis(_cachedOrigin.z, _segmentDirection.z, min.z, max.z, ref entry, ref exit))
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
                    outputIntersectionPoints.Add(_cachedOrigin + (_segmentDirection * t));
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

        Vector3d directionCrossEdge2 = Vector3d.Cross(_segmentDirection, edge2Displacement);
        Fixed64 determinant = Vector3d.Dot(edge1Displacement, directionCrossEdge2);

        if (determinant > -Fixed64.Epsilon && determinant < Fixed64.Epsilon)
            return false;

        Fixed64 invDeterminant = Fixed64.One / determinant;

        Vector3d originDifference = _cachedOrigin - triangle[0];
        Fixed64 u = Vector3d.Dot(originDifference, directionCrossEdge2) * invDeterminant;

        if (u < Fixed64.Zero || u > Fixed64.One)
            return false;

        Vector3d originDifferenceCrossEdge1 = Vector3d.Cross(originDifference, edge1Displacement);
        Fixed64 v = Vector3d.Dot(_segmentDirection, originDifferenceCrossEdge1) * invDeterminant;

        if (v < Fixed64.Zero || u + v > Fixed64.One)
            return false;

        t = Vector3d.Dot(edge2Displacement, originDifferenceCrossEdge1) * invDeterminant;

        return t > Fixed64.Epsilon && t <= _segmentLength;
    }

    private bool TryAddCapsuleCylinderPoint(
        LSCapsuleCollider capsuleCollider,
        Vector3d transformedRayOrigin,
        Vector3d transformedRayDirection,
        Fixed64 distance,
        ref SwiftList<Vector3d> outputIntersectionPoints)
    {
        if (distance < Fixed64.Zero || distance > _segmentLength)
            return false;

        Vector3d point = transformedRayOrigin + transformedRayDirection * distance;
        if (point.y < -capsuleCollider.Bounds.Scope.y || point.y > capsuleCollider.Bounds.Scope.y)
            return false;

        if (_calculateIntersections)
            outputIntersectionPoints.Add(capsuleCollider.Center + point);

        return true;
    }

    private bool CheckPointInsideSphere(
        Vector3d position,
        Fixed64 sqrRadius,
        ref SwiftList<Vector3d> outputIntersectionPoints)
    {
        if ((_cachedOrigin - position).SqrMagnitude > sqrRadius)
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
        if (_cachedOrigin.x < min.x || _cachedOrigin.x > max.x
            || _cachedOrigin.y < min.y || _cachedOrigin.y > max.y
            || _cachedOrigin.z < min.z || _cachedOrigin.z > max.z)
        {
            return false;
        }

        if (_calculateIntersections)
            outputIntersectionPoints.Add(_cachedOrigin);

        return true;
    }

    private void AddIntersectionPointIfOnSegment(Fixed64 distance, ref SwiftList<Vector3d> outputIntersectionPoints)
    {
        if (distance < Fixed64.Zero || distance > _segmentLength)
            return;

        outputIntersectionPoints.Add(_cachedOrigin + _segmentDirection * distance);
    }

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

        Fixed64 t1 = (min - position) / direction;
        Fixed64 t2 = (max - position) / direction;

        if (t1 > t2)
            (t1, t2) = (t2, t1);

        if (t1 > entry)
            entry = t1;

        if (t2 < exit)
            exit = t2;

        return entry <= exit;
    }
}
