using FixedMathSharp;
using Gravitas.Colliders;
using SwiftCollections;

namespace Gravitas.Raycasting;

/// <summary>
/// This class contains methods used for raycasting checks with various types of colliders.
/// </summary>
public static class RayCasterWorker
{
    // Cached variables used for raycasting
    private static Vector3d _cachedOrigin;
    private static Vector3d _cachedEnd;

    private static Fixed64 _axisMin;
    private static Fixed64 _axisMax;
    private static Vector3d _cacheAxisDirection;
    private static Vector3d _cacheAxisNormal;
    private static Vector3d _perpVector;
    private static Fixed64 _cacheProjPerp;

    private static bool _calculateIntersections;

    /// <summary>
    /// Prepares for the check of the overlap with the specified axis.
    /// </summary>
    public static void PrepareAxisCheck(Vector3d p1, Vector3d p2, bool calculateIntersectionPoints = true)
    {
        _cachedOrigin = p1;  // start point
        _cachedEnd = p2;  // end point
        _cacheAxisDirection = p2 - p1;
        _cacheAxisNormal = _cacheAxisDirection.LeftHandNormal;

        //Debug.DrawRay(_cacheP1.ToVector3(), _cacheAxisDirection.ToVector3(), Color.red);

        _axisMin = p1.Dot(_cacheAxisDirection.x, _cacheAxisDirection.y, _cacheAxisDirection.z);
        _axisMax = p2.Dot(_cacheAxisDirection.x, _cacheAxisDirection.y, _cacheAxisDirection.z);
        _cacheProjPerp = _cachedOrigin.Dot(_cacheAxisNormal.x, _cacheAxisNormal.y, _cacheAxisNormal.z);
        _perpVector = _cacheAxisNormal * _cacheProjPerp;

        _calculateIntersections = calculateIntersectionPoints;
    }

    public static bool CheckSphereOverlaps(LSSphereCollider sphereCollider, ref SwiftList<Vector3d> outputIntersectionPoints) =>
        CheckSphereOverlaps(sphereCollider.Center, sphereCollider.ScaledRadius, sphereCollider.ScaledRadiusSqr, ref outputIntersectionPoints);

    /// <summary>
    /// Checks if a circle collider overlaps with the ray. If there is an overlap, intersection points are calculated.
    /// </summary>
    public static bool CheckSphereOverlaps(
        Vector3d position,
        Fixed64 radius,
        Fixed64 sqrRadius,
        ref SwiftList<Vector3d> outputIntersectionPoints)
    {
        // Project circle center onto line
        Fixed64 projPos = position.Dot(_cacheAxisDirection.x, _cacheAxisDirection.y, _cacheAxisDirection.z);
        if (projPos >= _axisMin && projPos <= _axisMax)
        {
            // Calculate perpendicular distance from line to center of circle
            Fixed64 projPerp = position.Dot(_cacheAxisNormal.x, _cacheAxisNormal.y, _cacheAxisNormal.z);
            Fixed64 perpDif = _cacheProjPerp - projPerp;
            Fixed64 perpDist = perpDif.Abs();
            // If circle overlaps with the line, calculate intersection points
            if (perpDist <= radius)
            {
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

        // If circle does not overlap with the line, check if it overlaps with the end points
        Fixed64 p1Dist = position.SqrDistance(_cachedOrigin.x, _cachedOrigin.y, _cachedOrigin.z);
        if (p1Dist <= sqrRadius)
        {
            outputIntersectionPoints.Add(_cachedOrigin);
            return true;
        }

        Fixed64 p2Dist = position.SqrDistance(_cachedEnd.x, _cachedEnd.y, _cachedEnd.z);
        if (p2Dist <= sqrRadius)
        {
            outputIntersectionPoints.Add(_cachedEnd);
            return true;
        }

        return false;
    }

    public static bool CheckCapsuleOverlaps(LSCapsuleCollider capsuleCollider, ref SwiftList<Vector3d> outputIntersectionPoints)
    {
        // Check if the ray intersects the cylindrical part of the capsule
        bool intersects = CheckCylinderOverlaps(capsuleCollider, ref outputIntersectionPoints);

        // If the ray does not intersect the cylindrical part, check if it intersects the hemispherical parts
        if (!intersects)
        {
            intersects = CheckSphereOverlaps(capsuleCollider.LineSegmentEnd, capsuleCollider.ScaledRadius, capsuleCollider.ScaledRadiusSqr, ref outputIntersectionPoints)
                         || CheckSphereOverlaps(capsuleCollider.LineSegmentStart, capsuleCollider.ScaledRadius, capsuleCollider.ScaledRadiusSqr, ref outputIntersectionPoints);
        }

        return intersects;
    }

    private static bool CheckCylinderOverlaps(LSCapsuleCollider capsuleCollider, ref SwiftList<Vector3d> outputIntersectionPoints)
    {
        Vector3d transformedRayOrigin = _cachedOrigin - capsuleCollider.Center;

        // Add these lines to rotate the ray into the capsule's local coordinate system
        transformedRayOrigin = transformedRayOrigin * capsuleCollider.Rotation.Inverse();
        Vector3d transformedRayDirection = _cacheAxisDirection * capsuleCollider.Rotation.Inverse();

        // Replace '_cacheAxisDirection' with 'transformedRayDirection'
        Fixed64 a = transformedRayDirection.x * transformedRayDirection.x + transformedRayDirection.z * transformedRayDirection.z;
        Fixed64 b = 2 * (transformedRayOrigin.x * transformedRayDirection.x + transformedRayOrigin.z * transformedRayDirection.z);
        Fixed64 c = transformedRayOrigin.x * transformedRayOrigin.x + transformedRayOrigin.z * transformedRayOrigin.z - capsuleCollider.ScaledRadiusSqr;

        // Solve the quadratic equation to find the values of t for which the ray intersects the cylinder
        Fixed64 discriminant = b * b - 4 * a * c;
        if (discriminant < Fixed64.Zero)
        {
            // The ray does not intersect the cylinder
            return false;
        }
        else
        {
            Fixed64 t1 = (-b - FixedMath.Sqrt(discriminant)) / (2 * a);
            Fixed64 t2 = (-b + FixedMath.Sqrt(discriminant)) / (2 * a);

            // Check if the intersection points are within the bounds of the cylinder
            Vector3d p1 = transformedRayOrigin + _cacheAxisDirection * t1;
            Vector3d p2 = transformedRayOrigin + _cacheAxisDirection * t2;

            bool intersects = false;

            if (p1.y >= -capsuleCollider.Bounds.Scope.y && p1.y <= capsuleCollider.Bounds.Scope.y)
            {
                // The first intersection point is within the bounds of the cylinder
                // Transform it back into the original coordinate system and add it to the list of intersection points
                outputIntersectionPoints.Add(capsuleCollider.Center + p1);
                intersects = true;
            }

            if (p2.y >= -capsuleCollider.Bounds.Scope.y && p2.y <= capsuleCollider.Bounds.Scope.y)
            {
                // The second intersection point is within the bounds of the cylinder
                // Transform it back into the original coordinate system and add it to the list of intersection points
                outputIntersectionPoints.Add(capsuleCollider.Center + p2);
                intersects = true;
            }

            return intersects;
        }
    }

    public static bool CheckAABBoxOverlaps(LSCuboidCollider aabox, ref SwiftList<Vector3d> outputIntersectionPoints) =>
         CheckAABBoxOverlaps(aabox.BoundsMin, aabox.BoundsMax, ref outputIntersectionPoints);

    /// <summary>
    /// Checks if an Axis-Aligned Bounding Box (AABB) overlaps with the ray. If there is an overlap, intersection points are calculated.
    /// </summary>
    public static bool CheckAABBoxOverlaps(Vector3d min, Vector3d max, ref SwiftList<Vector3d> outputIntersectionPoints)
    {
        Fixed64 minDistance = Fixed64.MAX_VALUE;
        bool intersects = false;

        FixedRange[] boxRanges = new FixedRange[]
        {
                new(min.x, max.x),
                new(min.y, max.y),
                new(min.z, max.z)
        };

        FixedRange lineRange = new(_axisMin, _axisMax);

        // Loop through each axis
        for (int i = 0; i < 3; i++)
        {
            // If ranges overlap on this axis, calculate intersection points
            if (FixedRange.ComputeOverlapDepth(boxRanges[i], lineRange) > Fixed64.Zero)
            {
                // Calculate intersection points
                Fixed64 t1 = (boxRanges[i].Min - _axisMin) / (_axisMax - _axisMin);
                Fixed64 t2 = (boxRanges[i].Max - _axisMin) / (_axisMax - _axisMin);

                Vector3d intersection1 = Vector3d.UnclampedLerp(_cachedOrigin, _cachedEnd, t1);
                Vector3d intersection2 = Vector3d.UnclampedLerp(_cachedOrigin, _cachedEnd, t2);

                // Calculate intersection distances
                Fixed64 distance1 = (intersection1 - _cachedOrigin).Magnitude;
                Fixed64 distance2 = (intersection2 - _cachedOrigin).Magnitude;

                // Add intersection points and distances if they're closer than previous intersections
                if (distance1 < minDistance)
                {
                    outputIntersectionPoints.Add(intersection1);
                    minDistance = distance1;
                    intersects = true;
                }

                if (distance2 < minDistance)
                {
                    outputIntersectionPoints.Add(intersection2);
                    minDistance = distance2;
                    intersects = true;
                }
            }
        }

        return intersects;
    }

    /// <summary>
    /// Checks if an Oriented Bounding Box (OBB) overlaps with the ray. If there is an overlap, intersection points are calculated.
    /// </summary>
    public static bool CheckOBBoxOverlaps(LSCuboidCollider oobox, ref SwiftList<Vector3d> outputIntersectionPoints)
    {
        // Check for intersection with AABB in local space
        if (!CheckAABBoxOverlaps(oobox.BoundsMin, oobox.BoundsMax, ref outputIntersectionPoints))
            return false;

        // Transform intersection points back to world space
        for (int i = 0; i < outputIntersectionPoints.Count; i++)
        {
            Vector3d worldSpaceIntersection = outputIntersectionPoints[i].Rotate(oobox.Position, oobox.Rotation);
            outputIntersectionPoints[i] = worldSpaceIntersection;
        }

        return true;
    }

    public static bool CheckMeshOverlaps(LSMeshCollider collider, ref SwiftList<Vector3d> outputIntersectionPoints)
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
                Vector3d intersectionPoint = _cachedOrigin + (_cacheAxisDirection * t);
                outputIntersectionPoints.Add(intersectionPoint);
                return true;
            }
        }

        return false;
    }

    // uses the Möller–Trumbore intersection algorithm (applicable to all types of meshes, including non-convex ones)
    public static bool RayTriangleIntersection(Vector3d[] triangle, out Fixed64 t)
    {
        t = Fixed64.Zero;

        Vector3d edge1Displacement = triangle[1] - triangle[0];
        Vector3d edge2Displacement = triangle[2] - triangle[1];

        // Compute the determinant
        Vector3d directionCrossEdge2 = Vector3d.Cross(_cacheAxisDirection, edge2Displacement);
        Fixed64 determinant = Vector3d.Dot(edge1Displacement, directionCrossEdge2);

        // If the determinant is near zero, the ray lies in the plane of the triangle
        if (determinant > -Fixed64.Epsilon && determinant < Fixed64.Epsilon)
            return false;

        Fixed64 invDeterminant = Fixed64.One / determinant;

        // Calculate the U parameter and test bounds
        Vector3d originDifference = _cachedOrigin - triangle[0];
        Fixed64 u = Vector3d.Dot(originDifference, directionCrossEdge2) * invDeterminant;

        if (u < Fixed64.Zero || u > Fixed64.One)
            return false;

        // Prepare to test the V parameter
        Vector3d originDifferenceCrossEdge1 = Vector3d.Cross(originDifference, edge1Displacement);

        // Calculate the V parameter and test bounds
        Fixed64 v = Vector3d.Dot(_cacheAxisDirection, originDifferenceCrossEdge1) * invDeterminant;

        if (v < Fixed64.Zero || u + v > Fixed64.One)
            return false;

        // The ray intersects the triangle, compute the t parameter
        t = Vector3d.Dot(edge2Displacement, originDifferenceCrossEdge1) * invDeterminant;

        return t > Fixed64.Epsilon;
    }
}
