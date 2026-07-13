//=======================================================================
// MeshUtils.cs
//=======================================================================
// MIT License, Copyright (c) 2026–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp;
using FixedMathSharp.Bounds;
using System.Runtime.CompilerServices;

namespace Gravitas.Colliders;

public static class MeshUtils
{
    /// <summary>
    /// Finds the closest point on a triangle to the supplied point. The normal
    /// must match the winding of <paramref name="first"/>, <paramref name="second"/>,
    /// and <paramref name="third"/>.
    /// </summary>
    public static Vector3d ClosestPointOnTriangle(
        Vector3d first,
        Vector3d second,
        Vector3d third,
        Vector3d normal,
        Vector3d point)
    {
        Vector3d pointOnPlane = point - normal * Vector3d.Dot(point - first, normal);
        if (IsPointInTrianglePlane(first, second, third, normal, pointOnPlane))
            return pointOnPlane;

        return ClosestPointOnTriangleEdges(first, second, third, point);
    }

    /// <summary>
    /// Returns whether a coplanar point lies within or on the triangle boundary.
    /// The normal must match the triangle's vertex winding.
    /// </summary>
    public static bool IsPointInTrianglePlane(
        Vector3d first,
        Vector3d second,
        Vector3d third,
        Vector3d normal,
        Vector3d point)
    {
        // Check if the point is inside the triangle by checking if the point is to the left of each edge
        if (Vector3d.Dot(Vector3d.Cross((second - first).Normalized, point - first), normal) < Fixed64.Zero)
            return false;

        if (Vector3d.Dot(Vector3d.Cross((third - second).Normalized, point - second), normal) < Fixed64.Zero)
            return false;

        if (Vector3d.Dot(Vector3d.Cross((first - third).Normalized, point - third), normal) < Fixed64.Zero)
            return false;

        return true;
    }

    private static Vector3d ClosestPointOnTriangleEdges(
        Vector3d first,
        Vector3d second,
        Vector3d third,
        Vector3d point)
    {
        Vector3d closestPoint = ClosestPointOnEdge(first, second, point);
        Fixed64 minDistanceSquared = Vector3d.DistanceSquared(point, closestPoint);

        Vector3d pointOnEdge = ClosestPointOnEdge(second, third, point);
        Fixed64 distanceSquared = Vector3d.DistanceSquared(point, pointOnEdge);
        if (distanceSquared < minDistanceSquared)
        {
            minDistanceSquared = distanceSquared;
            closestPoint = pointOnEdge;
        }

        pointOnEdge = ClosestPointOnEdge(third, first, point);
        distanceSquared = Vector3d.DistanceSquared(point, pointOnEdge);
        if (distanceSquared < minDistanceSquared)
            closestPoint = pointOnEdge;

        return closestPoint;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector3d ClosestPointOnEdge(Vector3d start, Vector3d end, Vector3d point) =>
        new FixedSegment(start, end).ClosestPoint(point);
}
