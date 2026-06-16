using FixedMathSharp;

namespace Gravitas.Colliders;

public static class MeshUtils
{
    /// <summary>
    /// Finds the closest point on this triangle to the given point.
    /// 
    /// The method first calculates the closest point on the infinite plane defined by this triangle.
    /// This is done by projecting the given point onto the plane along the direction of the triangle's normal.
    ///
    /// It then checks whether this point on the plane is inside the triangle. 
    /// If it is, the point on the plane is indeed the closest point on the triangle and is returned. 
    /// 
    /// If the point on the plane is outside the triangle, it means the closest point to the given point lies somewhere on the edges of the triangle. 
    /// In this case, the method invokes the ClosestPointOnTriangleEdges method to calculate the closest point on each of the triangle's edges, 
    /// and then compares these distances to return the closest one.
    ///
    /// This method uses the method of separating axes to check whether the point on the plane is inside the triangle.
    /// </summary>
    public static Vector3d ClosestPointOnTriangle(Vector3d[] triangle, Vector3d normal, Vector3d point)
    {
        return ClosestPointOnTriangle(triangle[0], triangle[1], triangle[2], normal, point);
    }

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

    public static bool IsPointInTrianglePlane(Vector3d[] triangle, Vector3d normal, Vector3d point) =>
        IsPointInTrianglePlane(triangle[0], triangle[1], triangle[2], normal, point);

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

    public static Vector3d ClosestPointOnEdge(Vector3d start, Vector3d end, Vector3d point)
    {
        Vector3d displacement = end - start;
        // get the projection of the vector from the start of the line to the point onto the line.
        // since the line is not a unit vector, scale this projection by the square of the length of the line.
        Fixed64 t = Vector3d.Dot(point - start, displacement) / displacement.MagnitudeSquared;
        if (t < Fixed64.Zero)
            return start; // The point is closer to the start of the line

        if (t > Fixed64.One)
            return end; // The point is closer to the end of the line

        return start + t * displacement; // The point is on the line segment
    }
}
