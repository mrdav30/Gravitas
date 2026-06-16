using FixedMathSharp;
using Gravitas.Colliders;
using SwiftCollections;

namespace Gravitas.CollisionHandling;

public static class AxisProjectionHelper
{
    #region Axis Vectors

    public static readonly Fixed64 AngleThresholdDegrees = new(2); // 2 degrees
    public static readonly Fixed64 AngleThresholdRadians = FixedMath.DegToRad(AngleThresholdDegrees);
    public static readonly Fixed64 CosThreshold = FixedMath.Cos(AngleThresholdRadians);

    public static void GetCuboidAndCapsuleAxisVectors(
        LSCuboidCollider cuboid,
        LSCapsuleCollider capsule,
        ref SwiftHashSet<Vector3d> output)
    {
        // for each face of the first polyhedron, add the normal of the face to the list of potential separating axes
        for (int i = 0; i < cuboid.FaceNormals.Length; i++)
            output.Add(cuboid.FaceNormals[i]);

        // for each edge of the polyhedron
        foreach (Vector3d edge1 in cuboid.EdgeDirections)
        {
            if (Vector3d.AreAlmostParallel(edge1, capsule.LineDirection, CosThreshold))
            {
                output.Add(edge1);
                continue;
            }

            // cross product of the edge vector and the capsule direction
            Vector3d crossProduct = Vector3d.Cross(edge1, capsule.LineDirection);
            // crossProduct can be zero if the vectors are parallel - we check against a small number to account for floating point error
            // otherwise add the normalized cross product to the list of potential separating axes
            if (crossProduct.MagnitudeSquared <= Fixed64.Epsilon) continue;
            output.Add(crossProduct.Normalized);
        }
    }

    #endregion

    #region Projection

    /// <summary>
    /// Project all points onto axis vector and get distance from beginning of the axis.
    /// If done for all points in object, and minimum and maximum value recorded,
    /// the result is a set of two numbers that define from where to where projection goes.
    /// </summary>
    /// <param name="axisVector">The axis vector to project onto.</param>
    /// <param name="vertices">The vertices of the polygon.</param>
    /// <param name="projection">The resulting projection range.</param>
    public static void ProjectPolygonOntoAxis(
        Vector3d axisVector,
        Vector3d[] vertices,
        ref FixedRange projection)
    {
        Fixed64 min = Vector3d.Dot(axisVector, vertices[0]);
        Fixed64 max = min;
        Fixed64 projectionSize;

        for (int i = 0; i < vertices.Length; i++)
        {
            projectionSize = Vector3d.Dot(axisVector, vertices[i]); // this scalar value is equal to length of vector B projected onto vector A
            if (projectionSize < min) min = projectionSize;
            if (projectionSize > max) max = projectionSize;
        }

        projection.SetMinMax(min, max);
    }

    /// <summary>
    /// Project all points onto axis vector and get distance from beginning of the axis.
    /// If done for all points in object, and minimum and maximum value recorded,
    /// the result is a set of two numbers that define from where to where projection goes.
    /// </summary>
    /// <param name="axisVector">The axis vector to project onto.</param>
    /// <param name="vertices">The vertices of the polygon.</param>
    /// <param name="projection">The resulting projection range.</param>
    public static void ProjectPolygonOntoAxis(
        Vector3d axisVector,
        SwiftList<Vector3d> vertices,
        ref FixedRange projection)
    {
        Fixed64 min = Vector3d.Dot(axisVector, vertices[0]);
        Fixed64 max = min;
        Fixed64 projectionSize;

        for (int i = 0; i < vertices.Count; i++)
        {
            projectionSize = Vector3d.Dot(axisVector, vertices[i]); // this scalar value is equal to length of vector B projected onto vector A
            if (projectionSize < min) min = projectionSize;
            if (projectionSize > max) max = projectionSize;
        }

        projection.SetMinMax(min, max);
    }

    /// <summary>
    /// Project position onto axis vector and add/subtract diameter to get center projection
    /// </summary>
    /// <param name="axisVector"></param>
    /// <param name="position">Center position of circle</param>
    /// <param name="radius">Diameter of circle (2*radius)</param>
    /// <returns></returns>
    public static FixedRange ProjectSphereOntoAxis(Vector3d axisVector, Vector3d position, Fixed64 radius)
    {
        Fixed64 centerProjection = axisVector.Dot(position.X, position.Y, position.Z);
        return new FixedRange(centerProjection - radius, centerProjection + radius);
    }

    public static FixedRange ProjectCapsuleOntoAxis(Vector3d axisVector, Vector3d startPoint, Vector3d endPoint, Fixed64 radius)
    {
        // Project spherical ends
        Fixed64 startProjection = axisVector.Dot(startPoint.X, startPoint.Y, startPoint.Z);
        Fixed64 endProjection = axisVector.Dot(endPoint.X, endPoint.Y, endPoint.Z);

        // Compute the capsule's cylindrical projection range on the axis
        // Cylindrical projection is merely the projections of the end points on the axis.
        Fixed64 minCylinder = FixedMath.Min(startProjection, endProjection);
        Fixed64 maxCylinder = FixedMath.Max(startProjection, endProjection);

        // Since we consider the capsule as a cylinder capped with two hemispheres, 
        // the min and max projections will be the min and max of the cylindrical projection +/- the radius
        Fixed64 min = minCylinder - radius;
        Fixed64 max = maxCylinder + radius;

        return new FixedRange(min, max);
    }

    public static FixedRange ProjectCylinderOntoAxis(
        Vector3d axisVector,
        Vector3d startPoint,
        Vector3d endPoint,
        Vector3d cylinderAxis,
        Fixed64 radius)
    {
        Fixed64 startProjection = axisVector.Dot(startPoint.X, startPoint.Y, startPoint.Z);
        Fixed64 endProjection = axisVector.Dot(endPoint.X, endPoint.Y, endPoint.Z);
        Fixed64 minSegment = FixedMath.Min(startProjection, endProjection);
        Fixed64 maxSegment = FixedMath.Max(startProjection, endProjection);

        Fixed64 axialAlignment = Vector3d.Dot(axisVector, cylinderAxis);
        Fixed64 radialProjectionSqr = Fixed64.One - axialAlignment * axialAlignment;
        Fixed64 radialProjection = radialProjectionSqr <= Fixed64.Zero
            ? Fixed64.Zero
            : FixedMath.Sqrt(radialProjectionSqr);
        Fixed64 radiusOffset = radius * radialProjection;

        return new FixedRange(minSegment - radiusOffset, maxSegment + radiusOffset);
    }

    #endregion
}
