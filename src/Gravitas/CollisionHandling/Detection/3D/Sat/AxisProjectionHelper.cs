//=======================================================================
// AxisProjectionHelper.cs
//=======================================================================
// MIT License, Copyright (c) 2026–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp;
using Gravitas.Colliders;
using SwiftCollections;
using System;

namespace Gravitas.CollisionHandling;

public static class AxisProjectionHelper
{
    #region Axis Vectors

    public static void GetCuboidAndCapsuleAxisVectors(
        LSCuboidCollider cuboid,
        LSCapsuleCollider capsule,
        ref SwiftList<Vector3d> output)
    {
        // for each face of the first polyhedron, add the normal of the face to the list of potential separating axes
        for (int i = 0; i < cuboid.FaceNormals.Length; i++)
            output.Add(cuboid.FaceNormals[i]);

        // for each edge of the polyhedron
        foreach (Vector3d edge1 in cuboid.EdgeDirections)
        {
            // cross product of the edge vector and the capsule direction
            Vector3d crossProduct = Vector3d.Cross(edge1, capsule.LineDirection);
            // Fixed-point cross products are deterministic; skip only an exactly zero result.
            if (crossProduct.MagnitudeSquared == Fixed64.Zero) continue;
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
        ReadOnlySpan<Vector3d> vertices,
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
