//=======================================================================
// LSCuboidCollider.cs
//=======================================================================
// MIT License, Copyright (c) 2026–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp;
using FixedMathSharp.Bounds;
using Gravitas.Queries;
using SwiftCollections;
using System;
using System.Runtime.CompilerServices;

namespace Gravitas.Colliders;

public class LSCuboidCollider : LSCollider
{
    // Define faces with vertex indices
    private static readonly int[][] FaceDefinitions = new int[][]
    {
            new[] {0, 1, 3, 2}, // near quad
            new[] {4, 6, 7, 5}, // far quad
            new[] {2, 3, 7, 6}, // top quad
            new[] {4, 5, 1, 0}, // bottom quad
            new[] {0, 2, 6, 4}, // left quad
            new[] {5, 7, 3, 1}  // right quad
    };

    // Define edges with vertex indices
    internal static readonly int[][] EdgeDefinitions = new int[][]
    {
            new[] {0, 1}, new[] {2, 3}, new[] {0, 2}, new[] {1, 3},  // near quad
            new[] {4, 5}, new[] {6, 7}, new[] {4, 6}, new[] {5, 7},  // far quad
            new[] {0, 4}, new[] {1, 5}, new[] {2, 6}, new[] {3, 7}   // lines connecting the quads
    };

    private Vector3d[] _vertices = null!;
    /// <summary>
    /// The vertices of the cuboid.
    /// </summary>
    internal Vector3d[] Vertices => _vertices;

    protected Vector3d[] _faceNormals = null!;
    /// <summary>
    /// The normal vectors for each face of the cuboid.
    /// </summary>
    internal Vector3d[] FaceNormals => _faceNormals;

    protected Vector3d[] _edgeDirections = null!;
    /// <summary>
    /// A collection of normalized edge displacement vectors.
    /// Note, the displacement is a vector that points from the start of the edge (line segment) to the end.
    /// </summary>
    internal Vector3d[] EdgeDirections => _edgeDirections;

    /// <summary>
    /// Stores the length of the cuboid along the X axis.
    /// </summary>
    protected Fixed64 _xAxisLength;

    /// <summary>
    /// Stores the length of the cuboid along the Y axis.
    /// </summary>
    protected Fixed64 _yAxisLength;

    /// <summary>
    /// Stores the length of the cuboid along the Z axis.
    /// </summary>
    protected Fixed64 _zAxisLength;

    /// <summary>
    /// Stores the direction vector along the X axis of the cuboid.
    /// </summary>
    protected Vector3d _xAxisDirectionVector;

    /// <summary>
    /// Stores the direction vector along the Y axis of the cuboid.
    /// </summary>
    protected Vector3d _yAxisDirectionVector;

    /// <summary>
    /// Stores the direction vector along the Z axis of the cuboid.
    /// </summary>
    protected Vector3d _zAxisDirectionVector;

    public override ColliderType Shape => Rotation == FixedQuaternion.Identity ? ColliderType.AABox : ColliderType.OBBox;

    public override int Priority => ColliderSettings.GetPriority(Shape);

    public override Fixed64 ScaledRadius => ScaledSize.Magnitude * Fixed64.Half;

    public LSCuboidCollider()
    {
        _vertices = new Vector3d[8];
        _faceNormals = new Vector3d[FaceDefinitions.Length];
        _edgeDirections = new Vector3d[EdgeDefinitions.Length];
    }

    public LSCuboidCollider(ColliderShapeDefinition definition)
        : this()
    {
        definition.EnsureKind(ColliderShapeDefinitionKind.Cuboid);
        Material = definition.Material;
        Size = definition.Size;
    }

    protected override void BuildShape()
    {
        GenerateVertices();
        GenerateAxes();
        CalculateFaceNormals();
        CalculateEdgeDirections();
        GenerateArea();
    }

    protected void GenerateVertices()
    {
        if (Shape == ColliderType.OBBox)
        {
            FixedBoundBox orientedBounds = FixedBoundBox.FromCenterAndSize(Center, ScaledSize);

            for (int i = 0; i < _vertices.Length; i++)
                _vertices[i] = orientedBounds.GetCorner(i).Rotate(Center, Rotation);

            return;
        }

        Span<Vector3d> vertices = stackalloc Vector3d[FixedBoundBox.CornerCount];
        _bounds.CopyCorners(vertices);
        for (int i = 0; i < vertices.Length; i++)
            _vertices[i] = vertices[i];
    }

    protected void GenerateAxes()
    {
        _xAxisLength = (Vertices[1] - Vertices[0]).Magnitude;
        _yAxisLength = (Vertices[2] - Vertices[0]).Magnitude;
        _zAxisLength = (Vertices[4] - Vertices[0]).Magnitude;

        _xAxisDirectionVector = (Vertices[1] - Vertices[0]) / _xAxisLength;
        _yAxisDirectionVector = (Vertices[2] - Vertices[0]) / _yAxisLength;
        _zAxisDirectionVector = (Vertices[4] - Vertices[0]) / _zAxisLength;
    }

    private void CalculateFaceNormals()
    {
        for (int i = 0; i < FaceDefinitions.Length; i++)
        {
            int[] faceVertices = FaceDefinitions[i];
            Vector3d first = Vertices[faceVertices[0]];

            // Get two non-parallel edges
            Vector3d edge1 = Vertices[faceVertices[1]] - first;
            Vector3d edge2 = Vertices[faceVertices[2]] - first;

            // Calculate the normal using cross product
            _faceNormals[i] = Vector3d.Cross(edge1, edge2).Normalized;
        }
    }

    private void CalculateEdgeDirections()
    {
        for (int i = 0; i < _edgeDirections.Length; i++)
        {
            int[] edge = EdgeDefinitions[i];
            _edgeDirections[i] = (Vertices[edge[1]] - Vertices[edge[0]]).Normalized;
        }
    }

    protected virtual void GenerateArea() =>
        // Area calculation: A = 2lw + 2lh + 2wh
        Area = 2 * ScaledSize.X * ScaledSize.Z + 2 * ScaledSize.X * ScaledSize.Y + 2 * ScaledSize.Y * ScaledSize.Z;

    public override Fixed3x3 CalculateInertiaTensor(Fixed64 mass, Vector3d localCenterOfMassOffset)
    {
        Vector3d worldScaleSqr = ScaledSize * ScaledSize;

        // For a solid box, the inertia tensor is (m/12)*(h^2 + d^2), (m/12)*(w^2 + d^2), (m/12)*(w^2 + h^2) for the diagonal elements
        Fixed64 xx = (mass / (Fixed64)12) * (worldScaleSqr.Y + worldScaleSqr.Z);
        Fixed64 yy = (mass / (Fixed64)12) * (worldScaleSqr.X + worldScaleSqr.Z);
        Fixed64 zz = (mass / (Fixed64)12) * (worldScaleSqr.X + worldScaleSqr.Y);

        Fixed3x3 tensor = new(
            xx, Fixed64.Zero, Fixed64.Zero,
            Fixed64.Zero, yy, Fixed64.Zero,
            Fixed64.Zero, Fixed64.Zero, zz
        );
        return ShiftInertiaTensorFromLocalCenterOfMass(tensor, mass, localCenterOfMassOffset);
    }

    public override Fixed64 GetFrontalArea(Vector3d direction)
    {
        Fixed64 directionMagnitude = direction.Magnitude;
        if (directionMagnitude <= Fixed64.Epsilon)
            return Area;

        direction /= directionMagnitude;

        Fixed64 dotX = Vector3d.Dot(direction, _xAxisDirectionVector).Abs();
        Fixed64 dotY = Vector3d.Dot(direction, _yAxisDirectionVector).Abs();
        Fixed64 dotZ = Vector3d.Dot(direction, _zAxisDirectionVector).Abs();

        // The orthographic projection of a box is the sum of each face area's
        // contribution along the view direction.
        return _yAxisLength * _zAxisLength * dotX
            + _xAxisLength * _zAxisLength * dotY
            + _xAxisLength * _yAxisLength * dotZ;
    }

    public override Vector3d ClosestPointOnSurface(Vector3d other)
    {
        if (Shape == ColliderType.AABox)
            return Bounds.ClosestPointOnSurface(other);

        Vector3d axisX = Rotation.Rotate(Vector3d.Right);
        Vector3d axisY = Rotation.Rotate(Vector3d.Up);
        Vector3d axisZ = Rotation.Rotate(Vector3d.Forward);
        Vector3d halfExtents = ScaledSize * Fixed64.Half;
        Vector3d delta = other - Center;
        Fixed64 x = FixedMath.Clamp(Vector3d.Dot(delta, axisX), -halfExtents.X, halfExtents.X);
        Fixed64 y = FixedMath.Clamp(Vector3d.Dot(delta, axisY), -halfExtents.Y, halfExtents.Y);
        Fixed64 z = FixedMath.Clamp(Vector3d.Dot(delta, axisZ), -halfExtents.Z, halfExtents.Z);

        if (IsPointInsideOrientedBounds(delta, x, y, z, axisX, axisY, axisZ))
            SnapInsidePointToNearestFace(ref x, ref y, ref z, halfExtents.X, halfExtents.Y, halfExtents.Z);

        return Center
            + axisX * x
            + axisY * y
            + axisZ * z;
    }

    private static bool IsPointInsideOrientedBounds(
        Vector3d delta,
        Fixed64 x,
        Fixed64 y,
        Fixed64 z,
        Vector3d axisX,
        Vector3d axisY,
        Vector3d axisZ)
    {
        Vector3d closestDelta =
            axisX * x
            + axisY * y
            + axisZ * z;

        return (closestDelta - delta).MagnitudeSquared <= Fixed64.Epsilon;
    }

    private static void SnapInsidePointToNearestFace(
        ref Fixed64 x,
        ref Fixed64 y,
        ref Fixed64 z,
        Fixed64 halfX,
        Fixed64 halfY,
        Fixed64 halfZ)
    {
        Fixed64 distanceX = halfX - x.Abs();
        Fixed64 distanceY = halfY - y.Abs();
        Fixed64 distanceZ = halfZ - z.Abs();

        if (distanceX <= distanceY && distanceX <= distanceZ)
        {
            x = SignedHalfExtent(x, halfX);
            return;
        }

        if (distanceY <= distanceZ)
        {
            y = SignedHalfExtent(y, halfY);
            return;
        }

        z = SignedHalfExtent(z, halfZ);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static Fixed64 SignedHalfExtent(Fixed64 coordinate, Fixed64 halfExtent) =>
        coordinate < Fixed64.Zero ? -halfExtent : halfExtent;

    public override Vector3d GetNormalAtPoint(Vector3d point)
    {
        Vector3d axisX = Shape == ColliderType.AABox ? Vector3d.Right : Rotation.Rotate(Vector3d.Right);
        Vector3d axisY = Shape == ColliderType.AABox ? Vector3d.Up : Rotation.Rotate(Vector3d.Up);
        Vector3d axisZ = Shape == ColliderType.AABox ? Vector3d.Forward : Rotation.Rotate(Vector3d.Forward);
        Vector3d halfExtents = ScaledSize * Fixed64.Half;
        Vector3d delta = point - Center;
        Fixed64 x = Vector3d.Dot(delta, axisX);
        Fixed64 y = Vector3d.Dot(delta, axisY);
        Fixed64 z = Vector3d.Dot(delta, axisZ);

        Fixed64 distanceX = halfExtents.X - x.Abs();
        Fixed64 distanceY = halfExtents.Y - y.Abs();
        Fixed64 distanceZ = halfExtents.Z - z.Abs();

        if (distanceX <= distanceY && distanceX <= distanceZ)
            return axisX * SignedUnit(x);

        if (distanceY <= distanceZ)
            return axisY * SignedUnit(y);

        return axisZ * SignedUnit(z);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static Fixed64 SignedUnit(Fixed64 coordinate) =>
        coordinate < Fixed64.Zero ? -Fixed64.One : Fixed64.One;

    public override bool ColliderOverlapsRay(RaycastSegmentWorker worker, ref SwiftList<Vector3d> outputIntersectionPoints)
    {
        if (Shape == ColliderType.AABox)
            return worker.CheckAABBoxOverlaps(this, ref outputIntersectionPoints);

        return worker.CheckOBBoxOverlaps(this, ref outputIntersectionPoints);
    }
}
