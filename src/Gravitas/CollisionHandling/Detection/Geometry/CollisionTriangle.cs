//=======================================================================
// CollisionTriangle.cs
//=======================================================================
// MIT License, Copyright (c) 2026–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp;
using FixedMathSharp.Bounds;
using SwiftCollections.Query;
using System.Runtime.CompilerServices;

namespace Gravitas.CollisionHandling;

/// <summary>
/// World-space triangle geometry plus Gravitas-owned cached collision data.
/// </summary>
internal readonly struct CollisionTriangle
{
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public CollisionTriangle(FixedTriangle triangle, Vector3d normal, FixedBoundVolume queryBounds)
    {
        Triangle = triangle;
        Normal = normal;
        QueryBounds = queryBounds;
    }

    public FixedTriangle Triangle { get; }

    public Vector3d Normal { get; }

    public FixedBoundVolume QueryBounds { get; }

    public Vector3d A
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        get => Triangle.A;
    }

    public Vector3d B
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        get => Triangle.B;
    }

    public Vector3d C
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        get => Triangle.C;
    }

    public Vector3d Center
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        get => Triangle.Centroid;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public Vector3d GetEdgeVector(int index) => Triangle.GetEdge(index).Delta;
}
