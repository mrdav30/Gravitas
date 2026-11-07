//=======================================================================
// GjkResult.cs
//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp;
using FixedMathSharp.Geometry;

namespace Gravitas.Queries;

/// <summary>
/// Carries GJK intersection or separation evidence, including witnesses on both shapes.
/// </summary>
internal readonly struct GjkResult
{
    public GjkResult(
        bool intersects,
        Fixed64 distance,
        FixedPointAnchor pointA,
        FixedPointAnchor pointB,
        Vector3d normal)
    {
        Intersects = intersects;
        Distance = distance;
        PointA = pointA;
        PointB = pointB;
        Normal = normal;
    }

    public bool Intersects { get; }

    public Fixed64 Distance { get; }

    public FixedPointAnchor PointA { get; }

    public FixedPointAnchor PointB { get; }

    public Vector3d Normal { get; }

    public static GjkResult CreateIntersection(
        FixedPointAnchor pointA,
        FixedPointAnchor pointB) =>
        new(true, Fixed64.Zero, pointA, pointB, Vector3d.Zero);
}
