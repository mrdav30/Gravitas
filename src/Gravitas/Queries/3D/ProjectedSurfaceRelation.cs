//=======================================================================
// ProjectedSurfaceRelation.cs
//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp;
using FixedMathSharp.Geometry;

namespace Gravitas.Queries;

/// <summary>
/// Retains one admitted X/Z projected-surface relation and its independent
/// real 3D surface witness.
/// </summary>
internal readonly struct ProjectedSurfaceRelation
{
    internal ProjectedSurfaceRelation(
        Fixed64 distance,
        Vector2d offset,
        Vector2d direction,
        FixedPointAnchor contactAnchor,
        Vector3d outwardNormal,
        bool isContained)
    {
        Distance = distance;
        Offset = offset;
        Direction = direction;
        ContactAnchor = contactAnchor;
        OutwardNormal = outwardNormal;
        IsContained = isContained;
    }

    internal Fixed64 Distance { get; }

    internal Vector2d Offset { get; }

    internal Vector2d Direction { get; }

    internal FixedPointAnchor ContactAnchor { get; }

    internal Vector3d OutwardNormal { get; }

    internal bool IsContained { get; }
}
