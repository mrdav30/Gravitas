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
        FixedPointAnchor contactAnchor,
        Vector3d outwardNormal)
    {
        Distance = distance;
        Offset = offset;
        ContactAnchor = contactAnchor;
        OutwardNormal = outwardNormal;
    }

    internal Fixed64 Distance { get; }

    internal Vector2d Offset { get; }

    internal FixedPointAnchor ContactAnchor { get; }

    internal Vector3d OutwardNormal { get; }
}
