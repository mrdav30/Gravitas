//=======================================================================
// IConvexVertexSource2D.cs
//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp;
using FixedMathSharp.Geometry;

namespace Gravitas.Colliders;

/// <summary>
/// Exposes canonical scaled-local vertices and their center-relative rotation
/// for a 2D shape with one convex boundary.
/// </summary>
internal interface IConvexVertexSource2D
{
    int VertexCount { get; }

    Fixed64 Rotation { get; }

    Vector2d GetScaledLocalVertexUnchecked(int index);

    FixedPointAnchor2d GetSupportAnchor(Vector2d direction);
}
