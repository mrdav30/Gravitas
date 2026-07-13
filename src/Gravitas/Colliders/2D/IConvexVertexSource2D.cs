//=======================================================================
// IConvexVertexSource2D.cs
//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp;

namespace Gravitas.Colliders;

/// <summary>
/// Exposes deterministic world vertices only for 2D shapes with one convex boundary.
/// </summary>
internal interface IConvexVertexSource2D
{
    int VertexCount { get; }

    Vector2d GetVertexUnchecked(int index);
}
