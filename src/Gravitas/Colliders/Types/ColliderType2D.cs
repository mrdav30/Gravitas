//=======================================================================
// ColliderType2D.cs
//=======================================================================
// MIT License, Copyright (c) 2026–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

namespace Gravitas.Colliders;

/// <summary>
/// Identifies first-class pure 2D collider shapes.
/// </summary>
public enum ColliderType2D : byte
{
    None = 0,
    Circle = 1,
    AABox = 2,
    ConvexPolygon = 3,
    Compound = 4,
    Capsule = 5
}
