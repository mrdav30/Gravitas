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
    /// <summary>No collider shape.</summary>
    None = 0,
    /// <summary>A circle.</summary>
    Circle = 1,
    /// <summary>An axis-aligned box.</summary>
    AABox = 2,
    /// <summary>A convex polygon.</summary>
    ConvexPolygon = 3,
    /// <summary>A compound collider.</summary>
    Compound = 4,
    /// <summary>A capsule.</summary>
    Capsule = 5
}
