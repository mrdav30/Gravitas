//=======================================================================
// ColliderShapeDefinition2DKind.cs
//=======================================================================
// MIT License, Copyright (c) 2026–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

namespace Gravitas.Colliders;

/// <summary>
/// Identifies data-only pure 2D collider shape definitions.
/// </summary>
public enum ColliderShapeDefinition2DKind : byte
{
    /// <summary>No authored shape.</summary>
    Undefined = 0,
    /// <summary>An authored circle.</summary>
    Circle = 1,
    /// <summary>An authored axis-aligned box.</summary>
    AABBox = 2,
    /// <summary>An authored convex polygon.</summary>
    ConvexPolygon = 3,
    /// <summary>An authored capsule.</summary>
    Capsule = 4
}
