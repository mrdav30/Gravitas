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
    Undefined = 0,
    Circle = 1,
    AABBox = 2,
    ConvexPolygon = 3
}
