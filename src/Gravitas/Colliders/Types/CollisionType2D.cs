//=======================================================================
// CollisionType2D.cs
//=======================================================================
// MIT License, Copyright (c) 2026–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

namespace Gravitas.Colliders;

/// <summary>
/// Identifies the resolved pure 2D narrow-phase path for an ordered collider pair.
/// </summary>
public enum CollisionType2D : byte
{
    None,
    Circle_Circle,
    Circle_Convex,
    Convex_Circle,
    Convex_Convex,
    Capsule_Circle,
    Circle_Capsule,
    Capsule_Convex,
    Convex_Capsule,
    Capsule_Capsule,
    Compound
}
