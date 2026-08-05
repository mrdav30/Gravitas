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
    /// <summary>No supported collision path.</summary>
    None,
    /// <summary>Circle against circle.</summary>
    Circle_Circle,
    /// <summary>Circle against a convex shape.</summary>
    Circle_Convex,
    /// <summary>Convex shape against circle.</summary>
    Convex_Circle,
    /// <summary>Convex shape against convex shape.</summary>
    Convex_Convex,
    /// <summary>Capsule against circle.</summary>
    Capsule_Circle,
    /// <summary>Circle against capsule.</summary>
    Circle_Capsule,
    /// <summary>Capsule against a convex shape.</summary>
    Capsule_Convex,
    /// <summary>Convex shape against capsule.</summary>
    Convex_Capsule,
    /// <summary>Capsule against capsule.</summary>
    Capsule_Capsule,
    /// <summary>A pair involving at least one compound collider.</summary>
    Compound
}
