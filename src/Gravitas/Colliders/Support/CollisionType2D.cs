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
    Convex_Convex
}
