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
    Compound = 4
}
