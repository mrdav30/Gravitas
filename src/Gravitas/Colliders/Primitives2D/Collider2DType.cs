namespace Gravitas.Colliders;

/// <summary>
/// Identifies first-class pure 2D collider shapes.
/// </summary>
public enum Collider2DType : byte
{
    None = 0,
    Circle = 1,
    AABox = 2,
    ConvexPolygon = 3
}
