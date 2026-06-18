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
