namespace Gravitas.Colliders;

/// <summary>
/// Identifies a data-only collider shape definition that can be materialized
/// into a runtime collider.
/// </summary>
public enum ColliderShapeDefinitionKind
{
    Undefined = 0,
    Sphere = 1,
    Capsule = 2,
    Cuboid = 3,
    Cylinder = 4,
    ConvexMesh = 5
}
