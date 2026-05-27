namespace Gravitas.Colliders;

/// <summary>
/// Declares how a mesh collider should be treated by runtime collision policy.
/// </summary>
public enum MeshColliderMode : byte
{
    /// <summary>
    /// Mesh is intended to behave as a convex collision shape.
    /// </summary>
    Convex,

    /// <summary>
    /// Mesh is allowed to be concave and should be treated as triangle collision data.
    /// </summary>
    Concave
}
