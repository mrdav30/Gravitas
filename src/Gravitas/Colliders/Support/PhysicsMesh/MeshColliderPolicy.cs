namespace Gravitas.Colliders;

/// <summary>
/// Centralizes alpha mesh-mode constraints so future decomposition support has one policy seam.
/// </summary>
internal static class MeshColliderPolicy
{
    public static bool RequiresConvexDecomposition(MeshColliderMode mode, StiffBody? body) =>
        mode == MeshColliderMode.Concave
        && body != null
        && !body.Immovable
        && !body.IsKinematic;
}
