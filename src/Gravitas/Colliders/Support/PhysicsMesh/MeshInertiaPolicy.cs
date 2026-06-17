namespace Gravitas.Colliders;

/// <summary>
/// Selects how mesh inertia is derived when a mesh body participates in angular dynamics.
/// </summary>
public enum MeshInertiaPolicy
{
    /// <summary>
    /// Require a validated closed triangle volume and compute solid mass properties.
    /// </summary>
    RequireClosedVolume = 0,

    /// <summary>
    /// Use the legacy surface-area-weighted approximation for explicitly open or surface-only meshes.
    /// </summary>
    SurfaceApproximation = 1
}
