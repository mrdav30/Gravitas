namespace Gravitas.Colliders;

/// <summary>
/// Describes the deterministic closed-volume validation result for a triangle mesh.
/// </summary>
public enum MeshVolumeValidationResult
{
    /// <summary>
    /// The mesh is one consistently wound closed volume.
    /// </summary>
    Valid = 0,

    /// <summary>
    /// At least one edge belongs to only one triangle.
    /// </summary>
    BoundaryEdge = 1,

    /// <summary>
    /// At least one edge belongs to more than two triangles.
    /// </summary>
    NonManifoldEdge = 2,

    /// <summary>
    /// Adjacent triangles do not use opposite directions for a shared edge.
    /// </summary>
    InconsistentWinding = 3,

    /// <summary>
    /// The mesh contains multiple disconnected closed shells.
    /// </summary>
    DisconnectedShell = 4,

    /// <summary>
    /// The mesh has no usable signed volume after topology validation.
    /// </summary>
    ZeroVolume = 5,

    /// <summary>
    /// The mesh repeats a triangle with the same three vertex indices.
    /// </summary>
    DuplicateTriangle = 6
}
