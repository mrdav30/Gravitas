namespace Gravitas;

/// <summary>
/// Identifies the simulation dimensionality owned by a body or collider.
/// </summary>
public enum PhysicsDimension : byte
{
    /// <summary>
    /// No simulation dimension has been selected.
    /// </summary>
    None = 0,

    /// <summary>
    /// First-class 2D simulation using deterministic <see cref="FixedMathSharp.Vector2d"/> coordinates.
    /// </summary>
    TwoD = 2,

    /// <summary>
    /// First-class 3D simulation using deterministic <see cref="FixedMathSharp.Vector3d"/> coordinates.
    /// </summary>
    ThreeD = 3
}
