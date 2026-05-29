namespace Gravitas;

/// <summary>
/// Selects which dimensional runtime path a context advances.
/// </summary>
public enum PhysicsRuntimeMode : byte
{
    /// <summary>
    /// Advance only the pure 2D runtime path.
    /// </summary>
    TwoD = 2,

    /// <summary>
    /// Advance only the 3D runtime path.
    /// </summary>
    ThreeD = 3
}
