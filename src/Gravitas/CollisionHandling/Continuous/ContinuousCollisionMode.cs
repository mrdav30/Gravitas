namespace Gravitas;

/// <summary>
/// Selects how a body guards its frame movement against tunneling.
/// </summary>
public enum ContinuousCollisionMode : byte
{
    /// <summary>
    /// Uses the owning context's default continuous-collision mode.
    /// </summary>
    Inherit = 0,

    /// <summary>
    /// Uses the existing discrete integration path without a movement sweep.
    /// </summary>
    Discrete = 1,

    /// <summary>
    /// Sweeps the body through its intended frame displacement before committing position.
    /// </summary>
    Continuous = 2,

    /// <summary>
    /// Sweeps only when the intended frame displacement is larger than the body proxy radius.
    /// </summary>
    Auto = 3
}
