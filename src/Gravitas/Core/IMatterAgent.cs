using FixedMathSharp;

namespace Gravitas;

/// <summary>
/// Defines the host-owned object boundary Gravitas needs to bind matter into a simulation context.
/// </summary>
public interface IMatterAgent
{
    /// <summary>
    /// Gets the explicit Gravitas runtime context this agent belongs to.
    /// </summary>
    GravitasWorldContext Context { get; }

    /// <summary>
    /// Gets the world transform used for collider and visual positioning.
    /// </summary>
    FixedTransform Transform { get; }

    /// <summary>
    /// Gets whether this agent is the top-level collider owner in its host hierarchy.
    /// </summary>
    bool IsParent { get; }

    /// <summary>
    /// Gets whether the agent is currently interacting with another agent.
    /// </summary>
    bool IsInteracting { get; }
}
