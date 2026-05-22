using Gravitas.Support;
using GridForge.Grids;

namespace Gravitas;

public interface IMatterAgent
{
    // This is a marker interface for now, but we can add common properties or methods for matter agents here in the future

    GridWorld World { get; } // The GridWorld instance that this agent belongs to, used for accessing the physics world and other agents

    FixedTransform Transform { get; } // World transform of the agent, used for positioning the collider and visual representation

    bool IsParent { get; } // i.e. in Unity `transform.GetComponentsInParent<LSCollider>().Length == 1`

    bool IsInteracting { get; } // Whether the agent is currently interacting with another agent, used for modifying rotation speed and other interaction-related behaviors
}