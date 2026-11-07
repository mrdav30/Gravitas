//=======================================================================
// RagdollDefinition2D.cs
//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

namespace Gravitas.Constraints;

/// <summary>
/// Authored deterministic pure 2D ragdoll articulation definition.
/// </summary>
public sealed class RagdollDefinition2D
{
    /// <summary>
    /// Creates a pure 2D ragdoll definition.
    /// </summary>
    public RagdollDefinition2D(
        RagdollLinkDefinition2D[] links,
        RagdollJointDefinition2D[] joints,
        RagdollSelfCollisionPolicy selfCollisionPolicy = RagdollSelfCollisionPolicy.SuppressAdjacentLinks)
    {
        Links = links;
        Joints = joints;
        SelfCollisionPolicy = selfCollisionPolicy;
    }

    /// <summary>
    /// Gets authored ragdoll links.
    /// </summary>
    public RagdollLinkDefinition2D[] Links { get; }

    /// <summary>
    /// Gets authored ragdoll joints.
    /// </summary>
    public RagdollJointDefinition2D[] Joints { get; }

    /// <summary>
    /// Gets ragdoll-internal collision filtering behavior.
    /// </summary>
    public RagdollSelfCollisionPolicy SelfCollisionPolicy { get; }
}
