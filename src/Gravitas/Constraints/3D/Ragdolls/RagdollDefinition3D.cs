//=======================================================================
// RagdollDefinition3D.cs
//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

namespace Gravitas.Constraints;

/// <summary>
/// Authored deterministic 3D ragdoll articulation definition.
/// </summary>
public sealed class RagdollDefinition3D
{
    /// <summary>
    /// Creates a ragdoll definition.
    /// </summary>
    public RagdollDefinition3D(
        RagdollLinkDefinition3D[] links,
        RagdollJointDefinition3D[] joints,
        RagdollSelfCollisionPolicy selfCollisionPolicy = RagdollSelfCollisionPolicy.SuppressAdjacentLinks)
    {
        Links = links;
        Joints = joints;
        SelfCollisionPolicy = selfCollisionPolicy;
    }

    /// <summary>
    /// Gets authored ragdoll links.
    /// </summary>
    public RagdollLinkDefinition3D[] Links { get; }

    /// <summary>
    /// Gets authored ragdoll joints.
    /// </summary>
    public RagdollJointDefinition3D[] Joints { get; }

    /// <summary>
    /// Gets ragdoll-internal collision filtering behavior.
    /// </summary>
    public RagdollSelfCollisionPolicy SelfCollisionPolicy { get; }
}
