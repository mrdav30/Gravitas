//=======================================================================
// RagdollJointDefinition3D.cs
//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp;

namespace Gravitas.Constraints;

/// <summary>
/// Authored joint entry for a deterministic 3D ragdoll articulation.
/// </summary>
public readonly struct RagdollJointDefinition3D
{
    /// <summary>
    /// Creates a ragdoll joint definition with unrestricted limits, no motor, and adjacent self-collision suppression.
    /// </summary>
    public RagdollJointDefinition3D(
        int linkAId,
        int linkBId,
        JointType3D type,
        FixedTransform localFrameA,
        FixedTransform localFrameB)
        : this(
            linkAId,
            linkBId,
            type,
            localFrameA,
            localFrameB,
            JointLimit3D.Unrestricted,
            JointMotor3D.Disabled,
            JointCollisionPolicy.SuppressLinked)
    {
    }

    /// <summary>
    /// Creates a ragdoll joint definition.
    /// </summary>
    public RagdollJointDefinition3D(
        int linkAId,
        int linkBId,
        JointType3D type,
        FixedTransform localFrameA,
        FixedTransform localFrameB,
        JointLimit3D limits,
        JointMotor3D motor,
        JointCollisionPolicy collisionPolicy)
    {
        LinkAId = linkAId;
        LinkBId = linkBId;
        Type = type;
        LocalFrameA = localFrameA;
        LocalFrameB = localFrameB;
        Limits = limits;
        Motor = motor;
        CollisionPolicy = collisionPolicy;
    }

    /// <summary>
    /// Gets the first authored link ID.
    /// </summary>
    public int LinkAId { get; }

    /// <summary>
    /// Gets the second authored link ID.
    /// </summary>
    public int LinkBId { get; }

    /// <summary>
    /// Gets the joint type.
    /// </summary>
    public JointType3D Type { get; }

    /// <summary>
    /// Gets the local frame for the first link.
    /// </summary>
    public FixedTransform LocalFrameA { get; }

    /// <summary>
    /// Gets the local frame for the second link.
    /// </summary>
    public FixedTransform LocalFrameB { get; }

    /// <summary>
    /// Gets optional angular limit data.
    /// </summary>
    public JointLimit3D Limits { get; }

    /// <summary>
    /// Gets optional angular motor data.
    /// </summary>
    public JointMotor3D Motor { get; }

    /// <summary>
    /// Gets physical collision filtering behavior for the linked colliders.
    /// </summary>
    public JointCollisionPolicy CollisionPolicy { get; }
}
