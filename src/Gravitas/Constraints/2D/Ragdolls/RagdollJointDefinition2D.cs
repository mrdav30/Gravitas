//=======================================================================
// RagdollJointDefinition2D.cs
//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

namespace Gravitas.Constraints;

/// <summary>
/// Authored joint entry for a deterministic pure 2D ragdoll articulation.
/// </summary>
public readonly struct RagdollJointDefinition2D
{
    /// <summary>
    /// Creates a ragdoll joint definition with unrestricted limits, no motor, and adjacent self-collision suppression.
    /// </summary>
    public RagdollJointDefinition2D(
        int linkAId,
        int linkBId,
        JointType2D type,
        JointFrame2D localFrameA,
        JointFrame2D localFrameB)
        : this(
            linkAId,
            linkBId,
            type,
            localFrameA,
            localFrameB,
            JointLimit2D.Unrestricted,
            JointMotor2D.Disabled,
            JointCollisionPolicy.SuppressLinked)
    {
    }

    /// <summary>
    /// Creates a pure 2D ragdoll joint definition.
    /// </summary>
    public RagdollJointDefinition2D(
        int linkAId,
        int linkBId,
        JointType2D type,
        JointFrame2D localFrameA,
        JointFrame2D localFrameB,
        JointLimit2D limits,
        JointMotor2D motor,
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
    public JointType2D Type { get; }

    /// <summary>
    /// Gets the local frame for the first link.
    /// </summary>
    public JointFrame2D LocalFrameA { get; }

    /// <summary>
    /// Gets the local frame for the second link.
    /// </summary>
    public JointFrame2D LocalFrameB { get; }

    /// <summary>
    /// Gets optional scalar limit data.
    /// </summary>
    public JointLimit2D Limits { get; }

    /// <summary>
    /// Gets optional scalar motor data.
    /// </summary>
    public JointMotor2D Motor { get; }

    /// <summary>
    /// Gets physical collision filtering behavior for the linked colliders.
    /// </summary>
    public JointCollisionPolicy CollisionPolicy { get; }
}
