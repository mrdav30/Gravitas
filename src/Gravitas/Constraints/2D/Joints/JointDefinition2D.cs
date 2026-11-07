//=======================================================================
// JointDefinition2D.cs
//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

namespace Gravitas.Constraints;

/// <summary>
/// Authored payload used to register one context-owned deterministic pure 2D joint.
/// </summary>
public readonly struct JointDefinition2D
{
    /// <summary>
    /// Creates a pure 2D joint definition.
    /// </summary>
    public JointDefinition2D(
        SolidBody2D bodyA,
        SolidBody2D bodyB,
        JointFrame2D localFrameA,
        JointFrame2D localFrameB,
        JointType2D type,
        JointLimit2D limits,
        JointMotor2D motor,
        JointCollisionPolicy collisionPolicy)
    {
        BodyA = bodyA;
        BodyB = bodyB;
        LocalFrameA = localFrameA;
        LocalFrameB = localFrameB;
        Type = type;
        Limits = limits;
        Motor = motor;
        CollisionPolicy = collisionPolicy;
    }

    /// <summary>
    /// Gets the first body linked by the joint.
    /// </summary>
    public SolidBody2D BodyA { get; }

    /// <summary>
    /// Gets the second body linked by the joint.
    /// </summary>
    public SolidBody2D BodyB { get; }

    /// <summary>
    /// Gets the first local anchor frame. Values are copied at registration time.
    /// </summary>
    public JointFrame2D LocalFrameA { get; }

    /// <summary>
    /// Gets the second local anchor frame. Values are copied at registration time.
    /// </summary>
    public JointFrame2D LocalFrameB { get; }

    /// <summary>
    /// Gets the joint type.
    /// </summary>
    public JointType2D Type { get; }

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
