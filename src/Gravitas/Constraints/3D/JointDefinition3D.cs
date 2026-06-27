//=======================================================================
// JointDefinition3D.cs
//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp;

namespace Gravitas.Constraints;

/// <summary>
/// Authored payload used to register one context-owned deterministic 3D joint.
/// </summary>
public readonly struct JointDefinition3D
{
    /// <summary>
    /// Creates a 3D joint definition.
    /// </summary>
    public JointDefinition3D(
        SolidBody bodyA,
        SolidBody bodyB,
        FixedTransform localFrameA,
        FixedTransform localFrameB,
        JointType3D type,
        JointLimit3D limits,
        JointMotor3D motor,
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
    public SolidBody BodyA { get; }

    /// <summary>
    /// Gets the second body linked by the joint.
    /// </summary>
    public SolidBody BodyB { get; }

    /// <summary>
    /// Gets the first local anchor frame. Values are copied at registration time.
    /// </summary>
    public FixedTransform LocalFrameA { get; }

    /// <summary>
    /// Gets the second local anchor frame. Values are copied at registration time.
    /// </summary>
    public FixedTransform LocalFrameB { get; }

    /// <summary>
    /// Gets the joint type.
    /// </summary>
    public JointType3D Type { get; }

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
