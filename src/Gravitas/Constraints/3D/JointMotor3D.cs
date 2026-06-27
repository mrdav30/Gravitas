//=======================================================================
// JointMotor3D.cs
//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp;
using SwiftCollections;

namespace Gravitas.Constraints;

/// <summary>
/// Deterministic angular target-drive payload for a 3D joint.
/// </summary>
public readonly struct JointMotor3D
{
    /// <summary>
    /// Creates an angular motor payload.
    /// </summary>
    public JointMotor3D(
        FixedQuaternion targetLocalRotation,
        Fixed64 angularDriveStrength,
        Fixed64 angularDriveDamping,
        Fixed64 maximumMotorImpulse)
    {
        TargetLocalRotation = targetLocalRotation.Normalized;
        AngularDriveStrength = angularDriveStrength;
        AngularDriveDamping = angularDriveDamping;
        MaximumMotorImpulse = maximumMotorImpulse;
        Validate();
    }

    /// <summary>
    /// Gets a disabled motor payload.
    /// </summary>
    public static JointMotor3D Disabled => default;

    /// <summary>
    /// Gets the desired B-relative-to-A joint-space rotation.
    /// </summary>
    public FixedQuaternion TargetLocalRotation { get; }

    /// <summary>
    /// Gets the deterministic spring-like angular drive strength.
    /// </summary>
    public Fixed64 AngularDriveStrength { get; }

    /// <summary>
    /// Gets angular damping applied against relative angular velocity.
    /// </summary>
    public Fixed64 AngularDriveDamping { get; }

    /// <summary>
    /// Gets the maximum absolute impulse this motor may apply per row and iteration.
    /// </summary>
    public Fixed64 MaximumMotorImpulse { get; }

    /// <summary>
    /// Gets whether this motor can emit solver rows.
    /// </summary>
    public bool IsEnabled =>
        AngularDriveStrength > Fixed64.Zero
        && MaximumMotorImpulse > Fixed64.Zero;

    internal void Validate()
    {
        SwiftThrowHelper.ThrowIfArgument(
            AngularDriveStrength < Fixed64.Zero,
            nameof(AngularDriveStrength),
            "Angular drive strength cannot be negative.");
        SwiftThrowHelper.ThrowIfArgument(
            AngularDriveDamping < Fixed64.Zero,
            nameof(AngularDriveDamping),
            "Angular drive damping cannot be negative.");
        SwiftThrowHelper.ThrowIfArgument(
            MaximumMotorImpulse < Fixed64.Zero,
            nameof(MaximumMotorImpulse),
            "Maximum motor impulse cannot be negative.");
    }
}
