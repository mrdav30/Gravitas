//=======================================================================
// JointMotor2D.cs
//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp;

namespace Gravitas.Constraints;

/// <summary>
/// Deterministic scalar target-drive payload for a pure 2D joint.
/// </summary>
public readonly struct JointMotor2D
{
    private JointMotor2D(
        JointMotorKind2D kind,
        Fixed64 target,
        Fixed64 driveStrength,
        Fixed64 damping,
        Fixed64 maximumMotorImpulse)
    {
        Kind = kind;
        Target = target;
        DriveStrength = driveStrength;
        Damping = damping;
        MaximumMotorImpulse = maximumMotorImpulse;
        Validate();
    }

    /// <summary>
    /// Gets a disabled motor payload.
    /// </summary>
    public static JointMotor2D Disabled => default;

    /// <summary>
    /// Gets the active motor family.
    /// </summary>
    public JointMotorKind2D Kind { get; }

    /// <summary>
    /// Gets the target angle or slider translation for the active motor.
    /// </summary>
    public Fixed64 Target { get; }

    /// <summary>
    /// Gets the deterministic spring-like drive strength.
    /// </summary>
    public Fixed64 DriveStrength { get; }

    /// <summary>
    /// Gets damping applied against the solved relative scalar velocity.
    /// </summary>
    public Fixed64 Damping { get; }

    /// <summary>
    /// Gets the maximum absolute impulse this motor may apply per row and iteration.
    /// </summary>
    public Fixed64 MaximumMotorImpulse { get; }

    /// <summary>
    /// Gets whether this motor can emit solver rows.
    /// </summary>
    public bool IsEnabled =>
        Kind != JointMotorKind2D.Disabled
        && DriveStrength > Fixed64.Zero
        && MaximumMotorImpulse > Fixed64.Zero;

    /// <summary>
    /// Creates an angular motor that drives scalar relative yaw toward <paramref name="targetAngle"/>.
    /// </summary>
    public static JointMotor2D Angular(
        Fixed64 targetAngle,
        Fixed64 driveStrength,
        Fixed64 damping,
        Fixed64 maximumMotorImpulse) =>
        new(JointMotorKind2D.Angular, targetAngle, driveStrength, damping, maximumMotorImpulse);

    /// <summary>
    /// Creates a linear motor that drives prismatic translation toward <paramref name="targetTranslation"/>.
    /// </summary>
    public static JointMotor2D Linear(
        Fixed64 targetTranslation,
        Fixed64 driveStrength,
        Fixed64 damping,
        Fixed64 maximumMotorImpulse) =>
        new(JointMotorKind2D.Linear, targetTranslation, driveStrength, damping, maximumMotorImpulse);

    internal void Validate()
    {
        SwiftThrowHelper.ThrowIfArgument(
            Kind != JointMotorKind2D.Disabled
                && Kind != JointMotorKind2D.Angular
                && Kind != JointMotorKind2D.Linear,
            nameof(Kind),
            "Unsupported 2D joint motor kind.");
        SwiftThrowHelper.ThrowIfArgument(DriveStrength < Fixed64.Zero, nameof(DriveStrength), "Joint motor drive strength cannot be negative.");
        SwiftThrowHelper.ThrowIfArgument(Damping < Fixed64.Zero, nameof(Damping), "Joint motor damping cannot be negative.");
        SwiftThrowHelper.ThrowIfArgument(MaximumMotorImpulse < Fixed64.Zero, nameof(MaximumMotorImpulse), "Joint motor maximum impulse cannot be negative.");
    }
}
