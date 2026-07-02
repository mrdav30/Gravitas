//=======================================================================
// JointLimit3D.cs
//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp;

namespace Gravitas.Constraints;

/// <summary>
/// Optional deterministic angular limit payload for a 3D joint.
/// </summary>
public readonly struct JointLimit3D
{
    private JointLimit3D(
        JointLimitKind3D kind,
        Fixed64 maxHingeAngle,
        Fixed64 maxConeAngle,
        Fixed64 maxTwistAngle)
    {
        Kind = kind;
        MaxHingeAngle = maxHingeAngle;
        MaxConeAngle = maxConeAngle;
        MaxTwistAngle = maxTwistAngle;
    }

    /// <summary>
    /// Gets an unrestricted limit payload.
    /// </summary>
    public static JointLimit3D Unrestricted => default;

    /// <summary>
    /// Gets the active limit family.
    /// </summary>
    public JointLimitKind3D Kind { get; }

    /// <summary>
    /// Gets the maximum absolute hinge angle in radians.
    /// </summary>
    public Fixed64 MaxHingeAngle { get; }

    /// <summary>
    /// Gets the maximum cone swing angle in radians.
    /// </summary>
    public Fixed64 MaxConeAngle { get; }

    /// <summary>
    /// Gets the maximum absolute twist angle in radians.
    /// </summary>
    public Fixed64 MaxTwistAngle { get; }

    /// <summary>
    /// Creates a hinge limit around the local X axis.
    /// </summary>
    public static JointLimit3D Hinge(Fixed64 maxAngle)
    {
        ValidateNonNegative(maxAngle, nameof(maxAngle));
        return new JointLimit3D(JointLimitKind3D.Hinge, maxAngle, Fixed64.Zero, Fixed64.Zero);
    }

    /// <summary>
    /// Creates a cone-twist limit around the local Z axis.
    /// </summary>
    public static JointLimit3D ConeTwist(Fixed64 maxConeAngle, Fixed64 maxTwistAngle)
    {
        ValidateNonNegative(maxConeAngle, nameof(maxConeAngle));
        ValidateNonNegative(maxTwistAngle, nameof(maxTwistAngle));
        return new JointLimit3D(JointLimitKind3D.ConeTwist, Fixed64.Zero, maxConeAngle, maxTwistAngle);
    }

    internal void Validate()
    {
        SwiftThrowHelper.ThrowIfArgument(
            Kind != JointLimitKind3D.Unrestricted
                && Kind != JointLimitKind3D.Hinge
                && Kind != JointLimitKind3D.ConeTwist,
            nameof(Kind),
            "Unsupported joint limit kind.");
        ValidateNonNegative(MaxHingeAngle, nameof(MaxHingeAngle));
        ValidateNonNegative(MaxConeAngle, nameof(MaxConeAngle));
        ValidateNonNegative(MaxTwistAngle, nameof(MaxTwistAngle));
    }

    private static void ValidateNonNegative(Fixed64 value, string name) =>
        SwiftThrowHelper.ThrowIfArgument(value < Fixed64.Zero, name, "Joint limit angles cannot be negative.");
}
