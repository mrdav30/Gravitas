//=======================================================================
// JointLimit2D.cs
//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp;

namespace Gravitas.Constraints;

/// <summary>
/// Optional deterministic scalar limit payload for a pure 2D joint.
/// </summary>
public readonly struct JointLimit2D
{
    private JointLimit2D(
        JointLimitKind2D kind,
        Fixed64 targetDistance,
        Fixed64 minTranslation,
        Fixed64 maxTranslation,
        Fixed64 minAngle,
        Fixed64 maxAngle)
    {
        Kind = kind;
        TargetDistance = targetDistance;
        MinTranslation = minTranslation;
        MaxTranslation = maxTranslation;
        MinAngle = minAngle;
        MaxAngle = maxAngle;
    }

    /// <summary>
    /// Gets an unrestricted limit payload.
    /// </summary>
    public static JointLimit2D Unrestricted => default;

    /// <summary>
    /// Gets the active limit family.
    /// </summary>
    public JointLimitKind2D Kind { get; }

    /// <summary>
    /// Gets the target anchor distance for distance joints.
    /// </summary>
    public Fixed64 TargetDistance { get; }

    /// <summary>
    /// Gets the minimum slider translation in world units.
    /// </summary>
    public Fixed64 MinTranslation { get; }

    /// <summary>
    /// Gets the maximum slider translation in world units.
    /// </summary>
    public Fixed64 MaxTranslation { get; }

    /// <summary>
    /// Gets the minimum scalar relative angle in radians.
    /// </summary>
    public Fixed64 MinAngle { get; }

    /// <summary>
    /// Gets the maximum scalar relative angle in radians.
    /// </summary>
    public Fixed64 MaxAngle { get; }

    /// <summary>
    /// Creates a deterministic distance target.
    /// </summary>
    public static JointLimit2D Distance(Fixed64 targetDistance)
    {
        SwiftThrowHelper.ThrowIfArgument(
            targetDistance < Fixed64.Zero,
            nameof(targetDistance),
            "Joint target distance cannot be negative.");
        return new JointLimit2D(JointLimitKind2D.Distance, targetDistance, Fixed64.Zero, Fixed64.Zero, Fixed64.Zero, Fixed64.Zero);
    }

    /// <summary>
    /// Creates a deterministic prismatic slider translation range.
    /// </summary>
    public static JointLimit2D Slider(Fixed64 minTranslation, Fixed64 maxTranslation)
    {
        SwiftThrowHelper.ThrowIfArgument(
            minTranslation > maxTranslation,
            nameof(minTranslation),
            "Slider minimum translation cannot exceed maximum translation.");
        return new JointLimit2D(JointLimitKind2D.Slider, Fixed64.Zero, minTranslation, maxTranslation, Fixed64.Zero, Fixed64.Zero);
    }

    /// <summary>
    /// Creates a deterministic scalar angular range.
    /// </summary>
    public static JointLimit2D Angular(Fixed64 minAngle, Fixed64 maxAngle)
    {
        SwiftThrowHelper.ThrowIfArgument(
            minAngle > maxAngle,
            nameof(minAngle),
            "Angular minimum angle cannot exceed maximum angle.");
        return new JointLimit2D(JointLimitKind2D.Angular, Fixed64.Zero, Fixed64.Zero, Fixed64.Zero, minAngle, maxAngle);
    }

    internal void Validate()
    {
        SwiftThrowHelper.ThrowIfArgument(
            Kind != JointLimitKind2D.Unrestricted
                && Kind != JointLimitKind2D.Distance
                && Kind != JointLimitKind2D.Slider
                && Kind != JointLimitKind2D.Angular,
            nameof(Kind),
            "Unsupported 2D joint limit kind.");
        SwiftThrowHelper.ThrowIfArgument(TargetDistance < Fixed64.Zero, nameof(TargetDistance), "Joint target distance cannot be negative.");
        SwiftThrowHelper.ThrowIfArgument(MinTranslation > MaxTranslation, nameof(MinTranslation), "Slider minimum translation cannot exceed maximum translation.");
        SwiftThrowHelper.ThrowIfArgument(MinAngle > MaxAngle, nameof(MinAngle), "Angular minimum angle cannot exceed maximum angle.");
    }
}
