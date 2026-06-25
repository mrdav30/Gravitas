//=======================================================================
// PhysicsEnvironment.cs
//=======================================================================
// MIT License, Copyright (c) 2026–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp;

namespace Gravitas;

/// <summary>
/// Stores world-local physical environment values used by deterministic simulation.
/// </summary>
public sealed class PhysicsEnvironment
{
    /// <summary>
    /// Standard gravitational acceleration in world units per second squared.
    /// </summary>
    public static readonly Fixed64 DefaultGravity = (Fixed64)9.8f;

    /// <summary>
    /// Standard air density used by drag calculations.
    /// </summary>
    public static readonly Fixed64 DefaultAirDensity = (Fixed64)1.225f;

    /// <summary>
    /// Minimum speed treated as meaningful motion by the standard environment.
    /// </summary>
    public static readonly Fixed64 DefaultMinSpeed = (Fixed64)0.00001f;

    /// <summary>
    /// Maximum linear or angular speed used by the standard environment.
    /// </summary>
    public static readonly Fixed64 DefaultMaxSpeed = (Fixed64)7f;

    /// <summary>
    /// Maximum downward fall speed used by the standard environment.
    /// </summary>
    public static readonly Fixed64 DefaultMaxFallSpeed = DefaultGravity;

    /// <summary>
    /// Speed threshold used when transitioning friction behavior in the standard environment.
    /// </summary>
    public static readonly Fixed64 DefaultFrictionTransitionSpeed = (Fixed64)0.2f;

    /// <summary>
    /// Motion deceleration multiplier used by the standard environment.
    /// </summary>
    public static readonly Fixed64 DefaultDecelerationMultiplier = (Fixed64)10f;

    /// <summary>
    /// Angular velocity damping factor used by the standard environment.
    /// </summary>
    public static readonly Fixed64 DefaultDampingFactor = (Fixed64)0.95f;

    /// <summary>
    /// Frame-rate divisor used to derive the default maximum distance-based
    /// collision-culling score.
    /// </summary>
    public static readonly int DefaultCullDistanceFrameDivisor = 3;

    /// <summary>
    /// Distance threshold whose square becomes the default fast-collision
    /// preservation threshold.
    /// </summary>
    public static readonly int DefaultCullFastDistance = 4;

    /// <summary>
    /// Squared distance below which the standard environment preserves fast
    /// collision checks.
    /// </summary>
    public static readonly Fixed64 DefaultCullFastDistanceMax =
        Fixed64.One * DefaultCullFastDistance * (Fixed64.One * DefaultCullFastDistance);

    /// <summary>
    /// Velocity step used by standard collision culling.
    /// </summary>
    public static readonly int DefaultCullVelocityStep = 2;

    /// <summary>
    /// Maximum velocity-based collision-culling score used by the standard environment.
    /// </summary>
    public static readonly int DefaultCullVelocityMax = 4;

    /// <summary>
    /// Frame-rate multiplier used to derive the default collision-culling time step.
    /// </summary>
    public static readonly int DefaultCullTimeStepFrameMultiplier = 3;

    /// <summary>
    /// Frame-rate divisor used to derive the default maximum time-based
    /// collision-culling score.
    /// </summary>
    public static readonly int DefaultCullTimeMaxFrameDivisor = 5;

    /// <summary>
    /// One pound is equal to this many Newtons.
    /// </summary>
    public static readonly Fixed64 PoundToNewton = (Fixed64)4.44822162f;

    /// <summary>
    /// One kilogram is equal to this many pounds.
    /// </summary>
    public static readonly Fixed64 KilogramToPound = (Fixed64)2.20462262f;

    /// <summary>
    /// Gets or sets gravitational acceleration in world units per second squared.
    /// </summary>
    public Fixed64 Gravity { get; set; }

    /// <summary>
    /// Gets or sets air density used by drag calculations.
    /// </summary>
    public Fixed64 AirDensity { get; set; }

    /// <summary>
    /// Gets or sets the minimum speed treated as meaningful motion.
    /// </summary>
    public Fixed64 MinSpeed { get; set; }

    /// <summary>
    /// Gets or sets the maximum linear or angular speed.
    /// </summary>
    public Fixed64 MaxSpeed { get; set; }

    /// <summary>
    /// Gets or sets the maximum downward fall speed.
    /// </summary>
    public Fixed64 MaxFallSpeed { get; set; }

    /// <summary>
    /// Gets or sets the speed threshold used when transitioning friction behavior.
    /// </summary>
    public Fixed64 FrictionTransitionSpeed { get; set; }

    /// <summary>
    /// Gets or sets the multiplier applied when decelerating motion.
    /// </summary>
    public Fixed64 DecelerationMultiplier { get; set; }

    /// <summary>
    /// Gets or sets the damping factor applied to angular velocity.
    /// </summary>
    public Fixed64 DampingFactor { get; set; }

    /// <summary>
    /// Gets or sets the maximum distance-based collision-culling score.
    /// </summary>
    public int CullDistanceMax { get; set; }

    /// <summary>
    /// Gets or sets the squared distance below which fast collision checks are preserved.
    /// </summary>
    public Fixed64 CullFastDistanceMax { get; set; }

    /// <summary>
    /// Gets or sets the velocity step used by collision culling.
    /// </summary>
    public int CullVelocityStep { get; set; }

    /// <summary>
    /// Gets or sets the maximum velocity-based collision-culling score.
    /// </summary>
    public int CullVelocityMax { get; set; }

    /// <summary>
    /// Gets or sets the frame-count step used by collision culling.
    /// </summary>
    public int CullTimeStep { get; set; }

    /// <summary>
    /// Gets or sets the maximum time-based collision-culling score.
    /// </summary>
    public int CullTimeMax { get; set; }

    /// <summary>
    /// Initializes a new environment with explicit physical and culling values.
    /// </summary>
    public PhysicsEnvironment(
        Fixed64 gravity,
        Fixed64 airDensity,
        Fixed64 minSpeed,
        Fixed64 maxSpeed,
        Fixed64 maxFallSpeed,
        Fixed64 frictionTransitionSpeed,
        Fixed64 decelerationMultiplier,
        Fixed64 dampingFactor,
        int cullDistanceMax,
        Fixed64 cullFastDistanceMax,
        int cullVelocityStep,
        int cullVelocityMax,
        int cullTimeStep,
        int cullTimeMax)
    {
        Gravity = gravity;
        AirDensity = airDensity;
        MinSpeed = minSpeed;
        MaxSpeed = maxSpeed;
        MaxFallSpeed = maxFallSpeed;
        FrictionTransitionSpeed = frictionTransitionSpeed;
        DecelerationMultiplier = decelerationMultiplier;
        DampingFactor = dampingFactor;
        CullDistanceMax = cullDistanceMax;
        CullFastDistanceMax = cullFastDistanceMax;
        CullVelocityStep = cullVelocityStep;
        CullVelocityMax = cullVelocityMax;
        CullTimeStep = cullTimeStep;
        CullTimeMax = cullTimeMax;
    }

    /// <summary>
    /// Creates environment values for the standard Gravitas runtime defaults.
    /// </summary>
    /// <param name="frameRate">Frame rate used to initialize frame-derived culling thresholds.</param>
    /// <returns>A new environment instance.</returns>
    public static PhysicsEnvironment Default(int frameRate = PhysicsSettings.DefaultFrameRate)
    {
        SwiftThrowHelper.ThrowIfNegativeOrZero(frameRate, nameof(frameRate));

        return new PhysicsEnvironment(
            gravity: DefaultGravity,
            airDensity: DefaultAirDensity,
            minSpeed: DefaultMinSpeed,
            maxSpeed: DefaultMaxSpeed,
            maxFallSpeed: DefaultMaxFallSpeed,
            frictionTransitionSpeed: DefaultFrictionTransitionSpeed,
            decelerationMultiplier: DefaultDecelerationMultiplier,
            dampingFactor: DefaultDampingFactor,
            cullDistanceMax: frameRate / DefaultCullDistanceFrameDivisor,
            cullFastDistanceMax: DefaultCullFastDistanceMax,
            cullVelocityStep: DefaultCullVelocityStep,
            cullVelocityMax: DefaultCullVelocityMax,
            cullTimeStep: frameRate * DefaultCullTimeStepFrameMultiplier,
            cullTimeMax: frameRate / DefaultCullTimeMaxFrameDivisor);
    }
}
