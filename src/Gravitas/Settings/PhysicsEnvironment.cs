using FixedMathSharp;
using SwiftCollections;

namespace Gravitas;

/// <summary>
/// Stores world-local physical environment values used by deterministic simulation.
/// </summary>
public sealed class PhysicsEnvironment
{
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
    /// Creates environment values matching the legacy Gravitas defaults.
    /// </summary>
    /// <param name="frameRate">Frame rate used to initialize frame-derived culling thresholds.</param>
    /// <returns>A new environment instance.</returns>
    public static PhysicsEnvironment Default(int frameRate = PhysicsSettings.DefaultFrameRate)
    {
        SwiftThrowHelper.ThrowIfNegativeOrZero(frameRate, nameof(frameRate));

        return new PhysicsEnvironment(
            gravity: (Fixed64)9.8f,
            airDensity: (Fixed64)1.225f,
            minSpeed: (Fixed64)0.00001f,
            maxSpeed: (Fixed64)7f,
            maxFallSpeed: (Fixed64)9.8f,
            frictionTransitionSpeed: (Fixed64)0.2f,
            decelerationMultiplier: (Fixed64)10f,
            dampingFactor: (Fixed64)0.95f,
            cullDistanceMax: frameRate / 3,
            cullFastDistanceMax: Fixed64.One * 4 * (Fixed64.One * 4),
            cullVelocityStep: 2,
            cullVelocityMax: 4,
            cullTimeStep: frameRate * 3,
            cullTimeMax: frameRate / 5);
    }
}
