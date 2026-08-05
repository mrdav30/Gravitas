//=======================================================================
// PhysicsSettings.cs
//=======================================================================
// MIT License, Copyright (c) 2026–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp;
using Gravitas.Support;
using SwiftCollections;
using System;
using System.Runtime.CompilerServices;

namespace Gravitas;

/// <summary>
/// Stores context-local configuration for deterministic physics simulation.
/// </summary>
public sealed partial class PhysicsSettings
{
    /// <summary>
    /// Default fixed-step frame rate in simulation frames per second.
    /// </summary>
    public const int DefaultFrameRate = 32;

    /// <summary>
    /// Maximum supported fixed-step frame rate.
    /// </summary>
    /// <remarks>
    /// Higher values quantize <see cref="GravitasWorldContext.DeltaTime"/> at or below
    /// <see cref="Fixed64.Epsilon"/>, which would make integration and CCD degeneracy
    /// checks disagree about whether a step has meaningful duration.
    /// </remarks>
    public const int MaxResolvableFrameRate = (int)(FixedMath.ONE_L / (FixedMath.DEFAULT_TOLERANCE_L + 1));

    /// <summary>
    /// Maximum number of physics layers represented by a layer mask.
    /// </summary>
    public const int MaxLayers = 32;

    /// <summary>
    /// Default number of frames an empty partition remains retained for reuse.
    /// </summary>
    public const int DefaultRetainedPartitionTimeToKillFrames = DefaultFrameRate * 10;

    /// <summary>
    /// Default maximum number of retained partitions checked per retirement sweep.
    /// </summary>
    public const int DefaultRetainedPartitionRetirementSweepBudget = 64;

    /// <summary>
    /// Default maximum same-frame time-of-impact iterations for continuous collision.
    /// </summary>
    public const int DefaultContinuousCollisionMaxToiIterations = 4;

    /// <summary>
    /// Default projected-impulse iteration count for discrete 3D constraint islands.
    /// </summary>
    public const int DefaultDiscreteSolverIterations = 6;

    /// <summary>
    /// Default closing-speed threshold below which restitution is disabled.
    /// </summary>
    public static readonly Fixed64 DefaultRestitutionVelocityThreshold = (Fixed64)0.25f;

    /// <summary>
    /// Default Y-axis half-thickness for 2D colliders embedded in mixed queries and contacts.
    /// </summary>
    public static readonly Fixed64 DefaultMixed2DHalfThickness = Fixed64.Half;

    /// <summary>
    /// Default include mask used for ground and support checks.
    /// </summary>
    public static readonly PhysicsLayerMask DefaultGroundCheckLayerMask = PhysicsLayerMask.FromLayer(new PhysicsLayer(0));

    /// <summary>
    /// Gets the fixed-step frame rate in simulation frames per second.
    /// </summary>
    public int FrameRate { get; private set; }

    private readonly bool[,] _collisionMatrix;

    /// <summary>
    /// Gets the layer-to-layer physical collision enablement matrix.
    /// </summary>
    public bool[,] CollisionMatrix => _collisionMatrix;

    /// <summary>
    /// Gets or sets whether reusable runtime collision objects are pooled.
    /// </summary>
    public bool PoolingEnabled { get; set; } = true;

    /// <summary>
    /// Gets or sets the include mask used for ground and support checks.
    /// </summary>
    public PhysicsLayerMask GroundCheckLayerMask { get; set; }

    private int _retainedPartitionTimeToKillFrames = DefaultRetainedPartitionTimeToKillFrames;
    private int _retainedPartitionRetirementSweepBudget = DefaultRetainedPartitionRetirementSweepBudget;
    private ContinuousCollisionMode _defaultContinuousCollisionMode = ContinuousCollisionMode.Discrete;
    private int _continuousCollisionMaxToiIterations = DefaultContinuousCollisionMaxToiIterations;
    private int _discreteSolverIterations = DefaultDiscreteSolverIterations;
    private Fixed64 _restitutionVelocityThreshold = DefaultRestitutionVelocityThreshold;
    private PhysicsRuntimeMode _runtimeMode = PhysicsRuntimeMode.ThreeD;
    private Fixed64 _mixed2DHalfThickness = DefaultMixed2DHalfThickness;

    /// <summary>
    /// Gets or sets how many simulation frames an empty voxel partition should stay attached for fast reuse.
    /// A value of zero retires eligible partitions on the next retirement sweep.
    /// </summary>
    public int RetainedPartitionTimeToKillFrames
    {
        get => _retainedPartitionTimeToKillFrames;
        set
        {
            SwiftThrowHelper.ThrowIfNegative(value, nameof(value));
            _retainedPartitionTimeToKillFrames = value;
        }
    }

    /// <summary>
    /// Gets or sets the maximum retained partitions checked for retirement during one collision distribution step.
    /// A value of zero disables retirement sweeps.
    /// </summary>
    public int RetainedPartitionRetirementSweepBudget
    {
        get => _retainedPartitionRetirementSweepBudget;
        set
        {
            SwiftThrowHelper.ThrowIfNegative(value, nameof(value));
            _retainedPartitionRetirementSweepBudget = value;
        }
    }

    /// <summary>
    /// Gets or sets the default tunneling policy used by bodies configured to inherit from the context.
    /// A context default of <see cref="ContinuousCollisionMode.Inherit"/> resolves to
    /// <see cref="ContinuousCollisionMode.Discrete"/>.
    /// </summary>
    /// <exception cref="ArgumentOutOfRangeException">The value is not a declared continuous-collision mode.</exception>
    public ContinuousCollisionMode DefaultContinuousCollisionMode
    {
        get => _defaultContinuousCollisionMode;
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        set
        {
            value.ThrowIfInvalid(nameof(value));
            _defaultContinuousCollisionMode = value;
        }
    }

    /// <summary>
    /// Gets or sets the maximum same-frame continuous-collision TOI iterations one body or handoff queue may consume.
    /// </summary>
    public int ContinuousCollisionMaxToiIterations
    {
        get => _continuousCollisionMaxToiIterations;
        set
        {
            SwiftThrowHelper.ThrowIfNegativeOrZero(value, nameof(value));
            _continuousCollisionMaxToiIterations = value;
        }
    }

    /// <summary>
    /// Gets or sets the bounded projected-impulse iteration count used for 3D discrete
    /// contact and joint constraint islands. Contact-only single-pair scenes stay on
    /// the direct one-pass response path.
    /// </summary>
    public int DiscreteSolverIterations
    {
        get => _discreteSolverIterations;
        set
        {
            SwiftThrowHelper.ThrowIfNegativeOrZero(value, nameof(value));
            _discreteSolverIterations = value;
        }
    }

    /// <summary>
    /// Gets or sets the closing speed at or below which contact response uses zero restitution.
    /// </summary>
    public Fixed64 RestitutionVelocityThreshold
    {
        get => _restitutionVelocityThreshold;
        set
        {
            SwiftThrowHelper.ThrowIfArgument(
                value < Fixed64.Zero,
                nameof(value),
                "Restitution velocity threshold cannot be negative.");
            _restitutionVelocityThreshold = value;
        }
    }

    /// <summary>
    /// Gets or sets the default half-thickness used when pure 2D colliders are embedded into mixed 2D/3D contacts.
    /// </summary>
    public Fixed64 Mixed2DHalfThickness
    {
        get => _mixed2DHalfThickness;
        set
        {
            SwiftThrowHelper.ThrowIfArgument(
                value <= Fixed64.Zero,
                nameof(value),
                "Mixed 2D half-thickness must be greater than zero.");
            _mixed2DHalfThickness = value;
        }
    }

    /// <summary>
    /// Gets or sets which dimensional physics service this context should advance.
    /// </summary>
    public PhysicsRuntimeMode RuntimeMode
    {
        get => _runtimeMode;
        set
        {
            SwiftThrowHelper.ThrowIfArgument(
                !value.IsValid(),
                nameof(value),
                "Physics runtime mode must be TwoD, ThreeD, Both, or Mixed.");
            _runtimeMode = value;
        }
    }

    /// <summary>
    /// Creates physics settings, using registered-layer defaults for omitted values.
    /// </summary>
    public PhysicsSettings(
        int? frameRate,
        bool[,]? collisionMatrix,
        PhysicsLayerMask? groundCheckLayerMask = null)
    {
        SetFrameRate(frameRate ?? DefaultFrameRate);
        _collisionMatrix = collisionMatrix ?? GetRegisteredCollisionMatrix();
        GroundCheckLayerMask = groundCheckLayerMask ?? DefaultGroundCheckLayerMask;
    }

    /// <summary>
    /// Sets the fixed-step frame rate after validating its representable range.
    /// </summary>
    public void SetFrameRate(int frameRate)
    {
        ThrowIfInvalidFrameRate(frameRate);
        FrameRate = frameRate;
    }

    internal static void ThrowIfInvalidFrameRate(int frameRate)
    {
        SwiftThrowHelper.ThrowIfNegativeOrZero(frameRate, nameof(frameRate));
        if (frameRate > MaxResolvableFrameRate)
        {
            throw new ArgumentOutOfRangeException(
                nameof(frameRate),
                frameRate,
                $"Frame rate cannot exceed {MaxResolvableFrameRate}.");
        }
    }

    /// <summary>
    /// Creates settings with default values and the currently registered layer matrix.
    /// </summary>
    public static PhysicsSettings DefaultSettings()
    {
        bool[,] collisionMatrix = GetRegisteredCollisionMatrix();
        return new PhysicsSettings(DefaultFrameRate, collisionMatrix);
    }

    /// <summary>
    /// Creates a fully enabled square collision matrix sized to the registered layer names.
    /// </summary>
    public static bool[,] GetRegisteredCollisionMatrix()
    {
        SwiftList<string> layersList = new();

        for (int i = 0; i < MaxLayers; ++i)
        {
            string? layerName = PhysicsLayer.LayerToName(i);
            // Check if the layer has a name
            if (!string.IsNullOrEmpty(layerName))
                layersList.Add(layerName);
        }

        string[] layerNames = layersList.ToArray();
        int numberOfLayers = layerNames.Length;

        if (numberOfLayers == 0)
            return new bool[0, 0];

        bool[,] collisionMatrix = new bool[numberOfLayers, numberOfLayers];
        for (int i = 0; i < numberOfLayers; ++i)
            for (int j = 0; j < numberOfLayers; ++j)
                collisionMatrix[i, j] = true;

        return collisionMatrix;
    }
}
