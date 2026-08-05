//=======================================================================
// PhysicsSettingsSaver.cs
//=======================================================================
// MIT License, Copyright (c) 2026–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using Chronicler;
using FixedMathSharp;
using Gravitas.Support;
using MemoryPack;
using System;
using System.Text.Json.Serialization;

namespace Gravitas;

/// <summary>
/// Stores one serialized row of a physics collision matrix.
/// </summary>
[Serializable]
[MemoryPackable]
public partial struct MatrixRow
{
    /// <summary>
    /// Collision enablement values for this matrix row.
    /// </summary>
    [JsonInclude]
    public bool[] row;
}

/// <summary>
/// Serializes optional physics settings and applies them to an explicit world context.
/// </summary>
[Serializable]
[MemoryPackable]
public sealed partial class PhysicsSettingsSaver : DefaultSaver
{
    /// <summary>
    /// Optional fixed-step frame rate in simulation frames per second.
    /// </summary>
    [JsonInclude]
    public int? FrameRate;

    /// <summary>
    /// Optional square layer-to-layer collision matrix.
    /// </summary>
    [JsonInclude]
    public MatrixRow[]? CollisionMatrix;

    /// <summary>
    /// Optional include-mask bits for ground and support checks.
    /// </summary>
    [JsonInclude]
    public int? GroundCheckLayerMaskBits;

    /// <summary>
    /// Optional default continuous-collision mode.
    /// </summary>
    [JsonInclude]
    public ContinuousCollisionMode? DefaultContinuousCollisionMode;

    /// <summary>
    /// Optional maximum same-frame continuous-collision time-of-impact iterations.
    /// </summary>
    [JsonInclude]
    public int? ContinuousCollisionMaxToiIterations;

    /// <summary>
    /// Optional discrete 3D constraint-island solver iteration count.
    /// </summary>
    [JsonInclude]
    public int? DiscreteSolverIterations;

    /// <summary>
    /// Optional closing-speed threshold below which restitution is disabled.
    /// </summary>
    [JsonInclude]
    public Fixed64? RestitutionVelocityThreshold;

    /// <summary>
    /// Optional number of frames an empty partition remains retained for reuse.
    /// </summary>
    [JsonInclude]
    public int? RetainedPartitionTimeToKillFrames;

    /// <summary>
    /// Optional maximum retained partitions checked per retirement sweep.
    /// </summary>
    [JsonInclude]
    public int? RetainedPartitionRetirementSweepBudget;

    /// <summary>
    /// Optional dimensional runtime mode.
    /// </summary>
    [JsonInclude]
    public PhysicsRuntimeMode? RuntimeMode;

    /// <summary>
    /// Optional Y-axis half-thickness for 2D colliders embedded in mixed queries and contacts.
    /// </summary>
    [JsonInclude]
    public Fixed64? Mixed2DHalfThickness;

    [NonSerialized]
    [MemoryPackIgnore]
    private GravitasWorldContext? _context;

    /// <summary>
    /// Binds the context used by the inherited early-apply phase.
    /// </summary>
    public void BindContext(GravitasWorldContext context)
    {
        SwiftThrowHelper.ThrowIfNull(context, nameof(context));
        _context = context;
    }

    /// <summary>
    /// Creates and applies these settings to the specified context.
    /// </summary>
    public void ApplyTo(GravitasWorldContext context)
    {
        SwiftThrowHelper.ThrowIfNull(context, nameof(context));
        context.ApplySettings(CreateSettings());
    }

    /// <summary>
    /// Creates validated physics settings, using defaults for omitted values.
    /// </summary>
    public PhysicsSettings CreateSettings()
    {
        var settings = new PhysicsSettings(
            FrameRate,
            CreateCollisionMatrix(),
            GroundCheckLayerMaskBits.HasValue
                ? new PhysicsLayerMask(GroundCheckLayerMaskBits.Value)
                : null);

        if (DefaultContinuousCollisionMode.HasValue)
            settings.DefaultContinuousCollisionMode = DefaultContinuousCollisionMode.Value;
        if (ContinuousCollisionMaxToiIterations.HasValue)
            settings.ContinuousCollisionMaxToiIterations = ContinuousCollisionMaxToiIterations.Value;
        if (DiscreteSolverIterations.HasValue)
            settings.DiscreteSolverIterations = DiscreteSolverIterations.Value;
        if (RestitutionVelocityThreshold.HasValue)
            settings.RestitutionVelocityThreshold = RestitutionVelocityThreshold.Value;
        if (RetainedPartitionTimeToKillFrames.HasValue)
            settings.RetainedPartitionTimeToKillFrames = RetainedPartitionTimeToKillFrames.Value;
        if (RetainedPartitionRetirementSweepBudget.HasValue)
            settings.RetainedPartitionRetirementSweepBudget = RetainedPartitionRetirementSweepBudget.Value;
        if (RuntimeMode.HasValue)
            settings.RuntimeMode = RuntimeMode.Value;
        if (Mixed2DHalfThickness.HasValue)
            settings.Mixed2DHalfThickness = Mixed2DHalfThickness.Value;

        return settings;
    }

    /// <summary>
    /// Applies these settings to the context bound by <see cref="BindContext"/>.
    /// </summary>
    protected override void OnEarlyApply()
    {
        SwiftThrowHelper.ThrowIfTrue(
            _context == null,
            nameof(PhysicsSettingsSaver),
            "PhysicsSettingsSaver requires an explicit GravitasWorldContext. Call BindContext or ApplyTo before applying.");

        ApplyTo(_context!);
    }

    private bool[,]? CreateCollisionMatrix()
    {
        int numberOfLayers = CollisionMatrix?.Length ?? 0;
        if (CollisionMatrix == null || numberOfLayers == 0)
            return null;

        bool[,] matrix = new bool[numberOfLayers, numberOfLayers];
        for (int i = 0; i < numberOfLayers; ++i)
        {
            bool[] row = CollisionMatrix[i].row;
            SwiftThrowHelper.ThrowIfTrue(
                row == null || row.Length != numberOfLayers,
                nameof(CollisionMatrix),
                "Physics settings collision matrix rows must be square.");

            for (int j = 0; j < numberOfLayers; ++j)
                matrix[i, j] = row[j];
        }

        return matrix;
    }
}
