using Gravitas.Support;
using MemoryPack;
using System;
using SwiftCollections;

namespace Gravitas;

[Serializable]
[MemoryPackable]
public partial struct MatrixRow
{
    public bool[] row;
}

[Serializable]
[MemoryPackable]
public sealed partial class PhysicsSettingsSaver : DefaultSaver
{
    public int? FrameRate;

    public MatrixRow[]? CollisionMatrix;

    public int? GroundCheckLayerMaskBits;

    public ContinuousCollisionMode? DefaultContinuousCollisionMode;

    public int? RetainedPartitionTimeToKillFrames;

    public int? RetainedPartitionRetirementSweepBudget;

    [NonSerialized]
    [MemoryPackIgnore]
    private GravitasWorldContext? _context;

    public void BindContext(GravitasWorldContext context)
    {
        SwiftThrowHelper.ThrowIfNull(context, nameof(context));
        _context = context;
    }

    public void ApplyTo(GravitasWorldContext context)
    {
        SwiftThrowHelper.ThrowIfNull(context, nameof(context));
        context.ApplySettings(CreateSettings());
    }

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
        if (RetainedPartitionTimeToKillFrames.HasValue)
            settings.RetainedPartitionTimeToKillFrames = RetainedPartitionTimeToKillFrames.Value;
        if (RetainedPartitionRetirementSweepBudget.HasValue)
            settings.RetainedPartitionRetirementSweepBudget = RetainedPartitionRetirementSweepBudget.Value;

        return settings;
    }

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
                row == null || row.Length < numberOfLayers,
                nameof(CollisionMatrix),
                "Physics settings collision matrix rows must be square.");

            for (int j = 0; j < numberOfLayers; ++j)
                matrix[i, j] = row[j];
        }

        return matrix;
    }
}
