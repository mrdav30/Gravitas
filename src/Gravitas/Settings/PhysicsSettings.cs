using FixedMathSharp;
using Gravitas.Support;
using SwiftCollections;

namespace Gravitas;

public sealed class PhysicsSettings
{
    public const int DefaultFrameRate = 32;

    public const int MaxLayers = 32;

    public const int DefaultRetainedPartitionTimeToKillFrames = DefaultFrameRate * 10;

    public const int DefaultRetainedPartitionRetirementSweepBudget = 64;

    public static readonly Fixed64 DefaultMixed2DHalfThickness = Fixed64.Half;

    /// <summary>
    /// Legacy prototype example ground-check include mask. Hosts should configure
    /// this explicitly for their own layer model.
    /// </summary>
    public static readonly PhysicsLayerMask DefaultGroundCheckLayerMask = PhysicsLayerMask.Excluding(
        new PhysicsLayer(8),
        new PhysicsLayer(10),
        new PhysicsLayer(7),
        new PhysicsLayer(11),
        new PhysicsLayer(12),
        new PhysicsLayer(17),
        new PhysicsLayer(15));

    public int FrameRate { get; private set; }

    public Fixed64 FixedFrameRate => (Fixed64)FrameRate;

    private readonly bool[,] _collisionMatrix;
    public bool[,] CollisionMatrix => _collisionMatrix;

    public bool PoolingEnabled { get; set; } = true;

    public PhysicsLayerMask GroundCheckLayerMask { get; set; }

    private int _retainedPartitionTimeToKillFrames = DefaultRetainedPartitionTimeToKillFrames;
    private int _retainedPartitionRetirementSweepBudget = DefaultRetainedPartitionRetirementSweepBudget;
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
    /// </summary>
    public ContinuousCollisionMode DefaultContinuousCollisionMode { get; set; } = ContinuousCollisionMode.Discrete;

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

    public PhysicsSettings(
        int? frameRate,
        bool[,]? collisionMatrix,
        PhysicsLayerMask? groundCheckLayerMask = null)
    {
        SetFrameRate(frameRate ?? DefaultFrameRate);
        _collisionMatrix = collisionMatrix ?? GetRegisteredCollisionMatrix();
        GroundCheckLayerMask = groundCheckLayerMask ?? DefaultGroundCheckLayerMask;
    }

    public void SetFrameRate(int frameRate)
    {
        SwiftThrowHelper.ThrowIfNegativeOrZero(frameRate, nameof(frameRate));
        FrameRate = frameRate;
    }

    public static PhysicsSettings DefaultSettings()
    {
        bool[,] collisionMatrix = GetRegisteredCollisionMatrix();
        return new PhysicsSettings(DefaultFrameRate, collisionMatrix);
    }

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
