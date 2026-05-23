using FixedMathSharp;
using Gravitas.Support;
using SwiftCollections;

namespace Gravitas;

public sealed class PhysicsSettings
{
    public const int DefaultFrameRate = 32;

    public const int MaxLayers = 32;

    public int FrameRate { get; private set; }

    public Fixed64 FixedFrameRate => (Fixed64)FrameRate;

    private readonly bool[,] _collisionMatrix;
    public bool[,] CollisionMatrix => _collisionMatrix;

    public bool PoolingEnabled { get; set; } = true;

    public SingleLayer IgnoreForGroundCheck = ~(1 << 8 | 1 << 10 | 1 << 7 | 1 << 11 | 1 << 12 | 1 << 17 | 1 << 15);

    public PhysicsSettings(int? frameRate, bool[,]? collisionMatrix)
    {
        SetFrameRate(frameRate ?? DefaultFrameRate);
        _collisionMatrix = collisionMatrix ?? GetRegisteredCollisionMatrix();
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
            string? layerName = SingleLayer.LayerToName(i);
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
