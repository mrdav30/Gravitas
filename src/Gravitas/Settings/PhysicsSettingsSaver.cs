using Gravitas.Support;
using MemoryPack;
using System;

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

    protected override void OnEarlyApply()
    {
        // Convert the MatrixRow array back to a 2D bool array to pass to PhysicsManager
        int numberOfLayers = CollisionMatrix?.Length ?? 0;
        if (CollisionMatrix == null || numberOfLayers == 0)
        {
            // If there are no layers, we can just set an empty collision matrix
            PhysicsManager.Settings = PhysicsSettings.DefaultSettings();
            return;
        }

        bool[,] matrix = new bool[numberOfLayers, numberOfLayers];
        for (int i = 0; i < numberOfLayers; ++i)
            for (int j = 0; j < numberOfLayers; ++j)
                matrix[i, j] = CollisionMatrix[i].row[j];

        PhysicsManager.Settings = new PhysicsSettings(FrameRate, matrix);
    }
}