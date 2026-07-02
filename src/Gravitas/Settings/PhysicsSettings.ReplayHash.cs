//=======================================================================
// PhysicsSettings.ReplayHash.cs
//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp.Chronicler;
using Chronicler;

namespace Gravitas;

public sealed partial class PhysicsSettings
{
    internal void ContributeReplayHash(ref ChronicleHashWriter writer)
    {
        writer.WriteSection("settings", 1);
        writer.WriteInt32(FrameRate);
        writer.WriteBool(PoolingEnabled);
        writer.WritePhysicsLayerMask(GroundCheckLayerMask);
        writer.WriteInt32(RetainedPartitionTimeToKillFrames);
        writer.WriteInt32(RetainedPartitionRetirementSweepBudget);
        writer.WriteEnum(DefaultContinuousCollisionMode);
        writer.WriteInt32(ContinuousCollisionMaxToiIterations);
        writer.WriteInt32(DiscreteSolverIterations);
        writer.WriteFixed64(RestitutionVelocityThreshold);
        writer.WriteFixed64(Mixed2DHalfThickness);
        writer.WriteEnum(RuntimeMode);

        int rows = _collisionMatrix.GetLength(0);
        int columns = _collisionMatrix.GetLength(1);
        writer.WriteInt32(rows);
        writer.WriteInt32(columns);
        for (int row = 0; row < rows; row++)
        {
            for (int column = 0; column < columns; column++)
                writer.WriteBool(_collisionMatrix[row, column]);
        }
    }
}
