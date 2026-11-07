//=======================================================================
// JointSolveMetrics2D.cs
//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp;

namespace Gravitas.Constraints;

/// <summary>
/// Deterministic metrics captured from the most recent pure 2D joint solver pass.
/// </summary>
public readonly struct JointSolveMetrics2D
{
    internal JointSolveMetrics2D(
        int preparedRowCount,
        Fixed64 linearAnchorErrorMagnitude,
        Fixed64 angularErrorMagnitude,
        Fixed64 limitErrorMagnitude,
        Fixed64 accumulatedImpulseMagnitude,
        Fixed64 incrementalImpulseMagnitude,
        Fixed64 motorImpulseMagnitude,
        Fixed64 motorErrorMagnitude,
        int clampedRowCount)
    {
        PreparedRowCount = preparedRowCount;
        LinearAnchorErrorMagnitude = linearAnchorErrorMagnitude;
        AngularErrorMagnitude = angularErrorMagnitude;
        LimitErrorMagnitude = limitErrorMagnitude;
        AccumulatedImpulseMagnitude = accumulatedImpulseMagnitude;
        IncrementalImpulseMagnitude = incrementalImpulseMagnitude;
        MotorImpulseMagnitude = motorImpulseMagnitude;
        MotorErrorMagnitude = motorErrorMagnitude;
        ClampedRowCount = clampedRowCount;
    }

    /// <summary>
    /// Gets the number of solver rows prepared for the joint.
    /// </summary>
    public int PreparedRowCount { get; }

    /// <summary>
    /// Gets the world-space distance between the joint anchors before the pass.
    /// </summary>
    public Fixed64 LinearAnchorErrorMagnitude { get; }

    /// <summary>
    /// Gets the scalar angular frame error magnitude before the pass.
    /// </summary>
    public Fixed64 AngularErrorMagnitude { get; }

    /// <summary>
    /// Gets the total scalar limit violation magnitude emitted by limit rows.
    /// </summary>
    public Fixed64 LimitErrorMagnitude { get; }

    /// <summary>
    /// Gets the total absolute cached impulse magnitude after the pass.
    /// </summary>
    public Fixed64 AccumulatedImpulseMagnitude { get; }

    /// <summary>
    /// Gets the total absolute incremental impulse emitted during the pass.
    /// </summary>
    public Fixed64 IncrementalImpulseMagnitude { get; }

    /// <summary>
    /// Gets the absolute incremental impulse emitted by motor rows.
    /// </summary>
    public Fixed64 MotorImpulseMagnitude { get; }

    /// <summary>
    /// Gets the scalar motor target error magnitude before the pass.
    /// </summary>
    public Fixed64 MotorErrorMagnitude { get; }

    /// <summary>
    /// Gets the number of rows whose accumulated impulse hit a row bound.
    /// </summary>
    public int ClampedRowCount { get; }
}
