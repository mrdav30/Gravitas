//=======================================================================
// RotationalInterval.cs
//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp;

namespace Gravitas.CollisionHandling;

/// <summary>
/// Stores the time bounds and subdivision depth of a rotational CCD search interval.
/// </summary>
internal readonly struct RotationalInterval
{
    internal RotationalInterval(Fixed64 lowerTime, Fixed64 upperTime, int depth)
    {
        LowerTime = lowerTime;
        UpperTime = upperTime;
        Depth = depth;
    }

    internal Fixed64 LowerTime { get; }

    internal Fixed64 UpperTime { get; }

    internal int Depth { get; }
}
