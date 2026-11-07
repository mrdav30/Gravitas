//=======================================================================
// ContinuousCollisionMath.RotationalInterval.cs
//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp;

namespace Gravitas.CollisionHandling;

/// <content>Defines the nested RotationalInterval type.</content>
internal static partial class ContinuousCollisionMath
{
    public readonly struct RotationalInterval
    {
        public RotationalInterval(Fixed64 lowerTime, Fixed64 upperTime, int depth)
        {
            LowerTime = lowerTime;
            UpperTime = upperTime;
            Depth = depth;
        }

        public Fixed64 LowerTime { get; }

        public Fixed64 UpperTime { get; }

        public int Depth { get; }
    }
}
