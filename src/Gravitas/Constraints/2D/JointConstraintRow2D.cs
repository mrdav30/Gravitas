//=======================================================================
// JointConstraintRow2D.cs
//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp;

namespace Gravitas.Constraints;

internal enum JointConstraintRowKind2D : byte
{
    Linear,
    Angular,
    Motor
}

internal struct JointConstraintRow2D
{
    public JointConstraintRow2D(
        JointConstraintRowKind2D kind,
        Vector2d axis,
        Vector2d relativeAnchorA,
        Vector2d relativeAnchorB,
        Fixed64 biasVelocity,
        Fixed64 damping,
        Fixed64 lowerImpulse,
        Fixed64 upperImpulse,
        int cacheIndex)
    {
        Kind = kind;
        Axis = axis;
        RelativeAnchorA = relativeAnchorA;
        RelativeAnchorB = relativeAnchorB;
        BiasVelocity = biasVelocity;
        Damping = damping;
        LowerImpulse = lowerImpulse;
        UpperImpulse = upperImpulse;
        CacheIndex = cacheIndex;
        AccumulatedImpulse = Fixed64.Zero;
    }

    public JointConstraintRowKind2D Kind;
    public Vector2d Axis;
    public Vector2d RelativeAnchorA;
    public Vector2d RelativeAnchorB;
    public Fixed64 BiasVelocity;
    public Fixed64 Damping;
    public Fixed64 LowerImpulse;
    public Fixed64 UpperImpulse;
    public int CacheIndex;
    public Fixed64 AccumulatedImpulse;
}
