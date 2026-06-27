//=======================================================================
// JointConstraintRow3D.cs
//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp;

namespace Gravitas.Constraints;

internal enum JointConstraintRowKind3D : byte
{
    Linear,
    Angular,
    Motor
}

internal struct JointConstraintRow3D
{
    public JointConstraintRow3D(
        JointConstraintRowKind3D kind,
        Vector3d axis,
        Vector3d relativeAnchorA,
        Vector3d relativeAnchorB,
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

    public JointConstraintRowKind3D Kind;
    public Vector3d Axis;
    public Vector3d RelativeAnchorA;
    public Vector3d RelativeAnchorB;
    public Fixed64 BiasVelocity;
    public Fixed64 Damping;
    public Fixed64 LowerImpulse;
    public Fixed64 UpperImpulse;
    public int CacheIndex;
    public Fixed64 AccumulatedImpulse;
}
