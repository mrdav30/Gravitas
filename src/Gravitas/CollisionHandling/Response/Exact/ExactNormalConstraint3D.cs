//=======================================================================
// ExactNormalConstraint3D.cs
//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp;

namespace Gravitas.CollisionHandling;

/// <summary>
/// Describes the normal constraint that supplies a Coulomb response.
/// </summary>
internal readonly struct ExactNormalConstraint3D
{
    internal ExactNormalConstraint3D(
        in ExactContactResponseOperand3D first,
        in ExactContactResponseOperand3D second,
        Vector3d normal,
        Fixed64 restitution,
        Fixed64 restitutionVelocityThreshold,
        Fixed64 accumulatedImpulse,
        Fixed64 positiveImpulseScale,
        Fixed64 negativeImpulseScale)
    {
        First = first;
        Second = second;
        Normal = normal;
        Restitution = restitution;
        RestitutionVelocityThreshold = restitutionVelocityThreshold;
        AccumulatedImpulse = accumulatedImpulse;
        PositiveImpulseScale = positiveImpulseScale;
        NegativeImpulseScale = negativeImpulseScale;
    }

    internal ExactContactResponseOperand3D First { get; }

    internal ExactContactResponseOperand3D Second { get; }

    internal Vector3d Normal { get; }

    internal Fixed64 Restitution { get; }

    internal Fixed64 RestitutionVelocityThreshold { get; }

    internal Fixed64 AccumulatedImpulse { get; }

    internal Fixed64 PositiveImpulseScale { get; }

    internal Fixed64 NegativeImpulseScale { get; }
}
