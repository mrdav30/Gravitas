//=======================================================================
// ExactCoulombResponse3D.cs
//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp;

namespace Gravitas.CollisionHandling;

/// <summary>
/// Contains atomically materialized velocity deltas from an exact Coulomb
/// response.
/// </summary>
internal readonly struct ExactCoulombResponse3D
{
    private readonly Fixed64 _primaryAccumulatedImpulse;
    private readonly Fixed64 _secondaryAccumulatedImpulse;
    private readonly byte _projectionFlags;

    internal ExactCoulombResponse3D(
        bool hasAppliedImpulse,
        Vector3d firstLinearVelocityDelta,
        Vector3d firstAngularVelocityDelta,
        Vector3d secondLinearVelocityDelta,
        Vector3d secondAngularVelocityDelta,
        bool hasPrimaryAccumulatedImpulse,
        Fixed64 primaryAccumulatedImpulse,
        bool hasSecondaryAccumulatedImpulse,
        Fixed64 secondaryAccumulatedImpulse)
    {
        HasAppliedImpulse = hasAppliedImpulse;
        FirstLinearVelocityDelta = firstLinearVelocityDelta;
        FirstAngularVelocityDelta = firstAngularVelocityDelta;
        SecondLinearVelocityDelta = secondLinearVelocityDelta;
        SecondAngularVelocityDelta = secondAngularVelocityDelta;
        _primaryAccumulatedImpulse = primaryAccumulatedImpulse;
        _secondaryAccumulatedImpulse = secondaryAccumulatedImpulse;
        _projectionFlags = (byte)(
            (hasPrimaryAccumulatedImpulse ? 1 : 0)
            | (hasSecondaryAccumulatedImpulse ? 2 : 0));
    }

    internal bool HasAppliedImpulse { get; }

    internal Vector3d FirstLinearVelocityDelta { get; }

    internal Vector3d FirstAngularVelocityDelta { get; }

    internal Vector3d SecondLinearVelocityDelta { get; }

    internal Vector3d SecondAngularVelocityDelta { get; }

    internal bool TryGetPrimaryAccumulatedImpulse(out Fixed64 value)
    {
        value = _primaryAccumulatedImpulse;
        return (_projectionFlags & 1) != 0;
    }

    internal bool TryGetSecondaryAccumulatedImpulse(out Fixed64 value)
    {
        value = _secondaryAccumulatedImpulse;
        return (_projectionFlags & 2) != 0;
    }
}
