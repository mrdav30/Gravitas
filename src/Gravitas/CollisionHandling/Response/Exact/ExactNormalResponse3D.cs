//=======================================================================
// ExactNormalResponse3D.cs
//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp;

namespace Gravitas.CollisionHandling;

/// <summary>
/// Contains atomically materialized velocity deltas from an exact normal
/// response.
/// </summary>
internal readonly struct ExactNormalResponse3D
{
    private readonly Fixed64 _normalVelocity;
    private readonly Fixed64 _appliedImpulse;
    private readonly Fixed64 _accumulatedImpulse;
    private readonly byte _projectionFlags;

    internal ExactNormalResponse3D(
        bool isClosing,
        bool hasAppliedImpulse,
        Vector3d firstLinearVelocityDelta,
        Vector3d firstAngularVelocityDelta,
        Vector3d secondLinearVelocityDelta,
        Vector3d secondAngularVelocityDelta,
        bool hasNormalVelocity,
        Fixed64 normalVelocity,
        bool hasAppliedImpulseProjection,
        Fixed64 appliedImpulse,
        bool hasAccumulatedImpulse,
        Fixed64 accumulatedImpulse)
    {
        IsClosing = isClosing;
        HasAppliedImpulse = hasAppliedImpulse;
        FirstLinearVelocityDelta = firstLinearVelocityDelta;
        FirstAngularVelocityDelta = firstAngularVelocityDelta;
        SecondLinearVelocityDelta = secondLinearVelocityDelta;
        SecondAngularVelocityDelta = secondAngularVelocityDelta;
        _normalVelocity = normalVelocity;
        _appliedImpulse = appliedImpulse;
        _accumulatedImpulse = accumulatedImpulse;
        _projectionFlags = (byte)(
            (hasNormalVelocity ? 1 : 0)
            | (hasAppliedImpulseProjection ? 2 : 0)
            | (hasAccumulatedImpulse ? 4 : 0));
    }

    internal bool IsClosing { get; }

    internal bool HasAppliedImpulse { get; }

    internal Vector3d FirstLinearVelocityDelta { get; }

    internal Vector3d FirstAngularVelocityDelta { get; }

    internal Vector3d SecondLinearVelocityDelta { get; }

    internal Vector3d SecondAngularVelocityDelta { get; }

    internal bool TryGetNormalVelocity(out Fixed64 value)
    {
        value = _normalVelocity;
        return (_projectionFlags & 1) != 0;
    }

    internal bool TryGetAppliedImpulse(out Fixed64 value)
    {
        value = _appliedImpulse;
        return (_projectionFlags & 2) != 0;
    }

    internal bool TryGetAccumulatedImpulse(out Fixed64 value)
    {
        value = _accumulatedImpulse;
        return (_projectionFlags & 4) != 0;
    }
}
