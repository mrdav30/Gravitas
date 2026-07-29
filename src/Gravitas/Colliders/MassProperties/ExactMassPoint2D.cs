//=======================================================================
// ExactMassPoint2D.cs
//=======================================================================
// MIT License, Copyright (c) 2024-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using System;

using FixedMathSharp;

namespace Gravitas.Colliders;

/// <summary>
/// Represents a 2D mass-property point that may lie outside the scalar
/// coordinate domain while remaining available to aggregate operations.
/// </summary>
internal readonly struct ExactMassPoint2D
{
    internal readonly Signed320 XNumerator;
    internal readonly Signed320 YNumerator;

    internal ExactMassPoint2D(
        Signed320 xNumerator,
        Signed320 yNumerator)
    {
        XNumerator = xNumerator;
        YNumerator = yNumerator;
    }

    /// <summary>Creates a mass point from a representable vector.</summary>
    internal static ExactMassPoint2D FromPoint(Vector2d point) =>
        ExactMassProperties.CreatePoint(point);

    /// <summary>
    /// Creates the exact planar composition
    /// <c>outerPoint * outerScale + innerOffset * innerScale +
    /// Rotate(innerDisplacement, innerRotation)</c>.
    /// </summary>
    internal static ExactMassPoint2D CreateScaledLocalComposition(
        Vector2d outerPoint,
        Vector2d outerScale,
        Vector2d innerOffset,
        Vector2d innerScale,
        Vector2d innerDisplacement,
        Fixed64 innerRotation) =>
        ExactMassProperties.CreatePoint(
            outerPoint,
            outerScale,
            innerOffset,
            innerScale,
            innerDisplacement,
            innerRotation);

    /// <summary>Attempts to materialize this point as a Q32.32 vector.</summary>
    internal bool TryGetPoint(out Vector2d point) =>
        ExactMassProperties.TryGetPoint(this, out point);

    /// <summary>
    /// Attempts to obtain the exact positive-weight average of semantic mass
    /// points with one final conversion per component.
    /// </summary>
    internal static bool TryGetWeightedAverage(
        ReadOnlySpan<ExactMassPoint2D> points,
        ReadOnlySpan<ExactMassWeight> weights,
        out Vector2d average) =>
        ExactMassProperties.TryGetWeightedAverage(
            points,
            weights,
            out average);

    /// <summary>
    /// Attempts to add this point's parallel-axis contribution to a scalar
    /// moment about <paramref name="referencePoint"/>.
    /// </summary>
    internal bool TryAddParallelAxisMoment(
        Fixed64 centerMoment,
        Fixed64 mass,
        Vector2d referencePoint,
        out Fixed64 moment) =>
        ExactMassProperties.TryAddParallelAxisMoment(
            this,
            centerMoment,
            mass,
            referencePoint,
            out moment);
}
