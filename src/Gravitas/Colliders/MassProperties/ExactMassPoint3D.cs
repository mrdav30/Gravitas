//=======================================================================
// ExactMassPoint3D.cs
//=======================================================================
// MIT License, Copyright (c) 2024-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using System;

using FixedMathSharp;

namespace Gravitas.Colliders;

/// <summary>
/// Represents a 3D mass-property point that may lie outside the scalar
/// coordinate domain while remaining available to aggregate operations.
/// </summary>
internal readonly struct ExactMassPoint3D
{
    internal readonly Signed320 XNumerator;
    internal readonly Signed320 YNumerator;
    internal readonly Signed320 ZNumerator;

    internal ExactMassPoint3D(
        Signed320 xNumerator,
        Signed320 yNumerator,
        Signed320 zNumerator)
    {
        XNumerator = xNumerator;
        YNumerator = yNumerator;
        ZNumerator = zNumerator;
    }

    /// <summary>Creates a mass point from a representable vector.</summary>
    internal static ExactMassPoint3D FromPoint(Vector3d point) =>
        ExactMassProperties.CreatePoint(point);

    /// <summary>
    /// Creates the scale-invariant quaternion composition
    /// <c>outerPoint * outerScale + innerOffset * innerScale +
    /// innerRotation * innerDisplacement</c>.
    /// </summary>
    /// <remarks>
    /// The quaternion rotation uses the same rational basis as full-domain
    /// geometry and retains 64 fractional guard bits for later aggregation.
    /// </remarks>
    /// <exception cref="ArgumentException">
    /// <paramref name="innerRotation"/> is not normalized.
    /// </exception>
    internal static ExactMassPoint3D CreateScaledLocalComposition(
        Vector3d outerPoint,
        Vector3d outerScale,
        Vector3d innerOffset,
        Vector3d innerScale,
        Vector3d innerDisplacement,
        FixedQuaternion innerRotation)
    {
        if (!innerRotation.IsNormalized())
        {
            throw new ArgumentException(
                "The mass-point rotation must be normalized.",
                nameof(innerRotation));
        }

        return ExactMassProperties.CreatePoint(
            outerPoint,
            outerScale,
            innerOffset,
            innerScale,
            innerDisplacement,
            innerRotation);
    }

    /// <summary>Attempts to materialize this point as a Q32.32 vector.</summary>
    internal bool TryGetPoint(out Vector3d point) =>
        ExactMassProperties.TryGetPoint(this, out point);

    /// <summary>
    /// Attempts to obtain the exact positive-weight average of semantic mass
    /// points with one final conversion per component.
    /// </summary>
    internal static bool TryGetWeightedAverage(
        ReadOnlySpan<ExactMassPoint3D> points,
        ReadOnlySpan<ExactMassWeight> weights,
        out Vector3d average) =>
        ExactMassProperties.TryGetWeightedAverage(
            points,
            weights,
            out average);

    /// <summary>
    /// Attempts to add this point's parallel-axis contribution to a tensor
    /// about <paramref name="referencePoint"/>.
    /// </summary>
    internal bool TryAddParallelAxisTensor(
        Fixed3x3 centerTensor,
        Fixed64 mass,
        Vector3d referencePoint,
        out Fixed3x3 tensor) =>
        ExactMassProperties.TryAddParallelAxisTensor(
            this,
            centerTensor,
            mass,
            referencePoint,
            out tensor);
}
