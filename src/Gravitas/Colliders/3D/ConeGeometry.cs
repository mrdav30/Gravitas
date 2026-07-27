//=======================================================================
// ConeGeometry.cs
//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp;
using FixedMathSharp.Geometry;
using System.Runtime.CompilerServices;

namespace Gravitas.Colliders;

/// <summary>
/// Shared deterministic finite-cone geometry helpers.
/// </summary>
internal static class ConeGeometry
{
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static void CreateFiniteConeBounds(
        Vector3d apex,
        Vector3d baseCenter,
        Vector3d axis,
        Fixed64 baseRadius,
        out Vector3d min,
        out Vector3d max)
    {
        Vector3d normal = axis.MagnitudeSquared > Fixed64.Epsilon
            ? axis.Normalized
            : Vector3d.Up;
        FixedBoundBox bounds = FixedBoundBox.FromFiniteConeClippedToDomain(
            apex,
            baseCenter,
            normal,
            baseRadius);
        min = bounds.Min;
        max = bounds.Max;
    }
}
