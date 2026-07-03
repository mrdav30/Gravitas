//=======================================================================
// ConeGeometry.cs
//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp;
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
        Vector3d baseExtents = CreateBaseDiskExtents(normal, baseRadius);
        min = Vector3d.Min(apex, baseCenter - baseExtents);
        max = Vector3d.Max(apex, baseCenter + baseExtents);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static Vector3d CreateBaseDiskExtents(Vector3d axis, Fixed64 radius) =>
        new(
            GetDiskAxisExtent(axis.X, radius),
            GetDiskAxisExtent(axis.Y, radius),
            GetDiskAxisExtent(axis.Z, radius));

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static Fixed64 GetDiskAxisExtent(Fixed64 axisComponent, Fixed64 radius)
    {
        Fixed64 capacitySqr = Fixed64.One - axisComponent * axisComponent;
        if (capacitySqr <= Fixed64.Zero)
            return Fixed64.Zero;

        return radius * FixedMath.Sqrt(capacitySqr);
    }
}
