//=======================================================================
// InertiaTensorMath.cs
//=======================================================================
// MIT License, Copyright (c) 2026–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp;
using System.Runtime.CompilerServices;

namespace Gravitas;

/// <summary>
/// Deterministic helpers for 3D inertia tensors used by body mass properties
/// and solver effective mass calculations.
/// </summary>
internal static class InertiaTensorMath
{
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static bool IsDiagonal(Fixed3x3 tensor) =>
        tensor.M12 == Fixed64.Zero
        && tensor.M13 == Fixed64.Zero
        && tensor.M21 == Fixed64.Zero
        && tensor.M23 == Fixed64.Zero
        && tensor.M31 == Fixed64.Zero
        && tensor.M32 == Fixed64.Zero;

    public static Fixed3x3 InvertForSolver(Fixed3x3 tensor)
    {
        if (tensor == Fixed3x3.Zero)
            return Fixed3x3.Zero;

        if (IsDiagonal(tensor))
            return InvertDiagonalForSolver(tensor);

        if (!Fixed3x3.Invert(tensor, out Fixed3x3? inverse) || !inverse.HasValue)
            return Fixed3x3.Zero;

        return ClampNearZero(inverse.Value);
    }

    public static Fixed3x3 AddParallelAxisTensor(Fixed3x3 tensor, Fixed64 mass, Vector3d offset)
    {
        if (mass <= Fixed64.Zero || offset == Vector3d.Zero)
            return tensor;

        Fixed64 xx = mass * ((offset.Y * offset.Y) + (offset.Z * offset.Z));
        Fixed64 yy = mass * ((offset.X * offset.X) + (offset.Z * offset.Z));
        Fixed64 zz = mass * ((offset.X * offset.X) + (offset.Y * offset.Y));
        Fixed64 xy = mass * offset.X * offset.Y;
        Fixed64 xz = mass * offset.X * offset.Z;
        Fixed64 yz = mass * offset.Y * offset.Z;

        tensor.M11 += xx;
        tensor.M22 += yy;
        tensor.M33 += zz;
        tensor.M12 -= xy;
        tensor.M21 -= xy;
        tensor.M13 -= xz;
        tensor.M31 -= xz;
        tensor.M23 -= yz;
        tensor.M32 -= yz;
        return ClampNearZero(tensor);
    }

    public static Fixed3x3 SubtractParallelAxisTensor(Fixed3x3 tensor, Fixed64 mass, Vector3d offset)
    {
        if (mass <= Fixed64.Zero || offset == Vector3d.Zero)
            return tensor;

        Fixed64 xx = mass * ((offset.Y * offset.Y) + (offset.Z * offset.Z));
        Fixed64 yy = mass * ((offset.X * offset.X) + (offset.Z * offset.Z));
        Fixed64 zz = mass * ((offset.X * offset.X) + (offset.Y * offset.Y));
        Fixed64 xy = mass * offset.X * offset.Y;
        Fixed64 xz = mass * offset.X * offset.Z;
        Fixed64 yz = mass * offset.Y * offset.Z;

        tensor.M11 -= xx;
        tensor.M22 -= yy;
        tensor.M33 -= zz;
        tensor.M12 += xy;
        tensor.M21 += xy;
        tensor.M13 += xz;
        tensor.M31 += xz;
        tensor.M23 += yz;
        tensor.M32 += yz;
        return ClampNearZero(tensor);
    }

    public static Fixed3x3 RotateToFrame(Fixed3x3 tensor, FixedQuaternion rotation)
    {
        if (rotation == FixedQuaternion.Identity)
            return tensor;

        Fixed3x3 rotationMatrix = rotation.ToMatrix3x3();
        return ClampNearZero(rotationMatrix * tensor * rotationMatrix.Transpose());
    }

    private static Fixed3x3 InvertDiagonalForSolver(Fixed3x3 tensor) =>
        new(
            tensor.M11 > Fixed64.Zero ? Fixed64.One / tensor.M11 : Fixed64.Zero,
            Fixed64.Zero,
            Fixed64.Zero,
            Fixed64.Zero,
            tensor.M22 > Fixed64.Zero ? Fixed64.One / tensor.M22 : Fixed64.Zero,
            Fixed64.Zero,
            Fixed64.Zero,
            Fixed64.Zero,
            tensor.M33 > Fixed64.Zero ? Fixed64.One / tensor.M33 : Fixed64.Zero);

    private static Fixed3x3 ClampNearZero(Fixed3x3 tensor)
    {
        tensor.M11 = ClampNearZero(tensor.M11);
        tensor.M12 = ClampNearZero(tensor.M12);
        tensor.M13 = ClampNearZero(tensor.M13);
        tensor.M21 = ClampNearZero(tensor.M21);
        tensor.M22 = ClampNearZero(tensor.M22);
        tensor.M23 = ClampNearZero(tensor.M23);
        tensor.M31 = ClampNearZero(tensor.M31);
        tensor.M32 = ClampNearZero(tensor.M32);
        tensor.M33 = ClampNearZero(tensor.M33);
        return tensor;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static Fixed64 ClampNearZero(Fixed64 value) =>
        value.Abs() <= Fixed64.Epsilon ? Fixed64.Zero : value;
}
