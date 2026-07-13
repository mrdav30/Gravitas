//=======================================================================
// MeshCheckedMath.cs
//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp;

namespace Gravitas.Colliders;

internal static class MeshCheckedMath
{
    internal static bool TryCreateParallelAxisTensor(Vector3d offset, out Fixed3x3 tensor)
    {
        tensor = default;
        bool valid = TryMultiply(offset.X, offset.X, out Fixed64 xx);
        valid &= TryMultiply(offset.Y, offset.Y, out Fixed64 yy);
        valid &= TryMultiply(offset.Z, offset.Z, out Fixed64 zz);
        valid &= TryMultiply(offset.X, offset.Y, out Fixed64 xyProduct);
        valid &= TryMultiply(offset.X, offset.Z, out Fixed64 xzProduct);
        valid &= TryMultiply(offset.Y, offset.Z, out Fixed64 yzProduct);
        valid &= TryAdd(yy, zz, out Fixed64 diagonalX);
        valid &= TryAdd(xx, zz, out Fixed64 diagonalY);
        valid &= TryAdd(xx, yy, out Fixed64 diagonalZ);
        valid &= TryNegate(xyProduct, out Fixed64 xy);
        valid &= TryNegate(xzProduct, out Fixed64 xz);
        valid &= TryNegate(yzProduct, out Fixed64 yz);
        if (!valid)
            return false;

        tensor = new Fixed3x3(diagonalX, xy, xz, xy, diagonalY, yz, xz, yz, diagonalZ);
        return true;
    }

    internal static bool TryAdd(Vector3d first, Vector3d second, out Vector3d result) =>
        TryCreateVector(first.X + second.X, first.Y + second.Y, first.Z + second.Z, out result);

    internal static bool TrySubtract(Vector3d first, Vector3d second, out Vector3d result) =>
        TryCreateVector(first.X - second.X, first.Y - second.Y, first.Z - second.Z, out result);

    internal static bool TryMultiply(Vector3d first, Vector3d second, out Vector3d result)
    {
        result = default;
        bool valid = TryMultiply(first.X, second.X, out Fixed64 x);
        valid &= TryMultiply(first.Y, second.Y, out Fixed64 y);
        valid &= TryMultiply(first.Z, second.Z, out Fixed64 z);
        if (!valid)
            return false;

        result = new Vector3d(x, y, z);
        return true;
    }

    internal static bool TryMultiply(Vector3d value, Fixed64 scalar, out Vector3d result) =>
        TryCreateVector(value.X * scalar, value.Y * scalar, value.Z * scalar, out result);

    internal static bool TryDivide(Vector3d value, Fixed64 divisor, out Vector3d result) =>
        TryCreateVector(value.X / divisor, value.Y / divisor, value.Z / divisor, out result);

    internal static bool TryAdd(Fixed3x3 first, Fixed3x3 second, out Fixed3x3 result) =>
        TryCreateMatrix(first, second, add: true, out result);

    internal static bool TrySubtract(Fixed3x3 first, Fixed3x3 second, out Fixed3x3 result) =>
        TryCreateMatrix(first, second, add: false, out result);

    internal static bool TryMultiply(Fixed3x3 value, Fixed64 scalar, out Fixed3x3 result)
    {
        result = default;
        Fixed3x3 product = value * scalar;
        if (!IsRepresentable(product))
            return false;

        result = product;
        return true;
    }

    internal static bool TryAdd(Fixed64 first, Fixed64 second, out Fixed64 result)
    {
        result = first + second;
        return IsRepresentable(result);
    }

    internal static bool TrySubtract(Fixed64 first, Fixed64 second, out Fixed64 result)
    {
        result = first - second;
        return IsRepresentable(result);
    }

    internal static bool TryMultiply(Fixed64 first, Fixed64 second, out Fixed64 result)
    {
        result = first * second;
        return IsRepresentable(result);
    }

    internal static bool TryNegate(Fixed64 value, out Fixed64 result)
    {
        result = -value;
        return IsRepresentable(result);
    }

    internal static bool IsRepresentable(Vector3d value) =>
        IsRepresentable(value.X) & IsRepresentable(value.Y) & IsRepresentable(value.Z);

    internal static bool IsRepresentable(Fixed3x3 value) =>
        IsRepresentable(value.M11) & IsRepresentable(value.M12) & IsRepresentable(value.M13)
        & IsRepresentable(value.M21) & IsRepresentable(value.M22) & IsRepresentable(value.M23)
        & IsRepresentable(value.M31) & IsRepresentable(value.M32) & IsRepresentable(value.M33);

    internal static bool IsRepresentable(Fixed64 value) =>
        value != Fixed64.MinValue & value != Fixed64.MaxValue;

    private static bool TryCreateVector(Fixed64 x, Fixed64 y, Fixed64 z, out Vector3d result)
    {
        result = default;
        if (!(IsRepresentable(x) & IsRepresentable(y) & IsRepresentable(z)))
            return false;

        result = new Vector3d(x, y, z);
        return true;
    }

    private static bool TryCreateMatrix(Fixed3x3 first, Fixed3x3 second, bool add, out Fixed3x3 result)
    {
        result = default;
        bool valid = TryCombine(first.M11, second.M11, add, out Fixed64 m11);
        valid &= TryCombine(first.M12, second.M12, add, out Fixed64 m12);
        valid &= TryCombine(first.M13, second.M13, add, out Fixed64 m13);
        valid &= TryCombine(first.M21, second.M21, add, out Fixed64 m21);
        valid &= TryCombine(first.M22, second.M22, add, out Fixed64 m22);
        valid &= TryCombine(first.M23, second.M23, add, out Fixed64 m23);
        valid &= TryCombine(first.M31, second.M31, add, out Fixed64 m31);
        valid &= TryCombine(first.M32, second.M32, add, out Fixed64 m32);
        valid &= TryCombine(first.M33, second.M33, add, out Fixed64 m33);
        if (!valid)
            return false;

        result = new Fixed3x3(m11, m12, m13, m21, m22, m23, m31, m32, m33);
        return true;
    }

    private static bool TryCombine(Fixed64 first, Fixed64 second, bool add, out Fixed64 result) =>
        add ? TryAdd(first, second, out result) : TrySubtract(first, second, out result);
}
