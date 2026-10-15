// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.

using FixedMathSharp;
using FixedMathSharp.Geometry;

namespace Gravitas.Colliders;

/// <summary>Defines the supported effective geometry envelope, independently of world position.</summary>
internal static class ColliderGeometryPolicy
{
    internal static readonly Fixed64 MaximumGeometryReach = (Fixed64)4096;

    internal static void ValidateReach(Fixed64 reach, string parameterName) =>
        SwiftThrowHelper.ThrowIfArgument(reach < Fixed64.Zero || reach > MaximumGeometryReach,
            parameterName, "Effective geometry reach must not exceed 4,096 world units.");

    internal static Fixed64 Add(Fixed64 attachment, Fixed64 intrinsic) =>
        Fixed64.TryAdd(attachment, intrinsic, out Fixed64 reach) ? reach : Fixed64.MaxValue;

    internal static Fixed64 Magnitude(Vector3d vector) =>
        vector.TryGetMagnitudeCeiling(out Fixed64 reach) ? reach : Fixed64.MaxValue;

    internal static Fixed64 Magnitude(Vector2d vector) =>
        vector.TryGetMagnitudeCeiling(out Fixed64 reach) ? reach : Fixed64.MaxValue;

    internal static Fixed64 Attachment(Vector3d offset, Vector3d scale) =>
        offset == Vector3d.Zero ? Fixed64.Zero
            : WideGeometry.TryGetScaledMagnitudeCeiling(offset, scale, out Fixed64 reach)
                ? reach : Fixed64.MaxValue;

    internal static Fixed64 Attachment(Vector2d offset, Vector2d scale) =>
        Attachment(new Vector3d(offset.X, offset.Y, Fixed64.Zero),
            new Vector3d(scale.X, scale.Y, Fixed64.One));

    // Dividing raw lengths outward preserves the half-raw endpoint of an odd core/height.
    internal static Fixed64 Capsule(Fixed64 radius, Fixed64 coreLength) =>
        Add(radius, HalfCeiling(coreLength));

    internal static Fixed64 FiniteAxis(Fixed64 radius, Fixed64 height) =>
        Magnitude(new Vector3d(radius, HalfCeiling(height), radius));

    internal static Fixed64 Mixed(Fixed64 planarReach, Fixed64 halfThickness) =>
        Magnitude(new Vector2d(planarReach, halfThickness));

    private static Fixed64 HalfCeiling(Fixed64 length) =>
        Fixed64.FromRaw((length.m_rawValue >> 1) + (length.m_rawValue & 1));
}
