// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.

using FixedMathSharp;
using FixedMathSharp.Geometry;
using System;

namespace Gravitas.Colliders;

/// <summary>Defines the supported effective geometry envelope, independently of world position.</summary>
internal static class ColliderGeometryPolicy
{
    internal static readonly Fixed64 MaximumGeometryReach = (Fixed64)4096;

    internal static void ValidateReach(Fixed64 reach, string parameterName) =>
        ChronicleGuard.ThrowIfArgument(reach < Fixed64.Zero || reach > MaximumGeometryReach,
            parameterName, "Effective geometry reach must not exceed 4,096 world units.");

    internal static void ValidateReach(Vector2d envelope, string parameterName) =>
        ChronicleGuard.ThrowIfArgument(Vector2d.CompareMagnitudeSquared(
                envelope, new Vector2d(MaximumGeometryReach, Fixed64.Zero)) > 0,
            parameterName, "Effective geometry reach must not exceed 4,096 world units.");

    internal static void ValidateFiniteAxisReach(Fixed64 radius, Fixed64 height, string parameterName) =>
        ChronicleGuard.ThrowIfArgument(Fixed64.CompareMagnitudeSquared(
                radius, HalfCeiling(height), radius, Fixed64.Zero,
                MaximumGeometryReach, Fixed64.Zero, Fixed64.Zero, Fixed64.Zero) > 0,
            parameterName, "Effective finite-axis geometry reach must not exceed 4,096 world units.");

    internal static void ValidateWorldPolygonReach(ReadOnlySpan<Vector2d> vertices)
    {
        Signed192 sumX = default, sumY = default;
        for (int i = 0; i < vertices.Length; i++)
        {
            sumX = WideArithmetic.AddSigned192(sumX, Signed192.Raw(vertices[i].X));
            sumY = WideArithmetic.AddSigned192(sumY, Signed192.Raw(vertices[i].Y));
        }
        // Retain n*v - sum(v): a rounded mean can hide an excess at the cutoff.
        // Span counts keep these numerators below 95 bits and their squared sum below 191.
        Fixed64 countRaw = Fixed64.FromRaw(vertices.Length);
        Signed192 limit = Fixed64.GetExactRawProduct(MaximumGeometryReach, countRaw);
        Signed320 limitSquared = WideArithmetic.MultiplySigned192(limit, limit);
        for (int i = 0; i < vertices.Length; i++)
        {
            Signed192 x = WideArithmetic.SubtractSigned192(
                Fixed64.GetExactRawProduct(vertices[i].X, countRaw), sumX);
            Signed192 y = WideArithmetic.SubtractSigned192(
                Fixed64.GetExactRawProduct(vertices[i].Y, countRaw), sumY);
            ChronicleGuard.ThrowIfArgument(WideArithmetic.CompareMagnitude(
                    WideArithmetic.AddProducts(x, x, y, y), limitSquared) > 0,
                nameof(vertices), "Effective polygon geometry reach must not exceed 4,096 world units.");
        }
    }

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

    internal static Fixed64 RotatedReach(Fixed64 reach, bool planar)
    {
        if (reach == Fixed64.Zero)
            return Fixed64.Zero;

        // The rational 3D basis preserves norm; final component rounding adds
        // less than one raw unit to length. Planar sin/cos has operator norm
        // at most 1 + 2e; its scalar product rounding needs another half raw.
        if (planar)
            reach = Fixed64.MultiplyDivide(reach,
                Fixed64.One + Fixed64.Two * FixedMath.CanonicalSinCosErrorBound,
                Fixed64.One, out _);
        return Add(reach, Fixed64.FromRaw(planar ? 2 : 1));
    }

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
