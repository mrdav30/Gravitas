//=======================================================================
// CollisionDetectionMixed.Support.cs
//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp;
using FixedMathSharp.Geometry;
using Gravitas.Colliders;
using System.Runtime.CompilerServices;

namespace Gravitas.CollisionHandling;

public static partial class CollisionDetectionMixed
{
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static bool TryNormalizeAxis(
        Vector3d axis,
        out Vector3d normalizedAxis)
    {
        Fixed64 magnitudeSqr = axis.MagnitudeSquared;
        if (magnitudeSqr <= Fixed64.Epsilon)
        {
            normalizedAxis = Vector3d.Zero;
            return false;
        }

        normalizedAxis = axis.Normalized;
        return true;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static Vector3d GetEmbeddedCenter3D(LSCollider2D embedded) =>
        MixedEmbedded2DGeometry.GetCenter3D(embedded);

    private static void GetEmbeddedCapsuleAxes(
        LSCapsuleCollider2D capsule,
        out Vector2d planarAxis,
        out Vector3d axis,
        out Vector3d normal)
    {
        if (capsule.AxisLength <= Fixed64.Epsilon)
        {
            planarAxis = Vector2d.Forward;
            axis = Vector3d.Zero;
            normal = Vector3d.Zero;
            return;
        }

        planarAxis = new Vector2d(
            -FixedMath.Sin(capsule.Rotation),
            FixedMath.Cos(capsule.Rotation));
        axis = new Vector3d(
            planarAxis.X,
            Fixed64.Zero,
            planarAxis.Y);
        Vector2d planarNormal = planarAxis.RightHandNormal;
        normal = new Vector3d(
            planarNormal.X,
            Fixed64.Zero,
            planarNormal.Y);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static Vector3d GetRigidUpAxis(FixedQuaternion rotation) =>
        (rotation * Vector3d.Up).Normalized;

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static Vector3d ResolveFallbackNormal(
        Vector3d sphereCenter,
        LSCollider2D embedded)
    {
        Vector3d fallback = new(
            embedded.Center.X - sphereCenter.X,
            embedded.MixedSlabCenterY - sphereCenter.Y,
            embedded.Center.Y - sphereCenter.Z);
        return fallback.MagnitudeSquared > Fixed64.Epsilon
            ? fallback.Normalized
            : Vector3d.Right;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static bool NoContact(out MixedContact contact)
    {
        contact = default;
        return false;
    }
}
