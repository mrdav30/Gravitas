//=======================================================================
// ColliderScalePolicy.cs
//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp;

namespace Gravitas.Colliders;

/// <summary>
/// Owns Gravitas's physical-scale admission boundary. FixedTransform permits
/// signed and zero authored scale; collider geometry requires positive authored
/// ancestry and positive resulting world dimensions on every axis it consumes.
/// </summary>
internal static class ColliderScalePolicy
{
    internal static Vector2d ValidatePlanar(FixedTransform transform)
    {
        FixedTransform? current = transform;
        while (current != null)
        {
            Validate(current.LocalScaleXZ);
            current = current.Parent;
        }

        Vector2d worldScale = transform.LossyScale.ToVector2d();
        Validate(worldScale);
        return worldScale;
    }

    internal static Vector3d Validate(FixedTransform transform)
    {
        FixedTransform? current = transform;
        while (current != null)
        {
            Validate(current.LocalScale);
            current = current.Parent;
        }

        Vector3d worldScale = transform.LossyScale;
        Validate(worldScale);
        return worldScale;
    }

    internal static void Validate(Vector2d scale)
    {
        SwiftThrowHelper.ThrowIfArgument(
            scale.X <= Fixed64.Zero || scale.Y <= Fixed64.Zero,
            nameof(scale),
            "2D collider scale must be greater than zero on the X and Z axes throughout its transform ancestry.");
    }

    internal static void Validate(Vector3d scale)
    {
        SwiftThrowHelper.ThrowIfArgument(
            scale.X <= Fixed64.Zero || scale.Y <= Fixed64.Zero || scale.Z <= Fixed64.Zero,
            nameof(scale),
            "3D collider scale must be greater than zero on every axis throughout its transform ancestry.");
    }
}
