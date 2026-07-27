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
    internal static Vector2d CapturePlanar(
        FixedTransform transform,
        out Fixed4x4 worldMatrix,
        out Fixed64 worldRotation)
    {
        SwiftThrowHelper.ThrowIfNull(transform, nameof(transform));
        FixedTransform? current = transform;
        while (current != null)
        {
            Validate(current.LocalScaleXZ);
            current = current.Parent;
        }

        Vector2d worldScale = default;
        worldRotation = default;
        bool captured = transform.TryGetLocalToWorldMatrix(out worldMatrix)
            && TryExtractPlanarTransform(
                worldMatrix,
                out worldScale,
                out worldRotation);
        SwiftThrowHelper.ThrowIfArgument(
            !captured,
            nameof(transform),
            "2D collider transform hierarchy must compose to a representable, nonsheared X/Z transform.");
        Validate(worldScale);
        return worldScale;
    }

    internal static Vector3d Capture(
        FixedTransform transform,
        out Fixed4x4 worldMatrix,
        out FixedQuaternion worldRotation)
    {
        SwiftThrowHelper.ThrowIfNull(transform, nameof(transform));
        ValidateAncestry(transform);

        Vector3d worldScale = default;
        worldRotation = default;
        bool captured = transform.TryGetLocalToWorldMatrix(out worldMatrix)
            && Fixed4x4.Decompose(
                worldMatrix,
                out _,
                out worldRotation,
                out worldScale);
        SwiftThrowHelper.ThrowIfArgument(
            !captured,
            nameof(transform),
            "Collider transform hierarchy must compose to a representable, nonsheared transform.");
        Validate(worldScale);
        return worldScale;
    }

    internal static Vector3d CaptureScale(FixedTransform transform)
    {
        ValidateAncestry(transform);
        if (transform.Parent == null)
            return transform.LocalScale;

        Vector3d worldScale = default;
        bool capturedMatrix =
            transform.TryGetLocalToWorldMatrix(out Fixed4x4 worldMatrix);
        bool capturedScale = Fixed4x4.Decompose(
            worldMatrix,
            out _,
            out _,
            out worldScale);
        SwiftThrowHelper.ThrowIfArgument(
            !(capturedMatrix & capturedScale),
            nameof(transform),
            "Collider transform hierarchy must compose to a representable, nonsheared transform.");
        Validate(worldScale);
        return worldScale;
    }

    private static void ValidateAncestry(FixedTransform transform)
    {
        FixedTransform? current = transform;
        while (current != null)
        {
            Validate(current.LocalScale);
            current = current.Parent;
        }
    }

    private static bool TryExtractPlanarTransform(
        Fixed4x4 matrix,
        out Vector2d scale,
        out Fixed64 rotation)
    {
        // A planar collider requires a block-diagonal X/Z and Y basis. Checking
        // both directions prevents scale extraction from silently discarding
        // hierarchy coupling that happens not to move local Y=0 points.
        if ((matrix.M12 != Fixed64.Zero)
            | (matrix.M21 != Fixed64.Zero)
            | (matrix.M23 != Fixed64.Zero)
            | (matrix.M32 != Fixed64.Zero))
        {
            scale = Vector2d.Zero;
            rotation = Fixed64.Zero;
            return false;
        }

        Fixed4x4 planarMatrix = Fixed4x4.Identity;
        planarMatrix.M11 = matrix.M11;
        planarMatrix.M13 = matrix.M13;
        planarMatrix.M31 = matrix.M31;
        planarMatrix.M33 = matrix.M33;
        if (!Fixed4x4.Decompose(
                planarMatrix,
                out _,
                out FixedQuaternion planarRotation,
                out Vector3d planarScale)
            || planarScale.X <= Fixed64.Zero)
        {
            scale = Vector2d.Zero;
            rotation = Fixed64.Zero;
            return false;
        }

        scale = planarScale.ToVector2d();
        Vector3d right = planarRotation.Rotate(Vector3d.Right);
        rotation = PlanarRotation.Canonicalize(
            FixedMath.Atan2(right.Z, right.X));
        return true;
    }

    internal static void Validate(Vector2d scale)
    {
        SwiftThrowHelper.ThrowIfArgument(
            !IsPositive(scale),
            nameof(scale),
            "2D collider scale must be greater than zero on the X and Z axes throughout its transform ancestry.");
    }

    internal static void Validate(Vector3d scale)
    {
        SwiftThrowHelper.ThrowIfArgument(
            !IsPositive(scale),
            nameof(scale),
            "3D collider scale must be greater than zero on every axis throughout its transform ancestry.");
    }

    private static bool IsPositive(Vector2d scale) =>
        scale.X > Fixed64.Zero && scale.Y > Fixed64.Zero;

    private static bool IsPositive(Vector3d scale) =>
        scale.X > Fixed64.Zero
        && scale.Y > Fixed64.Zero
        && scale.Z > Fixed64.Zero;

    internal static Fixed64 Scale(
        Fixed64 value,
        Fixed64 ownerScale,
        Fixed64 partScale) =>
        Scale(value, ownerScale, partScale, Fixed64.One);

    internal static Fixed64 Scale(
        Fixed64 value,
        Fixed64 ownerScale,
        Fixed64 partScale,
        Fixed64 divisor)
    {
        SwiftThrowHelper.ThrowIfArgument(
            !Fixed64.TryMultiplyDivide(value, ownerScale, partScale, divisor, out Fixed64 result),
            nameof(value),
            "Scaled collider geometry must be representable.");
        return result;
    }

    internal static Vector2d Scale(
        Vector2d value,
        Vector2d ownerScale,
        Vector2d partScale) =>
        Scale(value, ownerScale, partScale, Fixed64.One);

    internal static Vector2d Scale(
        Vector2d value,
        Vector2d ownerScale,
        Vector2d partScale,
        Fixed64 divisor) =>
        new(
            Scale(value.X, ownerScale.X, partScale.X, divisor),
            Scale(value.Y, ownerScale.Y, partScale.Y, divisor));

    internal static Fixed64 ScalePositive(
        Fixed64 dimension,
        Fixed64 ownerScale,
        Fixed64 partScale)
    {
        Fixed64 result = Scale(dimension, ownerScale, partScale);
        SwiftThrowHelper.ThrowIfArgument(
            result <= Fixed64.Zero,
            nameof(dimension),
            "Scaled collider dimensions must remain greater than zero.");
        return result;
    }

    internal static Vector2d ScalePositive(
        Vector2d dimensions,
        Vector2d ownerScale,
        Vector2d partScale,
        Fixed64 divisor) =>
        new(
            ScalePositive(dimensions.X, ownerScale.X, partScale.X, divisor),
            ScalePositive(dimensions.Y, ownerScale.Y, partScale.Y, divisor));

    internal static Vector3d ScalePositive(
        Vector3d dimensions,
        Vector3d ownerScale,
        Vector3d partScale,
        Fixed64 divisor) =>
        new(
            ScalePositive(dimensions.X, ownerScale.X, partScale.X, divisor),
            ScalePositive(dimensions.Y, ownerScale.Y, partScale.Y, divisor),
            ScalePositive(dimensions.Z, ownerScale.Z, partScale.Z, divisor));

    private static Fixed64 ScalePositive(
        Fixed64 dimension,
        Fixed64 ownerScale,
        Fixed64 partScale,
        Fixed64 divisor)
    {
        Fixed64 result = Scale(dimension, ownerScale, partScale, divisor);
        SwiftThrowHelper.ThrowIfArgument(
            result <= Fixed64.Zero,
            nameof(dimension),
            "Scaled collider dimensions must remain greater than zero.");
        return result;
    }

}
