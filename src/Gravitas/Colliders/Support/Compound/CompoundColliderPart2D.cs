//=======================================================================
// CompoundColliderPart2D.cs
//=======================================================================
// MIT License, Copyright (c) 2026–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp;
using System.Runtime.CompilerServices;

namespace Gravitas.Colliders;

/// <summary>
/// Declares one data-only geometry part owned by an
/// <see cref="LSCompoundCollider2D"/>.
/// </summary>
public readonly struct CompoundColliderPart2D
{
    public CompoundColliderPart2D(ColliderShapeDefinition2D shape)
        : this(shape, Vector2d.Zero, Fixed64.Zero, Vector2d.One)
    { }

    public CompoundColliderPart2D(ColliderShapeDefinition2D shape, Vector2d localOffset)
        : this(shape, localOffset, Fixed64.Zero, Vector2d.One)
    { }

    public CompoundColliderPart2D(
        ColliderShapeDefinition2D shape,
        Vector2d localOffset,
        Fixed64 localRotation)
        : this(shape, localOffset, localRotation, Vector2d.One)
    { }

    public CompoundColliderPart2D(
        ColliderShapeDefinition2D shape,
        Vector2d localOffset,
        Fixed64 localRotation,
        Vector2d localScale)
    {
        shape.EnsureDefined();
        ValidateScale(localScale);

        Shape = shape;
        LocalOffset = localOffset;
        LocalRotation = localRotation;
        LocalScale = localScale;
    }

    /// <summary>
    /// Gets the authored data-only shape definition for this part.
    /// </summary>
    public ColliderShapeDefinition2D Shape { get; }

    /// <summary>
    /// Gets the deterministic local center offset applied relative to the
    /// owning compound collider.
    /// </summary>
    public Vector2d LocalOffset { get; }

    /// <summary>
    /// Gets the deterministic local rotation applied relative to the owning
    /// compound collider.
    /// </summary>
    public Fixed64 LocalRotation { get; }

    /// <summary>
    /// Gets the deterministic local scale applied relative to the owning
    /// compound collider.
    /// </summary>
    public Vector2d LocalScale { get; }

    public static CompoundColliderPart2D Circle(Fixed64 radius, Vector2d localOffset) =>
        new(ColliderShapeDefinition2D.Circle(radius), localOffset);

    public static CompoundColliderPart2D Circle(
        Fixed64 radius,
        Vector2d localOffset,
        Fixed64 localRotation,
        Vector2d localScale) =>
        new(ColliderShapeDefinition2D.Circle(radius), localOffset, localRotation, localScale);

    public static CompoundColliderPart2D AABBox(Vector2d size, Vector2d localOffset) =>
        new(ColliderShapeDefinition2D.AABBox(size), localOffset);

    public static CompoundColliderPart2D AABBox(
        Vector2d size,
        Vector2d localOffset,
        Fixed64 localRotation,
        Vector2d localScale) =>
        new(ColliderShapeDefinition2D.AABBox(size), localOffset, localRotation, localScale);

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static CompoundColliderPart2D AABox(Vector2d size, Vector2d localOffset) =>
        AABBox(size, localOffset);

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static CompoundColliderPart2D AABox(
        Vector2d size,
        Vector2d localOffset,
        Fixed64 localRotation,
        Vector2d localScale) =>
        AABBox(size, localOffset, localRotation, localScale);

    public static CompoundColliderPart2D ConvexPolygon(Vector2d[] vertices, Vector2d localOffset) =>
        new(ColliderShapeDefinition2D.ConvexPolygon(vertices), localOffset);

    public static CompoundColliderPart2D ConvexPolygon(
        Vector2d[] vertices,
        Vector2d localOffset,
        Fixed64 localRotation,
        Vector2d localScale) =>
        new(ColliderShapeDefinition2D.ConvexPolygon(vertices), localOffset, localRotation, localScale);

    internal bool IsDefault
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        get => Shape.Kind == ColliderShapeDefinition2DKind.Undefined;
    }

    private static void ValidateScale(Vector2d localScale)
    {
        SwiftThrowHelper.ThrowIfArgument(
            localScale.X <= Fixed64.Zero || localScale.Y <= Fixed64.Zero,
            nameof(localScale),
            "2D compound collider part scale components must be greater than zero.");
    }
}
