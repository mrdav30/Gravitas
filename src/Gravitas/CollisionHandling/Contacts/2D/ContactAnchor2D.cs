//=======================================================================
// ContactAnchor2D.cs
//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp;
using FixedMathSharp.Geometry;
using System.Runtime.CompilerServices;

namespace Gravitas.CollisionHandling;

/// <summary>
/// Represents a planar contact point as a world-space origin plus a rotated
/// local offset.
/// </summary>
public readonly struct ContactAnchor2D
{
    private readonly FixedPointAnchor2d _point;

    /// <summary>
    /// Creates a planar contact anchor in an identity-rotation rigid frame.
    /// </summary>
    public ContactAnchor2D(Vector2d origin, Vector2d offset)
        : this(origin, Fixed64.Zero, offset)
    {
    }

    /// <summary>
    /// Creates a planar contact anchor from a canonical local point.
    /// </summary>
    public ContactAnchor2D(
        Vector2d origin,
        Fixed64 rotation,
        Vector2d localPoint)
        : this(origin, rotation, localPoint, Vector2d.Zero)
    {
    }

    /// <summary>
    /// Creates a planar anchor whose two local terms remain separate until
    /// the exact rotated point is evaluated.
    /// </summary>
    public ContactAnchor2D(
        Vector2d origin,
        Fixed64 rotation,
        Vector2d localPoint,
        Vector2d localDisplacement)
    {
        _point = new FixedPointAnchor2d(
            origin,
            rotation,
            localPoint,
            localDisplacement);
    }

    /// <summary>
    /// Creates a physics-domain wrapper for an exact planar point anchor.
    /// </summary>
    public ContactAnchor2D(FixedPointAnchor2d point)
    {
        _point = point;
    }

    /// <summary>
    /// Gets the representable world-space origin.
    /// </summary>
    public Vector2d Origin => _point.Origin;

    /// <summary>
    /// Gets the point in the contact frame's local coordinates.
    /// </summary>
    public Vector2d LocalPoint => _point.LocalPoint;

    /// <summary>
    /// Gets the secondary local displacement retained separately from
    /// <see cref="LocalPoint"/>.
    /// </summary>
    public Vector2d LocalDisplacement => _point.LocalDisplacement;

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal int CompareLocalFeature(in ContactAnchor2D other) =>
        _point.CompareLocalFeature(other._point);

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal ulong GetLocalFeatureHash64() =>
        _point.GetLocalFeatureHash64();

    /// <summary>
    /// Gets the rotation applied to <see cref="LocalPoint"/>.
    /// </summary>
    public Fixed64 Rotation => _point.Rotation;

    /// <summary>
    /// Gets the rotated composite offset from the rigid-frame origin.
    /// </summary>
    /// <exception cref="System.InvalidOperationException">
    /// The rotated offset is outside the representable scalar domain. Use
    /// <see cref="TryGetOffset(out Vector2d)"/> when handling domain edges.
    /// </exception>
    public Vector2d Offset
    {
        get
        {
            bool succeeded = TryGetOffset(out Vector2d offset);
            SwiftThrowHelper.ThrowIfTrue(
                !succeeded,
                nameof(Offset),
                "The planar contact offset is outside the representable coordinate domain.");
            return offset;
        }
    }

    /// <summary>
    /// Creates an anchor for an already materialized world point.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static ContactAnchor2D FromWorldPoint(Vector2d point) =>
        new(Vector2d.Zero, point);

    /// <summary>
    /// Attempts to materialize the rotated composite rigid-frame offset.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public bool TryGetOffset(out Vector2d offset) =>
        _point.TryGetOffsetFrom(
            new FixedPointAnchor2d(
                Origin,
                Fixed64.Zero,
                Vector2d.Zero),
            out offset);

    /// <summary>
    /// Attempts to materialize the absolute world point without saturation.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public bool TryGetWorldPoint(out Vector2d point) =>
        _point.TryGetPoint(out point);

    /// <summary>
    /// Attempts to rebase the conceptual point onto another origin with one
    /// final exact component-wise add/subtract operation.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public bool TryGetOffsetFrom(Vector2d origin, out Vector2d offset) =>
        _point.TryGetOffsetFrom(
            new FixedPointAnchor2d(
                origin,
                Fixed64.Zero,
                Vector2d.Zero),
            out offset);

    /// <summary>
    /// Attempts to obtain this point's exact world-space offset from another
    /// planar contact anchor.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public bool TryGetOffsetFrom(
        ContactAnchor2D other,
        out Vector2d offset) =>
        _point.TryGetOffsetFrom(other._point, out offset);

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal FixedLever GetXZLeverFrom(in ContactAnchor2D other) =>
        _point.GetLeverFrom(other._point).ToXZLever();

    /// <summary>
    /// Attempts to express the same conceptual point relative to another
    /// representable origin.
    /// </summary>
    public bool TryRebase(Vector2d origin, out ContactAnchor2D anchor)
    {
        if (!_point.TryReframe(
                origin,
                Fixed64.Zero,
                out FixedPointAnchor2d point))
        {
            anchor = default;
            return false;
        }

        anchor = new ContactAnchor2D(point);
        return true;
    }

    /// <summary>
    /// Attempts to express the same conceptual point in another rotated frame.
    /// </summary>
    public bool TryRebase(
        Vector2d origin,
        Fixed64 rotation,
        out ContactAnchor2D anchor)
    {
        if (!_point.TryReframe(
                origin,
                rotation,
                out FixedPointAnchor2d point))
        {
            anchor = default;
            return false;
        }

        anchor = new ContactAnchor2D(point);
        return true;
    }
}
