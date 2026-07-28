//=======================================================================
// ContactAnchor.cs
//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp;
using FixedMathSharp.Geometry;
using System.Runtime.CompilerServices;

namespace Gravitas.CollisionHandling;

/// <summary>
/// Represents a 3D contact point in one rigid world-space frame.
/// </summary>
/// <remarks>
/// The underlying origin, rotation, and local point remain authoritative when
/// either the rotated offset or absolute world point lies outside the
/// <see cref="Fixed64"/> scalar range.
/// </remarks>
public readonly struct ContactAnchor
{
    private readonly FixedPointAnchor _point;

    /// <summary>
    /// Creates a contact anchor in an identity-rotation rigid frame.
    /// </summary>
    public ContactAnchor(Vector3d origin, Vector3d offset)
        : this(origin, FixedQuaternion.Identity, offset)
    {
    }

    /// <summary>
    /// Creates a contact anchor in a rigid local frame.
    /// </summary>
    public ContactAnchor(
        Vector3d origin,
        FixedQuaternion rotation,
        Vector3d localPoint)
        : this(origin, rotation, localPoint, Vector3d.Zero)
    {
    }

    /// <summary>
    /// Creates a contact anchor whose two local terms remain separate until
    /// the exact rotated point is evaluated.
    /// </summary>
    public ContactAnchor(
        Vector3d origin,
        FixedQuaternion rotation,
        Vector3d localPoint,
        Vector3d localDisplacement)
    {
        _point = new FixedPointAnchor(
            origin,
            rotation,
            localPoint,
            localDisplacement);
    }

    /// <summary>
    /// Creates a physics-domain wrapper for an exact point anchor.
    /// </summary>
    public ContactAnchor(FixedPointAnchor point)
    {
        if (!point.Rotation.IsNormalized())
        {
            throw new System.ArgumentException(
                "The point anchor must contain a normalized rotation.",
                nameof(point));
        }

        _point = point;
    }

    /// <summary>
    /// Gets the representable world-space origin.
    /// </summary>
    public Vector3d Origin => _point.Origin;

    /// <summary>
    /// Gets the normalized local-to-world frame rotation.
    /// </summary>
    public FixedQuaternion Rotation => _point.Rotation;

    internal bool IsValid => Rotation.IsNormalized();

    /// <summary>
    /// Gets the point in the contact frame's local coordinates.
    /// </summary>
    public Vector3d LocalPoint => _point.LocalPoint;

    /// <summary>
    /// Gets the secondary local displacement retained separately from
    /// <see cref="LocalPoint"/>.
    /// </summary>
    public Vector3d LocalDisplacement => _point.LocalDisplacement;

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal int CompareLocalFeature(in ContactAnchor other) =>
        _point.CompareLocalFeature(other._point);

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal ulong GetLocalFeatureHash64() =>
        _point.GetLocalFeatureHash64();

    /// <summary>
    /// Gets the rotated offset from <see cref="Origin"/> to the conceptual
    /// contact point.
    /// </summary>
    /// <exception cref="System.InvalidOperationException">
    /// The rotated offset lies outside the representable coordinate domain.
    /// </exception>
    public Vector3d Offset
    {
        get
        {
            bool succeeded = TryGetOffset(out Vector3d offset);
            SwiftThrowHelper.ThrowIfTrue(
                !succeeded,
                nameof(Offset),
                "The contact offset is outside the representable coordinate domain.");
            return offset;
        }
    }

    /// <summary>
    /// Creates an anchor for an already materialized world point.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static ContactAnchor FromWorldPoint(Vector3d point) =>
        new(Vector3d.Zero, point);

    /// <summary>
    /// Attempts to materialize the absolute world point without saturation.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public bool TryGetWorldPoint(out Vector3d point) =>
        _point.TryGetPoint(out point);

    /// <summary>
    /// Attempts to obtain the rotated offset from <see cref="Origin"/> to the
    /// conceptual contact point.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public bool TryGetOffset(out Vector3d offset) =>
        _point.TryGetOffsetFrom(
            new FixedPointAnchor(
                Origin,
                FixedQuaternion.Identity,
                Vector3d.Zero),
            out offset);

    /// <summary>
    /// Attempts to rebase the conceptual point onto another origin with one
    /// final exact component-wise add/subtract operation.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public bool TryGetOffsetFrom(Vector3d origin, out Vector3d offset) =>
        _point.TryGetOffsetFrom(
            new FixedPointAnchor(
                origin,
                FixedQuaternion.Identity,
                Vector3d.Zero),
            out offset);

    /// <summary>
    /// Attempts to obtain this point's exact world-space offset from another
    /// rigid-frame contact anchor.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public bool TryGetOffsetFrom(
        in ContactAnchor other,
        out Vector3d offset) =>
        _point.TryGetOffsetFrom(other._point, out offset);

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal FixedLever GetLeverFrom(in ContactAnchor other)
    {
        _ = _point.TryGetLeverFrom(other._point, out FixedLever lever);
        return lever;
    }

    /// <summary>
    /// Attempts to express the same conceptual point relative to another
    /// representable origin.
    /// </summary>
    public bool TryRebase(Vector3d origin, out ContactAnchor anchor)
    {
        if (!_point.TryReframe(
                origin,
                FixedQuaternion.Identity,
                out FixedPointAnchor point))
        {
            anchor = default;
            return false;
        }

        anchor = new ContactAnchor(point);
        return true;
    }

    /// <summary>
    /// Attempts to express the same conceptual point in another rigid frame.
    /// </summary>
    public bool TryRebase(
        Vector3d origin,
        FixedQuaternion rotation,
        out ContactAnchor anchor)
    {
        if (!_point.TryReframe(
                origin,
                rotation,
                out FixedPointAnchor point))
        {
            anchor = default;
            return false;
        }

        anchor = new ContactAnchor(point);
        return true;
    }
}
