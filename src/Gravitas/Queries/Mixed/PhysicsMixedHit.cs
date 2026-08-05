//=======================================================================
// PhysicsMixedHit.cs
//=======================================================================
// MIT License, Copyright (c) 2026–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp;
using Gravitas.Colliders;
using Gravitas.CollisionHandling;
using System;
using System.Runtime.CompilerServices;

namespace Gravitas.Queries;

/// <summary>
/// Result from an explicit mixed 3D/2D query.
/// </summary>
public readonly struct PhysicsMixedHit
{
    /// <summary>
    /// Creates a mixed query hit from world-space surface witnesses.
    /// </summary>
    public PhysicsMixedHit(
        LSCollider? collider3D,
        LSCollider2D? collider2D,
        Vector3d point3D,
        Vector3d point2D,
        Vector3d normal3DTo2D,
        PhysicsQueryReducerKind reducerKind,
        Fixed64 distance,
        Vector3d direction3D)
        : this(
            collider3D,
            collider2D,
            ContactAnchor.FromWorldPoint(point3D),
            ContactAnchor.FromWorldPoint(point2D),
            normal3DTo2D,
            reducerKind,
            distance,
            direction3D)
    {
    }

    /// <summary>
    /// Creates a mixed query hit with rigid-frame surface witnesses.
    /// </summary>
    /// <exception cref="ArgumentException">
    /// Either rigid-frame anchor is invalid.
    /// </exception>
    public PhysicsMixedHit(
        LSCollider? collider3D,
        LSCollider2D? collider2D,
        ContactAnchor anchor3D,
        ContactAnchor anchor2D,
        Vector3d normal3DTo2D,
        PhysicsQueryReducerKind reducerKind,
        Fixed64 distance,
        Vector3d direction3D)
    {
        if (!anchor3D.IsValid)
            throw new ArgumentException("The 3D hit anchor must be valid.", nameof(anchor3D));
        if (!anchor2D.IsValid)
            throw new ArgumentException("The 2D hit anchor must be valid.", nameof(anchor2D));

        Collider3D = collider3D;
        Collider2D = collider2D;
        Body3D = collider3D?.Body;
        Body2D = collider2D?.Body;
        Anchor3D = anchor3D;
        Anchor2D = anchor2D;
        Normal3DTo2D = normal3DTo2D;
        ReducerKind = reducerKind;
        Distance = distance;
        Direction3D = direction3D;
    }

    /// <summary>
    /// Gets the 3D collider participating in the hit, if registered.
    /// </summary>
    public LSCollider? Collider3D { get; }

    /// <summary>
    /// Gets the 2D collider participating in the hit, if registered.
    /// </summary>
    public LSCollider2D? Collider2D { get; }

    /// <summary>
    /// Gets the 3D collider's body, if any.
    /// </summary>
    public SolidBody? Body3D { get; }

    /// <summary>
    /// Gets the 2D collider's body, if any.
    /// </summary>
    public SolidBody2D? Body2D { get; }

    /// <summary>
    /// Gets the rigid-frame witness on the 3D shape.
    /// </summary>
    public ContactAnchor Anchor3D { get; }

    /// <summary>
    /// Gets the rigid-frame witness on the embedded 2D volume.
    /// </summary>
    public ContactAnchor Anchor2D { get; }

    /// <summary>
    /// Gets the materialized 3D witness.
    /// </summary>
    /// <exception cref="InvalidOperationException">
    /// The conceptual witness lies outside the representable coordinate range.
    /// </exception>
    public Vector3d Point3D => GetRequiredWorldPoint(Anchor3D, nameof(Point3D));

    /// <summary>
    /// Gets the materialized embedded-2D witness.
    /// </summary>
    /// <exception cref="InvalidOperationException">
    /// The conceptual witness lies outside the representable coordinate range.
    /// </exception>
    public Vector3d Point2D => GetRequiredWorldPoint(Anchor2D, nameof(Point2D));

    /// <summary>
    /// Attempts to materialize the 3D witness without saturation.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public bool TryGetPoint3D(out Vector3d point) => Anchor3D.TryGetWorldPoint(out point);

    /// <summary>
    /// Attempts to materialize the embedded-2D witness without saturation.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public bool TryGetPoint2D(out Vector3d point) => Anchor2D.TryGetWorldPoint(out point);

    /// <summary>
    /// Normal pointing from the 3D collider or swept 3D source toward the embedded 2D volume.
    /// </summary>
    public Vector3d Normal3DTo2D { get; }

    /// <summary>
    /// Gets whether this mixed query hit was produced by an exact shape reducer or a conservative fallback.
    /// </summary>
    public PhysicsQueryReducerKind ReducerKind { get; }

    /// <summary>
    /// Distance travelled by the swept source center before impact.
    /// </summary>
    public Fixed64 Distance { get; }

    /// <summary>
    /// Gets the world-space sweep direction associated with the hit.
    /// </summary>
    public Vector3d Direction3D { get; }

    /// <summary>
    /// Surface normal suitable for clamping a 3D source moving into an embedded 2D target.
    /// </summary>
    public Vector3d NormalFor3DSource
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        get => -Normal3DTo2D;
    }

    /// <summary>
    /// Planar normal suitable for clamping a 2D source moving into a 3D target.
    /// </summary>
    public Vector2d NormalFor2DSource
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        get
        {
            Vector2d normal = new(Normal3DTo2D.X, Normal3DTo2D.Z);
            return normal.MagnitudeSquared > Fixed64.Epsilon ? normal.Normalized : Vector2d.Zero;
        }
    }

    private static Vector3d GetRequiredWorldPoint(ContactAnchor anchor, string propertyName)
    {
        if (anchor.TryGetWorldPoint(out Vector3d point))
            return point;

        throw new InvalidOperationException(
            $"{propertyName} is outside the representable coordinate range. Use the corresponding TryGet method.");
    }
}
