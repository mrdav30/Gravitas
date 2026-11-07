//=======================================================================
// Physics3DHit.cs
//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using System;
using System.Runtime.CompilerServices;
using FixedMathSharp;
using Gravitas.Colliders;
using Gravitas.CollisionHandling;

namespace Gravitas.Queries;

/// <summary>
/// Result from a 3D physics query.
/// </summary>
public readonly struct Physics3DHit
{
    /// <summary>
    /// Creates a 3D query hit from a world-space surface witness.
    /// </summary>
    public Physics3DHit(LSCollider? collider, Vector3d point, Vector3d normal, Fixed64 distance, Vector3d direction)
        : this(
            collider,
            ContactAnchor.FromWorldPoint(point),
            normal,
            distance,
            direction)
    {
    }

    /// <summary>
    /// Creates a query hit with a rigid-frame surface witness.
    /// </summary>
    public Physics3DHit(
        LSCollider? collider,
        ContactAnchor anchor,
        Vector3d normal,
        Fixed64 distance,
        Vector3d direction)
    {
        if (!anchor.IsValid)
            throw new ArgumentException("The hit anchor must be valid.", nameof(anchor));

        Collider = collider;
        Body = collider?.Body;
        Anchor = anchor;
        Normal = normal;
        Distance = distance;
        Direction = direction;
    }

    /// <summary>
    /// Gets the collider reported by the query, if any.
    /// </summary>
    public LSCollider? Collider { get; }

    /// <summary>
    /// Gets the collider's body, or <see langword="null"/> for a bodyless hit.
    /// </summary>
    public SolidBody? Body { get; }

    /// <summary>
    /// Gets the authoritative rigid-frame query witness.
    /// </summary>
    public ContactAnchor Anchor { get; }

    /// <summary>
    /// Gets the materialized world-space query witness.
    /// </summary>
    /// <exception cref="InvalidOperationException">
    /// The conceptual witness lies outside the representable coordinate range.
    /// </exception>
    public Vector3d Point
    {
        get
        {
            if (TryGetPoint(out Vector3d point))
                return point;

            throw new InvalidOperationException(
                "Point is outside the representable coordinate range. Use TryGetPoint.");
        }
    }

    /// <summary>
    /// Attempts to materialize the world-space query witness without
    /// saturation.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public bool TryGetPoint(out Vector3d point) => Anchor.TryGetWorldPoint(out point);

    /// <summary>
    /// Gets the world-space surface normal.
    /// </summary>
    public Vector3d Normal { get; }

    /// <summary>
    /// Gets the query-specific distance used to rank this hit.
    /// </summary>
    /// <remarks>
    /// Rays and sweeps report query travel; cone volumes report axial distance.
    /// Projected overlaps use their overlap ranking distance, which can be zero
    /// for a contained origin. This need not equal origin-to-witness distance.
    /// </remarks>
    public Fixed64 Distance { get; }

    /// <summary>
    /// Gets the query direction associated with the hit.
    /// </summary>
    public Vector3d Direction { get; }
}
