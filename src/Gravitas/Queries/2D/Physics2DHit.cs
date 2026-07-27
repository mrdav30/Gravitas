//=======================================================================
// Physics2DHit.cs
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
/// Result from a pure 2D query.
/// </summary>
public readonly struct Physics2DHit
{
    public Physics2DHit(LSCollider2D collider, Vector2d point, Vector2d normal, Fixed64 distance)
        : this(
            collider,
            ContactAnchor2D.FromWorldPoint(point),
            normal,
            distance)
    {
    }

    /// <summary>
    /// Creates a 2D query hit with a rigid-frame surface witness.
    /// </summary>
    public Physics2DHit(
        LSCollider2D collider,
        ContactAnchor2D anchor,
        Vector2d normal,
        Fixed64 distance)
    {
        Collider = collider;
        Body = collider.Body;
        Anchor = anchor;
        Normal = normal;
        Distance = distance;
    }

    public LSCollider2D Collider { get; }

    public SolidBody2D? Body { get; }

    /// <summary>
    /// Gets the rigid-frame query witness.
    /// </summary>
    public ContactAnchor2D Anchor { get; }

    /// <summary>
    /// Gets the materialized world-space query witness.
    /// </summary>
    /// <exception cref="InvalidOperationException">
    /// The conceptual witness lies outside the representable coordinate range.
    /// </exception>
    public Vector2d Point
    {
        get
        {
            if (TryGetPoint(out Vector2d point))
                return point;

            throw new InvalidOperationException(
                "Point is outside the representable coordinate range. Use TryGetPoint.");
        }
    }

    /// <summary>
    /// Attempts to materialize the world-space query witness without saturation.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public bool TryGetPoint(out Vector2d point) => Anchor.TryGetWorldPoint(out point);

    public Vector2d Normal { get; }

    public Fixed64 Distance { get; }
}
