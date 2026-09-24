using FixedMathSharp;
using Gravitas.Colliders;
using Gravitas.Support;
using SwiftCollections;

namespace Gravitas.Queries;

/// <summary>Describes a downward circle probe for physical support of a registered 2D collider.</summary>
public readonly struct Physics2DSupportQuery
{
    /// <summary>Creates a support probe and normalizes its nonzero upward direction.</summary>
    public Physics2DSupportQuery(LSCollider2D source, Vector2d center, Fixed64 radius,
        Vector2d up, Fixed64 distance, Fixed64 minimumNormalDot, PhysicsLayerMask layers)
    {
        SwiftThrowHelper.ThrowIfNull(source, nameof(source));
        SwiftThrowHelper.ThrowIfArgument(radius <= Fixed64.Zero, nameof(radius), "Support radius must be positive.");
        SwiftThrowHelper.ThrowIfArgument(distance <= Fixed64.Zero, nameof(distance), "Support distance must be positive.");
        SwiftThrowHelper.ThrowIfArgument(up == Vector2d.Zero, nameof(up), "Support up must be nonzero.");
        SwiftThrowHelper.ThrowIfArgument(minimumNormalDot <= Fixed64.Zero || minimumNormalDot > Fixed64.One,
            nameof(minimumNormalDot), "Support normal threshold must be in (0, 1].");
        Source = source;
        Center = center;
        Radius = radius;
        Up = up.Normalized;
        Distance = distance;
        MinimumNormalDot = minimumNormalDot;
        Layers = layers;
    }

    /// <summary>Gets the registered collider whose physical collision policy applies.</summary>
    public LSCollider2D Source { get; }
    /// <summary>Gets the initial probe center in native physics coordinates.</summary>
    public Vector2d Center { get; }
    /// <summary>Gets the positive probe radius.</summary>
    public Fixed64 Radius { get; }
    /// <summary>Gets the nonzero direction opposite to the sweep.</summary>
    public Vector2d Up { get; }
    /// <summary>Gets the positive maximum travel distance.</summary>
    public Fixed64 Distance { get; }
    /// <summary>Gets the inclusive normal/up threshold in (0, 1].</summary>
    public Fixed64 MinimumNormalDot { get; }
    /// <summary>Gets the included support layers, in addition to physical collision filters.</summary>
    public PhysicsLayerMask Layers { get; }
}
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
