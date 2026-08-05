//=======================================================================
// ManifoldContact2D.cs
//=======================================================================
// MIT License, Copyright (c) 2026–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp;
using Gravitas.Materials;
using System;
using System.Runtime.CompilerServices;

namespace Gravitas.CollisionHandling;

/// <summary>
/// A deterministic pure 2D narrow-phase contact point for a collision manifold.
/// </summary>
public readonly struct ManifoldContact2D
{
    /// <summary>Creates a deterministic planar contact from world-space witness points.</summary>
    public ManifoldContact2D(
        ulong contactId,
        Vector2d pointA,
        Vector2d pointB,
        Fixed64 depth,
        Vector2d normal,
        bool hasMaterialOverride = false,
        PhysicsMaterial materialA = default,
        PhysicsMaterial materialB = default,
        bool depthIsClamped = false)
        : this(
            contactId,
            ContactAnchor2D.FromWorldPoint(pointA),
            ContactAnchor2D.FromWorldPoint(pointB),
            depth,
            normal,
            hasMaterialOverride,
            materialA,
            materialB,
            depthIsClamped)
    { }

    /// <summary>
    /// Creates a deterministic planar contact from authoritative relative
    /// anchors.
    /// </summary>
    public ManifoldContact2D(
        ulong contactId,
        ContactAnchor2D anchorA,
        ContactAnchor2D anchorB,
        Fixed64 depth,
        Vector2d normal,
        bool hasMaterialOverride = false,
        PhysicsMaterial materialA = default,
        PhysicsMaterial materialB = default,
        bool depthIsClamped = false)
        : this(
            contactId,
            anchorA,
            anchorB,
            depth,
            normal,
            hasMaterialOverride,
            materialA,
            materialB,
            depthIsClamped,
            featureNamespaceA: 0,
            featureNamespaceB: 0)
    { }

    internal ManifoldContact2D(
        ulong contactId,
        ContactAnchor2D anchorA,
        ContactAnchor2D anchorB,
        Fixed64 depth,
        Vector2d normal,
        bool hasMaterialOverride,
        PhysicsMaterial materialA,
        PhysicsMaterial materialB,
        bool depthIsClamped,
        int featureNamespaceA,
        int featureNamespaceB)
    {
        ContactId = contactId;
        AnchorA = anchorA;
        AnchorB = anchorB;
        FeatureNamespaceA = featureNamespaceA;
        FeatureNamespaceB = featureNamespaceB;
        Depth = depth.Abs();
        DepthIsClamped = depthIsClamped;
        Normal = normal.MagnitudeSquared > Fixed64.Epsilon
            ? normal.Normalized
            : Vector2d.Zero;
        HasMaterialOverride = hasMaterialOverride;
        MaterialA = hasMaterialOverride ? materialA : PhysicsMaterial.Default;
        MaterialB = hasMaterialOverride ? materialB : PhysicsMaterial.Default;
    }

    /// <summary>
    /// Stable identity derived from the unordered pair of pose-invariant
    /// canonical local feature terms.
    /// </summary>
    public ulong ContactId { get; }

    /// <summary>
    /// Authoritative canonical contact anchor on collider A.
    /// </summary>
    public ContactAnchor2D AnchorA { get; }

    /// <summary>
    /// Authoritative canonical contact anchor on collider B.
    /// </summary>
    public ContactAnchor2D AnchorB { get; }

    internal int FeatureNamespaceA { get; }

    internal int FeatureNamespaceB { get; }

    /// <summary>
    /// Gets the world-space X/Z-plane contact point on collider A.
    /// </summary>
    /// <exception cref="InvalidOperationException">
    /// The conceptual world point is outside the scalar range.
    /// </exception>
    public Vector2d PointA => GetRequiredWorldPoint(AnchorA, nameof(PointA));

    /// <summary>
    /// Gets the world-space X/Z-plane contact point on collider B.
    /// </summary>
    /// <exception cref="InvalidOperationException">
    /// The conceptual world point is outside the scalar range.
    /// </exception>
    public Vector2d PointB => GetRequiredWorldPoint(AnchorB, nameof(PointB));

    /// <summary>
    /// Attempts to materialize the world-space contact point on collider A.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public bool TryGetPointA(out Vector2d point) => AnchorA.TryGetWorldPoint(out point);

    /// <summary>
    /// Attempts to materialize the world-space contact point on collider B.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public bool TryGetPointB(out Vector2d point) => AnchorB.TryGetWorldPoint(out point);

    /// <summary>
    /// Penetration depth along <see cref="Normal"/>.
    /// </summary>
    public Fixed64 Depth { get; }

    /// <summary>
    /// Gets whether the conceptual penetration depth exceeded the scalar range.
    /// </summary>
    public bool DepthIsClamped { get; }

    /// <summary>
    /// Unit normal pointing from collider A toward collider B.
    /// </summary>
    public Vector2d Normal { get; }

    /// <summary>
    /// Gets whether this contact carries materials from private compound parts.
    /// </summary>
    public bool HasMaterialOverride { get; }

    /// <summary>
    /// Material for collider A when <see cref="HasMaterialOverride"/> is true.
    /// </summary>
    public PhysicsMaterial MaterialA { get; }

    /// <summary>
    /// Material for collider B when <see cref="HasMaterialOverride"/> is true.
    /// </summary>
    public PhysicsMaterial MaterialB { get; }

    private static Vector2d GetRequiredWorldPoint(ContactAnchor2D anchor, string propertyName)
    {
        if (anchor.TryGetWorldPoint(out Vector2d point))
            return point;

        throw new InvalidOperationException(
            $"{propertyName} is outside the representable coordinate range. Use the corresponding TryGet method.");
    }
}
