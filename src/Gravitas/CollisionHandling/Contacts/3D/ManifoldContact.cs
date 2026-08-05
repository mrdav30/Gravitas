//=======================================================================
// ManifoldContact.cs
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
/// A deterministic narrow-phase contact point for a collision manifold.
/// </summary>
public readonly struct ManifoldContact
{
    /// <summary>Creates a deterministic contact from world-space witness points.</summary>
    public ManifoldContact(
        ulong contactId,
        Vector3d pointA,
        Vector3d pointB,
        Fixed64 depth,
        Vector3d normal,
        bool hasMaterialOverride = false,
        PhysicsMaterial materialA = default,
        PhysicsMaterial materialB = default,
        bool depthIsClamped = false)
        : this(
            contactId,
            ContactAnchor.FromWorldPoint(pointA),
            ContactAnchor.FromWorldPoint(pointB),
            depth,
            normal,
            hasMaterialOverride,
            materialA,
            materialB,
            depthIsClamped)
    { }

    /// <summary>
    /// Creates a deterministic contact from authoritative rigid-frame anchors.
    /// </summary>
    public ManifoldContact(
        ulong contactId,
        ContactAnchor anchorA,
        ContactAnchor anchorB,
        Fixed64 depth,
        Vector3d normal,
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

    internal ManifoldContact(
        ulong contactId,
        ContactAnchor anchorA,
        ContactAnchor anchorB,
        Fixed64 depth,
        Vector3d normal,
        bool hasMaterialOverride,
        PhysicsMaterial materialA,
        PhysicsMaterial materialB,
        bool depthIsClamped,
        int featureNamespaceA,
        int featureNamespaceB)
    {
        if (!anchorA.IsValid)
            throw new ArgumentException("Contact anchor A must be valid.", nameof(anchorA));
        if (!anchorB.IsValid)
            throw new ArgumentException("Contact anchor B must be valid.", nameof(anchorB));

        ContactId = contactId;
        AnchorA = anchorA;
        AnchorB = anchorB;
        FeatureNamespaceA = featureNamespaceA;
        FeatureNamespaceB = featureNamespaceB;
        Depth = depth.Abs();
        DepthIsClamped = depthIsClamped;
        Normal = normal.MagnitudeSquared > Fixed64.Epsilon
            ? normal.Normalized
            : Vector3d.Zero;
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
    public ContactAnchor AnchorA { get; }

    /// <summary>
    /// Authoritative canonical contact anchor on collider B.
    /// </summary>
    public ContactAnchor AnchorB { get; }

    internal int FeatureNamespaceA { get; }

    internal int FeatureNamespaceB { get; }

    /// <summary>
    /// Gets the world-space contact point on collider A.
    /// </summary>
    /// <exception cref="InvalidOperationException">
    /// The conceptual world point is outside the scalar range.
    /// </exception>
    public Vector3d PointA => GetRequiredWorldPoint(AnchorA, nameof(PointA));

    /// <summary>
    /// Gets the world-space contact point on collider B.
    /// </summary>
    /// <exception cref="InvalidOperationException">
    /// The conceptual world point is outside the scalar range.
    /// </exception>
    public Vector3d PointB => GetRequiredWorldPoint(AnchorB, nameof(PointB));

    /// <summary>
    /// Attempts to materialize the world-space contact point on collider A.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public bool TryGetPointA(out Vector3d point) => AnchorA.TryGetWorldPoint(out point);

    /// <summary>
    /// Attempts to materialize the world-space contact point on collider B.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public bool TryGetPointB(out Vector3d point) => AnchorB.TryGetWorldPoint(out point);

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
    public Vector3d Normal { get; }

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

    private static Vector3d GetRequiredWorldPoint(ContactAnchor anchor, string propertyName)
    {
        if (anchor.TryGetWorldPoint(out Vector3d point))
            return point;

        throw new InvalidOperationException(
            $"{propertyName} is outside the representable coordinate range. Use the corresponding TryGet method.");
    }
}
