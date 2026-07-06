//=======================================================================
// ManifoldContact.cs
//=======================================================================
// MIT License, Copyright (c) 2026–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp;
using Gravitas.Materials;

namespace Gravitas.CollisionHandling;

/// <summary>
/// A deterministic narrow-phase contact point for a collision manifold.
/// </summary>
public readonly struct ManifoldContact
{
    public ManifoldContact(
        ulong contactId,
        Vector3d pointA,
        Vector3d pointB,
        Fixed64 depth,
        Vector3d normal,
        bool hasMaterialOverride = false,
        PhysicsMaterial materialA = default,
        PhysicsMaterial materialB = default)
    {
        ContactId = contactId;
        PointA = pointA;
        PointB = pointB;
        Depth = depth.Abs();
        Normal = normal.MagnitudeSquared > Fixed64.Epsilon
            ? normal.Normalized
            : Vector3d.Zero;
        HasMaterialOverride = hasMaterialOverride;
        MaterialA = hasMaterialOverride ? materialA : PhysicsMaterial.Default;
        MaterialB = hasMaterialOverride ? materialB : PhysicsMaterial.Default;
    }

    /// <summary>
    /// Stable identity derived from the unordered pair of world-space contact points.
    /// </summary>
    public ulong ContactId { get; }

    /// <summary>
    /// World-space contact point on collider A.
    /// </summary>
    public Vector3d PointA { get; }

    /// <summary>
    /// World-space contact point on collider B.
    /// </summary>
    public Vector3d PointB { get; }

    /// <summary>
    /// Penetration depth along <see cref="Normal"/>.
    /// </summary>
    public Fixed64 Depth { get; }

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

}
