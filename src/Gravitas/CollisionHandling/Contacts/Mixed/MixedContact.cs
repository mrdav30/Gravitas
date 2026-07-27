//=======================================================================
// MixedContact.cs
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
/// Contact generated between one 3D collider and one embedded 2D collider.
/// </summary>
public readonly struct MixedContact
{
    public MixedContact(
        Vector3d point3D,
        Vector3d point2D,
        Vector3d normal3DTo2D,
        Fixed64 depth,
        bool depthIsClamped = false)
        : this(
            ContactAnchor.FromWorldPoint(point3D),
            ContactAnchor.FromWorldPoint(point2D),
            normal3DTo2D,
            depth,
            depthIsClamped,
            hasMaterialOverride: false,
            default,
            default)
    { }

    /// <summary>
    /// Creates a mixed contact with authoritative rigid-frame witnesses.
    /// </summary>
    /// <exception cref="ArgumentException">
    /// Either rigid-frame anchor is invalid.
    /// </exception>
    public MixedContact(
        ContactAnchor anchor3D,
        ContactAnchor anchor2D,
        Vector3d normal3DTo2D,
        Fixed64 depth,
        bool depthIsClamped = false)
        : this(
            anchor3D,
            anchor2D,
            normal3DTo2D,
            depth,
            depthIsClamped,
            hasMaterialOverride: false,
            default,
            default)
    { }

    internal MixedContact(
        Vector3d point3D,
        Vector3d point2D,
        Vector3d normal3DTo2D,
        Fixed64 depth,
        PhysicsMaterial material3D,
        PhysicsMaterial material2D,
        bool depthIsClamped = false)
        : this(
            ContactAnchor.FromWorldPoint(point3D),
            ContactAnchor.FromWorldPoint(point2D),
            normal3DTo2D,
            depth,
            depthIsClamped,
            hasMaterialOverride: true,
            material3D,
            material2D)
    { }

    internal MixedContact(
        ContactAnchor anchor3D,
        ContactAnchor anchor2D,
        Vector3d normal3DTo2D,
        Fixed64 depth,
        PhysicsMaterial material3D,
        PhysicsMaterial material2D,
        bool depthIsClamped = false)
        : this(
            anchor3D,
            anchor2D,
            normal3DTo2D,
            depth,
            depthIsClamped,
            hasMaterialOverride: true,
            material3D,
            material2D)
    { }

    private MixedContact(
        ContactAnchor anchor3D,
        ContactAnchor anchor2D,
        Vector3d normal3DTo2D,
        Fixed64 depth,
        bool depthIsClamped,
        bool hasMaterialOverride,
        PhysicsMaterial material3D,
        PhysicsMaterial material2D)
    {
        if (!anchor3D.IsValid)
            throw new ArgumentException("The 3D contact anchor must be valid.", nameof(anchor3D));
        if (!anchor2D.IsValid)
            throw new ArgumentException("The 2D contact anchor must be valid.", nameof(anchor2D));

        Anchor3D = anchor3D;
        Anchor2D = anchor2D;
        Normal3DTo2D = normal3DTo2D;
        Depth = depth;
        DepthIsClamped = depthIsClamped;
        HasMaterialOverride = hasMaterialOverride;
        Material3D = hasMaterialOverride ? material3D : PhysicsMaterial.Default;
        Material2D = hasMaterialOverride ? material2D : PhysicsMaterial.Default;
        HasContact = true;
    }

    public bool HasContact { get; }

    public ContactAnchor Anchor3D { get; }

    public ContactAnchor Anchor2D { get; }

    public Vector3d Point3D => GetRequiredWorldPoint(Anchor3D, nameof(Point3D));

    public Vector3d Point2D => GetRequiredWorldPoint(Anchor2D, nameof(Point2D));

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public bool TryGetPoint3D(out Vector3d point) => Anchor3D.TryGetWorldPoint(out point);

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public bool TryGetPoint2D(out Vector3d point) => Anchor2D.TryGetWorldPoint(out point);

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal bool TryGetPlanarOffset2DFrom(
        Vector2d origin,
        out Vector2d offset) =>
        TryGetPlanarOffset2DFrom(
            origin,
            Fixed64.Zero,
            Vector2d.Zero,
            out offset);

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal bool TryGetPlanarOffset2DFrom(
        Vector2d origin,
        Fixed64 rotation,
        Vector2d localPoint,
        out Vector2d offset)
    {
        var reference = new ContactAnchor(
            new Vector3d(
                origin.X,
                Anchor2D.Origin.Y,
                origin.Y),
            FixedQuaternion.FromAxisAngle(Vector3d.Up, -rotation),
            new Vector3d(
                localPoint.X,
                Fixed64.Zero,
                localPoint.Y));
        if (!Anchor2D.TryGetOffsetFrom(
                reference,
                out Vector3d offset3D))
        {
            offset = default;
            return false;
        }

        offset = new Vector2d(offset3D.X, offset3D.Z);
        return true;
    }

    /// <summary>
    /// Contact normal pointing from the 3D collider toward the embedded 2D collider volume.
    /// </summary>
    public Vector3d Normal3DTo2D { get; }

    public Fixed64 Depth { get; }

    /// <summary>
    /// Gets whether the exact penetration depth exceeded the scalar domain and
    /// <see cref="Depth"/> therefore contains <see cref="Fixed64.MaxValue"/>.
    /// </summary>
    public bool DepthIsClamped { get; }

    public bool HasMaterialOverride { get; }

    public PhysicsMaterial Material3D { get; }

    public PhysicsMaterial Material2D { get; }

    internal MixedContact WithMaterialOverride(PhysicsMaterial material3D, PhysicsMaterial material2D) =>
        new(
            Anchor3D,
            Anchor2D,
            Normal3DTo2D,
            Depth,
            material3D,
            material2D,
            DepthIsClamped);

    internal MixedContact WithFallbackMaterials(PhysicsMaterial material3D, PhysicsMaterial material2D) =>
        HasMaterialOverride ? this : WithMaterialOverride(material3D, material2D);

    private static Vector3d GetRequiredWorldPoint(ContactAnchor anchor, string propertyName)
    {
        if (anchor.TryGetWorldPoint(out Vector3d point))
            return point;

        throw new InvalidOperationException(
            $"{propertyName} is outside the representable coordinate range. Use the corresponding TryGet method.");
    }
}
