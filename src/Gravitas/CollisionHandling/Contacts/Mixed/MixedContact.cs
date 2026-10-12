//=======================================================================
// MixedContact.cs
//=======================================================================
// MIT License, Copyright (c) 2026–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp;
using FixedMathSharp.Geometry;
using Gravitas.Colliders;
using Gravitas.Materials;
using System;
using System.Runtime.CompilerServices;

namespace Gravitas.CollisionHandling;

/// <summary>
/// Contact generated between one 3D collider and one embedded 2D collider.
/// </summary>
/// <remarks>A single contact may represent one row of a larger manifold. Its depth does not guarantee escape from a nonconvex collider.</remarks>
public readonly struct MixedContact
{
    /// <summary>Creates a mixed contact from world-space witness points.</summary>
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

    /// <summary>Gets whether this value contains a valid mixed contact.</summary>
    public bool HasContact { get; }

    /// <summary>Gets the authoritative contact anchor on the 3D collider.</summary>
    public ContactAnchor Anchor3D { get; }

    /// <summary>Gets the authoritative contact anchor on the embedded 2D collider.</summary>
    public ContactAnchor Anchor2D { get; }

    /// <summary>Gets the world-space witness point on the 3D collider.</summary>
    public Vector3d Point3D => GetRequiredWorldPoint(Anchor3D, nameof(Point3D));

    /// <summary>Gets the world-space witness point on the embedded 2D collider.</summary>
    public Vector3d Point2D => GetRequiredWorldPoint(Anchor2D, nameof(Point2D));

    /// <summary>Attempts to resolve the 3D anchor to a world-space point.</summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public bool TryGetPoint3D(out Vector3d point) => Anchor3D.TryGetWorldPoint(out point);

    /// <summary>Attempts to resolve the embedded 2D anchor to a world-space point.</summary>
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
        ContactAnchor reference =
            CreatePlanarReference(origin, rotation, localPoint);
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

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal ExactLever3D GetPlanarXZLeverFrom(
        Vector2d origin,
        Fixed64 rotation,
        Vector2d localPoint) =>
        Anchor2D.GetLeverFrom(
            CreatePlanarReference(origin, rotation, localPoint));

    /// <summary>
    /// Contact normal pointing from the 3D collider toward the embedded 2D collider volume.
    /// </summary>
    public Vector3d Normal3DTo2D { get; }

    /// <summary>Gets the penetration depth along <see cref="Normal3DTo2D"/>.</summary>
    public Fixed64 Depth { get; }

    /// <summary>
    /// Gets whether the exact penetration depth exceeded the scalar domain and
    /// <see cref="Depth"/> therefore contains <see cref="Fixed64.MaxValue"/>.
    /// </summary>
    public bool DepthIsClamped { get; }

    /// <summary>Gets whether the contact carries per-feature material values.</summary>
    public bool HasMaterialOverride { get; }

    /// <summary>Gets the material sampled from the 3D contact feature.</summary>
    public PhysicsMaterial Material3D { get; }

    /// <summary>Gets the material sampled from the embedded 2D contact feature.</summary>
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

    internal static MixedContact FromManifoldContact(in ManifoldContact contact) =>
        new(contact.AnchorA, contact.AnchorB, contact.Normal, contact.Depth,
            contact.DepthIsClamped, contact.HasMaterialOverride, contact.MaterialA, contact.MaterialB);

    internal ManifoldContact ToManifoldContact(int namespace3D = 0, int namespace2D = 0) =>
        new(ContactManifold.CreateContactId(Anchor3D, namespace3D, Anchor2D, namespace2D),
            Anchor3D, Anchor2D, Depth, Normal3DTo2D, HasMaterialOverride,
            Material3D, Material2D, DepthIsClamped, namespace3D, namespace2D);

    internal static bool TrySelectClosingContact(ContactManifold manifold,
        ContactAnchor center3D, Vector3d linear3D, Vector3d angular3D,
        ContactAnchor center2D, Vector2d linear2D, Fixed64 angular2D, out MixedContact contact,
        LSCollider2D? owner2D = null)
    {
        // Preserve a geometric witness for conservative stopping even when every row separates.
        contact = manifold.HasContact ? FromManifoldContact(manifold.PrimaryContact) : default;
        Vector3d spatialLinear2D = ExactContactLever2D.ToSpatial(linear2D);
        Vector3d spatialAngular2D = new(Fixed64.Zero, -angular2D, Fixed64.Zero);
        foreach (ManifoldContact row in manifold)
        {
            ContactAnchor normalAnchor2D = owner2D == null ? row.AnchorB
                : ExactContactLever2D.GetDetectedNormalAnchor(owner2D, row.FeatureNamespaceB, center2D, row.AnchorB);
            ExactLever3D.GetRelativePointVelocityRatio(linear3D, angular3D,
                row.AnchorA.GetLeverFrom(center3D), spatialLinear2D, spatialAngular2D,
                normalAnchor2D.GetLeverFrom(center2D), row.Normal,
                out Signed832 numerator, out Signed832 denominator, secondPlanarAngularVelocity: angular2D);
            // A is always 3D and B is always 2D. Negative relative normal speed closes the row.
            if (numerator.Sign >= 0 || (Fixed64.TryGetSignedRawRatio(numerator, denominator, 0,
                    out Fixed64 speed) && speed >= -Fixed64.Epsilon)) continue;
            contact = FromManifoldContact(row);
            return true;
        }
        return false;
    }

    internal static bool IsClosingProjection(Vector3d displacement, Vector3d outwardNormal)
    {
        if (Vector3d.TryDot(displacement, outwardNormal, out Fixed64 speed))
            return speed < -Fixed64.Epsilon;

        // A rounded unit normal can push a full-range projection beyond Q32.32.
        // Its exact sign still exists, and overflow already proves that its
        // magnitude exceeds the ordinary rounded closing-speed deadband.
        return WideGeometry.GetDifferenceDotProduct3D(
            displacement.X, Fixed64.Zero, displacement.Y, Fixed64.Zero, displacement.Z, Fixed64.Zero,
            outwardNormal.X, Fixed64.Zero, outwardNormal.Y, Fixed64.Zero, outwardNormal.Z, Fixed64.Zero).Sign < 0;
    }

    private ContactAnchor CreatePlanarReference(
        Vector2d origin,
        Fixed64 rotation,
        Vector2d localPoint) =>
        new(
            new Vector3d(
                origin.X,
                Anchor2D.Origin.Y,
                origin.Y),
            FixedQuaternion.FromAxisAngle(Vector3d.Up, -rotation),
            new Vector3d(
                localPoint.X,
                Fixed64.Zero,
                localPoint.Y));

    private static Vector3d GetRequiredWorldPoint(ContactAnchor anchor, string propertyName)
    {
        if (anchor.TryGetWorldPoint(out Vector3d point))
            return point;

        throw new InvalidOperationException(
            $"{propertyName} is outside the representable coordinate range. Use the corresponding TryGet method.");
    }
}
