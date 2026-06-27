//=======================================================================
// MixedContact.cs
//=======================================================================
// MIT License, Copyright (c) 2026–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp;
using Gravitas.Materials;

namespace Gravitas.CollisionHandling;

/// <summary>
/// Contact generated between one 3D collider and one embedded 2D collider.
/// </summary>
public readonly struct MixedContact
{
    public MixedContact(Vector3d point3D, Vector3d point2D, Vector3d normal3DTo2D, Fixed64 depth)
        : this(point3D, point2D, normal3DTo2D, depth, hasMaterialOverride: false, default, default)
    { }

    internal MixedContact(
        Vector3d point3D,
        Vector3d point2D,
        Vector3d normal3DTo2D,
        Fixed64 depth,
        PhysicsMaterial material3D,
        PhysicsMaterial material2D)
        : this(point3D, point2D, normal3DTo2D, depth, hasMaterialOverride: true, material3D, material2D)
    { }

    private MixedContact(
        Vector3d point3D,
        Vector3d point2D,
        Vector3d normal3DTo2D,
        Fixed64 depth,
        bool hasMaterialOverride,
        PhysicsMaterial material3D,
        PhysicsMaterial material2D)
    {
        Point3D = point3D;
        Point2D = point2D;
        Normal3DTo2D = normal3DTo2D;
        Depth = depth;
        HasMaterialOverride = hasMaterialOverride;
        Material3D = hasMaterialOverride ? material3D : PhysicsMaterial.Default;
        Material2D = hasMaterialOverride ? material2D : PhysicsMaterial.Default;
        HasContact = true;
    }

    public bool HasContact { get; }

    public Vector3d Point3D { get; }

    public Vector3d Point2D { get; }

    /// <summary>
    /// Contact normal pointing from the 3D collider toward the embedded 2D collider volume.
    /// </summary>
    public Vector3d Normal3DTo2D { get; }

    public Fixed64 Depth { get; }

    public bool HasMaterialOverride { get; }

    public PhysicsMaterial Material3D { get; }

    public PhysicsMaterial Material2D { get; }

    internal MixedContact WithMaterialOverride(PhysicsMaterial material3D, PhysicsMaterial material2D) =>
        new(Point3D, Point2D, Normal3DTo2D, Depth, material3D, material2D);

    internal MixedContact WithFallbackMaterials(PhysicsMaterial material3D, PhysicsMaterial material2D) =>
        HasMaterialOverride ? this : WithMaterialOverride(material3D, material2D);
}
