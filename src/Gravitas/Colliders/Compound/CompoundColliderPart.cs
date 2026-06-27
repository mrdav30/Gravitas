//=======================================================================
// CompoundColliderPart.cs
//=======================================================================
// MIT License, Copyright (c) 2026–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp;
using Gravitas.Materials;
using System.Runtime.CompilerServices;

namespace Gravitas.Colliders;

/// <summary>
/// Declares one data-only geometry part owned by an
/// <see cref="LSCompoundCollider"/>.
/// </summary>
public readonly struct CompoundColliderPart
{
    public CompoundColliderPart(ColliderShapeDefinition shape)
        : this(shape, Vector3d.Zero, FixedQuaternion.Identity, Vector3d.One, null)
    { }

    public CompoundColliderPart(ColliderShapeDefinition shape, Vector3d localOffset)
        : this(shape, localOffset, FixedQuaternion.Identity, Vector3d.One, null)
    { }

    public CompoundColliderPart(ColliderShapeDefinition shape, Vector3d localOffset, FixedQuaternion localRotation)
        : this(shape, localOffset, localRotation, Vector3d.One, null)
    { }

    public CompoundColliderPart(
        ColliderShapeDefinition shape,
        Vector3d localOffset,
        FixedQuaternion localRotation,
        Vector3d localScale,
        PhysicsMaterial? material = null)
    {
        shape.EnsureDefined();
        ValidateScale(localScale);

        Shape = shape;
        LocalOffset = localOffset;
        LocalRotation = localRotation;
        LocalScale = localScale;
        _material = material ?? PhysicsMaterial.Default;
        _hasMaterial = material.HasValue;
    }

    private readonly PhysicsMaterial _material;
    private readonly bool _hasMaterial;

    /// <summary>
    /// Gets the authored data-only shape definition for this part.
    /// </summary>
    public ColliderShapeDefinition Shape { get; }

    /// <summary>
    /// Gets the deterministic local center offset applied relative to the
    /// owning compound collider.
    /// </summary>
    public Vector3d LocalOffset { get; }

    /// <summary>
    /// Gets the deterministic local rotation applied relative to the owning
    /// compound collider.
    /// </summary>
    public FixedQuaternion LocalRotation { get; }

    /// <summary>
    /// Gets the deterministic local scale applied relative to the owning
    /// compound collider.
    /// </summary>
    public Vector3d LocalScale { get; }

    /// <summary>
    /// Gets the authored material for this part, or the shape/default material
    /// when no part-level material was supplied.
    /// </summary>
    public PhysicsMaterial Material => TryGetMaterial(out PhysicsMaterial material)
        ? material
        : PhysicsMaterial.Default;

    internal bool HasMaterial
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        get => _hasMaterial || Shape.HasMaterial;
    }

    public static CompoundColliderPart Sphere(Fixed64 radius, Vector3d localOffset) =>
        new(ColliderShapeDefinition.Sphere(radius), localOffset);

    public static CompoundColliderPart Sphere(
        Fixed64 radius,
        Vector3d localOffset,
        PhysicsMaterial material) =>
        new(ColliderShapeDefinition.Sphere(radius), localOffset, FixedQuaternion.Identity, Vector3d.One, material);

    public static CompoundColliderPart Sphere(
        Fixed64 radius,
        Vector3d localOffset,
        FixedQuaternion localRotation,
        Vector3d localScale) =>
        new(ColliderShapeDefinition.Sphere(radius), localOffset, localRotation, localScale);

    public static CompoundColliderPart Capsule(Fixed64 radius, Fixed64 height, Vector3d localOffset) =>
        new(ColliderShapeDefinition.Capsule(radius, height), localOffset);

    public static CompoundColliderPart Capsule(
        Fixed64 radius,
        Fixed64 height,
        Vector3d localOffset,
        PhysicsMaterial material) =>
        new(ColliderShapeDefinition.Capsule(radius, height), localOffset, FixedQuaternion.Identity, Vector3d.One, material);

    public static CompoundColliderPart Capsule(
        Fixed64 radius,
        Fixed64 height,
        Vector3d localOffset,
        FixedQuaternion localRotation,
        Vector3d localScale) =>
        new(ColliderShapeDefinition.Capsule(radius, height), localOffset, localRotation, localScale);

    public static CompoundColliderPart Cuboid(Vector3d size, Vector3d localOffset) =>
        new(ColliderShapeDefinition.Cuboid(size), localOffset);

    public static CompoundColliderPart Cuboid(
        Vector3d size,
        Vector3d localOffset,
        PhysicsMaterial material) =>
        new(ColliderShapeDefinition.Cuboid(size), localOffset, FixedQuaternion.Identity, Vector3d.One, material);

    public static CompoundColliderPart Cuboid(
        Vector3d size,
        Vector3d localOffset,
        FixedQuaternion localRotation,
        Vector3d localScale) =>
        new(ColliderShapeDefinition.Cuboid(size), localOffset, localRotation, localScale);

    public static CompoundColliderPart Cylinder(Fixed64 radius, Fixed64 height, Vector3d localOffset) =>
        new(ColliderShapeDefinition.Cylinder(radius, height), localOffset);

    public static CompoundColliderPart Cylinder(
        Fixed64 radius,
        Fixed64 height,
        Vector3d localOffset,
        PhysicsMaterial material) =>
        new(ColliderShapeDefinition.Cylinder(radius, height), localOffset, FixedQuaternion.Identity, Vector3d.One, material);

    public static CompoundColliderPart Cylinder(
        Fixed64 radius,
        Fixed64 height,
        Vector3d localOffset,
        FixedQuaternion localRotation,
        Vector3d localScale) =>
        new(ColliderShapeDefinition.Cylinder(radius, height), localOffset, localRotation, localScale);

    public static CompoundColliderPart ConvexMesh(
        Vector3d[] vertices,
        int[] triangles,
        Vector3d localOffset,
        MeshInertiaPolicy inertiaPolicy = MeshInertiaPolicy.RequireClosedVolume) =>
        new(ColliderShapeDefinition.ConvexMesh(vertices, triangles, inertiaPolicy), localOffset);

    public static CompoundColliderPart ConvexMesh(
        Vector3d[] vertices,
        int[] triangles,
        Vector3d localOffset,
        PhysicsMaterial material,
        MeshInertiaPolicy inertiaPolicy = MeshInertiaPolicy.RequireClosedVolume) =>
        new(ColliderShapeDefinition.ConvexMesh(vertices, triangles, inertiaPolicy), localOffset, FixedQuaternion.Identity, Vector3d.One, material);

    public static CompoundColliderPart ConvexMesh(
        Vector3d[] vertices,
        int[] triangles,
        Vector3d localOffset,
        FixedQuaternion localRotation,
        Vector3d localScale,
        MeshInertiaPolicy inertiaPolicy = MeshInertiaPolicy.RequireClosedVolume) =>
        new(ColliderShapeDefinition.ConvexMesh(vertices, triangles, inertiaPolicy), localOffset, localRotation, localScale);

    internal bool IsDefault
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        get => Shape.Kind == ColliderShapeDefinitionKind.Undefined;
    }

    internal bool TryGetMaterial(out PhysicsMaterial material)
    {
        if (_hasMaterial)
        {
            material = _material;
            return true;
        }

        if (Shape.HasMaterial)
        {
            material = Shape.Material;
            return true;
        }

        material = PhysicsMaterial.Default;
        return false;
    }

    internal PhysicsMaterial ResolveMaterial(PhysicsMaterial ownerMaterial) =>
        TryGetMaterial(out PhysicsMaterial material) ? material : ownerMaterial;

    private static void ValidateScale(Vector3d localScale)
    {
        SwiftThrowHelper.ThrowIfArgument(
            localScale.X <= Fixed64.Zero || localScale.Y <= Fixed64.Zero || localScale.Z <= Fixed64.Zero,
            nameof(localScale),
            "Compound collider part scale components must be greater than zero.");
    }
}
