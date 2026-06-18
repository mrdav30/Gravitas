using FixedMathSharp;
using System.Runtime.CompilerServices;

namespace Gravitas.Colliders;

/// <summary>
/// Declares one data-only geometry part owned by an
/// <see cref="LSCompoundCollider"/>.
/// </summary>
public readonly struct CompoundColliderPart
{
    public CompoundColliderPart(ColliderShapeDefinition shape)
        : this(shape, Vector3d.Zero, FixedQuaternion.Identity, Vector3d.One)
    { }

    public CompoundColliderPart(ColliderShapeDefinition shape, Vector3d localOffset)
        : this(shape, localOffset, FixedQuaternion.Identity, Vector3d.One)
    { }

    public CompoundColliderPart(ColliderShapeDefinition shape, Vector3d localOffset, FixedQuaternion localRotation)
        : this(shape, localOffset, localRotation, Vector3d.One)
    { }

    public CompoundColliderPart(
        ColliderShapeDefinition shape,
        Vector3d localOffset,
        FixedQuaternion localRotation,
        Vector3d localScale)
    {
        shape.EnsureDefined();
        ValidateScale(localScale);

        Shape = shape;
        LocalOffset = localOffset;
        LocalRotation = localRotation;
        LocalScale = localScale;
    }

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

    public static CompoundColliderPart Sphere(Fixed64 radius, Vector3d localOffset) =>
        new(ColliderShapeDefinition.Sphere(radius), localOffset);

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
        FixedQuaternion localRotation,
        Vector3d localScale) =>
        new(ColliderShapeDefinition.Capsule(radius, height), localOffset, localRotation, localScale);

    public static CompoundColliderPart Cuboid(Vector3d size, Vector3d localOffset) =>
        new(ColliderShapeDefinition.Cuboid(size), localOffset);

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
        FixedQuaternion localRotation,
        Vector3d localScale,
        MeshInertiaPolicy inertiaPolicy = MeshInertiaPolicy.RequireClosedVolume) =>
        new(ColliderShapeDefinition.ConvexMesh(vertices, triangles, inertiaPolicy), localOffset, localRotation, localScale);

    internal bool IsDefault
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        get => Shape.Kind == ColliderShapeDefinitionKind.Undefined;
    }

    private static void ValidateScale(Vector3d localScale)
    {
        SwiftThrowHelper.ThrowIfArgument(
            localScale.X <= Fixed64.Zero || localScale.Y <= Fixed64.Zero || localScale.Z <= Fixed64.Zero,
            nameof(localScale),
            "Compound collider part scale components must be greater than zero.");
    }
}
