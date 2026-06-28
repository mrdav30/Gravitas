//=======================================================================
// ColliderShapeDefinition.cs
//=======================================================================
// MIT License, Copyright (c) 2026–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp;
using Gravitas.Materials;
using System;
using System.Runtime.CompilerServices;

namespace Gravitas.Colliders;

/// <summary>
/// Describes authoritative collider shape input without runtime collider
/// lifecycle state such as IDs, bodies, partitions, pairs, or events.
/// </summary>
public readonly struct ColliderShapeDefinition : IEquatable<ColliderShapeDefinition>
{
    private readonly Vector3d[]? _meshVertices;
    private readonly int[]? _meshTriangles;
    private readonly PhysicsMaterial _material;
    private readonly bool _hasMaterial;

    private ColliderShapeDefinition(
        ColliderShapeDefinitionKind kind,
        Fixed64 radius,
        Fixed64 height,
        Vector3d size,
        MeshInertiaPolicy meshInertiaPolicy,
        Vector3d[]? meshVertices,
        int[]? meshTriangles,
        PhysicsMaterial? material)
    {
        Kind = kind;
        Radius = radius;
        Height = height;
        Size = size;
        MeshInertiaPolicy = meshInertiaPolicy;
        _meshVertices = meshVertices;
        _meshTriangles = meshTriangles;
        _material = material ?? PhysicsMaterial.Default;
        _hasMaterial = material.HasValue;
    }

    /// <summary>
    /// Gets the shape family represented by this definition.
    /// </summary>
    public ColliderShapeDefinitionKind Kind { get; }

    /// <summary>
    /// Gets the unscaled radius used by radius-based shapes.
    /// </summary>
    public Fixed64 Radius { get; }

    /// <summary>
    /// Gets the unscaled height used by capsule and finite-cylinder shapes.
    /// </summary>
    public Fixed64 Height { get; }

    /// <summary>
    /// Gets the unscaled size used by sized shapes. Radius-based definitions
    /// store their normalized runtime size here as well.
    /// </summary>
    public Vector3d Size { get; }

    /// <summary>
    /// Gets the inertia policy used when this definition represents a convex
    /// mesh.
    /// </summary>
    public MeshInertiaPolicy MeshInertiaPolicy { get; }

    /// <summary>
    /// Gets the authored surface material used when this definition creates a
    /// runtime collider directly.
    /// </summary>
    public PhysicsMaterial Material => _hasMaterial ? _material : PhysicsMaterial.Default;

    internal bool HasMaterial => _hasMaterial;

    /// <summary>
    /// Gets the number of local mesh vertices held by a convex mesh definition.
    /// </summary>
    public int MeshVertexCount
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        get => _meshVertices?.Length ?? 0;
    }

    /// <summary>
    /// Gets the number of triangle indices held by a convex mesh definition.
    /// </summary>
    public int MeshTriangleIndexCount
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        get => _meshTriangles?.Length ?? 0;
    }

    /// <summary>
    /// Creates a sphere shape definition.
    /// </summary>
    public static ColliderShapeDefinition Sphere(Fixed64 radius, PhysicsMaterial? material = null)
    {
        ValidateRadius(radius);
        Fixed64 diameter = radius * (Fixed64)2;
        return new(
            ColliderShapeDefinitionKind.Sphere,
            radius,
            diameter,
            new Vector3d(diameter, diameter, diameter),
            MeshInertiaPolicy.RequireClosedVolume,
            null,
            null,
            material);
    }

    /// <summary>
    /// Creates a capsule shape definition.
    /// </summary>
    public static ColliderShapeDefinition Capsule(Fixed64 radius, Fixed64 height, PhysicsMaterial? material = null)
    {
        ValidateRadius(radius);
        ValidateHeight(height);
        Fixed64 diameter = radius * (Fixed64)2;
        return new(
            ColliderShapeDefinitionKind.Capsule,
            radius,
            height,
            new Vector3d(diameter, height, diameter),
            MeshInertiaPolicy.RequireClosedVolume,
            null,
            null,
            material);
    }

    /// <summary>
    /// Creates a cuboid shape definition.
    /// </summary>
    public static ColliderShapeDefinition Cuboid(Vector3d size, PhysicsMaterial? material = null)
    {
        ValidateSize(size);
        return new(
            ColliderShapeDefinitionKind.Cuboid,
            Fixed64.Zero,
            size.Y,
            size,
            MeshInertiaPolicy.RequireClosedVolume,
            null,
            null,
            material);
    }

    /// <summary>
    /// Creates a finite-cylinder shape definition.
    /// </summary>
    public static ColliderShapeDefinition Cylinder(Fixed64 radius, Fixed64 height, PhysicsMaterial? material = null)
    {
        ValidateRadius(radius);
        ValidateHeight(height);
        Fixed64 diameter = radius * (Fixed64)2;
        return new(
            ColliderShapeDefinitionKind.Cylinder,
            radius,
            height,
            new Vector3d(diameter, height, diameter),
            MeshInertiaPolicy.RequireClosedVolume,
            null,
            null,
            material);
    }

    /// <summary>
    /// Creates a finite circular cone shape definition whose local origin is
    /// the bounding center between its base plane and apex.
    /// </summary>
    public static ColliderShapeDefinition Cone(Fixed64 radius, Fixed64 height, PhysicsMaterial? material = null)
    {
        ValidateRadius(radius);
        ValidateHeight(height);
        Fixed64 diameter = radius * (Fixed64)2;
        return new(
            ColliderShapeDefinitionKind.Cone,
            radius,
            height,
            new Vector3d(diameter, height, diameter),
            MeshInertiaPolicy.RequireClosedVolume,
            null,
            null,
            material);
    }

    /// <summary>
    /// Creates a convex mesh shape definition.
    /// </summary>
    public static ColliderShapeDefinition ConvexMesh(
        Vector3d[] vertices,
        int[] triangles,
        MeshInertiaPolicy inertiaPolicy = MeshInertiaPolicy.RequireClosedVolume,
        PhysicsMaterial? material = null)
    {
        ValidateMeshInput(vertices, triangles);
        ValidateMeshInertiaPolicy(inertiaPolicy);

        var vertexSnapshot = new Vector3d[vertices.Length];
        Array.Copy(vertices, vertexSnapshot, vertices.Length);
        var triangleSnapshot = new int[triangles.Length];
        Array.Copy(triangles, triangleSnapshot, triangles.Length);

        return new(
            ColliderShapeDefinitionKind.ConvexMesh,
            Fixed64.Zero,
            Fixed64.Zero,
            Vector3d.Zero,
            inertiaPolicy,
            vertexSnapshot,
            triangleSnapshot,
            material);
    }

    /// <summary>
    /// Gets a local mesh vertex by stable source order.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public Vector3d GetMeshVertex(int index)
    {
        EnsureMeshDefinition();
        SwiftThrowHelper.ThrowIfArrayIndexInvalid(index, _meshVertices!.Length, nameof(index));
        return _meshVertices[index];
    }

    /// <summary>
    /// Gets a mesh triangle index by stable source order.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public int GetMeshTriangleIndex(int index)
    {
        EnsureMeshDefinition();
        SwiftThrowHelper.ThrowIfArrayIndexInvalid(index, _meshTriangles!.Length, nameof(index));
        return _meshTriangles[index];
    }

    /// <summary>
    /// Creates a new unbound runtime collider from this shape definition.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public LSCollider CreateCollider() => CreateRuntimeCollider();

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal LSCollider CreateRuntimeCollider()
    {
        EnsureDefined();

        return Kind switch
        {
            ColliderShapeDefinitionKind.Sphere => new LSSphereCollider(this),
            ColliderShapeDefinitionKind.Capsule => new LSCapsuleCollider(this),
            ColliderShapeDefinitionKind.Cuboid => new LSCuboidCollider(this),
            ColliderShapeDefinitionKind.Cylinder => new LSCylinderCollider(this),
            ColliderShapeDefinitionKind.Cone => new LSConeCollider(this),
            ColliderShapeDefinitionKind.ConvexMesh => new LSMeshCollider(this),
            _ => throw new InvalidOperationException("Unsupported collider shape definition.")
        };
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal Vector3d[] GetMeshVerticesForRuntime()
    {
        EnsureMeshDefinition();
        return _meshVertices!;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal int[] GetMeshTrianglesForRuntime()
    {
        EnsureMeshDefinition();
        return _meshTriangles!;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal void EnsureDefined()
    {
        SwiftThrowHelper.ThrowIfArgument(
            Kind == ColliderShapeDefinitionKind.Undefined,
            nameof(ColliderShapeDefinition),
            "Collider shape definition cannot be default.");
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal void EnsureKind(ColliderShapeDefinitionKind expectedKind)
    {
        EnsureDefined();
        SwiftThrowHelper.ThrowIfArgument(
            Kind != expectedKind,
            nameof(ColliderShapeDefinition),
            $"Collider shape definition must be {expectedKind}.");
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private void EnsureMeshDefinition()
    {
        EnsureKind(ColliderShapeDefinitionKind.ConvexMesh);
        SwiftThrowHelper.ThrowIfArgument(
            _meshVertices == null || _meshTriangles == null,
            nameof(ColliderShapeDefinition),
            "Convex mesh definition is missing mesh data.");
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static void ValidateRadius(Fixed64 radius) =>
        SwiftThrowHelper.ThrowIfArgument(
            radius <= Fixed64.Zero,
            nameof(radius),
            "Collider radius must be greater than zero.");

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static void ValidateHeight(Fixed64 height) =>
        SwiftThrowHelper.ThrowIfArgument(
            height <= Fixed64.Zero,
            nameof(height),
            "Collider height must be greater than zero.");

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static void ValidateSize(Vector3d size) =>
        SwiftThrowHelper.ThrowIfArgument(
            size.X <= Fixed64.Zero || size.Y <= Fixed64.Zero || size.Z <= Fixed64.Zero,
            nameof(size),
            "Collider size components must be greater than zero.");

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static void ValidateMeshInertiaPolicy(MeshInertiaPolicy inertiaPolicy) =>
        SwiftThrowHelper.ThrowIfArgument(
            inertiaPolicy != MeshInertiaPolicy.RequireClosedVolume &&
            inertiaPolicy != MeshInertiaPolicy.SurfaceApproximation,
            nameof(inertiaPolicy),
            "Unsupported mesh inertia policy.");

    private static void ValidateMeshInput(Vector3d[] vertices, int[] triangles)
    {
        SwiftThrowHelper.ThrowIfNull(vertices, nameof(vertices));
        SwiftThrowHelper.ThrowIfNull(triangles, nameof(triangles));
        SwiftThrowHelper.ThrowIfArgument(vertices.Length < 3, nameof(vertices), "Mesh must contain at least three vertices.");
        SwiftThrowHelper.ThrowIfArgument(triangles.Length < 3, nameof(triangles), "Mesh must contain at least one triangle.");
        SwiftThrowHelper.ThrowIfArgument(triangles.Length % 3 != 0, nameof(triangles), "Triangle index count must be divisible by three.");
        SwiftThrowHelper.ThrowIfArgument(vertices.Length > PhysicsMesh.MaxVertexCount, nameof(vertices), "Mesh vertex count exceeds the deterministic runtime limit.");
        SwiftThrowHelper.ThrowIfArgument(triangles.Length / 3 > PhysicsMesh.MaxTriangleCount, nameof(triangles), "Mesh triangle count exceeds the deterministic runtime limit.");

        for (int i = 0; i < triangles.Length; i++)
        {
            int vertexIndex = triangles[i];
            SwiftThrowHelper.ThrowIfArgument(
                vertexIndex < 0 || vertexIndex >= vertices.Length,
                nameof(triangles),
                "Triangle index references a vertex outside the source vertex array.");
        }
    }

    public bool Equals(ColliderShapeDefinition other)
    {
        if (Kind != other.Kind
            || Radius != other.Radius
            || Height != other.Height
            || Size != other.Size
            || MeshInertiaPolicy != other.MeshInertiaPolicy
            || _hasMaterial != other._hasMaterial
            || (_hasMaterial && _material != other._material)
            || MeshVertexCount != other.MeshVertexCount
            || MeshTriangleIndexCount != other.MeshTriangleIndexCount)
        {
            return false;
        }

        for (int i = 0; i < MeshVertexCount; i++)
        {
            if (_meshVertices![i] != other._meshVertices![i])
                return false;
        }

        for (int i = 0; i < MeshTriangleIndexCount; i++)
        {
            if (_meshTriangles![i] != other._meshTriangles![i])
                return false;
        }

        return true;
    }

    public override bool Equals(object? obj) =>
        obj is ColliderShapeDefinition other && Equals(other);

    public override int GetHashCode()
    {
        unchecked
        {
            int hash = 17;
            hash = hash * 31 + Kind.GetHashCode();
            hash = hash * 31 + Radius.GetHashCode();
            hash = hash * 31 + Height.GetHashCode();
            hash = hash * 31 + Size.GetHashCode();
            hash = hash * 31 + MeshInertiaPolicy.GetHashCode();
            hash = hash * 31 + _hasMaterial.GetHashCode();
            if (_hasMaterial)
                hash = hash * 31 + _material.GetHashCode();

            for (int i = 0; i < MeshVertexCount; i++)
                hash = hash * 31 + _meshVertices![i].GetHashCode();
            for (int i = 0; i < MeshTriangleIndexCount; i++)
                hash = hash * 31 + _meshTriangles![i].GetHashCode();

            return hash;
        }
    }

    public static bool operator ==(ColliderShapeDefinition left, ColliderShapeDefinition right) =>
        left.Equals(right);

    public static bool operator !=(ColliderShapeDefinition left, ColliderShapeDefinition right) =>
        !left.Equals(right);
}
