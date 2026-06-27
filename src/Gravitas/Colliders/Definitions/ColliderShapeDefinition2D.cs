//=======================================================================
// ColliderShapeDefinition2D.cs
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
/// Describes authoritative pure 2D collider shape input without runtime
/// collider lifecycle state such as IDs, bodies, partitions, pairs, or events.
/// </summary>
public readonly struct ColliderShapeDefinition2D : IEquatable<ColliderShapeDefinition2D>
{
    private readonly Vector2d[]? _polygonVertices;
    private readonly PhysicsMaterial _material;
    private readonly bool _hasMaterial;

    private ColliderShapeDefinition2D(
        ColliderShapeDefinition2DKind kind,
        Fixed64 radius,
        Vector2d size,
        Vector2d[]? polygonVertices,
        PhysicsMaterial? material)
    {
        Kind = kind;
        Radius = radius;
        Size = size;
        _polygonVertices = polygonVertices;
        _material = material ?? PhysicsMaterial.Default;
        _hasMaterial = material.HasValue;
    }

    /// <summary>
    /// Gets the shape family represented by this definition.
    /// </summary>
    public ColliderShapeDefinition2DKind Kind { get; }

    /// <summary>
    /// Gets the unscaled radius used by circle definitions.
    /// </summary>
    public Fixed64 Radius { get; }

    /// <summary>
    /// Gets the unscaled size used by sized definitions.
    /// </summary>
    public Vector2d Size { get; }

    /// <summary>
    /// Gets the authored surface material used when this definition creates a
    /// runtime 2D collider directly.
    /// </summary>
    public PhysicsMaterial Material => _hasMaterial ? _material : PhysicsMaterial.Default;

    internal bool HasMaterial => _hasMaterial;

    /// <summary>
    /// Gets the number of local polygon vertices held by this definition.
    /// </summary>
    public int PolygonVertexCount
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        get => _polygonVertices?.Length ?? 0;
    }

    /// <summary>
    /// Creates a circle shape definition.
    /// </summary>
    public static ColliderShapeDefinition2D Circle(Fixed64 radius, PhysicsMaterial? material = null)
    {
        ValidateRadius(radius);
        Fixed64 diameter = radius * (Fixed64)2;
        return new(
            ColliderShapeDefinition2DKind.Circle,
            radius,
            new Vector2d(diameter, diameter),
            null,
            material);
    }

    /// <summary>
    /// Creates an axis-aligned box shape definition.
    /// </summary>
    public static ColliderShapeDefinition2D AABBox(Vector2d size, PhysicsMaterial? material = null)
    {
        ValidateSize(size);
        return new(ColliderShapeDefinition2DKind.AABBox, Fixed64.Zero, size, null, material);
    }

    /// <summary>
    /// Creates an axis-aligned box shape definition.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static ColliderShapeDefinition2D AABox(Vector2d size, PhysicsMaterial? material = null) =>
        AABBox(size, material);

    /// <summary>
    /// Creates a convex polygon shape definition.
    /// </summary>
    public static ColliderShapeDefinition2D ConvexPolygon(params Vector2d[] vertices) =>
        ConvexPolygon(null, vertices);

    /// <summary>
    /// Creates a convex polygon shape definition.
    /// </summary>
    public static ColliderShapeDefinition2D ConvexPolygon(PhysicsMaterial? material, params Vector2d[] vertices)
    {
        SwiftThrowHelper.ThrowIfNull(vertices, nameof(vertices));
        LSPolygonCollider2D.ValidateConvexPolygon(vertices);

        var vertexSnapshot = new Vector2d[vertices.Length];
        Array.Copy(vertices, vertexSnapshot, vertices.Length);
        return new(
            ColliderShapeDefinition2DKind.ConvexPolygon,
            Fixed64.Zero,
            Vector2d.Zero,
            vertexSnapshot,
            material);
    }

    /// <summary>
    /// Gets a local polygon vertex by stable source order.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public Vector2d GetPolygonVertex(int index)
    {
        EnsurePolygonDefinition();
        SwiftThrowHelper.ThrowIfArrayIndexInvalid(index, _polygonVertices!.Length, nameof(index));
        return _polygonVertices[index];
    }

    /// <summary>
    /// Creates a new unbound runtime 2D collider from this shape definition.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public LSCollider2D CreateCollider() => CreateRuntimeCollider();

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal LSCollider2D CreateRuntimeCollider()
    {
        EnsureDefined();

        return Kind switch
        {
            ColliderShapeDefinition2DKind.Circle => new LSCircleCollider2D(this),
            ColliderShapeDefinition2DKind.AABBox => new LSAABBoxCollider2D(this),
            ColliderShapeDefinition2DKind.ConvexPolygon => new LSPolygonCollider2D(this),
            _ => throw new InvalidOperationException("Unsupported 2D collider shape definition.")
        };
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal Vector2d[] GetPolygonVerticesForRuntime()
    {
        EnsurePolygonDefinition();
        return _polygonVertices!;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal void EnsureDefined()
    {
        SwiftThrowHelper.ThrowIfArgument(
            Kind == ColliderShapeDefinition2DKind.Undefined,
            nameof(ColliderShapeDefinition2D),
            "2D collider shape definition cannot be default.");
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal void EnsureKind(ColliderShapeDefinition2DKind expectedKind)
    {
        EnsureDefined();
        SwiftThrowHelper.ThrowIfArgument(
            Kind != expectedKind,
            nameof(ColliderShapeDefinition2D),
            $"2D collider shape definition must be {expectedKind}.");
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private void EnsurePolygonDefinition()
    {
        EnsureKind(ColliderShapeDefinition2DKind.ConvexPolygon);
        SwiftThrowHelper.ThrowIfArgument(
            _polygonVertices == null,
            nameof(ColliderShapeDefinition2D),
            "2D convex polygon definition is missing polygon data.");
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static void ValidateRadius(Fixed64 radius) =>
        SwiftThrowHelper.ThrowIfArgument(
            radius <= Fixed64.Zero,
            nameof(radius),
            "2D collider radius must be greater than zero.");

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static void ValidateSize(Vector2d size) =>
        SwiftThrowHelper.ThrowIfArgument(
            size.X <= Fixed64.Zero || size.Y <= Fixed64.Zero,
            nameof(size),
            "2D collider size components must be greater than zero.");

    public bool Equals(ColliderShapeDefinition2D other)
    {
        if (Kind != other.Kind
            || Radius != other.Radius
            || Size != other.Size
            || _hasMaterial != other._hasMaterial
            || (_hasMaterial && _material != other._material)
            || PolygonVertexCount != other.PolygonVertexCount)
        {
            return false;
        }

        for (int i = 0; i < PolygonVertexCount; i++)
        {
            if (_polygonVertices![i] != other._polygonVertices![i])
                return false;
        }

        return true;
    }

    public override bool Equals(object? obj) =>
        obj is ColliderShapeDefinition2D other && Equals(other);

    public override int GetHashCode()
    {
        unchecked
        {
            int hash = 17;
            hash = hash * 31 + Kind.GetHashCode();
            hash = hash * 31 + Radius.GetHashCode();
            hash = hash * 31 + Size.GetHashCode();
            hash = hash * 31 + _hasMaterial.GetHashCode();
            if (_hasMaterial)
                hash = hash * 31 + _material.GetHashCode();

            for (int i = 0; i < PolygonVertexCount; i++)
                hash = hash * 31 + _polygonVertices![i].GetHashCode();

            return hash;
        }
    }

    public static bool operator ==(ColliderShapeDefinition2D left, ColliderShapeDefinition2D right) =>
        left.Equals(right);

    public static bool operator !=(ColliderShapeDefinition2D left, ColliderShapeDefinition2D right) =>
        !left.Equals(right);
}
