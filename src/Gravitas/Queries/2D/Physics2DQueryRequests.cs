//=======================================================================
// Physics2DQueryRequests.cs
//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp;
using Gravitas.Colliders;
using Gravitas.Support;

namespace Gravitas.Queries;

/// <summary>
/// Describes one pure 2D segment raycast in a batched query call.
/// </summary>
public readonly struct PhysicsRaycast2DRequest
{
    /// <summary>
    /// Creates a pure 2D segment raycast request with an include layer mask.
    /// </summary>
    public PhysicsRaycast2DRequest(Vector2d start, Vector2d end, PhysicsLayerMask layerMask)
    {
        Start = start;
        End = end;
        LayerMask = layerMask;
    }

    /// <summary>
    /// Creates a pure 2D segment raycast request against all physics layers.
    /// </summary>
    public PhysicsRaycast2DRequest(Vector2d start, Vector2d end)
        : this(start, end, PhysicsLayerMask.All)
    {
    }

    /// <summary>
    /// Gets the segment start in the X/Z plane.
    /// </summary>
    public Vector2d Start { get; }

    /// <summary>
    /// Gets the segment end in the X/Z plane.
    /// </summary>
    public Vector2d End { get; }

    /// <summary>
    /// Gets the included physics layers.
    /// </summary>
    public PhysicsLayerMask LayerMask { get; }
}

/// <summary>
/// Describes one pure 2D circle overlap in a batched query call.
/// </summary>
public readonly struct PhysicsOverlapCircle2DRequest
{
    /// <summary>
    /// Creates a pure 2D circle-overlap request with an include layer mask.
    /// </summary>
    public PhysicsOverlapCircle2DRequest(Vector2d center, Fixed64 radius, PhysicsLayerMask layerMask)
    {
        Center = center;
        Radius = radius;
        LayerMask = layerMask;
    }

    /// <summary>
    /// Creates a pure 2D circle-overlap request against all physics layers.
    /// </summary>
    public PhysicsOverlapCircle2DRequest(Vector2d center, Fixed64 radius)
        : this(center, radius, PhysicsLayerMask.All)
    {
    }

    /// <summary>
    /// Gets the circle center in the X/Z plane.
    /// </summary>
    public Vector2d Center { get; }

    /// <summary>
    /// Gets the circle radius.
    /// </summary>
    public Fixed64 Radius { get; }

    /// <summary>
    /// Gets the included physics layers.
    /// </summary>
    public PhysicsLayerMask LayerMask { get; }
}

/// <summary>
/// Describes one pure 2D axis-aligned box overlap in a batched query call.
/// </summary>
public readonly struct PhysicsOverlapAabb2DRequest
{
    /// <summary>
    /// Creates a pure 2D axis-aligned box-overlap request with an include layer mask.
    /// </summary>
    public PhysicsOverlapAabb2DRequest(Vector2d center, Vector2d size, PhysicsLayerMask layerMask)
    {
        Center = center;
        Size = size;
        LayerMask = layerMask;
    }

    /// <summary>
    /// Creates a pure 2D axis-aligned box-overlap request against all physics layers.
    /// </summary>
    public PhysicsOverlapAabb2DRequest(Vector2d center, Vector2d size)
        : this(center, size, PhysicsLayerMask.All)
    {
    }

    /// <summary>
    /// Gets the box center in the X/Z plane.
    /// </summary>
    public Vector2d Center { get; }

    /// <summary>
    /// Gets the full box size in the X/Z plane.
    /// </summary>
    public Vector2d Size { get; }

    /// <summary>
    /// Gets the included physics layers.
    /// </summary>
    public PhysicsLayerMask LayerMask { get; }
}

/// <summary>
/// Describes one pure 2D convex polygon overlap by referencing a flat vertex buffer.
/// </summary>
public readonly struct PhysicsOverlapPolygon2DRequest
{
    /// <summary>
    /// Creates a pure 2D convex-polygon overlap request with an include layer mask.
    /// </summary>
    public PhysicsOverlapPolygon2DRequest(int vertexStart, int vertexCount, PhysicsLayerMask layerMask)
    {
        VertexStart = vertexStart;
        VertexCount = vertexCount;
        LayerMask = layerMask;
    }

    /// <summary>
    /// Creates a pure 2D convex-polygon overlap request against all physics layers.
    /// </summary>
    public PhysicsOverlapPolygon2DRequest(int vertexStart, int vertexCount)
        : this(vertexStart, vertexCount, PhysicsLayerMask.All)
    {
    }

    /// <summary>
    /// Gets the first polygon vertex index in the batch's shared vertex buffer.
    /// </summary>
    public int VertexStart { get; }

    /// <summary>
    /// Gets the number of polygon vertices in the shared vertex buffer.
    /// </summary>
    public int VertexCount { get; }

    /// <summary>
    /// Gets the included physics layers.
    /// </summary>
    public PhysicsLayerMask LayerMask { get; }
}

/// <summary>
/// Describes one pure 2D swept-circle query in a batched query call.
/// </summary>
public readonly struct PhysicsSweepCircle2DRequest
{
    /// <summary>
    /// Creates a pure 2D swept-circle request.
    /// </summary>
    public PhysicsSweepCircle2DRequest(
        Vector2d start,
        Vector2d end,
        Fixed64 radius,
        PhysicsLayerMask layerMask,
        LSCollider2D? excludedCollider = null,
        bool includeTriggers = true)
    {
        Start = start;
        End = end;
        Radius = radius;
        LayerMask = layerMask;
        ExcludedCollider = excludedCollider;
        IncludeTriggers = includeTriggers;
    }

    /// <summary>
    /// Creates a pure 2D swept-circle request against all physics layers.
    /// </summary>
    public PhysicsSweepCircle2DRequest(Vector2d start, Vector2d end, Fixed64 radius)
        : this(start, end, radius, PhysicsLayerMask.All)
    {
    }

    /// <summary>
    /// Gets the swept-circle center's start in the X/Z plane.
    /// </summary>
    public Vector2d Start { get; }

    /// <summary>
    /// Gets the swept-circle center's end in the X/Z plane.
    /// </summary>
    public Vector2d End { get; }

    /// <summary>
    /// Gets the swept circle radius.
    /// </summary>
    public Fixed64 Radius { get; }

    /// <summary>
    /// Gets the included physics layers.
    /// </summary>
    public PhysicsLayerMask LayerMask { get; }

    /// <summary>
    /// Gets the collider omitted from candidate results, if any.
    /// </summary>
    public LSCollider2D? ExcludedCollider { get; }

    /// <summary>
    /// Gets whether trigger colliders are included.
    /// </summary>
    public bool IncludeTriggers { get; }
}
