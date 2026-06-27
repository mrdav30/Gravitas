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
    public PhysicsRaycast2DRequest(Vector2d start, Vector2d end, PhysicsLayerMask layerMask)
    {
        Start = start;
        End = end;
        LayerMask = layerMask;
    }

    public PhysicsRaycast2DRequest(Vector2d start, Vector2d end)
        : this(start, end, PhysicsLayerMask.All)
    {
    }

    public Vector2d Start { get; }

    public Vector2d End { get; }

    public PhysicsLayerMask LayerMask { get; }
}

/// <summary>
/// Describes one pure 2D circle overlap in a batched query call.
/// </summary>
public readonly struct PhysicsOverlapCircle2DRequest
{
    public PhysicsOverlapCircle2DRequest(Vector2d center, Fixed64 radius, PhysicsLayerMask layerMask)
    {
        Center = center;
        Radius = radius;
        LayerMask = layerMask;
    }

    public PhysicsOverlapCircle2DRequest(Vector2d center, Fixed64 radius)
        : this(center, radius, PhysicsLayerMask.All)
    {
    }

    public Vector2d Center { get; }

    public Fixed64 Radius { get; }

    public PhysicsLayerMask LayerMask { get; }
}

/// <summary>
/// Describes one pure 2D axis-aligned box overlap in a batched query call.
/// </summary>
public readonly struct PhysicsOverlapAabb2DRequest
{
    public PhysicsOverlapAabb2DRequest(Vector2d center, Vector2d size, PhysicsLayerMask layerMask)
    {
        Center = center;
        Size = size;
        LayerMask = layerMask;
    }

    public PhysicsOverlapAabb2DRequest(Vector2d center, Vector2d size)
        : this(center, size, PhysicsLayerMask.All)
    {
    }

    public Vector2d Center { get; }

    public Vector2d Size { get; }

    public PhysicsLayerMask LayerMask { get; }
}

/// <summary>
/// Describes one pure 2D convex polygon overlap by referencing a flat vertex buffer.
/// </summary>
public readonly struct PhysicsOverlapPolygon2DRequest
{
    public PhysicsOverlapPolygon2DRequest(int vertexStart, int vertexCount, PhysicsLayerMask layerMask)
    {
        VertexStart = vertexStart;
        VertexCount = vertexCount;
        LayerMask = layerMask;
    }

    public PhysicsOverlapPolygon2DRequest(int vertexStart, int vertexCount)
        : this(vertexStart, vertexCount, PhysicsLayerMask.All)
    {
    }

    public int VertexStart { get; }

    public int VertexCount { get; }

    public PhysicsLayerMask LayerMask { get; }
}

/// <summary>
/// Describes one pure 2D swept-circle query in a batched query call.
/// </summary>
public readonly struct PhysicsSweepCircle2DRequest
{
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

    public PhysicsSweepCircle2DRequest(Vector2d start, Vector2d end, Fixed64 radius)
        : this(start, end, radius, PhysicsLayerMask.All)
    {
    }

    public Vector2d Start { get; }

    public Vector2d End { get; }

    public Fixed64 Radius { get; }

    public PhysicsLayerMask LayerMask { get; }

    public LSCollider2D? ExcludedCollider { get; }

    public bool IncludeTriggers { get; }
}
