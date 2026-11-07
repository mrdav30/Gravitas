//=======================================================================
// PhysicsSweepCircleAgainst3DRequest.cs
//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp;
using Gravitas.Colliders;
using Gravitas.Support;

namespace Gravitas.Queries;

/// <summary>
/// Describes one mixed 2D swept-circle query against 3D colliders.
/// </summary>
public readonly struct PhysicsSweepCircleAgainst3DRequest
{
    /// <summary>
    /// Creates a mixed 2D circle-slab sweep against 3D colliders.
    /// </summary>
    public PhysicsSweepCircleAgainst3DRequest(
        Vector2d start,
        Vector2d end,
        Fixed64 radius,
        Fixed64 slabCenterY,
        Fixed64 halfThickness,
        PhysicsLayerMask layerMask,
        LSCollider2D? excludedCollider = null,
        bool includeTriggers = true)
    {
        Start = start;
        End = end;
        Radius = radius;
        SlabCenterY = slabCenterY;
        HalfThickness = halfThickness;
        LayerMask = layerMask;
        ExcludedCollider = excludedCollider;
        IncludeTriggers = includeTriggers;
    }

    /// <summary>
    /// Creates a mixed 2D circle-slab sweep against all 3D physics layers.
    /// </summary>
    public PhysicsSweepCircleAgainst3DRequest(
        Vector2d start,
        Vector2d end,
        Fixed64 radius,
        Fixed64 slabCenterY,
        Fixed64 halfThickness)
        : this(start, end, radius, slabCenterY, halfThickness, PhysicsLayerMask.All)
    {
    }

    /// <summary>
    /// Gets the circle center's start in the X/Z plane.
    /// </summary>
    public Vector2d Start { get; }

    /// <summary>
    /// Gets the circle center's end in the X/Z plane.
    /// </summary>
    public Vector2d End { get; }

    /// <summary>
    /// Gets the swept circle radius.
    /// </summary>
    public Fixed64 Radius { get; }

    /// <summary>
    /// Gets the world-space Y coordinate at the center of the embedded slab.
    /// </summary>
    public Fixed64 SlabCenterY { get; }

    /// <summary>
    /// Gets the slab half-thickness along the Y axis.
    /// </summary>
    public Fixed64 HalfThickness { get; }

    /// <summary>
    /// Gets the included 3D target layers.
    /// </summary>
    public PhysicsLayerMask LayerMask { get; }

    /// <summary>
    /// Gets the optional 2D source collider used for mixed-collision filtering.
    /// </summary>
    public LSCollider2D? ExcludedCollider { get; }

    /// <summary>
    /// Gets whether trigger targets are included.
    /// </summary>
    public bool IncludeTriggers { get; }
}
