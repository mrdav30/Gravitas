//=======================================================================
// PhysicsSweepCircle2DRequest.cs
//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp;
using Gravitas.Colliders;
using Gravitas.Support;

namespace Gravitas.Queries;

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
