//=======================================================================
// PhysicsOverlapCircle2DRequest.cs
//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp;
using Gravitas.Support;

namespace Gravitas.Queries;

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
