//=======================================================================
// PhysicsOverlapAabb2DRequest.cs
//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp;
using Gravitas.Support;

namespace Gravitas.Queries;

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
