//=======================================================================
// PhysicsRaycast2DRequest.cs
//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp;
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
