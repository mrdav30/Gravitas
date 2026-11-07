//=======================================================================
// PhysicsOverlapCircle3DRequest.cs
//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp;
using Gravitas.Support;

namespace Gravitas.Queries;

/// <summary>
/// Describes one 3D X/Z circle overlap in a batched query call.
/// </summary>
public readonly struct PhysicsOverlapCircle3DRequest
{
    /// <summary>
    /// Creates a 3D collider-projection overlap request with an X/Z circle and include layer mask.
    /// </summary>
    public PhysicsOverlapCircle3DRequest(Vector3d position, Fixed64 radius, PhysicsLayerMask layerMask)
    {
        Position = position;
        Radius = radius;
        LayerMask = layerMask;
    }

    /// <summary>
    /// Creates a 3D collider-projection overlap request against all physics layers.
    /// </summary>
    public PhysicsOverlapCircle3DRequest(Vector3d position, Fixed64 radius)
        : this(position, radius, PhysicsLayerMask.All)
    {
    }

    /// <summary>
    /// Gets the circle center; only its X/Z components are used.
    /// </summary>
    public Vector3d Position { get; }

    /// <summary>
    /// Gets the projected circle radius.
    /// </summary>
    public Fixed64 Radius { get; }

    /// <summary>
    /// Gets the included physics layers.
    /// </summary>
    public PhysicsLayerMask LayerMask { get; }
}
