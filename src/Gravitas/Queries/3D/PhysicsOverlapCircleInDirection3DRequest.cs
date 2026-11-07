//=======================================================================
// PhysicsOverlapCircleInDirection3DRequest.cs
//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp;
using Gravitas.Support;

namespace Gravitas.Queries;

/// <summary>
/// Describes one directional 3D X/Z projected-circle overlap in a batched query call.
/// </summary>
public readonly struct PhysicsOverlapCircleInDirection3DRequest
{
    /// <summary>
    /// Creates a directional X/Z projected-circle overlap request with an include layer mask.
    /// </summary>
    public PhysicsOverlapCircleInDirection3DRequest(
        Vector3d position,
        Fixed64 radius,
        Vector3d direction,
        Fixed64 maxDistance,
        PhysicsLayerMask layerMask)
    {
        Position = position;
        Radius = radius;
        Direction = direction;
        MaxDistance = maxDistance;
        LayerMask = layerMask;
    }

    /// <summary>
    /// Creates a directional X/Z projected-circle overlap request against all physics layers.
    /// </summary>
    public PhysicsOverlapCircleInDirection3DRequest(
        Vector3d position,
        Fixed64 radius,
        Vector3d direction,
        Fixed64 maxDistance)
        : this(position, radius, direction, maxDistance, PhysicsLayerMask.All)
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
    /// Gets the direction used to filter exact projected separations.
    /// </summary>
    public Vector3d Direction { get; }

    /// <summary>
    /// Gets the maximum admitted projected separation distance.
    /// </summary>
    public Fixed64 MaxDistance { get; }

    /// <summary>
    /// Gets the included physics layers.
    /// </summary>
    public PhysicsLayerMask LayerMask { get; }
}
