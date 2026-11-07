//=======================================================================
// PhysicsOverlapCone3DRequest.cs
//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp;
using Gravitas.Support;

namespace Gravitas.Queries;

/// <summary>
/// Describes one directional 3D cone-volume overlap in a batched query call.
/// </summary>
public readonly struct PhysicsOverlapCone3DRequest
{
    /// <summary>
    /// Creates an apex-origin 3D cone-volume overlap request with an include layer mask.
    /// </summary>
    public PhysicsOverlapCone3DRequest(
        Vector3d origin,
        Vector3d direction,
        Fixed64 length,
        Fixed64 endRadius,
        PhysicsLayerMask layerMask)
    {
        Origin = origin;
        Direction = direction;
        Length = length;
        EndRadius = endRadius;
        LayerMask = layerMask;
    }

    /// <summary>
    /// Creates an apex-origin 3D cone-volume overlap request against all physics layers.
    /// </summary>
    public PhysicsOverlapCone3DRequest(
        Vector3d origin,
        Vector3d direction,
        Fixed64 length,
        Fixed64 endRadius)
        : this(origin, direction, length, endRadius, PhysicsLayerMask.All)
    {
    }

    /// <summary>
    /// Gets the world-space cone apex.
    /// </summary>
    public Vector3d Origin { get; }

    /// <summary>
    /// Gets the cone axis direction.
    /// </summary>
    public Vector3d Direction { get; }

    /// <summary>
    /// Gets the cone length from apex to base.
    /// </summary>
    public Fixed64 Length { get; }

    /// <summary>
    /// Gets the cone radius at its base.
    /// </summary>
    public Fixed64 EndRadius { get; }

    /// <summary>
    /// Gets the included physics layers.
    /// </summary>
    public PhysicsLayerMask LayerMask { get; }
}
