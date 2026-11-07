//=======================================================================
// PhysicsSweepSphere3DRequest.cs
//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp;
using Gravitas.Colliders;
using Gravitas.Support;

namespace Gravitas.Queries;

/// <summary>
/// Describes one 3D swept-sphere query in a batched query call.
/// </summary>
public readonly struct PhysicsSweepSphere3DRequest
{
    /// <summary>
    /// Creates a 3D swept-sphere request.
    /// </summary>
    public PhysicsSweepSphere3DRequest(
        Vector3d start,
        Vector3d end,
        Fixed64 radius,
        PhysicsLayerMask layerMask,
        LSCollider? excludedCollider = null)
    {
        Start = start;
        End = end;
        Radius = radius;
        LayerMask = layerMask;
        ExcludedCollider = excludedCollider;
    }

    /// <summary>
    /// Creates a 3D swept-sphere request against all physics layers.
    /// </summary>
    public PhysicsSweepSphere3DRequest(Vector3d start, Vector3d end, Fixed64 radius)
        : this(start, end, radius, PhysicsLayerMask.All)
    {
    }

    /// <summary>
    /// Gets the sphere center's world-space start.
    /// </summary>
    public Vector3d Start { get; }

    /// <summary>
    /// Gets the sphere center's world-space end.
    /// </summary>
    public Vector3d End { get; }

    /// <summary>
    /// Gets the swept sphere radius.
    /// </summary>
    public Fixed64 Radius { get; }

    /// <summary>
    /// Gets the included physics layers.
    /// </summary>
    public PhysicsLayerMask LayerMask { get; }

    /// <summary>
    /// Gets the collider omitted from candidate results, if any.
    /// </summary>
    public LSCollider? ExcludedCollider { get; }
}
