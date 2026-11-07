//=======================================================================
// PhysicsSweepSphereAgainst2DRequest.cs
//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp;
using Gravitas.Colliders;
using Gravitas.Support;

namespace Gravitas.Queries;

/// <summary>
/// Describes one mixed 3D swept-sphere query against embedded 2D slabs.
/// </summary>
public readonly struct PhysicsSweepSphereAgainst2DRequest
{
    /// <summary>
    /// Creates a mixed 3D sphere sweep against embedded 2D slabs.
    /// </summary>
    public PhysicsSweepSphereAgainst2DRequest(
        Vector3d start,
        Vector3d end,
        Fixed64 radius,
        PhysicsLayerMask layerMask,
        LSCollider? excludedCollider = null,
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
    /// Creates a mixed 3D sphere sweep against all embedded 2D physics layers.
    /// </summary>
    public PhysicsSweepSphereAgainst2DRequest(Vector3d start, Vector3d end, Fixed64 radius)
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
    /// Gets the included 2D target layers.
    /// </summary>
    public PhysicsLayerMask LayerMask { get; }

    /// <summary>
    /// Gets the optional 3D source collider used for mixed-collision filtering.
    /// </summary>
    public LSCollider? ExcludedCollider { get; }

    /// <summary>
    /// Gets whether trigger targets are included.
    /// </summary>
    public bool IncludeTriggers { get; }
}
