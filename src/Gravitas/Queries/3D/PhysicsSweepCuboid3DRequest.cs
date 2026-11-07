//=======================================================================
// PhysicsSweepCuboid3DRequest.cs
//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp;
using Gravitas.Colliders;
using Gravitas.Support;

namespace Gravitas.Queries;

/// <summary>
/// Describes one registered 3D cuboid-source sweep in a batched query call.
/// </summary>
public readonly struct PhysicsSweepCuboid3DRequest
{
    /// <summary>
    /// Creates a registered cuboid-source sweep request.
    /// </summary>
    public PhysicsSweepCuboid3DRequest(
        LSCuboidCollider source,
        Vector3d displacement,
        PhysicsLayerMask layerMask,
        LSCollider? excludedCollider = null,
        bool includeTriggers = true)
    {
        Source = source;
        Displacement = displacement;
        LayerMask = layerMask;
        ExcludedCollider = excludedCollider;
        IncludeTriggers = includeTriggers;
    }

    /// <summary>
    /// Gets the registered cuboid swept from its current transform.
    /// </summary>
    public LSCuboidCollider Source { get; }

    /// <summary>
    /// Gets the world-space sweep displacement.
    /// </summary>
    public Vector3d Displacement { get; }

    /// <summary>
    /// Gets the included physics layers.
    /// </summary>
    public PhysicsLayerMask LayerMask { get; }

    /// <summary>
    /// Gets the collider omitted from candidate results, if any.
    /// </summary>
    public LSCollider? ExcludedCollider { get; }

    /// <summary>
    /// Gets whether trigger colliders are included.
    /// </summary>
    public bool IncludeTriggers { get; }
}
