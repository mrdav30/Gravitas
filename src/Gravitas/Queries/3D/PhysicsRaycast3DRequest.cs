//=======================================================================
// PhysicsRaycast3DRequest.cs
//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp;
using Gravitas.Support;

namespace Gravitas.Queries;

/// <summary>
/// Describes one 3D segment raycast in a batched query call.
/// </summary>
public readonly struct PhysicsRaycast3DRequest
{
    /// <summary>
    /// Creates a 3D segment raycast request with an include layer mask.
    /// </summary>
    public PhysicsRaycast3DRequest(Vector3d start, Vector3d end, PhysicsLayerMask layerMask)
    {
        Start = start;
        End = end;
        LayerMask = layerMask;
    }

    /// <summary>
    /// Creates a 3D segment raycast request against all physics layers.
    /// </summary>
    public PhysicsRaycast3DRequest(Vector3d start, Vector3d end)
        : this(start, end, PhysicsLayerMask.All)
    {
    }

    /// <summary>
    /// Gets the segment start in world space.
    /// </summary>
    public Vector3d Start { get; }

    /// <summary>
    /// Gets the segment end in world space.
    /// </summary>
    public Vector3d End { get; }

    /// <summary>
    /// Gets the included physics layers.
    /// </summary>
    public PhysicsLayerMask LayerMask { get; }
}
