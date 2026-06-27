//=======================================================================
// PhysicsMixedQueryRequests.cs
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

    public PhysicsSweepSphereAgainst2DRequest(Vector3d start, Vector3d end, Fixed64 radius)
        : this(start, end, radius, PhysicsLayerMask.All)
    {
    }

    public Vector3d Start { get; }

    public Vector3d End { get; }

    public Fixed64 Radius { get; }

    public PhysicsLayerMask LayerMask { get; }

    public LSCollider? ExcludedCollider { get; }

    public bool IncludeTriggers { get; }
}

/// <summary>
/// Describes one mixed 2D swept-circle query against 3D colliders.
/// </summary>
public readonly struct PhysicsSweepCircleAgainst3DRequest
{
    public PhysicsSweepCircleAgainst3DRequest(
        Vector2d start,
        Vector2d end,
        Fixed64 radius,
        Fixed64 slabCenterY,
        Fixed64 halfThickness,
        PhysicsLayerMask layerMask,
        LSCollider2D? excludedCollider = null,
        bool includeTriggers = true)
    {
        Start = start;
        End = end;
        Radius = radius;
        SlabCenterY = slabCenterY;
        HalfThickness = halfThickness;
        LayerMask = layerMask;
        ExcludedCollider = excludedCollider;
        IncludeTriggers = includeTriggers;
    }

    public PhysicsSweepCircleAgainst3DRequest(
        Vector2d start,
        Vector2d end,
        Fixed64 radius,
        Fixed64 slabCenterY,
        Fixed64 halfThickness)
        : this(start, end, radius, slabCenterY, halfThickness, PhysicsLayerMask.All)
    {
    }

    public Vector2d Start { get; }

    public Vector2d End { get; }

    public Fixed64 Radius { get; }

    public Fixed64 SlabCenterY { get; }

    public Fixed64 HalfThickness { get; }

    public PhysicsLayerMask LayerMask { get; }

    public LSCollider2D? ExcludedCollider { get; }

    public bool IncludeTriggers { get; }
}
