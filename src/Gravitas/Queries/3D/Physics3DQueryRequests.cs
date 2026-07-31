//=======================================================================
// Physics3DQueryRequests.cs
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

    public PhysicsSweepSphere3DRequest(Vector3d start, Vector3d end, Fixed64 radius)
        : this(start, end, radius, PhysicsLayerMask.All)
    {
    }

    public Vector3d Start { get; }

    public Vector3d End { get; }

    public Fixed64 Radius { get; }

    public PhysicsLayerMask LayerMask { get; }

    public LSCollider? ExcludedCollider { get; }
}

/// <summary>
/// Describes one registered 3D capsule-source sweep in a batched query call.
/// </summary>
public readonly struct PhysicsSweepCapsule3DRequest
{
    public PhysicsSweepCapsule3DRequest(
        LSCapsuleCollider source,
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

    public LSCapsuleCollider Source { get; }

    public Vector3d Displacement { get; }

    public PhysicsLayerMask LayerMask { get; }

    public LSCollider? ExcludedCollider { get; }

    public bool IncludeTriggers { get; }
}

/// <summary>
/// Describes one registered 3D cuboid-source sweep in a batched query call.
/// </summary>
public readonly struct PhysicsSweepCuboid3DRequest
{
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

    public LSCuboidCollider Source { get; }

    public Vector3d Displacement { get; }

    public PhysicsLayerMask LayerMask { get; }

    public LSCollider? ExcludedCollider { get; }

    public bool IncludeTriggers { get; }
}

/// <summary>
/// Describes one registered 3D cylinder-source sweep in a batched query call.
/// </summary>
public readonly struct PhysicsSweepCylinder3DRequest
{
    public PhysicsSweepCylinder3DRequest(
        LSCylinderCollider source,
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

    public LSCylinderCollider Source { get; }

    public Vector3d Displacement { get; }

    public PhysicsLayerMask LayerMask { get; }

    public LSCollider? ExcludedCollider { get; }

    public bool IncludeTriggers { get; }
}

/// <summary>
/// Describes one registered 3D cone-source sweep in a batched query call.
/// </summary>
public readonly struct PhysicsSweepCone3DRequest
{
    public PhysicsSweepCone3DRequest(
        LSConeCollider source,
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

    public LSConeCollider Source { get; }

    public Vector3d Displacement { get; }

    public PhysicsLayerMask LayerMask { get; }

    public LSCollider? ExcludedCollider { get; }

    public bool IncludeTriggers { get; }
}

/// <summary>
/// Describes one registered convex mesh-source sweep in a batched query call.
/// </summary>
public readonly struct PhysicsSweepConvexMesh3DRequest
{
    public PhysicsSweepConvexMesh3DRequest(
        LSMeshCollider source,
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

    public LSMeshCollider Source { get; }

    public Vector3d Displacement { get; }

    public PhysicsLayerMask LayerMask { get; }

    public LSCollider? ExcludedCollider { get; }

    public bool IncludeTriggers { get; }
}

/// <summary>
/// Describes one registered compound-source sweep in a batched query call.
/// </summary>
public readonly struct PhysicsSweepCompound3DRequest
{
    public PhysicsSweepCompound3DRequest(
        LSCompoundCollider source,
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

    public LSCompoundCollider Source { get; }

    public Vector3d Displacement { get; }

    public PhysicsLayerMask LayerMask { get; }

    public LSCollider? ExcludedCollider { get; }

    public bool IncludeTriggers { get; }
}

/// <summary>
/// Describes one 3D X/Z circle overlap in a batched query call.
/// </summary>
public readonly struct PhysicsOverlapCircle3DRequest
{
    public PhysicsOverlapCircle3DRequest(Vector3d position, Fixed64 radius, PhysicsLayerMask layerMask)
    {
        Position = position;
        Radius = radius;
        LayerMask = layerMask;
    }

    public PhysicsOverlapCircle3DRequest(Vector3d position, Fixed64 radius)
        : this(position, radius, PhysicsLayerMask.All)
    {
    }

    public Vector3d Position { get; }

    public Fixed64 Radius { get; }

    public PhysicsLayerMask LayerMask { get; }
}

/// <summary>
/// Describes one directional 3D X/Z projected-circle overlap in a batched query call.
/// </summary>
public readonly struct PhysicsOverlapCircleInDirection3DRequest
{
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

    public PhysicsOverlapCircleInDirection3DRequest(
        Vector3d position,
        Fixed64 radius,
        Vector3d direction,
        Fixed64 maxDistance)
        : this(position, radius, direction, maxDistance, PhysicsLayerMask.All)
    {
    }

    public Vector3d Position { get; }

    public Fixed64 Radius { get; }

    public Vector3d Direction { get; }

    public Fixed64 MaxDistance { get; }

    public PhysicsLayerMask LayerMask { get; }
}

/// <summary>
/// Describes one directional 3D cone-volume overlap in a batched query call.
/// </summary>
public readonly struct PhysicsOverlapCone3DRequest
{
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

    public PhysicsOverlapCone3DRequest(
        Vector3d origin,
        Vector3d direction,
        Fixed64 length,
        Fixed64 endRadius)
        : this(origin, direction, length, endRadius, PhysicsLayerMask.All)
    {
    }

    public Vector3d Origin { get; }

    public Vector3d Direction { get; }

    public Fixed64 Length { get; }

    public Fixed64 EndRadius { get; }

    public PhysicsLayerMask LayerMask { get; }
}
