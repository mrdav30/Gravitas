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

/// <summary>
/// Describes one registered 3D capsule-source sweep in a batched query call.
/// </summary>
public readonly struct PhysicsSweepCapsule3DRequest
{
    /// <summary>
    /// Creates a registered capsule-source sweep request.
    /// </summary>
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

    /// <summary>
    /// Gets the registered capsule swept from its current transform.
    /// </summary>
    public LSCapsuleCollider Source { get; }

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

/// <summary>
/// Describes one registered 3D cylinder-source sweep in a batched query call.
/// </summary>
public readonly struct PhysicsSweepCylinder3DRequest
{
    /// <summary>
    /// Creates a registered cylinder-source sweep request.
    /// </summary>
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

    /// <summary>
    /// Gets the registered cylinder swept from its current transform.
    /// </summary>
    public LSCylinderCollider Source { get; }

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

/// <summary>
/// Describes one registered 3D cone-source sweep in a batched query call.
/// </summary>
public readonly struct PhysicsSweepCone3DRequest
{
    /// <summary>
    /// Creates a registered cone-source sweep request.
    /// </summary>
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

    /// <summary>
    /// Gets the registered cone swept from its current transform.
    /// </summary>
    public LSConeCollider Source { get; }

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

/// <summary>
/// Describes one registered convex mesh-source sweep in a batched query call.
/// </summary>
public readonly struct PhysicsSweepConvexMesh3DRequest
{
    /// <summary>
    /// Creates a registered convex-mesh-source sweep request.
    /// </summary>
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

    /// <summary>
    /// Gets the registered convex mesh swept from its current transform.
    /// </summary>
    public LSMeshCollider Source { get; }

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

/// <summary>
/// Describes one registered compound-source sweep in a batched query call.
/// </summary>
public readonly struct PhysicsSweepCompound3DRequest
{
    /// <summary>
    /// Creates a registered compound-source sweep request.
    /// </summary>
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

    /// <summary>
    /// Gets the registered compound collider swept from its current transform.
    /// </summary>
    public LSCompoundCollider Source { get; }

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
