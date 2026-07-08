//=======================================================================
// ConvexColliderSupport.cs
//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp;
using Gravitas.Colliders;
using System;
using System.Runtime.CompilerServices;

namespace Gravitas.CollisionHandling;

internal static class ConvexColliderSupport
{
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static bool IsSupported(LSCollider collider) =>
        collider is LSSphereCollider
            or LSCapsuleCollider
            or LSCuboidCollider
            or LSCylinderCollider
            or LSConeCollider
            or LSMeshCollider { Mode: MeshColliderMode.Convex };

    public static Vector3d Support(LSCollider collider, Vector3d direction)
    {
        Vector3d normal = direction.MagnitudeSquared > Fixed64.Epsilon
            ? direction.Normalized
            : Vector3d.Right;

        return collider switch
        {
            LSSphereCollider sphere => sphere.Center + normal * sphere.ScaledRadius,
            LSCapsuleCollider capsule => SupportCapsule(capsule, normal),
            LSCuboidCollider cuboid => SupportVertices(cuboid.Vertices, normal),
            LSCylinderCollider cylinder => SupportCylinder(cylinder, normal),
            LSConeCollider cone => SupportCone(cone, normal),
            LSMeshCollider mesh when mesh.Mode == MeshColliderMode.Convex => mesh.Mesh.GetSupportVertexWorld(normal),
            _ => throw new NotSupportedException(
                $"Convex support mapping does not support {collider.GetType().Name}.")
        };
    }

    public static FixedRange ProjectOntoAxis(LSCollider collider, Vector3d axis)
    {
        Vector3d normalized = axis.MagnitudeSquared > Fixed64.Epsilon
            ? axis.Normalized
            : Vector3d.Right;
        Fixed64 min = Vector3d.Dot(Support(collider, -normalized), normalized);
        Fixed64 max = Vector3d.Dot(Support(collider, normalized), normalized);
        return new FixedRange(min, max);
    }

    public static bool Intersects(LSCollider first, LSCollider second)
    {
        if (!IsSupported(first) || !IsSupported(second))
            return false;

        Span<Vector3d> simplex = stackalloc Vector3d[4];
        int count = 0;
        Vector3d direction = second.Center - first.Center;
        if (direction.MagnitudeSquared <= Fixed64.Epsilon)
            direction = Vector3d.Right;

        simplex[count++] = SupportMinkowski(first, second, direction);
        direction = -simplex[0];
        if (direction.MagnitudeSquared <= Fixed64.Epsilon)
            return true;

        for (int i = 0; i < 32; i++)
        {
            Vector3d point = SupportMinkowski(first, second, direction);
            if (Vector3d.Dot(point, direction) < -Fixed64.Epsilon)
                return false;

            GjkSimplexPolicy.AddPoint(simplex, ref count, point);
            if (GjkSimplexPolicy.Update(simplex, ref count, ref direction))
                return true;

            if (direction.MagnitudeSquared <= Fixed64.Epsilon)
                return true;
        }

        return false;
    }

    public static bool IntersectsConeVolume(
        LSCollider collider,
        Vector3d apex,
        Vector3d axis,
        Fixed64 length,
        Fixed64 endRadius)
    {
        if (!IsSupported(collider))
            return false;

        Span<Vector3d> simplex = stackalloc Vector3d[4];
        int count = 0;
        Vector3d coneCenter = apex + axis * (length * Fixed64.Half);
        Vector3d direction = collider.Center - coneCenter;
        if (direction.MagnitudeSquared <= Fixed64.Epsilon)
            direction = Vector3d.Right;

        simplex[count++] = SupportMinkowskiConeCollider(collider, apex, axis, length, endRadius, direction);
        direction = -simplex[0];
        if (direction.MagnitudeSquared <= Fixed64.Epsilon)
            return true;

        for (int i = 0; i < 32; i++)
        {
            Vector3d point = SupportMinkowskiConeCollider(collider, apex, axis, length, endRadius, direction);
            if (Vector3d.Dot(point, direction) < -Fixed64.Epsilon)
                return false;

            GjkSimplexPolicy.AddPoint(simplex, ref count, point);
            if (GjkSimplexPolicy.Update(simplex, ref count, ref direction))
                return true;

            if (direction.MagnitudeSquared <= Fixed64.Epsilon)
                return true;
        }

        return false;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static Vector3d SupportMinkowski(LSCollider first, LSCollider second, Vector3d direction) =>
        Support(first, direction) - Support(second, -direction);

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static Vector3d SupportMinkowskiConeCollider(
        LSCollider collider,
        Vector3d apex,
        Vector3d axis,
        Fixed64 length,
        Fixed64 endRadius,
        Vector3d direction) =>
        SupportQueryCone(apex, axis, length, endRadius, direction) - Support(collider, -direction);

    private static Vector3d SupportQueryCone(
        Vector3d apex,
        Vector3d axis,
        Fixed64 length,
        Fixed64 endRadius,
        Vector3d direction)
    {
        Vector3d baseCenter = apex + axis * length;
        Vector3d radial = direction - axis * Vector3d.Dot(direction, axis);
        Fixed64 radialMagnitude = radial.Magnitude;
        Vector3d baseSupport = radialMagnitude > Fixed64.Epsilon
            ? baseCenter + radial / radialMagnitude * endRadius
            : baseCenter;

        Fixed64 apexProjection = Vector3d.Dot(apex, direction);
        Fixed64 baseProjection = Vector3d.Dot(baseSupport, direction);
        return baseProjection >= apexProjection ? baseSupport : apex;
    }

    private static Vector3d SupportCapsule(LSCapsuleCollider capsule, Vector3d direction)
    {
        Fixed64 startProjection = Vector3d.Dot(capsule.LineSegmentStart, direction);
        Fixed64 endProjection = Vector3d.Dot(capsule.LineSegmentEnd, direction);
        Vector3d segmentPoint = endProjection > startProjection
            ? capsule.LineSegmentEnd
            : capsule.LineSegmentStart;
        return segmentPoint + direction * capsule.ScaledRadius;
    }

    private static Vector3d SupportCylinder(LSCylinderCollider cylinder, Vector3d direction)
    {
        Vector3d localDirection = cylinder.Rotation.Inverse() * direction;
        Vector3d radial = new(localDirection.X, Fixed64.Zero, localDirection.Z);
        Fixed64 radialMagnitude = radial.Magnitude;
        Vector3d radialSupport = radialMagnitude > Fixed64.Epsilon
            ? radial / radialMagnitude * cylinder.ScaledRadius
            : Vector3d.Right * cylinder.ScaledRadius;
        Fixed64 y = localDirection.Y >= Fixed64.Zero ? cylinder.HalfHeight : -cylinder.HalfHeight;
        return cylinder.Center + cylinder.Rotation * new Vector3d(radialSupport.X, y, radialSupport.Z);
    }

    private static Vector3d SupportCone(LSConeCollider cone, Vector3d direction)
    {
        Vector3d localDirection = cone.Rotation.Inverse() * direction;
        Vector3d radial = new(localDirection.X, Fixed64.Zero, localDirection.Z);
        Fixed64 radialMagnitude = radial.Magnitude;
        Vector3d radialSupport = radialMagnitude > Fixed64.Epsilon
            ? radial / radialMagnitude * cone.ScaledRadius
            : Vector3d.Right * cone.ScaledRadius;

        Vector3d localBase = new(radialSupport.X, -cone.HalfHeight, radialSupport.Z);
        Vector3d localApex = new(Fixed64.Zero, cone.HalfHeight, Fixed64.Zero);
        Fixed64 baseProjection = Vector3d.Dot(localBase, localDirection);
        Fixed64 apexProjection = Vector3d.Dot(localApex, localDirection);
        return cone.Center + cone.Rotation * (apexProjection >= baseProjection ? localApex : localBase);
    }

    private static Vector3d SupportVertices(Vector3d[] vertices, Vector3d direction)
    {
        Vector3d best = vertices[0];
        Fixed64 bestProjection = Vector3d.Dot(best, direction);
        for (int i = 1; i < vertices.Length; i++)
        {
            Fixed64 projection = Vector3d.Dot(vertices[i], direction);
            if (projection <= bestProjection)
                continue;

            bestProjection = projection;
            best = vertices[i];
        }

        return best;
    }

}
