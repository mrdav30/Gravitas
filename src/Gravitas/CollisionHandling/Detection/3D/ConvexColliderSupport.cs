//=======================================================================
// ConvexColliderSupport.cs
//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp;
using FixedMathSharp.Bounds;
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
        Vector3d normal = ResolveSupportDirection(direction);

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
        Vector3d normalized = axis != Vector3d.Zero
            ? axis.Normalized
            : Vector3d.Right;
        Fixed64 min = Vector3d.Dot(Support(collider, -normalized), normalized);
        Fixed64 max = Vector3d.Dot(Support(collider, normalized), normalized);
        return new FixedRange(min, max);
    }

    public static bool Intersects(LSCollider first, LSCollider second, int maxIterations = 32)
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

        for (int i = 0; i < maxIterations; i++)
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

        // No separating support was found within the bounded search, so preserve touching/overlap.
        return true;
    }

    public static bool IntersectsConeVolume(
        LSCollider collider,
        Vector3d apex,
        Vector3d baseCenter,
        Vector3d axis,
        Fixed64 endRadius,
        int maxIterations = 32)
    {
        if (!IsSupported(collider))
            return false;

        Span<Vector3d> simplex = stackalloc Vector3d[4];
        int count = 0;
        Vector3d coneCenter = Vector3d.Midpoint(apex, baseCenter);
        Vector3d direction = collider.Center - coneCenter;
        if (direction.MagnitudeSquared <= Fixed64.Epsilon)
            direction = Vector3d.Right;

        simplex[count++] = SupportMinkowskiConeCollider(collider, apex, baseCenter, axis, endRadius, direction);
        direction = -simplex[0];
        if (direction.MagnitudeSquared <= Fixed64.Epsilon)
            return true;

        for (int i = 0; i < maxIterations; i++)
        {
            Vector3d point = SupportMinkowskiConeCollider(collider, apex, baseCenter, axis, endRadius, direction);
            if (Vector3d.Dot(point, direction) < -Fixed64.Epsilon)
                return false;

            GjkSimplexPolicy.AddPoint(simplex, ref count, point);
            if (GjkSimplexPolicy.Update(simplex, ref count, ref direction))
                return true;

            if (direction.MagnitudeSquared <= Fixed64.Epsilon)
                return true;
        }

        // No separating support was found within the bounded search, so preserve touching/overlap.
        return true;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static Vector3d SupportMinkowski(LSCollider first, LSCollider second, Vector3d direction) =>
        Support(first, direction) - Support(second, -direction);

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static Vector3d SupportMinkowskiConeCollider(
        LSCollider collider,
        Vector3d apex,
        Vector3d baseCenter,
        Vector3d axis,
        Fixed64 endRadius,
        Vector3d direction) =>
        ConeGeometry.GetFiniteConeSupportPoint(
            apex,
            baseCenter,
            axis,
            endRadius,
            ResolveSupportDirection(direction))
        - Support(collider, -direction);

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static Vector3d ResolveSupportDirection(Vector3d direction) =>
        direction != Vector3d.Zero
            ? direction.Normalized
            : Vector3d.Right;

    private static Vector3d SupportCapsule(LSCapsuleCollider capsule, Vector3d direction)
    {
        Vector3d segmentPoint = Vector3d.CompareProjection(
                capsule.LineSegmentEnd,
                capsule.LineSegmentStart,
                direction) > 0
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
        return cone.Center + cone.Rotation * (
            Vector3d.CompareProjection(localApex, localBase, localDirection) >= 0
                ? localApex
                : localBase);
    }

    private static Vector3d SupportVertices(Vector3d[] vertices, Vector3d direction)
    {
        Vector3d best = vertices[0];
        for (int i = 1; i < vertices.Length; i++)
        {
            Vector3d candidate = vertices[i];
            if (Vector3d.CompareProjection(candidate, best, direction) <= 0)
                continue;

            best = candidate;
        }

        return best;
    }

}
