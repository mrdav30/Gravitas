//=======================================================================
// ConvexColliderSupport.cs
//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp;
using FixedMathSharp.Geometry;
using Gravitas.Colliders;
using System;
using System.Runtime.CompilerServices;

namespace Gravitas.CollisionHandling;

internal static class ConvexColliderSupport
{
    public static FixedPointAnchor GetSupportAnchor(
        LSCollider collider,
        Vector3d direction,
        Vector3d originOffset)
    {
        Vector3d normal = ResolveSupportDirection(direction);
        Vector3d localTranslation =
            GetLocalDisplacement(collider.Rotation, originOffset);
        if (collider is LSSphereCollider sphere)
        {
            return FixedSegment.GetCenteredCapsuleSupportAnchor(
                    sphere.Center,
                    sphere.Rotation,
                    Fixed64.Zero,
                    sphere.ScaledRadius,
                    normal)
                .WithLocalTranslation(localTranslation);
        }
        if (collider is LSCapsuleCollider capsule)
        {
            return FixedSegment.GetCenteredCapsuleSupportAnchor(
                    capsule.Center,
                    capsule.Rotation,
                    capsule.AxisLength,
                    capsule.ScaledRadius,
                    normal)
                .WithLocalTranslation(localTranslation);
        }
        if (collider is LSCuboidCollider cuboid)
        {
            return new FixedPointAnchor(
                cuboid.Center,
                cuboid.Rotation,
                cuboid.OrientedBox.GetLocalSupportPoint(normal))
                .WithLocalTranslation(localTranslation);
        }
        if (collider is LSCylinderCollider cylinder)
        {
            return FixedSegment.GetCenteredFiniteCylinderSupportAnchor(
                    cylinder.Center,
                    cylinder.Rotation,
                    cylinder.Height,
                    cylinder.ScaledRadius,
                    normal)
                .WithLocalTranslation(localTranslation);
        }
        if (collider is LSConeCollider cone)
        {
            return FixedSegment.GetCenteredFiniteConeSupportAnchor(
                    cone.Center,
                    cone.Rotation,
                    cone.Height,
                    cone.ScaledRadius,
                    normal)
                .WithLocalTranslation(localTranslation);
        }
        if (collider is LSMeshCollider { Mode: MeshColliderMode.Convex } mesh)
        {
            return new FixedPointAnchor(
                mesh.Mesh.Origin,
                mesh.Mesh.Rotation,
                mesh.Mesh.GetSupportVertexLocal(normal))
                .WithLocalTranslation(localTranslation);
        }

        throw new NotSupportedException(
            $"Convex support mapping does not support {collider.GetType().Name}.");
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static bool IsSupported(LSCollider collider) =>
        collider is LSSphereCollider
            or LSCapsuleCollider
            or LSCuboidCollider
            or LSCylinderCollider
            or LSConeCollider
            or LSMeshCollider { Mode: MeshColliderMode.Convex };

    public static bool Intersects(LSCollider first, LSCollider second, int maxIterations = 32)
    {
        if (!IsSupported(first) || !IsSupported(second))
            return false;

        Span<Vector3d> simplex = stackalloc Vector3d[4];
        int count = 0;
        Vector3d direction = second.Center - first.Center;
        if (direction.MagnitudeSquared <= Fixed64.Epsilon)
            direction = Vector3d.Right;

        if (!TrySupportMinkowski(first, second, direction, out simplex[count]))
            return false;
        count++;
        direction = -simplex[0];
        if (direction.MagnitudeSquared <= Fixed64.Epsilon)
            return true;

        for (int i = 0; i < maxIterations; i++)
        {
            if (!TrySupportMinkowski(first, second, direction, out Vector3d point))
                return false;
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

        if (!TrySupportMinkowskiConeCollider(
                collider,
                apex,
                baseCenter,
                axis,
                endRadius,
                direction,
                out simplex[count]))
        {
            return false;
        }
        count++;
        direction = -simplex[0];
        if (direction.MagnitudeSquared <= Fixed64.Epsilon)
            return true;

        for (int i = 0; i < maxIterations; i++)
        {
            if (!TrySupportMinkowskiConeCollider(
                    collider,
                    apex,
                    baseCenter,
                    axis,
                    endRadius,
                    direction,
                    out Vector3d point))
            {
                return false;
            }
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

    private static bool TrySupportMinkowski(
        LSCollider first,
        LSCollider second,
        Vector3d direction,
        out Vector3d difference)
    {
        Vector3d normal = ResolveSupportDirection(direction);
        if (first is LSCuboidCollider firstBox
            && second is LSCuboidCollider secondBox)
        {
            return firstBox.OrientedBox.TryGetSupportDifference(
                secondBox.OrientedBox,
                normal,
                out difference);
        }

        FixedPointAnchor supportA =
            GetSupportAnchor(first, normal, Vector3d.Zero);
        FixedPointAnchor supportB =
            GetSupportAnchor(second, -normal, Vector3d.Zero);
        return supportA.TryGetOffsetFrom(supportB, out difference);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static bool TrySupportMinkowskiConeCollider(
        LSCollider collider,
        Vector3d apex,
        Vector3d baseCenter,
        Vector3d axis,
        Fixed64 endRadius,
        Vector3d direction,
        out Vector3d difference)
    {
        Vector3d normal = ResolveSupportDirection(direction);
        Vector3d radialDirection =
            Vector3d.GetNormalizedProjectionOnPlane(normal, axis);
        var apexAnchor = new FixedPointAnchor(
            apex,
            FixedQuaternion.Identity,
            Vector3d.Zero);
        var baseAnchor = new FixedPointAnchor(
            baseCenter,
            FixedQuaternion.Identity,
            radialDirection * endRadius);
        FixedPointAnchor coneSupport =
            baseAnchor.ProjectNonNegativeOffsetFrom(apexAnchor, normal)
                > Fixed64.Zero
            || apexAnchor.ProjectNonNegativeOffsetFrom(baseAnchor, normal)
                == Fixed64.Zero
                ? baseAnchor
                : apexAnchor;
        FixedPointAnchor colliderSupport =
            GetSupportAnchor(collider, -normal, Vector3d.Zero);
        return coneSupport.TryGetOffsetFrom(
            colliderSupport,
            out difference);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static Vector3d ResolveSupportDirection(Vector3d direction) =>
        direction != Vector3d.Zero
            ? direction.Normalized
            : Vector3d.Right;

    internal static Vector3d GetLocalDisplacement(
        FixedQuaternion rotation,
        Vector3d worldDisplacement)
    {
        if (rotation.Inverse().TryRotate(
                worldDisplacement,
                out Vector3d localDisplacement))
        {
            return localDisplacement;
        }

        throw new InvalidOperationException(
            "The sweep displacement cannot be represented in the collider's canonical frame.");
    }

}
