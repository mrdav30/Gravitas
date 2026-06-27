//=======================================================================
// CollisionDetectionMixed.cs
//=======================================================================
// MIT License, Copyright (c) 2026–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp;
using FixedMathSharp.Bounds;
using Gravitas.Colliders;
using System.Runtime.CompilerServices;

namespace Gravitas.CollisionHandling;

/// <summary>
/// Deterministic mixed 2D/3D narrow-phase collision checks.
/// </summary>
public static partial class CollisionDetectionMixed
{
    public static bool TryCollide(LSCollider collider3D, LSCollider2D collider2D, out MixedContact contact)
    {
        SwiftThrowHelper.ThrowIfNull(collider3D, nameof(collider3D));
        SwiftThrowHelper.ThrowIfNull(collider2D, nameof(collider2D));

        if (!BoundsOverlap(collider3D.Bounds, collider2D.MixedBounds3D))
        {
            contact = default;
            return false;
        }

        if (collider2D is LSCompoundCollider2D compound2D)
            return TryEmbeddedCompound2D(collider3D, compound2D, out contact);

        return collider3D.Shape switch
        {
            ColliderType.Sphere => TrySphereEmbedded2D((LSSphereCollider)collider3D, collider2D, out contact),
            ColliderType.AABox or ColliderType.OBBox => TryCuboidEmbedded2D((LSCuboidCollider)collider3D, collider2D, out contact),
            ColliderType.Capsule => TryCapsuleEmbedded2D((LSCapsuleCollider)collider3D, collider2D, out contact),
            ColliderType.Cylinder => TryCylinderEmbedded2D((LSCylinderCollider)collider3D, collider2D, out contact),
            ColliderType.Mesh => TryMeshEmbedded2D((LSMeshCollider)collider3D, collider2D, out contact),
            ColliderType.Compound => TryCompoundEmbedded2D((LSCompoundCollider)collider3D, collider2D, out contact),
            _ => NoContact(out contact)
        };
    }

    private static bool TryEmbeddedCompound2D(
        LSCollider collider3D,
        LSCompoundCollider2D compound2D,
        out MixedContact contact)
    {
        bool found = false;
        MixedContact best = default;

        for (int i = 0; i < compound2D.PartCount; i++)
        {
            LSCollider2D part = compound2D.GetPartCollider(i);
            if (!BoundsOverlap(collider3D.Bounds, part.MixedBounds3D)
                || !TryCollide(collider3D, part, out MixedContact candidate))
            {
                continue;
            }

            candidate = candidate.WithFallbackMaterials(collider3D.Material, part.Material);
            if (!found || candidate.Depth < best.Depth)
            {
                best = candidate;
                found = true;
            }
        }

        if (!found)
            return NoContact(out contact);

        contact = best;
        return true;
    }

    private static bool TrySphereEmbedded2D(LSSphereCollider sphere, LSCollider2D embedded, out MixedContact contact)
    {
        Vector3d sphereCenter = sphere.Center;
        Vector2d planarCenter = new(sphereCenter.X, sphereCenter.Z);
        Fixed64 slabMinY = embedded.MixedBounds3D.Min.Y;
        Fixed64 slabMaxY = embedded.MixedBounds3D.Max.Y;
        bool planarInside = embedded.ContainsPoint(planarCenter);
        bool yInside = sphereCenter.Y >= slabMinY && sphereCenter.Y <= slabMaxY;

        if (planarInside && yInside)
            return TrySphereFromInsideEmbedded2D(sphere, embedded, planarCenter, slabMinY, slabMaxY, out contact);

        Vector2d closestPlanar = planarInside ? planarCenter : embedded.GetClosestPoint(planarCenter);
        Fixed64 closestY = Clamp(sphereCenter.Y, slabMinY, slabMaxY);
        Vector3d closestEmbeddedPoint = new(closestPlanar.X, closestY, closestPlanar.Y);
        Vector3d delta = closestEmbeddedPoint - sphereCenter;
        Fixed64 distanceSquared = delta.MagnitudeSquared;
        if (distanceSquared > sphere.ScaledRadiusSqr)
        {
            contact = default;
            return false;
        }

        Fixed64 distance = distanceSquared > Fixed64.Epsilon ? FixedMath.Sqrt(distanceSquared) : Fixed64.Zero;
        Vector3d normal = distance > Fixed64.Zero
            ? delta / distance
            : ResolveFallbackNormal(sphereCenter, embedded);
        contact = new MixedContact(
            sphereCenter + normal * sphere.ScaledRadius,
            closestEmbeddedPoint,
            normal,
            sphere.ScaledRadius - distance);
        return true;
    }

    private static bool TryCuboidEmbedded2D(LSCuboidCollider cuboid, LSCollider2D embedded, out MixedContact contact)
    {
        bool collided = embedded.Shape == ColliderType2D.Circle
            ? TryTestCuboidCircleSlab(cuboid, (LSCircleCollider2D)embedded, out AxisPenetration penetration)
            : TryTestCuboidPrism(cuboid, embedded, out penetration);

        return collided
            ? BuildCuboidContact(cuboid, embedded, penetration, out contact)
            : NoContact(out contact);
    }

    private static bool TryCapsuleEmbedded2D(LSCapsuleCollider capsule, LSCollider2D embedded, out MixedContact contact)
    {
        bool collided = embedded.Shape == ColliderType2D.Circle
            ? TryTestCapsuleCircleSlab(capsule, (LSCircleCollider2D)embedded, out AxisPenetration penetration)
            : TryTestCapsulePrism(capsule, embedded, out penetration);

        return collided
            ? BuildCapsuleContact(capsule, embedded, penetration, out contact)
            : NoContact(out contact);
    }

    private static bool TryCylinderEmbedded2D(LSCylinderCollider cylinder, LSCollider2D embedded, out MixedContact contact)
    {
        bool collided = embedded.Shape == ColliderType2D.Circle
            ? TryTestCylinderCircleSlab(cylinder, (LSCircleCollider2D)embedded, out AxisPenetration penetration)
            : TryTestCylinderPrism(cylinder, embedded, out penetration);

        return collided
            ? BuildCylinderContact(cylinder, embedded, penetration, out contact)
            : NoContact(out contact);
    }

    private static bool TryTestCuboidCircleSlab(
        LSCuboidCollider cuboid,
        LSCircleCollider2D circle,
        out AxisPenetration penetration)
    {
        penetration = default;

        if (!CheckCuboidCircleSlabAxis(cuboid, circle, Vector3d.Up, ref penetration))
            return false;

        for (int i = 0; i < cuboid.FaceNormals.Length; i++)
        {
            if (!CheckCuboidCircleSlabAxis(cuboid, circle, cuboid.FaceNormals[i], ref penetration))
                return false;
        }

        for (int i = 0; i < cuboid.EdgeDirections.Length; i++)
        {
            if (!CheckCuboidCircleSlabAxis(cuboid, circle, Vector3d.Cross(cuboid.EdgeDirections[i], Vector3d.Up), ref penetration))
                return false;
        }

        GetCircleSlabSegment(circle, out Vector3d start, out Vector3d end);
        Vector3d linePoint = Vector3d.ClosestPointOnLineSegment(cuboid.Center, start, end);
        Vector3d cuboidPoint = cuboid.ClosestPointOnSurface(linePoint);
        if (!CheckCuboidCircleSlabAxis(cuboid, circle, linePoint - cuboidPoint, ref penetration))
            return false;

        return penetration.HasValue;
    }

    private static bool TryTestCapsuleCircleSlab(
        LSCapsuleCollider capsule,
        LSCircleCollider2D circle,
        out AxisPenetration penetration)
    {
        penetration = default;
        GetCircleSlabSegment(circle, out Vector3d circleStart, out Vector3d circleEnd);

        if (!CheckCapsuleCircleSlabAxis(capsule, circle, Vector3d.Up, ref penetration))
            return false;

        if (!CheckCapsuleCircleSlabAxis(capsule, circle, capsule.LineDirection, ref penetration))
            return false;

        if (!CheckCapsuleCircleSlabAxis(capsule, circle, Vector3d.Cross(capsule.LineDirection, Vector3d.Up), ref penetration))
            return false;

        (Vector3d CapsulePoint, Vector3d CirclePoint) closest = ClosestPointsOnSegments(
            capsule.LineSegmentStart,
            capsule.LineSegmentEnd,
            circleStart,
            circleEnd);
        if (!CheckCapsuleCircleSlabAxis(capsule, circle, closest.CirclePoint - closest.CapsulePoint, ref penetration))
            return false;

        return penetration.HasValue;
    }

    private static bool TryTestCylinderCircleSlab(
        LSCylinderCollider cylinder,
        LSCircleCollider2D circle,
        out AxisPenetration penetration)
    {
        penetration = default;
        GetCircleSlabSegment(circle, out Vector3d circleStart, out Vector3d circleEnd);

        if (!CheckCylinderCircleSlabAxis(cylinder, circle, cylinder.LineDirection, ref penetration))
            return false;

        if (!CheckCylinderCircleSlabAxis(cylinder, circle, Vector3d.Up, ref penetration))
            return false;

        if (!CheckCylinderCircleSlabAxis(cylinder, circle, Vector3d.Cross(cylinder.LineDirection, Vector3d.Up), ref penetration))
            return false;

        (Vector3d CylinderPoint, Vector3d CirclePoint) closest = ClosestPointsOnSegments(
            cylinder.LineSegmentStart,
            cylinder.LineSegmentEnd,
            circleStart,
            circleEnd);
        if (!CheckCylinderCircleSlabAxis(cylinder, circle, closest.CirclePoint - closest.CylinderPoint, ref penetration))
            return false;

        return penetration.HasValue;
    }

    private static bool TryTestCuboidPrism(
        LSCuboidCollider cuboid,
        LSCollider2D prism,
        out AxisPenetration penetration)
    {
        penetration = default;

        if (!CheckCuboidPrismAxis(cuboid, prism, Vector3d.Up, ref penetration))
            return false;

        for (int i = 0; i < prism.VertexCount; i++)
        {
            GetPrismEdge(prism, i, out Vector2d edge2D);
            if (!CheckCuboidPrismAxis(cuboid, prism, GetPlanarEdgeNormal(edge2D), ref penetration))
                return false;
        }

        for (int i = 0; i < cuboid.FaceNormals.Length; i++)
        {
            if (!CheckCuboidPrismAxis(cuboid, prism, cuboid.FaceNormals[i], ref penetration))
                return false;
        }

        for (int i = 0; i < cuboid.EdgeDirections.Length; i++)
        {
            if (!CheckCuboidPrismAxis(cuboid, prism, Vector3d.Cross(cuboid.EdgeDirections[i], Vector3d.Up), ref penetration))
                return false;

            for (int j = 0; j < prism.VertexCount; j++)
            {
                GetPrismEdge(prism, j, out Vector2d edge2D);
                Vector3d edge3D = new(edge2D.X, Fixed64.Zero, edge2D.Y);
                if (!CheckCuboidPrismAxis(cuboid, prism, Vector3d.Cross(cuboid.EdgeDirections[i], edge3D), ref penetration))
                    return false;
            }
        }

        Vector3d embeddedPoint = MixedEmbedded2DGeometry.GetClosestPointOnEmbeddedVolume(prism, cuboid.Center);
        Vector3d cuboidPoint = cuboid.ClosestPointOnSurface(embeddedPoint);
        if (!CheckCuboidPrismAxis(cuboid, prism, embeddedPoint - cuboidPoint, ref penetration))
            return false;

        return penetration.HasValue;
    }

    private static bool TryTestCapsulePrism(
        LSCapsuleCollider capsule,
        LSCollider2D prism,
        out AxisPenetration penetration)
    {
        penetration = default;

        if (!CheckCapsulePrismAxis(capsule, prism, capsule.LineDirection, ref penetration))
            return false;

        if (!CheckCapsulePrismAxis(capsule, prism, Vector3d.Up, ref penetration))
            return false;

        if (!CheckCapsulePrismAxis(capsule, prism, Vector3d.Cross(capsule.LineDirection, Vector3d.Up), ref penetration))
            return false;

        for (int i = 0; i < prism.VertexCount; i++)
        {
            GetPrismEdge(prism, i, out Vector2d edge2D);
            Vector3d edge3D = new(edge2D.X, Fixed64.Zero, edge2D.Y);
            if (!CheckCapsulePrismAxis(capsule, prism, GetPlanarEdgeNormal(edge2D), ref penetration))
                return false;

            if (!CheckCapsulePrismAxis(capsule, prism, Vector3d.Cross(capsule.LineDirection, edge3D), ref penetration))
                return false;
        }

        Vector3d linePoint = Vector3d.ClosestPointOnLineSegment(GetEmbeddedCenter3D(prism), capsule.LineSegmentStart, capsule.LineSegmentEnd);
        Vector3d embeddedPoint = MixedEmbedded2DGeometry.GetClosestPointOnEmbeddedVolume(prism, linePoint);
        if (!CheckCapsulePrismAxis(capsule, prism, embeddedPoint - linePoint, ref penetration))
            return false;

        return penetration.HasValue;
    }

    private static bool TryTestCylinderPrism(
        LSCylinderCollider cylinder,
        LSCollider2D prism,
        out AxisPenetration penetration)
    {
        penetration = default;

        if (!CheckCylinderPrismAxis(cylinder, prism, cylinder.LineDirection, ref penetration))
            return false;

        if (!CheckCylinderPrismAxis(cylinder, prism, Vector3d.Up, ref penetration))
            return false;

        if (!CheckCylinderPrismAxis(cylinder, prism, Vector3d.Cross(cylinder.LineDirection, Vector3d.Up), ref penetration))
            return false;

        for (int i = 0; i < prism.VertexCount; i++)
        {
            GetPrismEdge(prism, i, out Vector2d edge2D);
            Vector3d edge3D = new(edge2D.X, Fixed64.Zero, edge2D.Y);
            if (!CheckCylinderPrismAxis(cylinder, prism, GetPlanarEdgeNormal(edge2D), ref penetration))
                return false;

            if (!CheckCylinderPrismAxis(cylinder, prism, Vector3d.Cross(cylinder.LineDirection, edge3D), ref penetration))
                return false;
        }

        Vector3d linePoint = Vector3d.ClosestPointOnLineSegment(GetEmbeddedCenter3D(prism), cylinder.LineSegmentStart, cylinder.LineSegmentEnd);
        Vector3d embeddedPoint = MixedEmbedded2DGeometry.GetClosestPointOnEmbeddedVolume(prism, linePoint);
        if (!CheckCylinderPrismAxis(cylinder, prism, embeddedPoint - linePoint, ref penetration))
            return false;

        return penetration.HasValue;
    }

    private static bool CheckCuboidCircleSlabAxis(
        LSCuboidCollider cuboid,
        LSCircleCollider2D circle,
        Vector3d axis,
        ref AxisPenetration penetration)
    {
        if (!TryNormalizeAxis(axis, out Vector3d normalizedAxis))
            return true;

        FixedRange cuboidProjection = FixedRange.MinRange;
        AxisProjectionHelper.ProjectPolygonOntoAxis(normalizedAxis, cuboid.Vertices, ref cuboidProjection);
        FixedRange circleProjection = ProjectCircleSlabOntoAxis(normalizedAxis, circle);
        return CheckProjectedAxis(cuboidProjection, circleProjection, normalizedAxis, GetEmbeddedCenter3D(circle) - cuboid.Center, ref penetration);
    }

    private static bool CheckCapsuleCircleSlabAxis(
        LSCapsuleCollider capsule,
        LSCircleCollider2D circle,
        Vector3d axis,
        ref AxisPenetration penetration)
    {
        if (!TryNormalizeAxis(axis, out Vector3d normalizedAxis))
            return true;

        FixedRange capsuleProjection = AxisProjectionHelper.ProjectCapsuleOntoAxis(
            normalizedAxis,
            capsule.LineSegmentStart,
            capsule.LineSegmentEnd,
            capsule.ScaledRadius);
        FixedRange circleProjection = ProjectCircleSlabOntoAxis(normalizedAxis, circle);
        return CheckProjectedAxis(capsuleProjection, circleProjection, normalizedAxis, GetEmbeddedCenter3D(circle) - capsule.Center, ref penetration);
    }

    private static bool CheckCylinderCircleSlabAxis(
        LSCylinderCollider cylinder,
        LSCircleCollider2D circle,
        Vector3d axis,
        ref AxisPenetration penetration)
    {
        if (!TryNormalizeAxis(axis, out Vector3d normalizedAxis))
            return true;

        FixedRange cylinderProjection = AxisProjectionHelper.ProjectCylinderOntoAxis(
            normalizedAxis,
            cylinder.LineSegmentStart,
            cylinder.LineSegmentEnd,
            cylinder.LineDirection,
            cylinder.ScaledRadius);
        FixedRange circleProjection = ProjectCircleSlabOntoAxis(normalizedAxis, circle);
        return CheckProjectedAxis(cylinderProjection, circleProjection, normalizedAxis, GetEmbeddedCenter3D(circle) - cylinder.Center, ref penetration);
    }

    private static bool CheckCuboidPrismAxis(
        LSCuboidCollider cuboid,
        LSCollider2D prism,
        Vector3d axis,
        ref AxisPenetration penetration)
    {
        if (!TryNormalizeAxis(axis, out Vector3d normalizedAxis))
            return true;

        FixedRange cuboidProjection = FixedRange.MinRange;
        AxisProjectionHelper.ProjectPolygonOntoAxis(normalizedAxis, cuboid.Vertices, ref cuboidProjection);
        FixedRange prismProjection = ProjectPrismOntoAxis(normalizedAxis, prism);
        return CheckProjectedAxis(cuboidProjection, prismProjection, normalizedAxis, GetEmbeddedCenter3D(prism) - cuboid.Center, ref penetration);
    }

    private static bool CheckCapsulePrismAxis(
        LSCapsuleCollider capsule,
        LSCollider2D prism,
        Vector3d axis,
        ref AxisPenetration penetration)
    {
        if (!TryNormalizeAxis(axis, out Vector3d normalizedAxis))
            return true;

        FixedRange capsuleProjection = AxisProjectionHelper.ProjectCapsuleOntoAxis(
            normalizedAxis,
            capsule.LineSegmentStart,
            capsule.LineSegmentEnd,
            capsule.ScaledRadius);
        FixedRange prismProjection = ProjectPrismOntoAxis(normalizedAxis, prism);
        return CheckProjectedAxis(capsuleProjection, prismProjection, normalizedAxis, GetEmbeddedCenter3D(prism) - capsule.Center, ref penetration);
    }

    private static bool CheckCylinderPrismAxis(
        LSCylinderCollider cylinder,
        LSCollider2D prism,
        Vector3d axis,
        ref AxisPenetration penetration)
    {
        if (!TryNormalizeAxis(axis, out Vector3d normalizedAxis))
            return true;

        FixedRange cylinderProjection = AxisProjectionHelper.ProjectCylinderOntoAxis(
            normalizedAxis,
            cylinder.LineSegmentStart,
            cylinder.LineSegmentEnd,
            cylinder.LineDirection,
            cylinder.ScaledRadius);
        FixedRange prismProjection = ProjectPrismOntoAxis(normalizedAxis, prism);
        return CheckProjectedAxis(cylinderProjection, prismProjection, normalizedAxis, GetEmbeddedCenter3D(prism) - cylinder.Center, ref penetration);
    }

    private static bool BuildCuboidContact(
        LSCuboidCollider cuboid,
        LSCollider2D embedded,
        AxisPenetration penetration,
        out MixedContact contact)
    {
        Vector3d reference = MixedEmbedded2DGeometry.GetClosestPointOnEmbeddedVolume(embedded, cuboid.Center);
        Vector3d point3D = cuboid.ClosestPointOnSurface(reference);
        Vector3d point2D = MixedEmbedded2DGeometry.GetClosestPointOnEmbeddedVolume(embedded, point3D);
        contact = new MixedContact(point3D, point2D, penetration.Axis, penetration.Depth);
        return true;
    }

    private static bool BuildCapsuleContact(
        LSCapsuleCollider capsule,
        LSCollider2D embedded,
        AxisPenetration penetration,
        out MixedContact contact)
    {
        Vector3d linePoint = Vector3d.ClosestPointOnLineSegment(GetEmbeddedCenter3D(embedded), capsule.LineSegmentStart, capsule.LineSegmentEnd);
        Vector3d point3D = linePoint + penetration.Axis * capsule.ScaledRadius;
        Vector3d point2D = MixedEmbedded2DGeometry.GetClosestPointOnEmbeddedVolume(embedded, point3D);
        contact = new MixedContact(point3D, point2D, penetration.Axis, penetration.Depth);
        return true;
    }

    private static bool BuildCylinderContact(
        LSCylinderCollider cylinder,
        LSCollider2D embedded,
        AxisPenetration penetration,
        out MixedContact contact)
    {
        Vector3d reference = MixedEmbedded2DGeometry.GetClosestPointOnEmbeddedVolume(embedded, cylinder.Center);
        Vector3d point3D = cylinder.ClosestPointOnSurface(reference);
        Vector3d point2D = MixedEmbedded2DGeometry.GetClosestPointOnEmbeddedVolume(embedded, point3D);
        contact = new MixedContact(point3D, point2D, penetration.Axis, penetration.Depth);
        return true;
    }

    private static bool TrySphereFromInsideEmbedded2D(
        LSSphereCollider sphere,
        LSCollider2D embedded,
        Vector2d planarCenter,
        Fixed64 slabMinY,
        Fixed64 slabMaxY,
        out MixedContact contact)
    {
        Vector3d sphereCenter = sphere.Center;
        Fixed64 minYDistance = sphereCenter.Y - slabMinY;
        Fixed64 maxYDistance = slabMaxY - sphereCenter.Y;
        Fixed64 bestDistance = minYDistance;
        Vector3d normal = -Vector3d.Up;
        Vector3d embeddedPoint = new(planarCenter.X, slabMinY, planarCenter.Y);

        if (maxYDistance < bestDistance)
        {
            bestDistance = maxYDistance;
            normal = Vector3d.Up;
            embeddedPoint = new Vector3d(planarCenter.X, slabMaxY, planarCenter.Y);
        }

        if (MixedEmbedded2DGeometry.TryGetPlanarBoundaryPoint(embedded, planarCenter, out Vector2d planarBoundary, out Fixed64 planarDistance)
            && planarDistance < bestDistance)
        {
            bestDistance = planarDistance;
            Vector2d planarNormal = planarDistance > Fixed64.Epsilon
                ? (planarBoundary - planarCenter) / planarDistance
                : Vector2d.Right;
            normal = new Vector3d(planarNormal.X, Fixed64.Zero, planarNormal.Y);
            embeddedPoint = new Vector3d(planarBoundary.X, sphereCenter.Y, planarBoundary.Y);
        }

        contact = new MixedContact(
            sphereCenter + normal * sphere.ScaledRadius,
            embeddedPoint,
            normal,
            sphere.ScaledRadius + bestDistance);
        return true;
    }

    private static FixedRange ProjectCircleSlabOntoAxis(Vector3d axis, LSCircleCollider2D circle)
    {
        GetCircleSlabSegment(circle, out Vector3d start, out Vector3d end);
        return AxisProjectionHelper.ProjectCylinderOntoAxis(axis, start, end, Vector3d.Up, circle.ScaledRadius);
    }

    private static FixedRange ProjectPrismOntoAxis(Vector3d axis, LSCollider2D prism)
    {
        Vector2d first = GetPrismVertex(prism, 0);
        Fixed64 min = ProjectPrismVertex(axis, first, prism.MixedBounds3D.Min.Y);
        Fixed64 max = min;

        for (int i = 0; i < prism.VertexCount; i++)
        {
            Vector2d vertex = GetPrismVertex(prism, i);
            Fixed64 bottom = ProjectPrismVertex(axis, vertex, prism.MixedBounds3D.Min.Y);
            Fixed64 top = ProjectPrismVertex(axis, vertex, prism.MixedBounds3D.Max.Y);
            if (bottom < min)
                min = bottom;
            if (bottom > max)
                max = bottom;
            if (top < min)
                min = top;
            if (top > max)
                max = top;
        }

        return new FixedRange(min, max);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static Fixed64 ProjectPrismVertex(Vector3d axis, Vector2d vertex, Fixed64 y) =>
        Vector3d.Dot(axis, new Vector3d(vertex.X, y, vertex.Y));

    private static bool CheckProjectedAxis(
        FixedRange projection3D,
        FixedRange projection2D,
        Vector3d axis,
        Vector3d displacement3DTo2D,
        ref AxisPenetration penetration)
    {
        if (projection3D.Max < projection2D.Min || projection2D.Max < projection3D.Min)
            return false;

        Fixed64 depth = ComputeMinimumProjectionOverlap(projection3D, projection2D);
        if (!penetration.HasValue || depth < penetration.Depth)
        {
            Vector3d orientedAxis = Vector3d.Dot(axis, displacement3DTo2D) < Fixed64.Zero ? -axis : axis;
            penetration = new AxisPenetration(orientedAxis, depth);
        }

        return true;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static Fixed64 ComputeMinimumProjectionOverlap(FixedRange projection3D, FixedRange projection2D)
    {
        Fixed64 push3DLeft = projection3D.Max - projection2D.Min;
        Fixed64 push3DRight = projection2D.Max - projection3D.Min;
        Fixed64 overlap = FixedMath.Min(push3DLeft, push3DRight);
        return overlap > Fixed64.Zero ? overlap : Fixed64.Zero;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static bool TryNormalizeAxis(Vector3d axis, out Vector3d normalizedAxis)
    {
        Fixed64 magnitudeSqr = axis.MagnitudeSquared;
        if (magnitudeSqr <= Fixed64.Epsilon)
        {
            normalizedAxis = Vector3d.Zero;
            return false;
        }

        normalizedAxis = axis / FixedMath.Sqrt(magnitudeSqr);
        return true;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static Vector3d GetPlanarEdgeNormal(Vector2d edge)
    {
        Vector2d normal = new(edge.Y, -edge.X);
        return new Vector3d(normal.X, Fixed64.Zero, normal.Y);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static void GetCircleSlabSegment(LSCircleCollider2D circle, out Vector3d start, out Vector3d end)
    {
        Vector3d center = GetEmbeddedCenter3D(circle);
        start = new Vector3d(center.X, circle.MixedBounds3D.Min.Y, center.Z);
        end = new Vector3d(center.X, circle.MixedBounds3D.Max.Y, center.Z);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static Vector3d GetEmbeddedCenter3D(LSCollider2D embedded) =>
        new(embedded.Center.X, embedded.MixedSlabCenterY, embedded.Center.Y);

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static void GetPrismEdge(LSCollider2D prism, int index, out Vector2d edge)
    {
        Vector2d current = GetPrismVertex(prism, index);
        Vector2d next = GetPrismVertex(prism, (index + 1) % prism.VertexCount);
        edge = next - current;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static Vector2d GetPrismVertex(LSCollider2D prism, int index) =>
        prism.GetVertexUnchecked(index);

    private static (Vector3d First, Vector3d Second) ClosestPointsOnSegments(
        Vector3d firstStart,
        Vector3d firstEnd,
        Vector3d secondStart,
        Vector3d secondEnd)
    {
        bool firstDegenerate = (firstEnd - firstStart).MagnitudeSquared <= Fixed64.Epsilon;
        bool secondDegenerate = (secondEnd - secondStart).MagnitudeSquared <= Fixed64.Epsilon;

        if (firstDegenerate && secondDegenerate)
            return (firstStart, secondStart);

        if (firstDegenerate)
            return (firstStart, Vector3d.ClosestPointOnLineSegment(firstStart, secondStart, secondEnd));

        if (secondDegenerate)
            return (Vector3d.ClosestPointOnLineSegment(secondStart, firstStart, firstEnd), secondStart);

        return Vector3d.ClosestPointsOnTwoLines(firstStart, firstEnd, secondStart, secondEnd);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static bool BoundsOverlap(FixedBoundBox first, FixedBoundBox second) =>
        first.Max.X >= second.Min.X
        && first.Min.X <= second.Max.X
        && first.Max.Y >= second.Min.Y
        && first.Min.Y <= second.Max.Y
        && first.Max.Z >= second.Min.Z
        && first.Min.Z <= second.Max.Z;

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static Vector3d ResolveFallbackNormal(Vector3d sphereCenter, LSCollider2D embedded)
    {
        Vector3d fallback = new(
            embedded.Center.X - sphereCenter.X,
            embedded.MixedSlabCenterY - sphereCenter.Y,
            embedded.Center.Y - sphereCenter.Z);
        return fallback.MagnitudeSquared > Fixed64.Epsilon ? fallback.Normalized : Vector3d.Right;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static Fixed64 Clamp(Fixed64 value, Fixed64 min, Fixed64 max) =>
        value < min ? min : value > max ? max : value;

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static bool NoContact(out MixedContact contact)
    {
        contact = default;
        return false;
    }

}
