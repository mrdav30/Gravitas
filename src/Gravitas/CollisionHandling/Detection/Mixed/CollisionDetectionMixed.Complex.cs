//=======================================================================
// CollisionDetectionMixed.Complex.cs
//=======================================================================
// MIT License, Copyright (c) 2026–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp;
using FixedMathSharp.Bounds;
using Gravitas.Colliders;
using SwiftCollections;
using SwiftCollections.Query;
using System.Runtime.CompilerServices;

namespace Gravitas.CollisionHandling;

public static partial class CollisionDetectionMixed
{
    private static bool TryCompoundEmbedded2D(LSCompoundCollider compound, LSCollider2D embedded, out MixedContact contact)
    {
        bool found = false;
        MixedContact best = default;

        for (int i = 0; i < compound.PartCount; i++)
        {
            LSCollider part = compound.GetPartCollider(i);
            if (!BoundsOverlap(part.Bounds, embedded.MixedBounds3D)
                || !TryCollide(part, embedded, out MixedContact candidate))
            {
                continue;
            }

            candidate = candidate.WithFallbackMaterials(part.Material, embedded.Material);
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

    private static bool TryMeshEmbedded2D(LSMeshCollider mesh, LSCollider2D embedded, out MixedContact contact)
    {
        SwiftList<int> triangleBuffer = mesh.Context.CollisionScratch.MeshTriangleCandidatesA;
        mesh.GetTrianglesInBounds(new FixedBoundVolume(embedded.MixedBounds3D.Min, embedded.MixedBounds3D.Max), triangleBuffer);

        bool found = false;
        MixedContact best = default;
        for (int i = 0; i < triangleBuffer.Count; i++)
        {
            GetMeshTriangle(mesh, triangleBuffer[i], out MixedTriangle triangle);
            if (!BoundsOverlap(triangle.Bounds, embedded.MixedBounds3D)
                || !TryTriangleEmbedded2D(triangle, embedded, out AxisPenetration penetration))
            {
                continue;
            }

            BuildMeshContact(embedded, triangle, penetration, out MixedContact candidate);
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

    private static bool TryTriangleEmbedded2D(
        MixedTriangle triangle,
        LSCollider2D embedded,
        out AxisPenetration penetration)
    {
        switch (embedded.Shape)
        {
            case ColliderType2D.Circle:
                return TryTestTriangleCircleSlab(triangle, (LSCircleCollider2D)embedded, out penetration);
            case ColliderType2D.Capsule:
            case ColliderType2D.AABox:
            case ColliderType2D.ConvexPolygon:
                return TryTestTrianglePrism(triangle, embedded, out penetration);
            default:
                penetration = default;
                return false;
        }
    }

    private static bool TryTestTriangleCircleSlab(
        MixedTriangle triangle,
        LSCircleCollider2D circle,
        out AxisPenetration penetration)
    {
        penetration = default;

        if (!CheckTriangleCircleSlabAxis(triangle, circle, Vector3d.Up, ref penetration))
            return false;

        if (!CheckTriangleCircleSlabAxis(triangle, circle, triangle.Normalized, ref penetration))
            return false;

        for (int i = 0; i < 3; i++)
        {
            if (!CheckTriangleCircleSlabAxis(triangle, circle, Vector3d.Cross(triangle.GetEdge(i), Vector3d.Up), ref penetration))
                return false;
        }

        GetCircleSlabSegment(circle, out Vector3d circleStart, out Vector3d circleEnd);
        ClosestPointsSegmentTriangle(circleStart, circleEnd, triangle, out Vector3d linePoint, out Vector3d trianglePoint);
        if (!CheckTriangleCircleSlabAxis(triangle, circle, linePoint - trianglePoint, ref penetration))
            return false;

        return penetration.HasValue;
    }

    private static bool TryTestTrianglePrism(
        MixedTriangle triangle,
        LSCollider2D prism,
        out AxisPenetration penetration)
    {
        penetration = default;

        if (!CheckTrianglePrismAxis(triangle, prism, Vector3d.Up, ref penetration))
            return false;

        if (!CheckTrianglePrismAxis(triangle, prism, triangle.Normalized, ref penetration))
            return false;

        if (prism is LSCapsuleCollider2D embeddedCapsule)
        {
            if (!CheckEmbeddedCapsuleAxes(triangle, embeddedCapsule, ref penetration))
                return false;
        }
        else
        {
            for (int i = 0; i < prism.VertexCount; i++)
            {
                GetPrismEdge(prism, i, out Vector2d edge2D);
                if (!CheckTrianglePrismAxis(triangle, prism, GetPlanarEdgeNormal(edge2D), ref penetration))
                    return false;
            }
        }

        for (int i = 0; i < 3; i++)
        {
            Vector3d triangleEdge = triangle.GetEdge(i);
            if (!CheckTrianglePrismAxis(triangle, prism, Vector3d.Cross(triangleEdge, Vector3d.Up), ref penetration))
                return false;

            if (prism is LSCapsuleCollider2D capsule2D)
            {
                if (!CheckTriangleEmbeddedCapsuleEdgeAxis(triangle, capsule2D, triangleEdge, ref penetration))
                    return false;
            }
            else
            {
                for (int j = 0; j < prism.VertexCount; j++)
                {
                    GetPrismEdge(prism, j, out Vector2d edge2D);
                    Vector3d prismEdge = new(edge2D.X, Fixed64.Zero, edge2D.Y);
                    if (!CheckTrianglePrismAxis(triangle, prism, Vector3d.Cross(triangleEdge, prismEdge), ref penetration))
                        return false;
                }
            }
        }

        Vector3d embeddedPoint = MixedEmbedded2DGeometry.GetClosestPointOnEmbeddedVolume(prism, triangle.Center);
        Vector3d trianglePoint = MeshUtils.ClosestPointOnTriangle(triangle.A, triangle.B, triangle.C, triangle.Normalized, embeddedPoint);
        if (!CheckTrianglePrismAxis(triangle, prism, embeddedPoint - trianglePoint, ref penetration))
            return false;

        return penetration.HasValue;
    }

    private static bool CheckTriangleCircleSlabAxis(
        MixedTriangle triangle,
        LSCircleCollider2D circle,
        Vector3d axis,
        ref AxisPenetration penetration)
    {
        if (!TryNormalizeAxis(axis, out Vector3d normalizedAxis))
            return true;

        FixedRange triangleProjection = ProjectTriangleOntoAxis(triangle, normalizedAxis);
        FixedRange circleProjection = ProjectCircleSlabOntoAxis(normalizedAxis, circle);
        return CheckProjectedAxis(
            triangleProjection,
            circleProjection,
            normalizedAxis,
            GetEmbeddedCenter3D(circle) - triangle.Center,
            ref penetration);
    }

    private static bool CheckTrianglePrismAxis(
        MixedTriangle triangle,
        LSCollider2D prism,
        Vector3d axis,
        ref AxisPenetration penetration)
    {
        if (!TryNormalizeAxis(axis, out Vector3d normalizedAxis))
            return true;

        FixedRange triangleProjection = ProjectTriangleOntoAxis(triangle, normalizedAxis);
        FixedRange prismProjection = ProjectPrismOntoAxis(normalizedAxis, prism);
        return CheckProjectedAxis(
            triangleProjection,
            prismProjection,
            normalizedAxis,
            GetEmbeddedCenter3D(prism) - triangle.Center,
            ref penetration);
    }

    private static bool CheckEmbeddedCapsuleAxes(
        MixedTriangle triangle,
        LSCapsuleCollider2D capsule,
        ref AxisPenetration penetration)
    {
        GetEmbeddedCapsuleAxes(capsule, out Vector3d axis, out Vector3d normal);
        return CheckTrianglePrismAxis(triangle, capsule, axis, ref penetration)
            && CheckTrianglePrismAxis(triangle, capsule, normal, ref penetration);
    }

    private static bool CheckTriangleEmbeddedCapsuleEdgeAxis(
        MixedTriangle triangle,
        LSCapsuleCollider2D capsule,
        Vector3d triangleEdge,
        ref AxisPenetration penetration)
    {
        GetEmbeddedCapsuleAxes(capsule, out Vector3d capsuleAxis, out _);
        return CheckTrianglePrismAxis(triangle, capsule, Vector3d.Cross(triangleEdge, capsuleAxis), ref penetration);
    }

    private static void BuildMeshContact(
        LSCollider2D embedded,
        MixedTriangle triangle,
        AxisPenetration penetration,
        out MixedContact contact)
    {
        Vector3d embeddedCenter = GetEmbeddedCenter3D(embedded);
        Vector3d point3D = MeshUtils.ClosestPointOnTriangle(triangle.A, triangle.B, triangle.C, triangle.Normalized, embeddedCenter);
        Vector3d point2D = MixedEmbedded2DGeometry.GetClosestPointOnEmbeddedVolume(embedded, point3D);
        contact = new MixedContact(point3D, point2D, penetration.Axis, penetration.Depth);
    }

    private static void ClosestPointsSegmentTriangle(
        Vector3d segmentStart,
        Vector3d segmentEnd,
        MixedTriangle triangle,
        out Vector3d pointOnSegment,
        out Vector3d pointOnTriangle)
    {
        pointOnSegment = segmentStart;
        pointOnTriangle = MeshUtils.ClosestPointOnTriangle(triangle.A, triangle.B, triangle.C, triangle.Normalized, segmentStart);
        Fixed64 bestDistanceSqr = Vector3d.DistanceSquared(pointOnSegment, pointOnTriangle);

        Vector3d segment = segmentEnd - segmentStart;
        Fixed64 denominator = Vector3d.Dot(triangle.Normalized, segment);
        if (denominator.Abs() > Fixed64.Epsilon)
        {
            Fixed64 t = Vector3d.Dot(triangle.Normalized, triangle.A - segmentStart) / denominator;
            if (t >= Fixed64.Zero && t <= Fixed64.One)
            {
                Vector3d intersection = segmentStart + segment * t;
                if (MeshUtils.IsPointInTrianglePlane(triangle.A, triangle.B, triangle.C, triangle.Normalized, intersection))
                {
                    pointOnSegment = intersection;
                    pointOnTriangle = intersection;
                    return;
                }
            }
        }

        TrySetCloserPointTriangle(segmentEnd, triangle, ref pointOnSegment, ref pointOnTriangle, ref bestDistanceSqr);
        TrySetCloserSegmentEdge(segmentStart, segmentEnd, triangle.A, triangle.B, ref pointOnSegment, ref pointOnTriangle, ref bestDistanceSqr);
        TrySetCloserSegmentEdge(segmentStart, segmentEnd, triangle.B, triangle.C, ref pointOnSegment, ref pointOnTriangle, ref bestDistanceSqr);
        TrySetCloserSegmentEdge(segmentStart, segmentEnd, triangle.C, triangle.A, ref pointOnSegment, ref pointOnTriangle, ref bestDistanceSqr);
    }

    private static void TrySetCloserPointTriangle(
        Vector3d point,
        MixedTriangle triangle,
        ref Vector3d pointOnSegment,
        ref Vector3d pointOnTriangle,
        ref Fixed64 bestDistanceSqr)
    {
        Vector3d candidate = MeshUtils.ClosestPointOnTriangle(triangle.A, triangle.B, triangle.C, triangle.Normalized, point);
        Fixed64 distanceSqr = Vector3d.DistanceSquared(point, candidate);
        if (distanceSqr >= bestDistanceSqr)
            return;

        bestDistanceSqr = distanceSqr;
        pointOnSegment = point;
        pointOnTriangle = candidate;
    }

    private static void TrySetCloserSegmentEdge(
        Vector3d segmentStart,
        Vector3d segmentEnd,
        Vector3d edgeStart,
        Vector3d edgeEnd,
        ref Vector3d pointOnSegment,
        ref Vector3d pointOnTriangle,
        ref Fixed64 bestDistanceSqr)
    {
        (Vector3d segmentPoint, Vector3d edgePoint) = ClosestPointsOnSegments(segmentStart, segmentEnd, edgeStart, edgeEnd);
        Fixed64 distanceSqr = Vector3d.DistanceSquared(segmentPoint, edgePoint);
        if (distanceSqr >= bestDistanceSqr)
            return;

        bestDistanceSqr = distanceSqr;
        pointOnSegment = segmentPoint;
        pointOnTriangle = edgePoint;
    }

    private static void GetMeshTriangle(LSMeshCollider mesh, int triangleIndex, out MixedTriangle triangle)
    {
        mesh.Mesh.GetTriangleVertices(triangleIndex, out Vector3d first, out Vector3d second, out Vector3d third);
        triangle = new MixedTriangle(
            first,
            second,
            third,
            mesh.Mesh.GetFaceNormalWorld(triangleIndex),
            CreateTriangleBounds(first, second, third));
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static FixedRange ProjectTriangleOntoAxis(MixedTriangle triangle, Vector3d axis)
    {
        Fixed64 min = Vector3d.Dot(axis, triangle.A);
        Fixed64 max = min;
        IncludeProjection(axis, triangle.B, ref min, ref max);
        IncludeProjection(axis, triangle.C, ref min, ref max);
        return new FixedRange(min, max);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static void IncludeProjection(Vector3d axis, Vector3d point, ref Fixed64 min, ref Fixed64 max)
    {
        Fixed64 projection = Vector3d.Dot(axis, point);
        if (projection < min)
            min = projection;
        if (projection > max)
            max = projection;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static FixedBoundVolume CreateTriangleBounds(Vector3d first, Vector3d second, Vector3d third) =>
        new(
            Vector3d.Min(Vector3d.Min(first, second), third),
            Vector3d.Max(Vector3d.Max(first, second), third));

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static bool BoundsOverlap(FixedBoundVolume first, FixedBoundBox second) =>
        first.Max.X >= second.Min.X
        && first.Min.X <= second.Max.X
        && first.Max.Y >= second.Min.Y
        && first.Min.Y <= second.Max.Y
        && first.Max.Z >= second.Min.Z
        && first.Min.Z <= second.Max.Z;

}
