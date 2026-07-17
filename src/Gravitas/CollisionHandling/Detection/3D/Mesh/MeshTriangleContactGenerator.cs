//=======================================================================
// MeshTriangleContactGenerator.cs
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

/// <summary>
/// Builds deterministic triangle-level manifolds for mesh paths that cannot use
/// whole-shape convex assumptions.
/// </summary>
internal static class MeshTriangleContactGenerator
{
    public static bool TryBuildMeshSphereManifold(
        CollisionWorkItem pair,
        LSMeshCollider mesh,
        LSSphereCollider sphere,
        SwiftList<int> triangleBuffer)
    {
        mesh.GetTrianglesInBounds(CreateBounds(sphere.BoundsMin, sphere.BoundsMax), triangleBuffer);
        for (int i = 0; i < triangleBuffer.Count; i++)
            TryAddSphereTriangleContact(pair, mesh, triangleBuffer[i], sphere);

        return pair.Manifold.HasContact;
    }

    public static bool TryBuildMeshCapsuleManifold(
        CollisionWorkItem pair,
        LSMeshCollider mesh,
        LSCapsuleCollider capsule,
        SwiftList<int> triangleBuffer)
    {
        Vector3d radius = Vector3d.One * capsule.ScaledRadius;
        Vector3d min = Vector3d.Min(capsule.LineSegmentStart, capsule.LineSegmentEnd) - radius;
        Vector3d max = Vector3d.Max(capsule.LineSegmentStart, capsule.LineSegmentEnd) + radius;
        mesh.GetTrianglesInBounds(CreateBounds(min, max), triangleBuffer);

        for (int i = 0; i < triangleBuffer.Count; i++)
            TryAddCapsuleTriangleContact(pair, mesh, triangleBuffer[i], capsule);

        return pair.Manifold.HasContact;
    }

    public static bool TryBuildMeshCuboidManifold(
        CollisionWorkItem pair,
        LSMeshCollider mesh,
        LSCuboidCollider cuboid,
        SwiftList<int> triangleBuffer)
    {
        mesh.GetTrianglesInBounds(CreateBounds(cuboid.BoundsMin, cuboid.BoundsMax), triangleBuffer);
        for (int i = 0; i < triangleBuffer.Count; i++)
            TryAddCuboidTriangleContact(pair, mesh, triangleBuffer[i], cuboid);

        return pair.Manifold.HasContact;
    }

    public static bool TryBuildMeshCylinderManifold(
        CollisionWorkItem pair,
        LSMeshCollider mesh,
        LSCylinderCollider cylinder,
        SwiftList<int> triangleBuffer)
    {
        mesh.GetTrianglesInBounds(CreateBounds(cylinder.BoundsMin, cylinder.BoundsMax), triangleBuffer);
        for (int i = 0; i < triangleBuffer.Count; i++)
            TryAddCylinderTriangleContact(pair, mesh, triangleBuffer[i], cylinder);

        return pair.Manifold.HasContact;
    }

    public static bool TryBuildMeshMeshManifold(
        CollisionWorkItem pair,
        LSMeshCollider meshA,
        LSMeshCollider meshB,
        SwiftList<int> triangleBufferA,
        SwiftList<int> triangleBufferB)
    {
        meshA.GetTrianglesInBounds(CreateBounds(meshB.BoundsMin, meshB.BoundsMax), triangleBufferA);
        for (int i = 0; i < triangleBufferA.Count; i++)
        {
            int triangleA = triangleBufferA[i];
            GetTriangle(meshA, triangleA, out CollisionTriangle first);
            meshB.GetTrianglesInBounds(first.QueryBounds, triangleBufferB);

            for (int j = 0; j < triangleBufferB.Count; j++)
            {
                int triangleB = triangleBufferB[j];
                GetTriangle(meshB, triangleB, out CollisionTriangle second);
                if (!TryTestTriangles(first, second, meshB.Center - meshA.Center, out Vector3d normal, out Fixed64 depth))
                    continue;

                Vector3d pointA = MeshUtils.ClosestPointOnTriangle(first.A, first.B, first.C, first.Normal, second.Center);
                Vector3d pointB = MeshUtils.ClosestPointOnTriangle(second.A, second.B, second.C, second.Normal, pointA);
                if (Vector3d.DistanceSquared(pointA, pointB) <= Fixed64.Epsilon)
                    pointB = pointA - normal * depth;

                AddContact(pair, pointA, pointB, depth, normal);
            }
        }

        return pair.Manifold.HasContact;
    }

    private static void TryAddSphereTriangleContact(
        CollisionWorkItem pair,
        LSMeshCollider mesh,
        int triangleIndex,
        LSSphereCollider sphere)
    {
        GetTriangle(mesh, triangleIndex, out CollisionTriangle triangle);
        Vector3d pointOnMesh = MeshUtils.ClosestPointOnTriangle(triangle.A, triangle.B, triangle.C, triangle.Normal, sphere.Center);
        Vector3d delta = sphere.Center - pointOnMesh;
        Fixed64 distanceSqr = delta.MagnitudeSquared;
        if (distanceSqr > sphere.ScaledRadiusSqr + Fixed64.Epsilon)
            return;

        Fixed64 distance = distanceSqr <= Fixed64.Epsilon
            ? Fixed64.Zero
            : FixedMath.Sqrt(distanceSqr);
        Vector3d normal = distance > Fixed64.Epsilon
            ? delta / distance
            : OrientNormal(triangle.Normal, sphere.Center - triangle.Center);
        Vector3d pointOnSphere = sphere.Center - normal * sphere.ScaledRadius;
        AddContact(pair, pointOnMesh, pointOnSphere, sphere.ScaledRadius - distance, normal);
    }

    private static void TryAddCapsuleTriangleContact(
        CollisionWorkItem pair,
        LSMeshCollider mesh,
        int triangleIndex,
        LSCapsuleCollider capsule)
    {
        GetTriangle(mesh, triangleIndex, out CollisionTriangle triangle);
        ClosestPointsSegmentTriangle(
            capsule.LineSegmentStart,
            capsule.LineSegmentEnd,
            triangle,
            out Vector3d pointOnCapsuleLine,
            out Vector3d pointOnMesh);

        Vector3d delta = pointOnCapsuleLine - pointOnMesh;
        Fixed64 distanceSqr = delta.MagnitudeSquared;
        if (distanceSqr > capsule.ScaledRadiusSqr + Fixed64.Epsilon)
            return;

        Fixed64 distance = distanceSqr <= Fixed64.Epsilon
            ? Fixed64.Zero
            : FixedMath.Sqrt(distanceSqr);
        Vector3d normal = distance > Fixed64.Epsilon
            ? delta / distance
            : OrientNormal(triangle.Normal, capsule.Center - triangle.Center);
        Vector3d pointOnCapsule = pointOnCapsuleLine - normal * capsule.ScaledRadius;
        AddContact(pair, pointOnMesh, pointOnCapsule, capsule.ScaledRadius - distance, normal);
    }

    private static void TryAddCuboidTriangleContact(
        CollisionWorkItem pair,
        LSMeshCollider mesh,
        int triangleIndex,
        LSCuboidCollider cuboid)
    {
        GetTriangle(mesh, triangleIndex, out CollisionTriangle triangle);
        if (!TryTestTriangleCuboid(triangle, cuboid, out Vector3d normal, out Fixed64 depth))
            return;

        if (TryAddCuboidFaceContacts(pair, mesh, triangle, cuboid, normal, depth))
            return;

        Vector3d pointOnMesh = MeshUtils.ClosestPointOnTriangle(triangle.A, triangle.B, triangle.C, triangle.Normal, cuboid.Center);
        Vector3d pointOnCuboid = pointOnMesh - normal * depth;
        AddContact(pair, pointOnMesh, pointOnCuboid, depth, normal);
    }

    private static void TryAddCylinderTriangleContact(
        CollisionWorkItem pair,
        LSMeshCollider mesh,
        int triangleIndex,
        LSCylinderCollider cylinder)
    {
        GetTriangle(mesh, triangleIndex, out CollisionTriangle triangle);
        Vector3d normal = OrientNormal(triangle.Normal, cylinder.Center - triangle.Center);
        if (TryAddCylinderCapTriangleContacts(pair, mesh, triangle, cylinder, normal))
            return;

        Vector3d pointOnMesh = MeshUtils.ClosestPointOnTriangle(triangle.A, triangle.B, triangle.C, triangle.Normal, cylinder.Center);
        if (!IsPointInsideCylinder(cylinder, pointOnMesh))
            return;

        Vector3d pointOnCylinder = cylinder.ClosestPointOnSurface(pointOnMesh);
        Fixed64 depth = Vector3d.Distance(pointOnMesh, pointOnCylinder);
        AddContact(pair, pointOnMesh, pointOnCylinder, depth, normal);
    }

    private static bool TryAddCuboidFaceContacts(
        CollisionWorkItem pair,
        LSMeshCollider mesh,
        CollisionTriangle triangle,
        LSCuboidCollider cuboid,
        Vector3d normal,
        Fixed64 depth)
    {
        Fixed64 minProjection = Fixed64.MaxValue;
        for (int i = 0; i < cuboid.Vertices.Length; i++)
        {
            Fixed64 projection = Vector3d.Dot(cuboid.Vertices[i], normal);
            if (projection < minProjection)
                minProjection = projection;
        }

        int initialCount = pair.Manifold.Count;
        for (int i = 0; i < cuboid.Vertices.Length; i++)
        {
            Vector3d pointOnCuboid = cuboid.Vertices[i];
            if (Vector3d.Dot(pointOnCuboid, normal) > minProjection + Fixed64.Epsilon)
                continue;

            Vector3d pointOnMesh = pointOnCuboid + normal * depth;
            if (!MeshUtils.IsPointInTrianglePlane(triangle.A, triangle.B, triangle.C, triangle.Normal, pointOnMesh))
                continue;

            AddContact(pair, pointOnMesh, pointOnCuboid, depth, normal);
        }

        return pair.Manifold.Count > initialCount;
    }

    private static bool TryAddCylinderCapTriangleContacts(
        CollisionWorkItem pair,
        LSMeshCollider mesh,
        CollisionTriangle triangle,
        LSCylinderCollider cylinder,
        Vector3d normal)
    {
        if (!CylinderContactGeometry.IsAxisAligned(normal, cylinder.LineDirection))
            return false;

        Vector3d capCenter = CylinderContactGeometry.GetCapCenter(cylinder, -normal);
        Fixed64 signedDistance = Vector3d.Dot(capCenter - triangle.A, normal);
        if (signedDistance > Fixed64.Epsilon)
            return false;

        Fixed64 depth = signedDistance < Fixed64.Zero ? -signedDistance : Fixed64.Zero;
        int initialCount = pair.Manifold.Count;
        CylinderContactGeometry.GetCapBasis(cylinder, out Vector3d tangentA, out Vector3d tangentB);
        AddCylinderCapTriangleContact(pair, mesh, triangle, cylinder, capCenter + tangentA * cylinder.ScaledRadius, depth, normal);
        AddCylinderCapTriangleContact(pair, mesh, triangle, cylinder, capCenter - tangentA * cylinder.ScaledRadius, depth, normal);
        AddCylinderCapTriangleContact(pair, mesh, triangle, cylinder, capCenter + tangentB * cylinder.ScaledRadius, depth, normal);
        AddCylinderCapTriangleContact(pair, mesh, triangle, cylinder, capCenter - tangentB * cylinder.ScaledRadius, depth, normal);

        if (pair.Manifold.Count > initialCount)
            return true;

        AddCylinderCapTriangleContact(pair, mesh, triangle, cylinder, capCenter, depth, normal);
        return pair.Manifold.Count > initialCount;
    }

    private static void AddCylinderCapTriangleContact(
        CollisionWorkItem pair,
        LSMeshCollider mesh,
        CollisionTriangle triangle,
        LSCylinderCollider cylinder,
        Vector3d pointOnCylinder,
        Fixed64 depth,
        Vector3d normal)
    {
        Vector3d pointOnMesh = pointOnCylinder + normal * depth;
        if (!MeshUtils.IsPointInTrianglePlane(triangle.A, triangle.B, triangle.C, triangle.Normal, pointOnMesh))
            return;

        AddContact(pair, pointOnMesh, pointOnCylinder, depth, normal);
    }

    private static bool TryTestTriangleCuboid(
        CollisionTriangle triangle,
        LSCuboidCollider cuboid,
        out Vector3d normal,
        out Fixed64 depth)
    {
        normal = Vector3d.Zero;
        depth = Fixed64.MaxValue;
        Vector3d desiredDirection = cuboid.Center - triangle.Center;

        for (int i = 0; i < cuboid.FaceNormals.Length; i++)
        {
            if (!CheckTriangleCuboidAxis(triangle, cuboid, cuboid.FaceNormals[i], desiredDirection, ref normal, ref depth))
                return false;
        }

        if (!CheckTriangleCuboidAxis(triangle, cuboid, triangle.Normal, desiredDirection, ref normal, ref depth))
            return false;

        for (int i = 0; i < 3; i++)
        {
            Vector3d triangleEdge = triangle.GetEdgeVector(i);
            for (int j = 0; j < cuboid.EdgeDirections.Length; j++)
            {
                Vector3d axis = Vector3d.Cross(triangleEdge, cuboid.EdgeDirections[j]);
                if (!CheckTriangleCuboidAxis(triangle, cuboid, axis, desiredDirection, ref normal, ref depth))
                    return false;
            }
        }

        return normal.MagnitudeSquared > Fixed64.Epsilon;
    }

    private static bool TryTestTriangles(
        CollisionTriangle first,
        CollisionTriangle second,
        Vector3d desiredDirection,
        out Vector3d normal,
        out Fixed64 depth)
    {
        normal = Vector3d.Zero;
        depth = Fixed64.MaxValue;

        if (!CheckTriangleTriangleAxis(first, second, first.Normal, desiredDirection, ref normal, ref depth))
            return false;
        if (!CheckTriangleTriangleAxis(first, second, second.Normal, desiredDirection, ref normal, ref depth))
            return false;

        for (int i = 0; i < 3; i++)
        {
            Vector3d firstEdge = first.GetEdgeVector(i);
            if (!CheckTriangleTriangleAxis(first, second, Vector3d.Cross(first.Normal, firstEdge), desiredDirection, ref normal, ref depth))
                return false;

            Vector3d secondEdge = second.GetEdgeVector(i);
            if (!CheckTriangleTriangleAxis(first, second, Vector3d.Cross(second.Normal, secondEdge), desiredDirection, ref normal, ref depth))
                return false;

            for (int j = 0; j < 3; j++)
            {
                Vector3d axis = Vector3d.Cross(firstEdge, second.GetEdgeVector(j));
                if (!CheckTriangleTriangleAxis(first, second, axis, desiredDirection, ref normal, ref depth))
                    return false;
            }
        }

        return normal.MagnitudeSquared > Fixed64.Epsilon;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static bool CheckTriangleCuboidAxis(
        CollisionTriangle triangle,
        LSCuboidCollider cuboid,
        Vector3d axis,
        Vector3d desiredDirection,
        ref Vector3d normal,
        ref Fixed64 depth)
    {
        if (!TryNormalizeAxis(axis, out Vector3d NormalAxis))
            return true;

        ProjectTriangle(triangle, NormalAxis, out Fixed64 minA, out Fixed64 maxA);
        ProjectVertices(cuboid.Vertices, NormalAxis, out Fixed64 minB, out Fixed64 maxB);
        return CheckProjectedAxis(minA, maxA, minB, maxB, NormalAxis, desiredDirection, ref normal, ref depth);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static bool CheckTriangleTriangleAxis(
        CollisionTriangle first,
        CollisionTriangle second,
        Vector3d axis,
        Vector3d desiredDirection,
        ref Vector3d normal,
        ref Fixed64 depth)
    {
        Fixed64 axisMagnitudeSqr = axis.MagnitudeSquared;
        if (axisMagnitudeSqr <= Fixed64.Epsilon)
            return true;

        ProjectTriangle(first, axis, out Fixed64 minA, out Fixed64 maxA);
        ProjectTriangle(second, axis, out Fixed64 minB, out Fixed64 maxB);
        return CheckProjectedTriangleAxis(minA, maxA, minB, maxB, axis, axisMagnitudeSqr, desiredDirection, ref normal, ref depth);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static bool CheckProjectedAxis(
        Fixed64 minA,
        Fixed64 maxA,
        Fixed64 minB,
        Fixed64 maxB,
        Vector3d axis,
        Vector3d desiredDirection,
        ref Vector3d normal,
        ref Fixed64 depth)
    {
        if (maxA < minB || maxB < minA)
            return false;

        Fixed64 overlap = FixedMath.Min(maxA - minB, maxB - minA);
        if (overlap > Fixed64.Zero && overlap >= depth)
            return true;

        depth = overlap;
        normal = OrientNormal(axis, desiredDirection);
        return true;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static bool CheckProjectedTriangleAxis(
        Fixed64 minA,
        Fixed64 maxA,
        Fixed64 minB,
        Fixed64 maxB,
        Vector3d axis,
        Fixed64 axisMagnitudeSqr,
        Vector3d desiredDirection,
        ref Vector3d normal,
        ref Fixed64 depth)
    {
        if (maxA < minB || maxB < minA)
            return false;

        Fixed64 overlap = FixedMath.Min(maxA - minB, maxB - minA);
        if (overlap > Fixed64.Zero
            && depth != Fixed64.MaxValue
            && overlap * overlap >= depth * depth * axisMagnitudeSqr)
        {
            return true;
        }

        Fixed64 axisMagnitude = FixedMath.Sqrt(axisMagnitudeSqr);
        depth = overlap / axisMagnitude;
        normal = OrientNormal(axis / axisMagnitude, desiredDirection);
        return true;
    }

    private static void ClosestPointsSegmentTriangle(
        Vector3d segmentStart,
        Vector3d segmentEnd,
        CollisionTriangle triangle,
        out Vector3d pointOnSegment,
        out Vector3d pointOnTriangle)
    {
        pointOnSegment = segmentStart;
        pointOnTriangle = MeshUtils.ClosestPointOnTriangle(triangle.A, triangle.B, triangle.C, triangle.Normal, segmentStart);
        Fixed64 bestDistanceSqr = Vector3d.DistanceSquared(pointOnSegment, pointOnTriangle);

        Vector3d segment = segmentEnd - segmentStart;
        Fixed64 denominator = Vector3d.Dot(triangle.Normal, segment);
        if (denominator.Abs() > Fixed64.Epsilon)
        {
            Fixed64 t = Vector3d.Dot(triangle.Normal, triangle.A - segmentStart) / denominator;
            if (t >= Fixed64.Zero && t <= Fixed64.One)
            {
                Vector3d intersection = segmentStart + segment * t;
                if (MeshUtils.IsPointInTrianglePlane(triangle.A, triangle.B, triangle.C, triangle.Normal, intersection))
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
        CollisionTriangle triangle,
        ref Vector3d pointOnSegment,
        ref Vector3d pointOnTriangle,
        ref Fixed64 bestDistanceSqr)
    {
        Vector3d candidate = MeshUtils.ClosestPointOnTriangle(triangle.A, triangle.B, triangle.C, triangle.Normal, point);
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
        (Vector3d segmentPoint, Vector3d edgePoint) = new FixedSegment(
            segmentStart,
            segmentEnd).GetClosestPoints(new FixedSegment(edgeStart, edgeEnd));
        Fixed64 distanceSqr = Vector3d.DistanceSquared(segmentPoint, edgePoint);
        if (distanceSqr >= bestDistanceSqr)
            return;

        bestDistanceSqr = distanceSqr;
        pointOnSegment = segmentPoint;
        pointOnTriangle = edgePoint;
    }

    private static void GetTriangle(LSMeshCollider mesh, int triangleIndex, out CollisionTriangle triangle)
    {
        mesh.Mesh.GetTriangleVertices(triangleIndex, out Vector3d first, out Vector3d second, out Vector3d third);
        triangle = new CollisionTriangle(
            new FixedTriangle(first, second, third),
            mesh.Mesh.GetFaceNormalWorld(triangleIndex),
            CreateTriangleBounds(first, second, third));
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static FixedBoundVolume CreateTriangleBounds(Vector3d first, Vector3d second, Vector3d third) =>
        CreateBounds(
            Vector3d.Min(Vector3d.Min(first, second), third),
            Vector3d.Max(Vector3d.Max(first, second), third));

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static FixedBoundVolume CreateBounds(Vector3d min, Vector3d max) =>
        new(min, max);

    private static void ProjectTriangle(CollisionTriangle triangle, Vector3d axis, out Fixed64 min, out Fixed64 max)
    {
        min = Vector3d.Dot(axis, triangle.A);
        max = min;
        IncludeProjection(axis, triangle.B, ref min, ref max);
        IncludeProjection(axis, triangle.C, ref min, ref max);
    }

    private static void ProjectVertices(Vector3d[] vertices, Vector3d axis, out Fixed64 min, out Fixed64 max)
    {
        min = Vector3d.Dot(axis, vertices[0]);
        max = min;
        for (int i = 1; i < vertices.Length; i++)
            IncludeProjection(axis, vertices[i], ref min, ref max);
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
    private static bool TryNormalizeAxis(Vector3d axis, out Vector3d NormalAxis)
    {
        Fixed64 magnitudeSqr = axis.MagnitudeSquared;
        if (magnitudeSqr <= Fixed64.Epsilon)
        {
            NormalAxis = Vector3d.Zero;
            return false;
        }

        NormalAxis = axis / FixedMath.Sqrt(magnitudeSqr);
        return true;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static bool IsPointInsideCylinder(LSCylinderCollider cylinder, Vector3d point)
    {
        Vector3d local = cylinder.Rotation.Inverse() * (point - cylinder.Center);
        Fixed64 radialSqr = local.X * local.X + local.Z * local.Z;
        return radialSqr <= cylinder.ScaledRadiusSqr + Fixed64.Epsilon
            && local.Y >= -cylinder.HalfHeight - Fixed64.Epsilon
            && local.Y <= cylinder.HalfHeight + Fixed64.Epsilon;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static Vector3d OrientNormal(Vector3d normal, Vector3d desiredDirection)
    {
        Vector3d resolved = normal.Normalized;
        return Vector3d.Dot(resolved, desiredDirection) < Fixed64.Zero ? -resolved : resolved;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static void AddContact(
        CollisionWorkItem pair,
        Vector3d pointOnFirst,
        Vector3d pointOnSecond,
        Fixed64 depth,
        Vector3d normalFirstToSecond)
    {
        pair.Manifold.AddContact(pointOnFirst, pointOnSecond, depth, normalFirstToSecond);
    }

}
