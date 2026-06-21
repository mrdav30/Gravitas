//=======================================================================
// MeshTriangleContactGenerator.cs
//=======================================================================
// MIT License, Copyright (c) 2026–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp;
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
            GetTriangle(meshA, triangleA, out TriangleData first);
            meshB.GetTrianglesInBounds(first.Bounds, triangleBufferB);

            for (int j = 0; j < triangleBufferB.Count; j++)
            {
                int triangleB = triangleBufferB[j];
                GetTriangle(meshB, triangleB, out TriangleData second);
                if (!TryTestTriangles(first, second, meshB.Center - meshA.Center, out Vector3d normal, out Fixed64 depth))
                    continue;

                Vector3d pointA = MeshUtils.ClosestPointOnTriangle(first.A, first.B, first.C, first.Normal, second.Center);
                Vector3d pointB = MeshUtils.ClosestPointOnTriangle(second.A, second.B, second.C, second.Normal, pointA);
                if (Vector3d.DistanceSquared(pointA, pointB) <= Fixed64.Epsilon)
                    pointB = pointA - normal * depth;

                AddContactInPairOrder(pair, meshA, pointA, meshB, pointB, depth, normal);
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
        GetTriangle(mesh, triangleIndex, out TriangleData triangle);
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
        AddContactInPairOrder(pair, mesh, pointOnMesh, sphere, pointOnSphere, sphere.ScaledRadius - distance, normal);
    }

    private static void TryAddCapsuleTriangleContact(
        CollisionWorkItem pair,
        LSMeshCollider mesh,
        int triangleIndex,
        LSCapsuleCollider capsule)
    {
        GetTriangle(mesh, triangleIndex, out TriangleData triangle);
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
        AddContactInPairOrder(pair, mesh, pointOnMesh, capsule, pointOnCapsule, capsule.ScaledRadius - distance, normal);
    }

    private static void TryAddCuboidTriangleContact(
        CollisionWorkItem pair,
        LSMeshCollider mesh,
        int triangleIndex,
        LSCuboidCollider cuboid)
    {
        GetTriangle(mesh, triangleIndex, out TriangleData triangle);
        if (!TryTestTriangleCuboid(triangle, cuboid, out Vector3d normal, out Fixed64 depth))
            return;

        Vector3d pointOnMesh = MeshUtils.ClosestPointOnTriangle(triangle.A, triangle.B, triangle.C, triangle.Normal, cuboid.Center);
        Vector3d pointOnCuboid = pointOnMesh - normal * depth;
        AddContactInPairOrder(pair, mesh, pointOnMesh, cuboid, pointOnCuboid, depth, normal);
    }

    private static void TryAddCylinderTriangleContact(
        CollisionWorkItem pair,
        LSMeshCollider mesh,
        int triangleIndex,
        LSCylinderCollider cylinder)
    {
        GetTriangle(mesh, triangleIndex, out TriangleData triangle);
        Vector3d pointOnMesh = MeshUtils.ClosestPointOnTriangle(triangle.A, triangle.B, triangle.C, triangle.Normal, cylinder.Center);
        if (!IsPointInsideCylinder(cylinder, pointOnMesh))
            return;

        Vector3d pointOnCylinder = cylinder.ClosestPointOnSurface(pointOnMesh);
        Fixed64 depth = Vector3d.Distance(pointOnMesh, pointOnCylinder);
        Vector3d normal = OrientNormal(triangle.Normal, cylinder.Center - pointOnMesh);
        AddContactInPairOrder(pair, mesh, pointOnMesh, cylinder, pointOnCylinder, depth, normal);
    }

    private static bool TryTestTriangleCuboid(
        TriangleData triangle,
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
            Vector3d triangleEdge = triangle.GetEdge(i);
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
        TriangleData first,
        TriangleData second,
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
            Vector3d firstEdge = first.GetEdge(i);
            if (!CheckTriangleTriangleAxis(first, second, Vector3d.Cross(first.Normal, firstEdge), desiredDirection, ref normal, ref depth))
                return false;

            Vector3d secondEdge = second.GetEdge(i);
            if (!CheckTriangleTriangleAxis(first, second, Vector3d.Cross(second.Normal, secondEdge), desiredDirection, ref normal, ref depth))
                return false;

            for (int j = 0; j < 3; j++)
            {
                Vector3d axis = Vector3d.Cross(firstEdge, second.GetEdge(j));
                if (!CheckTriangleTriangleAxis(first, second, axis, desiredDirection, ref normal, ref depth))
                    return false;
            }
        }

        return normal.MagnitudeSquared > Fixed64.Epsilon;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static bool CheckTriangleCuboidAxis(
        TriangleData triangle,
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
        TriangleData first,
        TriangleData second,
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

        if (overlap < Fixed64.Zero)
            overlap = Fixed64.Zero;

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

        if (overlap < Fixed64.Zero)
            overlap = Fixed64.Zero;

        Fixed64 axisMagnitude = FixedMath.Sqrt(axisMagnitudeSqr);
        depth = overlap / axisMagnitude;
        normal = OrientNormal(axis / axisMagnitude, desiredDirection);
        return true;
    }

    private static void ClosestPointsSegmentTriangle(
        Vector3d segmentStart,
        Vector3d segmentEnd,
        TriangleData triangle,
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
        TriangleData triangle,
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
        (Vector3d segmentPoint, Vector3d edgePoint) = Vector3d.ClosestPointsOnTwoLines(segmentStart, segmentEnd, edgeStart, edgeEnd);
        Fixed64 distanceSqr = Vector3d.DistanceSquared(segmentPoint, edgePoint);
        if (distanceSqr >= bestDistanceSqr)
            return;

        bestDistanceSqr = distanceSqr;
        pointOnSegment = segmentPoint;
        pointOnTriangle = edgePoint;
    }

    private static void GetTriangle(LSMeshCollider mesh, int triangleIndex, out TriangleData triangle)
    {
        mesh.Mesh.GetTriangleVertices(triangleIndex, out Vector3d first, out Vector3d second, out Vector3d third);
        triangle = new TriangleData(
            first,
            second,
            third,
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

    private static void ProjectTriangle(TriangleData triangle, Vector3d axis, out Fixed64 min, out Fixed64 max)
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
        Vector3d resolved = normal.MagnitudeSquared > Fixed64.Epsilon
            ? normal.Normalized
            : ResolveNormal(desiredDirection);
        return Vector3d.Dot(resolved, desiredDirection) < Fixed64.Zero ? -resolved : resolved;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static Vector3d ResolveNormal(Vector3d direction) =>
        direction.MagnitudeSquared > Fixed64.Epsilon ? direction.Normalized : Vector3d.Right;

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static void AddContactInPairOrder(
        CollisionWorkItem pair,
        LSCollider first,
        Vector3d pointOnFirst,
        LSCollider second,
        Vector3d pointOnSecond,
        Fixed64 depth,
        Vector3d normalFirstToSecond)
    {
        if (ReferenceEquals(pair.ColliderA, first))
        {
            pair.Manifold.AddContact(pointOnFirst, pointOnSecond, depth, normalFirstToSecond);
            return;
        }

        if (ReferenceEquals(pair.ColliderA, second))
            pair.Manifold.AddContact(pointOnSecond, pointOnFirst, depth, -normalFirstToSecond);
    }

    private readonly struct TriangleData
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public TriangleData(Vector3d a, Vector3d b, Vector3d c, Vector3d normal, FixedBoundVolume bounds)
        {
            A = a;
            B = b;
            C = c;
            Normal = normal;
            Bounds = bounds;
            Center = (a + b + c) / (Fixed64)3;
        }

        public Vector3d A { get; }

        public Vector3d B { get; }

        public Vector3d C { get; }

        public Vector3d Normal { get; }

        public FixedBoundVolume Bounds { get; }

        public Vector3d Center { get; }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public Vector3d GetEdge(int index) =>
            index switch
            {
                0 => B - A,
                1 => C - B,
                _ => A - C
            };
    }
}
