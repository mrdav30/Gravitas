//=======================================================================
// MeshTriangleContactGenerator.cs
//=======================================================================
// MIT License, Copyright (c) 2026–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp;
using FixedMathSharp.Geometry;
using Gravitas.Colliders;
using SwiftCollections;
using SwiftCollections.Query;
using System;
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
        mesh.GetTrianglesInBounds(CreateBounds(capsule.BoundsMin, capsule.BoundsMax), triangleBuffer);

        bool overlaps = false;
        for (int i = 0; i < triangleBuffer.Count; i++)
            overlaps |= TryAddCapsuleTriangleContact(pair, mesh, triangleBuffer[i], capsule);

        return overlaps;
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
        bool overlaps = false;
        for (int i = 0; i < triangleBuffer.Count; i++)
            overlaps |= TryAddCylinderTriangleContact(pair, mesh, triangleBuffer[i], cylinder);

        return overlaps;
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
            meshA.Mesh.GetLocalTriangleVertices(
                triangleA,
                out Vector3d firstVertex,
                out Vector3d secondVertex,
                out Vector3d thirdVertex);
            var firstTriangle =
                new FixedTriangle(firstVertex, secondVertex, thirdVertex);
            var first = new CollisionTriangle(
                firstTriangle,
                firstTriangle.Normal,
                CreateTriangleBounds(firstVertex, secondVertex, thirdVertex));
            GetTriangleInFrame(
                meshA,
                triangleA,
                meshB.Mesh.Origin,
                meshB.Mesh.Rotation,
                out CollisionTriangle firstInSecondFrame);
            meshB.Mesh.GetTrianglesInLocalBounds(
                firstInSecondFrame.QueryBounds,
                triangleBufferB);

            for (int j = 0; j < triangleBufferB.Count; j++)
            {
                int triangleB = triangleBufferB[j];
                GetTriangleInFrame(
                    meshB,
                    triangleB,
                    meshA.Mesh.Origin,
                    meshA.Mesh.Rotation,
                    out CollisionTriangle second);

                Vector3d desiredDirection =
                    second.Center - first.Center;
                if (!TryTestTriangles(
                        first,
                        second,
                        desiredDirection,
                        out Vector3d localNormal,
                        out Fixed64 depth))
                {
                    continue;
                }

                Vector3d pointA = first.Triangle.ClosestPoint(second.Center);
                Vector3d pointB = second.Triangle.ClosestPoint(pointA);
                if (Vector3d.DistanceSquared(pointA, pointB) <= Fixed64.Epsilon)
                    pointB = pointA - localNormal * depth;

                FixedPointAnchor firstAnchor =
                    meshA.Mesh.CreatePointAnchor(pointA);
                var secondInFirstFrame = new FixedPointAnchor(
                    meshA.Mesh.Origin,
                    meshA.Mesh.Rotation,
                    pointB);
                _ = secondInFirstFrame.TryGetLocalPointIn(
                    meshB.Mesh.Origin,
                    meshB.Mesh.Rotation,
                    out Vector3d secondLocalPoint);

                AddContact(
                    pair,
                    new ContactAnchor(firstAnchor),
                    new ContactAnchor(
                        meshB.Mesh.Origin,
                        meshB.Mesh.Rotation,
                        secondLocalPoint),
                    depth,
                    meshA.Mesh.Rotation.Rotate(localNormal).Normalized,
                    depthIsClamped: false);
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
        mesh.Mesh.GetLocalTriangleVertices(
            triangleIndex,
            out Vector3d first,
            out Vector3d second,
            out Vector3d third);
        if (!new FixedTriangle(first, second, third).TryGetSphereContact(
                mesh.Mesh.Origin,
                mesh.Mesh.Rotation,
                sphere.Center,
                sphere.Rotation,
                sphere.ScaledRadius,
                out FixedContactAnchors contact))
        {
            return;
        }

        AddContact(
            pair,
            new ContactAnchor(contact.FirstAnchor),
            new ContactAnchor(contact.SecondAnchor),
            contact.Depth,
            contact.Normal,
            contact.DepthIsClamped);
    }

    private static bool TryAddCapsuleTriangleContact(
        CollisionWorkItem pair,
        LSMeshCollider mesh,
        int triangleIndex,
        LSCapsuleCollider capsule)
    {
        mesh.Mesh.GetLocalTriangleVertices(
            triangleIndex,
            out Vector3d first,
            out Vector3d second,
            out Vector3d third);
        var triangle = new FixedTriangle(first, second, third);
        Vector3d fallbackNormal = OrientNormal(
            mesh.Mesh.CreatePointAnchor(triangle.Centroid),
            new FixedPointAnchor(
                capsule.Center,
                FixedQuaternion.Identity,
                Vector3d.Zero),
            mesh.Mesh.GetFaceNormalWorld(triangleIndex));
        if (!triangle.TryGetCenteredCapsuleContact(
                mesh.Mesh.Origin,
                mesh.Mesh.Rotation,
                capsule.Center,
                capsule.Rotation,
                capsule.AxisLength,
                capsule.ScaledRadius,
                fallbackNormal,
                out FixedContactAnchors contact))
        {
            return false;
        }

        AddContact(
            pair,
            new ContactAnchor(contact.FirstAnchor),
            new ContactAnchor(contact.SecondAnchor),
            contact.Depth,
            contact.Normal,
            contact.DepthIsClamped);
        return true;
    }

    private static void TryAddCuboidTriangleContact(
        CollisionWorkItem pair,
        LSMeshCollider mesh,
        int triangleIndex,
        LSCuboidCollider cuboid)
    {
        mesh.Mesh.GetLocalTriangleVertices(
            triangleIndex,
            out Vector3d first,
            out Vector3d second,
            out Vector3d third);
        var triangle = new FixedTriangle(first, second, third);
        Span<FixedContactLocalPoints> faceContacts =
            stackalloc FixedContactLocalPoints[4];
        if (!cuboid.OrientedBox.TryGetTriangleContact(
                mesh.Mesh.Origin,
                mesh.Mesh.Rotation,
                triangle,
                faceContacts,
                out FixedContactAnchors contact,
                out int faceContactCount))
        {
            return;
        }

        if (faceContactCount > 0)
        {
            for (int index = 0; index < faceContactCount; index++)
            {
                AddReversedBoxTriangleContact(
                    pair,
                    contact,
                    faceContacts[index]);
            }
            return;
        }

        AddReversedBoxTriangleContact(pair, contact);
    }

    private static void AddReversedBoxTriangleContact(
        CollisionWorkItem pair,
        FixedContactAnchors contact)
    {
        AddContact(
            pair,
            new ContactAnchor(contact.SecondAnchor),
            new ContactAnchor(contact.FirstAnchor),
            contact.Depth,
            -contact.Normal,
            contact.DepthIsClamped);
    }

    private static void AddReversedBoxTriangleContact(
        CollisionWorkItem pair,
        FixedContactAnchors primary,
        FixedContactLocalPoints contact)
    {
        AddContact(
            pair,
            new ContactAnchor(
                primary.SecondAnchor.Origin,
                primary.SecondAnchor.Rotation,
                contact.SecondLocalPoint),
            new ContactAnchor(
                primary.FirstAnchor.Origin,
                primary.FirstAnchor.Rotation,
                contact.FirstLocalPoint),
            primary.Depth,
            -primary.Normal,
            primary.DepthIsClamped);
    }

    private static bool TryAddCylinderTriangleContact(
        CollisionWorkItem pair,
        LSMeshCollider mesh,
        int triangleIndex,
        LSCylinderCollider cylinder)
    {
        mesh.Mesh.GetLocalTriangleVertices(
            triangleIndex,
            out Vector3d first,
            out Vector3d second,
            out Vector3d third);
        var triangle = new FixedTriangle(first, second, third);
        var cylinderCenter = new FixedPointAnchor(
            cylinder.Center,
            FixedQuaternion.Identity,
            Vector3d.Zero);
        Vector3d normal = OrientNormal(
            mesh.Mesh.CreatePointAnchor(triangle.Centroid),
            cylinderCenter,
            mesh.Mesh.GetFaceNormalWorld(triangleIndex));
        if (TryAddCylinderCapTriangleContacts(
                pair,
                mesh,
                triangle,
                cylinder,
                normal))
        {
            return true;
        }

        if (!cylinderCenter.TryGetLocalPointIn(
                mesh.Mesh.Origin,
                mesh.Mesh.Rotation,
                out Vector3d localCylinderCenter))
        {
            return false;
        }

        Vector3d localPointOnMesh =
            triangle.ClosestPoint(localCylinderCenter);
        FixedPointAnchor meshAnchor =
            mesh.Mesh.CreatePointAnchor(localPointOnMesh);
        // The closest point belongs to the admitted triangle candidate, so its
        // cylinder-frame offset is finite once the center entered the mesh frame.
        _ = meshAnchor.TryGetLocalPointIn(
            cylinder.Center,
            cylinder.Rotation,
            out Vector3d localPointInCylinder);
        if (!FixedSegment.ContainsPointInCenteredFiniteCylinder(
                localPointInCylinder,
                Vector3d.Zero,
                Vector3d.Up,
                cylinder.Height,
                cylinder.ScaledRadius))
        {
            return false;
        }
        // Exact containment proves the centered canonical surface offset
        // remains inside the admitted radius and height.
        _ = FixedSegment.TryGetClosestCenteredFiniteCylinderSurfaceOffset(
            localPointInCylinder,
            Vector3d.Zero,
            Vector3d.Up,
            cylinder.Height,
            cylinder.ScaledRadius,
            Vector3d.Right,
            out Vector3d localCylinderPoint,
            out _,
            out Fixed64 signedDistance);

        Fixed64 depth = -signedDistance;
        AddContact(
            pair,
            new ContactAnchor(meshAnchor),
            new ContactAnchor(
                cylinder.Center,
                cylinder.Rotation,
                localCylinderPoint),
            depth,
            normal,
            depthIsClamped: false);
        return true;
    }

    private static bool TryAddCylinderCapTriangleContacts(
        CollisionWorkItem pair,
        LSMeshCollider mesh,
        FixedTriangle triangle,
        LSCylinderCollider cylinder,
        Vector3d normal)
    {
        if (!CylinderContactGeometry.IsAxisAligned(
                cylinder.Rotation,
                Vector3d.Up,
                normal))
            return false;

        int initialCount = pair.Manifold.Count;
        CylinderContactGeometry.GetCapBasis(cylinder, out Vector3d tangentA, out Vector3d tangentB);
        TryAddCylinderCapTriangleContact(pair, mesh, triangle, cylinder, -normal + tangentA, normal);
        TryAddCylinderCapTriangleContact(pair, mesh, triangle, cylinder, -normal - tangentA, normal);
        TryAddCylinderCapTriangleContact(pair, mesh, triangle, cylinder, -normal + tangentB, normal);
        TryAddCylinderCapTriangleContact(pair, mesh, triangle, cylinder, -normal - tangentB, normal);

        if (pair.Manifold.Count > initialCount)
            return true;

        TryAddCylinderCapTriangleContact(pair, mesh, triangle, cylinder, -normal, normal);
        return pair.Manifold.Count > initialCount;
    }

    private static void TryAddCylinderCapTriangleContact(
        CollisionWorkItem pair,
        LSMeshCollider mesh,
        FixedTriangle triangle,
        LSCylinderCollider cylinder,
        Vector3d supportDirection,
        Vector3d normal)
    {
        if (!triangle.TryGetCenteredFiniteCylinderSupportContact(
                mesh.Mesh.Origin,
                mesh.Mesh.Rotation,
                cylinder.Center,
                cylinder.Rotation,
                cylinder.Height,
                cylinder.ScaledRadius,
                supportDirection,
                normal,
                out FixedContactAnchors contact))
        {
            return;
        }

        AddContact(
            pair,
            new ContactAnchor(contact.FirstAnchor),
            new ContactAnchor(contact.SecondAnchor),
            contact.Depth,
            contact.Normal,
            contact.DepthIsClamped);
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

    private static void GetTriangleInFrame(
        LSMeshCollider mesh,
        int triangleIndex,
        Vector3d frameOrigin,
        FixedQuaternion frameRotation,
        out CollisionTriangle triangle)
    {
        mesh.Mesh.GetLocalTriangleVertices(
            triangleIndex,
            out Vector3d localFirst,
            out Vector3d localSecond,
            out Vector3d localThird);
        // Scale admission and candidate-bound overlap keep all triangle
        // vertices representable in the paired mesh frame.
        _ = mesh.Mesh.CreatePointAnchor(localFirst).TryGetLocalPointIn(
            frameOrigin,
            frameRotation,
            out Vector3d first);
        _ = mesh.Mesh.CreatePointAnchor(localSecond).TryGetLocalPointIn(
            frameOrigin,
            frameRotation,
            out Vector3d second);
        _ = mesh.Mesh.CreatePointAnchor(localThird).TryGetLocalPointIn(
            frameOrigin,
            frameRotation,
            out Vector3d third);

        var fixedTriangle = new FixedTriangle(
            first,
            second,
            third);
        triangle = new CollisionTriangle(
            fixedTriangle,
            fixedTriangle.Normal,
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
    private static Vector3d OrientNormal(Vector3d normal, Vector3d desiredDirection)
    {
        Vector3d resolved = normal.Normalized;
        return Vector3d.Dot(resolved, desiredDirection) < Fixed64.Zero ? -resolved : resolved;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static Vector3d OrientNormal(
        in FixedPointAnchor source,
        in FixedPointAnchor target,
        Vector3d normal)
    {
        Vector3d resolved = normal.Normalized;
        return source.ProjectNonNegativeOffsetFrom(target, resolved)
                > Fixed64.Zero
            ? -resolved
            : resolved;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static void AddContact(
        CollisionWorkItem pair,
        ContactAnchor anchorOnFirst,
        ContactAnchor anchorOnSecond,
        Fixed64 depth,
        Vector3d normalFirstToSecond,
        bool depthIsClamped)
    {
        pair.Manifold.AddContact(
            anchorOnFirst,
            anchorOnSecond,
            depth,
            normalFirstToSecond,
            depthIsClamped);
    }

}
