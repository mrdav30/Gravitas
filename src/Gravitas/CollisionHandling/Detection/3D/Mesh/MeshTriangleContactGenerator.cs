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
        bool reverseContact = meshA.Id > meshB.Id;
        LSMeshCollider firstMesh = reverseContact ? meshB : meshA;
        LSMeshCollider secondMesh = reverseContact ? meshA : meshB;
        firstMesh.GetTrianglesInBounds(
            CreateBounds(secondMesh.BoundsMin, secondMesh.BoundsMax),
            triangleBufferA);
        for (int i = 0; i < triangleBufferA.Count; i++)
        {
            int triangleA = triangleBufferA[i];
            firstMesh.Mesh.GetLocalTriangleVertices(
                triangleA,
                out Vector3d firstVertex,
                out Vector3d secondVertex,
                out Vector3d thirdVertex);
            var firstTriangle =
                new FixedTriangle(firstVertex, secondVertex, thirdVertex);
            GetTriangleBoundsInFrame(
                firstTriangle.Bounds,
                firstMesh.Mesh.Origin,
                firstMesh.Mesh.Rotation,
                secondMesh.Mesh.Origin,
                secondMesh.Mesh.Rotation,
                out FixedBoundVolume firstInSecondFrameBounds);
            secondMesh.Mesh.GetTrianglesInLocalBounds(
                firstInSecondFrameBounds,
                triangleBufferB);

            for (int j = 0; j < triangleBufferB.Count; j++)
            {
                int triangleB = triangleBufferB[j];
                secondMesh.Mesh.GetLocalTriangleVertices(
                    triangleB,
                    out Vector3d secondFirstVertex,
                    out Vector3d secondSecondVertex,
                    out Vector3d secondThirdVertex);
                var secondTriangle = new FixedTriangle(
                    secondFirstVertex,
                    secondSecondVertex,
                    secondThirdVertex);
                if (!firstTriangle.TryGetContact(
                        firstMesh.Mesh.Origin,
                        firstMesh.Mesh.Rotation,
                        secondMesh.Mesh.Origin,
                        secondMesh.Mesh.Rotation,
                        secondTriangle,
                        out FixedContactAnchors contact))
                {
                    continue;
                }

                if (reverseContact)
                {
                    AddContact(
                        pair,
                        new ContactAnchor(contact.SecondAnchor),
                        new ContactAnchor(contact.FirstAnchor),
                        contact.Depth,
                        -contact.Normal,
                        contact.DepthIsClamped);
                }
                else
                {
                    AddContact(
                        pair,
                        new ContactAnchor(contact.FirstAnchor),
                        new ContactAnchor(contact.SecondAnchor),
                        contact.Depth,
                        contact.Normal,
                        contact.DepthIsClamped);
                }
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

    private static void GetTriangleBoundsInFrame(
        FixedBoundBox triangleBounds,
        Vector3d sourceOrigin,
        FixedQuaternion sourceRotation,
        Vector3d frameOrigin,
        FixedQuaternion frameRotation,
        out FixedBoundVolume bounds)
    {
        FixedBoundBox reframedBounds =
            FixedBoundBox.FromRelativeRotatedBoundsClippedToDomain(
                sourceOrigin,
                sourceRotation,
                triangleBounds.Min,
                triangleBounds.Max,
                frameOrigin,
                frameRotation);
        bounds = CreateBounds(reframedBounds.Min, reframedBounds.Max);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static FixedBoundVolume CreateBounds(Vector3d min, Vector3d max) =>
        new(min, max);

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
