//=======================================================================
// CollisionDetection.Cone.cs
//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp;
using FixedMathSharp.Geometry;
using Gravitas.Colliders;
using SwiftCollections.Query;
using System.Runtime.CompilerServices;

namespace Gravitas.CollisionHandling;

public static partial class CollisionDetection
{
    private static bool DoConeSphereCheck(CollisionWorkItem pair)
    {
        var cone = (LSConeCollider)pair.ColliderA;
        var sphere = (LSSphereCollider)pair.ColliderB;

        if (!FixedSegment.TryGetClosestCenteredFiniteConeSurfaceAnchor(
                sphere.Center,
                cone.Center,
                cone.Rotation,
                Vector3d.Up,
                cone.Height,
                cone.ScaledRadius,
                Vector3d.Right,
                out FixedPointAnchor coneAnchor,
                out Vector3d outwardNormal,
                out Fixed64 signedDistance))
        {
            return false;
        }
        if (signedDistance > sphere.ScaledRadius)
            return false;

        Vector3d normal = signedDistance < Fixed64.Zero
            ? -outwardNormal
            : outwardNormal;
        ResolvePenetrationDepth(
            sphere.ScaledRadius,
            signedDistance,
            out Fixed64 penetrationDepth,
            out bool depthIsClamped);
        pair.Manifold.SetContact(
            new ContactAnchor(coneAnchor),
            new ContactAnchor(
                FixedSegment.GetCenteredCapsuleSupportAnchor(
                    sphere.Center,
                    sphere.Rotation,
                    Fixed64.Zero,
                    sphere.ScaledRadius,
                    -normal)),
            penetrationDepth,
            normal,
            depthIsClamped);

        return true;
    }

    private static bool DoConeConvexCheck(CollisionWorkItem pair)
    {
        GetConeConvexColliders(pair, out LSConeCollider cone, out LSCollider convex);

        if (!ConvexColliderSupport.Intersects(cone, convex))
            return false;

        Vector3d normalConeToConvex = ResolveNormal(convex.Center - cone.Center);
        normalConeToConvex = normalConeToConvex.Normalized;
        FixedPointAnchor pointOnCone = ConvexColliderSupport.GetSupportAnchor(
            cone,
            normalConeToConvex,
            Vector3d.Zero);
        FixedPointAnchor pointOnConvex = ConvexColliderSupport.GetSupportAnchor(
            convex,
            -normalConeToConvex,
            Vector3d.Zero);
        Fixed64 depth = pointOnCone.ProjectNonNegativeOffsetFrom(
            pointOnConvex,
            normalConeToConvex);

        SetContactInPairOrder(
            pair,
            cone,
            new ContactAnchor(pointOnCone),
            convex,
            new ContactAnchor(pointOnConvex),
            depth,
            normalConeToConvex,
            depthIsClamped: false);
        return true;
    }

    private static bool DoMeshConeCheck(CollisionWorkItem pair)
    {
        var mesh = (LSMeshCollider)pair.ColliderA;
        var cone = (LSConeCollider)pair.ColliderB;

        if (TryFindMeshConeTriangleContact(
                mesh,
                cone,
                pair.Context.CollisionScratch.MeshTriangleCandidatesA,
                out ContactAnchor meshAnchor,
                out ContactAnchor coneAnchor,
                out Vector3d normalMeshToCone,
                out Fixed64 depth,
                out bool depthIsClamped))
        {
            pair.Manifold.SetContact(
                meshAnchor,
                coneAnchor,
                depth,
                normalMeshToCone,
                depthIsClamped);
            return true;
        }

        if (mesh.Mode == MeshColliderMode.Concave || !ConvexColliderSupport.Intersects(mesh, cone))
            return false;

        normalMeshToCone = ResolveNormal(cone.Center - mesh.Center);
        FixedPointAnchor pointOnMesh = ConvexColliderSupport.GetSupportAnchor(
            mesh,
            normalMeshToCone,
            Vector3d.Zero);
        FixedPointAnchor pointOnCone = ConvexColliderSupport.GetSupportAnchor(
            cone,
            -normalMeshToCone,
            Vector3d.Zero);
        depth = pointOnMesh.ProjectNonNegativeOffsetFrom(
            pointOnCone,
            normalMeshToCone);

        pair.Manifold.SetContact(
            new ContactAnchor(pointOnMesh),
            new ContactAnchor(pointOnCone),
            depth,
            normalMeshToCone);
        return true;
    }

    private static bool TryFindMeshConeTriangleContact(
        LSMeshCollider mesh,
        LSConeCollider cone,
        SwiftCollections.SwiftList<int> triangleBuffer,
        out ContactAnchor meshAnchor,
        out ContactAnchor coneAnchor,
        out Vector3d normalMeshToCone,
        out Fixed64 depth,
        out bool depthIsClamped)
    {
        meshAnchor = default;
        coneAnchor = default;
        normalMeshToCone = Vector3d.Zero;
        depth = Fixed64.Zero;
        depthIsClamped = false;

        mesh.GetTrianglesInBounds(new FixedBoundVolume(cone.BoundsMin, cone.BoundsMax), triangleBuffer);
        bool found = false;
        Fixed64 bestDepth = Fixed64.MaxValue;

        for (int i = 0; i < triangleBuffer.Count; i++)
        {
            int triangleIndex = triangleBuffer[i];
            mesh.Mesh.GetLocalTriangleVertices(
                triangleIndex,
                out Vector3d first,
                out Vector3d second,
                out Vector3d third);
            Vector3d windingNormal = mesh.Mesh.GetFaceNormalWorld(triangleIndex);
            var triangle = new FixedTriangle(first, second, third);
            var coneCenterAnchor = new FixedPointAnchor(
                cone.Center,
                FixedQuaternion.Identity,
                Vector3d.Zero);
            if (!coneCenterAnchor.TryGetLocalPointIn(
                    mesh.Mesh.Origin,
                    mesh.Mesh.Rotation,
                    out Vector3d localConeCenter))
            {
                continue;
            }

            Vector3d candidatePointOnMesh =
                triangle.ClosestPoint(localConeCenter);
            FixedPointAnchor candidateMeshAnchor =
                mesh.Mesh.CreatePointAnchor(candidatePointOnMesh);
            Vector3d candidateNormalMeshToCone =
                OrientNormal(
                    candidateMeshAnchor,
                    coneCenterAnchor,
                    windingNormal).Normalized;
            FixedPointAnchor candidateConeAnchor;
            Fixed64 candidateDepth;
            bool candidateDepthIsClamped;
            bool closestMeshPointInsideCone =
                candidateMeshAnchor.TryGetLocalPointIn(
                    cone.Center,
                    cone.Rotation,
                    out Vector3d localPointInCone)
                && FixedSegment.ContainsPointInCenteredFiniteCone(
                    localPointInCone,
                    Vector3d.Zero,
                    Vector3d.Up,
                    cone.Height,
                    cone.ScaledRadius);
            if (closestMeshPointInsideCone)
            {
                if (!FixedSegment.TryGetClosestCenteredFiniteConeSurfaceOffset(
                        localPointInCone,
                        Vector3d.Zero,
                        Vector3d.Up,
                        cone.Height,
                        cone.ScaledRadius,
                        Vector3d.Right,
                        out Vector3d localConePoint,
                        out _,
                        out Fixed64 signedDistance))
                {
                    continue;
                }

                candidateDepthIsClamped = !Fixed64.TrySubtract(
                    Fixed64.Zero,
                    signedDistance,
                    out candidateDepth);
                if (candidateDepthIsClamped)
                    candidateDepth = Fixed64.MaxValue;
                candidateConeAnchor = new FixedPointAnchor(
                    cone.Center,
                    cone.Rotation,
                    localConePoint);
            }
            else if (triangle.TryGetCenteredFiniteConeSupportContact(
                         mesh.Mesh.Origin,
                         mesh.Mesh.Rotation,
                         cone.Center,
                         cone.Rotation,
                         cone.Height,
                         cone.ScaledRadius,
                         -candidateNormalMeshToCone,
                         candidateNormalMeshToCone,
                         out FixedContactAnchors contact))
            {
                candidateMeshAnchor = contact.FirstAnchor;
                candidateConeAnchor = contact.SecondAnchor;
                candidateDepth = contact.Depth;
                candidateDepthIsClamped = contact.DepthIsClamped;
            }
            else
            {
                continue;
            }

            if (found && candidateDepth >= bestDepth)
                continue;

            found = true;
            bestDepth = candidateDepth;
            meshAnchor = new ContactAnchor(candidateMeshAnchor);
            coneAnchor = new ContactAnchor(candidateConeAnchor);
            normalMeshToCone = candidateNormalMeshToCone;
            depth = candidateDepth;
            depthIsClamped = candidateDepthIsClamped;
        }

        return found;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static void GetConeConvexColliders(
        CollisionWorkItem pair,
        out LSConeCollider cone,
        out LSCollider convex)
    {
        if (pair.ColliderA is LSConeCollider coneA && ConvexColliderSupport.IsSupported(pair.ColliderB))
        {
            cone = coneA;
            convex = pair.ColliderB;
            return;
        }

        cone = (LSConeCollider)pair.ColliderB;
        convex = pair.ColliderA;
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

}
