//=======================================================================
// CollisionDetection.Cone.cs
//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp;
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

        Vector3d conePoint = cone.ClosestPointOnSurface(sphere.Center);
        Vector3d delta = sphere.Center - conePoint;
        if (delta.MagnitudeSquared > sphere.ScaledRadiusSqr)
            return false;

        Fixed64 distance = delta.Magnitude;
        Vector3d normal = ResolveNormal(delta, sphere.Center - cone.Center);
        Vector3d spherePoint = sphere.Center - normal * sphere.ScaledRadius;
        pair.Manifold.SetContact(
            conePoint,
            spherePoint,
            sphere.ScaledRadius - distance,
            normal);

        return true;
    }

    private static bool DoConeConvexCheck(CollisionWorkItem pair)
    {
        GetConeConvexColliders(pair, out LSConeCollider cone, out LSCollider convex);

        if (!ConvexColliderSupport.Intersects(cone, convex))
            return false;

        Vector3d normalConeToConvex = OrientNormal(convex.Center - cone.Center, convex.Center - cone.Center);
        normalConeToConvex = normalConeToConvex.Normalized;
        Vector3d pointOnCone = ConvexColliderSupport.Support(cone, normalConeToConvex);
        Vector3d pointOnConvex = ConvexColliderSupport.Support(convex, -normalConeToConvex);
        Fixed64 depth = Vector3d.Dot(pointOnCone - pointOnConvex, normalConeToConvex);
        if (depth < Fixed64.Zero)
            depth = Fixed64.Zero;

        SetContactInPairOrder(
            pair,
            cone,
            pointOnCone,
            convex,
            pointOnConvex,
            depth,
            normalConeToConvex);
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
                out Vector3d pointOnMesh,
                out Vector3d pointOnCone,
                out Vector3d normalMeshToCone,
                out Fixed64 depth))
        {
            pair.Manifold.SetContact(pointOnMesh, pointOnCone, depth, normalMeshToCone);
            return true;
        }

        if (mesh.Mode == MeshColliderMode.Concave || !ConvexColliderSupport.Intersects(mesh, cone))
            return false;

        normalMeshToCone = OrientNormal(cone.Center - mesh.Center, cone.Center - mesh.Center).Normalized;
        pointOnMesh = ConvexColliderSupport.Support(mesh, normalMeshToCone);
        pointOnCone = ConvexColliderSupport.Support(cone, -normalMeshToCone);
        depth = Vector3d.Dot(pointOnMesh - pointOnCone, normalMeshToCone);
        if (depth < Fixed64.Zero)
            depth = Fixed64.Zero;

        pair.Manifold.SetContact(pointOnMesh, pointOnCone, depth, normalMeshToCone);
        return true;
    }

    private static bool TryFindMeshConeTriangleContact(
        LSMeshCollider mesh,
        LSConeCollider cone,
        SwiftCollections.SwiftList<int> triangleBuffer,
        out Vector3d pointOnMesh,
        out Vector3d pointOnCone,
        out Vector3d normalMeshToCone,
        out Fixed64 depth)
    {
        pointOnMesh = Vector3d.Zero;
        pointOnCone = Vector3d.Zero;
        normalMeshToCone = Vector3d.Zero;
        depth = Fixed64.Zero;

        mesh.GetTrianglesInBounds(new FixedBoundVolume(cone.BoundsMin, cone.BoundsMax), triangleBuffer);
        bool found = false;
        Fixed64 bestDepth = Fixed64.MaxValue;

        for (int i = 0; i < triangleBuffer.Count; i++)
        {
            int triangleIndex = triangleBuffer[i];
            mesh.Mesh.GetTriangleVertices(triangleIndex, out Vector3d first, out Vector3d second, out Vector3d third);
            Vector3d windingNormal = mesh.Mesh.GetFaceNormalWorld(triangleIndex);
            // Triangle containment is winding-sensitive, while the contact normal may face either side.
            Vector3d candidatePointOnMesh = MeshUtils.ClosestPointOnTriangle(
                first,
                second,
                third,
                windingNormal,
                cone.Center);
            Vector3d candidateNormalMeshToCone = OrientNormal(
                windingNormal,
                cone.Center - candidatePointOnMesh).Normalized;
            Vector3d candidatePointOnCone = Vector3d.Zero;
            bool closestMeshPointInsideCone = cone.ContainsWorldPoint(candidatePointOnMesh, Fixed64.Epsilon);
            if (!closestMeshPointInsideCone
                && !TryFindConeTriangleSupportContact(
                    cone,
                    first,
                    second,
                    third,
                    windingNormal,
                    candidateNormalMeshToCone,
                    out candidatePointOnMesh,
                    out candidatePointOnCone))
            {
                continue;
            }

            if (closestMeshPointInsideCone)
                candidatePointOnCone = cone.ClosestPointOnSurface(candidatePointOnMesh);
            else
                candidatePointOnMesh = MeshUtils.ClosestPointOnTriangle(
                    first,
                    second,
                    third,
                    windingNormal,
                    candidatePointOnCone);

            Fixed64 candidateDepth = Vector3d.Distance(candidatePointOnMesh, candidatePointOnCone);
            if (found && candidateDepth >= bestDepth)
                continue;

            found = true;
            bestDepth = candidateDepth;
            pointOnMesh = candidatePointOnMesh;
            pointOnCone = candidatePointOnCone;
            normalMeshToCone = candidateNormalMeshToCone;
            depth = candidateDepth;
        }

        return found;
    }

    private static bool TryFindConeTriangleSupportContact(
        LSConeCollider cone,
        Vector3d first,
        Vector3d second,
        Vector3d third,
        Vector3d windingNormal,
        Vector3d normalMeshToCone,
        out Vector3d pointOnMesh,
        out Vector3d pointOnCone)
    {
        pointOnMesh = Vector3d.Zero;
        pointOnCone = ConvexColliderSupport.Support(cone, -normalMeshToCone);
        Fixed64 signedDistance = Vector3d.Dot(pointOnCone - first, normalMeshToCone);
        if (signedDistance > Fixed64.Epsilon)
            return false;

        pointOnMesh = pointOnCone - normalMeshToCone * signedDistance;
        return MeshUtils.IsPointInTrianglePlane(first, second, third, windingNormal, pointOnMesh);
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

}
