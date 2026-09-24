// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.

using FixedMathSharp;
using FixedMathSharp.Geometry;
using Gravitas.Colliders;
using System;

namespace Gravitas.CollisionHandling;

/// <content>Classifies posture clearance before solver contact rounding.</content>
public static partial class CollisionDetection
{
    // The transaction admits sphere, capsule and finite-cylinder candidates.
    // Pair policy, supported collision types, root bounds and root ordering are
    // checked by SolidBody. Compound parts are geometry only and must retain
    // their parent's blocker identity.
    internal static bool DoesPostureCandidatePenetrate(LSCollider candidate, LSCollider other)
    {
        if (other is LSCompoundCollider compound)
        {
            for (int i = 0; i < compound.PartCount; i++)
            {
                LSCollider part = compound.GetPartCollider(i);
                if (candidate.Bounds.Intersects(part.Bounds)
                    && DoesPostureLeafPenetrate(candidate, part))
                    return true;
            }
            return false;
        }
        return DoesPostureLeafPenetrate(candidate, other);
    }

    private static bool DoesPostureLeafPenetrate(LSCollider candidate, LSCollider other)
    {
        if (candidate is LSCylinderCollider cylinder)
            return DoesPostureCylinderPenetrate(cylinder, other);
        Fixed64 length = candidate is LSCapsuleCollider capsule ? capsule.AxisLength : Fixed64.Zero;
        return DoesPostureCapsulePenetrate(candidate, length, other);
    }

    private static bool DoesPostureCapsulePenetrate(LSCollider candidate, Fixed64 length, LSCollider other)
    {
        switch (other)
        {
            case LSSphereCollider sphere:
                return FixedSegment.DoCenteredCapsulesOverlapStrict(
                    candidate.Center, candidate.Rotation, Vector3d.Up, length, candidate.ScaledRadius,
                    sphere.Center, sphere.Rotation, Vector3d.Up, Fixed64.Zero, sphere.ScaledRadius);
            case LSCapsuleCollider capsule:
                return FixedSegment.DoCenteredCapsulesOverlapStrict(
                    candidate.Center, candidate.Rotation, Vector3d.Up, length, candidate.ScaledRadius,
                    capsule.Center, capsule.Rotation, Vector3d.Up, capsule.AxisLength, capsule.ScaledRadius);
            case LSCuboidCollider box:
                return WideOrientedBox.DoesCenteredCapsulePenetrate(
                    box.Center, box.Rotation, box.OrientedBox.HalfExtents,
                    candidate.Center, candidate.Rotation, Vector3d.Up, length, candidate.ScaledRadius);
            case LSCylinderCollider cylinder:
                return length == Fixed64.Zero
                    ? WideFiniteAxisIntersection.DoesCenteredFiniteCylinderPenetrateSphere(
                        cylinder.Center, cylinder.Rotation, cylinder.Height, cylinder.ScaledRadius,
                        candidate.Center, candidate.ScaledRadius)
                    : WideFiniteAxisIntersection.DoesCenteredFiniteCylinderPenetrateCapsule(
                        cylinder.Center, cylinder.Rotation, cylinder.Height, cylinder.ScaledRadius,
                        candidate.Center, candidate.Rotation, Vector3d.Up, length, candidate.ScaledRadius);
            case LSConeCollider cone:
                return length == Fixed64.Zero
                    ? WideFiniteAxisIntersection.DoesCenteredFiniteConePenetrateSphere(
                        cone.Center, cone.Rotation, cone.Height, cone.ScaledRadius,
                        candidate.Center, candidate.ScaledRadius)
                    : WideFiniteAxisIntersection.DoesCenteredFiniteConePenetrateCapsule(
                        cone.Center, cone.Rotation, cone.Height, cone.ScaledRadius,
                        candidate.Center, candidate.Rotation, Vector3d.Up, length, candidate.ScaledRadius);
            default:
                return DoesPostureCapsulePenetrateMesh(candidate, length, (LSMeshCollider)other);
        }
    }

    private static bool DoesPostureCylinderPenetrate(LSCylinderCollider cylinder, LSCollider other)
    {
        switch (other)
        {
            case LSSphereCollider sphere:
                return WideFiniteAxisIntersection.DoesCenteredFiniteCylinderPenetrateSphere(
                    cylinder.Center, cylinder.Rotation, cylinder.Height, cylinder.ScaledRadius,
                    sphere.Center, sphere.ScaledRadius);
            case LSCapsuleCollider capsule:
                return WideFiniteAxisIntersection.DoesCenteredFiniteCylinderPenetrateCapsule(
                    cylinder.Center, cylinder.Rotation, cylinder.Height, cylinder.ScaledRadius,
                    capsule.Center, capsule.Rotation, Vector3d.Up, capsule.AxisLength, capsule.ScaledRadius);
            case LSCuboidCollider box:
                return WideOrientedBox.DoesCenteredCylinderPenetrateBox(
                    cylinder.Center, cylinder.Rotation, cylinder.Height, cylinder.ScaledRadius,
                    box.Center, box.Rotation, box.OrientedBox.HalfExtents);
            case LSCylinderCollider otherCylinder:
                return WideFiniteAxisIntersection.DoesCenteredFiniteCylindersPenetrate(
                    cylinder.Center, cylinder.Rotation, cylinder.Height, cylinder.ScaledRadius,
                    otherCylinder.Center, otherCylinder.Rotation, otherCylinder.Height, otherCylinder.ScaledRadius);
            case LSConeCollider cone:
                return WideFiniteAxisIntersection.DoesCenteredFiniteCylinderPenetrateCone(
                    cylinder.Center, cylinder.Rotation, cylinder.Height, cylinder.ScaledRadius,
                    cone.Center, cone.Rotation, cone.Height, cone.ScaledRadius);
            default:
                return DoesPostureCylinderPenetrateMesh(cylinder, (LSMeshCollider)other);
        }
    }

    private static bool DoesPostureCapsulePenetrateMesh(LSCollider candidate, Fixed64 length, LSMeshCollider collider)
    {
        PhysicsMesh mesh = collider.Mesh;
        if (collider.Mode == MeshColliderMode.Convex && collider.IsClosedSurface)
        {
            return WideOrientedBox.DoesConvexHullCenteredCapsuleStrictlyOverlap(
                mesh.Origin, mesh.Rotation, mesh.ScaledLocalVertices, mesh.Triangles,
                mesh.ConvexSatEdgeVertexPairs, candidate.Center, candidate.Rotation,
                Vector3d.Up, length, candidate.ScaledRadius);
        }

        // Open convex meshes and concave meshes are surfaces, not filled hulls.
        // Pass authored local vertices plus the exact rigid frame: transforming
        // triangle points into rounded world coordinates changes boundary truth.
        ReadOnlySpan<int> indices = mesh.Triangles;
        ReadOnlySpan<Vector3d> points = mesh.ScaledLocalVertices;
        Span<Vector3d> triangle = stackalloc Vector3d[3];
        ReadOnlySpan<int> face = stackalloc int[] { 0, 1, 2 };
        ReadOnlySpan<int> edges = stackalloc int[] { 0, 1, 1, 2, 2, 0 };
        for (int i = 0; i < indices.Length; i += 3)
        {
            triangle[0] = points[indices[i]];
            triangle[1] = points[indices[i + 1]];
            triangle[2] = points[indices[i + 2]];
            if (WideOrientedBox.DoesConvexHullCenteredCapsuleStrictlyOverlap(
                mesh.Origin, mesh.Rotation, triangle, face, edges,
                candidate.Center, candidate.Rotation, Vector3d.Up, length, candidate.ScaledRadius))
                return true;
        }
        return false;
    }

    private static bool DoesPostureCylinderPenetrateMesh(LSCylinderCollider cylinder, LSMeshCollider collider)
    {
        PhysicsMesh mesh = collider.Mesh;
        if (collider.Mode == MeshColliderMode.Convex)
        {
            return WideOrientedBox.DoesCenteredCylinderPenetrateConvexHull(
                cylinder.Center, cylinder.Rotation, cylinder.Height, cylinder.ScaledRadius,
                mesh.Origin, mesh.Rotation, mesh.ScaledLocalVertices, mesh.Triangles, collider.IsClosedSurface);
        }
        ReadOnlySpan<int> indices = mesh.Triangles;
        ReadOnlySpan<Vector3d> points = mesh.ScaledLocalVertices;
        for (int i = 0; i < indices.Length; i += 3)
        {
            var triangle = new FixedTriangle(points[indices[i]], points[indices[i + 1]], points[indices[i + 2]]);
            if (WideOrientedBox.DoesCenteredCylinderPenetrateTriangle(
                cylinder.Center, cylinder.Rotation, cylinder.Height, cylinder.ScaledRadius,
                triangle, mesh.Origin, mesh.Rotation))
                return true;
        }
        return false;
    }
}
