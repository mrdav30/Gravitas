//=======================================================================
// ConvexShape.InitialMeshContact.cs
//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp;
using FixedMathSharp.Geometry;
using Gravitas.Colliders;
using System;
using System.Diagnostics;

namespace Gravitas.Queries;

/// <content>Resolves initial mesh contact witnesses from the selected convex source leaf.</content>
internal readonly partial struct ConvexShape
{
    // Both views describe their initial committed poses. Compound reduction
    // must call this while the actual source and target leaves are retained.
    internal bool TryGetInitialMeshContactNormal(
        in ConvexShape source,
        out FixedPointAnchor targetAnchor,
        out Vector3d normal)
    {
        Debug.Assert(_offset == Vector3d.Zero && source._offset == Vector3d.Zero);
        FixedContactAnchors contact;
        if (_kind == ConvexShapeKind.Triangle)
        {
            bool found = TryGetInitialTriangleContact(source, out contact);
            targetAnchor = contact.FirstAnchor;
            normal = contact.Normal;
            if (found)
            {
                PhysicsMesh target = _triangleOwner!.Mesh;
                FixedPointAnchor selectedTargetSupport = GetSupportAnchor(normal);
                FixedPointAnchor selectedSourceSupport = source.GetSupportAnchor(-normal);
                Vector3d planeNormal = GetInitialExitNormal(source,
                    target.GetFaceNormalWorld(_triangleIndex));
                FixedPointAnchor support = source.GetSupportAnchor(-planeNormal);
                var triangle = new FixedTriangle(_triangleA, _triangleB, _triangleC);
                if (WidePointAnchor3d.CompareProjectedOffsets(selectedTargetSupport,
                        selectedSourceSupport, selectedTargetSupport, selectedTargetSupport, normal) > 0
                    && WideTriangleRelations.ContainsProjection(triangle,
                        target.Origin, target.Rotation, support))
                {
                    // A source support projecting onto this face certifies a
                    // face contact. Prefer it to an internal triangulation
                    // seam's oblique escape axis when the source overlaps.
                    normal = planeNormal;
                    targetAnchor = triangle.GetClosestPointAnchor(
                        target.Origin, target.Rotation, support);
                }
            }
            return found;
        }

        var mesh = (LSMeshCollider)_collider!;
        PhysicsMesh hull = mesh.Mesh;
        bool hasContact;
        switch (source._collider)
        {
            case LSCuboidCollider box:
                hasContact = WideOrientedBox.TryGetConvexHullContact(
                    box.Center, box.Rotation, box.OrientedBox.HalfExtents,
                    hull.Origin, hull.Rotation, hull.ScaledLocalVertices,
                    hull.Triangles, hull.ConvexSatEdgeVertexPairs, out contact);
                targetAnchor = contact.SecondAnchor;
                normal = -contact.Normal;
                return hasContact;
            case LSCapsuleCollider capsule:
                hasContact = TryGetHullCapsuleContact(hull, capsule.Center,
                    capsule.Rotation, capsule.AxisLength, capsule.ScaledRadius, out contact);
                break;
            case LSSphereCollider sphere:
                hasContact = TryGetHullCapsuleContact(hull, sphere.Center,
                    sphere.Rotation, Fixed64.Zero, sphere.ScaledRadius, out contact);
                break;
            case LSMeshCollider sourceMesh:
                PhysicsMesh sourceHull = sourceMesh.Mesh;
                hasContact = FixedConvexHullRelations.TryGetContact(
                    hull.Origin, hull.Rotation, hull.ScaledLocalVertices,
                    hull.Triangles, hull.ConvexSatEdgeVertexPairs,
                    sourceHull.Origin, sourceHull.Rotation, sourceHull.ScaledLocalVertices,
                    sourceHull.Triangles, sourceHull.ConvexSatEdgeVertexPairs, out contact);
                break;
            default:
                if (source._kind == ConvexShapeKind.Sphere)
                {
                    hasContact = TryGetHullCapsuleContact(hull, source._center,
                        FixedQuaternion.Identity, Fixed64.Zero, source._radius, out contact);
                    break;
                }
                GetInitialHullFaceExit(source, hull, out targetAnchor, out normal);
                return true;
        }
        targetAnchor = contact.FirstAnchor;
        normal = contact.Normal;
        return hasContact;
    }

    private bool TryGetInitialTriangleContact(in ConvexShape source, out FixedContactAnchors contact)
    {
        var triangle = new FixedTriangle(_triangleA, _triangleB, _triangleC);
        PhysicsMesh targetMesh = _triangleOwner!.Mesh;
        if (source._kind == ConvexShapeKind.Sphere)
            return triangle.TryGetSphereContact(targetMesh.Origin, targetMesh.Rotation,
                source._center, FixedQuaternion.Identity, source._radius, out contact);
        if (source._kind == ConvexShapeKind.CircleSlab)
            return triangle.TryGetCircleSlabContact(targetMesh.Origin, targetMesh.Rotation,
                source._center, Fixed64.Zero, source._halfHeight, source._radius, out contact);

        switch (source._collider)
        {
            case LSSphereCollider sphere:
                return triangle.TryGetSphereContact(targetMesh.Origin, targetMesh.Rotation,
                    sphere.Center, sphere.Rotation, sphere.ScaledRadius, out contact);
            case LSCapsuleCollider capsule:
                Vector3d fallback = GetInitialExitNormal(source,
                    targetMesh.GetFaceNormalWorld(_triangleIndex));
                return triangle.TryGetCenteredCapsuleContact(targetMesh.Origin, targetMesh.Rotation,
                    capsule.Center, capsule.Rotation, capsule.AxisLength, capsule.ScaledRadius,
                    fallback, out contact);
            case LSCuboidCollider box:
                bool found = box.OrientedBox.TryGetTriangleContact(targetMesh.Origin,
                    targetMesh.Rotation, triangle, out FixedContactAnchors reversed);
                contact = new FixedContactAnchors(reversed.SecondAnchor, reversed.FirstAnchor,
                    -reversed.Normal, reversed.Depth, reversed.DepthIsClamped);
                return found;
            case LSCylinderCollider cylinder:
                return triangle.TryGetCenteredFiniteCylinderContact(targetMesh.Origin, targetMesh.Rotation,
                    cylinder.Center, cylinder.Rotation, cylinder.Height, cylinder.ScaledRadius, out contact);
            case LSConeCollider cone:
                return triangle.TryGetCenteredFiniteConeContact(targetMesh.Origin, targetMesh.Rotation,
                    cone.Center, cone.Rotation, cone.Height, cone.ScaledRadius, out contact);
            default:
                var mesh = (LSMeshCollider)source._collider!;
                PhysicsMesh sourceHull = mesh.Mesh;
                ReadOnlySpan<Vector3d> vertices = stackalloc Vector3d[3]
                    { _triangleA, _triangleB, _triangleC };
                ReadOnlySpan<int> triangles = stackalloc int[3] { 0, 1, 2 };
                ReadOnlySpan<int> edges = stackalloc int[6] { 0, 1, 1, 2, 2, 0 };
                return FixedConvexHullRelations.TryGetContact(
                    targetMesh.Origin, targetMesh.Rotation, vertices, triangles, edges,
                    sourceHull.Origin, sourceHull.Rotation, sourceHull.ScaledLocalVertices,
                    sourceHull.Triangles, sourceHull.ConvexSatEdgeVertexPairs, out contact);
        }
    }

    internal Vector3d GetInitialExitNormal(in ConvexShape source, Vector3d axis)
    {
        FixedPointAnchor targetMax = GetSupportAnchor(axis);
        FixedPointAnchor sourceMin = source.GetSupportAnchor(-axis);
        FixedPointAnchor sourceMax = source.GetSupportAnchor(axis);
        FixedPointAnchor targetMin = GetSupportAnchor(-axis);
        return WidePointAnchor3d.CompareProjectedOffsets(
            targetMax, sourceMin, sourceMax, targetMin, axis) <= 0 ? axis : -axis;
    }

    private void GetInitialHullFaceExit(in ConvexShape source, PhysicsMesh hull,
        out FixedPointAnchor targetAnchor, out Vector3d normal)
    {
        // A two-sided triangle MTV does not describe exit from a solid hull.
        // Rank full-shape exit projections on the cached unit face axes. This
        // preserves hull containment and authored ties without claiming a
        // complete curved-shape/hull MTV (edge axes can provide shorter exits).
        // Committed mesh topology has at least one nondegenerate triangle;
        // scale validation preserves nonzero cached face normals.
        normal = GetInitialExitNormal(source, hull.GetFaceNormalWorld(0));
        targetAnchor = GetSupportAnchor(normal);
        FixedPointAnchor selectedSource = source.GetSupportAnchor(-normal);
        for (int index = 1; index < hull.TriangleCount; index++)
        {
            Vector3d axis = hull.GetFaceNormalWorld(index);
            Debug.Assert(axis != Vector3d.Zero);
            axis = GetInitialExitNormal(source, axis);
            FixedPointAnchor targetSupport = GetSupportAnchor(axis);
            FixedPointAnchor sourceSupport = source.GetSupportAnchor(-axis);
            if (WidePointAnchor3d.CompareProjectedOffsets(
                    targetSupport, sourceSupport, targetAnchor, selectedSource,
                    axis, normal) >= 0)
                continue;
            normal = axis;
            targetAnchor = targetSupport;
            selectedSource = sourceSupport;
        }
    }

    private static bool TryGetHullCapsuleContact(PhysicsMesh hull, Vector3d center,
        FixedQuaternion rotation, Fixed64 axisLength, Fixed64 radius, out FixedContactAnchors contact) =>
        FixedConvexHullRelations.TryGetCenteredCapsuleContact(
            hull.Origin, hull.Rotation, hull.ScaledLocalVertices,
            hull.Triangles, hull.ConvexSatEdgeVertexPairs,
            center, rotation, Vector3d.Up, axisLength, radius, out contact);
}
