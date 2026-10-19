//=======================================================================
// ContinuousCollisionBounds.cs
//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp;
using FixedMathSharp.Geometry;
using Gravitas.Colliders;
using SwiftCollections.Query;

namespace Gravitas.CollisionHandling;

/// <summary>
/// Certifies empty rotational intervals from midpoint geometry and the outward-rounded
/// bound on both participants' relative travel. Every leaf pair must be separated.
/// </summary>
internal static class ContinuousCollisionBounds
{
    internal static bool AreSeparated(LSCollider source, LSCollider target, Fixed64 motionBound)
    {
        if (ContinuousCollisionMath.AreBoundsSeparatedByMoreThan(source.Bounds, target.Bounds, motionBound))
            return true;

        var sourceCompound = source as LSCompoundCollider;
        var targetCompound = target as LSCompoundCollider;
        for (int i = 0; i < (sourceCompound?.PartCount ?? 1); i++)
        for (int j = 0; j < (targetCompound?.PartCount ?? 1); j++)
        {
            LSCollider first = sourceCompound?.GetPartCollider(i) ?? source;
            LSCollider second = targetCompound?.GetPartCollider(j) ?? target;
            if (ContinuousCollisionMath.AreBoundsSeparatedByMoreThan(first.Bounds, second.Bounds, motionBound))
                continue;
            if (second is LSMeshCollider meshTarget && IsMeshSeparated(meshTarget, first, motionBound))
                continue;
            if (first is LSMeshCollider meshSource && IsMeshSeparated(meshSource, second, motionBound))
                continue;
            return false;
        }
        return true;
    }

    internal static bool AreSeparated(LSCollider2D source, LSCollider2D target, Fixed64 motionBound)
    {
        if (ContinuousCollisionMath.AreBoundsSeparatedByMoreThan(source.Bounds, target.Bounds, motionBound))
            return true;

        var sourceCompound = source as LSCompoundCollider2D;
        var targetCompound = target as LSCompoundCollider2D;
        for (int i = 0; i < (sourceCompound?.PartCount ?? 1); i++)
        for (int j = 0; j < (targetCompound?.PartCount ?? 1); j++)
        {
            LSCollider2D first = sourceCompound?.GetPartCollider(i) ?? source;
            LSCollider2D second = targetCompound?.GetPartCollider(j) ?? target;
            if (!ContinuousCollisionMath.AreBoundsSeparatedByMoreThan(first.Bounds, second.Bounds, motionBound))
                return false;
        }
        return true;
    }

    internal static bool AreSeparated(LSCollider source, LSCollider2D target, Fixed64 motionBound)
    {
        if (ContinuousCollisionMath.AreBoundsSeparatedByMoreThan(source.Bounds, target.MixedBounds3D, motionBound))
            return true;

        var sourceCompound = source as LSCompoundCollider;
        var targetCompound = target as LSCompoundCollider2D;
        for (int i = 0; i < (sourceCompound?.PartCount ?? 1); i++)
        for (int j = 0; j < (targetCompound?.PartCount ?? 1); j++)
        {
            LSCollider first = sourceCompound?.GetPartCollider(i) ?? source;
            LSCollider2D second = targetCompound?.GetPartCollider(j) ?? target;
            if (ContinuousCollisionMath.AreBoundsSeparatedByMoreThan(first.Bounds, second.MixedBounds3D, motionBound))
                continue;
            if (first is LSMeshCollider mesh)
            {
                ColliderCanonicalBounds2D.GetMixedRelativeBounds(second, mesh.Mesh.Origin, mesh.Mesh.Rotation,
                    out FixedBoundBox localBounds, out FixedOrientedBox orientedBounds);
                if (IsMeshSeparated(mesh, localBounds, motionBound, orientedBounds))
                    continue;
            }
            return false;
        }
        return true;
    }

    private static bool IsMeshSeparated(LSMeshCollider mesh, LSCollider other, Fixed64 motionBound)
    {
        // Admitted local capsule extents fit; retain clipping only in the final mesh-relative frame.
        return IsMeshSeparated(mesh, ColliderCanonicalBounds.GetRelativeBounds(other,
            mesh.Mesh.Origin, mesh.Mesh.Rotation), motionBound, (other as LSCuboidCollider)?.OrientedBox);
    }

    private static bool IsMeshSeparated(LSMeshCollider mesh, FixedBoundBox localBounds, Fixed64 motionBound,
        FixedOrientedBox? orientedBox = null)
    {
        // An empty boundary query cannot certify the empty interior of a solid hull.
        if (mesh.Mode == MeshColliderMode.Convex && mesh.IsClosedSurface)
            return false;

        if (!mesh.Mesh.GeometryFrame.IsRigid)
            return IsScaledMeshSeparated(mesh, localBounds, motionBound, orientedBox);

        // The combined source/target motion ball fits in this midpoint-frame box.
        // Only mesh-local endpoints are clipped; all stored triangles lie in that domain.
        Vector3d expansion = Vector3d.One * motionBound;
        var candidates = mesh.Context.CollisionScratch.MeshTriangleCandidatesA;
        FixedBoundBox expanded = FixedBoundBox.FromMinMax(localBounds.Min - expansion, localBounds.Max + expansion);
        mesh.Mesh.GetTrianglesInGeometryBounds(new FixedBoundVolume(expanded.Min, expanded.Max), candidates);
        if (candidates.Count == 0)
            return true;

        // A triangle's box also contains empty corners. Use the existing exact
        // finite-triangle SAT to certify them, without constructing world witnesses.
        FixedOrientedBox box;
        Vector3d triangleOrigin;
        FixedQuaternion triangleRotation;
        if (orientedBox is FixedOrientedBox rigidBox)
        {
            // Preserve the shape enclosure's orientation: its AABB can cover empty corners
            // throughout rotation. The same motion ball fits along each box axis.
            if (!Vector3d.TryAdd(rigidBox.HalfExtents, expansion, out Vector3d scope))
                return false;
            box = new FixedOrientedBox(rigidBox.Center, rigidBox.Orientation, scope);
            triangleOrigin = mesh.Mesh.Origin;
            triangleRotation = mesh.Mesh.Rotation;
        }
        else
        {
            Vector3d center = expanded.Center;
            bool scopeResolved = Vector3d.TrySubtract(center, expanded.Min, out Vector3d lowerScope)
                & Vector3d.TrySubtract(expanded.Max, center, out Vector3d upperScope);
            if (!scopeResolved)
                return false;
            Vector3d scope = new(FixedMath.Max(lowerScope.X, upperScope.X),
                FixedMath.Max(lowerScope.Y, upperScope.Y), FixedMath.Max(lowerScope.Z, upperScope.Z));
            if (scope.X == Fixed64.Zero || scope.Y == Fixed64.Zero || scope.Z == Fixed64.Zero)
                return false;
            box = new FixedOrientedBox(center, FixedQuaternion.Identity, scope);
            triangleOrigin = Vector3d.Zero;
            triangleRotation = FixedQuaternion.Identity;
        }
        for (int i = 0; i < candidates.Count; i++)
        {
            mesh.Mesh.GetGeometryTriangleVertices(candidates[i], out Vector3d first, out Vector3d second, out Vector3d third);
            var triangle = new FixedTriangle(first, second, third);
            // Degeneracy is a distinct false result, so it cannot prove separation.
            if (triangle.IsDegenerate || box.TryGetTriangleContact(triangleOrigin, triangleRotation, triangle, out _))
                return false;
        }
        return true;
    }
    private static bool IsScaledMeshSeparated(LSMeshCollider collider, FixedBoundBox physicalBounds,
        Fixed64 motionBound, FixedOrientedBox? orientedBox)
    {
        PhysicsMesh mesh = collider.Mesh;
        Vector3d expansion = Vector3d.One * motionBound;
        FixedBoundBox expanded = FixedBoundBox.FromMinMax(physicalBounds.Min - expansion, physicalBounds.Max + expansion);
        // Motion is a physical sphere. Expand before converting to the authored
        // chart, where nonuniform scale would otherwise distort that sphere.
        var physicalFrame = ScaledRigidFrame3d.Rigid(mesh.Origin, mesh.Rotation);
        bool transformed = physicalFrame.TryGetBoundsIn(expanded.Min, expanded.Max, mesh.GeometryFrame,
            out FixedBoundBox candidatesBounds);
        // Collider scale admission guarantees an invertible target mapping.
        System.Diagnostics.Debug.Assert(transformed);
        var candidates = collider.Context.CollisionScratch.MeshTriangleCandidatesA;
        mesh.GetTrianglesInGeometryBounds(new FixedBoundVolume(candidatesBounds.Min, candidatesBounds.Max), candidates);
        if (candidates.Count == 0) return true;
        ScaledRigidFrame3d boxFrame;
        Vector3d scope;
        if (orientedBox is FixedOrientedBox rigid)
        {
            if (!Vector3d.TryAdd(rigid.HalfExtents, expansion, out scope)) return false;
            boxFrame = ScaledRigidFrame3d.Rigid(rigid.Center, rigid.Orientation);
        }
        else
        {
            Vector3d center = expanded.Center;
            if (!Vector3d.TrySubtract(center, expanded.Min, out Vector3d lower)
                || !Vector3d.TrySubtract(expanded.Max, center, out Vector3d upper)) return false;
            scope = new Vector3d(FixedMath.Max(lower.X, upper.X), FixedMath.Max(lower.Y, upper.Y),
                FixedMath.Max(lower.Z, upper.Z));
            boxFrame = new ScaledRigidFrame3d(mesh.Origin, mesh.Rotation, center, Vector3d.One,
                mesh.Rotation, Vector3d.Zero, Vector3d.One);
        }
        if (scope.X == Fixed64.Zero || scope.Y == Fixed64.Zero || scope.Z == Fixed64.Zero) return false;
        System.Span<Vector3d> boxVertices = stackalloc Vector3d[8];
        WideOrientedBox.GetBoxHullVertices(scope, boxVertices);
        System.Span<Vector3d> points = stackalloc Vector3d[3];
        System.ReadOnlySpan<int> face = stackalloc int[3] { 0, 1, 2 };
        System.ReadOnlySpan<int> edges = stackalloc int[6] { 0, 1, 1, 2, 2, 0 };
        for (int index = 0; index < candidates.Count; index++)
        {
            mesh.GetGeometryTriangleVertices(candidates[index], out points[0], out points[1], out points[2]);
            // Authored mesh admission rejects degenerate triangles, and the
            // positive-scale affine mapping preserves their exact nonzero area.
            if (WideOrientedBox.TryGetConvexHullPenetration(boxFrame, boxVertices,
                    WideOrientedBox.BoxHullTriangles, WideOrientedBox.BoxHullEdges, mesh.GeometryFrame,
                    points, face, edges, out _)) return false;
        }
        return true;
    }

}
