//=======================================================================
// MeshSurfaceBoundaryTopology.cs
//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================
using FixedMathSharp;
using FixedMathSharp.Geometry;
using Gravitas.Colliders;
using System;

namespace Gravitas.CollisionHandling;

/// <summary>Exact inward sectors of a trusted planar surface's genuine perimeter.</summary>
internal static class MeshSurfaceBoundaryTopology
{
    internal static int FindEdgeRegion(PhysicsMesh mesh, int owner, int first, int second,
        MeshSurfaceConnectivity connectivity, in ConePlaneRayFrame frame) =>
        FindEdgeRegion(mesh, owner, first, second, connectivity, new ConeSection(frame));

    internal static int FindEdgeRegion(PhysicsMesh mesh, int owner, int first, int second,
        MeshSurfaceConnectivity connectivity, in CapsuleFiniteSectionFrame frame) =>
        FindEdgeRegion(mesh, owner, first, second, connectivity, new CapsuleSection(frame));

    internal static int FindEdgeRegion(PhysicsMesh mesh, int owner, int first, int second,
        MeshSurfaceConnectivity connectivity, in FiniteSlabSectionFrame frame, LSCollider2D? embedded) =>
        FindEdgeRegion(mesh, owner, first, second, connectivity, new SlabSection(frame, embedded));

    internal static int FindEdgeRegion(PhysicsMesh mesh, int owner, int first, int second,
        MeshSurfaceConnectivity connectivity, in FiniteSlabSectionFrame frame, ReadOnlyMemory<Vector2d> offsets) =>
        FindEdgeRegion(mesh, owner, first, second, connectivity, new SlabSection(frame, null, offsets));

    internal static int FindSupportEdgeRegion(PhysicsMesh mesh, int owner, int first, int second,
        MeshSurfaceConnectivity connectivity, MeshSlabFaceRegions faces, LSCollider2D? embedded) =>
        FindEdgeRegion(mesh, owner, first, second, connectivity, new SlabSupport(faces, embedded));

    internal static int FindSupportEdgeRegion(PhysicsMesh mesh, int owner, int first, int second,
        MeshSurfaceConnectivity connectivity, MeshHullSurfaceContacts faces) =>
        FindEdgeRegion(mesh, owner, first, second, connectivity, new HullSupport(faces));

    private static int FindEdgeRegion<TSection>(PhysicsMesh mesh, int owner, int first, int second,
        MeshSurfaceConnectivity connectivity, in TSection frame) where TSection : struct, ISection
    {
        ReadOnlySpan<Vector3d> vertices = mesh.ScaledLocalVertices;
        // Reduced edges can span many authored subedges. Their first corner's
        // triangle need not touch the finite shape; locate an admitted piece.
        ReadOnlySpan<int> perimeter = mesh.GetManifoldSurfaceBoundaryVertexPairs(owner);
        if (perimeter.IsEmpty) return connectivity.GetRegionOrdinal(owner);
        mesh.GetLocalTriangleVertices(owner, out Vector3d a, out Vector3d b, out Vector3d c);
        new FixedTriangle(a, b, c).GetExactNormal(out Signed192 nx, out Signed192 ny, out _, out _);
        int axis = !nx.IsZero ? 0 : !ny.IsZero ? 1 : 2;
        Vector2d p = Project(vertices[first], axis), q = Project(vertices[second], axis);
        bool useX = p.X != q.X;
        Fixed64 low = FixedMath.Min(useX ? p.X : p.Y, useX ? q.X : q.Y);
        Fixed64 high = FixedMath.Max(useX ? p.X : p.Y, useX ? q.X : q.Y);
        for (int edge = 0; edge < perimeter.Length; edge += 2)
        {
            int start = perimeter[edge], end = perimeter[edge + 1];
            Vector2d s = Project(vertices[start], axis), t = Project(vertices[end], axis);
            if (Vector2d.OrientationSign(p, q, s) != 0 || Vector2d.OrientationSign(p, q, t) != 0
                || (useX ? s.X : s.Y) < low || (useX ? s.X : s.Y) > high
                || !frame.IntersectsSegment(new FixedSegment(vertices[start], vertices[end]))) continue;
            // A trusted exposed subedge with its start in this canonical span
            // cannot leave it: reduction merges monotone collinear chains,
            // while the embedding certificate rejects overlap/backtracking.
            foreach (int incident in mesh.GetWeldedVertexTriangleIndices(start))
            {
                if (mesh.GetWeldedTriangleVertexIndices(incident).IndexOf(end) < 0) continue;
                int region = connectivity.GetRegionOrdinal(incident);
                if (region >= 0) return region;
            }
        }
        return -1;
    }

    internal static int GetIncidentFan(PhysicsMesh mesh, int owner, int origin, int opposite,
        bool interior, Span<WideAxis3> fan)
    {
        ReadOnlySpan<Vector3d> vertices = mesh.ScaledLocalVertices;
        Vector3d p = vertices[origin], q = vertices[opposite];
        mesh.GetLocalTriangleVertices(owner, out Vector3d a, out Vector3d b, out Vector3d c);
        new FixedTriangle(a, b, c).GetExactNormal(out Signed192 nx, out Signed192 ny, out _, out _);
        int axis = !nx.IsZero ? 0 : !ny.IsZero ? 1 : 2;
        Vector2d projectedOrigin = Project(p, axis), projectedOpposite = Project(q, axis);
        fan[0] = Ray(p, q);
        if (interior)
        {
            fan[1] = Negate(fan[0]);
            int inward = -1;
            bool useX = projectedOpposite.X != projectedOrigin.X;
            int direction = useX ? projectedOpposite.X.CompareTo(projectedOrigin.X)
                : projectedOpposite.Y.CompareTo(projectedOrigin.Y);
            foreach (int incident in mesh.GetWeldedVertexTriangleIndices(origin))
            {
                if (mesh.GetManifoldSurfaceOwner(incident) != owner) continue;
                ReadOnlySpan<int> triangle = mesh.GetWeldedTriangleVertexIndices(incident);
                bool onEdge = false;
                foreach (int vertex in triangle)
                {
                    Vector2d v = Project(vertices[vertex], axis);
                    // Only the local ray direction matters. A positive
                    // collinear incident edge supplies the same boundary
                    // half-neighborhood after ±edge enforces tangency.
                    if (v != projectedOrigin && Vector2d.OrientationSign(projectedOrigin, projectedOpposite, v) == 0
                        && (useX ? v.X.CompareTo(projectedOrigin.X) : v.Y.CompareTo(projectedOrigin.Y)) == direction) onEdge = true;
                }
                if (!onEdge) continue;
                foreach (int vertex in triangle)
                    if (Vector2d.OrientationSign(projectedOrigin, projectedOpposite, Project(vertices[vertex], axis)) != 0)
                        inward = vertex;
                break;
            }
            System.Diagnostics.Debug.Assert(inward >= 0);
            fan[2] = Ray(p, vertices[inward]);
            return 3;
        }
        int neighbor = -1;
        ReadOnlySpan<int> boundary = mesh.GetCanonicalSurfaceBoundaryVertexPairs(owner);
        for (int edge = 0; edge < boundary.Length; edge += 2)
        {
            int first = boundary[edge], second = boundary[edge + 1];
            if (first == origin && second != opposite) neighbor = second;
            else if (second == origin && first != opposite) neighbor = first;
        }
        System.Diagnostics.Debug.Assert(neighbor >= 0);
        fan[1] = Ray(p, vertices[neighbor]);
        Vector2d projectedNeighbor = Project(vertices[neighbor], axis);
        int orientation = Vector2d.OrientationSign(projectedOrigin, projectedOpposite, projectedNeighbor);
        // A certified simple planar boundary has exactly two noncollinear
        // corner rays. Check the actual incident sectors before choosing their
        // minor positive hull: a reflex fan has only plane-normal support.
        foreach (int incident in mesh.GetWeldedVertexTriangleIndices(origin))
        {
            if (mesh.GetManifoldSurfaceOwner(incident) != owner) continue;
            foreach (int vertex in mesh.GetWeldedTriangleVertexIndices(incident))
            {
                Vector2d v = Project(vertices[vertex], axis);
                if (orientation * Vector2d.OrientationSign(projectedOrigin, projectedOpposite, v) >= 0
                    && orientation * Vector2d.OrientationSign(projectedOrigin, v, projectedNeighbor) >= 0) continue;
                fan[2] = Negate(fan[0]); fan[3] = Negate(fan[1]);
                return 4;
            }
        }
        return 2;
    }

    private static Vector2d Project(Vector3d value, int axis) => axis == 0
        ? new Vector2d(value.Y, value.Z) : axis == 1 ? new Vector2d(value.X, value.Z) : new Vector2d(value.X, value.Y);

    private static WideAxis3 Negate(WideAxis3 value) => new(
        WideArithmetic.Negate(value.X), WideArithmetic.Negate(value.Y), WideArithmetic.Negate(value.Z));

    internal static WideAxis3 Ray(Vector3d first, Vector3d second) => new(
        Signed320.ExtendValue(WideArithmetic.SubtractSigned192(Signed192.Raw(second.X), Signed192.Raw(first.X))),
        Signed320.ExtendValue(WideArithmetic.SubtractSigned192(Signed192.Raw(second.Y), Signed192.Raw(first.Y))),
        Signed320.ExtendValue(WideArithmetic.SubtractSigned192(Signed192.Raw(second.Z), Signed192.Raw(first.Z))));

    private interface ISection { bool IntersectsSegment(FixedSegment segment); }

    private readonly struct ConeSection : ISection
    {
        private readonly ConePlaneRayFrame _frame;
        internal ConeSection(in ConePlaneRayFrame frame) => _frame = frame;
        public bool IntersectsSegment(FixedSegment segment) => _frame.IntersectsSegment(segment);
    }

    private readonly struct CapsuleSection : ISection
    {
        private readonly CapsuleFiniteSectionFrame _frame;
        internal CapsuleSection(in CapsuleFiniteSectionFrame frame) => _frame = frame;
        public bool IntersectsSegment(FixedSegment segment) => _frame.IntersectsSegment(segment);
    }

    private readonly struct SlabSection : ISection
    {
        private readonly FiniteSlabSectionFrame frame;
        private readonly LSCollider2D? embedded;
        private readonly ReadOnlyMemory<Vector2d> offsets;
        internal SlabSection(in FiniteSlabSectionFrame frame, LSCollider2D? embedded, ReadOnlyMemory<Vector2d> offsets = default)
        { this.frame = frame; this.embedded = embedded; this.offsets = offsets; }
        public bool IntersectsSegment(FixedSegment segment)
        {
            Span<Vector2d> box = stackalloc Vector2d[4];
            return frame.IntersectsSegment(segment, offsets.IsEmpty ? CollisionDetectionMixed.GetConvexPrismOffsets(embedded, box) : offsets.Span);
        }
    }

    private readonly struct SlabSupport : ISection
    {
        private readonly MeshSlabFaceRegions faces;
        private readonly LSCollider2D? embedded;
        internal SlabSupport(MeshSlabFaceRegions faces, LSCollider2D? embedded)
        { this.faces = faces; this.embedded = embedded; }
        public bool IntersectsSegment(FixedSegment segment)
        {
            Span<Vector2d> box = stackalloc Vector2d[4];
            return faces.IntersectsSupport(segment, faces.PrismOffsets.IsEmpty
                ? CollisionDetectionMixed.GetConvexPrismOffsets(embedded, box) : faces.PrismOffsets.Span);
        }
    }

    private readonly struct HullSupport : ISection
    {
        private readonly MeshHullSurfaceContacts faces;
        internal HullSupport(MeshHullSurfaceContacts faces) => this.faces = faces;
        public bool IntersectsSegment(FixedSegment segment) => faces.IntersectsSupport(segment);
    }
}
