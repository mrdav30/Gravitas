//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================
using FixedMathSharp;
using FixedMathSharp.Geometry;
using Gravitas.Colliders;
using Gravitas.Materials;
using SwiftCollections;
using System;

namespace Gravitas.CollisionHandling;

/// <summary>Preserves matched finite slab contacts on a trusted surface's genuine perimeter.</summary>
internal sealed class MeshSlabSurfaceBoundary
{
    private readonly SwiftHashSet<int> _visitedCorners = new();
    private readonly SwiftList<int> _supportVertices = new();
    private readonly SwiftList<WideAxis3> _supportFan = new();

    internal void AppendSupportFallback(LSMeshCollider collider, LSCollider2D? embedded,
        int surfaceTriangle, MeshSlabFaceRegions faces, ContactManifold manifold, int namespace3D, int namespace2D)
    {
        PhysicsMesh mesh = collider.Mesh;
        ReadOnlySpan<int> boundary = mesh.GetCanonicalSurfaceBoundaryVertexPairs(surfaceTriangle);
        ReadOnlySpan<Vector3d> vertices = mesh.GeometryVertices;
        FiniteSlabRayFrame frame = faces.Frame;
        _supportVertices.FastClear(); _visitedCorners.Clear();
        foreach (int vertex in boundary)
            if (_visitedCorners.Add(vertex)) _supportVertices.Add(vertex);
        _supportFan.EnsureCapacity(_supportVertices.Count);
        bool found = false;
        FixedContactAnchors best = default;
        int bestFeature = 0;
        for (int edge = 0; edge < boundary.Length; edge += 2)
        {
            int first = boundary[edge], second = boundary[edge + 1];
            Keep(first, second, checked(3 * (edge / 2)), false);
            if (_visitedCorners.Remove(first)) Keep(first, first, checked(3 * (edge / 2) + 1), true);
            if (_visitedCorners.Remove(second)) Keep(second, second, checked(3 * (edge / 2) + 2), true);
        }
        // A farthest owner/slab pair supplies an actual boundary vertex with a
        // global support normal, so the complete vertex charts cannot be empty.
        System.Diagnostics.Debug.Assert(found);
        manifold.AddContact(new ContactAnchor(best.FirstAnchor), new ContactAnchor(best.SecondAnchor),
            best.Depth, best.Normal, collider.Material, embedded?.Material ?? collider.Material, best.DepthIsClamped,
            featureNamespaceA: namespace3D, featureNamespaceB: namespace2D,
            group: new ContactGroupKey(namespace3D, namespace2D, mesh.GetCanonicalSurfaceOrdinal(surfaceTriangle), 0, -bestFeature - 1));

        void Keep(int first, int second, int feature, bool pointOnly)
        {
            _supportFan.FastClear();
            foreach (int vertex in _supportVertices.AsReadOnlySpan())
                if (vertex != first) _supportFan.Add(-mesh.GeometryFrame.GetWorldDirectionNumerators(
                    MeshSurfaceBoundaryTopology.Ray(mesh.GeometryVertices[first], mesh.GeometryVertices[vertex])));
            // Global support makes the matched exit a separating plane for the
            // whole fill; a local hole-edge exit need not clear another region.
            Span<Vector2d> localBox = stackalloc Vector2d[4];
            if (!SegmentSlabSurfaceContact.TryGetFeaturePenetration(new FixedSegment(mesh.GeometryVertices[first], mesh.GeometryVertices[second]),
                mesh.GeometryFrame, frame.Center, frame.SlabRotation, frame.Section.RadialFrame.Length,
                frame.RadiusValue, frame.HalfThickness, CollisionDetectionMixed.GetConvexPrismOffsets(embedded, localBox),
                _supportFan.AsReadOnlySpan(), !pointOnly, out FixedContactAnchors contact)) return;
            if (found && (contact.DepthIsClamped && !best.DepthIsClamped
                || contact.DepthIsClamped == best.DepthIsClamped && contact.Depth >= best.Depth)) return;
            // Only final depth ranking is Q32: unclamped candidate order differs
            // from exact ranking by at most one raw unit; canonical ties stay first.
            found = true; best = contact; bestFeature = feature;
        }
    }

    internal void BuildSurfaceContacts(LSMeshCollider collider, LSCollider2D? embedded,
        int surfaceTriangle, MeshSurfaceConnectivity connectivity, MeshSlabFaceRegions faces,
        ContactManifold manifold, int namespace3D, int namespace2D, PhysicsMaterial? targetMaterial = null)
    {
        PhysicsMesh mesh = collider.Mesh;
        int owner = mesh.GetManifoldSurfaceOwner(surfaceTriangle);
        int surface = mesh.GetCanonicalSurfaceOrdinal(surfaceTriangle);
        ReadOnlySpan<int> boundary = mesh.GetCanonicalSurfaceBoundaryVertexPairs(surfaceTriangle);
        ReadOnlySpan<Vector3d> vertices = mesh.GeometryVertices;
        FiniteSlabRayFrame frame = faces.Frame;
        Fixed64 length = frame.Section.RadialFrame.Length;
        Fixed64 radius = frame.RadiusValue;
        Fixed64 halfThickness = frame.HalfThickness;
        Span<Vector2d> box = stackalloc Vector2d[4];
        ReadOnlySpan<Vector2d> offsets = !faces.PrismOffsets.IsEmpty ? faces.PrismOffsets.Span
            : CollisionDetectionMixed.GetConvexPrismOffsets(embedded, box);
        WideDirection3d worldNormal = WideTriangleRelations.GetScaledTriangleLocalNormal(frame.SourcePlane, mesh.GeometryFrame)
            .Transform(mesh.GeometryFrame.GeometryBasis);
        WideAxis3 normal = new(Signed320.NarrowValue(worldNormal.X), Signed320.NarrowValue(worldNormal.Y), Signed320.NarrowValue(worldNormal.Z));
        _visitedCorners.Clear();
        MeshSurfaceBoundaryDomain? domain = faces.JoinedBoundary;
        if (domain != null)
        {
            for (int span = 0; span < domain.Spans.Length; span++)
            {
                var piece = domain.Spans[span];
                if (faces.IsJoinedSpan(span) || !frame.Section.IntersectsSegment(new FixedSegment(piece.Start, piece.End), offsets)) continue;
                int region = MeshSurfaceBoundaryTopology.FindEdgeRegion(mesh, owner, piece.First, piece.Second,
                    connectivity, frame.Section, faces.PrismOffsets);
                int feature = checked(3 * span);
                RationalSegmentInterval clip = faces.GetRetainedInterval(ref span);
                // An intersected trusted perimeter has an admitted incident triangle.
                System.Diagnostics.Debug.Assert(region >= 0);
                KeepFeature(piece.First, piece.Second, feature, region, false, offsets, clip);
            }
            for (int edge = 0; edge < boundary.Length; edge += 2)
            {
                int first = boundary[edge], second = boundary[edge + 1];
                int region = MeshSurfaceBoundaryTopology.FindEdgeRegion(mesh, owner, first, second,
                    connectivity, frame.Section, faces.PrismOffsets);
                if (region < 0) continue;
                int feature = checked(3 * (domain.Spans.Length + edge / 2));
                if (_visitedCorners.Add(first) && frame.Section.ContainsPoint(vertices[first], offsets))
                    KeepFeature(first, second, feature + 1, region, true, offsets);
                if (_visitedCorners.Add(second) && frame.Section.ContainsPoint(vertices[second], offsets))
                    KeepFeature(second, first, feature + 2, region, true, offsets);
            }
            return;
        }
        for (int edge = 0; edge < boundary.Length; edge += 2)
        {
            int first = boundary[edge], second = boundary[edge + 1];
            int feature = checked(3 * (edge / 2));
            int region = connectivity.RegionCount == 1
                ? frame.Section.IntersectsSegment(new FixedSegment(vertices[first], vertices[second]), offsets) ? 0 : -1
                : !faces.PrismOffsets.IsEmpty
                    ? MeshSurfaceBoundaryTopology.FindEdgeRegion(mesh, owner, first, second, connectivity, frame.Section, faces.PrismOffsets)
                    : MeshSurfaceBoundaryTopology.FindEdgeRegion(mesh, owner, first, second, connectivity, frame.Section, embedded);
            if (region < 0) continue;
            KeepFeature(first, second, feature, region, pointOnly: false, offsets);
            if (_visitedCorners.Add(first) && frame.Section.ContainsPoint(vertices[first], offsets))
                KeepFeature(first, second, feature + 1, region, pointOnly: true, offsets);
            if (_visitedCorners.Add(second) && frame.Section.ContainsPoint(vertices[second], offsets))
                KeepFeature(second, first, feature + 2, region, pointOnly: true, offsets);
        }

        void KeepFeature(int first, int second, int ordinal, int region, bool pointOnly, ReadOnlySpan<Vector2d> prismOffsets,
            in RationalSegmentInterval clip = default)
        {
            bool suppress = false;
            ReadOnlySpan<WideAxis3> joinedFan = domain != null && pointOnly
                ? faces.GetJoinedCornerFan(collider, surfaceTriangle, first, out suppress) : default;
            if (suppress) return;
            Span<WideAxis3> fan = stackalloc WideAxis3[Math.Max(5, joinedFan.Length + 1)];
            int count;
            if (joinedFan.IsEmpty)
            {
                count = MeshSurfaceBoundaryTopology.GetIncidentFan(mesh, owner, first, second, !pointOnly, fan);
                for (int index = 0; index < count; index++)
                    fan[index] = mesh.GeometryFrame.GetWorldDirectionNumerators(fan[index]);
            }
            else { joinedFan.CopyTo(fan); count = joinedFan.Length; }
            // Topology admits the contact normal p-q. The mathematics owner
            // admits the opposite exit ray q-p; negate its halfspaces once.
            for (int index = 0; index < count; index++) fan[index] = -fan[index];
            fan[count++] = faces.Orientation > 0 ? -normal : normal;
            var segment = new FixedSegment(mesh.GeometryVertices[first],
                mesh.GeometryVertices[pointOnly ? first : second]);
            if (!SegmentSlabSurfaceContact.TryGetContact(segment, mesh.GeometryFrame,
                frame.Center, frame.SlabRotation, length, radius, halfThickness, prismOffsets, fan[..count],
                interiorOnly: !pointOnly, clip, out FixedContactAnchors contact, fullHeight: frame.FullHeight != Fixed64.Zero)) return;
            var group = new ContactGroupKey(namespace3D, namespace2D, surface, 0, checked(-ordinal - 1), sourceIdA: faces.SourceId);
            manifold.AddContact(new ContactAnchor(contact.FirstAnchor), new ContactAnchor(contact.SecondAnchor),
                contact.Depth, contact.Normal, collider.Material, targetMaterial ?? embedded?.Material ?? collider.Material,
                contact.DepthIsClamped, featureNamespaceA: namespace3D, featureNamespaceB: namespace2D, group: group);
        }
    }
}
