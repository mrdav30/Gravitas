//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================
using FixedMathSharp;
using FixedMathSharp.Geometry;
using Gravitas.Colliders;
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
        ReadOnlySpan<Vector3d> vertices = mesh.ScaledLocalVertices;
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
                if (vertex != first) _supportFan.Add(-MeshSurfaceBoundaryTopology.Ray(mesh.ScaledLocalVertices[first], mesh.ScaledLocalVertices[vertex]));
            // Global support makes the matched exit a separating plane for the
            // whole fill; a local hole-edge exit need not clear another region.
            Span<Vector2d> localBox = stackalloc Vector2d[4];
            if (!SegmentSlabSurfaceContact.TryGetFeaturePenetration(new FixedSegment(mesh.ScaledLocalVertices[first], mesh.ScaledLocalVertices[second]),
                frame.Origin, frame.Rotation, frame.Center, frame.SlabRotation, frame.Section.RadialFrame.Length,
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
        ContactManifold manifold, int namespace3D, int namespace2D)
    {
        PhysicsMesh mesh = collider.Mesh;
        int owner = mesh.GetManifoldSurfaceOwner(surfaceTriangle);
        int surface = mesh.GetCanonicalSurfaceOrdinal(surfaceTriangle);
        ReadOnlySpan<int> boundary = mesh.GetCanonicalSurfaceBoundaryVertexPairs(surfaceTriangle);
        ReadOnlySpan<Vector3d> vertices = mesh.ScaledLocalVertices;
        FiniteSlabRayFrame frame = faces.Frame;
        Fixed64 length = frame.Section.RadialFrame.Length;
        Fixed64 radius = frame.RadiusValue;
        Fixed64 halfThickness = frame.HalfThickness;
        Span<Vector2d> box = stackalloc Vector2d[4];
        ReadOnlySpan<Vector2d> offsets = embedded == null ? ReadOnlySpan<Vector2d>.Empty
            : CollisionDetectionMixed.GetConvexPrismOffsets(embedded, box);
        frame.SourcePlane.GetExactNormal(out Signed192 nx, out Signed192 ny, out Signed192 nz, out _);
        WideAxis3 normal = new(Signed320.ExtendValue(nx), Signed320.ExtendValue(ny), Signed320.ExtendValue(nz));
        _visitedCorners.Clear();
        for (int edge = 0; edge < boundary.Length; edge += 2)
        {
            int first = boundary[edge], second = boundary[edge + 1];
            int feature = checked(3 * (edge / 2));
            int region = connectivity.RegionCount == 1
                ? frame.Section.IntersectsSegment(new FixedSegment(vertices[first], vertices[second]), offsets) ? 0 : -1
                : MeshSurfaceBoundaryTopology.FindEdgeRegion(mesh, owner, first, second, connectivity, frame.Section, embedded);
            if (region < 0) continue;
            KeepFeature(first, second, feature, region, pointOnly: false, offsets);
            if (_visitedCorners.Add(first) && frame.Section.ContainsPoint(vertices[first], offsets))
                KeepFeature(first, second, feature + 1, region, pointOnly: true, offsets);
            if (_visitedCorners.Add(second) && frame.Section.ContainsPoint(vertices[second], offsets))
                KeepFeature(second, first, feature + 2, region, pointOnly: true, offsets);
        }

        void KeepFeature(int first, int second, int ordinal, int region, bool pointOnly, ReadOnlySpan<Vector2d> prismOffsets)
        {
            Span<WideAxis3> fan = stackalloc WideAxis3[5];
            int count = MeshSurfaceBoundaryTopology.GetIncidentFan(mesh, owner, first, second, !pointOnly, fan);
            // Topology admits the contact normal p-q. The mathematics owner
            // admits the opposite exit ray q-p; negate its halfspaces once.
            for (int index = 0; index < count; index++) fan[index] = -fan[index];
            fan[count++] = faces.Orientation > 0 ? -normal : normal;
            var segment = new FixedSegment(mesh.ScaledLocalVertices[first],
                mesh.ScaledLocalVertices[pointOnly ? first : second]);
            if (!SegmentSlabSurfaceContact.TryGetContact(segment, frame.Origin, frame.Rotation,
                frame.Center, frame.SlabRotation, length, radius, halfThickness, prismOffsets, fan[..count],
                interiorOnly: !pointOnly, out FixedContactAnchors contact)) return;
            var group = new ContactGroupKey(namespace3D, namespace2D, surface, 0, checked(-ordinal - 1));
            manifold.AddContact(new ContactAnchor(contact.FirstAnchor), new ContactAnchor(contact.SecondAnchor),
                contact.Depth, contact.Normal, collider.Material, embedded?.Material ?? collider.Material,
                contact.DepthIsClamped, featureNamespaceA: namespace3D, featureNamespaceB: namespace2D, group: group);
        }
    }
}
