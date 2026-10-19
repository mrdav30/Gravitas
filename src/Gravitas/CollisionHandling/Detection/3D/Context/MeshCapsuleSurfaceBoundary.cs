//=======================================================================
// MeshCapsuleSurfaceBoundary.cs
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

/// <summary>Preserves finite radial contacts on a trusted surface's genuine perimeter.</summary>
internal sealed class MeshCapsuleSurfaceBoundary
{
    private readonly SwiftHashSet<int> _visitedCorners = new();
    private WideAxis3[] _joinedFan = Array.Empty<WideAxis3>();
    private WideDirection3d[] _transformedFan = Array.Empty<WideDirection3d>();
    internal void BuildSurfaceContacts(LSMeshCollider collider, PhysicsMaterial otherMaterial,
        Vector3d center, FixedQuaternion rotation, Fixed64 coreLength, Fixed64 radius,
        int surfaceTriangle, MeshSurfaceConnectivity connectivity, MeshCapsuleFaceRegions faces,
        ContactManifold manifold)
    {
        PhysicsMesh mesh = collider.Mesh;
        int owner = mesh.GetManifoldSurfaceOwner(surfaceTriangle);
        int surface = mesh.GetCanonicalSurfaceOrdinal(surfaceTriangle);
        ReadOnlySpan<int> boundary = mesh.GetCanonicalSurfaceBoundaryVertexPairs(surfaceTriangle);
        ReadOnlySpan<Vector3d> vertices = mesh.GeometryVertices;
        CapsulePlaneRayFrame frame = faces.GetBoundaryFrame(collider, surfaceTriangle, out Vector3d normal);
        _visitedCorners.Clear();
        MeshSurfaceBoundaryDomain? domain = faces.GetJoinedBoundary(collider, surfaceTriangle);
        if (domain != null)
        {
            ReadOnlySpan<MeshSurfaceBoundaryDomain.BoundarySpan> spans = domain.Spans;
            for (int span = 0; span < spans.Length; span++)
            {
                var piece = spans[span];
                if (!frame.Finite.IntersectsSegment(new FixedSegment(piece.Start, piece.End), piece.Interval)) continue;
                int region = connectivity.RegionCount == 1 ? 0 : MeshSurfaceBoundaryTopology.FindEdgeRegion(mesh,
                    owner, piece.First, piece.Second, connectivity, frame.Finite);
                // Exact admission of a certified boundary span implies an
                // admitted incident triangle in the same local component.
                System.Diagnostics.Debug.Assert(region >= 0);
                int ordinal = checked(3 * span);
                // Joined interiors disappear, but their endpoints may own a
                // genuine outer corner whose other sectors have higher IDs.
                if (!faces.IsJoinedSpan(collider, surfaceTriangle, span))
                    KeepFeature(piece.First, piece.Second, ordinal, region, false, piece.Interval, span);
                for (int end = 0; end < 2; end++)
                {
                    bool upper = end != 0;
                    Signed832 one = Signed832.ExtendValue(Signed192.One);
                    Signed832 n = !piece.Interval.IsClipped ? upper ? one : default
                        : upper ? piece.Interval.WideUpperNumerator : piece.Interval.WideLowerNumerator;
                    Signed832 d = !piece.Interval.IsClipped ? one
                        : upper ? piece.Interval.WideUpperDenominator : piece.Interval.WideLowerDenominator;
                    // Authored corners share vertex identity; consecutive pieces
                    // of one canonical edge share their exact cut endpoint.
                    int vertex = n.IsZero ? piece.First : n.Equals(d) ? piece.Second : -1;
                    if (vertex >= 0)
                    { if (!_visitedCorners.Add(vertex)) continue; }
                    // Every interior lower cut is the preceding span's upper
                    // cut. Closed admission visits that shared point there.
                    else if (!upper) continue;
                    var point = new RationalSegmentInterval(n, d, n, d);
                    KeepFeature(piece.First, piece.Second, ordinal + end + 1, region, true, point, span, upper);
                }
            }
            return;
        }
        for (int edge = 0; edge < boundary.Length; edge += 2)
        {
            int first = boundary[edge], second = boundary[edge + 1];
            int feature = checked(3 * (edge / 2));
            // A straight edge's admitted interval needs no subedge routing
            // when the surface has just one connected finite region.
            int region = connectivity.RegionCount == 1
                ? frame.Finite.IntersectsSegment(new FixedSegment(vertices[first], vertices[second])) ? 0 : -1
                : MeshSurfaceBoundaryTopology.FindEdgeRegion(mesh, owner, first, second,
                    connectivity, frame.Finite);
            if (region < 0) continue;
            KeepFeature(first, second, feature, region, pointOnly: false);
            if (_visitedCorners.Add(first) && frame.Finite.ContainsPoint(vertices[first]))
                KeepFeature(first, second, feature + 1, region, pointOnly: true);
            if (_visitedCorners.Add(second) && frame.Finite.ContainsPoint(vertices[second]))
                KeepFeature(second, first, feature + 2, region, pointOnly: true);
        }

        void KeepFeature(int first, int second, int ordinal, int region, bool pointOnly,
            in RationalSegmentInterval clip = default, int span = -1, bool upper = false)
        {
            int other = pointOnly && domain == null ? first : second;
            var triangle = new FixedTriangle(mesh.GeometryVertices[first],
                mesh.GeometryVertices[other], mesh.GeometryVertices[other]);
            int orientation = faces.GetBoundaryOrientation(collider, surfaceTriangle, region);
            Vector3d fallback = -normal * orientation;
            int endpoint = 0;
            FixedContactAnchors contact;
            AxisTriangleContactFeature feature;
            bool localChart;
            bool found = domain == null
                ? triangle.TryGetCenteredCapsuleContact(mesh.GeometryFrame, center, rotation,
                    coreLength, radius, fallback, out contact, out feature, out localChart)
                : triangle.TryGetClippedCenteredCapsuleContact(mesh.GeometryFrame, center, rotation,
                    coreLength, radius, fallback, clip, out contact, out feature, out endpoint, out localChart);
            if (!found || !pointOnly && (endpoint != 0 || feature.TriangleVertex >= 0)) return;
            bool suppress = false;
            ReadOnlySpan<WideAxis3> joinedFan = domain != null && pointOnly
                ? faces.GetJoinedEndpointFan(collider, surfaceTriangle, span, upper, out suppress) : default;
            if (suppress) return;
            if (joinedFan.Length + 1 > _joinedFan.Length && !joinedFan.IsEmpty)
                Array.Resize(ref _joinedFan, Math.Max(joinedFan.Length + 1, 2 * _joinedFan.Length));
            Span<WideAxis3> fan = joinedFan.IsEmpty ? stackalloc WideAxis3[5] : _joinedFan;
            int count;
            if (joinedFan.IsEmpty)
            {
                bool interior = !pointOnly || domain != null && !clip.WideLowerNumerator.IsZero
                    && !clip.WideLowerNumerator.Equals(clip.WideLowerDenominator);
                bool reversed = domain != null && pointOnly && clip.WideLowerNumerator.Equals(clip.WideLowerDenominator);
                count = MeshSurfaceBoundaryTopology.GetIncidentFan(mesh, owner,
                    reversed ? second : first, reversed ? first : second, interior, fan);
            }
            else { joinedFan.CopyTo(fan); count = joinedFan.Length; }
            // Joined rays have world orientation. Preserve the ordinary local
            // chart when possible; otherwise map both exact separation and
            // native incident rays into the same world chart before testing.
            bool worldChart = !localChart || !joinedFan.IsEmpty;
            WideAxis3 canonicalNormal = frame.AuthoredNormal;
            if (worldChart)
            {
                if (localChart) feature = feature.GetWorldFeatureFromRigidLocal(mesh.GeometryFrame.Rotation);
                if (joinedFan.IsEmpty)
                    for (int index = 0; index < count; index++)
                        fan[index] = mesh.GeometryFrame.GetWorldDirectionNumerators(fan[index]);
                WideDirection3d worldNormal = new WideDirection3d(frame.AuthoredNormal).Transform(mesh.GeometryFrame.GeometryBasis);
                canonicalNormal = new WideAxis3(Signed320.NarrowValue(worldNormal.X),
                    Signed320.NarrowValue(worldNormal.Y), Signed320.NarrowValue(worldNormal.Z));
            }
            if (!SupportsFan(feature, fan[..count], worldChart)
                || !SupportsSelectedHemisphere(feature, canonicalNormal, orientation, worldChart)) return;
            fan[count++] = orientation > 0 ? canonicalNormal : -canonicalNormal;
            if (coreLength > Fixed64.Zero && feature.IsZeroSeparation
                && TryGetPerpendicularFallback(frame.Finite, rotation, fan[..count], worldChart, out fallback))
            {
                // Select a perpendicular direction in the exact capsule chart,
                // then construct the actual ray using its final Q32.32 normal.
                // The inherited closest query can quantize its core direction;
                // do not infer the final ray depth from the ideal radius alone.
                found = domain == null
                    ? triangle.TryGetCenteredCapsuleContact(mesh.GeometryFrame,
                        center, rotation, coreLength, radius, fallback, out contact, out _, out _)
                    : triangle.TryGetClippedCenteredCapsuleContact(mesh.GeometryFrame,
                        center, rotation, coreLength, radius, fallback, clip, out contact, out _, out _, out _);
                System.Diagnostics.Debug.Assert(found);
            }
            var group = new ContactGroupKey(0, 0, surface, 0, checked(-ordinal - 1));
            manifold.AddContact(new ContactAnchor(contact.FirstAnchor), new ContactAnchor(contact.SecondAnchor),
                contact.Depth, contact.Normal, collider.Material, otherMaterial, contact.DepthIsClamped, group: group);
        }
    }

    private static bool SupportsFan(in AxisTriangleContactFeature feature, ReadOnlySpan<WideAxis3> fan, bool wide)
    {
        foreach (WideAxis3 inward in fan)
        {
            if ((wide ? feature.CompareSeparationProjection(inward)
                : feature.CompareSeparationProjection(Signed192.NarrowProven(inward.X),
                    Signed192.NarrowProven(inward.Y), Signed192.NarrowProven(inward.Z))) > 0) return false;
        }
        return true;
    }

    private static bool SupportsSelectedHemisphere(in AxisTriangleContactFeature feature,
        in WideAxis3 canonicalNormal, int orientation, bool wide)
    {
        return orientation * (wide ? feature.CompareSeparationProjection(canonicalNormal) : feature.CompareSeparationProjection(
            Signed192.NarrowProven(canonicalNormal.X), Signed192.NarrowProven(canonicalNormal.Y),
            Signed192.NarrowProven(canonicalNormal.Z))) <= 0;
    }

    private bool TryGetPerpendicularFallback(in CapsuleFiniteSectionFrame finite,
        FixedQuaternion rotation, ReadOnlySpan<WideAxis3> authoredFan, bool worldChart, out Vector3d fallback)
    {
        fallback = default;
        if (authoredFan.Length > 5 && _transformedFan.Length < authoredFan.Length)
            Array.Resize(ref _transformedFan, Math.Max(authoredFan.Length, 2 * _transformedFan.Length));
        Span<WideDirection3d> fan = authoredFan.Length <= 5 ? stackalloc WideDirection3d[5] : _transformedFan;
        WideRationalBasis3d basis = worldChart ? WideRationalBasis3d.CreateRelative(new WideRationalBasis3d(rotation), new WideRationalBasis3d(FixedQuaternion.Identity)) : finite.ShapeFrame.Basis;
        // Authored rays need the relative source basis; joined rays already
        // have world orientation and need only the capsule's inverse basis.
        // Complete transformed rays fit Signed576, including affine scale.
        for (int index = 0; index < authoredFan.Length; index++)
            fan[index] = new WideDirection3d(authoredFan[index]).Transform(basis);
        ReadOnlySpan<WideDirection3d> constraints = fan[..authoredFan.Length];
        FixedQuaternion targetRotation = rotation;
        WideDirection3d selected = -constraints[^1];
        if (KeepDirection(new WideDirection3d(selected.X, default, selected.Z), constraints, targetRotation, out fallback)) return true;
        foreach (WideDirection3d wall in constraints)
        {
            WideDirection3d direction = new(wall.Z, default, WideArithmetic.SubtractSigned576(default, wall.X));
            if (KeepDirection(direction, constraints, targetRotation, out fallback)
                || KeepDirection(-direction, constraints, targetRotation, out fallback)) return true;
        }
        // A genuine perimeter fan has two noncollinear rays, so its circle
        // constraints cannot all vanish. Every nonempty clipped circle has
        // an active wall already represented by the signed crosses above.
        return false;
    }

    private static bool KeepDirection(WideDirection3d direction, ReadOnlySpan<WideDirection3d> fan,
        FixedQuaternion rotation, out Vector3d result)
    {
        result = default;
        if (direction.IsZero) return false;
        foreach (WideDirection3d inward in fan)
            if (WideDirection3d.Dot(direction, inward).Sign > 0) return false;
        // Local-Up projection preserves the complete fan width. The final
        // normalized rigid transform still fits Signed576, including the
        // exact world rays supplied by differently scaled joined members.
        result = direction.Transform(new WideRationalBasis3d(rotation)).GetNormal(false);
        return true;
    }

}
