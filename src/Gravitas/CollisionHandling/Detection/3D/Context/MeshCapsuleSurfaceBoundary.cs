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
    private WideAxis3[] _joinedFan = Array.Empty<WideAxis3>(), _transformedFan = Array.Empty<WideAxis3>();
    internal void BuildSurfaceContacts(LSMeshCollider collider, PhysicsMaterial otherMaterial,
        Vector3d center, FixedQuaternion rotation, Fixed64 coreLength, Fixed64 radius,
        int surfaceTriangle, MeshSurfaceConnectivity connectivity, MeshCapsuleFaceRegions faces,
        ContactManifold manifold)
    {
        PhysicsMesh mesh = collider.Mesh;
        int owner = mesh.GetManifoldSurfaceOwner(surfaceTriangle);
        int surface = mesh.GetCanonicalSurfaceOrdinal(surfaceTriangle);
        ReadOnlySpan<int> boundary = mesh.GetCanonicalSurfaceBoundaryVertexPairs(surfaceTriangle);
        ReadOnlySpan<Vector3d> vertices = mesh.ScaledLocalVertices;
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
                    Signed320 n = !piece.Interval.IsClipped ? upper ? Signed320.One : default
                        : upper ? piece.Interval.UpperNumerator : piece.Interval.LowerNumerator;
                    Signed320 d = !piece.Interval.IsClipped ? Signed320.One
                        : upper ? piece.Interval.UpperDenominator : piece.Interval.LowerDenominator;
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
            var triangle = new FixedTriangle(mesh.ScaledLocalVertices[first],
                mesh.ScaledLocalVertices[other], mesh.ScaledLocalVertices[other]);
            int orientation = faces.GetBoundaryOrientation(collider, surfaceTriangle, region);
            Vector3d fallback = -normal * orientation;
            int endpoint = 0;
            FixedContactAnchors contact;
            AxisTriangleContactFeature feature;
            bool found = domain == null
                ? triangle.TryGetCenteredCapsuleContact(mesh.Origin, mesh.Rotation, center, rotation,
                    coreLength, radius, fallback, out contact, out feature, out _)
                : triangle.TryGetClippedCenteredCapsuleContact(mesh.Origin, mesh.Rotation, center, rotation,
                    coreLength, radius, fallback, clip, out contact, out feature, out endpoint);
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
                bool interior = !pointOnly || domain != null && !clip.LowerNumerator.IsZero
                    && !clip.LowerNumerator.Equals(clip.LowerDenominator);
                bool reversed = domain != null && pointOnly && clip.LowerNumerator.Equals(clip.LowerDenominator);
                count = MeshSurfaceBoundaryTopology.GetIncidentFan(mesh, owner,
                    reversed ? second : first, reversed ? first : second, interior, fan);
            }
            else { joinedFan.CopyTo(fan); count = joinedFan.Length; }
            // A successful contact between admitted local envelopes keeps the
            // capsule center representable in the mesh chart (two reaches plus final rounding).
            if (!SupportsFan(feature, fan[..count], !joinedFan.IsEmpty)
                || !SupportsSelectedHemisphere(feature,
                    frame.AuthoredNormal, orientation)) return;
            fan[count++] = orientation > 0 ? frame.AuthoredNormal : -frame.AuthoredNormal;
            if (coreLength > Fixed64.Zero && feature.IsZeroSeparation
                && TryGetPerpendicularFallback(frame.Finite, rotation, fan[..count], out fallback))
            {
                // Select a perpendicular direction in the exact capsule chart,
                // then construct the actual ray using its final Q32.32 normal.
                // The inherited closest query can quantize its core direction;
                // do not infer the final ray depth from the ideal radius alone.
                found = domain == null
                    ? triangle.TryGetCenteredCapsuleContact(mesh.Origin, mesh.Rotation,
                        center, rotation, coreLength, radius, fallback, out contact)
                    : triangle.TryGetClippedCenteredCapsuleContact(mesh.Origin, mesh.Rotation,
                        center, rotation, coreLength, radius, fallback, clip, out contact, out _, out _);
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
        in WideAxis3 canonicalNormal, int orientation)
    {
        return orientation * feature.CompareSeparationProjection(
            Signed192.NarrowProven(canonicalNormal.X), Signed192.NarrowProven(canonicalNormal.Y),
            Signed192.NarrowProven(canonicalNormal.Z)) <= 0;
    }

    private bool TryGetPerpendicularFallback(in CapsuleFiniteSectionFrame finite,
        FixedQuaternion rotation, ReadOnlySpan<WideAxis3> authoredFan, out Vector3d fallback)
    {
        fallback = default;
        var core = new WideAxis3(default, Signed320.ExtendValue(Signed192.Signed(1)), default);
        if (authoredFan.Length > 5 && _transformedFan.Length < authoredFan.Length)
            Array.Resize(ref _transformedFan, Math.Max(authoredFan.Length, 2 * _transformedFan.Length));
        Span<WideAxis3> fan = authoredFan.Length <= 5 ? stackalloc WideAxis3[5] : _transformedFan;
        // Admitted mesh edges span at most 8,192 units (<46 raw bits). A joined
        // relative basis adds <132 bits, so the authored rays still fit Signed192.
        for (int index = 0; index < authoredFan.Length; index++)
            fan[index] = WideRigidProjection.TransformLocalAxis(
                finite.ShapeFrame.Basis, Signed192.NarrowProven(authoredFan[index].X),
                Signed192.NarrowProven(authoredFan[index].Y), Signed192.NarrowProven(authoredFan[index].Z));
        ReadOnlySpan<WideAxis3> constraints = fan[..authoredFan.Length];
        FixedQuaternion targetRotation = rotation;
        WideAxis3 selected = -constraints[^1];
        if (KeepDirection(WideAxis3.Cross(core, WideAxis3.Cross(selected, core)), constraints, targetRotation, out fallback)) return true;
        foreach (WideAxis3 wall in constraints)
        {
            WideAxis3 direction = WideAxis3.Cross(core, wall);
            if (KeepDirection(direction, constraints, targetRotation, out fallback)
                || KeepDirection(-direction, constraints, targetRotation, out fallback)) return true;
        }
        // A genuine perimeter fan has two noncollinear rays, so its circle
        // constraints cannot all vanish. Every nonempty clipped circle has
        // an active wall already represented by the signed crosses above.
        return false;
    }

    private static bool KeepDirection(WideAxis3 direction, ReadOnlySpan<WideAxis3> fan,
        FixedQuaternion rotation, out Vector3d result)
    {
        result = default;
        if (direction.IsZero) return false;
        foreach (WideAxis3 inward in fan)
            if (WideAxis3.Dot(direction, inward).Sign > 0) return false;
        // The exact relative basis adds <=130 bits to authored constraints
        // (<=130 bits). Crossing the integer local Up axis stays within
        // Signed320; the final normalized rigid transform fits Signed576.
        WideRigidProjection.TransformLocalAxis(new WideRationalBasis3d(rotation),
            direction, out Signed576 x, out Signed576 y, out Signed576 z);
        result = WideNormalization.GetNormalized(x, y, z);
        return true;
    }

}
