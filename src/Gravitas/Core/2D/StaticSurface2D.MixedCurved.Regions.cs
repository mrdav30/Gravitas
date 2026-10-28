// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
using System;
using FixedMathSharp;
using FixedMathSharp.Geometry;
using Gravitas.Colliders;
using Gravitas.CollisionHandling;

namespace Gravitas;

/// <content>Connects exact finite source strata before reducing curved response samples.</content>
internal sealed partial class StaticSurface2D
{
    private void AddMixedCurvedCertificatePlane(int selected, bool physical)
    {
        MixedCurvedCertificate seed = _mixedCurvedCertificates[selected];
        int original = _mixedMember;
        for (int index = 0; index < _mixedCurvedCertificates.Count; index++)
        {
            MixedCurvedCertificate source = _mixedCurvedCertificates[index];
            if (_mixedCurvedCertificateGroups[index] >= 0
                || _mixedComponents[source.Member] != _mixedComponents[seed.Member]
                || !SameMixedCurvedPlane(seed.Axis, seed.Reference, source.Axis, source.Reference)) continue;
            // An own physical proof forces a physical source response; far
            // certificates are prepared only for components without one.
            System.Diagnostics.Debug.Assert(source.Physical == physical);
            _mixedMember = source.Member; var window = source.Window;
            _mixedCurvedCertificateGroups[index] = _mixedCurvedGroups.Count;
            AddMixedCurvedSample(source.Contact, window.Face, physical, window.Interval, window.Lower, window.Upper);
        }
        _mixedMember = original;
    }

    private void GetMixedCurvedPlaneForRow(int member, int face, in FixedContactAnchors contact,
        out WideDirection3d axis, out RationalPoint3d reference)
    {
        if (face >= -2) GetMixedCurvedPlane(member, face, out axis, out reference);
        else
        {
            // Nonface minima have one isolated sample; this rounded direction
            // is used only by final sample reduction, never geometry admission.
            axis = new(Signed576.ExtendValue(Signed192.Raw(contact.Normal.X)),
                Signed576.ExtendValue(Signed192.Raw(contact.Normal.Y)), Signed576.ExtendValue(Signed192.Raw(contact.Normal.Z)));
            reference = default;
        }
    }

    private void CompleteMixedCurvedRegions(bool cone, Fixed64 height, Span<WideDirection3d> vertices)
    {
        for (int second = 0; second < _mixedCurvedGroups.Count; second++)
        {
            CurvedGroup b = _mixedCurvedGroups[second];
            if (b.Face < -2 || b.FarBoundary) continue;
            for (int first = 0; first < second; first++)
            {
                CurvedGroup a = _mixedCurvedGroups[first];
                if (a.Face < -2 || FindMixedRegion(first) == FindMixedRegion(second)
                    || _mixedComponents[a.Member] != _mixedComponents[b.Member]
                    || !SameMixedCurvedPlane(a.Axis, a.Reference, b.Axis, b.Reference)) continue;
                var member = _mixedLocal[a.Member]; var prism = _mixedPrisms[a.Member]; var mover = _mixedMoving!;
                var geometry = new CurvedPrismContactGeometry(mover.Center, mover.Rotation, Vector3d.Up,
                    height, mover.ScaledRadius, MixedEmbedded2DGeometry.GetCenter3D(member), member.ConvexRotation,
                    prism.Offsets, member.MixedHalfThickness, cone, vertices);
                bool connected = a.Face < 0 ? MixedCurvedCapsTouch(a, b, geometry) : MixedCurvedSidesTouch(a, b, geometry);
                if (connected) _mixedRegionParents[FindMixedRegion(second)] = FindMixedRegion(first);
            }
        }
    }

    private bool MixedCurvedCapsTouch(in CurvedGroup a, in CurvedGroup b, in CurvedPrismContactGeometry geometry)
    {
        // Both physical rows lie in this one exposed convex cap. Its
        // intersection with a convex cone/cylinder is connected even when
        // boundary-line producers meet it on opposite sides of the section.
        if (a.Member == b.Member) return true;
        MixedPrism prism = _mixedPrisms[a.Member];
        for (int bandIndex = 0; bandIndex < prism.BandCount; bandIndex++)
        {
            MixedBand band = prism.Bands[bandIndex];
            if (a.Lower < band.Lower || a.Lower > band.Upper) continue;
            foreach (var span in band.Boundary.Spans)
            {
                if (span.Kind != MeshSurfaceBoundaryDomain2D.SpanKind.Joined || span.OtherMember != b.Member) continue;
                RationalSegmentInterval clip = span.Interval;
                if (a.BoundaryEdge >= 0 && !IntersectMixedCurvedEdges(a.Member, span.First, clip,
                    a.Member, a.BoundaryEdge, a.Clip, out clip)) continue;
                if (b.BoundaryEdge >= 0 && !IntersectMixedCurvedEdges(a.Member, span.First, clip,
                    b.Member, b.BoundaryEdge, b.Clip, out clip)) continue;
                if (CurvedPrismContact.IntersectsBoundaryLine(geometry, span.First, clip, a.Lower, a.Lower)) return true;
            }
        }
        return false;
    }

    // Cooriented side windows have disjoint interiors; shared strata are lines.
    private bool MixedCurvedSidesTouch(in CurvedGroup a, in CurvedGroup b, in CurvedPrismContactGeometry geometry)
    {
        Fixed64 delta = _mixedLocal[b.Member].MixedSlabCenterY - _mixedLocal[a.Member].MixedSlabCenterY;
        Fixed64 lower = FixedMath.Max(a.Lower, b.Lower + delta), upper = FixedMath.Min(a.Upper, b.Upper + delta);
        if (lower > upper || !IntersectMixedCurvedEdges(a.Member, a.Face, a.Clip, b.Member, b.Face, b.Clip, out var clip)) return false;
        if (a.Member == b.Member) return CurvedPrismContact.IntersectsBoundaryLine(geometry, a.Face, clip, lower, upper);
        MixedPrism prism = _mixedPrisms[a.Member];
        for (int bandIndex = 0; bandIndex < prism.BandCount; bandIndex++)
        {
            MixedBand band = prism.Bands[bandIndex];
            Fixed64 bottom = FixedMath.Max(lower, band.Lower), top = FixedMath.Min(upper, band.Upper);
            if (bottom > top) continue;
            foreach (var span in band.Boundary.Spans)
                if (span.Kind == MeshSurfaceBoundaryDomain2D.SpanKind.Joined && span.OtherMember == b.Member
                    && IntersectMixedCurvedEdges(a.Member, a.Face, clip, a.Member, span.First, span.Interval, out var shared)
                    && CurvedPrismContact.IntersectsBoundaryLine(geometry, a.Face, shared, bottom, top)) return true;
        }
        return false;
    }

    private bool IntersectMixedCurvedEdges(int member, int edge, in RationalSegmentInterval clip,
        int otherMember, int otherEdge, in RationalSegmentInterval otherClip, out RationalSegmentInterval intersection)
    {
        MixedPrism a = _mixedPrisms[member], b = _mixedPrisms[otherMember];
        var first = _mixedLocal[member]; var second = _mixedLocal[otherMember];
        return WideConvex2dRelations.TryIntersectBoundarySegments(first.Center, first.ConvexRotation,
            new(a.Offsets[edge], a.Offsets[(edge + 1) % a.Offsets.Length]), clip, second.Center, second.ConvexRotation,
            new(b.Offsets[otherEdge], b.Offsets[(otherEdge + 1) % b.Offsets.Length]), otherClip, out intersection);
    }

    private bool IsBestMixedCurvedFarRegion(int group)
    {
        CurvedGroup source = _mixedCurvedGroups[group]; int component = _mixedComponents[source.Member];
        // Far groups are never coalesced and retain one sample at their group index.
        Fixed64 depth = _mixedSamples[group].Contact.Depth;
        for (int other = 0; other < _mixedCurvedGroups.Count; other++)
            if (other != group && FindMixedRegion(other) == other && _mixedComponents[_mixedCurvedGroups[other].Member] == component)
            {
                Fixed64 candidate = _mixedSamples[other].Contact.Depth;
                if (candidate < depth || candidate == depth && other < group) return false;
            }
        return true;
    }

    private void RequestMixedCurvedFarReach()
    {
        for (int group = 0; group < _mixedCurvedGroups.Count; group++)
            if (FindMixedRegion(group) == group && _mixedCurvedGroups[group].FarBoundary && IsBestMixedCurvedFarRegion(group))
                RequestMixedReach(_mixedSamples[group].Contact.Depth);
    }

    private void AppendMixedCurvedRegions(LSCollider2D overlap, ContactManifold destination, int namespace3D, int namespace2D)
    {
        Span<int> chosen = stackalloc int[ContactManifold.MaxContactsPerGroup];
        var body = _mixedMoving!.Body ?? _mixedMoving.CompoundOwner?.Body;
        var center = body == null ? new FixedPointAnchor(_mixedMoving.Center, _mixedMoving.Rotation, Vector3d.Zero)
            : new FixedPointAnchor(body.Position3d, body.Rotation, body.LocalCenterOfMassOffset);
        for (int group = 0; group < _mixedCurvedGroups.Count; group++)
        {
            CurvedGroup row = _mixedCurvedGroups[group];
            if (FindMixedRegion(group) != group || _mixedCurvedCarriers[_mixedComponents[row.Member]] != overlap.Id
                || row.FarBoundary && !IsBestMixedCurvedFarRegion(group)) continue;
            AppendMixedRegionSamples(group, row.Axis, destination, namespace3D, namespace2D, chosen, center);
        }
    }
}
