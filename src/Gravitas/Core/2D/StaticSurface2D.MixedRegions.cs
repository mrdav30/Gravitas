// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
using FixedMathSharp;
using FixedMathSharp.Geometry;
using Gravitas.CollisionHandling;
using SwiftCollections;
using System;

namespace Gravitas;

/// <content>Separates projected finite supports before reducing their response samples.</content>
internal sealed partial class StaticSurface2D
{
    private readonly SwiftList<MixedRegion> _mixedRegions = new();
    private readonly SwiftList<int> _mixedRegionParents = new(), _mixedRegionOrdinals = new();
    private readonly SwiftList<MixedGroup> _mixedRegionResponses = new();
    private readonly MixedRegionSupport _mixedRegionSupport = new();

    private void AddMixedRegion(int member, int face, in MeshSurfaceBoundaryDomain2D.BoundarySpan span,
        Fixed64 lower, Fixed64 upper, int firstSample, int band = -1, int edgeStart = -1, int edgeEnd = -1,
        RationalSegmentInterval edgeInterval = default)
    {
        if (firstSample == _mixedSamples.Count) return;
        int region = _mixedRegions.Count;
        _mixedRegions.Add(new(_mixedGroup, member, face, band, span, lower, upper, edgeStart, edgeEnd, edgeInterval));
        _mixedRegionParents.Add(region);
        for (int index = firstSample; index < _mixedSamples.Count; index++)
        {
            MixedSample sample = _mixedSamples[index];
            _mixedSamples[index] = new(region, sample.Supplier, sample.Point, sample.Contact);
        }
    }

    private bool HasMixedFaceRegion(int group)
    {
        foreach (MixedRegion region in _mixedRegions.AsReadOnlySpan())
            if (region.Group == group && region.Face >= 0) return true;
        return false;
    }

    private void CompleteMixedRegions()
    {
        for (int second = 0; second < _mixedRegions.Count; second++)
            for (int first = 0; first < second; first++)
            {
                MixedRegion a = _mixedRegions[first], b = _mixedRegions[second];
                if (a.Group != b.Group || FindMixedRegion(first) == FindMixedRegion(second)) continue;
                // One exact plane cannot mix horizontal caps with side faces,
                // but it can meet another prism only along its vertical edge.
                bool connected = a.Face < 0
                    ? b.Face < 0 ? MixedEdgeRegionsTouch(a, b) : MixedFaceEdgeRegionsTouch(b, a)
                    : b.Face < 0 ? MixedFaceEdgeRegionsTouch(a, b) : MixedRegionsTouch(a, b);
                if (!connected) continue;
                _mixedRegionParents[FindMixedRegion(second)] = FindMixedRegion(first);
            }
        _mixedRegionResponses.FastClear();
        foreach (MixedGroup response in _mixedGroups.AsReadOnlySpan()) _mixedRegionResponses.Add(response);
        _mixedGroups.FastClear(); _mixedRegionOrdinals.FastClear();
        for (int index = 0; index < _mixedRegions.Count; index++) _mixedRegionOrdinals.Add(-1);
        for (int index = 0; index < _mixedRegions.Count; index++)
        {
            int root = FindMixedRegion(index);
            if (_mixedRegionOrdinals[root] >= 0) continue;
            _mixedRegionOrdinals[root] = _mixedGroups.Count;
            _mixedGroups.Add(_mixedRegionResponses[_mixedRegions[index].Group]);
        }
        for (int index = 0; index < _mixedSamples.Count; index++)
        {
            MixedSample sample = _mixedSamples[index];
            _mixedSamples[index] = new(_mixedRegionOrdinals[FindMixedRegion(sample.Group)],
                sample.Supplier, sample.Point, sample.Contact);
        }
        // Adjacent patches may emit the same final witness. Deduplicate only
        // after exact region ownership is known, so disconnected rims survive.
        for (int index = _mixedSamples.Count - 1; index >= 0; index--)
            for (int previous = 0; previous < index; previous++)
                if (_mixedSamples[index].Group == _mixedSamples[previous].Group
                    && _mixedSamples[index].Point == _mixedSamples[previous].Point)
                { _mixedSamples.RemoveAt(index); break; }
    }

    private int FindMixedRegion(int index)
    {
        while (_mixedRegionParents[index] != index) index = _mixedRegionParents[index];
        return index;
    }

    private bool MixedEdgeRegionsTouch(in MixedRegion a, in MixedRegion b)
    {
        if (a.Member == b.Member && a.EdgeStart == b.EdgeStart && a.EdgeEnd == b.EdgeEnd)
        {
            return TryIntersectMixedIntervals(a.EdgeInterval, b.EdgeInterval, out var clip)
                && IntersectsMixedRegionEdge(a, clip, -1);
        }
        Vector3d a0 = _mixedPrisms[a.Member].Points[a.EdgeStart], a1 = _mixedPrisms[a.Member].Points[a.EdgeEnd],
            b0 = _mixedPrisms[b.Member].Points[b.EdgeStart], b1 = _mixedPrisms[b.Member].Points[b.EdgeEnd];
        if (a0.Y != a1.Y && b0.Y != b1.Y) return MixedVerticalEdgeRegionsTouch(a, b, a0, a1, b0, b1);
        for (int endpoint = 0; endpoint < 2; endpoint++)
        {
            bool hit = _mixedQuaternion
                ? MixedEdgeRegionEndpoint(a, b, _mixedPrisms[a.Member].Frame, _mixedPrisms[b.Member].Frame, endpoint)
                    || MixedEdgeRegionEndpoint(b, a, _mixedPrisms[b.Member].Frame, _mixedPrisms[a.Member].Frame, endpoint)
                : MixedEdgeRegionEndpoint(a, b, _mixedPrisms[a.Member].NativeFrame, _mixedPrisms[b.Member].NativeFrame, endpoint)
                    || MixedEdgeRegionEndpoint(b, a, _mixedPrisms[b.Member].NativeFrame, _mixedPrisms[a.Member].NativeFrame, endpoint);
            if (hit) return true;
        }
        return false;
    }

    private bool MixedVerticalEdgeRegionsTouch(in MixedRegion a, in MixedRegion b,
        Vector3d a0, Vector3d a1, Vector3d b0, Vector3d b1)
    {
        var pointA = new FixedSegment(new(a0.X, Fixed64.Zero, a0.Z), new(a0.X, Fixed64.Zero, a0.Z));
        var pointB = new FixedSegment(new(b0.X, Fixed64.Zero, b0.Z), new(b0.X, Fixed64.Zero, b0.Z));
        Span<ulong> values = stackalloc ulong[52]; Span<int> signs = stackalloc int[4];
        var n = ContactQuadratic.At(values, signs, 0, 13); var d = ContactQuadratic.At(values, signs, 1, 13);
        n.Clear(); d.Set(Signed576.ExtendValue(Signed192.Signed(1)));
        int location = _mixedQuaternion
            ? WideTriangleRelations.LocateFrameEdgePoint(pointA, _mixedPrisms[a.Member].FootFrame, n, d,
                pointB, _mixedPrisms[b.Member].FootFrame)
            : WideTriangleRelations.LocateFrameEdgePoint(pointA, _mixedPrisms[a.Member].NativeFootFrame, n, d,
                pointB, _mixedPrisms[b.Member].NativeFootFrame);
        if (location < 0 || !TryGetMixedHeights(a.Member, b.Member, out _, out _)) return false;
        // Coincident physical footprints put both original vertical segments
        // in A's bounded chart; reuse exact interval mapping before fan admission.
        Fixed64 delta = _mixedLocal[b.Member].MixedSlabCenterY - _mixedLocal[a.Member].MixedSlabCenterY;
        var source = new FixedSegment(new(a0.X, b0.Y + delta, a0.Z), new(a0.X, b1.Y + delta, a0.Z));
        return b.EdgeInterval.TryMapToSegment(source, new(a0, a1), out var mapped)
            && TryIntersectMixedIntervals(a.EdgeInterval, mapped, out var clip)
            && IntersectsMixedRegionEdge(a, clip, b.Member);
    }

    private static bool TryIntersectMixedIntervals(in RationalSegmentInterval a, in RationalSegmentInterval b,
        out RationalSegmentInterval intersection)
    {
        Signed832 one = Signed832.ExtendValue(Signed192.Signed(1));
        Signed832 ln = a.IsClipped ? a.WideLowerNumerator : default, ld = a.IsClipped ? a.WideLowerDenominator : one,
            un = a.IsClipped ? a.WideUpperNumerator : one, ud = a.IsClipped ? a.WideUpperDenominator : one;
        if (b.IsClipped)
        {
            if (RationalSegmentInterval.CompareParameters(ln, ld, b.WideLowerNumerator, b.WideLowerDenominator) < 0)
            { ln = b.WideLowerNumerator; ld = b.WideLowerDenominator; }
            if (RationalSegmentInterval.CompareParameters(un, ud, b.WideUpperNumerator, b.WideUpperDenominator) > 0)
            { un = b.WideUpperNumerator; ud = b.WideUpperDenominator; }
        }
        intersection = default;
        if (RationalSegmentInterval.CompareParameters(ln, ld, un, ud) > 0) return false;
        intersection = new(ln, ld, un, ud); return true;
    }

    private bool MixedFaceEdgeRegionsTouch(in MixedRegion face, in MixedRegion edge)
    {
        Vector3d a = _mixedPrisms[edge.Member].Points[edge.EdgeStart], b = _mixedPrisms[edge.Member].Points[edge.EdgeEnd];
        var point = new FixedSegment(new(a.X, Fixed64.Zero, a.Z), new(a.X, Fixed64.Zero, a.Z));
        var side = new FixedSegment(new(face.Span.Start.X, Fixed64.Zero, face.Span.Start.Y), new(face.Span.End.X, Fixed64.Zero, face.Span.End.Y));
        Span<ulong> values = stackalloc ulong[52]; Span<int> signs = stackalloc int[4];
        var n = ContactQuadratic.At(values, signs, 0, 13); var d = ContactQuadratic.At(values, signs, 1, 13);
        n.Clear(); d.Set(Signed576.ExtendValue(Signed192.Signed(1)));
        int location = _mixedQuaternion
            ? WideTriangleRelations.LocateFrameEdgePoint(point, _mixedPrisms[edge.Member].FootFrame, n, d,
                side, _mixedPrisms[face.Member].FootFrame, face.Span.Interval)
            : WideTriangleRelations.LocateFrameEdgePoint(point, _mixedPrisms[edge.Member].NativeFootFrame, n, d,
                side, _mixedPrisms[face.Member].NativeFootFrame, face.Span.Interval);
        if (location < 0 || !TryGetMixedHeights(edge.Member, face.Member, out _, out _)) return false;
        // Same horizontal-normal group guarantees an increasing vertical edge.
        // Intersect in its bounded local chart, retaining the original parameter.
        Fixed64 delta = _mixedLocal[face.Member].MixedSlabCenterY - _mixedLocal[edge.Member].MixedSlabCenterY;
        Fixed64 lower = FixedMath.Max(a.Y, face.Lower + delta), upper = FixedMath.Min(b.Y, face.Upper + delta);
        if (lower > upper) return false;
        Signed320 denominator = Signed320.ExtendValue(Signed192.Raw(b.Y - a.Y));
        var heights = new RationalSegmentInterval(Signed320.ExtendValue(Signed192.Raw(lower - a.Y)), denominator,
            Signed320.ExtendValue(Signed192.Raw(upper - a.Y)), denominator);
        return TryIntersectMixedIntervals(edge.EdgeInterval, heights, out var clip)
            && IntersectsMixedRegionEdge(edge, clip, face.Member);
    }

    private bool MixedEdgeRegionEndpoint<TFrame>(in MixedRegion a, in MixedRegion b,
        in TFrame firstFrame, in TFrame secondFrame, int endpoint) where TFrame : struct, IForwardHullFrame3d
    {
        Signed832 one = Signed832.ExtendValue(Signed192.Signed(1)), value = endpoint == 0 ? default : one;
        Span<ulong> values = stackalloc ulong[52]; Span<int> signs = stackalloc int[4];
        var n = ContactQuadratic.At(values, signs, 0, 13); var d = ContactQuadratic.At(values, signs, 1, 13);
        n.Set(value); d.Set(one);
        if (a.EdgeInterval.IsClipped && !a.EdgeInterval.Contains(n, d, ReadOnlySpan<ulong>.Empty)) return false;
        Vector3d point = _mixedPrisms[a.Member].Points[endpoint == 0 ? a.EdgeStart : a.EdgeEnd];
        var source = new FixedSegment(point, point);
        var target = new FixedSegment(_mixedPrisms[b.Member].Points[b.EdgeStart], _mixedPrisms[b.Member].Points[b.EdgeEnd]);
        if (WideTriangleRelations.LocateFrameEdgePoint(source, firstFrame, n, d, target, secondFrame, b.EdgeInterval) < 0) return false;
        return IntersectsMixedRegionEdge(a, new(value, one, value, one), b.Member);
    }

    private bool IntersectsMixedRegionEdge(in MixedRegion region, in RationalSegmentInterval clip, int other)
    {
        int saved = _mixedMember; _mixedMember = region.Member;
        MixedPrism prism = _mixedPrisms[region.Member];
        bool intersects = (_mixedQuaternion
            ? VisitMixedEdgeInterval(prism.Frame, prism, region.EdgeStart, region.EdgeEnd, clip, _mixedGroups[region.Group].Penetration, true)
            : VisitMixedEdgeInterval(prism.NativeFrame, prism, region.EdgeStart, region.EdgeEnd, clip, _mixedGroups[region.Group].Penetration, true)) > 0;
        // Fan traversal proves the shared endpoint belongs to the declared
        // continuity; coincident but undeclared/ambiguous rims stay independent.
        intersects &= other < 0 || _mixedFan.Contains(other);
        _mixedMember = saved;
        return intersects;
    }

    private MeshSurfaceBoundaryDomain2D MixedRegionBoundary(in MixedRegion region)
    {
        MixedPrism prism = _mixedPrisms[region.Member];
        int band = region.Band >= 0 ? region.Band : region.Face == 0 ? 0 : prism.BandCount - 1;
        return prism.Bands[band].Boundary;
    }

    private bool MixedRegionsTouch(in MixedRegion a, in MixedRegion b)
    {
        Fixed64 lower = a.Lower, upper = a.Upper;
        if (a.Member != b.Member)
        {
            if (!TryGetMixedHeights(a.Member, b.Member, out _, out _)) return false;
            Fixed64 delta = _mixedLocal[b.Member].MixedSlabCenterY - _mixedLocal[a.Member].MixedSlabCenterY;
            lower = FixedMath.Max(lower, b.Lower + delta); upper = FixedMath.Min(upper, b.Upper + delta);
            if (lower > upper) return false;
            bool joined = false;
            foreach (var span in MixedRegionBoundary(a).Spans)
                joined |= span.Kind == MeshSurfaceBoundaryDomain2D.SpanKind.Joined && span.OtherMember == b.Member;
            if (!joined) return false;
        }
        else { lower = FixedMath.Max(lower, b.Lower); upper = FixedMath.Min(upper, b.Upper); }
        if (lower > upper) return false;
        if (a.Face < 2)
        {
            foreach (var span in MixedRegionBoundary(a).Spans)
                if (span.Kind == MeshSurfaceBoundaryDomain2D.SpanKind.Joined && span.OtherMember == b.Member)
                {
                    var segment = new FixedSegment(new(span.Start.X, lower, span.Start.Y), new(span.End.X, lower, span.End.Y));
                    MixedPrism prism = _mixedPrisms[a.Member];
                    // Strict convex admission makes the next original vertex
                    // noncollinear with this original consecutive boundary edge.
                    Vector2d third = prism.Offsets[(span.Second + 1) % prism.Offsets.Length];
                    var triangle = new FixedTriangle(segment.Start, segment.End, new(third.X, lower, third.Y));
                    bool vertex = _mixedQuaternion ? MixedCapSupportVertex(prism.Frame, triangle, span.Interval, a.Group)
                        : MixedCapSupportVertex(prism.NativeFrame, triangle, span.Interval, a.Group);
                    if (vertex || IntersectsMixedRegionSegment(a.Member, segment, span.Interval, a.Group)) return true;
                }
            return false;
        }
        return _mixedQuaternion
            ? MixedSideRegionsTouch(a, b, _mixedPrisms[a.Member].Frame, _mixedPrisms[b.Member].Frame, lower)
            : MixedSideRegionsTouch(a, b, _mixedPrisms[a.Member].NativeFrame, _mixedPrisms[b.Member].NativeFrame, lower);
    }

    private bool MixedCapSupportVertex<TFrame>(in TFrame frame, in FixedTriangle triangle,
        in RationalSegmentInterval clip, int group) where TFrame : struct, IForwardHullFrame3d
    {
        Span<Signed832> halfPlanes = stackalloc Signed832[3];
        WideDirection3d axis = _mixedGroups[group].Axis;
        Signed832 support = WideOrientedBox.GetScaledHullSupport(_mixedMovingFrame, MixedMovingPoints, axis, true);
        Span<ulong> values = stackalloc ulong[4 * 14]; Span<int> signs = stackalloc int[4];
        var n = ContactQuadratic.At(values, signs, 0, 14); var d = ContactQuadratic.At(values, signs, 1, 14);
        foreach (Vector3d vertex in MixedMovingPoints)
        {
            RationalPoint3d point = _mixedMovingFrame.GetPoint(vertex);
            if (!WideOrientedBox.IsScaledHullSupport(point, axis, support)) continue;
            frame.TryGetTriangleHalfPlanes(triangle, point, halfPlanes);
            if (!halfPlanes[0].IsZero || halfPlanes[1].Sign < 0 || halfPlanes[2].Sign < 0) continue;
            // On AB, H0=0; u=H2/(H1+H2). A 14-word borrowed bank
            // retains the carry when adding full 832-bit finite weights.
            n.Set(halfPlanes[2]); d.Set(halfPlanes[1]); d.Add(n);
            if (!clip.IsClipped || clip.Contains(n, d, ReadOnlySpan<ulong>.Empty)) return true;
        }
        return false;
    }

    private static FixedTriangle MixedRegionRectangle(in MixedRegion region) => new(
        new Vector3d(region.Span.Start.X, region.Lower, region.Span.Start.Y),
        new(region.Span.End.X, region.Lower, region.Span.End.Y),
        new(region.Span.Start.X, region.Upper, region.Span.Start.Y));

    private bool MixedSideRegionsTouch<TFrame>(in MixedRegion a, in MixedRegion b,
        in TFrame firstFrame, in TFrame secondFrame, Fixed64 lower)
        where TFrame : struct, IForwardHullFrame3d
    {
        FixedTriangle firstRectangle = MixedRegionRectangle(a), secondRectangle = MixedRegionRectangle(b);
        MixedGroup response = _mixedGroups[a.Group]; WideDirection3d axis = response.Axis;
        Signed832 support = WideOrientedBox.GetScaledHullSupport(_mixedMovingFrame, MixedMovingPoints, axis, true);
        const int words = 32;
        Span<ulong> values = stackalloc ulong[16 * words]; Span<int> signs = stackalloc int[16];
        var an = ContactQuadratic.At(values, signs, 0, words); var ad = ContactQuadratic.At(values, signs, 1, words);
        var ax = ContactQuadratic.At(values, signs, 2, words); var ay = ContactQuadratic.At(values, signs, 3, words);
        var bn = ContactQuadratic.At(values, signs, 4, words); var bd = ContactQuadratic.At(values, signs, 5, words);
        var bx = ContactQuadratic.At(values, signs, 6, words); var by = ContactQuadratic.At(values, signs, 7, words);
        // Clip original support edges independently, then intersect their exact
        // parameter windows. Retained cuts can exceed the compact 832-bit type.
        ReadOnlySpan<Vector3d> points = MixedMovingPoints; ReadOnlySpan<int> edges = MixedMovingEdges;
        foreach (Vector3d vertex in points)
        {
            RationalPoint3d point = _mixedMovingFrame.GetPoint(vertex);
            if (WideOrientedBox.IsScaledHullSupport(point, axis, support)
                && WideTriangleRelations.TryGetProjectedRectangleInterval(firstRectangle, firstFrame, point, point, a.Span.Interval, an, ad, ax, ay)
                && WideTriangleRelations.TryGetProjectedRectangleInterval(secondRectangle, secondFrame, point, point, b.Span.Interval, bn, bd, bx, by)) return true;
        }
        for (int edge = 0; edge < edges.Length; edge += 2)
        {
            RationalPoint3d start = _mixedMovingFrame.GetPoint(points[edges[edge]]), end = _mixedMovingFrame.GetPoint(points[edges[edge + 1]]);
            if (!WideOrientedBox.IsScaledHullSupport(start, axis, support) || !WideOrientedBox.IsScaledHullSupport(end, axis, support)
                || !WideTriangleRelations.TryGetProjectedRectangleInterval(firstRectangle, firstFrame, start, end, a.Span.Interval, an, ad, ax, ay)
                || !WideTriangleRelations.TryGetProjectedRectangleInterval(secondRectangle, secondFrame, start, end, b.Span.Interval, bn, bd, bx, by)) continue;
            if (ContactQuadratic.CompareRatios(an, ad, ReadOnlySpan<ulong>.Empty, bx, by, ReadOnlySpan<ulong>.Empty) <= 0
                && ContactQuadratic.CompareRatios(bn, bd, ReadOnlySpan<ulong>.Empty, ax, ay, ReadOnlySpan<ulong>.Empty) <= 0) return true;
        }
        // With no support vertex/boundary crossing, a nonempty common
        // rectangle lies wholly inside the convex support. Its lower edge is
        // sufficient; both charts preserve endpoints of nested retained cuts.
        var segment = new FixedSegment(new(a.Span.Start.X, lower, a.Span.Start.Y), new(a.Span.End.X, lower, a.Span.End.Y));
        if (IntersectsMixedRegionSegment(a.Member, segment, a.Span.Interval, a.Group, b.Member, b.Span)) return true;
        Fixed64 delta = _mixedLocal[a.Member].MixedSlabCenterY - _mixedLocal[b.Member].MixedSlabCenterY;
        lower += delta;
        segment = new(new(b.Span.Start.X, lower, b.Span.Start.Y), new(b.Span.End.X, lower, b.Span.End.Y));
        return IntersectsMixedRegionSegment(b.Member, segment, b.Span.Interval, a.Group, a.Member, a.Span);
    }

    private bool IntersectsMixedRegionSegment(int member, in FixedSegment segment,
        in RationalSegmentInterval clip, int group, int other = -1,
        MeshSurfaceBoundaryDomain2D.BoundarySpan otherSpan = default)
    {
        _mixedRegionSupport.Owner = this; _mixedRegionSupport.Member = member; _mixedRegionSupport.Other = other;
        _mixedRegionSupport.Segment = segment; _mixedRegionSupport.OtherSpan = otherSpan; _mixedRegionSupport.Intervals[0] = clip;
        Span<Vector3d> points = stackalloc Vector3d[2] { segment.Start, segment.End };
        ReadOnlySpan<int> edges = stackalloc int[2] { 0, 1 };
        MixedPrism prism = _mixedPrisms[member];
        return _mixedQuaternion ? VisitMixedRegionSegment(prism.Frame, member, points, edges, group)
            : VisitMixedRegionSegment(prism.NativeFrame, member, points, edges, group);
    }

    private bool VisitMixedRegionSegment<TFrame>(in TFrame frame, int member, ReadOnlySpan<Vector3d> points,
        ReadOnlySpan<int> edges, int group) where TFrame : struct, IForwardHullFrame3d =>
        WideOrientedBox.VisitConvexHullContacts(_mixedMovingFrame, _mixedMoving!.Center, MixedMovingPoints,
            MixedMovingTriangles, MixedMovingEdges, frame, MixedEmbedded2DGeometry.GetCenter3D(_mixedLocal[member]),
            points, ReadOnlySpan<int>.Empty, edges, _mixedGroups[group].Penetration,
            this, static (_, _) => { }, _mixedRegionSupport, admissionOnly: true) > 0;

    private readonly struct MixedRegion
    {
        internal readonly int Group, Member, Face, Band, EdgeStart, EdgeEnd;
        internal readonly RationalSegmentInterval EdgeInterval;
        internal readonly MeshSurfaceBoundaryDomain2D.BoundarySpan Span;
        internal readonly Fixed64 Lower, Upper;
        internal MixedRegion(int group, int member, int face, int band, in MeshSurfaceBoundaryDomain2D.BoundarySpan span,
            Fixed64 lower, Fixed64 upper, int edgeStart, int edgeEnd, in RationalSegmentInterval edgeInterval)
        { Group = group; Member = member; Face = face; Band = band; Span = span; Lower = lower; Upper = upper;
            EdgeStart = edgeStart; EdgeEnd = edgeEnd; EdgeInterval = edgeInterval; }
    }

    private sealed class MixedRegionSupport : IScaledConvexHullContactDomain
    {
        internal StaticSurface2D? Owner;
        internal int Member, Other;
        internal FixedSegment Segment;
        internal MeshSurfaceBoundaryDomain2D.BoundarySpan OtherSpan;
        internal readonly RationalSegmentInterval[] Intervals = new RationalSegmentInterval[1];
        public ReadOnlySpan<RationalSegmentInterval> GetEdgeIntervals(bool first, int start, int end, in WideDirection3d normal)
            => first ? MixedFullInterval : Intervals;
        public bool ContainsPoint(bool first, int start, int end, in Signed832 n, in Signed832 d, in WideDirection3d normal)
        {
            System.Diagnostics.Debug.Assert(!first);
            Span<ulong> values = stackalloc ulong[52]; Span<int> signs = stackalloc int[4];
            var numerator = ContactQuadratic.At(values, signs, 0, 13); var denominator = ContactQuadratic.At(values, signs, 1, 13);
            numerator.Set(n); denominator.Set(d);
            return ContainsPoint(false, start, end, numerator, denominator, normal);
        }
        public bool ContainsPoint(bool first, int start, int end, scoped ContactQuadratic n,
            scoped ContactQuadratic d, in WideDirection3d normal)
        {
            if (first) return true;
            if (start == end)
            {
                Span<ulong> values = stackalloc ulong[52]; Span<int> signs = stackalloc int[4];
                var endpoint = ContactQuadratic.At(values, signs, 0, 13); var one = ContactQuadratic.At(values, signs, 1, 13);
                endpoint.Set(Signed576.ExtendValue(Signed192.Signed(start))); one.Set(Signed576.ExtendValue(Signed192.Signed(1)));
                return ContainsParameter(endpoint, one);
            }
            return ContainsParameter(n, d);
        }
        private bool ContainsParameter(scoped ContactQuadratic n, scoped ContactQuadratic d)
        {
            if (Intervals[0].IsClipped && !Intervals[0].Contains(n, d, ReadOnlySpan<ulong>.Empty)) return false;
            if (Other < 0) return true;
            StaticSurface2D owner = Owner!;
            FixedSegment source = new(new(Segment.Start.X, Fixed64.Zero, Segment.Start.Z), new(Segment.End.X, Fixed64.Zero, Segment.End.Z));
            FixedSegment target = new(new(OtherSpan.Start.X, Fixed64.Zero, OtherSpan.Start.Y), new(OtherSpan.End.X, Fixed64.Zero, OtherSpan.End.Y));
            return owner._mixedQuaternion
                ? WideTriangleRelations.LocateFrameEdgePoint(source, owner._mixedPrisms[Member].FootFrame, n, d,
                    target, owner._mixedPrisms[Other].FootFrame, OtherSpan.Interval) >= 0
                : WideTriangleRelations.LocateFrameEdgePoint(source, owner._mixedPrisms[Member].NativeFootFrame, n, d,
                    target, owner._mixedPrisms[Other].NativeFootFrame, OtherSpan.Interval) >= 0;
        }
    }
}
