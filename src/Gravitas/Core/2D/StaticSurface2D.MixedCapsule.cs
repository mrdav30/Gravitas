// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
using System;
using FixedMathSharp;
using FixedMathSharp.Geometry;
using Gravitas.Colliders;
using Gravitas.CollisionHandling;
using SwiftCollections;

namespace Gravitas;

/// <content>Retains finite capsule support intervals on original native prism faces.</content>
internal sealed partial class StaticSurface2D : IPrismCapsuleContactDomain
{
    private readonly PrismPointAdmission _mixedCapsuleAdmit;
    private readonly SwiftList<RationalSegmentInterval> _mixedCapsuleCore = new();
    private readonly SwiftList<bool> _mixedCapsuleSelected = new(), _mixedCapsuleUseCuts = new();
    private readonly SwiftList<MixedCurvedFaceWindow> _mixedCapsuleParents = new();
    private bool _mixedCapsuleChoiceMade;
    private bool _mixedCapsuleRecovery;
    bool IPrismCapsuleContactDomain.IncludeRecovery => _mixedCapsuleRecovery;

    private void BuildMixedCapsule()
    {
        _mixedSphereGroups.FastClear(); _mixedSamples.FastClear(); _mixedRegionParents.FastClear();
        _mixedCapsuleCore.FastClear(); _mixedCapsuleSelected.FastClear(); _mixedCapsuleUseCuts.FastClear();
        _mixedCurvedBoundarySamples.FastClear(); _mixedOverlaps.Clear(); _mixedSpherePhysicalComponents.Clear();
        for (int member = 0; member < _mixedLocal.Count; member++) _mixedCapsuleUseCuts.Add(false);
        var capsule = (LSCapsuleCollider)_mixedMoving!;
        // Prepare topology before FMS borrows support intervals during callbacks.
        if (!_mixedBoundaryLinesCurrent) PrepareMixedBoundaryLines();
        _mixedCapsuleRecovery = false;
        for (_mixedMember = 0; _mixedMember < _mixedLocal.Count; _mixedMember++)
        {
            LSCollider2D member = _mixedLocal[_mixedMember];
            if (!capsule.Bounds.Intersects(member.MixedBounds3D)) continue;
            MixedPrism prism = _mixedPrisms[_mixedMember];
            if (!prism.Joined)
            {
                if (FixedConvexPrismRelations.TryGetCenteredCapsuleContact(capsule.Center, capsule.Rotation, Vector3d.Up,
                    capsule.AxisLength, capsule.ScaledRadius, MixedEmbedded2DGeometry.GetCenter3D(member),
                    member.ConvexRotation, prism.Offsets, member.MixedHalfThickness, out var separate))
                    RequestMixedReach(separate.Depth);
                if (_mixedRequestedReach > _mixedReach) return;
                continue;
            }
            VisitMember(member, prism);
            if (_mixedRequestedReach > _mixedReach) return;
        }
        AppendMixedCapsuleBoundaryPeaks(capsule);
        // Admitted geometry still connects finite patches, but only the chosen
        // source alternative contributes solver samples or blocks far recovery.
        for (int sample = _mixedSamples.Count - 1; sample >= 0; sample--)
            if (!_mixedCapsuleSelected[_mixedSamples[sample].Group]) _mixedSamples.RemoveAt(sample);
            else _mixedSpherePhysicalComponents.Add(_mixedSphereGroups[_mixedSamples[sample].Group].Component);
        // Genuine local boundary constraints take precedence over a far exit
        // through the solid. Only wholly embedded components need that recovery search.
        _mixedCapsuleRecovery = true;
        for (_mixedMember = 0; _mixedMember < _mixedLocal.Count; _mixedMember++)
        {
            LSCollider2D member = _mixedLocal[_mixedMember]; MixedPrism prism = _mixedPrisms[_mixedMember];
            if (!prism.Joined || !_mixedOverlaps.Contains(member.Id)
                || _mixedSpherePhysicalComponents.Contains(_mixedComponents[_mixedMember])) continue;
            VisitMember(member, prism);
            if (_mixedRequestedReach > _mixedReach) return;
        }

        void VisitMember(LSCollider2D member, MixedPrism prism)
        {
            _mixedCapsuleChoiceMade = _mixedCapsuleRecovery || _mixedBoundaryLines.Count == 0;
            WideConvexPrismRelations.VisitCapsuleContacts(capsule.Center, capsule.Rotation, capsule.AxisLength, capsule.ScaledRadius,
                MixedEmbedded2DGeometry.GetCenter3D(member), member.ConvexRotation, prism.Offsets, member.MixedHalfThickness,
                _mixedCapsuleAdmit, this, out bool overlaps);
            if (overlaps) _mixedOverlaps.Add(member.Id);
            if (overlaps && !_mixedCapsuleChoiceMade)
                PrepareMixedCapsuleChoice(default, hasNative: false, hasParent: false);
        }
    }

    private void PrepareMixedCapsuleChoice(scoped in CapsulePrismContactDepth native, bool hasNative, bool hasParent)
    {
        _mixedCapsuleChoiceMade = true;
        var capsule = (LSCapsuleCollider)_mixedMoving!;
        var member = _mixedLocal[_mixedMember]; var prism = _mixedPrisms[_mixedMember];
        int start = _mixedCurvedBoundarySamples.Count;
        for (int ordinal = 0; ordinal < _mixedBoundaryLines.Count; ordinal++)
        {
            MixedBoundaryLine line = _mixedBoundaryLines[ordinal];
            if (line.Member != _mixedMember) continue;
            int face = line.Vertical == 0 ? line.Edge : line.Vertical > 0 ? -1 : -2;
            var numeric = new PrismCurvedBoundaryLine(face, line.Clip, line.Lower, line.Upper);
            _mixedCurvedActiveLines.FastClear(); _mixedCurvedActiveLines.Add(numeric);
            if (hasParent) RemoveMixedCapsuleParentWindows(face);
            bool unchanged = _mixedCurvedActiveLines.Count == 1
                && _mixedCurvedActiveLines[0].Lower == line.Lower && _mixedCurvedActiveLines[0].Upper == line.Upper
                && SameMixedBoundaryInterval(_mixedCurvedActiveLines[0].Clip, line.Clip);
            _mixedCapsuleBoundary ??= new();
            if (!_mixedCapsuleBoundary.TryGetContact(capsule.Center, capsule.Rotation, capsule.AxisLength, capsule.ScaledRadius,
                MixedEmbedded2DGeometry.GetCenter3D(member), member.ConvexRotation, prism.Offsets[line.Edge],
                prism.Offsets[(line.Edge + 1) % prism.Offsets.Length], prism.Offsets[(line.Edge + 2) % prism.Offsets.Length],
                line.Clip, line.Lower, line.Upper, line.Vertical, out var contact)) continue;
            int comparison = hasNative && unchanged ? _mixedCapsuleBoundary.CompareDepth(native) : -1;
            _mixedCurvedBoundarySamples.Add(new(ordinal, contact, comparison, !unchanged));
            if (unchanged) continue;
            // The complete line still connects regions. Rank only its pieces
            // outside admitted parent faces, not a maximum attained inside one.
            foreach (var cut in _mixedCurvedActiveLines.AsReadOnlySpan())
                if (_mixedCapsuleBoundary.TryGetContact(capsule.Center, capsule.Rotation, capsule.AxisLength, capsule.ScaledRadius,
                    MixedEmbedded2DGeometry.GetCenter3D(member), member.ConvexRotation, prism.Offsets[line.Edge],
                    prism.Offsets[(line.Edge + 1) % prism.Offsets.Length], prism.Offsets[(line.Edge + 2) % prism.Offsets.Length],
                    cut.Clip, cut.Lower, cut.Upper, line.Vertical, out contact))
                    _mixedCurvedBoundarySamples.Add(new(ordinal, contact, _mixedCapsuleBoundary.CompareDepth(native)));
        }
        var cuts = _mixedCurvedBoundarySamples.AsReadOnlySpan()[start..];
        bool useCuts = false;
        // A finite cut family needs every independent row. Its exact maximum
        // competes with the native exit; ties preserve that native alternative.
        foreach (var cut in cuts)
        {
            if (cut.GeometryOnly) continue;
            if (cut.CompareToNative >= 0) { useCuts = false; break; }
            useCuts = true;
        }
        _mixedCapsuleUseCuts[_mixedMember] = useCuts && AreMixedCurvedCutRowsCompatible(cuts);
    }

    private void RemoveMixedCapsuleParentWindows(int face)
    {
        foreach (var parent in _mixedCapsuleParents.AsReadOnlySpan())
        {
            if (parent.Face != face) continue;
            for (int index = _mixedCurvedActiveLines.Count - 1; index >= 0; index--)
            {
                var line = _mixedCurvedActiveLines[index];
                Fixed64 lower = FixedMath.Max(line.Lower, parent.Lower), upper = FixedMath.Min(line.Upper, parent.Upper);
                if (lower > upper || !TryIntersectMixedIntervals(line.Clip, parent.Interval, out var overlap)) continue;
                _mixedCurvedActiveLines.RemoveAt(index);
                if (line.Lower != line.Upper)
                {
                    if (line.Lower < lower) _mixedCurvedActiveLines.Add(new(face, line.Clip, line.Lower, lower));
                    if (upper < line.Upper) _mixedCurvedActiveLines.Add(new(face, line.Clip, upper, line.Upper));
                }
                else
                {
                    GetMixedBoundaryBounds(line.Clip, out var ln, out var ld, out var un, out var ud);
                    GetMixedBoundaryBounds(overlap, out var oln, out var old, out var oun, out var oud);
                    if (RationalSegmentInterval.CompareParameters(ln, ld, oln, old) < 0)
                        _mixedCurvedActiveLines.Add(new(face, new(ln, ld, oln, old), line.Lower, line.Upper));
                    if (RationalSegmentInterval.CompareParameters(oun, oud, un, ud) < 0)
                        _mixedCurvedActiveLines.Add(new(face, new(oun, oud, un, ud), line.Lower, line.Upper));
                }
            }
        }
    }

    ReadOnlySpan<RationalSegmentInterval> IPrismCapsuleContactDomain.GetSupportIntervals(in WideDirection3d normal,
        ReadOnlySpan<Signed832> projected, Signed832 denominator, in RationalSegmentInterval solidClip)
    {
        _mixedIntervals.FastClear();
        _mixedCapsuleParents.FastClear();
        WideDirection3d supportNormal = normal;
        MixedPrism prism = _mixedPrisms[_mixedMember];
        LSCollider2D member = _mixedLocal[_mixedMember];
        for (int bandIndex = 0; bandIndex < prism.BandCount; bandIndex++)
        {
            MixedBand band = prism.Bands[bandIndex];
            foreach (var span in band.Boundary.Spans)
                if (span.Kind != MeshSurfaceBoundaryDomain2D.SpanKind.Joined
                    && WideConvexPrismRelations.TryClipProjectedPrismFace(projected, denominator, member.ConvexRotation,
                        span.Start, span.End, span.Interval, band.Lower, band.Upper, solidClip, out var retained))
                { Keep(retained); KeepParent(span.First, span.Interval, band.Lower, band.Upper); }
        }
        for (int cap = 0; cap < 2; cap++)
        {
            Fixed64 height = cap == 0 ? -member.MixedHalfThickness : member.MixedHalfThickness;
            if (WideConvexPrismRelations.TryClipProjectedPrismFace(projected, denominator, member.ConvexRotation,
                default, default, default, height, height, solidClip, out var retained, cap: true))
            {
                bool onEdge = false;
                for (int edge = 0; edge < prism.Offsets.Length; edge++)
                {
                    Vector2d a = prism.Offsets[edge], b = prism.Offsets[(edge + 1) % prism.Offsets.Length];
                    if (!WideConvexPrismRelations.TryClipProjectedPrismFace(projected, denominator, member.ConvexRotation,
                        a, b, default, height, height, retained, out var edgeClip)
                        || !SameMixedCapsuleIntervalWide(edgeClip, retained)) continue;
                    // A cap support along its footprint rim may cross a taller
                    // neighbor. Retain its exact fan windows, not one midpoint.
                    foreach (var span in prism.Bands[cap == 0 ? 0 : prism.BandCount - 1].Boundary.Spans)
                        if (span.First == edge && WideConvexPrismRelations.TryClipProjectedPrismFace(projected, denominator,
                            member.ConvexRotation, a, b, span.Interval, height, height, retained, out var window)) Keep(window);
                    onEdge = true; break;
                }
                if (!onEdge) Keep(retained);
                KeepParent(cap == 0 ? -1 : -2, default, height, height);
            }
        }
        return _mixedIntervals.AsReadOnlySpan();

        void Keep(in RationalSegmentInterval interval)
        {
            foreach (var previous in _mixedIntervals)
                if (SameMixedCapsuleIntervalWide(previous, interval)) return;
            _mixedIntervals.Add(interval);
        }
        void KeepParent(int face, in RationalSegmentInterval interval, Fixed64 lower, Fixed64 upper)
        {
            GetMixedCurvedPlane(_mixedMember, face, out var axis, out _);
            if (WideDirection3d.AreParallel(supportNormal, axis) && WideDirection3d.Dot(supportNormal, axis).Sign > 0)
                _mixedCapsuleParents.Add(new(face, interval, lower, upper));
        }
    }

    private static bool SameMixedCapsuleIntervalWide(in RationalSegmentInterval a, in RationalSegmentInterval b)
    {
        Signed832 one = Signed832.ExtendValue(Signed192.Signed(1));
        return RationalSegmentInterval.CompareParameters(a.IsClipped ? a.WideLowerNumerator : default,
            a.IsClipped ? a.WideLowerDenominator : one, b.IsClipped ? b.WideLowerNumerator : default,
            b.IsClipped ? b.WideLowerDenominator : one) == 0
            && RationalSegmentInterval.CompareParameters(a.IsClipped ? a.WideUpperNumerator : one,
                a.IsClipped ? a.WideUpperDenominator : one, b.IsClipped ? b.WideUpperNumerator : one,
                b.IsClipped ? b.WideUpperDenominator : one) == 0;
    }

    void IPrismCapsuleContactDomain.AddSupportContact(in FixedContactAnchors contact, scoped in CapsulePrismContactDepth native, in WideDirection3d normal,
        in WideConvexPrismRelations.ProjectionDepth depth, bool physical, int interval)
    {
        if (!_mixedCapsuleChoiceMade) PrepareMixedCapsuleChoice(native, hasNative: true, hasParent: true);
        int component = _mixedComponents[_mixedMember];
        RationalPoint3d reference = MixedSupportPoint(_mixedPrisms[_mixedMember].NativeFrame, _mixedPrisms[_mixedMember], normal);
        int group = AddMixedCapsuleSample(new(component, _mixedLocal[_mixedMember].Id, _mixedMember,
            normal, reference, depth, contact, !physical), _mixedIntervals[interval],
            _mixedCapsuleRecovery || !_mixedCapsuleUseCuts[_mixedMember]);
        if (physical)
            // A common component/plane alone does not declare a local join:
            // an ambiguous cap can connect to the same component elsewhere.
            for (int previous = 0; previous < group; previous++)
                if (SameMixedCapsulePlane(group, previous)
                    && (_mixedSphereGroups[previous].Supplier == _mixedMember || _mixedFan.Contains(_mixedSphereGroups[previous].Supplier))
                    && TryIntersectMixedIntervals(_mixedCapsuleCore[group], _mixedCapsuleCore[previous], out _))
                    _mixedRegionParents[FindMixedRegion(group)] = FindMixedRegion(previous);
        RequestMixedReach(contact.Depth);
    }

    private int AddMixedCapsuleSample(in SphereGroup row, in RationalSegmentInterval core, bool selected = true)
    {
        int group = _mixedSphereGroups.Count;
        _mixedSphereGroups.Add(row); _mixedCapsuleCore.Add(core); _mixedRegionParents.Add(group);
        _mixedCapsuleSelected.Add(selected);
        bool represented = row.Contact.SecondAnchor.TryGetPointInFrame(_mixedMoving!.Center, out Vector3d point);
        System.Diagnostics.Debug.Assert(represented);
        _mixedSamples.Add(new(group, row.Supplier, point, row.Contact));
        return group;
    }

    private bool SameMixedCapsulePlane(int a, int b)
    {
        SphereGroup first = _mixedSphereGroups[a], second = _mixedSphereGroups[b];
        return first.Component == second.Component && WideDirection3d.AreParallel(first.Axis, second.Axis)
            && WideDirection3d.Dot(first.Axis, second.Axis).Sign > 0
            && WideTriangleRelations.DotScaledDirections(first.Axis, WideTriangleRelations.GetScaledPointDifference(
                first.Reference, second.Reference, Signed192.Signed(1))).IsZero;
    }

    ReadOnlySpan<RationalSegmentInterval> IPrismCapsuleContactDomain.GetEdgeIntervals(int vertex, int kind)
    {
        GetMixedCapsuleEdge(vertex, kind, out int start, out int end);
        GetMixedEdgeIntervals(start, end);
        // Several height bands can map the same original cap edge window.
        for (int index = _mixedIntervals.Count - 1; index >= 0; index--)
            for (int previous = 0; previous < index; previous++)
                if (SameMixedCapsuleIntervalWide(_mixedIntervals[index], _mixedIntervals[previous]))
                { _mixedIntervals.RemoveAt(index); break; }
        return _mixedIntervals.AsReadOnlySpan();
    }

    bool IPrismCapsuleContactDomain.AdmitEdgePoint(int vertex, int kind, in Signed832 edgeN,
        in Signed832 edgeD, in WideDirection3d normal)
    {
        GetMixedCapsuleEdge(vertex, kind, out int start, out int end);
        MixedPrism prism = _mixedPrisms[_mixedMember];
        Vector3d a = prism.Points[start], b = prism.Points[end];
        const int words = 32;
        Span<ulong> values = stackalloc ulong[12 * words]; Span<int> signs = stackalloc int[12];
        var n = ContactQuadratic.At(values, signs, 0, words); var d = ContactQuadratic.At(values, signs, 1, words);
        var y = ContactQuadratic.At(values, signs, 2, words); var yd = ContactQuadratic.At(values, signs, 3, words);
        var term = ContactQuadratic.At(values, signs, 4, words);
        n.Set(edgeN); d.Set(edgeD);
        ContactQuadratic.Scale(d, Signed576.ExtendValue(Signed192.Raw(a.Y)), y);
        ContactQuadratic.Scale(n, Signed576.ExtendValue(Signed192.Raw(b.Y - a.Y)), term); y.Add(term);
        d.CopyTo(yd);
        if (kind == 2) { n.Clear(); d.Set(Signed576.ExtendValue(Signed192.Signed(1))); }
        return AdmitMixedSphereFan(new(new(a.X, Fixed64.Zero, a.Z), new(b.X, Fixed64.Zero, b.Z)), n, d, y, yd, normal);
    }

    void IPrismCapsuleContactDomain.AddEdgeContact(in FixedContactAnchors contact, scoped in CapsulePrismContactDepth native, in WideDirection3d normal,
        int vertex, int kind, int interval)
    {
        // Every admitted edge pair supports the entire convex prism and core,
        // so distinct callbacks have the same certified closest-pair depth.
        if (!_mixedCapsuleChoiceMade) PrepareMixedCapsuleChoice(native, hasNative: true, hasParent: false);
        MixedPrism prism = _mixedPrisms[_mixedMember]; int component = _mixedComponents[_mixedMember];
        AddMixedCapsuleSample(new(component, _mixedLocal[_mixedMember].Id, _mixedMember, normal,
            MixedSupportPoint(prism.NativeFrame, prism, normal), default, contact), default,
            !_mixedCapsuleUseCuts[_mixedMember]);
        RequestMixedReach(contact.Depth);
    }

    private void GetMixedCapsuleEdge(int vertex, int kind, out int start, out int end)
    {
        int count = _mixedPrisms[_mixedMember].Offsets.Length;
        start = vertex * 2 + (kind == 1 ? 1 : 0);
        end = kind == 2 ? start + 1 : ((vertex + 1) % count) * 2 + (kind == 1 ? 1 : 0);
    }

    private bool AppendMixedCapsule(LSCollider2D overlap, ContactManifold destination, int namespace3D, int namespace2D)
    {
        Span<int> chosen = stackalloc int[ContactManifold.MaxContactsPerGroup];
        SolidBody? body = _mixedMoving!.Body ?? _mixedMoving.CompoundOwner?.Body;
        var center = body == null ? new FixedPointAnchor(_mixedMoving.Center, _mixedMoving.Rotation, Vector3d.Zero)
            : new FixedPointAnchor(body.Position3d, body.Rotation, body.LocalCenterOfMassOffset);
        for (int group = 0; group < _mixedSphereGroups.Count; group++)
        {
            if (FindMixedRegion(group) != group) continue;
            SphereGroup row = _mixedSphereGroups[group];
            if (row.Carrier != overlap.Id) continue;
            if (row.Recovery)
            {
                bool better = false;
                for (int other = 0; other < _mixedSphereGroups.Count; other++)
                    if (_mixedCapsuleSelected[other] && _mixedSphereGroups[other].Component == row.Component)
                    {
                        System.Diagnostics.Debug.Assert(_mixedSphereGroups[other].Recovery);
                        int order = WideConvexPrismRelations.CompareProjectionDepths(_mixedSphereGroups[other].Depth, row.Depth);
                        if (order < 0 || order == 0 && other < group) { better = true; break; }
                    }
                if (better) continue;
            }
            AppendMixedRegionSamples(group, row.Axis, destination, namespace3D, namespace2D, chosen, center);
        }
        return _mixedOverlaps.Contains(overlap.Id);
    }
}
