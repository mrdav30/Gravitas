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
    private bool _mixedCapsuleRecovery;
    bool IPrismCapsuleContactDomain.IncludeRecovery => _mixedCapsuleRecovery;

    private void BuildMixedCapsule()
    {
        _mixedSphereGroups.FastClear(); _mixedSamples.FastClear(); _mixedRegionParents.FastClear();
        _mixedCapsuleCore.FastClear(); _mixedOverlaps.Clear(); _mixedSpherePhysicalComponents.Clear();
        var capsule = (LSCapsuleCollider)_mixedMoving!;
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
                    RequestMixedCapsuleReach(separate.Depth);
                if (_mixedRequestedReach > _mixedReach) return;
                continue;
            }
            VisitMember(member, prism);
            if (_mixedRequestedReach > _mixedReach) return;
        }
        AppendMixedCapsuleBoundaryPeaks(capsule);
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
            WideConvexPrismRelations.VisitCapsuleContacts(capsule.Center, capsule.Rotation, capsule.AxisLength, capsule.ScaledRadius,
                MixedEmbedded2DGeometry.GetCenter3D(member), member.ConvexRotation, prism.Offsets, member.MixedHalfThickness,
                _mixedCapsuleAdmit, this, out bool overlaps);
            if (overlaps) _mixedOverlaps.Add(member.Id);
        }
    }

    ReadOnlySpan<RationalSegmentInterval> IPrismCapsuleContactDomain.GetSupportIntervals(in WideDirection3d normal,
        ReadOnlySpan<Signed832> projected, Signed832 denominator, in RationalSegmentInterval solidClip)
    {
        _mixedIntervals.FastClear();
        MixedPrism prism = _mixedPrisms[_mixedMember];
        LSCollider2D member = _mixedLocal[_mixedMember];
        for (int bandIndex = 0; bandIndex < prism.BandCount; bandIndex++)
        {
            MixedBand band = prism.Bands[bandIndex];
            foreach (var span in band.Boundary.Spans)
                if (span.Kind != MeshSurfaceBoundaryDomain2D.SpanKind.Joined
                    && WideConvexPrismRelations.TryClipProjectedPrismFace(projected, denominator, member.ConvexRotation,
                        span.Start, span.End, span.Interval, band.Lower, band.Upper, solidClip, out var retained)) Keep(retained);
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
            }
        }
        return _mixedIntervals.AsReadOnlySpan();

        void Keep(in RationalSegmentInterval interval)
        {
            foreach (var previous in _mixedIntervals)
                if (SameMixedCapsuleIntervalWide(previous, interval)) return;
            _mixedIntervals.Add(interval);
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

    void IPrismCapsuleContactDomain.AddSupportContact(in FixedContactAnchors contact, in WideDirection3d normal,
        in WideConvexPrismRelations.ProjectionDepth depth, bool physical, int interval)
    {
        int component = _mixedComponents[_mixedMember];
        if (physical) _mixedSpherePhysicalComponents.Add(component);
        RationalPoint3d reference = MixedSupportPoint(_mixedPrisms[_mixedMember].NativeFrame, _mixedPrisms[_mixedMember], normal);
        int group = AddMixedCapsuleSample(new(component, _mixedLocal[_mixedMember].Id, _mixedMember,
            normal, reference, depth, contact, !physical), _mixedIntervals[interval]);
        if (physical)
            // A common component/plane alone does not declare a local join:
            // an ambiguous cap can connect to the same component elsewhere.
            for (int previous = 0; previous < group; previous++)
                if (SameMixedCapsulePlane(group, previous)
                    && (_mixedSphereGroups[previous].Supplier == _mixedMember || _mixedFan.Contains(_mixedSphereGroups[previous].Supplier))
                    && TryIntersectMixedIntervals(_mixedCapsuleCore[group], _mixedCapsuleCore[previous], out _))
                    _mixedRegionParents[FindMixedRegion(group)] = FindMixedRegion(previous);
        RequestMixedCapsuleReach(contact.Depth);
    }

    private void RequestMixedCapsuleReach(Fixed64 depth)
    {
        Fixed64 reach = _mixedMoving!.CanonicalCenteredProxyRadius + depth + Fixed64.MinIncrement + Fixed64.MinIncrement;
        if (reach > _mixedRequestedReach) _mixedRequestedReach = reach;
    }

    private int AddMixedCapsuleSample(in SphereGroup row, in RationalSegmentInterval core)
    {
        int group = _mixedSphereGroups.Count;
        _mixedSphereGroups.Add(row); _mixedCapsuleCore.Add(core); _mixedRegionParents.Add(group);
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

    void IPrismCapsuleContactDomain.AddEdgeContact(in FixedContactAnchors contact, in WideDirection3d normal,
        int vertex, int kind, int interval)
    {
        MixedPrism prism = _mixedPrisms[_mixedMember]; int component = _mixedComponents[_mixedMember];
        _mixedSpherePhysicalComponents.Add(component);
        AddMixedCapsuleSample(new(component, _mixedLocal[_mixedMember].Id, _mixedMember, normal,
            MixedSupportPoint(prism.NativeFrame, prism, normal), default, contact), default);
        RequestMixedCapsuleReach(contact.Depth);
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
                    if (_mixedSphereGroups[other].Component == row.Component)
                    {
                        int order = WideConvexPrismRelations.CompareProjectionDepths(_mixedSphereGroups[other].Depth, row.Depth);
                        if (order < 0 || order == 0 && other < group) { better = true; break; }
                    }
                if (better) continue;
            }
            _mixedPool.FastClear();
            foreach (MixedSample sample in _mixedSamples.AsReadOnlySpan())
                if (FindMixedRegion(sample.Group) == group)
                {
                    bool duplicate = false;
                    foreach (MixedSample kept in _mixedPool.AsReadOnlySpan()) duplicate |= kept.Point == sample.Point;
                    if (!duplicate) _mixedPool.Add(sample);
                }
            _mixedPool.SortInPlace(MixedPointOrder);
            // Keep the deepest, lowest-lever physical point before reducing
            // coverage; the manifold assigns its own canonical solver order.
            int seed = 0;
            for (int index = 1; index < _mixedPool.Count; index++)
            {
                FixedContactAnchors candidate = _mixedPool[index].Contact, current = _mixedPool[seed].Contact;
                int depth = candidate.Depth.CompareTo(current.Depth);
                if (depth > 0 || depth == 0 && center.CompareSquaredDistance(candidate.FirstAnchor, current.FirstAnchor) < 0)
                    seed = index;
            }
            if (seed != 0) (_mixedPool[0], _mixedPool[seed]) = (_mixedPool[seed], _mixedPool[0]);
            int count = Math.Min(chosen.Length, _mixedPool.Count);
            for (int slot = 0; slot < count; slot++)
                chosen[slot] = slot == 0 ? 0 : MeshPlaneSampleReduction.ChooseNext(_mixedPool.AsReadOnlySpan(), chosen[..slot], row.Axis, MixedPointOrder);
            for (int slot = 0; slot < count; slot++)
            {
                MixedSample sample = _mixedPool[chosen[slot]];
                FixedContactAnchors contact = sample.Contact; LSCollider2D supplier = _mixedLocal[sample.Supplier];
                destination.AddContact(new ContactAnchor(contact.FirstAnchor), new ContactAnchor(contact.SecondAnchor),
                    contact.Depth, contact.Normal, _mixedMoving!.Material, supplier.Material, contact.DepthIsClamped,
                    featureNamespaceA: namespace3D, featureNamespaceB: namespace2D,
                    group: new ContactGroupKey(namespace3D, namespace2D, region: group, sourceIdB: supplier.Id + 1));
            }
        }
        return _mixedOverlaps.Contains(overlap.Id);
    }
}
