// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
using System;
using FixedMathSharp;
using FixedMathSharp.Geometry;
using Gravitas.Colliders;
using Gravitas.CollisionHandling;
using SwiftCollections;

namespace Gravitas;

/// <content>Retains native cylinder/cone supports on exposed prism face windows.</content>
internal sealed partial class StaticSurface2D
{
    private ConePrismBoundaryContacts? _mixedConeBoundary;
    private readonly SwiftList<CurvedGroup> _mixedCurvedGroups = new();
    private readonly SwiftList<int> _mixedCurvedCarriers = new();
    private readonly SwiftList<PrismCurvedBoundaryLine> _mixedCurvedActiveLines = new();
    private readonly SwiftList<CurvedBoundarySample> _mixedCurvedBoundarySamples = new();
    private PrismCurvedBoundaryContactVisitor? _mixedCurvedBoundaryVisitor;

    private void BuildMixedCurved()
    {
        _mixedSamples.FastClear(); _mixedCurvedGroups.FastClear(); _mixedRegionParents.FastClear(); _mixedOverlaps.Clear();
        _mixedSpherePhysicalComponents.Clear(); _mixedCurvedCarriers.FastClear();
        for (int member = 0; member < _mixedLocal.Count; member++) _mixedCurvedCarriers.Add(-1);
        LSCollider mover = _mixedMoving!;
        bool cone = mover is LSConeCollider;
        Fixed64 height = cone ? ((LSConeCollider)mover).Height : ((LSCylinderCollider)mover).Height;
        Span<WideDirection3d> vertices = stackalloc WideDirection3d[32];
        // Geometric overlap belongs to each member; a finite support supplier
        // can lie elsewhere in that same declared component.
        for (_mixedMember = 0; _mixedMember < _mixedLocal.Count; _mixedMember++)
        {
            LSCollider2D member = _mixedLocal[_mixedMember]; MixedPrism prism = _mixedPrisms[_mixedMember];
            if (!mover.Bounds.Intersects(member.MixedBounds3D)
                || !WideConvexPrismRelations.IntersectsCurvedPrism(cone, mover.Center, mover.Rotation, Vector3d.Up,
                    height, mover.ScaledRadius, MixedEmbedded2DGeometry.GetCenter3D(member), member.ConvexRotation,
                    prism.Offsets, member.MixedHalfThickness)) continue;
            _mixedOverlaps.Add(member.Id);
            int component = _mixedComponents[_mixedMember], carrier = _mixedCurvedCarriers[component];
            if (carrier < 0 || member.Id < carrier) _mixedCurvedCarriers[component] = member.Id;
        }
        if (!_mixedBoundaryLinesCurrent) PrepareMixedBoundaryLines();
        PrepareMixedCurvedCertificates(cone, height, vertices);
        for (_mixedMember = 0; _mixedMember < _mixedLocal.Count; _mixedMember++)
        {
            if (!_mixedOverlaps.Contains(_mixedLocal[_mixedMember].Id)) continue;
            VisitMixedCurvedMember(cone, height, vertices, farBoundary: false);
            if (_mixedRequestedReach > _mixedReach) return;
        }
        // Far certificates are needed only when no genuine physical boundary
        // exists in a component; ordinary exterior contacts skip that work.
        PrepareMixedCurvedCertificates(cone, height, vertices, farBoundary: true);
        for (_mixedMember = 0; _mixedMember < _mixedLocal.Count; _mixedMember++)
        {
            if (!_mixedPrisms[_mixedMember].Joined || _mixedCurvedCarriers[_mixedComponents[_mixedMember]] < 0
                || _mixedSpherePhysicalComponents.Contains(_mixedComponents[_mixedMember])) continue;
            VisitMixedCurvedMember(cone, height, vertices, farBoundary: true);
        }
        CompleteMixedCurvedRegions(cone, height, vertices);
        RequestMixedCurvedFarReach();
    }

    private void VisitMixedCurvedMember(bool cone, Fixed64 height, Span<WideDirection3d> vertices, bool farBoundary)
    {
        LSCollider mover = _mixedMoving!;
        LSCollider2D member = _mixedLocal[_mixedMember]; MixedPrism prism = _mixedPrisms[_mixedMember];
        Vector3d origin = MixedEmbedded2DGeometry.GetCenter3D(member);
        if (!prism.Joined)
        {
            bool ordinaryContactFound = CurvedPrismContact.TryGetContact(mover.Center, mover.Rotation, Vector3d.Up, height, mover.ScaledRadius,
                origin, member.ConvexRotation, prism.Offsets, member.MixedHalfThickness, cone, out var separate);
            System.Diagnostics.Debug.Assert(ordinaryContactFound);
            RequestMixedReach(separate.Depth);
            return;
        }
        var geometry = new CurvedPrismContactGeometry(mover.Center, mover.Rotation, Vector3d.Up,
            height, mover.ScaledRadius, origin, member.ConvexRotation, prism.Offsets, member.MixedHalfThickness, cone, vertices);
        var prepared = PrepareMixedCurvedDomain(geometry, farBoundary);
        _mixedCurvedActiveLines.FastClear();
        if (!farBoundary)
            foreach (var line in _mixedCurvedDomains[_mixedMember].Lines.AsReadOnlySpan())
            {
                bool parent = false;
                foreach (var proof in _mixedCurvedCertificates.AsReadOnlySpan())
                    if (proof.Physical && proof.Member == _mixedMember && proof.Window.Face == line.Face
                        && ContainsMixedCurvedBoundaryWindow(proof.Window, line)) { parent = true; break; }
                // An own admitted parent MAX dominates its subset line. A
                // borrowed coplanar proof may live in a disconnected window;
                // it remains a native bound, not this line's parent admission.
                if (!parent) _mixedCurvedActiveLines.Add(line);
            }
        ReadOnlySpan<PrismCurvedBoundaryLine> lines = _mixedCurvedActiveLines.AsReadOnlySpan();
        _mixedCurvedBoundarySamples.FastClear(); _mixedCurvedBoundaryVisitor ??= KeepMixedCurvedBoundary;
        if (cone && !lines.IsEmpty) _mixedConeBoundary ??= new();
        var domain = new PrismCurvedContactDomain(prepared.Edges, prepared.FanCovectors, includeFarBoundary: farBoundary, certificates: prepared.Certificates);
        bool found = CurvedPrismContact.TryGetContact(geometry, domain, mover.Center, mover.Rotation, origin,
            out var contact, out int face, out bool physical, out int certificate,
            lines, _mixedCurvedBoundaryVisitor, _mixedConeBoundary);
        // One source has two response alternatives: its whole-prism native exit
        // or its complete independent finite cut set. The latter needs every
        // constraint, so compare the native depth to the set's exact maximum.
        // FMS streams comparisons while its native algebraic selection is alive;
        // these buffers retain only ordinary contact payloads and line ordinals.
        bool useCuts = _mixedCurvedBoundarySamples.Count > 0;
        foreach (var sample in _mixedCurvedBoundarySamples.AsReadOnlySpan())
            if (sample.CompareToNative >= 0) { useCuts = false; break; }
        if (useCuts && !AreMixedCurvedCutRowsCompatible(_mixedCurvedBoundarySamples.AsReadOnlySpan())) useCuts = false;
        if (useCuts)
        {
            foreach (var sample in _mixedCurvedBoundarySamples.AsReadOnlySpan())
            {
                var line = lines[sample.Ordinal];
                AddMixedCurvedSample(sample.Contact, line.Face, true, line.Clip, line.Lower, line.Upper, line.BoundaryEdge);
            }
        }
        else if (found)
        {
            // Decide the source obligation before pooling actual supplier
            // windows. Borrowed suppliers keep their identities even when their
            // own source subsequently selects a different response family.
            if (certificate >= 0) AddMixedCurvedCertificatePlane(_mixedCurvedCertificateSources[certificate], physical);
            else AddMixedCurvedSample(contact, face, physical);
        }
    }

    private static bool ContainsMixedCurvedBoundaryWindow(in MixedCurvedFaceWindow window, in PrismCurvedBoundaryLine line)
    {
        if (window.Lower > line.Lower || window.Upper < line.Upper) return false;
        // A complete original cap contains every original footprint edge.
        if (window.Face < 0 || !window.Interval.IsClipped) return true;
        var interval = line.Clip;
        Signed832 one = Signed832.ExtendValue(Signed192.Signed(1));
        Signed832 ln = interval.IsClipped ? interval.WideLowerNumerator : default,
            ld = interval.IsClipped ? interval.WideLowerDenominator : one,
            un = interval.IsClipped ? interval.WideUpperNumerator : one,
            ud = interval.IsClipped ? interval.WideUpperDenominator : one;
        return RationalSegmentInterval.CompareParameters(ln, ld, window.Interval.WideLowerNumerator, window.Interval.WideLowerDenominator) >= 0
            && RationalSegmentInterval.CompareParameters(un, ud, window.Interval.WideUpperNumerator, window.Interval.WideUpperDenominator) <= 0;
    }
    // This checks the published solver rows, not exact geometric admission or
    // depth ranking. Prism faces are horizontal normals or +/-Y, so translation
    // compatibility separates into one planar semicircle and two vertical signs.
    private static bool AreMixedCurvedCutRowsCompatible(ReadOnlySpan<CurvedBoundarySample> samples)
    {
        bool up = false, down = false, positiveUp = false, positiveDown = false, positiveHorizontal = false;
        foreach (var sample in samples)
        {
            if (sample.GeometryOnly) continue;
            Vector3d normal = sample.Contact.Normal; bool positive = sample.Contact.Depth > Fixed64.Zero;
            if (normal.Y > Fixed64.Zero) { up = true; positiveUp |= positive; }
            else if (normal.Y < Fixed64.Zero) { down = true; positiveDown |= positive; }
            else positiveHorizontal |= positive;
        }
        if (up && down && (positiveUp || positiveDown)) return false;
        if (!positiveHorizontal) return true;
        bool semicircle = false;
        foreach (var first in samples)
        {
            if (first.GeometryOnly) continue;
            Vector3d normal = first.Contact.Normal;
            if (normal.Y != Fixed64.Zero) continue;
            var axis = new WideDirection3d(Signed576.ExtendValue(Signed192.Raw(normal.X)), default,
                Signed576.ExtendValue(Signed192.Raw(normal.Z)));
            bool contains = true;
            foreach (var second in samples)
            {
                if (second.GeometryOnly) continue;
                Vector3d other = second.Contact.Normal;
                if (other.Y != Fixed64.Zero) continue;
                var candidate = new WideDirection3d(Signed576.ExtendValue(Signed192.Raw(other.X)), default,
                    Signed576.ExtendValue(Signed192.Raw(other.Z)));
                int cross = WideDirection3d.Cross(axis, candidate).Y.Sign;
                if (cross < 0) contains = false;
                else if (cross == 0 && WideDirection3d.Dot(axis, candidate).Sign < 0
                    && (first.Contact.Depth > Fixed64.Zero || second.Contact.Depth > Fixed64.Zero)) return false;
            }
            // Inspect all pivots even after finding a semicircle: a positive
            // duplicate of either zero-depth extreme can forbid that motion.
            semicircle |= contains;
        }
        return semicircle;
    }
    private void KeepMixedCurvedBoundary(int ordinal, in FixedContactAnchors contact, int compareToNative) =>
        _mixedCurvedBoundarySamples.Add(new(ordinal, contact, compareToNative));

    private readonly struct CurvedBoundarySample
    {
        internal readonly int Ordinal, CompareToNative;
        internal readonly bool GeometryOnly;
        internal readonly FixedContactAnchors Contact;
        internal CurvedBoundarySample(int ordinal, FixedContactAnchors contact, int compareToNative, bool geometryOnly = false)
        { Ordinal = ordinal; Contact = contact; CompareToNative = compareToNative; GeometryOnly = geometryOnly; }
    }
    private void AddMixedCurvedSample(in FixedContactAnchors contact, int face, bool physical,
        RationalSegmentInterval clip = default, Fixed64 lower = default, Fixed64 upper = default, int boundaryEdge = -1)
    {
        bool represented = contact.SecondAnchor.TryGetPointInFrame(_mixedMoving!.Center, out Vector3d point);
        System.Diagnostics.Debug.Assert(represented);
        _mixedSamples.Add(new(_mixedCurvedGroups.Count, _mixedMember, point, contact));
        GetMixedCurvedPlaneForRow(_mixedMember, face, contact, out var axis, out var reference);
        _mixedCurvedGroups.Add(new(_mixedMember, face, clip, lower, upper, boundaryEdge, !physical, axis, reference));
        _mixedRegionParents.Add(_mixedRegionParents.Count);
        if (physical) { _mixedSpherePhysicalComponents.Add(_mixedComponents[_mixedMember]); RequestMixedReach(contact.Depth); }
    }

    private bool AppendMixedCurved(LSCollider2D overlap, ContactManifold destination, int namespace3D, int namespace2D)
    {
#if DEBUG
        // This checks completeness of the admitted candidate inventory; it is
        // not another geometry restriction. Keep the extra scan out of Release.
        int component = -1;
        for (int member = 0; member < _mixedLocal.Count; member++)
            if (_mixedLocal[member] == overlap && _mixedOverlaps.Contains(overlap.Id))
            { component = _mixedComponents[member]; break; }
        if (component >= 0 && (_mixedMoving!.Body ?? _mixedMoving.CompoundOwner?.Body) is { HasSolverMobility: true })
        {
            bool boundary = false;
            foreach (MixedSample sample in _mixedSamples.AsReadOnlySpan())
                boundary |= _mixedComponents[sample.Supplier] == component;
            System.Diagnostics.Debug.Assert(boundary,
                "A response-required closed-solid overlap must retain an admitted boundary constraint.");
        }
#endif
        AppendMixedCurvedRegions(overlap, destination, namespace3D, namespace2D);
        return _mixedOverlaps.Contains(overlap.Id);
    }

    private readonly struct CurvedGroup
    {
        internal readonly int Member, Face, BoundaryEdge;
        internal readonly RationalSegmentInterval Clip;
        internal readonly Fixed64 Lower, Upper;
        internal readonly bool FarBoundary;
        internal readonly WideDirection3d Axis;
        internal readonly RationalPoint3d Reference;
        internal CurvedGroup(int member, int face, RationalSegmentInterval clip, Fixed64 lower, Fixed64 upper,
            int boundaryEdge, bool farBoundary, WideDirection3d axis, RationalPoint3d reference)
        { Member = member; Face = face; Clip = clip; Lower = lower; Upper = upper;
            BoundaryEdge = boundaryEdge; FarBoundary = farBoundary; Axis = axis; Reference = reference; }
    }
}
