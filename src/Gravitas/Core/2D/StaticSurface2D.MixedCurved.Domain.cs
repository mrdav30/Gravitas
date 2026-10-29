// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
using System;
using System.Collections.Generic;
using FixedMathSharp;
using FixedMathSharp.Geometry;
using Gravitas.CollisionHandling;
using SwiftCollections;

namespace Gravitas;

/// <content>Reuses prepared native boundary strata and their exact incident support fans.</content>
public sealed partial class StaticSurface2D
{
    private readonly SwiftList<MixedCurvedDomain> _mixedCurvedDomains = new();
    private readonly SwiftList<RationalSegmentInterval> _mixedCurvedCuts = new();
    private readonly SwiftList<MixedCurvedCertificate> _mixedCurvedCertificates = new();
    private readonly SwiftList<PrismCurvedFaceCertificate> _mixedCurvedFaceCertificates = new();
    private readonly SwiftList<int> _mixedCurvedCertificateSources = new(), _mixedCurvedCertificateGroups = new();
    private bool _mixedCurvedDomainsCurrent;
    private static readonly IComparer<RationalSegmentInterval> MixedCurvedCutOrder =
        Comparer<RationalSegmentInterval>.Create(static (a, b) => RationalSegmentInterval.CompareParameters(
            a.WideLowerNumerator, a.WideLowerDenominator, b.WideLowerNumerator, b.WideLowerDenominator));

    private PrismCurvedContactDomain PrepareMixedCurvedDomain(in CurvedPrismContactGeometry geometry, bool farBoundary)
    {
        MixedCurvedDomain domain = _mixedCurvedDomains[_mixedMember];
        if (!domain.EdgesCurrent) PrepareMixedCurvedEdges(domain);
        domain.ActiveEdges.FastClear();
        if (!farBoundary)
            foreach (var edge in domain.Edges.AsReadOnlySpan())
                if (CurvedPrismContact.IntersectsRealEdge(geometry, edge.Edge, edge.Interval)) domain.ActiveEdges.Add(edge);
        ReadOnlySpan<PrismCurvedEdgeWindow> edges = farBoundary ? domain.Edges.AsReadOnlySpan() : domain.ActiveEdges.AsReadOnlySpan();
        domain.LocalFans.FastClear();
        foreach (WideDirection3d world in edges.IsEmpty ? ReadOnlySpan<WideDirection3d>.Empty : domain.WorldFans.AsReadOnlySpan())
        {
            // Native points share Q^2. Their full-domain differences need fewer
            // than 131 bits, and B^T adds at most 65: no coordinate is rounded.
            var delta = new WideAxis3(Signed320.NarrowValue(world.X),
                Signed320.NarrowValue(world.Y), Signed320.NarrowValue(world.Z));
            WideRigidProjection.GetLocalAxisProjections(delta, geometry.WorldBasis, out var x, out var y, out var z);
            domain.LocalFans.Add(new(x, y, z));
        }
        _mixedCurvedFaceCertificates.FastClear(); _mixedCurvedCertificateSources.FastClear();
        for (int face = -2; face < _mixedPrisms[_mixedMember].Offsets.Length; face++)
        {
            GetMixedCurvedPlane(_mixedMember, face, out var axis, out var reference);
            for (int certificate = 0; certificate < _mixedCurvedCertificates.Count; certificate++)
            {
                MixedCurvedCertificate source = _mixedCurvedCertificates[certificate];
                if (_mixedComponents[source.Member] != _mixedComponents[_mixedMember]
                    || !SameMixedCurvedPlane(axis, reference, source.Axis, source.Reference)) continue;
                _mixedCurvedFaceCertificates.Add(new(face, source.Contact, source.Physical));
                _mixedCurvedCertificateSources.Add(certificate);
            }
        }
        return new(edges, domain.LocalFans.AsReadOnlySpan(),
            certificates: _mixedCurvedFaceCertificates.AsReadOnlySpan());
    }

    private void PrepareMixedCurvedDomains()
    {
        int original = _mixedMember;
        while (_mixedCurvedDomains.Count < _mixedLocal.Count) _mixedCurvedDomains.Add(new());
        for (_mixedMember = 0; _mixedMember < _mixedLocal.Count; _mixedMember++)
        {
            MixedPrism prism = _mixedPrisms[_mixedMember];
            MixedCurvedDomain domain = _mixedCurvedDomains[_mixedMember];
            domain.Faces.FastClear();
            domain.Lines.FastClear(); domain.EdgesCurrent = false;
            foreach (var line in _mixedBoundaryLines.AsReadOnlySpan())
                if (line.Member == _mixedMember)
                    domain.Lines.Add(new(line.Vertical == 0 ? line.Edge : line.Vertical > 0 ? -1 : -2,
                        line.Clip, line.Lower, line.Upper, line.Vertical == 0 ? -1 : line.Edge));
            for (int bandIndex = 0; bandIndex < prism.BandCount; bandIndex++)
            {
                MixedBand band = prism.Bands[bandIndex];
                foreach (var span in band.Boundary.Spans)
                    if (span.Kind != MeshSurfaceBoundaryDomain2D.SpanKind.Joined)
                        KeepMixedCurvedWindow(domain, new(span.First, span.Interval, band.Lower, band.Upper));
            }
            Fixed64 half = _mixedLocal[_mixedMember].MixedHalfThickness;
            domain.Faces.Add(new(-1, default, -half, -half)); domain.Faces.Add(new(-2, default, half, half));
            _mixedCurvedActiveLines.EnsureCapacity(domain.Lines.Count);
            _mixedCurvedBoundarySamples.EnsureCapacity(domain.Lines.Count);
        }
        _mixedMember = original; _mixedCurvedDomainsCurrent = true;
    }

    private void PrepareMixedCurvedEdges(MixedCurvedDomain domain)
    {
        domain.Edges.FastClear(); domain.WorldFans.FastClear();
        // Nearby members may only supply a face witness. Build their larger
        // edge/fan inventory only when they become a visited response source;
        // both physical and far visits share the same exact topology lifetime.
        for (int edge = 0; edge < 3 * _mixedPrisms[_mixedMember].Offsets.Length; edge++)
            PrepareMixedCurvedEdge(domain, edge);
        domain.ActiveEdges.EnsureCapacity(domain.Edges.Count);
        domain.EdgesCurrent = true;
    }

    private void PrepareMixedCurvedEdge(MixedCurvedDomain domain, int edge)
    {
        GetMixedCapsuleEdge(edge / 3, edge % 3, out int first, out int last);
        if (edge % 3 == 2) GetMixedEdgeIntervals(first, last);
        else
        {
            _mixedIntervals.FastClear();
            MixedPrism prism = _mixedPrisms[_mixedMember];
            int band = edge % 3 == 0 ? 0 : prism.BandCount - 1;
            foreach (var span in prism.Bands[band].Boundary.Spans)
                if (span.First == edge / 3) _mixedIntervals.Add(span.Interval);
        }
        _mixedCurvedCuts.FastClear();
        foreach (RationalSegmentInterval interval in _mixedIntervals.AsReadOnlySpan())
        {
            Signed832 one = Signed832.ExtendValue(Signed192.Signed(1));
            AddCut(interval.IsClipped ? interval.WideLowerNumerator : default,
                interval.IsClipped ? interval.WideLowerDenominator : one);
            AddCut(interval.IsClipped ? interval.WideUpperNumerator : one,
                interval.IsClipped ? interval.WideUpperDenominator : one);
        }
        _mixedCurvedCuts.SortInPlace(MixedCurvedCutOrder);
        for (int cut = 0; cut < _mixedCurvedCuts.Count; cut++)
        {
            RationalSegmentInterval point = _mixedCurvedCuts[cut];
            Keep(point, true);
            if (cut + 1 < _mixedCurvedCuts.Count)
            {
                RationalSegmentInterval next = _mixedCurvedCuts[cut + 1];
                Keep(new(point.WideLowerNumerator, point.WideLowerDenominator,
                    next.WideLowerNumerator, next.WideLowerDenominator), false);
            }
        }

        void AddCut(Signed832 numerator, Signed832 denominator)
        {
            foreach (RationalSegmentInterval point in _mixedCurvedCuts.AsReadOnlySpan())
                if (RationalSegmentInterval.CompareParameters(numerator, denominator,
                        point.WideLowerNumerator, point.WideLowerDenominator) == 0) return;
            _mixedCurvedCuts.Add(new(numerator, denominator, numerator, denominator));
        }

        void Keep(in RationalSegmentInterval interval, bool point)
        {
            const int words = 32;
            Span<ulong> values = stackalloc ulong[16 * words]; Span<int> signs = stackalloc int[16];
            var n = ContactQuadratic.At(values, signs, 0, words); var d = ContactQuadratic.At(values, signs, 1, words);
            var y = ContactQuadratic.At(values, signs, 2, words); var yd = ContactQuadratic.At(values, signs, 3, words);
            var ln = ContactQuadratic.At(values, signs, 4, words); var ld = ContactQuadratic.At(values, signs, 5, words);
            var un = ContactQuadratic.At(values, signs, 6, words); var ud = ContactQuadratic.At(values, signs, 7, words);
            // The complete [0,1] interval canonicalizes to default, whose
            // stored fields are all zero. Reconstruct its unit denominators;
            // a homogeneous 0/0 point would otherwise reach every fan neighbor.
            Signed832 one = Signed832.ExtendValue(Signed192.Signed(1));
            ln.Set(interval.IsClipped ? interval.WideLowerNumerator : default);
            ld.Set(interval.IsClipped ? interval.WideLowerDenominator : one);
            un.Set(interval.IsClipped ? interval.WideUpperNumerator : one);
            ud.Set(interval.IsClipped ? interval.WideUpperDenominator : one);
            ContactQuadratic.Multiply(ln, ud, ReadOnlySpan<ulong>.Empty, n);
            ContactQuadratic.Multiply(un, ld, ReadOnlySpan<ulong>.Empty, y); n.Add(y);
            ContactQuadratic.Multiply(ld, ud, ReadOnlySpan<ulong>.Empty, d); d.Add(d);
            bool covered = false;
            foreach (RationalSegmentInterval retained in _mixedIntervals.AsReadOnlySpan())
                covered |= !retained.IsClipped || retained.Contains(n, d, ReadOnlySpan<ulong>.Empty);
            if (!covered) return;
            MixedPrism prism = _mixedPrisms[_mixedMember];
            Vector3d a = prism.Points[first], b = prism.Points[last];
            ContactQuadratic.Scale(d, Signed576.ExtendValue(Signed192.Raw(a.Y)), y);
            ContactQuadratic.Scale(n, Signed576.ExtendValue(WideArithmetic.Difference(b.Y, a.Y)), yd); y.Add(yd); d.CopyTo(yd);
            if (edge % 3 == 2) { n.Clear(); d.Set(Signed576.ExtendValue(Signed192.Signed(1))); }
            // A zero normal requests only the existing exact incident-network
            // traversal. Open intervals and endpoints gather independently;
            // crossing a join at one endpoint must not hide its stricter fan.
            AdmitMixedSphereFan(new(new(a.X, Fixed64.Zero, a.Z), new(b.X, Fixed64.Zero, b.Z)), n, d, y, yd, default);
            int reference = point && RationalSegmentInterval.CompareParameters(interval.WideLowerNumerator,
                interval.WideLowerDenominator, Signed832.ExtendValue(Signed192.Signed(1)),
                Signed832.ExtendValue(Signed192.Signed(1))) == 0 ? last : first;
            RationalPoint3d origin = prism.NativeFrame.GetPoint(prism.Points[reference]);
            int fanStart = domain.WorldFans.Count;
            foreach (int member in _mixedFan.AsReadOnlySpan())
            {
                if (member == _mixedMember) continue;
                MixedPrism neighbor = _mixedPrisms[member];
                foreach (Vector3d vertex in neighbor.Points)
                    domain.WorldFans.Add(WideTriangleRelations.GetScaledPointDifference(
                        neighbor.NativeFrame.GetPoint(vertex), origin, prism.NativeFrame.RotationDenominator));
            }
            domain.Edges.Add(new(edge, interval, point, reference, fanStart, domain.WorldFans.Count - fanStart));
        }
    }

    private static void KeepMixedCurvedWindow(MixedCurvedDomain domain, in MixedCurvedFaceWindow window)
    {
        for (int index = 0; index < domain.Faces.Count; index++)
        {
            MixedCurvedFaceWindow previous = domain.Faces[index];
            if (previous.Face == window.Face && SameMixedBoundaryInterval(previous.Interval, window.Interval)
                && previous.Upper == window.Lower)
            {
                domain.Faces[index] = new(window.Face, window.Interval, previous.Lower, window.Upper); return;
            }
        }
        domain.Faces.Add(window);
    }

    private void PrepareMixedCurvedCertificates(bool cone, Fixed64 height, Span<WideDirection3d> vertices, bool farBoundary = false)
    {
        if (!_mixedCurvedDomainsCurrent) PrepareMixedCurvedDomains();
        if (!farBoundary) { _mixedCurvedCertificates.FastClear(); _mixedCurvedCertificateGroups.FastClear(); }
        int original = _mixedMember;
        for (_mixedMember = 0; _mixedMember < _mixedLocal.Count; _mixedMember++)
        {
            MixedPrism prism = _mixedPrisms[_mixedMember];
            if (!prism.Joined || _mixedCurvedCarriers[_mixedComponents[_mixedMember]] < 0
                || farBoundary && _mixedSpherePhysicalComponents.Contains(_mixedComponents[_mixedMember])) continue;
            var member = _mixedLocal[_mixedMember]; var mover = _mixedMoving!;
            Vector3d origin = MixedEmbedded2DGeometry.GetCenter3D(member);
            var geometry = new CurvedPrismContactGeometry(mover.Center, mover.Rotation, Vector3d.Up,
                height, mover.ScaledRadius, origin, member.ConvexRotation, prism.Offsets, member.MixedHalfThickness, cone, vertices);
            foreach (var window in _mixedCurvedDomains[_mixedMember].Faces.AsReadOnlySpan())
            {
                if (!CurvedPrismContact.TryGetFaceSupportContact(geometry, window.Face, window.Interval, window.Lower, window.Upper,
                    mover.Center, mover.Rotation, origin, farBoundary, out var contact, out bool physical)) continue;
                GetMixedCurvedPlane(_mixedMember, window.Face, out var axis, out var reference);
                _mixedCurvedCertificates.Add(new(_mixedMember, window, axis, reference, contact, physical));
                _mixedCurvedCertificateGroups.Add(-1);
            }
        }
        _mixedMember = original;
    }

    private void GetMixedCurvedPlane(int member, int face, out WideDirection3d axis, out RationalPoint3d reference)
    {
        MixedPrism prism = _mixedPrisms[member];
        if (face < 0) axis = new(default, Signed576.ExtendValue(Signed192.Signed(face == -1 ? 1 : -1)), default);
        else
        {
            Vector2d a = prism.Offsets[face], b = prism.Offsets[(face + 1) % prism.Offsets.Length],
                inside = prism.Offsets[(face + 2) % prism.Offsets.Length];
            var triangle = new FixedTriangle(new(a.X, Fixed64.Zero, a.Y), new(b.X, Fixed64.Zero, b.Y), new(inside.X, Fixed64.Zero, inside.Y));
            axis = prism.NativeFrame.GetInwardAxis(triangle, triangle.A, triangle.B);
        }
        reference = prism.NativeFrame.GetPoint(prism.Points[face < 0 ? face == -1 ? 0 : 1 : 2 * face]);
    }

    private static bool SameMixedCurvedPlane(in WideDirection3d a, in RationalPoint3d first,
        in WideDirection3d b, in RationalPoint3d second) => WideDirection3d.AreParallel(a, b)
        && WideDirection3d.Dot(a, b).Sign > 0 && WideTriangleRelations.DotScaledDirections(a,
            WideTriangleRelations.GetScaledPointDifference(first, second, Signed192.Signed(1))).IsZero;

    private readonly struct MixedCurvedCertificate
    {
        internal readonly int Member;
        internal readonly MixedCurvedFaceWindow Window;
        internal readonly WideDirection3d Axis;
        internal readonly RationalPoint3d Reference;
        internal readonly FixedContactAnchors Contact;
        internal readonly bool Physical;
        internal MixedCurvedCertificate(int member, MixedCurvedFaceWindow window, WideDirection3d axis,
            RationalPoint3d reference, FixedContactAnchors contact, bool physical)
        { Member = member; Window = window; Axis = axis; Reference = reference; Contact = contact; Physical = physical; }
    }

    private readonly struct MixedCurvedFaceWindow
    {
        internal readonly int Face;
        internal readonly RationalSegmentInterval Interval;
        internal readonly Fixed64 Lower, Upper;
        internal MixedCurvedFaceWindow(int face, RationalSegmentInterval interval, Fixed64 lower, Fixed64 upper)
        { Face = face; Interval = interval; Lower = lower; Upper = upper; }
    }

    private sealed class MixedCurvedDomain
    {
        internal bool EdgesCurrent;
        internal readonly SwiftList<MixedCurvedFaceWindow> Faces = new();
        internal readonly SwiftList<PrismCurvedBoundaryLine> Lines = new();
        internal readonly SwiftList<PrismCurvedEdgeWindow> Edges = new(), ActiveEdges = new();
        internal readonly SwiftList<WideDirection3d> WorldFans = new(), LocalFans = new();
    }
}
