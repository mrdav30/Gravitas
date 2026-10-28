// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
using FixedMathSharp;
using FixedMathSharp.Geometry;
using Gravitas.Colliders;
using Gravitas.CollisionHandling;
using SwiftCollections;

namespace Gravitas;

/// <content>Prepares exact concave boundary lines shared by native rounded-solid contacts.</content>
internal sealed partial class StaticSurface2D
{
    private readonly SwiftList<MixedBoundaryLine> _mixedBoundaryLines = new();
    private readonly SwiftList<int> _mixedCapsuleLineGroups = new();
    private CapsulePrismBoundaryContacts? _mixedCapsuleBoundary;
    private bool _mixedBoundaryLinesCurrent;

    private void PrepareMixedBoundaryLines()
    {
        _mixedBoundaryLines.FastClear();
        if (!_mixedSphereConcavityCurrent) PrepareMixedSphereConcavity();
        for (_mixedMember = 0; _mixedMember < _mixedLocal.Count; _mixedMember++)
        {
            MixedPrism prism = _mixedPrisms[_mixedMember];
            if (!prism.SphereHasConcaveFace) continue;
            for (int index = 0; index < prism.BandCount; index++)
            {
                MixedBand band = prism.Bands[index];
                foreach (var span in band.Boundary.Spans)
                {
                    if (span.Kind == MeshSurfaceBoundaryDomain2D.SpanKind.Joined) continue;
                    GetMixedBoundaryBounds(span.Interval, out var ln, out var ld, out var un, out var ud);
                    Probe(new(_mixedMember, span.First, 0, new(ln, ld, ln, ld), band.Lower, band.Upper));
                    Probe(new(_mixedMember, span.First, 0, new(un, ud, un, ud), band.Lower, band.Upper));
                    // A height step may cut only part of this exposed span.
                    // Intersect with the incident band's actual joined windows before probing.
                    for (int end = 0; end < 2; end++)
                    {
                        Fixed64 y = end == 0 ? band.Lower : band.Upper;
                        int adjacent = end == 0 ? index - 1 : index + 1;
                        if (adjacent >= 0 && adjacent < prism.BandCount)
                            foreach (var joined in prism.Bands[adjacent].Boundary.Spans)
                                if (joined.First == span.First && joined.Kind == MeshSurfaceBoundaryDomain2D.SpanKind.Joined
                                    && TryIntersectMixedIntervals(span.Interval, joined.Interval, out var clip))
                                    Probe(new(_mixedMember, span.First, 0, clip, y, y));
                        // Incident fans can become blocked at a single endpoint only.
                        Probe(new(_mixedMember, span.First, 0, new(ln, ld, ln, ld), y, y));
                        Probe(new(_mixedMember, span.First, 0, new(un, ud, un, ud), y, y));
                    }
                }
            }
            for (int cap = 0; cap < 2; cap++)
            {
                Fixed64 y = cap == 0 ? -_mixedLocal[_mixedMember].MixedHalfThickness : _mixedLocal[_mixedMember].MixedHalfThickness;
                foreach (var span in prism.Bands[cap == 0 ? 0 : prism.BandCount - 1].Boundary.Spans)
                {
                    int vertical = cap == 0 ? 1 : -1;
                    if (span.Kind == MeshSurfaceBoundaryDomain2D.SpanKind.Joined)
                        Probe(new(_mixedMember, span.First, vertical, span.Interval, y, y));
                    GetMixedBoundaryBounds(span.Interval, out var ln, out var ld, out var un, out var ud);
                    Probe(new(_mixedMember, span.First, vertical, new(ln, ld, ln, ld), y, y));
                    Probe(new(_mixedMember, span.First, vertical, new(un, ud, un, ud), y, y));
                }
            }
        }
        _mixedBoundaryLinesCurrent = true;

        void Probe(in MixedBoundaryLine line)
        {
            MixedPrism prism = _mixedPrisms[line.Member];
            Vector2d a = prism.Offsets[line.Edge], b = prism.Offsets[(line.Edge + 1) % prism.Offsets.Length];
            WideDirection3d normal = MixedBoundaryLineNormal(line);
            GetMixedBoundaryBounds(line.Clip, out var ln, out var ld, out var un, out var ud);
            const int words = 32;
            System.Span<ulong> values = stackalloc ulong[8 * words]; System.Span<int> signs = stackalloc int[8];
            var n = ContactQuadratic.At(values, signs, 0, words); var d = ContactQuadratic.At(values, signs, 1, words);
            var y = ContactQuadratic.At(values, signs, 2, words); var yd = ContactQuadratic.At(values, signs, 3, words);
            n.Set(WideArithmetic.AddSigned576(WideArithmetic.MultiplySigned320(ln, ud), WideArithmetic.MultiplySigned320(un, ld)));
            Signed576 divisor = WideArithmetic.MultiplySigned320(ld, ud);
            d.Set(WideArithmetic.AddSigned576(divisor, divisor));
            y.Set(Signed576.ExtendValue(WideArithmetic.AddSigned192(Signed192.Raw(line.Lower), Signed192.Raw(line.Upper))));
            yd.Set(Signed576.ExtendValue(Signed192.Signed(2)));
            if (AdmitMixedSphereFan(new(new(a.X, Fixed64.Zero, a.Y), new(b.X, Fixed64.Zero, b.Y)), n, d, y, yd, normal)) return;
            KeepMixedBoundaryLine(line);
        }
    }

    private void KeepMixedBoundaryLine(MixedBoundaryLine line)
    {
        // Remove artificial height-band cuts before selecting a peak, so a
        // remote neighbor cannot change the sampling of one continuous line.
        for (int index = _mixedBoundaryLines.Count - 1; index >= 0; index--)
        {
            MixedBoundaryLine old = _mixedBoundaryLines[index];
            if (line.Member != old.Member || line.Edge != old.Edge || line.Vertical != old.Vertical) continue;
            bool sameTangent = SameMixedBoundaryInterval(line.Clip, old.Clip);
            if (sameTangent && line.Lower <= old.Upper && old.Lower <= line.Upper)
            {
                line = new(line.Member, line.Edge, line.Vertical, line.Clip,
                    FixedMath.Min(line.Lower, old.Lower), FixedMath.Max(line.Upper, old.Upper));
                _mixedBoundaryLines.RemoveAt(index); continue;
            }
            if (line.Lower != line.Upper || old.Lower != old.Upper || line.Lower != old.Lower
                || !TryIntersectMixedIntervals(line.Clip, old.Clip, out _)) continue;
            GetMixedBoundaryBounds(line.Clip, out var ln, out var ld, out var un, out var ud);
            GetMixedBoundaryBounds(old.Clip, out var oln, out var oldd, out var oun, out var oud);
            if (RationalSegmentInterval.CompareParameters(ln, ld, oln, oldd) > 0) { ln = oln; ld = oldd; }
            if (RationalSegmentInterval.CompareParameters(un, ud, oun, oud) < 0) { un = oun; ud = oud; }
            line = new(line.Member, line.Edge, line.Vertical, new(ln, ld, un, ud), line.Lower, line.Upper);
            _mixedBoundaryLines.RemoveAt(index);
        }
        _mixedBoundaryLines.Add(line);
    }

    private void AppendMixedCapsuleBoundaryPeaks(LSCapsuleCollider capsule)
    {
        if (!_mixedBoundaryLinesCurrent) PrepareMixedBoundaryLines();
        _mixedCapsuleLineGroups.FastClear();
        int supportCount = _mixedSphereGroups.Count;
        foreach (MixedBoundaryLine line in _mixedBoundaryLines.AsReadOnlySpan())
        {
            _mixedCapsuleLineGroups.Add(-1);
            _mixedMember = line.Member;
            LSCollider2D member = _mixedLocal[line.Member]; MixedPrism prism = _mixedPrisms[line.Member];
            if (!_mixedOverlaps.Contains(member.Id)) continue;
            _mixedCapsuleBoundary ??= new();
            if (!_mixedCapsuleBoundary.TryGetContact(capsule.Center, capsule.Rotation, capsule.AxisLength, capsule.ScaledRadius,
                MixedEmbedded2DGeometry.GetCenter3D(member), member.ConvexRotation, prism.Offsets[line.Edge],
                prism.Offsets[(line.Edge + 1) % prism.Offsets.Length], prism.Offsets[(line.Edge + 2) % prism.Offsets.Length],
                line.Clip, line.Lower, line.Upper, line.Vertical, out var contact)) continue;
            WideDirection3d normal = MixedBoundaryLineNormal(line);
            int component = _mixedComponents[line.Member]; _mixedSpherePhysicalComponents.Add(component);
            int group = AddMixedCapsuleSample(new(component, member.Id, line.Member, normal,
                MixedSupportPoint(prism.NativeFrame, prism, normal), default, contact), default);
            _mixedCapsuleLineGroups[_mixedCapsuleLineGroups.Count - 1] = group;
            for (int support = 0; support < supportCount; support++)
                if (_mixedSphereGroups[support].Supplier == line.Member
                    && SameMixedCapsulePlane(group, support)
                    && _mixedCapsuleBoundary.ConnectsSupport(_mixedCapsuleCore[support]))
                    _mixedRegionParents[FindMixedRegion(group)] = FindMixedRegion(support);
            for (int previous = 0; previous < _mixedCapsuleLineGroups.Count - 1; previous++)
            {
                int other = _mixedCapsuleLineGroups[previous];
                if (other < 0 || FindMixedRegion(group) == FindMixedRegion(other) || !SameMixedCapsulePlane(group, other)) continue;
                MixedBoundaryLine old = _mixedBoundaryLines[previous];
                // Same-source strata meet at a rational native point. Test the
                // actual finite capsule there; a rounded peak is not connectivity evidence.
                if (line.Member == old.Member && line.Edge == old.Edge
                    && FixedMath.Max(line.Lower, old.Lower) <= FixedMath.Min(line.Upper, old.Upper)
                    && TryIntersectMixedIntervals(line.Clip, old.Clip, out var clip))
                {
                    GetMixedBoundaryBounds(clip, out var n, out var d, out _, out _);
                    if (_mixedCapsuleBoundary.ContainsPoint(n, d, FixedMath.Max(line.Lower, old.Lower)))
                        _mixedRegionParents[FindMixedRegion(group)] = FindMixedRegion(other);
                }
                else if (line.Member == old.Member && line.Vertical != 0)
                {
                    int count = prism.Offsets.Length;
                    bool atStart = (old.Edge + 1) % count == line.Edge
                        && ContainsEndpoint(old.Clip, true) && ContainsEndpoint(line.Clip, false);
                    bool atEnd = (line.Edge + 1) % count == old.Edge
                        && ContainsEndpoint(line.Clip, true) && ContainsEndpoint(old.Clip, false);
                    if ((atStart || atEnd) && _mixedCapsuleBoundary.ContainsPoint(atStart ? default : Signed320.One,
                        Signed320.One, line.Lower))
                        _mixedRegionParents[FindMixedRegion(group)] = FindMixedRegion(other);
                }
            }
        }

        static bool ContainsEndpoint(in RationalSegmentInterval clip, bool upper)
        {
            GetMixedBoundaryBounds(clip, out var ln, out var ld, out var un, out var ud);
            Signed320 n = upper ? Signed320.One : default;
            return RationalSegmentInterval.CompareParameters(ln, ld, n, Signed320.One) <= 0
                && RationalSegmentInterval.CompareParameters(un, ud, n, Signed320.One) >= 0;
        }
    }

    private WideDirection3d MixedBoundaryLineNormal(in MixedBoundaryLine line)
    {
        if (line.Vertical != 0) return new(default, Signed576.ExtendValue(Signed192.Signed(line.Vertical)), default);
        MixedPrism prism = _mixedPrisms[line.Member];
        Vector2d a = prism.Offsets[line.Edge], b = prism.Offsets[(line.Edge + 1) % prism.Offsets.Length],
            inside = prism.Offsets[(line.Edge + 2) % prism.Offsets.Length];
        var triangle = new FixedTriangle(new(a.X, Fixed64.Zero, a.Y), new(b.X, Fixed64.Zero, b.Y), new(inside.X, Fixed64.Zero, inside.Y));
        return prism.NativeFrame.GetInwardAxis(triangle, triangle.A, triangle.B);
    }

    private static void GetMixedBoundaryBounds(in RationalSegmentInterval clip,
        out Signed320 ln, out Signed320 ld, out Signed320 un, out Signed320 ud)
    {
        Signed320 one = Signed320.One;
        ln = clip.IsClipped ? clip.LowerNumerator : default; ld = clip.IsClipped ? clip.LowerDenominator : one;
        un = clip.IsClipped ? clip.UpperNumerator : one; ud = clip.IsClipped ? clip.UpperDenominator : one;
    }

    private static bool SameMixedBoundaryInterval(in RationalSegmentInterval a, in RationalSegmentInterval b)
    {
        GetMixedBoundaryBounds(a, out var ln, out var ld, out var un, out var ud);
        GetMixedBoundaryBounds(b, out var bln, out var bld, out var bun, out var bud);
        return RationalSegmentInterval.CompareParameters(ln, ld, bln, bld) == 0
            && RationalSegmentInterval.CompareParameters(un, ud, bun, bud) == 0;
    }

    private readonly struct MixedBoundaryLine
    {
        internal readonly int Member, Edge, Vertical;
        internal readonly RationalSegmentInterval Clip;
        internal readonly Fixed64 Lower, Upper;
        internal MixedBoundaryLine(int member, int edge, int vertical, RationalSegmentInterval clip, Fixed64 lower, Fixed64 upper)
        { Member = member; Edge = edge; Vertical = vertical; Clip = clip; Lower = lower; Upper = upper; }
    }
}
