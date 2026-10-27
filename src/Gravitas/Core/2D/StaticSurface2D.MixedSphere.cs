// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
using FixedMathSharp;
using FixedMathSharp.Geometry;
using Gravitas.Colliders;
using Gravitas.CollisionHandling;
using System;
using SwiftCollections;

namespace Gravitas;

/// <content>Admits rounded point supports against the existing exposed prism domain.</content>
internal sealed partial class StaticSurface2D
{
    private readonly PrismPointAdmission _mixedSphereAdmit;
    private readonly PrismSphereBoundaryAdmission _mixedSphereBoundaryAdmit;
    private FixedSegment _mixedSphereFaceFootprint;

    private readonly SwiftList<SphereGroup> _mixedSphereGroups = new();
    private readonly SwiftList<SpherePeak> _mixedSpherePeaks = new();
    private readonly SwiftHashSet<int> _mixedSphereInsideComponents = new();
    private readonly SwiftHashSet<int> _mixedSpherePhysicalComponents = new();
    private bool _mixedSphereInside, _mixedSphereConcavityCurrent;

    private void BuildMixedSphere()
    {
        if (!_mixedSphereConcavityCurrent) PrepareMixedSphereConcavity();
        _mixedSphereGroups.FastClear(); _mixedSpherePeaks.FastClear(); _mixedOverlaps.Clear(); _mixedSphereInsideComponents.Clear(); _mixedSpherePhysicalComponents.Clear();
        Fixed64 radius = _mixedMoving is LSSphereCollider sphere ? sphere.ScaledRadius : ((LSCapsuleCollider)_mixedMoving!).ScaledRadius;
        for (int member = 0; member < _mixedLocal.Count; member++)
        {
            LSCollider2D source = _mixedLocal[member];
            if (source.ContainsPoint(new(_mixedMoving!.Center.X, _mixedMoving.Center.Z))
                && MixedEmbedded2DGeometry.ContainsPointInSlab(source, _mixedMoving.Center.Y))
                _mixedSphereInsideComponents.Add(_mixedComponents[member]);
        }
        for (_mixedMember = 0; _mixedMember < _mixedLocal.Count; _mixedMember++)
        {
            LSCollider2D member = _mixedLocal[_mixedMember];
            if (!_mixedMoving!.Bounds.Intersects(member.MixedBounds3D)) continue;
            MixedPrism prism = _mixedPrisms[_mixedMember];
            if (!prism.Joined)
            {
                // Preserve the ordinary per-member path, but discover any
                // remote neighbor reached by a deep exit before concluding it is unjoined.
                if (FixedConvexPrismRelations.TryGetSphereContact(_mixedMoving.Center, radius,
                    MixedEmbedded2DGeometry.GetCenter3D(member), member.ConvexRotation, prism.Offsets,
                    member.MixedHalfThickness, out var separate))
                {
                    Fixed64 separateReach = radius + separate.Depth + Fixed64.MinIncrement + Fixed64.MinIncrement;
                    if (separateReach > _mixedRequestedReach) { _mixedRequestedReach = separateReach; return; }
                }
                continue;
            }
            int component = _mixedComponents[_mixedMember];
            _mixedSphereInside = _mixedSphereInsideComponents.Contains(component);
            bool found = WideConvexPrismRelations.TryGetSphereContact(_mixedMoving.Center, radius,
                MixedEmbedded2DGeometry.GetCenter3D(member), member.ConvexRotation, prism.Offsets,
                member.MixedHalfThickness, _mixedSphereAdmit, out var contact, out var axis, out var depth, out bool overlaps);
            if (overlaps) _mixedOverlaps.Add(member.Id);
            bool physical = found && WideConvexPrismRelations.IsSphereProjectionWithinRadius(depth);
            if (physical) _mixedSpherePhysicalComponents.Add(component);
            if (prism.SphereHasConcaveFace) AppendMixedSphereFacePeaks(radius);
            if (!found) continue;
            Fixed64 reach = radius + contact.Depth + Fixed64.MinIncrement + Fixed64.MinIncrement;
            if (reach > _mixedRequestedReach) { _mixedRequestedReach = reach; return; }
            RationalPoint3d reference = MixedSupportPoint(prism.NativeFrame, prism, axis);
            int group = 0;
            for (; group < _mixedSphereGroups.Count; group++)
            {
                SphereGroup previous = _mixedSphereGroups[group];
                if (previous.Component != component) continue;
                if (!physical && previous.Recovery || physical && !previous.Recovery && WideDirection3d.AreParallel(axis, previous.Axis)
                    && WideDirection3d.Dot(axis, previous.Axis).Sign > 0
                    && WideTriangleRelations.DotScaledDirections(axis, WideTriangleRelations.GetScaledPointDifference(
                        reference, previous.Reference, Signed192.Signed(1))).IsZero) break;
            }
            var row = new SphereGroup(component, member.Id, _mixedMember, axis, reference, depth, contact, !physical);
            if (group == _mixedSphereGroups.Count) _mixedSphereGroups.Add(row);
            else if (!physical && WideConvexPrismRelations.CompareProjectionDepths(depth, _mixedSphereGroups[group].Depth) < 0)
                _mixedSphereGroups[group] = row;
        }
    }

    private void PrepareMixedSphereConcavity()
    {
        // A component wholly inside every exposed face half-plane cannot have
        // a concave face peak. Cache this sufficient certificate with its geometry;
        // uncertain/nonconvex pieces still use exact pointwise incident fans.
        for (int member = 0; member < _mixedLocal.Count; member++)
        {
            MixedPrism prism = _mixedPrisms[member]; prism.SphereHasConcaveFace = false;
            if (!prism.Joined) continue;
            for (int face = -2; face < prism.Offsets.Length; face++)
            {
                Vector3d reference; WideDirection3d normal;
                if (face < 0)
                {
                    reference = prism.Points[face == -2 ? 0 : 1];
                    normal = new(default, Signed576.ExtendValue(Signed192.Signed(face == -2 ? 1 : -1)), default);
                }
                else
                {
                    bool exposed = false;
                    for (int band = 0; band < prism.BandCount; band++)
                        foreach (var span in prism.Bands[band].Boundary.Spans)
                            if (span.First == face && span.Kind != MeshSurfaceBoundaryDomain2D.SpanKind.Joined) exposed = true;
                    if (!exposed) continue;
                    Vector2d a = prism.Offsets[face], b = prism.Offsets[(face + 1) % prism.Offsets.Length];
                    Vector2d inside = prism.Offsets[(face + 2) % prism.Offsets.Length];
                    var footprint = new FixedTriangle(new(a.X, Fixed64.Zero, a.Y), new(b.X, Fixed64.Zero, b.Y), new(inside.X, Fixed64.Zero, inside.Y));
                    reference = footprint.A;
                    normal = prism.NativeFrame.GetInwardAxis(footprint, footprint.A, footprint.B);
                }
                for (int other = 0; other < _mixedLocal.Count; other++)
                    if (other != member && _mixedComponents[other] == _mixedComponents[member]
                        && !SupportsMixedFan(_mixedPrisms[other].NativeFrame, prism.NativeFrame, reference,
                            _mixedPrisms[other].Points, normal))
                    { prism.SphereHasConcaveFace = true; break; }
                if (prism.SphereHasConcaveFace) break;
            }
        }
        _mixedSphereConcavityCurrent = true;
    }

    private bool AppendMixedSphere(LSCollider2D overlap, ContactManifold destination, int namespace3D, int namespace2D)
    {
        for (int group = 0; group < _mixedSphereGroups.Count; group++)
        {
            SphereGroup row = _mixedSphereGroups[group];
            if (row.Carrier != overlap.Id || row.Recovery && _mixedSpherePhysicalComponents.Contains(row.Component)) continue;
            Append(row.Contact, row.Supplier, group);
        }
        for (int peak = 0; peak < _mixedSpherePeaks.Count; peak++)
        {
            SpherePeak row = _mixedSpherePeaks[peak];
            if (_mixedLocal[row.Member].Id == overlap.Id) Append(row.Contact, row.Member, _mixedSphereGroups.Count + peak);
        }
        void Append(in FixedContactAnchors contact, int member, int group)
        {
            LSCollider2D supplier = _mixedLocal[member];
            destination.AddContact(new ContactAnchor(contact.FirstAnchor), new ContactAnchor(contact.SecondAnchor),
                contact.Depth, contact.Normal, _mixedMoving!.Material, supplier.Material, contact.DepthIsClamped,
                featureNamespaceA: namespace3D, featureNamespaceB: namespace2D,
                group: new ContactGroupKey(namespace3D, namespace2D, region: group, sourceIdB: supplier.Id + 1));
        }
        return _mixedOverlaps.Contains(overlap.Id);
    }

    private readonly struct SphereGroup
    {
        internal readonly int Component, Carrier, Supplier;
        internal readonly WideDirection3d Axis;
        internal readonly RationalPoint3d Reference;
        internal readonly WideConvexPrismRelations.ProjectionDepth Depth;
        internal readonly FixedContactAnchors Contact;
        internal readonly bool Recovery;
        internal SphereGroup(int component, int carrier, int supplier, WideDirection3d axis, RationalPoint3d reference,
            WideConvexPrismRelations.ProjectionDepth depth, FixedContactAnchors contact, bool recovery = false)
        { Component = component; Carrier = carrier; Supplier = supplier; Axis = axis; Reference = reference; Depth = depth; Contact = contact; Recovery = recovery; }
    }

    private readonly struct SpherePeak
    {
        internal readonly int Member, Edge, Vertical;
        internal readonly Fixed64 Height;
        internal readonly Signed320 Numerator, Denominator;
        internal readonly FixedContactAnchors Contact;
        internal SpherePeak(int member, int edge, int next, int vertical, Fixed64 height,
            in RationalSegmentInterval parameter, in FixedContactAnchors contact)
        {
            Member = member; Edge = edge; Vertical = vertical; Height = height;
            // Native boundary cuts use Signed320, and a fresh closest-point
            // Gram ratio needs <162 bits. The wider math helper is not narrowed generically.
            Numerator = parameter.LowerNumerator; Denominator = parameter.LowerDenominator; Contact = contact;
            // Two footprint edges can sample one cap vertex; sides retain their
            // own normals, while cap endpoints share a canonical vertex key.
            if (vertical != 0 && RationalSegmentInterval.CompareParameters(Numerator, Denominator, Signed320.One, Signed320.One) == 0)
            { Edge = next; Numerator = default; Denominator = Signed320.One; }
        }
    }

    private void AppendMixedSphereFacePeaks(Fixed64 radius)
    {
        MixedPrism prism = _mixedPrisms[_mixedMember];
        LSCollider2D member = _mixedLocal[_mixedMember];
        Span<ulong> storage = stackalloc ulong[CapsulePlaneRaySelection.StorageWords];
        Span<int> signs = stackalloc int[CapsulePlaneRaySelection.SignCount];
        Span<sbyte> valueSigns = stackalloc sbyte[CapsulePlaneRaySelection.ValueSignCount];
        var selection = new CapsulePlaneRaySelection(storage, signs, valueSigns);
        for (int bandIndex = 0; bandIndex < prism.BandCount; bandIndex++)
        {
            MixedBand band = prism.Bands[bandIndex];
            foreach (var span in band.Boundary.Spans)
            {
                if (span.Kind == MeshSurfaceBoundaryDomain2D.SpanKind.Joined) continue;
                Keep(span, band.Lower, band.Upper, 0, ref selection);
            }
        }
        for (int cap = 0; cap < 2; cap++)
        {
            Fixed64 y = cap == 0 ? -member.MixedHalfThickness : member.MixedHalfThickness;
            MixedBand band = prism.Bands[cap == 0 ? 0 : prism.BandCount - 1];
            foreach (var span in band.Boundary.Spans)
                if (span.Kind == MeshSurfaceBoundaryDomain2D.SpanKind.Joined)
                    Keep(span, y, y, cap == 0 ? 1 : -1, ref selection);
        }

        void Keep(in MeshSurfaceBoundaryDomain2D.BoundarySpan span, Fixed64 lower, Fixed64 upper, int vertical,
            ref CapsulePlaneRaySelection retained)
        {
            _mixedSphereFaceFootprint = new(new(span.Start.X, Fixed64.Zero, span.Start.Y), new(span.End.X, Fixed64.Zero, span.End.Y));
            bool found = WideConvexPrismRelations.TryGetSphereBoundaryFaceContact(_mixedMoving!.Center, radius,
                MixedEmbedded2DGeometry.GetCenter3D(member), member.ConvexRotation, span.Start, span.End,
                prism.Offsets[(span.Second + 1) % prism.Offsets.Length], span.Interval, lower, upper,
                ref retained, out _, out var contact, out var parameter, out Fixed64 localY, vertical, _mixedSphereBoundaryAdmit);
            if (!found) return;
            int component = _mixedComponents[_mixedMember]; _mixedSpherePhysicalComponents.Add(component);
            // Height bands may share the same finite peak. Only an identical
            // point on the same original face is redundant; coplanar regions are not.
            var candidate = new SpherePeak(_mixedMember, span.First, span.Second, vertical, localY, parameter, contact);
            foreach (SpherePeak peak in _mixedSpherePeaks)
                if (peak.Member == candidate.Member && peak.Edge == candidate.Edge && peak.Vertical == candidate.Vertical
                    && peak.Height == candidate.Height && RationalSegmentInterval.CompareParameters(
                        peak.Numerator, peak.Denominator, candidate.Numerator, candidate.Denominator) == 0) return;
            _mixedSpherePeaks.Add(candidate);
        }
    }

    private bool AdmitMixedSphereBoundaryPoint(in RationalSegmentInterval parameter, Fixed64 localY,
        in WideDirection3d normal)
    {
        const int words = 32;
        Span<ulong> values = stackalloc ulong[8 * words]; Span<int> signs = stackalloc int[8];
        var n = ContactQuadratic.At(values, signs, 0, words); var d = ContactQuadratic.At(values, signs, 1, words);
        var height = ContactQuadratic.At(values, signs, 2, words); var heightD = ContactQuadratic.At(values, signs, 3, words);
        n.Set(parameter.WideLowerNumerator); d.Set(parameter.WideLowerDenominator);
        height.Set(Signed576.ExtendValue(Signed192.Raw(localY))); heightD.Set(Signed576.ExtendValue(Signed192.Signed(1)));
        // A blocked incident normal identifies a genuine concave cut. Flat
        // joins and convex rims stay with the original finite support owner.
        return !AdmitMixedSphereFan(_mixedSphereFaceFootprint, n, d, height, heightD, normal);
    }

    private bool AdmitMixedSpherePoint(in WideDirection3d normal, in WideConvexPrismRelations.ProjectionDepth depth, scoped ContactQuadratic x,
        scoped ContactQuadratic y, scoped ContactQuadratic z, scoped ContactQuadratic d)
    {
        // Exterior contacts require an actual finite boundary inside the sphere.
        // A far side through the solid is recovery, not another exterior row.
        if (!_mixedSphereInside && depth.Rational.Sign > 0) return false;
        MixedPrism prism = _mixedPrisms[_mixedMember];
        const int words = 32;
        Span<ulong> values = stackalloc ulong[12 * words]; Span<int> signs = stackalloc int[12];
        var nx = ContactQuadratic.At(values, signs, 0, words);
        var nz = ContactQuadratic.At(values, signs, 1, words);
        var term = ContactQuadratic.At(values, signs, 2, words);
        var side = ContactQuadratic.At(values, signs, 3, words);
        var parameter = ContactQuadratic.At(values, signs, 4, words);
        var divisor = ContactQuadratic.At(values, signs, 5, words);
        for (int edge = 0; edge < prism.Offsets.Length; edge++)
        {
            Vector2d a = prism.Offsets[edge], b = prism.Offsets[(edge + 1) % prism.Offsets.Length];
            x.CopyTo(nx); ContactQuadratic.Scale(d, Signed576.ExtendValue(Signed192.Raw(a.X)), term); nx.Add(term, -1);
            z.CopyTo(nz); ContactQuadratic.Scale(d, Signed576.ExtendValue(Signed192.Raw(a.Y)), term); nz.Add(term, -1);
            ContactQuadratic.Scale(nx, Signed576.ExtendValue(Signed192.Raw(b.Y - a.Y)), side);
            ContactQuadratic.Scale(nz, Signed576.ExtendValue(Signed192.Raw(b.X - a.X)), term); side.Add(term, -1);
            if (side.Signs[0] != 0) continue;
            Fixed64 delta = b.X != a.X ? b.X - a.X : b.Y - a.Y;
            (b.X != a.X ? nx : nz).CopyTo(parameter);
            ContactQuadratic.Scale(d, Signed576.ExtendValue(Signed192.Raw(delta)), divisor);
            if (delta < Fixed64.Zero) { parameter.MultiplySign(-1); divisor.MultiplySign(-1); }
            return AdmitMixedSphereFan(new(new(a.X, Fixed64.Zero, a.Y), new(b.X, Fixed64.Zero, b.Y)),
                parameter, divisor, y, d, normal);
        }
        return true;
    }

    private bool AdmitMixedSphereFan(in FixedSegment footprint, scoped ContactQuadratic n,
        scoped ContactQuadratic d, scoped ContactQuadratic height, scoped ContactQuadratic heightD,
        in WideDirection3d normal)
    {
        MixedPrism source = _mixedPrisms[_mixedMember];
        Signed832 support = WideOrientedBox.GetScaledHullSupport(source.NativeFrame, source.Points, normal, false);
        int vertex = 0;
        while (!WideOrientedBox.IsScaledHullSupport(source.NativeFrame.GetPoint(source.Points[vertex]), normal, support)) vertex++;
        const int words = 32;
        Span<ulong> values = stackalloc ulong[4 * words]; Span<int> signs = stackalloc int[4];
        var wall = ContactQuadratic.At(values, signs, 0, words);
        var term = ContactQuadratic.At(values, signs, 1, words);
        _mixedFan.FastClear(); _mixedFan.Add(_mixedMember);
        for (int cursor = 0; cursor < _mixedFan.Count; cursor++)
        {
            int member = _mixedFan[cursor]; MixedPrism prism = _mixedPrisms[member];
            if (member != _mixedMember && !SupportsMixedFan(prism.NativeFrame, source.NativeFrame,
                source.Points[vertex], prism.Points, normal)) return false;
            Fixed64 delta = _mixedLocal[member].MixedSlabCenterY - _mixedLocal[_mixedMember].MixedSlabCenterY;
            for (int bandIndex = 0; bandIndex < prism.BandCount; bandIndex++)
            {
                MixedBand band = prism.Bands[bandIndex];
                height.CopyTo(wall); ContactQuadratic.Scale(heightD, Signed576.ExtendValue(Signed192.Raw(delta + band.Lower)), term); wall.Add(term, -1);
                if (wall.Signs[0] < 0) continue;
                height.CopyTo(wall); ContactQuadratic.Scale(heightD, Signed576.ExtendValue(Signed192.Raw(delta + band.Upper)), term); wall.Add(term, -1);
                if (wall.Signs[0] > 0) continue;
                foreach (var span in band.Boundary.Spans)
                {
                    if (span.Kind != MeshSurfaceBoundaryDomain2D.SpanKind.Joined || _mixedFan.Contains(span.OtherMember)) continue;
                    var edge = new FixedSegment(new(span.Start.X, Fixed64.Zero, span.Start.Y), new(span.End.X, Fixed64.Zero, span.End.Y));
                    if (WideTriangleRelations.LocateFrameEdgePoint(footprint, source.NativeFootFrame, n, d,
                        edge, prism.NativeFootFrame, span.Interval) >= 0) _mixedFan.Add(span.OtherMember);
                }
            }
        }
        return true;
    }
}
