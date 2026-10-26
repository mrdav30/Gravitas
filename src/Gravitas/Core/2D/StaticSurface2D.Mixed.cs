// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
using FixedMathSharp;
using FixedMathSharp.Geometry;
using Gravitas.Colliders;
using Gravitas.CollisionHandling;
using Gravitas.Support;
using SwiftCollections;
using System;
using System.Collections.Generic;

namespace Gravitas;

/// <content>Admits finite exposed prism supports before native mixed response selection.</content>
internal sealed partial class StaticSurface2D : IScaledConvexHullContactDomain
{
    private readonly SwiftList<LSCollider2D> _mixedLocal = new();
    private readonly SwiftList<MixedPrism> _mixedPrisms = new();
    private readonly SwiftList<MixedSample> _mixedSamples = new(), _mixedPool = new();
    private readonly SwiftList<MixedGroup> _mixedGroups = new();
    private readonly SwiftHashSet<int> _mixedOverlaps = new();
    private readonly SwiftList<RationalSegmentInterval> _mixedIntervals = new();
    private readonly SwiftList<int> _mixedFan = new();
    private readonly SwiftList<Fixed64> _mixedHeights = new();
    private int _mixedEdgeStart, _mixedEdgeEnd;
    private readonly RationalSegmentInterval[] _mixedEdgeClip = new RationalSegmentInterval[1];
    private static readonly RationalSegmentInterval[] MixedFullInterval = { default };
    private readonly Vector3d[] _mixedBox = new Vector3d[8];
    private Vector3d[] _mixedFace = new Vector3d[4];
    private int[] _mixedFaceTriangles = new int[6], _mixedFaceEdges = new int[8], _mixedComponents = Array.Empty<int>();
    private readonly ScaledHullPenetrationAdmission _mixedAdmit;
    private readonly PointSpanPenetrationAdmission _mixedAdmitRigid;
    private WidePointSpanPenetration _mixedRigidPenetration;
    private bool _mixedUsingRigid;
    private LSCollider? _mixedMoving;
    private ScaledRigidFrame3d _mixedMovingFrame;
    private uint _mixedMovingBroad, _mixedMovingDependency;
    private Vector3d _mixedMovingCenter;
    private FixedQuaternion _mixedMovingRotation;
    private bool _mixedCurrent, _mixedQuaternion;
    private Fixed64 _mixedReach, _mixedRequestedReach;
    private int _mixedMember, _mixedGroup, _mixedSupplier, _mixedPreparedCount = -1;
    private static readonly IComparer<MixedSample> MixedPointOrder = Comparer<MixedSample>.Create(static (a, b) =>
    {
        int order = a.Contact.SecondAnchor.CompareLocalFeature(b.Contact.SecondAnchor);
        return order != 0 ? order : a.Supplier.CompareTo(b.Supplier);
    });

    private ReadOnlySpan<Vector3d> MixedMovingPoints => _mixedMoving is LSMeshCollider mesh
        ? mesh.Mesh.GeometryVertices : _mixedBox;
    private ReadOnlySpan<int> MixedMovingTriangles => _mixedMoving is LSMeshCollider mesh
        ? mesh.Mesh.Triangles : WideOrientedBox.BoxHullTriangles;
    private ReadOnlySpan<int> MixedMovingEdges => _mixedMoving is LSMeshCollider mesh
        ? mesh.Mesh.ConvexSatEdgeVertexPairs : WideOrientedBox.BoxHullEdges;

    internal bool AppendMixed(LSCollider mover, LSCollider2D overlap, ContactManifold destination,
        int namespace3D, int namespace2D)
    {
        if (_members.Length == 1)
            return CollisionDetectionMixed.AppendManifold(mover, overlap, destination, namespace3D, namespace2D, false);
        bool current = _mixedCurrent && _mixedMoving == mover && _mixedMovingBroad == mover.BroadPhaseVersion
            && _mixedMovingDependency == (mover.CompoundOwner ?? mover).ContactDependencyVersion
            && _mixedMovingCenter == mover.Center && _mixedMovingRotation == mover.Rotation;
        if (!current) _mixedReach = mover.CanonicalCenteredProxyRadius * Fixed64.Two;
        while (true)
        {
            GatherMixed(mover, _mixedReach);
            if (current && _mixedCurrent) break;
            _mixedMoving = mover;
            _mixedMovingFrame = mover is LSMeshCollider mesh ? mesh.Mesh.GeometryFrame
                : ScaledRigidFrame3d.Rigid(mover.Center, mover.Rotation);
            if (mover is LSCuboidCollider box) WideOrientedBox.GetBoxHullVertices(box.OrientedBox.HalfExtents, _mixedBox);
            _mixedRequestedReach = _mixedReach;
            BuildMixed();
            if (_mixedRequestedReach > _mixedReach)
            {
                // A deep placement can project beyond the body's initial query.
                // Grow only to a considered native SAT exit, never a guessed iteration budget.
                _mixedReach = _mixedRequestedReach; current = false; continue;
            }
            _mixedMovingBroad = mover.BroadPhaseVersion; _mixedMovingDependency = (mover.CompoundOwner ?? mover).ContactDependencyVersion;
            _mixedMovingCenter = mover.Center; _mixedMovingRotation = mover.Rotation; _mixedCurrent = true;
            break;
        }
        for (int member = 0; member < _mixedLocal.Count; member++)
            if (_mixedLocal[member] == overlap && !_mixedPrisms[member].Joined)
                return CollisionDetectionMixed.AppendManifold(mover, overlap, destination, namespace3D, namespace2D, false);
        Span<int> chosen = stackalloc int[ContactManifold.MaxContactsPerGroup];
        for (int group = 0; group < _mixedGroups.Count; group++)
        {
            MixedGroup response = _mixedGroups[group];
            if (response.Carrier != overlap.Id) continue;
            _mixedPool.FastClear();
            foreach (MixedSample sample in _mixedSamples.AsReadOnlySpan()) if (sample.Group == group) _mixedPool.Add(sample);
            _mixedPool.SortInPlace(MixedPointOrder);
            int count = Math.Min(chosen.Length, _mixedPool.Count);
            for (int slot = 0; slot < count; slot++)
                chosen[slot] = slot == 0 ? 0 : MeshPlaneSampleReduction.ChooseNext(_mixedPool.AsReadOnlySpan(),
                    chosen[..slot], response.Axis, MixedPointOrder);
            for (int slot = 0; slot < count; slot++)
            {
                MixedSample sample = _mixedPool[chosen[slot]]; LSCollider2D supplier = _mixedLocal[sample.Supplier];
                FixedContactAnchors contact = sample.Contact;
                destination.AddContact(new ContactAnchor(contact.FirstAnchor), new ContactAnchor(contact.SecondAnchor),
                    contact.Depth, contact.Normal, mover.Material, supplier.Material, contact.DepthIsClamped,
                    featureNamespaceA: namespace3D, featureNamespaceB: namespace2D,
                    group: new ContactGroupKey(namespace3D, namespace2D, region: group, sourceIdB: supplier.Id + 1));
            }
        }
        return _mixedOverlaps.Contains(overlap.Id);
    }

    private void GatherMixed(LSCollider mover, Fixed64 reach)
    {
        FixedBoundBox bounds = FixedBoundBox.FromCenterAndScopeClippedToDomain(mover.Center,
            Vector3d.One * reach);
        _context.MixedCollisions.Collect2DCandidatesInMixedBounds(bounds.Min, bounds.Max, PhysicsLayerMask.All,
            _candidates, staticStyleOnly: true, cachePartitionRefresh: true);
        int previous = _mixedLocal.Count;
        _mixedLocal.FastClear();
        foreach (LSCollider2D candidate in _candidates.AsReadOnlySpan())
            if (candidate.StaticSurface == this && !candidate.IsTrigger
                && _context.MixedCollisions.RequireCollisionPair(mover.CompoundOwner ?? mover, candidate)) _mixedLocal.Add(candidate);
        if (previous != _mixedLocal.Count) _mixedCurrent = false;
        bool quaternion = mover is LSMeshCollider;
        bool prepared = _mixedQuaternion == quaternion && _mixedPreparedCount == _mixedLocal.Count;
        while (_mixedPrisms.Count < _mixedLocal.Count) _mixedPrisms.Add(new MixedPrism());
        for (int member = 0; member < _mixedLocal.Count; member++)
            prepared &= _mixedPrisms[member].Matches(_mixedLocal[member]);
        if (!prepared) { _mixedQuaternion = quaternion; PrepareMixed(); _mixedCurrent = false; }

    }

    private void PrepareMixed()
    {
        _mixedPreparedCount = _mixedLocal.Count;
        if (_mixedComponents.Length < _mixedLocal.Count) Array.Resize(ref _mixedComponents, _mixedLocal.Count);
        for (int member = 0; member < _mixedLocal.Count; member++)
        {
            _mixedPrisms[member].Prepare(_mixedLocal[member], _mixedQuaternion);
            _mixedPrisms[member].Joined = false;
            PrepareMixedBands(member);
            _mixedComponents[member] = member;
        }
        for (int member = 0; member < _mixedLocal.Count; member++)
          for (int band = 0; band < _mixedPrisms[member].BandCount; band++)
            foreach (var span in _mixedPrisms[member].Bands[band].Boundary.Spans)
                if (span.Kind == MeshSurfaceBoundaryDomain2D.SpanKind.Joined)
                {
                    _mixedPrisms[member].Joined = true;
                    int from = _mixedComponents[span.OtherMember], to = _mixedComponents[member];
                    for (int index = 0; index < _mixedLocal.Count; index++)
                        if (_mixedComponents[index] == from) _mixedComponents[index] = to;
                }
    }

    private void PrepareMixedBands(int member)
    {
        MixedPrism prism = _mixedPrisms[member];
        Fixed64 half = _mixedLocal[member].MixedHalfThickness;
        _mixedHeights.FastClear(); _mixedHeights.Add(-half); _mixedHeights.Add(half);
        for (int other = 0; other < _mixedLocal.Count; other++)
            if (TryGetMixedHeights(member, other, out Fixed64 lower, out Fixed64 upper))
            {
                if (lower > -half && lower < half) _mixedHeights.Add(lower);
                if (upper > -half && upper < half) _mixedHeights.Add(upper);
            }
        _mixedHeights.SortInPlace();
        int count = 0;
        for (int height = 1; height < _mixedHeights.Count; height++)
        {
            Fixed64 lower = _mixedHeights[height - 1], upper = _mixedHeights[height];
            if (lower == upper) continue;
            if (prism.Bands.Count == count) prism.Bands.Add(new MixedBand());
            MixedBand band = prism.Bands[count++];
            band.Lower = lower; band.Upper = upper;
            band.Boundary.Build(_mixedLocal.AsReadOnlySpan(), member, mixed: _mixedQuaternion,
                lowerHeight: lower, upperHeight: upper);
        }
        prism.BandCount = count;
    }

    private void BuildMixed()
    {
        _mixedSamples.FastClear(); _mixedGroups.FastClear(); _mixedOverlaps.Clear();
        _mixedRegions.FastClear(); _mixedRegionParents.FastClear();
        for (_mixedMember = 0; _mixedMember < _mixedLocal.Count; _mixedMember++)
        {
            MixedPrism prism = _mixedPrisms[_mixedMember];
            if (!_mixedMoving!.Bounds.Intersects(_mixedLocal[_mixedMember].MixedBounds3D)) continue;
            bool hit = _mixedQuaternion ? FindMixedPenetration(prism.Frame, prism, out var penetration)
                : FindMixedPenetration(prism.NativeFrame, prism, out penetration);
            if (!hit) continue;
            _mixedOverlaps.Add(_mixedLocal[_mixedMember].Id);
            if (!prism.Joined) { HasMixedReach(penetration); continue; }
            if (!AdmitMixedAxis(penetration))
            {
                if (_mixedRequestedReach > _mixedReach) return;
                if (_mixedQuaternion) FindMixedPenetration(prism.Frame, prism, out penetration, _mixedAdmit);
                else FindMixedPenetration(prism.NativeFrame, prism, out penetration, _mixedAdmit);
            }
            if (_mixedRequestedReach > _mixedReach) return;
            if (!penetration.HasValue) continue;
            _mixedGroup = FindMixedGroup(penetration);
            if (IsMixedFaceAxis(penetration) && HasMixedFaceRegion(_mixedGroup)) continue;
            if (_mixedGroup == _mixedGroups.Count)
                _mixedGroups.Add(new(_mixedComponents[_mixedMember], _mixedLocal[_mixedMember].Id, penetration));
            VisitMixed(penetration, false);
        }
        CompleteMixedRegions();
    }

    private bool FindMixedPenetration<TFrame>(in TFrame frame, MixedPrism prism, out WideScaledHullPenetration penetration,
        ScaledHullPenetrationAdmission? admission = null) where TFrame : struct, IForwardHullFrame3d
    {
        _mixedUsingRigid = _mixedMovingFrame.IsRigid && prism.Rigid;
        if (_mixedUsingRigid)
        {
            bool hit = WideOrientedBox.TryGetConvexHullPenetration(_mixedMovingFrame.RigidOrigin, _mixedMovingFrame.Rotation,
                MixedMovingPoints, MixedMovingTriangles, MixedMovingEdges, prism.Frame.RigidOrigin, prism.Frame.Rotation,
                prism.Points, prism.Triangles, prism.Edges, out WidePointSpanPenetration selected,
                skipParallelDirections: true, admission: admission == null ? null : _mixedAdmitRigid);
            // Admission borrows the current candidate for rigid visitation;
            // it must not alias the SAT accumulator through this field.
            _mixedRigidPenetration = selected;
            penetration = new(_mixedRigidPenetration);
            return hit;
        }
        return WideOrientedBox.TryGetConvexHullPenetration(_mixedMovingFrame, MixedMovingPoints, MixedMovingTriangles,
            MixedMovingEdges, frame, prism.Points, prism.Triangles, prism.Edges, out penetration,
            skipParallelDirections: true, admission: admission);
    }

    private bool AdmitMixedRigidAxis(in WidePointSpanPenetration penetration)
    {
        _mixedRigidPenetration = penetration;
        return AdmitMixedAxis(new WideScaledHullPenetration(penetration));
    }

    private bool AdmitMixedAxis(in WideScaledHullPenetration penetration) =>
        HasMixedReach(penetration) && (IsMixedFaceAxis(penetration) && HasMixedFaceRegion(FindMixedGroup(penetration))
        || VisitMixed(penetration, true) > 0);

    private bool HasMixedReach(in WideScaledHullPenetration penetration)
    {
        // Every source support point is within its radius; projecting it to
        // this plane adds at most the exact depth. One raw unit raises the
        // nearest depth outward; another covers final anchor rounding.
        Fixed64 reach = _mixedMoving!.CanonicalCenteredProxyRadius
            + penetration.GetRoundedDepth(out _) + Fixed64.MinIncrement + Fixed64.MinIncrement;
        if (reach <= _mixedReach) return true;
        if (reach > _mixedRequestedReach) _mixedRequestedReach = reach;
        return false;
    }

    private int VisitMixed(in WideScaledHullPenetration penetration, bool admissionOnly)
    {
        // Select the original prism's actual feature. A remote coplanar face
        // must not suppress a disconnected vertex/edge supplier on this plane.
        if (IsMixedFaceAxis(penetration)) return VisitMixedFaces(penetration, admissionOnly);
        MixedPrism prism = _mixedPrisms[_mixedMember];
        _mixedSupplier = _mixedMember;
        return _mixedQuaternion ? VisitMixedEdges(prism.Frame, prism, penetration, admissionOnly)
            : VisitMixedEdges(prism.NativeFrame, prism, penetration, admissionOnly);
    }

    private bool IsMixedFaceAxis(in WideScaledHullPenetration penetration)
    {
        MixedPrism prism = _mixedPrisms[_mixedMember];
        WideDirection3d axis = penetration.Negate ? -penetration.Axis : penetration.Axis;
        return _mixedQuaternion ? SupportsFace(prism.Frame) : SupportsFace(prism.NativeFrame);
        bool SupportsFace<TFrame>(TFrame frame) where TFrame : struct, IForwardHullFrame3d
        {
            Signed832 support = WideOrientedBox.GetScaledHullSupport(frame, prism.Points, axis, false);
            for (int triangle = 0; triangle < prism.Triangles.Length; triangle += 3)
                if (WideOrientedBox.IsScaledHullSupport(frame.GetPoint(prism.Points[prism.Triangles[triangle]]), axis, support)
                    && WideOrientedBox.IsScaledHullSupport(frame.GetPoint(prism.Points[prism.Triangles[triangle + 1]]), axis, support)
                    && WideOrientedBox.IsScaledHullSupport(frame.GetPoint(prism.Points[prism.Triangles[triangle + 2]]), axis, support)) return true;
            return false;
        }
    }


    private int VisitMixedEdges<TFrame>(in TFrame frame, MixedPrism prism, in WideScaledHullPenetration penetration,
        bool admissionOnly) where TFrame : struct, IForwardHullFrame3d
    {
        WideDirection3d axis = penetration.Negate ? -penetration.Axis : penetration.Axis;
        Signed832 support = WideOrientedBox.GetScaledHullSupport(frame, prism.Points, axis, false);
        int count = 0; bool hasEdge = false;
        for (int edge = 0; edge < prism.Edges.Length; edge += 2)
        {
            int start = prism.Edges[edge], end = prism.Edges[edge + 1];
            if (!WideOrientedBox.IsScaledHullSupport(frame.GetPoint(prism.Points[start]), axis, support)
                || !WideOrientedBox.IsScaledHullSupport(frame.GetPoint(prism.Points[end]), axis, support)) continue;
            hasEdge = true;
            foreach (var clip in GetMixedEdgeIntervals(start, end))
            {
                int firstSample = _mixedSamples.Count;
                count += VisitMixedEdgeInterval(frame, prism, start, end, clip, penetration, admissionOnly);
                if (!admissionOnly) AddMixedRegion(_mixedMember, -1, default, default, default, firstSample,
                    edgeStart: start, edgeEnd: end, edgeInterval: clip);
                if (admissionOnly && count > 0) return count;
            }
        }
        if (hasEdge) return count;
        for (int vertex = 0; vertex < prism.Points.Length; vertex++)
        {
            if (!WideOrientedBox.IsScaledHullSupport(frame.GetPoint(prism.Points[vertex]), axis, support)) continue;
            int firstSample = _mixedSamples.Count;
            count += VisitMixedEdgeInterval(frame, prism, vertex, vertex, default, penetration, admissionOnly);
            if (!admissionOnly) AddMixedRegion(_mixedMember, -1, default, default, default, firstSample,
                edgeStart: vertex, edgeEnd: vertex);
            break;
        }
        return count;
    }

    private int VisitMixedEdgeInterval<TFrame>(in TFrame frame, MixedPrism prism, int start, int end,
        in RationalSegmentInterval clip, in WideScaledHullPenetration penetration, bool admissionOnly)
        where TFrame : struct, IForwardHullFrame3d
    {
        _mixedEdgeStart = start; _mixedEdgeEnd = end; _mixedEdgeClip[0] = clip;
        Span<Vector3d> points = stackalloc Vector3d[2] { prism.Points[start], prism.Points[end] };
        ReadOnlySpan<int> edges = stackalloc int[2] { 0, 1 };
        int count = WideOrientedBox.VisitConvexHullContacts(_mixedMovingFrame, _mixedMoving!.Center, MixedMovingPoints,
            MixedMovingTriangles, MixedMovingEdges, frame, MixedEmbedded2DGeometry.GetCenter3D(_mixedLocal[_mixedMember]),
            points, ReadOnlySpan<int>.Empty, edges, penetration, this,
            static (owner, contact) => owner.AddMixedSample(contact), this, admissionOnly);
        if (count == 0 && start != end)
            count = VisitMixedEdgeInterior(frame, points, clip, penetration, admissionOnly);
        return count;
    }

    private int VisitMixedEdgeInterior<TFrame>(in TFrame frame, ReadOnlySpan<Vector3d> points,
        in RationalSegmentInterval clip, in WideScaledHullPenetration penetration, bool admissionOnly)
        where TFrame : struct, IForwardHullFrame3d
    {
        Signed832 one = Signed832.ExtendValue(Signed192.Signed(1));
        Signed832 ln = clip.IsClipped ? clip.WideLowerNumerator : default, ld = clip.IsClipped ? clip.WideLowerDenominator : one;
        Signed832 un = clip.IsClipped ? clip.WideUpperNumerator : one, ud = clip.IsClipped ? clip.WideUpperDenominator : one;
        if (RationalSegmentInterval.CompareParameters(ln, ld, un, ud) >= 0) return 0;
        Span<ulong> values = stackalloc ulong[6 * 14]; Span<int> signs = stackalloc int[6];
        var n = ContactQuadratic.At(values, signs, 0, 14); var d = ContactQuadratic.At(values, signs, 1, 14);
        var term = ContactQuadratic.At(values, signs, 2, 14);
        // The exact mediant is strictly inside a nonempty retained band. Its
        // <833-bit values fit the existing triangle witness's 2048-bit banks.
        n.Set(ln); term.Set(un); n.Add(term); d.Set(ld); term.Set(ud); d.Add(term);
        WideDirection3d axis = penetration.Negate ? -penetration.Axis : penetration.Axis;
        if (!ContainsMixedPoint(0, 1, n, d, axis)) return 0;
        Signed832 support = WideOrientedBox.GetScaledHullSupport(_mixedMovingFrame, MixedMovingPoints, axis, true);
        RationalPoint3d a = frame.GetPoint(points[0]), b = frame.GetPoint(points[1]);
        ReadOnlySpan<Vector3d> moving = MixedMovingPoints; ReadOnlySpan<int> triangles = MixedMovingTriangles;
        for (int triangle = 0; triangle < triangles.Length; triangle += 3)
        {
            Vector3d x = moving[triangles[triangle]], y = moving[triangles[triangle + 1]], z = moving[triangles[triangle + 2]];
            if (!WideOrientedBox.IsScaledHullSupport(_mixedMovingFrame.GetPoint(x), axis, support)
                || !WideOrientedBox.IsScaledHullSupport(_mixedMovingFrame.GetPoint(y), axis, support)
                || !WideOrientedBox.IsScaledHullSupport(_mixedMovingFrame.GetPoint(z), axis, support)) continue;
            if (!WideTriangleRelations.TryGetProjectedFrameSegmentFaceAnchors(new(x, y, z), _mixedMovingFrame, a, b,
                default, default, MixedEmbedded2DGeometry.GetCenter3D(_mixedLocal[_mixedMember]), _mixedMoving!.Center,
                0, out FixedPointAnchor target, out FixedPointAnchor source, requirePhysicalTargetBoundary: false,
                materialize: !admissionOnly, wideN: n, wideD: d)) continue;
            if (!admissionOnly) AddMixedSample(new(source, target, axis.GetNormal(false),
                penetration.GetRoundedDepth(out bool clamped), clamped));
            return 1;
        }
        return 0;
    }

    private int VisitMixedFaces(in WideScaledHullPenetration penetration, bool admissionOnly)
    {
        int source = _mixedMember, count = 0;
        bool rigid = _mixedUsingRigid;
        MixedPrism original = _mixedPrisms[source];
        WideDirection3d axis = penetration.Negate ? -penetration.Axis : penetration.Axis;
        RationalPoint3d reference = _mixedQuaternion ? MixedSupportPoint(original.Frame, original, axis)
            : MixedSupportPoint(original.NativeFrame, original, axis);
        for (int member = 0; member < _mixedLocal.Count; member++)
        {
            if (_mixedComponents[member] != _mixedComponents[source] || !MaySupplyMixedFace(member, axis)) continue;
            MixedPrism prism = _mixedPrisms[member];
            RationalPoint3d support = _mixedQuaternion ? MixedSupportPoint(prism.Frame, prism, axis)
                : MixedSupportPoint(prism.NativeFrame, prism, axis);
            Signed192 denominator = _mixedQuaternion ? original.Frame.RotationDenominator : original.NativeFrame.RotationDenominator;
            if (!WideTriangleRelations.DotScaledDirections(axis,
                WideTriangleRelations.GetScaledPointDifference(support, reference, denominator)).IsZero) continue;
            _mixedMember = member;
            _mixedUsingRigid = _mixedMovingFrame.IsRigid && prism.Rigid;
            count += _mixedQuaternion ? VisitMixedFaces(prism.Frame, prism, penetration, admissionOnly)
                : VisitMixedFaces(prism.NativeFrame, prism, penetration, admissionOnly);
            if (admissionOnly && count > 0) break;
        }
        _mixedMember = source; _mixedUsingRigid = rigid;
        return count;
    }

    private static RationalPoint3d MixedSupportPoint<TFrame>(in TFrame frame, MixedPrism prism,
        in WideDirection3d axis) where TFrame : struct, IForwardHullFrame3d
    {
        Signed832 support = WideOrientedBox.GetScaledHullSupport(frame, prism.Points, axis, maximize: false);
        int vertex = 0;
        // Admitted nonempty geometry has a vertex attaining its exact support.
        while (!WideOrientedBox.IsScaledHullSupport(frame.GetPoint(prism.Points[vertex]), axis, support)) vertex++;
        return frame.GetPoint(prism.Points[vertex]);
    }

    private bool MaySupplyMixedFace(int member, in WideDirection3d normal)
    {
        FixedBoundBox bounds = _mixedLocal[member].MixedBounds3D;
        for (int axis = 0; axis < 3; axis++)
            if ((axis == 0 ? normal.X : axis == 1 ? normal.Y : normal.Z).IsZero
                && (bounds.Max[axis] < _mixedMoving!.Bounds.Min[axis]
                    || bounds.Min[axis] > _mixedMoving.Bounds.Max[axis])) return false;
        return true;
    }

    private int VisitMixedFaces<TFrame>(in TFrame frame, MixedPrism prism, in WideScaledHullPenetration penetration,
        bool admissionOnly) where TFrame : struct, IForwardHullFrame3d
    {
        WideDirection3d axis = penetration.Negate ? -penetration.Axis : penetration.Axis;
        Signed832 support = WideOrientedBox.GetScaledHullSupport(frame, prism.Points, axis, maximize: false);
        int emitted = 0;
        _mixedSupplier = _mixedMember;
        for (int face = 0; face < prism.Offsets.Length + 2; face++)
        {
            int count = face < 2 ? prism.Offsets.Length : 4;
            if (_mixedFace.Length < count) Array.Resize(ref _mixedFace, count);
            if (_mixedFaceTriangles.Length < 3 * (count - 2)) Array.Resize(ref _mixedFaceTriangles, 3 * (count - 2));
            if (_mixedFaceEdges.Length < 2 * count) Array.Resize(ref _mixedFaceEdges, 2 * count);
            for (int vertex = 0; vertex < count; vertex++)
            {
                int edge = face - 2, next = (edge + 1) % prism.Offsets.Length;
                int index = face < 2 ? 2 * vertex + face
                    : vertex == 0 ? 2 * edge : vertex == 1 ? 2 * next : vertex == 2 ? 2 * next + 1 : 2 * edge + 1;
                _mixedFace[vertex] = prism.Points[index];
                _mixedFaceEdges[2 * vertex] = vertex; _mixedFaceEdges[2 * vertex + 1] = (vertex + 1) % count;
                if (vertex < 2) continue;
                int triangle = 3 * (vertex - 2);
                _mixedFaceTriangles[triangle] = 0; _mixedFaceTriangles[triangle + 1] = vertex - 1; _mixedFaceTriangles[triangle + 2] = vertex;
            }
            bool supporting = true;
            for (int vertex = 0; vertex < count; vertex++)
                supporting &= WideArithmetic.SubtractSigned832(WideOrientedBox.ProjectScaledHullPoint(
                    frame.GetPoint(_mixedFace[vertex]), axis, Signed192.One), support).IsZero;
            if (!supporting) continue;
            if (face >= 2)
            {
                for (int bandIndex = 0; bandIndex < prism.BandCount; bandIndex++)
                {
                    MixedBand band = prism.Bands[bandIndex];
                    foreach (var span in band.Boundary.Spans)
                    {
                        if (span.First != face - 2) continue;
                        if (span.Kind != MeshSurfaceBoundaryDomain2D.SpanKind.Joined)
                            emitted += VisitMixedSide(frame, span, band.Lower, band.Upper, penetration, admissionOnly, bandIndex);
                        if (admissionOnly && emitted > 0) return emitted;
                    }
                }
                continue;
            }
            int firstSample = _mixedSamples.Count;
            emitted += _mixedUsingRigid
                ? VisitMixedRigidFace(prism, count, admissionOnly)
                : WideOrientedBox.VisitConvexHullContacts(_mixedMovingFrame, _mixedMoving!.Center,
                MixedMovingPoints, MixedMovingTriangles, MixedMovingEdges, frame,
                MixedEmbedded2DGeometry.GetCenter3D(_mixedLocal[_mixedMember]), _mixedFace.AsSpan(0, count),
                _mixedFaceTriangles.AsSpan(0, 3 * (count - 2)), _mixedFaceEdges.AsSpan(0, 2 * count), penetration,
                this, static (owner, contact) => owner.AddMixedSample(contact), admissionOnly: admissionOnly);
            if (!admissionOnly) AddMixedRegion(_mixedMember, face, default, _mixedFace[0].Y, _mixedFace[0].Y, firstSample);
            if (admissionOnly && emitted > 0) return emitted;
        }
        return emitted;
    }

    private int VisitMixedSide<TFrame>(in TFrame frame, in MeshSurfaceBoundaryDomain2D.BoundarySpan span,
        Fixed64 lower, Fixed64 upper, in WideScaledHullPenetration penetration, bool admissionOnly, int band)
        where TFrame : struct, IForwardHullFrame3d
    {
        MixedPrism prism = _mixedPrisms[_mixedMember];
        int firstSample = _mixedSamples.Count;
        int emitted;
        if (_mixedUsingRigid && !span.Interval.IsClipped
            && lower == -_mixedLocal[_mixedMember].MixedHalfThickness && upper == _mixedLocal[_mixedMember].MixedHalfThickness)
            emitted = VisitMixedRigidFace(prism, 4, admissionOnly);
        else
        {
            FixedTriangle rectangle = new(new Vector3d(span.Start.X, lower, span.Start.Y),
                new(span.End.X, lower, span.End.Y), new(span.Start.X, upper, span.Start.Y));
            emitted = WideOrientedBox.VisitProjectedHullRectangleContacts(_mixedMovingFrame, _mixedMoving!.Center,
                MixedMovingPoints, MixedMovingTriangles, MixedMovingEdges, frame,
                MixedEmbedded2DGeometry.GetCenter3D(_mixedLocal[_mixedMember]), rectangle, span.Interval,
                penetration, this, static (owner, contact) => owner.AddMixedSample(contact), admissionOnly);
        }
        if (!admissionOnly) AddMixedRegion(_mixedMember, span.First + 2, span, lower, upper, firstSample, band);
        return emitted;
    }

    private int VisitMixedRigidFace(MixedPrism prism, int count, bool admissionOnly) =>
        WideOrientedBox.VisitConvexHullContacts(_mixedMovingFrame.RigidOrigin, _mixedMovingFrame.Rotation,
            MixedMovingPoints, MixedMovingTriangles, MixedMovingEdges, prism.Frame.RigidOrigin, prism.Frame.Rotation,
            _mixedFace.AsSpan(0, count), _mixedFaceTriangles.AsSpan(0, 3 * (count - 2)), _mixedFaceEdges.AsSpan(0, 2 * count),
            _mixedRigidPenetration, this, static (owner, contact) => owner.AddMixedSample(contact), admissionOnly: admissionOnly);

    private int FindMixedGroup(in WideScaledHullPenetration penetration)
    {
        WideDirection3d axis = penetration.Negate ? -penetration.Axis : penetration.Axis;
        for (int group = 0; group < _mixedGroups.Count; group++)
        {
            MixedGroup existing = _mixedGroups[group];
            if (existing.Component == _mixedComponents[_mixedMember] && WideDirection3d.AreParallel(axis, existing.Axis)
                && WideDirection3d.Dot(axis, existing.Axis).Sign > 0
                && !existing.Penetration.ShouldReplace(penetration.Overlap, penetration.SquaredAxisLength, penetration.CommonDenominator)
                && !penetration.ShouldReplace(existing.Penetration.Overlap, existing.Penetration.SquaredAxisLength,
                    existing.Penetration.CommonDenominator)) return group;
        }
        return _mixedGroups.Count;
    }

    ReadOnlySpan<RationalSegmentInterval> IScaledConvexHullContactDomain.GetEdgeIntervals(bool first, int start, int end,
        in WideDirection3d normal)
        => first ? MixedFullInterval : _mixedEdgeClip;

    private ReadOnlySpan<RationalSegmentInterval> GetMixedEdgeIntervals(int start, int end)
    {
        _mixedIntervals.FastClear();
        MixedPrism prism = _mixedPrisms[_mixedMember];
        Vector3d a = prism.Points[start], b = prism.Points[end];
        if (a.Y == b.Y)
        {
          for (int bandIndex = 0; bandIndex < prism.BandCount; bandIndex++)
            foreach (var span in prism.Bands[bandIndex].Boundary.Spans)
                if (span.Interval.TryMapToSegment(new FixedSegment(new(span.Start.X, a.Y, span.Start.Y),
                    new(span.End.X, a.Y, span.End.Y)), new FixedSegment(a, b), out var mapped)) _mixedIntervals.Add(mapped);
        }
        else
        {
            // Neighbor caps split an original vertical edge into exact bands.
            for (int bandIndex = 0; bandIndex < prism.BandCount; bandIndex++)
            {
                MixedBand band = prism.Bands[bandIndex];
                Fixed64 lower = band.Lower - a.Y, upper = band.Upper - a.Y;
                // Original vertical prism edges are always bottom to top.
                Signed320 denominator = Signed320.ExtendValue(Signed192.Raw(b.Y - a.Y));
                _mixedIntervals.Add(new(Signed320.ExtendValue(Signed192.Raw(lower)), denominator,
                    Signed320.ExtendValue(Signed192.Raw(upper)), denominator));
            }
        }
        return _mixedIntervals.AsReadOnlySpan();
    }

    bool IScaledConvexHullContactDomain.ContainsPoint(bool first, int start, int end, in Signed832 numerator,
        in Signed832 denominator, in WideDirection3d normal)
    {
        System.Diagnostics.Debug.Assert(!first);
        Span<ulong> values = stackalloc ulong[52]; Span<int> signs = stackalloc int[4];
        var n = ContactQuadratic.At(values, signs, 0, 13); var d = ContactQuadratic.At(values, signs, 1, 13);
        n.Set(numerator); d.Set(denominator);
        return ContainsMixedPoint(start, end, n, d, normal);
    }

    bool IScaledConvexHullContactDomain.ContainsPoint(bool first, int start, int end, scoped ContactQuadratic numerator,
        scoped ContactQuadratic denominator, in WideDirection3d normal) =>
        first || ContainsMixedPoint(start, end, numerator, denominator, normal);

    private bool ContainsMixedPoint(int start, int end, scoped ContactQuadratic n, scoped ContactQuadratic d,
        in WideDirection3d normal)
    {
        // This domain belongs to the current original edge, represented by the
        // two-point visitor as indices 0/1. Preserve its exact parameter.
        if (start == end)
            {
                Span<ulong> storage = stackalloc ulong[52]; Span<int> signs = stackalloc int[4];
                var endpoint = ContactQuadratic.At(storage, signs, 0, 13); var one = ContactQuadratic.At(storage, signs, 1, 13);
                endpoint.Set(Signed576.ExtendValue(Signed192.Signed(start))); one.Set(Signed576.ExtendValue(Signed192.Signed(1)));
                if (_mixedEdgeClip[0].IsClipped && !_mixedEdgeClip[0].Contains(endpoint, one, ReadOnlySpan<ulong>.Empty)) return false;
                start = end = start == 0 ? _mixedEdgeStart : _mixedEdgeEnd;
            }
            else
            {
                if (_mixedEdgeClip[0].IsClipped && !_mixedEdgeClip[0].Contains(n, d, ReadOnlySpan<ulong>.Empty)) return false;
                start = _mixedEdgeStart; end = _mixedEdgeEnd;
            }
        MixedPrism source = _mixedPrisms[_mixedMember];
        Vector3d a = source.Points[start], b = source.Points[end];
        FixedSegment footprint = new(new(a.X, Fixed64.Zero, a.Z), new(b.X, Fixed64.Zero, b.Z));
        _mixedFan.FastClear(); _mixedFan.Add(_mixedMember);
        for (int cursor = 0; cursor < _mixedFan.Count; cursor++)
        {
            int member = _mixedFan[cursor]; MixedPrism prism = _mixedPrisms[member];
            if (member != _mixedMember)
            {
                // Every queried target edge lies on the selected support plane;
                // its original endpoint is an exact projection reference too.
                bool supports = _mixedQuaternion ? SupportsMixedFan(prism.Frame, source.Frame, a, prism.Points, normal)
                    : SupportsMixedFan(prism.NativeFrame, source.NativeFrame, a, prism.Points, normal);
                if (!supports) return false;
            }
          for (int bandIndex = 0; bandIndex < prism.BandCount; bandIndex++)
          {
            MixedBand band = prism.Bands[bandIndex];
            // A reached fan member already contains the queried height. Each
            // prepared band is covered completely by every Joined neighbor.
            Fixed64 delta = _mixedLocal[member].MixedSlabCenterY - _mixedLocal[_mixedMember].MixedSlabCenterY;
            if (!MixedHeightContains(a.Y, b.Y, n, d, delta + band.Lower, delta + band.Upper)) continue;
            foreach (var span in band.Boundary.Spans)
            {
                if (span.Kind != MeshSurfaceBoundaryDomain2D.SpanKind.Joined || _mixedFan.Contains(span.OtherMember)) continue;
                FixedSegment edge = new(new(span.Start.X, Fixed64.Zero, span.Start.Y), new(span.End.X, Fixed64.Zero, span.End.Y));
                int location = _mixedQuaternion
                    ? WideTriangleRelations.LocateFrameEdgePoint(footprint, source.FootFrame, n, d, edge, prism.FootFrame, span.Interval)
                    : WideTriangleRelations.LocateFrameEdgePoint(footprint, source.NativeFootFrame, n, d, edge, prism.NativeFootFrame, span.Interval);
                if (location >= 0) _mixedFan.Add(span.OtherMember);
            }
          }
        }
        return true;
    }

    private static bool SupportsMixedFan<TFrame>(in TFrame frame, in TFrame source, Vector3d reference,
        ReadOnlySpan<Vector3d> points, in WideDirection3d normal) where TFrame : struct, IForwardHullFrame3d
    {
        RationalPoint3d origin = source.GetPoint(reference);
        foreach (Vector3d vertex in points)
            if (WideTriangleRelations.DotScaledDirections(normal, WideTriangleRelations.GetScaledPointDifference(
                frame.GetPoint(vertex), origin, source.RotationDenominator)).Sign < 0) return false;
        return true;
    }

    private bool TryGetMixedHeights(int source, int other, out Fixed64 lower, out Fixed64 upper)
    {
        LSCollider2D a = _mixedLocal[source], b = _mixedLocal[other];
        lower = upper = default;
        if (a.MixedBounds3D.Max.Y < b.MixedBounds3D.Min.Y || a.MixedBounds3D.Min.Y > b.MixedBounds3D.Max.Y) return false;
        // Intersecting bounded extrusions have a bounded center difference.
        // Absolute cap coordinates may exceed Fixed64 and must not be clamped.
        Fixed64 delta = b.MixedSlabCenterY - a.MixedSlabCenterY;
        lower = delta - b.MixedHalfThickness; upper = delta + b.MixedHalfThickness;
        return true;
    }

    private static bool MixedHeightContains(Fixed64 first, Fixed64 second, scoped ContactQuadratic n,
        scoped ContactQuadratic d, Fixed64 minimum, Fixed64 maximum)
    {
        if (second < minimum || first > maximum) return false;
        if (first == second) return true;
        Fixed64 lower = minimum > first ? minimum : first, upper = maximum < second ? maximum : second;
        Signed320 denominator = Signed320.ExtendValue(Signed192.Raw(second - first));
        var interval = new RationalSegmentInterval(Signed320.ExtendValue(Signed192.Raw(lower - first)), denominator,
            Signed320.ExtendValue(Signed192.Raw(upper - first)), denominator);
        return interval.Contains(n, d, ReadOnlySpan<ulong>.Empty);
    }

    private void AddMixedSample(FixedContactAnchors contact)
    {
        bool represented = contact.SecondAnchor.TryGetPointInFrame(_mixedMoving!.Center, out Vector3d point);
        System.Diagnostics.Debug.Assert(represented);
        _mixedSamples.Add(new(_mixedGroup, _mixedSupplier, point, contact));
    }

    private readonly struct MixedSample : IPlaneContactSample
    {
        internal readonly int Group, Supplier;
        internal readonly FixedContactAnchors Contact;
        public Vector3d Point { get; }
        internal MixedSample(int group, int supplier, Vector3d point, FixedContactAnchors contact)
        { Group = group; Supplier = supplier; Point = point; Contact = contact; }
    }

    private readonly struct MixedGroup
    {
        internal readonly int Component, Carrier;
        internal readonly WideScaledHullPenetration Penetration;
        internal WideDirection3d Axis => Penetration.Negate ? -Penetration.Axis : Penetration.Axis;
        internal MixedGroup(int component, int carrier, WideScaledHullPenetration penetration)
        { Component = component; Carrier = carrier; Penetration = penetration; }
    }

    private sealed class MixedPrism
    {
        internal readonly SwiftList<MixedBand> Bands = new();
        internal int BandCount;
        internal Vector2d[] Offsets = Array.Empty<Vector2d>();
        internal Vector3d[] Points = Array.Empty<Vector3d>();
        internal int[] Triangles = Array.Empty<int>(), Edges = Array.Empty<int>();
        internal ScaledRigidFrame3d Frame;
        internal ScaledRigidFrame3d FootFrame;
        internal NativePrismYawFrame3d NativeFrame, NativeFootFrame;
        internal bool Rigid, Joined;
        private int _id;
        private long _lifetime;
        private uint _broad, _dependency;
        internal bool Matches(LSCollider2D member) => _id == member.Id && _lifetime == member.LifetimeVersion
            && _broad == member.BroadPhaseVersion && _dependency == member.ContactDependencyVersion;
        internal void Prepare(LSCollider2D member, bool quaternion)
        {
            int count = member.VertexCount;
            if (Offsets.Length != count)
            {
                Offsets = new Vector2d[count]; Points = new Vector3d[2 * count];
                Triangles = new int[12 * count - 12]; Edges = new int[6 * count];
            }
            for (int vertex = 0; vertex < count; vertex++) Offsets[vertex] = member.GetScaledLocalVertexUnchecked(vertex);
            CollisionDetectionMixed.FillPrismGeometry(Offsets, member.MixedHalfThickness, Points, Triangles, Edges);
            Vector3d center = MixedEmbedded2DGeometry.GetCenter3D(member);
            Rigid = quaternion || member.ConvexRotation == Fixed64.Zero;
            if (Rigid) Frame = ScaledRigidFrame3d.Rigid(center, FixedQuaternion.FromAxisAngle(Vector3d.Up, -member.ConvexRotation));
            Vector3d footprint = new(center.X, Fixed64.Zero, center.Z);
            if (quaternion) FootFrame = ScaledRigidFrame3d.Rigid(footprint, Frame.Rotation);
            else { NativeFrame = new(center, member.ConvexRotation); NativeFootFrame = new(footprint, member.ConvexRotation); }
            _id = member.Id; _lifetime = member.LifetimeVersion; _broad = member.BroadPhaseVersion;
            _dependency = member.ContactDependencyVersion;
        }
    }

    private sealed class MixedBand
    {
        internal readonly MeshSurfaceBoundaryDomain2D Boundary = new();
        internal Fixed64 Lower, Upper;
    }
}
