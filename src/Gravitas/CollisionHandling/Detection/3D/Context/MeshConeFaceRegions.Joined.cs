//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================
using FixedMathSharp;
using FixedMathSharp.Geometry;
using Gravitas.Colliders;
using SwiftCollections;
using SwiftCollections.Query;
using System;
using Member = Gravitas.CollisionHandling.MeshJoinedSurfaceRegions<FixedMathSharp.Geometry.ConePlaneRayFrame>.Member;

namespace Gravitas.CollisionHandling;

/// <content>Visits declared native member domains through the existing cone event and sample reducer.</content>
internal sealed partial class MeshConeFaceRegions
{
    private MeshJoinedSurfaceRegions<ConePlaneRayFrame>? _joined;
    private readonly SwiftHashSet<int> _visitedJoinedRegions = new();
    private int _currentMember = -1, _currentSupplier = -1, _currentRegion, _otherMember, _currentSpan;
    private ConePlaneRayEventVisitor? _joinedVisitor, _joinVisitor;

    internal void ReleaseJoined() { _joined = null; RegionCount = 0; }

    internal void BuildJoined(ReadOnlySpan<(LSMeshCollider Collider, int Triangle)> sources,
        LSConeCollider cone, ReadOnlySpan<PhysicsMesh> meshes,
        SwiftDictionary<(PhysicsMesh, int), MeshSurfaceBoundaryDomain> boundaries)
    {
        var state = _joined ??= new();
        state.Prepare(sources, meshes, boundaries);
        state.PrepareResponseComponents();
        // Response batching does not certify complete finite cone exits.
        // The trusted finite-region union below owns perimeter suppression.
        foreach (Member member in state.Members.AsReadOnlySpan()[..state.Inputs.Count])
            Array.Clear(member.JoinedSpans, 0, member.Boundary.Spans.Length);
        var bounds = new FixedBoundVolume(cone.BoundsMin, cone.BoundsMax);
        for (int index = 0; index < state.Inputs.Count; index++)
        {
            Member member = state.Members[index]; PhysicsMesh mesh = member.Collider.Mesh;
            mesh.GetGeometryTriangleVertices(member.Triangle, out Vector3d a, out Vector3d b, out Vector3d c);
            member.Frame = new(new FixedTriangle(a, b, c), mesh.GeometryFrame,
                cone.Center, cone.Rotation, cone.Height, cone.ScaledRadius);
            member.Admitted.FastClear();
            member.Collider.GetTrianglesInBounds(bounds, state.Candidates);
            foreach (int triangle in state.Candidates.AsReadOnlySpan())
            {
                if (mesh.GetManifoldSurfaceOwner(triangle) != member.Owner) continue;
                mesh.GetGeometryTriangleVertices(triangle, out a, out b, out c);
                if (ConePlaneRayEvents.IntersectsTriangle(member.Frame, new FixedTriangle(a, b, c))) member.Admitted.Add(triangle);
            }
            member.Connectivity.Build(mesh, member.Triangle, member.Frame, member.Admitted.AsReadOnlySpan());
            if (member.Connectivity.RegionCount > 0) state.AddAdmittedMember(index);
            state.AddRegions(index);
        }
        bool wide = false;
        foreach (Member member in state.Members.AsReadOnlySpan()[..state.Inputs.Count])
        {
            wide |= member.Frame.UsesWideBanks;
            foreach (var span in member.Boundary.Spans) wide |= span.Interval.HasWideBounds;
        }
        if (wide)
            foreach (Member member in state.Members.AsReadOnlySpan()[..state.Inputs.Count])
                member.Frame = member.Frame.WithWideParameters();
        _fieldWords = state.Members[0].Frame.FieldWords; _rootWords = state.Members[0].Frame.RootWords;
        JoinFiniteSpans();
        state.CompleteRegions();
        RegionCount = state.Roots.Count; HasBoundaryIntersection = false;
        Reserve(RegionCount); Array.Clear(_winners, 0, 2 * RegionCount); _poolEvents.FastClear();
        _samplingOrigin = cone.Center;
        _joinedVisitor ??= AdmitJoinedPoint;
        for (int region = 0; region < RegionCount; region++)
        {
            _currentRegion = region; _currentMember = state.NodeMembers[state.Roots[region]];
            _frame = GetFrame(_currentMember); _currentSpan = -1;
            _wholeConeSamplingRangeRepresentable = ConePlaneRayPointMaterialization.IsRangeRepresentable(_frame, _samplingOrigin);
            ConePlaneRayEvents.VisitEvents(ConePlaneRayEventSource.Plane, _frame, _joinedVisitor);
        }
        _currentRegion = -1;
        for (int member = 0; member < state.Inputs.Count; member++)
        {
            Member source = state.Members[member];
            if (source.Connectivity.RegionCount == 0) continue;
            _currentMember = member; _frame = source.Frame;
            ReadOnlySpan<MeshSurfaceBoundaryDomain.BoundarySpan> spans = source.Boundary.Spans;
            for (int span = 0; span < spans.Length; span++)
            {
                if (source.JoinedSpans[span] || !_frame.IntersectsSegment(new FixedSegment(spans[span].Start, spans[span].End))) continue;
                _currentSpan = span;
                var segment = new FixedSegment(spans[span].Start, spans[span].End);
                RationalSegmentInterval clip = source.GetRetainedInterval(ref span);
                ConePlaneRayEvents.VisitEvents(new ConePlaneRayEventSource(segment), _frame, clip, _joinedVisitor);
            }
        }
        for (int region = 0; region < RegionCount; region++)
        {
            ReduceRegion(region);
            Member source = state.Members[state.NodeMembers[state.Roots[region]]];
            source.Collider.Mesh.GetGeometryTriangleVertices(source.Triangle, out Vector3d a, out Vector3d b, out Vector3d c);
            ReduceSamples(region, new WidePlaneMetrics(new FixedTriangle(a, b, c), source.Collider.Mesh.GeometryFrame));
        }
        state.Commit();
    }

    private void JoinFiniteSpans()
    {
        var state = _joined!; _joinVisitor ??= JoinAtPoint;
        for (int member = 0; member < state.Inputs.Count; member++)
        {
            Member source = state.Members[member];
            // Finite face exits require a complete canonical-owner domain.
            // A certified span can still suppress its seam and share response,
            // but ambiguous owner coverage cannot merge the exit regions.
            if (!source.BoundaryTrusted) continue;
            ReadOnlySpan<MeshSurfaceBoundaryDomain.BoundarySpan> spans = source.Boundary.Spans;
            for (int span = 0; span < spans.Length; span++)
            {
                var edge = spans[span];
                if (edge.Kind != MeshSurfaceBoundaryDomain.SpanKind.Joined
                    || !state.MemberBySurface.TryGetValue((state.Meshes[edge.OtherMember], edge.OtherSurface), out int other)) continue;
                Member target = state.Members[other];
                if (!target.BoundaryTrusted) continue;
                if (!edge.Interval.IsClipped && source.Connectivity.RegionCount == 1 && target.Connectivity.RegionCount == 1)
                {
                    if (source.Frame.IntersectsSegment(new FixedSegment(edge.Start, edge.End)))
                    { state.Union(source.Offset, target.Offset); source.JoinedSpans[span] = true; }
                    continue;
                }
                _currentMember = member; _otherMember = other; _currentSpan = span;
                ConePlaneRayEvents.VisitEvents(new ConePlaneRayEventSource(new FixedSegment(edge.Start, edge.End)),
                    source.Frame, edge.Interval, _joinVisitor);
            }
        }
    }

    private void JoinAtPoint(in ConePlaneRayEventSource source, in ConePlaneRayFrame frame, ConePlaneRayEvent descriptor,
        scoped ConePlaneRayPoint point, scoped ReadOnlySpan<ulong> root,
        scoped in ConePlaneRaySelection positive, scoped in ConePlaneRaySelection negative)
    {
        var state = _joined!; Member first = state.Members[_currentMember], second = state.Members[_otherMember];
        foreach (int a in first.Connectivity.TriangleIndices)
        {
            first.Collider.Mesh.GetGeometryTriangleVertices(a, out Vector3d aa, out Vector3d ab, out Vector3d ac);
            if (!ConePlaneRayPointExits.ContainsTrianglePoint(new FixedTriangle(aa, ab, ac), first.Frame, point, root)) continue;
            foreach (int b in second.Connectivity.TriangleIndices)
            {
                second.Collider.Mesh.GetGeometryTriangleVertices(b, out Vector3d ba, out Vector3d bb, out Vector3d bc);
                if (!ConePlaneRayPointExits.ContainsTrianglePoint(new FixedTriangle(ba, bb, bc), second.Frame, point, root, frame)) continue;
                state.Union(first.Offset + first.Connectivity.GetRegionOrdinal(a), second.Offset + second.Connectivity.GetRegionOrdinal(b));
                first.JoinedSpans[_currentSpan] = true;
            }
        }
    }

    private void AdmitJoinedPoint(in ConePlaneRayEventSource source, in ConePlaneRayFrame frame, ConePlaneRayEvent descriptor,
        scoped ConePlaneRayPoint point, scoped ReadOnlySpan<ulong> root,
        scoped in ConePlaneRaySelection positive, scoped in ConePlaneRaySelection negative)
    {
        var state = _joined!; _visitedJoinedRegions.Clear();
        int first = _currentRegion < 0 ? _currentMember : 0, end = _currentRegion < 0 ? first + 1 : state.Inputs.Count;
        for (int member = first; member < end; member++)
        {
            Member target = state.Members[member];
            foreach (int triangle in target.Connectivity.TriangleIndices)
            {
                int region = state.RegionByRoot[state.Find(target.Offset + target.Connectivity.GetRegionOrdinal(triangle))];
                if (_currentRegion >= 0 && region != _currentRegion || _visitedJoinedRegions.Contains(region)) continue;
                target.Collider.Mesh.GetGeometryTriangleVertices(triangle, out Vector3d a, out Vector3d b, out Vector3d c);
                if (!ConePlaneRayPointExits.ContainsTrianglePoint(new FixedTriangle(a, b, c), target.Frame, point, root, frame)) continue;
                _visitedJoinedRegions.Add(region); _currentSupplier = member;
                int aligned = WideDirection3d.Dot(frame.Normal, GetRegionFrame(region).Normal).Sign;
                KeepJoinedWinner(2 * region + (aligned > 0 ? 0 : 1), positive);
                KeepJoinedWinner(2 * region + (aligned > 0 ? 1 : 0), negative);
                bool represented = ConePlaneRayPointMaterialization.TryGetPointInFrame(frame, point, root, _samplingOrigin, out Vector3d p);
                HasBoundaryIntersection |= source.Scope == ConePlaneRayEventScope.Segment;
                var sample = new PoolEvent(source, descriptor, region, p, CheckRay(aligned > 0 ? positive : negative, aligned, represented),
                    CheckRay(aligned > 0 ? negative : positive, -aligned, represented), _currentMember, member);
                // Final coverage drops coincident exact points; retaining compact
                // descriptors here avoids storing a second bank of wide points.
                _poolEvents.Add(sample);
            }
        }
    }

    private void KeepJoinedWinner(int slot, scoped in ConePlaneRaySelection candidate)
    {
        if (!candidate.HasValue) return;
        var previous = BorrowWinner(slot);
        if (previous.HasValue)
        {
            int order = candidate.CompareDepth(previous);
            if (order < 0) return;
            if (order == 0)
            {
                Winner old = _winners[slot];
                order = candidate.Point.CompareTo(previous.Point, candidate.Root, previous.Root, _frame, GetFrame(old.Member));
                if (order > 0 || order == 0 && _currentSupplier >= old.Supplier) return;
            }
        }
        candidate.Values.CopyTo(previous.Values); candidate.Signs.CopyTo(previous.Signs);
        _winners[slot] = new(candidate.MaximumSource, candidate.MaximumEvent, _currentMember, _currentSupplier, _currentSpan);
    }

    private RationalSegmentInterval GetClip(int member, int span) => span < 0 ? default : _joined!.Members[member].GetRetainedInterval(ref span);
    private ref readonly ConePlaneRayFrame GetFrame(int member) => ref (member < 0 ? ref _frame : ref _joined!.Members[member].Frame);
    private ref readonly ConePlaneRayFrame GetRegionFrame(int region) => ref GetFrame(_joined!.NodeMembers[_joined.Roots[region]]);
    private int GetNativeOrientation(int region, int member, int orientation) => member < 0 ? orientation
        : orientation * WideDirection3d.Dot(GetFrame(member).Normal, GetRegionFrame(region).Normal).Sign;

    private int CompareJoinedPoints(Sample first, Sample second)
    {
        Span<ulong> a = stackalloc ulong[PointWords], b = stackalloc ulong[PointWords];
        Span<int> sa = stackalloc int[ConePlaneRayPoint.SignCount], sb = stackalloc int[ConePlaneRayPoint.SignCount];
        Span<ulong> ra = stackalloc ulong[_rootWords], rb = stackalloc ulong[_rootWords];
        var pa = new ConePlaneRayPoint(a, sa, _fieldWords); var pb = new ConePlaneRayPoint(b, sb, _fieldWords);
        ref readonly ConePlaneRayFrame fa = ref GetFrame(first.Member);
        ref readonly ConePlaneRayFrame fb = ref GetFrame(second.Member);
        bool foundA = ConePlaneRayEvents.ReconstructPoint(first.Source, fa, first.Event, GetClip(first.Member, first.Span), pa, ra);
        bool foundB = ConePlaneRayEvents.ReconstructPoint(second.Source, fb, second.Event, GetClip(second.Member, second.Span), pb, rb);
        System.Diagnostics.Debug.Assert(foundA && foundB);
        return pa.CompareTo(pb, ra, rb, fa, fb);
    }

    internal bool IsResponseCarrier(LSMeshCollider collider, int region) =>
        _joined!.Members[_joined.GetResponseMember(_joined.NodeMembers[_joined.Roots[region]])].Collider == collider;

    internal FixedContactAnchors GetJoinedSample(int region, int index, out LSMeshCollider supplier, out int owner)
    {
        Sample sample = _samples[region * ContactManifold.MaxContactsPerGroup + index];
        supplier = _joined!.Members[sample.Supplier].Collider;
        owner = _joined.Members[sample.Supplier].Owner;
        return new(new FixedPointAnchor(_samplingOrigin, FixedQuaternion.Identity, sample.MeshPoint),
            new FixedPointAnchor(_samplingOrigin, GetFrame(sample.Member).ConeRotation, sample.ConeLocalPoint), GetNormal(region), sample.Depth, false);
    }

    internal MeshSurfaceBoundaryDomain? GetJoinedBoundary(LSMeshCollider collider, int triangle) => _joined?.Active == true
        && GetMember(collider, triangle).BoundaryTrusted ? GetMember(collider, triangle).Boundary : null;
    internal RationalSegmentInterval GetRetainedBoundaryInterval(LSMeshCollider collider, int triangle, ref int span) =>
        GetMember(collider, triangle).GetRetainedInterval(ref span);
    private Member GetMember(LSMeshCollider collider, int triangle) => _joined!.Members[
        _joined.MemberBySurface[(collider.Mesh, collider.Mesh.GetManifoldSurfaceOwner(triangle))]];
    internal ConePlaneRayFrame GetBoundaryFrame(LSMeshCollider collider, int triangle) => _joined?.Active == true ? GetMember(collider, triangle).Frame : _frame;
    internal int GetBoundaryOrientation(LSMeshCollider collider, int triangle, int region)
    {
        if (_joined?.Active != true) return GetOrientation(region);
        Member member = GetMember(collider, triangle);
        int joined = _joined.RegionByRoot[_joined.Find(member.Offset + region)];
        return WideDirection3d.Dot(member.Frame.Normal, GetRegionFrame(joined).Normal).Sign * GetOrientation(joined);
    }
    internal bool IsJoinedSpan(LSMeshCollider collider, int triangle, int span) => GetMember(collider, triangle).JoinedSpans[span];
    internal ReadOnlySpan<WideAxis3> GetJoinedCornerFan(LSMeshCollider collider, int triangle, int vertex, out bool suppress) =>
        _joined!.GetCornerFan(collider, triangle, vertex, out suppress);

    internal bool AppendJoinedBoundaries(LSMeshCollider collider, LSConeCollider cone, MeshConeSurfaceBoundary boundary, ContactManifold manifold)
    {
        bool overlaps = false;
        for (int index = 0; index < _joined!.Inputs.Count; index++)
        {
            Member member = _joined.Members[index];
            if (member.Connectivity.RegionCount == 0) continue;
            overlaps |= member.Collider == collider;
            if (_joined.Members[_joined.GetResponseMember(index)].Collider != collider) continue;
            // Face samples and their finite perimeter share one solver pass.
            if (HasBoundaryIntersection) boundary.BuildSurfaceContacts(member.Collider, cone, member.Triangle, member.Connectivity, this, manifold);
        }
        return overlaps;
    }
}
