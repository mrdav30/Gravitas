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
using Member = Gravitas.CollisionHandling.MeshJoinedSurfaceRegions<FixedMathSharp.Geometry.FiniteSlabRayFrame>.Member;

namespace Gravitas.CollisionHandling;

/// <content>Combines native finite extrusion supports across declared surface members.</content>
internal sealed partial class MeshSlabFaceRegions
{
    private MeshJoinedSurfaceRegions<FiniteSlabRayFrame> _joined = null!;
    private readonly SwiftList<Sample> _joinedEvents = new();
    private readonly MeshSurfaceConnectivity _boundaryConnectivity = new();
    private readonly Vector2d[] _boxOffsets = new Vector2d[4];
    private int[] _memberCounts = Array.Empty<int>(), _orientations = Array.Empty<int>(), _dimensions = Array.Empty<int>();
    private int[] _components = Array.Empty<int>(), _responseMembers = Array.Empty<int>();
    private bool[] _completeSupports = Array.Empty<bool>();
    private int _member = -1, _otherMember;
    private FiniteSlabRayCandidateVisitor? _joinVisitor;
    internal ReadOnlyMemory<Vector2d> PrismOffsets { get; private set; }
    internal int SourceId => _member < 0 ? 0 : _joined.Members[_member].Collider.Id + 1;

    internal void ReleaseJoined() { _joined = null!; _member = -1; _joinedEvents.FastClear(); }

    internal void BuildJoined(ReadOnlySpan<(LSMeshCollider Collider, int Triangle)> sources,
        LSCollider mover, Fixed64 radius, Fixed64 halfThickness, Vector2d halfFootprint,
        ReadOnlySpan<PhysicsMesh> meshes, SwiftDictionary<(PhysicsMesh, int), MeshSurfaceBoundaryDomain> boundaries)
    {
        PrismOffsets = default;
        if (halfFootprint != Vector2d.Zero)
        {
            _boxOffsets[0] = -halfFootprint; _boxOffsets[1] = new(halfFootprint.X, -halfFootprint.Y);
            _boxOffsets[2] = halfFootprint; _boxOffsets[3] = new(-halfFootprint.X, halfFootprint.Y);
            PrismOffsets = _boxOffsets;
        }
        _joined ??= new();
        _joined.Prepare(sources, meshes, boundaries); _joinedEvents.FastClear();
        if (_memberCounts.Length < _joined.Inputs.Count)
        {
            Array.Resize(ref _memberCounts, _joined.Inputs.Count);
            Array.Resize(ref _orientations, _joined.Inputs.Count); Array.Resize(ref _dimensions, _joined.Inputs.Count);
            Array.Resize(ref _components, _joined.Inputs.Count); Array.Resize(ref _responseMembers, _joined.Inputs.Count);
            Array.Resize(ref _completeSupports, _joined.Inputs.Count);
        }
        for (int index = 0; index < _joined.Inputs.Count; index++)
        { _components[index] = index; _responseMembers[index] = -1; _memberCounts[index] = 0; _completeSupports[index] = true; }
        // Geometry certifies coverage; finite support admission below decides
        // connectivity. A covered rim cannot become exposed merely because its
        // owner contributes no final face sample.
        for (int index = 0; index < _joined.Inputs.Count; index++)
        {
            Member member = _joined.Members[index];
            for (int span = 0; span < member.Boundary.Spans.Length; span++)
            {
                var edge = member.Boundary.Spans[span];
                member.JoinedSpans[span] = member.BoundaryTrusted && edge.Kind == MeshSurfaceBoundaryDomain.SpanKind.Joined
                    && _joined.MemberBySurface.TryGetValue((_joined.Meshes[edge.OtherMember], edge.OtherSurface), out int other)
                    && _joined.Members[other].BoundaryTrusted;
                if (member.JoinedSpans[span])
                {
                    int a = FindComponent(index), b = FindComponent(_joined.MemberBySurface[(_joined.Meshes[edge.OtherMember], edge.OtherSurface)]);
                    _components[Math.Max(a, b)] = Math.Min(a, b);
                }
            }
        }
        var bounds = new FixedBoundVolume(mover.BoundsMin, mover.BoundsMax);
        for (int index = 0; index < _joined.Inputs.Count; index++)
        {
            Member member = _joined.Members[index]; PhysicsMesh mesh = member.Collider.Mesh;
            mesh.GetLocalTriangleVertices(member.Triangle, out Vector3d a, out Vector3d b, out Vector3d c);
            member.Frame = new(new FixedTriangle(a, b, c), mesh.Origin, mesh.Rotation,
                mover.Center, mover.Rotation, Fixed64.Zero, radius, halfThickness, fullHeight: mover is LSCylinderCollider);
            member.Admitted.FastClear(); member.Collider.GetTrianglesInBounds(bounds, _joined.Candidates);
            foreach (int triangle in _joined.Candidates.AsReadOnlySpan())
            {
                if (mesh.GetManifoldSurfaceOwner(triangle) != member.Owner) continue;
                mesh.GetLocalTriangleVertices(triangle, out a, out b, out c);
                if (member.Frame.Section.IntersectsTriangle(new FixedTriangle(a, b, c), PrismOffsets.Span)) member.Admitted.Add(triangle);
            }
            if (member.Admitted.Count > 0)
            {
                int component = FindComponent(index);
                if (_responseMembers[component] < 0) _responseMembers[component] = index;
            }
        }
        CertifyCompleteSupports();
        for (int index = 0; index < _joined.Inputs.Count; index++)
        {
            Member member = _joined.Members[index];
            // Only an actually intersected coplanar component owns response.
            // The carrier's pair remains an ordinary, genuine overlap; samples
            // keep their own finite supplier, material and provenance.
            if (_responseMembers[FindComponent(index)] < 0) { _joined.AddRegions(index, 0); continue; }
            PhysicsMesh mesh = member.Collider.Mesh;
            if (_completeSupports[FindComponent(index)] && member.Admitted.Count == 0) { _joined.AddRegions(index, 0); continue; }
            member.Connectivity.BuildSlab(mesh, member.Triangle, member.Frame.Section, PrismOffsets, member.Admitted.AsReadOnlySpan());
            _member = index; PrepareFrame(member.Frame, PrismOffsets.Span);
            if (BorrowPeak(1).CompareDepth(BorrowPeak(-1)) == 0)
            {
                // All native charts share this physical slab chart. Resolve a
                // symmetric exit before rounding, independent of mesh winding.
                WideAxis3 normal = member.Frame.Plane.Normal;
                int sign = !normal.X.IsZero ? normal.X.Sign : !normal.Y.IsZero ? normal.Y.Sign : normal.Z.Sign;
                _orientation = -sign;
            }
            _orientations[index] = _orientation; _dimensions[index] = _supportDimension;
            if (_completeSupports[FindComponent(index)])
            {
                _events.FastClear(); _region = 0; _intrinsicOnly = _usesSupportGraph = false;
                // An admitted member without a pole still connects neighboring
                // suppliers through the shared finite span graph.
                RegionCount = 1;
                VisitPlane(PrismOffsets.Span);
            }
            else Build(mesh, member.Triangle, member.Connectivity, PrismOffsets.Span, null);
            _memberCounts[index] = RegionCount; _joined.AddRegions(index, RegionCount);
            foreach (Sample sample in _events.AsReadOnlySpan()) _joinedEvents.Add(sample.WithRegion(member.Offset + sample.Region));
        }
        JoinSupportSpans(); _joined.CompleteRegions();
        RegionCount = _joined.Roots.Count;
        for (int index = 0; index < _joinedEvents.Count; index++)
        {
            Sample sample = _joinedEvents[index];
            _joinedEvents[index] = sample.WithRegion(_joined.RegionByRoot[_joined.Find(sample.Region)]);
        }
        _joined.Commit();
    }

    private void CertifyCompleteSupports()
    {
        for (int index = 0; index < _joined.Inputs.Count; index++)
        {
            Member member = _joined.Members[index]; int component = FindComponent(index);
            if (!_completeSupports[component]) continue;
            if (!member.BoundaryTrusted || !member.Frame.Plane.Normal.X.IsZero || !member.Frame.Plane.Normal.Z.IsZero)
            { _completeSupports[component] = false; continue; }
            // For an axial cap the support footprint equals the original
            // convex section. One filled point and no real perimeter crossing
            // certify complete fill, including exclusion of enclosed holes.
            for (int span = 0; span < member.Boundary.Spans.Length; span++)
            {
                var edge = member.Boundary.Spans[span];
                if (!member.JoinedSpans[span] && member.Frame.Section.IntersectsSegment(new FixedSegment(edge.Start, edge.End), PrismOffsets.Span))
                { _completeSupports[component] = false; break; }
            }
        }
    }

    private bool ContainsSupportPoint(scoped in FiniteSlabRaySelection candidate)
    {
        Member member = _joined.Members[_member];
        foreach (int triangle in member.Admitted.AsReadOnlySpan())
        {
            member.Collider.Mesh.GetLocalTriangleVertices(triangle, out Vector3d a, out Vector3d b, out Vector3d c);
            if (candidate.ContainsTrianglePoint(member.Frame, new(a, b, c), member.Frame)) return true;
        }
        return false;
    }

    private int FindComponent(int member)
    {
        while (_components[member] != member)
        { _components[member] = _components[_components[member]]; member = _components[member]; }
        return member;
    }

    private void GatherJoinedBoundary(PhysicsMesh mesh, int triangle, MeshSurfaceConnectivity connectivity)
    {
        Member member = _joined.Members[_member];
        Span<ulong> values = stackalloc ulong[FiniteSlabRaySelection.StorageWords];
        Span<sbyte> signs = stackalloc sbyte[FiniteSlabRaySelection.SignCount];
        for (int span = 0; span < member.Boundary.Spans.Length; span++)
        {
            var piece = member.Boundary.Spans[span];
            if (member.JoinedSpans[span]) continue;
            RationalSegmentInterval clip = member.GetRetainedInterval(ref span);
            _region = _usesSupportGraph ? MeshSurfaceBoundaryTopology.FindSupportEdgeRegion(mesh, member.Owner,
                piece.First, piece.Second, connectivity, this, null) : 0;
            if (_region < 0) continue;
            var maximum = new FiniteSlabRaySelection(values, signs);
            FiniteSlabRayExtrema.AccumulateSegment(_frame, new FixedSegment(piece.Start, piece.End), clip,
                PrismOffsets.Span, _orientation, ref maximum, _visitor);
        }
    }

    private void SelectJoinedMember(int index)
    {
        _member = index; PrepareFrame(_joined.Members[index].Frame, PrismOffsets.Span);
        _orientation = _orientations[index]; _supportDimension = _dimensions[index];
    }

    private void JoinSupportSpans()
    {
        _joinVisitor ??= JoinAtSupportPoint;
        Span<ulong> values = stackalloc ulong[FiniteSlabRaySelection.StorageWords];
        Span<sbyte> signs = stackalloc sbyte[FiniteSlabRaySelection.SignCount];
        for (int index = 0; index < _joined.Inputs.Count; index++)
        {
            if (_memberCounts[index] == 0) continue;
            SelectJoinedMember(index); Member member = _joined.Members[index];
            for (int span = 0; span < member.Boundary.Spans.Length; span++)
            {
                if (!member.JoinedSpans[span]) continue;
                var edge = member.Boundary.Spans[span];
                _otherMember = _joined.MemberBySurface[(_joined.Meshes[edge.OtherMember], edge.OtherSurface)];
                if (_memberCounts[_otherMember] == 0) continue;
                var maximum = new FiniteSlabRaySelection(values, signs);
                FiniteSlabRayExtrema.AccumulateSegment(_frame, new FixedSegment(edge.Start, edge.End), edge.Interval,
                    PrismOffsets.Span, _orientation, ref maximum, _joinVisitor);
            }
        }
    }

    private void JoinAtSupportPoint(scoped in FiniteSlabRaySelection candidate)
    {
        if (candidate.CompareDepth(BorrowPeak(_orientation)) != 0) return;
        Member first = _joined.Members[_member], second = _joined.Members[_otherMember];
        if (_memberCounts[_member] == 1 && _memberCounts[_otherMember] == 1)
        {
            _joined.Union(first.Offset, second.Offset);
            return;
        }
        // The certified joined span already lies in both finite owners. A
        // unique support region needs no initial-section triangle lookup:
        // oblique support projections can lie outside that initial section.
        Span<int> singleton = stackalloc int[1] { -1 };
        ReadOnlySpan<int> firstDomain = _memberCounts[_member] == 1 ? singleton : first.Connectivity.TriangleIndices;
        ReadOnlySpan<int> secondDomain = _memberCounts[_otherMember] == 1 ? singleton : second.Connectivity.TriangleIndices;
        foreach (int a in firstDomain)
        {
            if (a >= 0)
            {
                first.Collider.Mesh.GetLocalTriangleVertices(a, out Vector3d aa, out Vector3d ab, out Vector3d ac);
                if (!candidate.ContainsTrianglePoint(first.Frame, new(aa, ab, ac), _frame)) continue;
            }
            foreach (int b in secondDomain)
            {
                if (b >= 0)
                {
                    second.Collider.Mesh.GetLocalTriangleVertices(b, out Vector3d ba, out Vector3d bb, out Vector3d bc);
                    if (!candidate.ContainsTrianglePoint(second.Frame, new(ba, bb, bc), _frame)) continue;
                }
                _joined.Union(first.Offset + (a < 0 ? 0 : first.Connectivity.GetRegionOrdinal(a)),
                    second.Offset + (b < 0 ? 0 : second.Connectivity.GetRegionOrdinal(b)));
            }
        }
    }

    internal bool AppendJoinedContacts(LSMeshCollider collider, LSCollider mover, MeshSlabSurfaceBoundary boundary, ContactManifold manifold)
    {
        Span<int> chosen = stackalloc int[ContactManifold.MaxContactsPerGroup];
        for (int region = 0; region < _joined.Roots.Count; region++)
        {
            int rootMember = _joined.NodeMembers[_joined.Roots[region]];
            Member root = _joined.Members[rootMember];
            if (_joined.Members[_responseMembers[FindComponent(rootMember)]].Collider != collider) continue;
            int count = ReduceSamples(region, _joinedEvents.AsReadOnlySpan(),
                new WidePlaneMetrics(root.Frame.SourcePlane, root.Collider.Mesh.Rotation), _dimensions[rootMember], chosen);
            for (int index = 0; index < count; index++)
            {
                Sample sample = _pool[chosen[index]]; Member supplier = _joined.Members[sample.Member];
                manifold.AddContact(new ContactAnchor(new FixedPointAnchor(supplier.Frame.Origin, supplier.Frame.Rotation, sample.Authored)),
                    new ContactAnchor(sample.Exit), sample.Depth, root.Frame.GetNormal(_orientations[rootMember]), supplier.Collider.Material,
                    mover.Material, sample.Clamped, group: new ContactGroupKey(0, 0, supplier.Owner + 1, 0, region, sourceIdA: supplier.Collider.Id + 1));
            }
        }
        bool overlaps = false;
        for (int member = 0; member < _joined.Inputs.Count; member++)
        {
            Member source = _joined.Members[member];
            if (source.Admitted.Count == 0) continue;
            overlaps |= source.Collider == collider;
            if (_joined.Members[_responseMembers[FindComponent(member)]].Collider != collider) continue;
            SelectJoinedMember(member);
            // Keep the component's face and perimeter rows in one solver pass;
            // otherwise matching normals receive positional correction twice.
            _boundaryConnectivity.BuildSlab(source.Collider.Mesh, source.Triangle, source.Frame.Section, PrismOffsets, source.Admitted.AsReadOnlySpan());
            boundary.BuildSurfaceContacts(source.Collider, null, source.Triangle, _boundaryConnectivity, this, manifold, 0, 0, mover.Material);
        }
        return overlaps;
    }

    internal MeshSurfaceBoundaryDomain? JoinedBoundary => _member < 0 || !_joined.Members[_member].BoundaryTrusted
        ? null : _joined.Members[_member].Boundary;
    internal bool IsJoinedSpan(int span) => _joined.Members[_member].JoinedSpans[span];
    internal RationalSegmentInterval GetRetainedInterval(ref int span) => _joined.Members[_member].GetRetainedInterval(ref span);
    internal ReadOnlySpan<WideAxis3> GetJoinedCornerFan(LSMeshCollider collider, int triangle, int vertex, out bool suppress) =>
        _joined.GetCornerFan(collider, triangle, vertex, out suppress);
}
