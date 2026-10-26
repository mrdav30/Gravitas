//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================
using FixedMathSharp;
using FixedMathSharp.Geometry;
using Gravitas.Colliders;
using Gravitas.Materials;
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
    private Vector2d[] _boxOffsets = new Vector2d[4];
    private int[] _memberCounts = Array.Empty<int>(), _orientations = Array.Empty<int>(), _dimensions = Array.Empty<int>();
    private bool[] _completeSupports = Array.Empty<bool>();
    private int _member = -1, _otherMember;
    private FixedBoundVolume _slabBounds;
    private FiniteSlabRayCandidateVisitor? _joinVisitor;
    private FiniteSlabRayCandidateVisitor? _convexSupportVisitor;
    private bool _convexSupportCovered;
    internal ReadOnlyMemory<Vector2d> PrismOffsets { get; private set; }
    internal int SourceId => _member < 0 ? 0 : _joined.Members[_member].Collider.Id + 1;

    internal void ReleaseJoined() { _joined = null!; _member = -1; _joinedEvents.FastClear(); }

    internal void BuildJoined(ReadOnlySpan<(LSMeshCollider Collider, int Triangle)> sources,
        LSCollider mover, Fixed64 radius, Fixed64 halfThickness, Vector2d halfFootprint,
        ReadOnlySpan<PhysicsMesh> meshes, SwiftDictionary<(PhysicsMesh, int), MeshSurfaceBoundaryDomain> boundaries)
    {
        Span<Vector2d> offsets = stackalloc Vector2d[4];
        if (halfFootprint != Vector2d.Zero)
        {
            offsets[0] = -halfFootprint; offsets[1] = new(halfFootprint.X, -halfFootprint.Y);
            offsets[2] = halfFootprint; offsets[3] = new(-halfFootprint.X, halfFootprint.Y);
        }
        BuildJoined(sources, mover.Center, mover.Rotation, Fixed64.Zero, radius, halfThickness,
            halfFootprint == Vector2d.Zero ? ReadOnlySpan<Vector2d>.Empty : offsets,
            new(mover.BoundsMin, mover.BoundsMax), mover is LSCylinderCollider, meshes, boundaries);
    }

    internal void BuildJoined(ReadOnlySpan<(LSMeshCollider Collider, int Triangle)> sources,
        Vector3d center, FixedQuaternion rotation, Fixed64 length, Fixed64 radius, Fixed64 halfThickness,
        ReadOnlySpan<Vector2d> offsets, FixedBoundVolume bounds, bool fullHeight,
        ReadOnlySpan<PhysicsMesh> meshes, SwiftDictionary<(PhysicsMesh, int), MeshSurfaceBoundaryDomain> boundaries)
    {
        _slabBounds = bounds;
        if (_boxOffsets.Length < offsets.Length) Array.Resize(ref _boxOffsets, offsets.Length);
        offsets.CopyTo(_boxOffsets);
        PrismOffsets = _boxOffsets.AsMemory(0, offsets.Length);
        _joined ??= new();
        _joined.Prepare(sources, meshes, boundaries); _joinedEvents.FastClear();
        if (_memberCounts.Length < _joined.Inputs.Count)
        {
            Array.Resize(ref _memberCounts, _joined.Inputs.Count);
            Array.Resize(ref _orientations, _joined.Inputs.Count); Array.Resize(ref _dimensions, _joined.Inputs.Count);
            Array.Resize(ref _completeSupports, _joined.Inputs.Count);
        }
        _joined.PrepareResponseComponents();
        for (int index = 0; index < _joined.Inputs.Count; index++)
        { _memberCounts[index] = 0; _completeSupports[index] = true; }
        for (int index = 0; index < _joined.Inputs.Count; index++)
        {
            Member member = _joined.Members[index]; PhysicsMesh mesh = member.Collider.Mesh;
            mesh.GetGeometryTriangleVertices(member.Triangle, out Vector3d a, out Vector3d b, out Vector3d c);
            member.Frame = new(new FixedTriangle(a, b, c), mesh.GeometryFrame,
                center, rotation, length, radius, halfThickness, fullHeight: fullHeight);
            member.Admitted.FastClear(); member.Collider.GetTrianglesInBounds(bounds, _joined.Candidates);
            foreach (int triangle in _joined.Candidates.AsReadOnlySpan())
            {
                if (mesh.GetManifoldSurfaceOwner(triangle) != member.Owner) continue;
                mesh.GetGeometryTriangleVertices(triangle, out a, out b, out c);
                if (member.Frame.Section.IntersectsTriangle(new FixedTriangle(a, b, c), PrismOffsets.Span)) member.Admitted.Add(triangle);
            }
            if (member.Admitted.Count > 0) _joined.AddAdmittedMember(index);
        }
        bool wideBanks = false;
        foreach (Member member in _joined.Members.AsReadOnlySpan()[.._joined.Inputs.Count])
        {
            wideBanks |= member.Frame.UsesWideBanks;
            foreach (var span in member.Boundary.Spans) wideBanks |= span.Interval.HasWideBounds;
        }
        if (wideBanks)
            foreach (Member member in _joined.Members.AsReadOnlySpan()[.._joined.Inputs.Count])
                member.Frame = member.Frame.WithWideParameters();
        CertifyCompleteSupports();
        for (int index = 0; index < _joined.Inputs.Count; index++)
        {
            Member member = _joined.Members[index];
            // Only an actually intersected coplanar component owns response.
            // The carrier's pair remains an ordinary, genuine overlap; samples
            // keep their own finite supplier, material and provenance.
            if (_joined.GetResponseMember(index) < 0) { _joined.AddRegions(index, 0); continue; }
            PhysicsMesh mesh = member.Collider.Mesh;
            if (_completeSupports[_joined.FindResponseComponent(index)] && member.Admitted.Count == 0) { _joined.AddRegions(index, 0); continue; }
            if (!CanReachSupport(member.Frame, mesh, member.Collider.Bounds))
            { _joined.AddRegions(index, 0); continue; }
            member.Connectivity.BuildSlab(mesh, member.Triangle, member.Frame.Section, PrismOffsets, member.Admitted.AsReadOnlySpan());
            _member = index; PrepareFrame(member.Frame, PrismOffsets.Span);
            _orientations[index] = _orientation; _dimensions[index] = _supportDimension;
            if (_completeSupports[_joined.FindResponseComponent(index)])
            {
                _events.FastClear(); _region = 0; _intrinsicOnly = _usesSupportGraph = false;
                // An admitted member without a pole still connects neighboring
                // suppliers through the shared finite span graph.
                RegionCount = 1;
                VisitPlane(PrismOffsets.Span);
            }
            else if (!TryUseConvexSupport(mesh, member.Triangle)) Build(mesh, member.Triangle, member.Connectivity, PrismOffsets.Span, null,
                member.Admitted.Count > 0 && ContainsSection(mesh, member.Triangle, member.Frame, PrismOffsets.Span));
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

    private bool TryUseConvexSupport(PhysicsMesh mesh, int triangle)
    {
        // A nonaxial extrusion support is a point, segment or rectangle. Its
        // exact extremes inside one convex fill certify the entire support.
        // Axial disk caps need the existing perimeter proof, not sampled poles.
        if (mesh.GetConvexCoplanarPatchCornerVertexIndices(triangle).IsEmpty
            || _frame.Plane.Normal.X.IsZero && _frame.Plane.Normal.Z.IsZero) return false;
        _events.FastClear(); _region = 0; _intrinsicOnly = _usesSupportGraph = false;
        _convexSupportCovered = true;
        _convexSupportVisitor ??= AdmitConvexSupport;
        VisitPlane(PrismOffsets.Span, _convexSupportVisitor);
        if (!_convexSupportCovered || _events.Count == 0) { _events.FastClear(); return false; }
        RegionCount = 1;
        return true;
    }

    private void AdmitConvexSupport(scoped in FiniteSlabRaySelection candidate)
    {
        if (candidate.CompareDepth(BorrowPeak()) != 0) return;
        Member member = _joined.Members[_member];
        PhysicsMesh mesh = member.Collider.Mesh;
        ReadOnlySpan<int> corners = mesh.GetConvexCoplanarPatchCornerVertexIndices(member.Triangle);
        ReadOnlySpan<Vector3d> vertices = mesh.GeometryVertices;
        for (int corner = 1; corner < corners.Length - 1; corner++)
            if (candidate.ContainsTrianglePoint(member.Frame,
                new(vertices[corners[0]], vertices[corners[corner]], vertices[corners[corner + 1]]), member.Frame))
            { AdmitPoint(candidate); return; }
        _convexSupportCovered = false;
    }

    private void CertifyCompleteSupports()
    {
        for (int index = 0; index < _joined.Inputs.Count; index++)
        {
            Member member = _joined.Members[index]; int component = _joined.FindResponseComponent(index);
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
            member.Collider.Mesh.GetGeometryTriangleVertices(triangle, out Vector3d a, out Vector3d b, out Vector3d c);
            if (candidate.ContainsTrianglePoint(member.Frame, new(a, b, c), member.Frame)) return true;
        }
        return false;
    }

    private void GatherJoinedBoundary(PhysicsMesh mesh, int triangle, MeshSurfaceConnectivity connectivity)
    {
        Member member = _joined.Members[_member];
        Span<ulong> values = stackalloc ulong[SelectionWords];
        Span<sbyte> signs = stackalloc sbyte[FiniteSlabRaySelection.SignCount];
        for (int span = 0; span < member.Boundary.Spans.Length; span++)
        {
            var piece = member.Boundary.Spans[span];
            if (member.JoinedSpans[span]) continue;
            FixedBoundBox bounds = mesh.GeometryFrame.GetWorldBounds(Vector3d.Min(piece.Start, piece.End),
                Vector3d.Max(piece.Start, piece.End));
            if (!CanReachSupport(member.Frame, mesh, bounds)) continue;
            RationalSegmentInterval clip = member.GetRetainedInterval(ref span);
            _region = _usesSupportGraph ? MeshSurfaceBoundaryTopology.FindSupportEdgeRegion(mesh, member.Owner,
                piece.First, piece.Second, connectivity, this, null) : 0;
            if (_region < 0) continue;
            var maximum = new FiniteSlabRaySelection(values, signs, _fieldWords);
            FiniteSlabRayExtrema.AccumulateSegment(_frame, new FixedSegment(piece.Start, piece.End), clip,
                PrismOffsets.Span, _orientation, ref maximum, _visitor);
        }
    }

    private bool CanReachSupport(in FiniteSlabRayFrame frame, PhysicsMesh mesh, FixedBoundBox bounds)
    {
        // Translation along the exact plane normal leaves its zero components
        // unchanged. Disjoint outward bounds on those axes cannot supply support.
        WideDirection3d normal = new WideDirection3d(frame.Plane.AuthoredNormal).Transform(mesh.GeometryFrame.GeometryBasis);
        return (!normal.X.IsZero || bounds.Max.X >= _slabBounds.Min.X && bounds.Min.X <= _slabBounds.Max.X)
            && (!normal.Y.IsZero || bounds.Max.Y >= _slabBounds.Min.Y && bounds.Min.Y <= _slabBounds.Max.Y)
            && (!normal.Z.IsZero || bounds.Max.Z >= _slabBounds.Min.Z && bounds.Min.Z <= _slabBounds.Max.Z);
    }

    private void SelectJoinedMember(int index)
    {
        _member = index; _frame = _joined.Members[index].Frame;
        _fieldWords = _frame.FieldWords;
        _orientation = _orientations[index]; _supportDimension = _dimensions[index];
    }

    private void JoinSupportSpans()
    {
        _joinVisitor ??= JoinAtSupportPoint;
        Span<ulong> values = stackalloc ulong[SelectionWords];
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
                var maximum = new FiniteSlabRaySelection(values, signs, _fieldWords);
                FiniteSlabRayExtrema.AccumulateSegment(_frame, new FixedSegment(edge.Start, edge.End), edge.Interval,
                    PrismOffsets.Span, _orientation, ref maximum, _joinVisitor);
            }
        }
    }

    private void JoinAtSupportPoint(scoped in FiniteSlabRaySelection candidate)
    {
        if (candidate.CompareDepth(BorrowPeak()) != 0) return;
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
                first.Collider.Mesh.GetGeometryTriangleVertices(a, out Vector3d aa, out Vector3d ab, out Vector3d ac);
                if (!candidate.ContainsTrianglePoint(first.Frame, new(aa, ab, ac), _frame)) continue;
            }
            foreach (int b in secondDomain)
            {
                if (b >= 0)
                {
                    second.Collider.Mesh.GetGeometryTriangleVertices(b, out Vector3d ba, out Vector3d bb, out Vector3d bc);
                    if (!candidate.ContainsTrianglePoint(second.Frame, new(ba, bb, bc), _frame)) continue;
                }
                _joined.Union(first.Offset + (a < 0 ? 0 : first.Connectivity.GetRegionOrdinal(a)),
                    second.Offset + (b < 0 ? 0 : second.Connectivity.GetRegionOrdinal(b)));
            }
        }
    }

    internal bool AppendJoinedContacts(LSMeshCollider collider, PhysicsMaterial material, MeshSlabSurfaceBoundary boundary,
        ContactManifold manifold, LSCollider2D? embedded = null, int namespace3D = 0, int namespace2D = 0)
    {
        Span<int> chosen = stackalloc int[ContactManifold.MaxContactsPerGroup];
        for (int region = 0; region < _joined.Roots.Count; region++)
        {
            int rootMember = _joined.NodeMembers[_joined.Roots[region]];
            Member root = _joined.Members[rootMember];
            if (_joined.Members[_joined.GetResponseMember(rootMember)].Collider != collider) continue;
            int count = ReduceSamples(region, _joinedEvents.AsReadOnlySpan(),
                new WidePlaneMetrics(root.Frame.SourcePlane, root.Collider.Mesh.GeometryFrame), _dimensions[rootMember], chosen);
            for (int index = 0; index < count; index++)
            {
                Sample sample = _pool[chosen[index]]; Member supplier = _joined.Members[sample.Member];
                manifold.AddContact(new ContactAnchor(new FixedPointAnchor(supplier.Frame.Center, FixedQuaternion.Identity, sample.Point)),
                    new ContactAnchor(sample.Exit), sample.Depth, root.Frame.GetNormal(_orientations[rootMember]), supplier.Collider.Material,
                    material, sample.Clamped, featureNamespaceA: namespace3D, featureNamespaceB: namespace2D,
                    group: new ContactGroupKey(namespace3D, namespace2D, supplier.Owner + 1, 0, region, sourceIdA: supplier.Collider.Id + 1));
            }
        }
        bool overlaps = false;
        for (int member = 0; member < _joined.Inputs.Count; member++)
        {
            Member source = _joined.Members[member];
            if (source.Admitted.Count == 0) continue;
            overlaps |= source.Collider == collider;
            if (_joined.Members[_joined.GetResponseMember(member)].Collider != collider) continue;
            SelectJoinedMember(member);
            // Keep the component's face and perimeter rows in one solver pass;
            // otherwise matching normals receive positional correction twice.
            _boundaryConnectivity.BuildSlab(source.Collider.Mesh, source.Triangle, source.Frame.Section, PrismOffsets, source.Admitted.AsReadOnlySpan());
            boundary.BuildSurfaceContacts(source.Collider, embedded, source.Triangle, _boundaryConnectivity, this,
                manifold, namespace3D, namespace2D, material);
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
