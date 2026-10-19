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
using System.Collections.Generic;

namespace Gravitas.CollisionHandling;

/// <summary>Admits native mesh supports against a declared surface's retained finite features.</summary>
internal sealed class MeshHullSurfaceContacts : IConvexHullContactDomain, IScaledConvexHullContactDomain
{
    private MeshJoinedSurfaceRegions<WideDirection3d> _joined = new();
    private readonly SwiftList<Vector3d> _points = new();
    private readonly SwiftList<int> _vertices = new(), _triangles = new(), _edges = new();
    private readonly SwiftList<Vector3d> _facePoints = new();
    private readonly SwiftList<int> _faceTriangles = new(), _faceEdges = new();
    private readonly SwiftList<RationalSegmentInterval> _intervals = new();
    private readonly SwiftList<int> _supportCandidates = new();
    private readonly SwiftList<int> _supportTriangles = new();
    private readonly RationalSegmentInterval[] _supportClip = new RationalSegmentInterval[1];
    private readonly SwiftList<Sample> _samples = new(), _pool = new();
    private readonly SwiftList<ResponseGroup> _groups = new();
    private readonly SwiftList<(int Member, int Triangle, NativePenetration Penetration)> _admitted = new();
    private readonly SwiftHashSet<int> _overlappingMembers = new();
    private readonly SwiftList<Vector3d> _movingPoints = new();
    private readonly SwiftList<int> _movingTriangles = new(), _movingEdges = new();
    private readonly SwiftHashSet<int> _movingOwners = new();
    private readonly PointSpanPenetrationAdmission _admit;
    private readonly ScaledHullPenetrationAdmission _admitScaled;
    private static readonly RationalSegmentInterval[] FullInterval = { default };
    private static readonly int[] TriangleIndices = { 0, 1, 2 }, TriangleEdges = { 0, 1, 1, 2, 2, 0 };
    private static readonly IComparer<Sample> PointComparer = Comparer<Sample>.Create(CompareSamples);
    private PhysicsMesh _movingMesh = null!;
    private Vector3d _movingCenter;
    private FixedBoundVolume _supportBounds;
    private NativePenetration _supportPenetration;
    private bool _supportQuery, _movingPatch;
    private int _member, _group, _movingFeature = -1, _sampleRegion = -1;
    private ReadOnlySpan<Vector3d> MovingPoints => _movingFeature < 0 ? _movingMesh.GeometryVertices : _movingPoints.AsReadOnlySpan();
    private ReadOnlySpan<int> MovingTriangles => _movingFeature < 0 ? _movingMesh.Triangles
        : _movingPatch ? _movingTriangles.AsReadOnlySpan() : TriangleIndices;
    private ReadOnlySpan<int> MovingEdges => _movingFeature < 0 ? _movingMesh.ConvexSatEdgeVertexPairs
        : _movingPatch ? _movingEdges.AsReadOnlySpan() : TriangleEdges;

    internal MeshHullSurfaceContacts() { _admit = AdmitAxis; _admitScaled = AdmitScaledAxis; }

    internal void Release()
    { _joined = null!; _movingMesh = null!; _samples.FastClear(); _groups.FastClear(); }

    internal void Build(ReadOnlySpan<(LSMeshCollider Collider, int Triangle)> sources, LSMeshCollider mover,
        ReadOnlySpan<PhysicsMesh> meshes, SwiftDictionary<(PhysicsMesh, int), MeshSurfaceBoundaryDomain> boundaries)
    {
        _movingMesh = mover.Mesh; _movingCenter = mover.Center; _samples.FastClear(); _groups.FastClear(); _overlappingMembers.Clear();
        FixedBoundBox supportBounds = FixedBoundBox.FromCenterAndScopeClippedToDomain(mover.Center,
            Vector3d.One * (mover.CanonicalCenteredProxyRadius * Fixed64.Two));
        _supportBounds = new(supportBounds.Min, supportBounds.Max);
        _joined ??= new();
        _joined.Prepare(sources, meshes, boundaries);
        for (int member = 0; member < _joined.Inputs.Count; member++)
        {
            var input = _joined.Members[member]; var mesh = input.Collider.Mesh;
            mesh.GetGeometryTriangleVertices(input.Triangle, out Vector3d a, out Vector3d b, out Vector3d c);
            input.Frame = WideTriangleRelations.GetScaledTriangleLocalNormal(new FixedTriangle(a, b, c),
                mesh.GeometryFrame).Transform(mesh.GeometryFrame.GeometryBasis);
        }
        var bounds = new FixedBoundVolume(mover.BoundsMin, mover.BoundsMax);
        _movingPatch = false;
        if (mover.Mode == MeshColliderMode.Convex && _movingMesh.IsClosedSurface)
        { _movingFeature = -1; BuildPart(bounds); }
        else
        {
            _movingOwners.Clear();
            for (int triangle = 0; triangle < _movingMesh.TriangleCount; triangle++)
            {
                int owner = _movingMesh.GetManifoldSurfaceOwner(triangle);
                ReadOnlySpan<int> corners = _movingMesh.GetConvexCoplanarPatchCornerVertexIndices(triangle);
                _movingPatch = !corners.IsEmpty;
                if (_movingPatch)
                {
                    if (!_movingOwners.Add(owner)) continue;
                    _movingFeature = owner;
                    FillPatch(_movingMesh, corners, _movingPoints, _movingTriangles, _movingEdges);
                }
                else
                {
                    _movingFeature = triangle;
                    _movingMesh.GetGeometryTriangleVertices(triangle, out var a, out var b, out var c);
                    _movingPoints.FastClear(); _movingPoints.Add(a); _movingPoints.Add(b); _movingPoints.Add(c);
                }
                BuildPart(bounds);
            }
        }
        _joined.Commit();
    }

    private void BuildPart(FixedBoundVolume bounds)
    {
        _admitted.FastClear(); _joined.PrepareResponseComponents();
        for (_member = 0; _member < _joined.Inputs.Count; _member++)
        {
            var member = _joined.Members[_member];
            member.Admitted.FastClear();
            ReadOnlySpan<int> corners = _movingFeature < 0 || _movingPatch
                ? member.Collider.Mesh.GetConvexCoplanarPatchCornerVertexIndices(member.Triangle) : default;
            if (!corners.IsEmpty)
            {
                SetPatch(corners);
                if (FindPenetration(out var penetration, filtered: false))
                { member.Admitted.Add(member.Triangle); _admitted.Add((_member, -1, penetration)); }
            }
            else
            {
                member.Collider.GetTrianglesInBounds(bounds, _joined.Candidates);
                foreach (int triangle in _joined.Candidates.AsReadOnlySpan())
                {
                    if (member.Collider.Mesh.GetManifoldSurfaceOwner(triangle) != member.Owner) continue;
                    SetTriangle(triangle);
                    if (FindPenetration(out var penetration, filtered: false))
                    { member.Admitted.Add(triangle); _admitted.Add((_member, triangle, penetration)); }
                }
            }
            if (member.Admitted.Count > 0)
            { _joined.AddAdmittedMember(_member); _overlappingMembers.Add(member.Collider.Id); }
        }
        foreach (var admitted in _admitted.AsReadOnlySpan())
        {
            _member = admitted.Member;
            var member = _joined.Members[_member]; PhysicsMesh mesh = member.Collider.Mesh;
            if (admitted.Triangle < 0) SetPatch(mesh.GetConvexCoplanarPatchCornerVertexIndices(member.Triangle));
            else SetTriangle(admitted.Triangle);
            BuildContacts(admitted.Penetration);
        }
    }

    private void SetPatch(ReadOnlySpan<int> corners)
    {
        _vertices.FastClear(); _vertices.AddRange(corners);
        FillPatch(_joined.Members[_member].Collider.Mesh, corners, _points, _triangles, _edges);
    }

    private static void FillPatch(PhysicsMesh mesh, ReadOnlySpan<int> corners, SwiftList<Vector3d> points,
        SwiftList<int> triangles, SwiftList<int> edges)
    {
        points.FastClear(); triangles.FastClear(); edges.FastClear();
        for (int index = 0; index < corners.Length; index++)
        {
            points.Add(mesh.GeometryVertices[corners[index]]);
            edges.Add(index); edges.Add((index + 1) % corners.Length);
            if (index < 2) continue;
            triangles.Add(0); triangles.Add(index - 1); triangles.Add(index);
        }
    }

    private void SetTriangle(int triangle)
    {
        PhysicsMesh mesh = _joined.Members[_member].Collider.Mesh;
        _vertices.FastClear(); _points.FastClear(); _triangles.FastClear(); _edges.FastClear();
        foreach (int vertex in mesh.GetWeldedTriangleVertexIndices(triangle))
        { _vertices.Add(vertex); _points.Add(mesh.GeometryVertices[vertex]); }
        _triangles.AddRange(TriangleIndices); _edges.AddRange(TriangleEdges);
    }

    private bool FindPenetration(out NativePenetration best, bool filtered)
    {
        PhysicsMesh target = _joined.Members[_member].Collider.Mesh;
        if (!_movingMesh.GeometryFrame.IsRigid || !target.GeometryFrame.IsRigid)
        {
            bool overlaps = WideOrientedBox.TryGetConvexHullPenetration(_movingMesh.GeometryFrame,
                MovingPoints, MovingTriangles, MovingEdges, target.GeometryFrame,
                _points.AsReadOnlySpan(), _triangles.AsReadOnlySpan(), _edges.AsReadOnlySpan(),
                out WideScaledHullPenetration scaled, skipParallelDirections: true,
                admission: filtered ? _admitScaled : null, planarPair: _movingFeature >= 0);
            best = new NativePenetration(scaled);
            return overlaps;
        }
        bool rigidOverlap = _movingFeature >= 0 && !_movingPatch
        ? WideTriangleRelations.TryGetPenetration(new(_movingPoints[0], _movingPoints[1], _movingPoints[2]),
            _movingMesh.Origin, _movingMesh.Rotation, new(_points[0], _points[1], _points[2]),
            target.Origin, target.Rotation,
            out var rigid, filtered ? _admit : null)
        : WideOrientedBox.TryGetConvexHullPenetration(_movingMesh.Origin, _movingMesh.Rotation,
            MovingPoints, MovingTriangles, MovingEdges,
            target.Origin, target.Rotation,
            _points.AsReadOnlySpan(), _triangles.AsReadOnlySpan(), _edges.AsReadOnlySpan(),
            out rigid, skipParallelDirections: true, admission: filtered ? _admit : null, planarPair: _movingPatch);
        best = new NativePenetration(rigid);
        return rigidOverlap;
    }

    private bool AdmitAxis(in WidePointSpanPenetration candidate) => AdmitAxis(new NativePenetration(candidate));
    private bool AdmitScaledAxis(in WideScaledHullPenetration candidate) => AdmitAxis(new NativePenetration(candidate));
    private bool AdmitAxis(in NativePenetration candidate) => IsFaceNormal(candidate.Axis)
        ? FindGroup(candidate) < _groups.Count || VisitFace(candidate, admissionOnly: true) > 0
        : Visit(candidate, admissionOnly: true) > 0;

    private int VisitFace(in NativePenetration penetration, bool admissionOnly)
    {
        int source = _member, component = _joined.FindResponseComponent(source), count = 0;
        int firstSample = _samples.Count;
        if (!admissionOnly)
        {
            _supportPenetration = penetration;
            _joined.Parents.FastClear(); _joined.Roots.FastClear(); _joined.RegionByRoot.FastClear(); _joined.NodeMembers.FastClear();
        }
        Span<Vector3d> points = stackalloc Vector3d[3];
        // A plane's support may project onto a member with no initial overlap.
        // Its genuine overlap carrier owns response; finite suppliers own anchors.
        for (int member = 0; member < _joined.Inputs.Count; member++)
        {
            if (_joined.FindResponseComponent(member) != component
                || !MaySupplyFace(_joined.Members[member].Collider, penetration.Axis))
            { if (!admissionOnly) _joined.AddRegions(member, 0); continue; }
            _member = member;
            var supplier = _joined.Members[member]; var mesh = supplier.Collider.Mesh;
            ReadOnlySpan<int> corners = mesh.GetConvexCoplanarPatchCornerVertexIndices(supplier.Triangle);
            if (!corners.IsEmpty)
            {
                FillPatch(mesh, corners, _facePoints, _faceTriangles, _faceEdges);
                _sampleRegion = admissionOnly ? -1 : _joined.Parents.Count;
                int supplied = VisitFacePatch(mesh, _facePoints.AsReadOnlySpan(), _faceTriangles.AsReadOnlySpan(),
                    _faceEdges.AsReadOnlySpan(), penetration, admissionOnly);
                count += supplied;
                if (!admissionOnly) _joined.AddRegions(member, supplied > 0 ? 1 : 0);
                if (admissionOnly && count > 0) { _member = source; return count; }
                continue;
            }
            supplier.Collider.GetTrianglesInBounds(_supportBounds, _supportCandidates);
            _supportTriangles.FastClear();
            foreach (int triangle in _supportCandidates.AsReadOnlySpan())
            {
                if (mesh.GetManifoldSurfaceOwner(triangle) != supplier.Owner) continue;
                mesh.GetGeometryTriangleVertices(triangle, out points[0], out points[1], out points[2]);
                int supplied = VisitFacePatch(mesh, points, TriangleIndices, TriangleEdges, penetration, admissionOnly: true);
                count += supplied;
                if (supplied > 0) _supportTriangles.Add(triangle);
                if (admissionOnly && count > 0) { _member = source; return count; }
            }
            if (admissionOnly) continue;
            supplier.Connectivity.BuildHullSupport(mesh, supplier.Triangle, this, _supportTriangles.AsReadOnlySpan());
            _joined.AddRegions(member);
            foreach (int triangle in _supportTriangles.AsReadOnlySpan())
            {
                _sampleRegion = supplier.Offset + supplier.Connectivity.GetRegionOrdinal(triangle);
                mesh.GetGeometryTriangleVertices(triangle, out points[0], out points[1], out points[2]);
                VisitFacePatch(mesh, points, TriangleIndices, TriangleEdges, penetration, admissionOnly: false);
            }
        }
        if (!admissionOnly) CompleteFaceRegions(firstSample);
        _sampleRegion = -1;
        _member = source;
        return count;
    }

    private bool MaySupplyFace(LSMeshCollider supplier, in WideDirection3d normal)
    {
        // Plane projection preserves coordinates whose exact normal component
        // is zero. Cull only strict separation there, including remote suppliers.
        for (int coordinate = 0; coordinate < 3; coordinate++)
        {
            if (!(coordinate == 0 ? normal.X : coordinate == 1 ? normal.Y : normal.Z).IsZero) continue;
            if (supplier.BoundsMax[coordinate] < _movingMesh.Bounds.Min[coordinate]
                || supplier.BoundsMin[coordinate] > _movingMesh.Bounds.Max[coordinate]) return false;
        }
        return true;
    }

    private void CompleteFaceRegions(int firstSample)
    {
        for (_member = 0; _member < _joined.Inputs.Count; _member++)
        {
            var member = _joined.Members[_member];
            if (RegionCount(_member) == 0) continue;
            for (int span = 0; span < member.Boundary.Spans.Length; span++)
            {
                if (!member.JoinedSpans[span]) continue;
                var edge = member.Boundary.Spans[span];
                int other = _joined.MemberBySurface[(_joined.Meshes[edge.OtherMember], edge.OtherSurface)];
                if (other <= _member || RegionCount(other) == 0
                    || !IntersectsSupport(new FixedSegment(edge.Start, edge.End), edge.Interval)) continue;
                int first = SupportEdgeRegion(_member, edge.First, edge.Second);
                var neighbor = _joined.Members[other];
                int saved = _member; _member = other;
                foreach (var reciprocal in neighbor.Boundary.Spans)
                {
                    if (reciprocal.Kind != MeshSurfaceBoundaryDomain.SpanKind.Joined
                        || reciprocal.OtherMember != member.Boundary.SourceMember || reciprocal.OtherSurface != member.Owner
                        || !IsReciprocalSpan(edge, member.Collider.Mesh, reciprocal, neighbor.Collider.Mesh)) continue;
                    int second = SupportEdgeRegion(other, reciprocal.First, reciprocal.Second);
                    System.Diagnostics.Debug.Assert(first >= 0 && second >= 0);
                    _joined.Union(member.Offset + first, neighbor.Offset + second);
                }
                _member = saved;
            }
        }
        _joined.CompleteRegions();
        ResponseGroup response = _groups[_group];
        for (int region = 1; region < _joined.Roots.Count; region++) _groups.Add(response);
        for (int index = firstSample; index < _samples.Count; index++)
        {
            Sample sample = _samples[index];
            _samples[index] = new(_group + _joined.RegionByRoot[_joined.Find(sample.Group)], sample.Member, sample.Point, sample.Contact);
        }
    }

    private static bool IsReciprocalSpan(in MeshSurfaceBoundaryDomain.BoundarySpan source, PhysicsMesh sourceMesh,
        in MeshSurfaceBoundaryDomain.BoundarySpan target, PhysicsMesh targetMesh)
    {
        // Trusted reciprocal spans are positive maximal runs after collinear
        // reduction and equal-class merging. Both exact ends identify the same
        // window; matching owners alone could cross-link separated U-shaped arms.
        Signed832 one = Signed832.ExtendValue(Signed192.One);
        Signed832 lowerN = source.Interval.IsClipped ? source.Interval.WideLowerNumerator : default;
        Signed832 lowerD = source.Interval.IsClipped ? source.Interval.WideLowerDenominator : one;
        Signed832 upperN = source.Interval.IsClipped ? source.Interval.WideUpperNumerator : one;
        Signed832 upperD = source.Interval.IsClipped ? source.Interval.WideUpperDenominator : one;
        var edge = new FixedSegment(source.Start, source.End);
        var other = new FixedSegment(target.Start, target.End);
        return WideTriangleRelations.LocateEdgePoint(edge, sourceMesh.GeometryFrame, lowerN, lowerD,
                other, targetMesh.GeometryFrame, target.Interval) >= 0
            && WideTriangleRelations.LocateEdgePoint(edge, sourceMesh.GeometryFrame, upperN, upperD,
                other, targetMesh.GeometryFrame, target.Interval) >= 0;
    }

    private int RegionCount(int member) => (member + 1 < _joined.Inputs.Count
        ? _joined.Members[member + 1].Offset : _joined.Parents.Count) - _joined.Members[member].Offset;

    private int SupportEdgeRegion(int index, int first, int second)
    {
        if (RegionCount(index) == 1) return 0;
        var member = _joined.Members[index];
        return MeshSurfaceBoundaryTopology.FindSupportEdgeRegion(member.Collider.Mesh, member.Owner,
            first, second, member.Connectivity, this);
    }

    internal bool IntersectsSupport(FixedSegment segment) => IntersectsSupport(segment, default);

    private bool IntersectsSupport(FixedSegment segment, in RationalSegmentInterval clip)
    {
        Span<Vector3d> points = stackalloc Vector3d[2] { segment.Start, segment.End };
        var mesh = _joined.Members[_member].Collider.Mesh;
        _supportQuery = true; _supportClip[0] = clip;
        bool intersects = _supportPenetration.IsScaled
            ? WideOrientedBox.VisitConvexHullContacts(_movingMesh.GeometryFrame, _movingMesh.Origin,
                MovingPoints, MovingTriangles, MovingEdges, mesh.GeometryFrame, mesh.Origin,
                points, ReadOnlySpan<int>.Empty, TriangleEdges.AsSpan(0, 2), _supportPenetration.Scaled,
                this, static (owner, contact) => owner.AddSample(contact), this, admissionOnly: true) > 0
            : WideOrientedBox.VisitConvexHullContacts(_movingMesh.Origin, _movingMesh.Rotation,
            MovingPoints, MovingTriangles, MovingEdges,
            mesh.Origin, mesh.Rotation, points, ReadOnlySpan<int>.Empty, TriangleEdges.AsSpan(0, 2), _supportPenetration.Rigid,
            this, static (owner, contact) => owner.AddSample(contact), this, admissionOnly: true) > 0;
        _supportQuery = false;
        return intersects;
    }

    private int VisitFacePatch(PhysicsMesh mesh, ReadOnlySpan<Vector3d> points, ReadOnlySpan<int> triangles,
        ReadOnlySpan<int> edges, in NativePenetration penetration, bool admissionOnly) => penetration.IsScaled
        ? WideOrientedBox.VisitConvexHullContacts(_movingMesh.GeometryFrame, _movingMesh.Origin,
            MovingPoints, MovingTriangles, MovingEdges, mesh.GeometryFrame, mesh.Origin,
            points, triangles, edges, penetration.Scaled,
            this, static (owner, contact) => owner.AddSample(contact), admissionOnly: admissionOnly)
        : WideOrientedBox.VisitConvexHullContacts(_movingMesh.Origin, _movingMesh.Rotation,
            MovingPoints, MovingTriangles, MovingEdges,
            mesh.Origin, mesh.Rotation, points, triangles, edges, penetration.Rigid,
            this, static (owner, contact) => owner.AddSample(contact), admissionOnly: admissionOnly);

    private int Visit(in NativePenetration penetration, bool admissionOnly) => penetration.IsScaled
        ? WideOrientedBox.VisitConvexHullContacts(_movingMesh.GeometryFrame, _movingMesh.Origin,
            MovingPoints, MovingTriangles, MovingEdges, _joined.Members[_member].Collider.Mesh.GeometryFrame,
            _joined.Members[_member].Collider.Mesh.Origin, _points.AsReadOnlySpan(), _triangles.AsReadOnlySpan(),
            _edges.AsReadOnlySpan(), penetration.Scaled, this, static (owner, contact) => owner.AddSample(contact), this, admissionOnly)
        : WideOrientedBox.VisitConvexHullContacts(_movingMesh.Origin, _movingMesh.Rotation,
            MovingPoints, MovingTriangles, MovingEdges,
            _joined.Members[_member].Collider.Mesh.Origin, _joined.Members[_member].Collider.Mesh.Rotation,
            _points.AsReadOnlySpan(), _triangles.AsReadOnlySpan(), _edges.AsReadOnlySpan(), penetration.Rigid,
            this, static (owner, contact) => owner.AddSample(contact), this, admissionOnly);

    private void BuildContacts(NativePenetration penetration)
    {
        // An already collected plane group is also its exact finite admission certificate.
        if ((_movingFeature < 0 || _movingPatch) && IsFaceNormal(penetration.Axis) && FindGroup(penetration) < _groups.Count) return;
        // Native triangle contacts historically orient by centroids. Response
        // admission must rank the actual signed pushes instead of that legacy orientation.
        if (_movingFeature >= 0 && !_movingPatch || !AdmitAxis(penetration))
        {
            // Admission changes response ranking, never the already proved overlap.
            bool overlaps = FindPenetration(out penetration, filtered: true);
            System.Diagnostics.Debug.Assert(overlaps);
            if (!penetration.HasValue) return;
        }
        int component = _joined.FindResponseComponent(_member);
        WideDirection3d axis = penetration.Negate ? -penetration.Axis : penetration.Axis;
        _group = FindGroup(penetration);
        bool face = IsFaceNormal(axis);
        if (_group == _groups.Count) _groups.Add(new(component, axis, penetration, _movingFeature, _joined.GetResponseMember(component)));
        else if (face) return; // The first accepted plane already collected every supplier.
        if (face) VisitFace(penetration, admissionOnly: false);
        else Visit(penetration, admissionOnly: false);
    }

    private int FindGroup(in NativePenetration penetration)
    {
        int component = _joined.FindResponseComponent(_member);
        WideDirection3d axis = penetration.Negate ? -penetration.Axis : penetration.Axis;
        int group = 0;
        for (; group < _groups.Count; group++)
        {
            ResponseGroup existing = _groups[group];
            if (existing.Component == component && existing.Feature == _movingFeature && WideDirection3d.AreParallel(axis, existing.Axis)
                && WideDirection3d.Dot(axis, existing.Axis).Sign > 0
                && !existing.Penetration.ShouldReplace(penetration)
                && !penetration.ShouldReplace(existing.Penetration)) break;
        }
        return group;
    }

    private void AddSample(FixedContactAnchors contact)
    {
        // Rank final points in one translated world chart without narrowing an absolute world position.
        bool represented = contact.SecondAnchor.TryGetPointInFrame(_movingCenter, out Vector3d point);
        System.Diagnostics.Debug.Assert(represented);
        _samples.Add(new(_sampleRegion < 0 ? _group : _sampleRegion, _member, point, contact));
    }

    internal bool Append(LSMeshCollider collider, LSMeshCollider mover, ContactManifold manifold, bool reverse)
    {
        Span<int> chosen = stackalloc int[ContactManifold.MaxContactsPerGroup];
        for (int group = 0; group < _groups.Count; group++)
        {
            ResponseGroup response = _groups[group];
            if (_joined.Members[response.Carrier].Collider != collider) continue;
            _pool.FastClear();
            foreach (Sample sample in _samples.AsReadOnlySpan()) if (sample.Group == group) _pool.Add(sample);
            _pool.SortInPlace(PointComparer);
            int unique = 0;
            foreach (Sample sample in _pool.AsReadOnlySpan())
            {
                if (unique > 0 && ComparePoints(sample, _pool[unique - 1]) == 0) continue;
                _pool[unique++] = sample;
            }
            ReadOnlySpan<Sample> pool = _pool.AsReadOnlySpan()[..unique];
            int count = Math.Min(pool.Length, chosen.Length);
            if (response.Penetration.IsScaled)
            {
                for (int slot = 0; slot < count; slot++)
                    chosen[slot] = slot == 0 ? 0 : MeshPlaneSampleReduction.ChooseNext(pool, chosen[..slot], response.Axis, PointComparer);
                if (count > 2 && WidePlaneMetrics.GetAreaSign(response.Axis,
                    pool[chosen[0]].Point, pool[chosen[1]].Point, pool[chosen[2]].Point) == 0) count = 2;
            }
            else
            {
                var metrics = new WidePlaneMetrics(response.Penetration.Rigid.Axis);
                for (int slot = 0; slot < count; slot++)
                    chosen[slot] = slot == 0 ? 0 : MeshPlaneSampleReduction.ChooseNext(pool, chosen[..slot], metrics, PointComparer);
                if (count > 2 && metrics.GetAreaSquared(pool[chosen[0]].Point, pool[chosen[1]].Point, pool[chosen[2]].Point).IsZero) count = 2;
            }
            for (int slot = 0; slot < count; slot++)
            {
                Sample sample = pool[chosen[slot]]; var supplier = _joined.Members[sample.Member];
                FixedContactAnchors contact = sample.Contact;
                int movingSurface = response.Feature < 0 ? 0 : _movingMesh.GetCanonicalSurfaceOrdinal(response.Feature) + 1;
                var key = new ContactGroupKey(0, 0, supplier.Collider.Mesh.GetCanonicalSurfaceOrdinal(supplier.Owner) + 1,
                    movingSurface, group, sourceIdA: supplier.Collider.Id + 1);
                manifold.AddContact(new ContactAnchor(reverse ? contact.FirstAnchor : contact.SecondAnchor),
                    new ContactAnchor(reverse ? contact.SecondAnchor : contact.FirstAnchor), contact.Depth,
                    reverse ? contact.Normal : -contact.Normal, reverse ? mover.Material : supplier.Collider.Material,
                    reverse ? supplier.Collider.Material : mover.Material, contact.DepthIsClamped,
                    group: reverse ? key.Remap(0, 0, true) : key);
            }
        }
        return _overlappingMembers.Contains(collider.Id);
    }

    ReadOnlySpan<RationalSegmentInterval> IConvexHullContactDomain.GetEdgeIntervals(bool first, int start, int end,
        in WideAxis3 normal) => GetEdgeIntervals(first, start, end);

    public ReadOnlySpan<RationalSegmentInterval> GetEdgeIntervals(bool first, int start, int end,
        in WideDirection3d normal) => GetEdgeIntervals(first, start, end);

    private ReadOnlySpan<RationalSegmentInterval> GetEdgeIntervals(bool first, int start, int end)
    {
        if (first) return FullInterval;
        if (_supportQuery) return _supportClip;
        var member = _joined.Members[_member];
        if (!member.BoundaryTrusted) return FullInterval;
        // Face visitation uses the filled supplier directly; only nonface
        // candidates and the support-query branch above borrow this domain.
        _intervals.FastClear();
        for (int span = 0; span < member.Boundary.Spans.Length; span++)
        {
            var edge = member.Boundary.Spans[span];
            if (member.JoinedSpans[span]) continue;
            RationalSegmentInterval interval = member.GetRetainedInterval(ref span);
            if (MapBoundaryInterval(edge, interval, start, end, out var mapped)) _intervals.Add(mapped);
        }
        return _intervals.AsReadOnlySpan();
    }

    private bool MapBoundaryInterval(in MeshSurfaceBoundaryDomain.BoundarySpan edge, in RationalSegmentInterval interval,
        int start, int end, out RationalSegmentInterval mapped)
    {
        if (edge.First == _vertices[start] && edge.Second == _vertices[end])
        { mapped = interval; return true; }
        // Canonical perimeters merge authored collinear subedges. Remap their
        // cuts before witness selection; matching only vertex IDs loses real rims.
        // Authored coordinates can be large even when scaled reach is small;
        // the interval owner retains full products before reducing these bounds.
        return interval.TryMapToSegment(new FixedSegment(edge.Start, edge.End),
            new FixedSegment(_points[start], _points[end]), out mapped);
    }

    bool IConvexHullContactDomain.ContainsPoint(bool first, int start, int end, in Signed832 numerator,
        in Signed832 denominator, in WideAxis3 normal) => ContainsPoint(first, start, end, numerator, denominator, new WideDirection3d(normal));

    public bool ContainsPoint(bool first, int start, int end, in Signed832 numerator, in Signed832 denominator, in WideDirection3d normal)
    {
        if (first) return true;
        if (_supportQuery)
        {
            var clip = _supportClip[0];
            if (!clip.IsClipped) return true;
            Signed832 n = start == end ? (start == 0 ? default : denominator) : numerator;
            return WideTriangleRelations.CompareCoreParameters(n, denominator,
                clip.WideLowerNumerator, clip.WideLowerDenominator) >= 0
                && WideTriangleRelations.CompareCoreParameters(n, denominator,
                clip.WideUpperNumerator, clip.WideUpperDenominator) <= 0;
        }
        var member = _joined.Members[_member];
        if (!member.BoundaryTrusted) return true;
        int a = _vertices[start], b = _vertices[end];
        if (start == end || numerator.IsZero || WideArithmetic.SubtractSigned832(denominator, numerator).IsZero)
        {
            int vertex = start == end || numerator.IsZero ? a : b;
            return SupportsJoinedFan(_joined.Fan.BuildAtPoint(_joined.Meshes.AsReadOnlySpan(), member.Boundary.SourceMember,
                member.Owner, member.Collider.Mesh.GeometryVertices[vertex], _joined.PreparedBoundaries.AsReadOnlySpan()), normal);
        }
        for (int span = 0; span < member.Boundary.Spans.Length; span++)
        {
            var edge = member.Boundary.Spans[span];
            if (member.JoinedSpans[span] || !MapBoundaryInterval(edge, edge.Interval, start, end, out var mapped)) continue;
            Signed832 lowerN = mapped.WideLowerNumerator, lowerD = mapped.WideLowerDenominator;
            Signed832 upperN = mapped.WideUpperNumerator, upperD = mapped.WideUpperDenominator;
            if (!mapped.IsClipped) { lowerN = default; lowerD = upperN = upperD = Signed832.ExtendValue(Signed192.Signed(1)); }
            int lower = WideTriangleRelations.CompareCoreParameters(numerator, denominator, lowerN, lowerD);
            int upper = WideTriangleRelations.CompareCoreParameters(numerator, denominator, upperN, upperD);
            if (lower < 0 || upper > 0) continue;
            if (lower > 0 && upper < 0) return true;
            int coordinate = edge.Start.X != edge.End.X ? 0 : edge.Start.Y != edge.End.Y ? 1 : 2;
            bool reverse = (_points[start][coordinate] > _points[end][coordinate]) != (edge.Start[coordinate] > edge.End[coordinate]);
            return SupportsJoinedFan(_joined.Fan.BuildEndpoint(_joined.Meshes.AsReadOnlySpan(), member.Boundary, span, (upper == 0) != reverse,
                _joined.PreparedBoundaries.AsReadOnlySpan()), normal);
        }
        return false;
    }

    public bool ContainsPoint(bool first, int start, int end, scoped ContactQuadratic numerator,
        scoped ContactQuadratic denominator, in WideDirection3d normal)
    {
        if (first) return true;
        if (_supportQuery)
        {
            var clip = _supportClip[0];
            return !clip.IsClipped || clip.Contains(numerator, denominator, ReadOnlySpan<ulong>.Empty);
        }
        var member = _joined.Members[_member];
        if (!member.BoundaryTrusted) return true;
        int a = _vertices[start], b = _vertices[end];
        bool atStart = numerator.Signs[0] == 0;
        bool atEnd = CompareBorrowedParameter(numerator, denominator, Signed832.ExtendValue(Signed192.One),
            Signed832.ExtendValue(Signed192.One)) == 0;
        // Borrowed ratios come only from the native edge/edge fallback;
        // point/face witnesses use the compact overload above.
        if (atStart || atEnd)
        {
            int vertex = atStart ? a : b;
            return SupportsJoinedFan(_joined.Fan.BuildAtPoint(_joined.Meshes.AsReadOnlySpan(), member.Boundary.SourceMember,
                member.Owner, member.Collider.Mesh.GeometryVertices[vertex], _joined.PreparedBoundaries.AsReadOnlySpan()), normal);
        }
        Signed832 one = Signed832.ExtendValue(Signed192.One);
        for (int span = 0; span < member.Boundary.Spans.Length; span++)
        {
            var edge = member.Boundary.Spans[span];
            if (member.JoinedSpans[span] || !MapBoundaryInterval(edge, edge.Interval, start, end, out var mapped)) continue;
            int lower = CompareBorrowedParameter(numerator, denominator,
                mapped.IsClipped ? mapped.WideLowerNumerator : default, mapped.IsClipped ? mapped.WideLowerDenominator : one);
            int upper = CompareBorrowedParameter(numerator, denominator,
                mapped.IsClipped ? mapped.WideUpperNumerator : one, mapped.IsClipped ? mapped.WideUpperDenominator : one);
            if (lower < 0 || upper > 0) continue;
            if (lower > 0 && upper < 0) return true;
            int coordinate = edge.Start.X != edge.End.X ? 0 : edge.Start.Y != edge.End.Y ? 1 : 2;
            bool reverse = (_points[start][coordinate] > _points[end][coordinate]) != (edge.Start[coordinate] > edge.End[coordinate]);
            return SupportsJoinedFan(_joined.Fan.BuildEndpoint(_joined.Meshes.AsReadOnlySpan(), member.Boundary, span,
                (upper == 0) != reverse, _joined.PreparedBoundaries.AsReadOnlySpan()), normal);
        }
        return false;
    }

    private static int CompareBorrowedParameter(scoped ContactQuadratic numerator, scoped ContactQuadratic denominator,
        Signed832 otherN, Signed832 otherD)
    {
        Span<ulong> values = stackalloc ulong[52]; Span<int> signs = stackalloc int[4];
        var n = ContactQuadratic.At(values, signs, 0, 13); var d = ContactQuadratic.At(values, signs, 1, 13);
        n.Set(otherN); d.Set(otherD);
        return ContactQuadratic.CompareRatios(numerator, denominator, ReadOnlySpan<ulong>.Empty,
            n, d, ReadOnlySpan<ulong>.Empty);
    }

    private bool IsFaceNormal(in WideDirection3d normal)
    {
        return WideDirection3d.AreParallel(normal, _joined.Members[_member].Frame);
    }

    private bool SupportsJoinedFan(bool built, in WideDirection3d normal)
    {
        // Failed or ineligible neighbor certificates preserve the native feature.
        if (!built) return true;
        foreach (var owner in _joined.Fan.Owners)
            if (!_joined.MemberBySurface.TryGetValue((_joined.Meshes[owner.Member], owner.Surface), out int member)
                || !_joined.Members[member].BoundaryTrusted) return true;
        return Supports(_joined.Fan.Rays, normal);
    }

    private bool Supports(ReadOnlySpan<WideAxis3> rays, in WideDirection3d normal)
    {
        WideDirection3d inward = -normal;
        foreach (WideAxis3 ray in rays)
        {
            if (WideDirection3d.Dot(inward, ray).Sign > 0) return false;
        }
        return true;
    }

    private static int ComparePoints(Sample a, Sample b)
    {
        int order = a.Point.X.CompareTo(b.Point.X);
        if (order == 0) order = a.Point.Y.CompareTo(b.Point.Y);
        if (order == 0) order = a.Point.Z.CompareTo(b.Point.Z);
        return order != 0 ? order : a.Contact.FirstAnchor.CompareLocalFeature(b.Contact.FirstAnchor);
    }
    private static int CompareSamples(Sample a, Sample b)
    { int order = ComparePoints(a, b); return order != 0 ? order : a.Member.CompareTo(b.Member); }
    private readonly struct Sample : IPlaneContactSample
    {
        internal readonly int Group, Member;
        internal readonly FixedContactAnchors Contact;
        public Vector3d Point { get; }
        internal Sample(int group, int member, Vector3d point, FixedContactAnchors contact)
        { Group = group; Member = member; Point = point; Contact = contact; }
    }
    private readonly struct ResponseGroup
    {
        internal readonly int Component, Feature, Carrier;
        internal readonly WideDirection3d Axis;
        internal readonly NativePenetration Penetration;
        internal ResponseGroup(int component, WideDirection3d axis, NativePenetration penetration, int feature, int carrier)
        { Component = component; Axis = axis; Penetration = penetration; Feature = feature; Carrier = carrier; }
    }

    private readonly struct NativePenetration
    {
        internal readonly WidePointSpanPenetration Rigid;
        internal readonly WideScaledHullPenetration Scaled;
        internal readonly bool IsScaled;
        internal NativePenetration(WidePointSpanPenetration value) { this = default; Rigid = value; }
        internal NativePenetration(WideScaledHullPenetration value) { this = default; Scaled = value; IsScaled = true; }
        internal bool HasValue => IsScaled ? Scaled.HasValue : Rigid.HasValue;
        internal bool Negate => IsScaled ? Scaled.Negate : Rigid.Negate;
        internal WideDirection3d Axis => IsScaled ? Scaled.Axis : new WideDirection3d(Rigid.Axis);
        private Signed832 Overlap => IsScaled ? Scaled.Overlap : Signed832.ExtendValue(Rigid.ExactOverlap);
        private Signed832 SquaredAxisLength => IsScaled ? Scaled.SquaredAxisLength : Signed832.ExtendValue(Rigid.ExactSquaredAxisLength);
        private Signed576 Denominator => IsScaled ? Scaled.CommonDenominator : Signed576.ExtendValue(Rigid.ExactCommonDenominator);
        // Group comparison sees only successful SAT results or constructed
        // admission candidates; a filtered empty result is rejected earlier.
        internal bool ShouldReplace(in NativePenetration candidate) => !IsScaled && !candidate.IsScaled
            ? Rigid.ShouldReplace(candidate.Rigid.ExactOverlap, candidate.Rigid.ExactSquaredAxisLength, candidate.Rigid.ExactCommonDenominator)
            : WideArithmetic.CompareNonNegativeNormalizedDepths(candidate.Overlap, candidate.SquaredAxisLength, candidate.Denominator,
                Overlap, SquaredAxisLength, Denominator) < 0;
    }
}

