//=======================================================================
// MeshSurfaceConnectivity.cs
//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================
using FixedMathSharp;
using FixedMathSharp.Geometry;
using Gravitas.Colliders;
using SwiftCollections;
using System;
using System.Collections.Generic;

namespace Gravitas.CollisionHandling;

/// <summary>Retains exact admitted connectivity for one prepared mesh surface.</summary>
internal sealed class MeshSurfaceConnectivity
{
    private readonly SwiftDictionary<int, int> _nodeByTriangle = new();
    private readonly SwiftList<int> _triangles = new();
    private readonly SwiftList<int> _parents = new();
    private readonly SwiftList<int> _regionByRoot = new();
    private readonly SwiftList<int> _roots = new();
    private readonly SwiftHashSet<int> _visitedVertices = new();
    private readonly IComparer<int> _regionComparer;
    private ulong[] _minimumValues = Array.Empty<ulong>();
    private int[] _minimumSigns = Array.Empty<int>();
    private bool[] _hasMinimum = Array.Empty<bool>();
    private int[] _boundaryRanks = Array.Empty<int>();
    private bool _ordersSupport;

    internal int RegionCount => _roots.Count;

    internal ReadOnlySpan<int> TriangleIndices => _triangles.AsReadOnlySpan();

    internal bool IsCompleteSingleRegion(PhysicsMesh mesh)
    {
        if (RegionCount != 1) return false;
        // Prepared connected owners record every shared-edge neighbor. A
        // nonempty admitted subset closed under them contains the whole owner.
        foreach (int triangle in TriangleIndices)
            foreach (int neighbor in mesh.GetCoplanarTriangleNeighbors(triangle))
                if (GetRegionOrdinal(neighbor) != 0) return false;
        return true;
    }

    internal MeshSurfaceConnectivity() => _regionComparer = Comparer<int>.Create(CompareRegions);

    internal void Build(PhysicsMesh mesh, int surfaceTriangle, Vector3d coneCenter,
        FixedQuaternion coneRotation, Fixed64 height, Fixed64 radius, ReadOnlySpan<int> admittedTriangles)
    {
        mesh.GetLocalTriangleVertices(surfaceTriangle, out Vector3d a, out Vector3d b, out Vector3d c);
        var frame = new ConePlaneRayFrame(new FixedTriangle(a, b, c), mesh.Origin, mesh.Rotation,
            coneCenter, coneRotation, height, radius);
        Build(mesh, surfaceTriangle, frame, admittedTriangles);
    }

    internal void BuildCapsule(PhysicsMesh mesh, int surfaceTriangle, Vector3d capsuleCenter,
        FixedQuaternion capsuleRotation, Fixed64 coreLength, Fixed64 radius, ReadOnlySpan<int> admittedTriangles)
    {
        var frame = new CapsuleFiniteSectionFrame(mesh.Origin, mesh.Rotation, capsuleCenter, capsuleRotation, coreLength, radius);
        Build(mesh, surfaceTriangle, new CapsuleSection(frame), admittedTriangles);
    }

    internal void BuildCapsule(PhysicsMesh mesh, int surfaceTriangle, in CapsuleFiniteSectionFrame frame,
        ReadOnlySpan<int> admittedTriangles) => Build(mesh, surfaceTriangle, new CapsuleSection(frame), admittedTriangles);

    internal void BuildSlab(PhysicsMesh mesh, int surfaceTriangle, in FiniteSlabSectionFrame frame,
        LSCollider2D? embedded, ReadOnlySpan<int> admittedTriangles) =>
        Build(mesh, surfaceTriangle, new SlabSection(frame, embedded), admittedTriangles);

    internal void BuildSlab(PhysicsMesh mesh, int surfaceTriangle, in FiniteSlabSectionFrame frame,
        ReadOnlyMemory<Vector2d> offsets, ReadOnlySpan<int> admittedTriangles) =>
        Build(mesh, surfaceTriangle, new SlabSection(frame, null, offsets), admittedTriangles);

    internal void Build(PhysicsMesh mesh, int surfaceTriangle, in ConePlaneRayFrame frame, ReadOnlySpan<int> admittedTriangles) =>
        Build(mesh, surfaceTriangle, new ConeSection(frame), admittedTriangles);

    /// <remarks>
    /// The input contains every exactly admitted triangle, without duplicates.
    /// Only the seed's prepared surface owner is consumed. Preparation alone
    /// never establishes connectivity: every join needs finite shape admission.
    /// </remarks>
    private void Build<TSection>(PhysicsMesh mesh, int surfaceTriangle, in TSection section, ReadOnlySpan<int> admittedTriangles)
        where TSection : struct, ISection
    {
        BuildGraph(mesh, surfaceTriangle, section, admittedTriangles);
        _ordersSupport = false;
        OrderRegions(mesh, surfaceTriangle, section);
    }

    private void BuildGraph<TSection>(PhysicsMesh mesh, int surfaceTriangle, in TSection section, ReadOnlySpan<int> admittedTriangles)
        where TSection : struct, IAdmission
    {
        _nodeByTriangle.Clear(); _triangles.FastClear(); _parents.FastClear();
        _regionByRoot.FastClear(); _roots.FastClear();
        _visitedVertices.Clear();
        if (admittedTriangles.IsEmpty) return;
        int owner = mesh.GetManifoldSurfaceOwner(surfaceTriangle);
        _nodeByTriangle.EnsureCapacity(admittedTriangles.Length);
        _triangles.EnsureCapacity(admittedTriangles.Length);
        _parents.EnsureCapacity(admittedTriangles.Length);
        _regionByRoot.EnsureCapacity(admittedTriangles.Length);
        _roots.EnsureCapacity(admittedTriangles.Length);
        _visitedVertices.EnsureCapacity(admittedTriangles.Length * 3);
        foreach (int triangle in admittedTriangles)
        {
            if (mesh.GetManifoldSurfaceOwner(triangle) != owner) continue;
            int node = _triangles.Count;
            _nodeByTriangle.Add(triangle, node); _triangles.Add(triangle);
            _parents.Add(node); _regionByRoot.Add(-1);
        }
        if (_triangles.Count == 0) return;
        JoinAdmittedFeatures(mesh, section);
        for (int node = 0; node < _triangles.Count; node++)
        {
            int root = Find(node);
            if (_regionByRoot[root] < 0)
            {
                _regionByRoot[root] = _roots.Count;
                _roots.Add(root);
            }
        }
    }

    private void OrderRegions<TSection>(PhysicsMesh mesh, int surfaceTriangle, in TSection section)
        where TSection : struct, ISection
    {
        // If a connected section misses the true domain boundary it is the
        // whole convex shape section, hence the only region. Otherwise exact
        // minima on exposed segments order regions independently of authored
        // diagonals, discovery order and collinear perimeter subdivision.
        if (_roots.Count > 1)
        {
            // One region always has ordinal zero. Its exact boundary extrema
            // cannot affect ordering, so neither retain nor evaluate them.
            if (_minimumValues.Length < _roots.Count * FiniteSectionPoint.StorageWords)
            {
                Array.Resize(ref _minimumValues, _roots.Count * FiniteSectionPoint.StorageWords);
                Array.Resize(ref _minimumSigns, _roots.Count * FiniteSectionPoint.SignCount);
                Array.Resize(ref _hasMinimum, _roots.Count);
            }
            Array.Clear(_hasMinimum, 0, _roots.Count);
            FindBoundaryMinima(mesh, surfaceTriangle, section);
#if DEBUG
            for (int region = 0; region < _roots.Count; region++)
                System.Diagnostics.Debug.Assert(_hasMinimum[region]);
#endif
            _roots.SortInPlace(_regionComparer);
        }
        for (int region = 0; region < _roots.Count; region++)
            _regionByRoot[_roots[region]] = region;
    }

    internal void BuildSlabSupport(PhysicsMesh mesh, int surfaceTriangle, MeshSlabFaceRegions faces,
        LSCollider2D? embedded, ReadOnlySpan<int> admittedTriangles) =>
        BuildSupport(mesh, surfaceTriangle, new SlabSupport(faces, embedded), admittedTriangles);

    internal void BuildHullSupport(PhysicsMesh mesh, int surfaceTriangle, MeshHullSurfaceContacts faces,
        ReadOnlySpan<int> admittedTriangles) =>
        BuildSupport(mesh, surfaceTriangle, new HullSupport(faces), admittedTriangles);

    private void BuildSupport<TSupport>(PhysicsMesh mesh, int surfaceTriangle, in TSupport support,
        ReadOnlySpan<int> admittedTriangles) where TSupport : struct, ISupport
    {
        BuildGraph(mesh, surfaceTriangle, support, admittedTriangles);
        _ordersSupport = true;
        if (_roots.Count > 1)
        {
            if (_boundaryRanks.Length < _parents.Count) Array.Resize(ref _boundaryRanks, _parents.Count);
            Array.Fill(_boundaryRanks, int.MaxValue, 0, _parents.Count);
            ReadOnlySpan<int> edges = mesh.GetCanonicalSurfaceBoundaryVertexPairs(surfaceTriangle);
            int owner = mesh.GetManifoldSurfaceOwner(surfaceTriangle);
            for (int edge = 0; edge < edges.Length; edge += 2)
            {
                int region = support.FindEdgeRegion(mesh, owner, edges[edge], edges[edge + 1], this);
                if (region >= 0) _boundaryRanks[_roots[region]] = Math.Min(_boundaryRanks[_roots[region]], edge / 2);
            }
            // A convex support footprint meets a straight boundary span in one
            // interval: distinct connected regions have distinct boundary ranks.
#if DEBUG
            foreach (int root in _roots.AsReadOnlySpan()) System.Diagnostics.Debug.Assert(_boundaryRanks[root] != int.MaxValue);
#endif
            _roots.SortInPlace(_regionComparer);
        }
        for (int region = 0; region < _roots.Count; region++) _regionByRoot[_roots[region]] = region;
    }

    internal int GetRegionOrdinal(int triangleIndex) => _nodeByTriangle.TryGetValue(triangleIndex, out int node)
        ? _regionByRoot[Find(node)] : -1;

    private void JoinAdmittedFeatures<TSection>(PhysicsMesh mesh, in TSection section) where TSection : struct, IAdmission
    {
        int components = _triangles.Count;
        if (components == 1) return;
        // Exact admitted joins are transitive. Once every node is connected,
        // further feature tests cannot change the single region or its ordinal.
        ReadOnlySpan<Vector3d> vertices = mesh.ScaledLocalVertices;
        for (int node = 0; node < _triangles.Count; node++)
        {
            int triangle = _triangles[node];
            ReadOnlySpan<int> welded = mesh.GetWeldedTriangleVertexIndices(triangle);
            foreach (int neighbor in mesh.GetCoplanarTriangleNeighbors(triangle))
            {
                if (!_nodeByTriangle.TryGetValue(neighbor, out int other) || other <= node) continue;
                ReadOnlySpan<int> adjacent = mesh.GetWeldedTriangleVertexIndices(neighbor);
                int first = -1, second = -1;
                foreach (int vertex in welded)
                    if (adjacent.IndexOf(vertex) >= 0)
                    {
                        if (first < 0) first = vertex;
                        else second = vertex;
                    }
                System.Diagnostics.Debug.Assert(first >= 0 && second >= 0);
                if (section.IntersectsSegment(new FixedSegment(vertices[first], vertices[second]))
                    && Union(node, other) && --components == 1) return;
            }
            foreach (int vertex in welded)
            {
                // Each welded star is traversed once, even when an admitted
                // center belongs to every face of a large planar fan.
                if (!_visitedVertices.Add(vertex) || !section.ContainsPoint(vertices[vertex])) continue;
                foreach (int incident in mesh.GetWeldedVertexTriangleIndices(vertex))
                    if (_nodeByTriangle.TryGetValue(incident, out int other)
                        && Union(node, other) && --components == 1) return;
            }
        }
    }

    private void FindBoundaryMinima<TSection>(PhysicsMesh mesh, int surfaceTriangle, in TSection section) where TSection : struct, ISection
    {
        Span<ulong> values = stackalloc ulong[FiniteSectionPoint.StorageWords];
        Span<int> signs = stackalloc int[FiniteSectionPoint.SignCount];
        var point = new FiniteSectionPoint(values, signs);
        ReadOnlySpan<int> pairs = mesh.GetManifoldSurfaceBoundaryVertexPairs(surfaceTriangle);
        ReadOnlySpan<Vector3d> vertices = mesh.ScaledLocalVertices;
        // Multiple regions require a certified patch with a true perimeter.
        // A declined owner contains only its single convex triangle section.
        System.Diagnostics.Debug.Assert(!pairs.IsEmpty);
        for (int edge = 0; edge < pairs.Length; edge += 2) KeepBoundaryPoint(mesh, pairs[edge], pairs[edge + 1], section, point, vertices);
    }

    private void KeepBoundaryPoint<TSection>(PhysicsMesh mesh, int first, int second, in TSection section,
        FiniteSectionPoint point, ReadOnlySpan<Vector3d> vertices) where TSection : struct, ISection
    {
        if (!section.TryGetFirstSegmentPoint(new FixedSegment(vertices[first], vertices[second]), point)) return;
        foreach (int triangle in mesh.GetWeldedVertexTriangleIndices(first))
        {
            if (!_nodeByTriangle.TryGetValue(triangle, out int node)
                || mesh.GetWeldedTriangleVertexIndices(triangle).IndexOf(second) < 0) continue;
            int region = _regionByRoot[Find(node)];
            FiniteSectionPoint minimum = Minimum(region);
            if (!_hasMinimum[region] || FiniteSectionPoint.CompareLexicographic(point, minimum) < 0)
            {
                point.CopyTo(minimum); _hasMinimum[region] = true;
            }
            break; // A trusted exposed edge has one owner on this surface.
        }
    }

    private int CompareRegions(int first, int second) => _ordersSupport ? _boundaryRanks[first].CompareTo(_boundaryRanks[second])
        : FiniteSectionPoint.CompareLexicographic(Minimum(_regionByRoot[first]), Minimum(_regionByRoot[second]));

    private FiniteSectionPoint Minimum(int region) => new(_minimumValues.AsSpan(region * FiniteSectionPoint.StorageWords, FiniteSectionPoint.StorageWords),
        _minimumSigns.AsSpan(region * FiniteSectionPoint.SignCount, FiniteSectionPoint.SignCount));

    private int Find(int node)
    {
        int root = node;
        while (_parents[root] != root) root = _parents[root];
        while (_parents[node] != node)
        {
            int next = _parents[node]; _parents[node] = root; node = next;
        }
        return root;
    }

    private bool Union(int first, int second)
    {
        first = Find(first); second = Find(second);
        if (first == second) return false;
        _parents[Math.Max(first, second)] = Math.Min(first, second);
        return true;
    }

    // Constrained value-type calls share the graph without delegates, boxing,
    // or storing both shape frames in every operation.
    private interface IAdmission
    {
        bool ContainsPoint(Vector3d point);
        bool IntersectsSegment(FixedSegment segment);
    }

    private interface ISection : IAdmission
    {
        bool TryGetFirstSegmentPoint(FixedSegment segment, FiniteSectionPoint point);
    }

    private interface ISupport : IAdmission
    {
        int FindEdgeRegion(PhysicsMesh mesh, int owner, int first, int second, MeshSurfaceConnectivity connectivity);
    }

    private readonly struct HullSupport : ISupport
    {
        private readonly MeshHullSurfaceContacts faces;
        internal HullSupport(MeshHullSurfaceContacts faces) => this.faces = faces;
        public bool ContainsPoint(Vector3d point) => IntersectsSegment(new FixedSegment(point, point));
        public bool IntersectsSegment(FixedSegment segment) => faces.IntersectsSupport(segment);
        public int FindEdgeRegion(PhysicsMesh mesh, int owner, int first, int second, MeshSurfaceConnectivity connectivity) =>
            MeshSurfaceBoundaryTopology.FindSupportEdgeRegion(mesh, owner, first, second, connectivity, faces);
    }

    private readonly struct SlabSupport : ISupport
    {
        private readonly MeshSlabFaceRegions faces;
        private readonly LSCollider2D? embedded;
        internal SlabSupport(MeshSlabFaceRegions faces, LSCollider2D? embedded)
        { this.faces = faces; this.embedded = embedded; }
        public bool ContainsPoint(Vector3d point) => IntersectsSegment(new FixedSegment(point, point));
        public bool IntersectsSegment(FixedSegment segment)
        {
            Span<Vector2d> box = stackalloc Vector2d[4];
            return faces.IntersectsSupport(segment, faces.PrismOffsets.IsEmpty
                ? CollisionDetectionMixed.GetConvexPrismOffsets(embedded, box) : faces.PrismOffsets.Span);
        }
        public int FindEdgeRegion(PhysicsMesh mesh, int owner, int first, int second, MeshSurfaceConnectivity connectivity) =>
            MeshSurfaceBoundaryTopology.FindSupportEdgeRegion(mesh, owner, first, second, connectivity, faces, embedded);
    }

    private readonly struct ConeSection : ISection
    {
        private readonly ConePlaneRayFrame frame;
        internal ConeSection(ConePlaneRayFrame frame) => this.frame = frame;
        public bool ContainsPoint(Vector3d point) => frame.ContainsPoint(point);
        public bool IntersectsSegment(FixedSegment segment) => frame.IntersectsSegment(segment);
        public bool TryGetFirstSegmentPoint(FixedSegment segment, FiniteSectionPoint point) =>
            FiniteSectionPoint.TryGetFirstSegmentPoint(segment, frame.Finite, point);
    }

    private readonly struct CapsuleSection : ISection
    {
        private readonly CapsuleFiniteSectionFrame frame;
        internal CapsuleSection(CapsuleFiniteSectionFrame frame) => this.frame = frame;
        public bool ContainsPoint(Vector3d point) => frame.ContainsPoint(point);
        public bool IntersectsSegment(FixedSegment segment) => frame.IntersectsSegment(segment);
        public bool TryGetFirstSegmentPoint(FixedSegment segment, FiniteSectionPoint point) =>
            FiniteSectionPoint.TryGetFirstSegmentPoint(segment, frame, point);
    }

    private readonly struct SlabSection : ISection
    {
        private readonly FiniteSlabSectionFrame frame;
        private readonly LSCollider2D? embedded;
        private readonly ReadOnlyMemory<Vector2d> offsets;
        internal SlabSection(in FiniteSlabSectionFrame frame, LSCollider2D? embedded, ReadOnlyMemory<Vector2d> offsets = default)
        { this.frame = frame; this.embedded = embedded; this.offsets = offsets; }
        public bool ContainsPoint(Vector3d point)
        {
            Span<Vector2d> box = stackalloc Vector2d[4];
            return frame.ContainsPoint(point, offsets.IsEmpty ? CollisionDetectionMixed.GetConvexPrismOffsets(embedded, box) : offsets.Span);
        }
        public bool IntersectsSegment(FixedSegment segment)
        {
            Span<Vector2d> box = stackalloc Vector2d[4];
            return frame.IntersectsSegment(segment, offsets.IsEmpty ? CollisionDetectionMixed.GetConvexPrismOffsets(embedded, box) : offsets.Span);
        }
        public bool TryGetFirstSegmentPoint(FixedSegment segment, FiniteSectionPoint point)
        {
            Span<Vector2d> box = stackalloc Vector2d[4];
            return FiniteSectionPoint.TryGetFirstSegmentPoint(segment, frame,
                offsets.IsEmpty ? CollisionDetectionMixed.GetConvexPrismOffsets(embedded, box) : offsets.Span, point);
        }
    }
}
