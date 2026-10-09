//=======================================================================
// MeshConeSurfaceConnectivity.cs
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
internal sealed class MeshConeSurfaceConnectivity
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

    internal int RegionCount => _roots.Count;

    internal MeshConeSurfaceConnectivity() => _regionComparer = Comparer<int>.Create(CompareRegions);

    internal void Build(PhysicsMesh mesh, int surfaceTriangle, Vector3d coneCenter,
        FixedQuaternion coneRotation, Fixed64 height, Fixed64 radius, ReadOnlySpan<int> admittedTriangles)
    {
        mesh.GetLocalTriangleVertices(surfaceTriangle, out Vector3d a, out Vector3d b, out Vector3d c);
        var frame = new ConePlaneRayFrame(new FixedTriangle(a, b, c), mesh.Origin, mesh.Rotation,
            coneCenter, coneRotation, height, radius);
        Build(mesh, surfaceTriangle, frame, admittedTriangles);
    }

    /// <remarks>
    /// The input contains every exactly admitted triangle, without duplicates.
    /// Only the seed's prepared surface owner is consumed. Preparation alone
    /// never establishes connectivity: every join needs finite cone admission.
    /// </remarks>
    internal void Build(PhysicsMesh mesh, int surfaceTriangle, in ConePlaneRayFrame frame, ReadOnlySpan<int> admittedTriangles)
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
        JoinAdmittedFeatures(mesh, frame);
        for (int node = 0; node < _triangles.Count; node++)
        {
            int root = Find(node);
            if (_regionByRoot[root] < 0)
            {
                _regionByRoot[root] = _roots.Count;
                _roots.Add(root);
            }
        }
        // If a connected section misses the true domain boundary it is the
        // whole convex cone section, hence the only region. Otherwise exact
        // minima on exposed segments order regions independently of authored
        // diagonals, discovery order and collinear perimeter subdivision.
        if (_roots.Count > 1)
        {
            // One region always has ordinal zero. Its exact boundary extrema
            // cannot affect ordering, so neither retain nor evaluate them.
            if (_minimumValues.Length < _roots.Count * ConeSectionPoint.StorageWords)
            {
                Array.Resize(ref _minimumValues, _roots.Count * ConeSectionPoint.StorageWords);
                Array.Resize(ref _minimumSigns, _roots.Count * ConeSectionPoint.SignCount);
                Array.Resize(ref _hasMinimum, _roots.Count);
            }
            Array.Clear(_hasMinimum, 0, _roots.Count);
            FindBoundaryMinima(mesh, surfaceTriangle, frame);
#if DEBUG
            for (int region = 0; region < _roots.Count; region++)
                System.Diagnostics.Debug.Assert(_hasMinimum[region]);
#endif
            _roots.SortInPlace(_regionComparer);
        }
        for (int region = 0; region < _roots.Count; region++)
            _regionByRoot[_roots[region]] = region;
    }

    internal int GetRegionOrdinal(int triangleIndex) => _nodeByTriangle.TryGetValue(triangleIndex, out int node)
        ? _regionByRoot[Find(node)] : -1;

    private void JoinAdmittedFeatures(PhysicsMesh mesh, in ConePlaneRayFrame frame)
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
                if (frame.IntersectsSegment(new FixedSegment(vertices[first], vertices[second]))
                    && Union(node, other) && --components == 1) return;
            }
            foreach (int vertex in welded)
            {
                // Each welded star is traversed once, even when an admitted
                // center belongs to every face of a large planar fan.
                if (!_visitedVertices.Add(vertex) || !frame.ContainsPoint(vertices[vertex])) continue;
                foreach (int incident in mesh.GetWeldedVertexTriangleIndices(vertex))
                    if (_nodeByTriangle.TryGetValue(incident, out int other)
                        && Union(node, other) && --components == 1) return;
            }
        }
    }

    private void FindBoundaryMinima(PhysicsMesh mesh, int surfaceTriangle, in ConePlaneRayFrame frame)
    {
        Span<ulong> values = stackalloc ulong[ConeSectionPoint.StorageWords];
        Span<int> signs = stackalloc int[ConeSectionPoint.SignCount];
        var point = new ConeSectionPoint(values, signs);
        ReadOnlySpan<int> pairs = mesh.GetManifoldSurfaceBoundaryVertexPairs(surfaceTriangle);
        ReadOnlySpan<Vector3d> vertices = mesh.ScaledLocalVertices;
        // Multiple regions require a certified patch with a true perimeter.
        // A declined owner contains only its single convex triangle section.
        System.Diagnostics.Debug.Assert(!pairs.IsEmpty);
        for (int edge = 0; edge < pairs.Length; edge += 2) KeepBoundaryPoint(mesh, pairs[edge], pairs[edge + 1], frame, point, vertices);
    }

    private void KeepBoundaryPoint(PhysicsMesh mesh, int first, int second, in ConePlaneRayFrame frame,
        ConeSectionPoint point, ReadOnlySpan<Vector3d> vertices)
    {
        if (!ConeSectionPoint.TryGetFirstSegmentPoint(new FixedSegment(vertices[first], vertices[second]), frame.Finite, point)) return;
        foreach (int triangle in mesh.GetWeldedVertexTriangleIndices(first))
        {
            if (!_nodeByTriangle.TryGetValue(triangle, out int node)
                || mesh.GetWeldedTriangleVertexIndices(triangle).IndexOf(second) < 0) continue;
            int region = _regionByRoot[Find(node)];
            ConeSectionPoint minimum = Minimum(region);
            if (!_hasMinimum[region] || ConeSectionPoint.CompareLexicographic(point, minimum) < 0)
            {
                point.CopyTo(minimum); _hasMinimum[region] = true;
            }
            break; // A trusted exposed edge has one owner on this surface.
        }
    }

    private int CompareRegions(int first, int second) => ConeSectionPoint.CompareLexicographic(
        Minimum(_regionByRoot[first]), Minimum(_regionByRoot[second]));

    private ConeSectionPoint Minimum(int region) => new(_minimumValues.AsSpan(region * ConeSectionPoint.StorageWords, ConeSectionPoint.StorageWords),
        _minimumSigns.AsSpan(region * ConeSectionPoint.SignCount, ConeSectionPoint.SignCount));

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
}
