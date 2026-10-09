//=======================================================================
// PhysicsMesh.SurfaceTopology.cs
//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp;
using SwiftCollections;
using System;
using System.Collections.Generic;

namespace Gravitas.Colliders;

/// <content>Retains canonical surface provenance and welded incidence for committed mesh geometry.</content>
public partial class PhysicsMesh
{
    private int[] _canonicalSurfaceOrdinals = Array.Empty<int>();
    private int[] _preparedCanonicalSurfaceOrdinals = Array.Empty<int>();
    private int[] _coplanarNeighborOffsets = Array.Empty<int>();
    private int[] _preparedCoplanarNeighborOffsets = Array.Empty<int>();
    private int[] _coplanarNeighbors = Array.Empty<int>();
    private int[] _preparedCoplanarNeighbors = Array.Empty<int>();
    private int[] _weldedTriangleVertices = Array.Empty<int>();
    private int[] _preparedWeldedTriangleVertices = Array.Empty<int>();
    private int[] _weldedVertexRepresentatives = Array.Empty<int>();
    private int[] _preparedWeldedVertexRepresentatives = Array.Empty<int>();
    private int[] _weldedVertexTriangleOffsets = Array.Empty<int>();
    private int[] _preparedWeldedVertexTriangleOffsets = Array.Empty<int>();
    private int[] _weldedVertexTriangles = Array.Empty<int>();
    private int[] _preparedWeldedVertexTriangles = Array.Empty<int>();

    private readonly SwiftList<(int First, int Second)> _surfaceNeighborPairs = new();
    private readonly SwiftList<SurfaceBoundaryEdge> _surfaceBoundaryEdges = new();
    private int[] _surfaceWriteOffsets = Array.Empty<int>();
    private int[] _surfaceKeyOffsets = Array.Empty<int>();
    private readonly SwiftList<int> _surfaceOrder = new();
    private int[] _surfacePreviousByVertex = Array.Empty<int>();
    private int[] _surfaceCycleVertices = Array.Empty<int>();
    private int[] _surfaceCycleOffsets = Array.Empty<int>();
    private Vector3d[] _surfacePreparationVertices = Array.Empty<Vector3d>();
    private IComparer<SurfaceBoundaryEdge>? _surfaceBoundaryEdgeComparer;
    private IComparer<int>? _surfaceOrderComparer;

    /// <summary>
    /// Gets the collision-free geometric ordering of the committed surface.
    /// Trusted patches use their exact boundary domain; declined patches retain
    /// individual triangle geometry. This is not an admitted contact-region ID.
    /// </summary>
    internal int GetCanonicalSurfaceOrdinal(int triangleIndex)
    {
        SwiftThrowHelper.ThrowIfArrayIndexInvalid(triangleIndex, _triangleCount, nameof(triangleIndex));
        return _canonicalSurfaceOrdinals[triangleIndex];
    }

    /// <summary>Gets trusted coplanar shared-edge neighbors in authored triangle order.</summary>
    internal ReadOnlySpan<int> GetCoplanarTriangleNeighbors(int triangleIndex)
    {
        SwiftThrowHelper.ThrowIfArrayIndexInvalid(triangleIndex, _triangleCount, nameof(triangleIndex));
        return _coplanarNeighbors.AsSpan(_coplanarNeighborOffsets[triangleIndex],
            _coplanarNeighborOffsets[triangleIndex + 1] - _coplanarNeighborOffsets[triangleIndex]);
    }

    /// <summary>Gets the committed exactly welded representatives of a triangle's three vertices.</summary>
    internal ReadOnlySpan<int> GetWeldedTriangleVertexIndices(int triangleIndex)
    {
        SwiftThrowHelper.ThrowIfArrayIndexInvalid(triangleIndex, _triangleCount, nameof(triangleIndex));
        return _weldedTriangleVertices.AsSpan(triangleIndex * 3, 3);
    }

    /// <summary>
    /// Gets all triangles incident to an authored vertex's committed welded position.
    /// Incidence alone does not establish admitted connectivity or trusted seam ownership.
    /// </summary>
    internal ReadOnlySpan<int> GetWeldedVertexTriangleIndices(int vertexIndex)
    {
        SwiftThrowHelper.ThrowIfArrayIndexInvalid(vertexIndex, VertexCount, nameof(vertexIndex));
        int representative = _weldedVertexRepresentatives[vertexIndex];
        return _weldedVertexTriangles.AsSpan(_weldedVertexTriangleOffsets[representative],
            _weldedVertexTriangleOffsets[representative + 1] - _weldedVertexTriangleOffsets[representative]);
    }

    private void PrepareWeldedSurfaceIncidence(int[] triangles)
    {
        // Snapshot before convex/canonical boundary preparation borrows the
        // welding representative map as outgoing boundary-link scratch.
        EnsureTopologyBuffer(ref _preparedWeldedTriangleVertices, triangles.Length);
        EnsureTopologyBuffer(ref _preparedWeldedVertexRepresentatives, VertexCount);
        EnsureTopologyBuffer(ref _preparedWeldedVertexTriangleOffsets, VertexCount + 1);
        EnsureTopologyBuffer(ref _preparedWeldedVertexTriangles, triangles.Length);
        EnsureTopologyBuffer(ref _surfaceWriteOffsets, Math.Max(VertexCount, _triangleCount) + 1);
        Array.Copy(triangles, _preparedWeldedTriangleVertices, triangles.Length);
        Array.Copy(_topologyRepresentativeIndices, _preparedWeldedVertexRepresentatives, VertexCount);
        int[] offsets = _preparedWeldedVertexTriangleOffsets;
        Array.Clear(offsets, 0, VertexCount + 1);
        for (int i = 0; i < triangles.Length; i++)
            offsets[triangles[i] + 1]++;
        for (int i = 1; i <= VertexCount; i++)
            offsets[i] += offsets[i - 1];
        Array.Copy(offsets, _surfaceWriteOffsets, VertexCount);
        for (int i = 0; i < triangles.Length; i++)
            _preparedWeldedVertexTriangles[_surfaceWriteOffsets[triangles[i]]++] = i / 3;
    }

    private void PrepareSurfaceTopology(Vector3d[] vertices, int[] triangles,
        int[] patchIds, int[] boundaryOffsets, int[] boundaries)
    {
        PrepareManifoldPatchTrust(vertices, triangles, patchIds, boundaryOffsets, boundaries);
        patchIds = _preparedManifoldPatchIds;
        PrepareCoplanarNeighbors(patchIds);
        _surfacePreparationVertices = vertices;
        _surfaceBoundaryEdgeComparer ??= Comparer<SurfaceBoundaryEdge>.Create(CompareSurfaceBoundaryEdges);
        _surfaceOrderComparer ??= Comparer<int>.Create(CompareSurfaceDomains);
        _surfaceBoundaryEdges.FastClear();
        int edgeCapacity = 0;
        for (int i = 0; i < _triangleCount; i++)
            if (patchIds[i] < 0)
                edgeCapacity += 3;
            else if (patchIds[i] == i)
                edgeCapacity += (boundaryOffsets[i + 1] - boundaryOffsets[i]) / 2;
        _surfaceBoundaryEdges.EnsureCapacity(edgeCapacity);
        EnsureTopologyBuffer(ref _preparedCanonicalSurfaceOrdinals, _triangleCount);
        EnsureTopologyBuffer(ref _surfaceKeyOffsets, _triangleCount + 1);
        _surfaceOrder.FastClear();
        _surfaceOrder.EnsureCapacity(_triangleCount);
        EnsureTopologyBuffer(ref _surfacePreviousByVertex, VertexCount);
        for (int surface = 0; surface < _triangleCount; surface++)
        {
            if (patchIds[surface] >= 0 && patchIds[surface] != surface)
                continue;
            _surfaceOrder.Add(surface);
            if (patchIds[surface] < 0)
            {
                int index = surface * 3;
                AddSurfaceBoundaryEdge(surface, triangles[index], triangles[index + 1]);
                AddSurfaceBoundaryEdge(surface, triangles[index + 1], triangles[index + 2]);
                AddSurfaceBoundaryEdge(surface, triangles[index + 2], triangles[index]);
            }
            else
                AddCanonicalPatchBoundary(surface, triangles, boundaryOffsets, boundaries);
        }
        _surfaceBoundaryEdges.SortInPlace(_surfaceBoundaryEdgeComparer);
        Array.Clear(_surfaceKeyOffsets, 0, _triangleCount + 1);
        ReadOnlySpan<SurfaceBoundaryEdge> edges = _surfaceBoundaryEdges.AsReadOnlySpan();
        for (int i = 0; i < edges.Length; i++)
            _surfaceKeyOffsets[edges[i].Surface + 1]++;
        for (int i = 1; i <= _triangleCount; i++)
            _surfaceKeyOffsets[i] += _surfaceKeyOffsets[i - 1];
        _surfaceOrder.SortInPlace(_surfaceOrderComparer);
        int ordinal = 0;
        for (int i = 0; i < _surfaceOrder.Count; i++)
        {
            if (i > 0 && CompareSurfaceDomains(_surfaceOrder[i - 1], _surfaceOrder[i]) != 0)
                ordinal++;
            _preparedCanonicalSurfaceOrdinals[_surfaceOrder[i]] = ordinal;
        }
        for (int i = 0; i < _triangleCount; i++)
            if (patchIds[i] >= 0)
                _preparedCanonicalSurfaceOrdinals[i] = _preparedCanonicalSurfaceOrdinals[patchIds[i]];
    }

    private void PrepareCoplanarNeighbors(int[] patchIds)
    {
        EnsureTopologyBuffer(ref _preparedCoplanarNeighborOffsets, _triangleCount + 1);
        int[] offsets = _preparedCoplanarNeighborOffsets;
        Array.Clear(offsets, 0, _triangleCount + 1);
        ReadOnlySpan<(int First, int Second)> pairs = _surfaceNeighborPairs.AsReadOnlySpan();
        for (int i = 0; i < pairs.Length; i++)
        {
            (int first, int second) = pairs[i];
            if (patchIds[first] < 0)
                continue;
            offsets[first + 1]++;
            offsets[second + 1]++;
        }
        for (int i = 1; i <= _triangleCount; i++)
            offsets[i] += offsets[i - 1];
        EnsureTopologyBuffer(ref _preparedCoplanarNeighbors, offsets[_triangleCount]);
        Array.Copy(offsets, _surfaceWriteOffsets, _triangleCount);
        for (int i = 0; i < pairs.Length; i++)
        {
            (int first, int second) = pairs[i];
            if (patchIds[first] < 0)
                continue;
            _preparedCoplanarNeighbors[_surfaceWriteOffsets[first]++] = second;
            _preparedCoplanarNeighbors[_surfaceWriteOffsets[second]++] = first;
        }
        for (int i = 0; i < _triangleCount; i++)
            Array.Sort(_preparedCoplanarNeighbors, offsets[i], offsets[i + 1] - offsets[i]);
    }

    private void AddCanonicalPatchBoundary(int surface, int[] triangles, int[] offsets, int[] boundaries)
    {
        int index = surface * 3;
        int axis = GetPatchProjectionAxis(_surfacePreparationVertices[triangles[index]],
            _surfacePreparationVertices[triangles[index + 1]], _surfacePreparationVertices[triangles[index + 2]]);
        int cycles = PrepareReducedSurfaceCycles(_surfacePreparationVertices, surface, offsets, boundaries, axis);
        // Keep directed cycles until trust has been established. Only the
        // canonical provenance key discards orientation and sorts segments.
        for (int cycle = 0; cycle < cycles; cycle++)
        {
            int begin = _surfaceCycleOffsets[cycle], end = _surfaceCycleOffsets[cycle + 1];
            for (int corner = begin; corner < end; corner++)
                AddSurfaceBoundaryEdge(surface, _surfaceCycleVertices[corner],
                    _surfaceCycleVertices[corner + 1 == end ? begin : corner + 1]);
        }
    }

    private int PrepareReducedSurfaceCycles(Vector3d[] vertices, int surface,
        int[] offsets, int[] boundaries, int axis)
    {
        int begin = offsets[surface], end = offsets[surface + 1];
        int[] next = _topologyRepresentativeIndices;
        EnsureTopologyBuffer(ref _surfacePreviousByVertex, VertexCount);
        for (int i = begin; i < end; i += 2)
        {
            next[boundaries[i]] = boundaries[i + 1];
            _surfacePreviousByVertex[boundaries[i + 1]] = boundaries[i];
        }
        EnsureTopologyBuffer(ref _surfaceCycleVertices, (end - begin) / 2);
        EnsureTopologyBuffer(ref _surfaceCycleOffsets, (end - begin) / 2 + 1);
        int count = 0, cycles = 0;
        // Degree two supplies closed directed walks, not geometric simplicity.
        // Remove only monotone collinear vertices: retain every backtrack so
        // the embedding certificate can reject it rather than erase overlap.
        for (int i = begin; i < end; i += 2)
        {
            int start = boundaries[i];
            if (next[start] < 0)
                continue;
            _surfaceCycleOffsets[cycles++] = count;
            int current = start;
            do
            {
                int successor = next[current];
                int previous = _surfacePreviousByVertex[current];
                Vector3d a = vertices[previous];
                Vector3d b = vertices[current];
                Vector3d c = vertices[successor];
                if (Vector2d.OrientationSign(ProjectToBoundaryPlane(a, axis),
                        ProjectToBoundaryPlane(b, axis), ProjectToBoundaryPlane(c, axis)) != 0
                    || Math.Sign(CompareSurfacePositions(a, b)) == Math.Sign(CompareSurfacePositions(c, b)))
                    _surfaceCycleVertices[count++] = current;
                next[current] = -1;
                current = successor;
            }
            while (current != start);
        }
        _surfaceCycleOffsets[cycles] = count;
        return cycles;
    }

    private void AddSurfaceBoundaryEdge(int surface, int start, int end)
    {
        if (CompareSurfacePositions(_surfacePreparationVertices[start], _surfacePreparationVertices[end]) > 0)
            (start, end) = (end, start);
        _surfaceBoundaryEdges.Add(new SurfaceBoundaryEdge(surface, start, end));
    }

    private int CompareSurfaceBoundaryEdges(SurfaceBoundaryEdge first, SurfaceBoundaryEdge second)
    {
        int comparison = first.Surface.CompareTo(second.Surface);
        return comparison != 0 ? comparison : CompareSurfaceEdgeGeometry(first, second);
    }

    private int CompareSurfaceEdgeGeometry(SurfaceBoundaryEdge first, SurfaceBoundaryEdge second)
    {
        int comparison = CompareSurfacePositions(_surfacePreparationVertices[first.Start],
            _surfacePreparationVertices[second.Start]);
        return comparison != 0 ? comparison : CompareSurfacePositions(_surfacePreparationVertices[first.End],
            _surfacePreparationVertices[second.End]);
    }

    private int CompareSurfaceDomains(int first, int second)
    {
        int firstStart = _surfaceKeyOffsets[first], secondStart = _surfaceKeyOffsets[second];
        int firstCount = _surfaceKeyOffsets[first + 1] - firstStart;
        int secondCount = _surfaceKeyOffsets[second + 1] - secondStart;
        ReadOnlySpan<SurfaceBoundaryEdge> edges = _surfaceBoundaryEdges.AsReadOnlySpan();
        for (int i = 0; i < Math.Min(firstCount, secondCount); i++)
        {
            int comparison = CompareSurfaceEdgeGeometry(edges[firstStart + i], edges[secondStart + i]);
            if (comparison != 0)
                return comparison;
        }
        return firstCount.CompareTo(secondCount);
    }

    private static int CompareSurfacePositions(Vector3d first, Vector3d second)
    {
        int comparison = first.X.CompareTo(second.X);
        if (comparison != 0)
            return comparison;
        comparison = first.Y.CompareTo(second.Y);
        return comparison != 0 ? comparison : first.Z.CompareTo(second.Z);
    }

    private void PublishPreparedSurfaceTopology()
    {
        (_manifoldPatchIds, _preparedManifoldPatchIds) = (_preparedManifoldPatchIds, _manifoldPatchIds);
        (_canonicalSurfaceOrdinals, _preparedCanonicalSurfaceOrdinals) =
            (_preparedCanonicalSurfaceOrdinals, _canonicalSurfaceOrdinals);
        (_coplanarNeighborOffsets, _preparedCoplanarNeighborOffsets) =
            (_preparedCoplanarNeighborOffsets, _coplanarNeighborOffsets);
        (_coplanarNeighbors, _preparedCoplanarNeighbors) = (_preparedCoplanarNeighbors, _coplanarNeighbors);
        (_weldedTriangleVertices, _preparedWeldedTriangleVertices) =
            (_preparedWeldedTriangleVertices, _weldedTriangleVertices);
        (_weldedVertexRepresentatives, _preparedWeldedVertexRepresentatives) =
            (_preparedWeldedVertexRepresentatives, _weldedVertexRepresentatives);
        (_weldedVertexTriangleOffsets, _preparedWeldedVertexTriangleOffsets) =
            (_preparedWeldedVertexTriangleOffsets, _weldedVertexTriangleOffsets);
        (_weldedVertexTriangles, _preparedWeldedVertexTriangles) =
            (_preparedWeldedVertexTriangles, _weldedVertexTriangles);
    }

    private readonly struct SurfaceBoundaryEdge
    {
        internal readonly int Surface, Start, End;
        internal SurfaceBoundaryEdge(int surface, int start, int end)
        {
            Surface = surface; Start = start; End = end;
        }
    }
}
