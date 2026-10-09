//=======================================================================
// PhysicsMesh.CoplanarPatches.cs
//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp;
using SwiftCollections;
using System;
using System.Collections.Generic;

namespace Gravitas.Colliders;

/// <content>Retains exact coplanar seam ownership with each committed scaled mesh.</content>
public partial class PhysicsMesh
{
    private int[] _coplanarPatchBoundaryOffsets = Array.Empty<int>();
    private int[] _coplanarPatchBoundaryVertexPairs = Array.Empty<int>();
    private int[] _convexCoplanarPatchCornerOffsets = Array.Empty<int>();
    private int[] _convexCoplanarPatchCornerVertexIndices = Array.Empty<int>();
    private int[] _preparedCoplanarPatchIds = Array.Empty<int>();
    private int[] _preparedCoplanarPatchBoundaryOffsets = Array.Empty<int>();
    private int[] _preparedCoplanarPatchBoundaryVertexPairs = Array.Empty<int>();
    private int[] _preparedConvexCoplanarPatchCornerOffsets = Array.Empty<int>();
    private int[] _preparedConvexCoplanarPatchCornerVertexIndices = Array.Empty<int>();
    // Authored counts are fixed. Mutable preparation scratch is never exposed
    // through committed spans and only grows when a larger candidate needs it.
    private readonly SwiftList<EdgeUse> _patchEdgeUses = new();
    private int[] _patchParents = Array.Empty<int>();
    private bool[] _patchRejected = Array.Empty<bool>();
    private int[] _patchCounts = Array.Empty<int>();
    private readonly SwiftList<PatchBoundaryVertexUse> _patchBoundaryVertexUses = new();
    private BoundaryVertex[] _patchWalkedBoundary = Array.Empty<BoundaryVertex>();
    private static readonly IComparer<EdgeUse> PatchEdgeUseComparer = Comparer<EdgeUse>.Create(CompareEdgeUses);
    private static readonly IComparer<PatchBoundaryVertexUse> PatchBoundaryVertexUseComparer =
        Comparer<PatchBoundaryVertexUse>.Create(ComparePatchBoundaryVertexUses);

    // An oriented strict-corner ring, starting at its minimum welded index.
    // Empty means that convex fill is untrusted, not that the patch is empty.
    // The borrower obtains neighboring corner triplets by cyclic indexing.
    internal ReadOnlySpan<int> GetConvexCoplanarPatchCornerVertexIndices(int triangleIndex)
    {
        SwiftThrowHelper.ThrowIfArrayIndexInvalid(triangleIndex, _triangleCount, nameof(triangleIndex));
        int patch = _manifoldPatchIds[triangleIndex];
        return patch < 0 ? ReadOnlySpan<int>.Empty
            : _convexCoplanarPatchCornerVertexIndices.AsSpan(_convexCoplanarPatchCornerOffsets[patch],
                _convexCoplanarPatchCornerOffsets[patch + 1] - _convexCoplanarPatchCornerOffsets[patch]);
    }

    private static void EnsureTopologyBuffer<T>(ref T[] buffer, int count)
    {
        if (buffer.Length < count)
            Array.Resize(ref buffer, count);
    }

    private void PrepareCoplanarPatches(Vector3d[] vertices)
    {
        // Scaling can round distinct positions together or destroy coplanarity.
        // Weld and classify the candidate geometry, never the prior snapshot.
        int[] triangles = CreateTopologyTriangles(vertices, _triangles);
        PrepareWeldedSurfaceIncidence(triangles);
        _surfaceNeighborPairs.FastClear();
        _surfaceNeighborPairs.EnsureCapacity(triangles.Length / 2);
        _patchEdgeUses.FastClear();
        _patchEdgeUses.EnsureCapacity(triangles.Length);
        EnsureTopologyBuffer(ref _patchParents, _triangleCount);
        EnsureTopologyBuffer(ref _patchRejected, _triangleCount);
        EnsureTopologyBuffer(ref _patchCounts, _triangleCount);
        int[] parents = _patchParents;
        bool[] rejected = _patchRejected;
        Array.Clear(rejected, 0, _triangleCount);
        for (int i = 0; i < _triangleCount; i++)
        {
            parents[i] = i;
            int index = i * 3;
            _patchEdgeUses.Add(EdgeUse.Create(triangles[index], triangles[index + 1], i));
            _patchEdgeUses.Add(EdgeUse.Create(triangles[index + 1], triangles[index + 2], i));
            _patchEdgeUses.Add(EdgeUse.Create(triangles[index + 2], triangles[index], i));
        }
        _patchEdgeUses.SortInPlace(PatchEdgeUseComparer);
        Span<EdgeUse> edgeUses = _patchEdgeUses.AsSpan();
        for (int start = 0; start < edgeUses.Length;)
        {
            int end = FindEdgeUseGroupEnd(edgeUses, start);
            if (end - start > 2)
            {
                for (int i = start; i < end; i++)
                    rejected[edgeUses[i].TriangleIndex] = true;
            }
            else if (end - start == 2)
            {
                EdgeUse first = edgeUses[start], second = edgeUses[start + 1];
                int seam = ClassifyCoplanarSeam(vertices, triangles, first, second);
                if (seam > 0)
                {
                    Union(parents, first.TriangleIndex, second.TriangleIndex);
                    _surfaceNeighborPairs.Add((first.TriangleIndex, second.TriangleIndex));
                }
                else if (seam < 0)
                    rejected[first.TriangleIndex] = rejected[second.TriangleIndex] = true;
            }
            start = end;
        }

        int[] counts = _patchCounts;
        Array.Clear(counts, 0, _triangleCount);
        for (int i = 0; i < parents.Length; i++)
        {
            parents[i] = Find(parents, i);
            counts[parents[i]]++;
            if (rejected[i])
                rejected[parents[i]] = true;
        }

        // Compact the remaining edges in place. Every removed edge has two
        // consistently wound coplanar faces on opposite geometric sides.
        int boundaryCount = 0;
        for (int start = 0; start < edgeUses.Length;)
        {
            int end = FindEdgeUseGroupEnd(edgeUses, start);
            if (end - start != 2
                || parents[edgeUses[start].TriangleIndex] != parents[edgeUses[start + 1].TriangleIndex])
            {
                for (int i = start; i < end; i++)
                {
                    EdgeUse edge = edgeUses[i];
                    int patch = parents[edge.TriangleIndex];
                    if (counts[patch] > 1 && !rejected[patch])
                        edgeUses[boundaryCount++] = EdgeUse.Create(
                            edge.Direction > 0 ? edge.StartVertexIndex : edge.EndVertexIndex,
                            edge.Direction > 0 ? edge.EndVertexIndex : edge.StartVertexIndex, patch);
                }
            }
            start = end;
        }

        RejectAmbiguousPatchBoundaryVertices(edgeUses, boundaryCount, rejected);
        EnsureTopologyBuffer(ref _preparedCoplanarPatchIds, _triangleCount);
        EnsureTopologyBuffer(ref _preparedCoplanarPatchBoundaryOffsets, _triangleCount + 1);
        int[] ids = _preparedCoplanarPatchIds;
        int[] offsets = _preparedCoplanarPatchBoundaryOffsets;
        Array.Clear(offsets, 0, _triangleCount + 1);
        for (int i = 0; i < ids.Length; i++)
            ids[i] = counts[parents[i]] > 1 && !rejected[parents[i]] ? parents[i] : -1;
        for (int i = 0; i < boundaryCount; i++)
        {
            int patch = edgeUses[i].TriangleIndex;
            if (!rejected[patch])
                offsets[patch + 1] += 2;
        }
        for (int i = 1; i < offsets.Length; i++)
            offsets[i] += offsets[i - 1];
        EnsureTopologyBuffer(ref _preparedCoplanarPatchBoundaryVertexPairs, offsets[_triangleCount]);
        int[] boundaries = _preparedCoplanarPatchBoundaryVertexPairs;
        Array.Copy(offsets, counts, counts.Length);
        for (int i = 0; i < boundaryCount; i++)
        {
            EdgeUse edge = edgeUses[i];
            if (rejected[edge.TriangleIndex])
                continue;
            int index = counts[edge.TriangleIndex];
            boundaries[index] = edge.Direction > 0 ? edge.StartVertexIndex : edge.EndVertexIndex;
            boundaries[index + 1] = edge.Direction > 0 ? edge.EndVertexIndex : edge.StartVertexIndex;
            counts[edge.TriangleIndex] += 2;
        }
        PrepareConvexCoplanarPatchCorners(vertices, triangles, ids, offsets, boundaries);
        PrepareSurfaceTopology(vertices, triangles, ids, offsets, boundaries);
    }

    private void PrepareConvexCoplanarPatchCorners(Vector3d[] vertices, int[] triangles,
        int[] ids, int[] boundaryOffsets, int[] boundaries)
    {
        EnsureTopologyBuffer(ref _preparedConvexCoplanarPatchCornerOffsets, _triangleCount + 1);
        EnsureTopologyBuffer(ref _preparedConvexCoplanarPatchCornerVertexIndices, boundaryOffsets[_triangleCount] / 2);
        int[] offsets = _preparedConvexCoplanarPatchCornerOffsets;
        int[] corners = _preparedConvexCoplanarPatchCornerVertexIndices;
        // Welding has finished consuming the representative map. Reuse its
        // vertex-sized storage for each patch's outgoing successor links.
        int[] nextByVertex = _topologyRepresentativeIndices;
        int cornerCount = 0;
        for (int patch = 0; patch < _triangleCount; patch++)
        {
            offsets[patch] = cornerCount;
            if (ids[patch] != patch)
                continue;
            int begin = boundaryOffsets[patch], end = boundaryOffsets[patch + 1];
            int boundaryStart = int.MaxValue;
            for (int i = begin; i < end; i += 2)
            {
                int vertex = boundaries[i];
                nextByVertex[vertex] = boundaries[i + 1];
                boundaryStart = Math.Min(boundaryStart, vertex);
            }

            // The existing degree-two check and opposite-wound cancellation
            // give one incoming/outgoing edge per boundary vertex. Populate
            // only this patch's successors: no per-patch whole-vertex clearing.
            int triangle = patch * 3;
            int axis = GetPatchProjectionAxis(vertices[triangles[triangle]],
                vertices[triangles[triangle + 1]], vertices[triangles[triangle + 2]]);
            int edgeCount = (end - begin) / 2, visited = 0, current = boundaryStart;
            EnsureTopologyBuffer(ref _patchWalkedBoundary, edgeCount);
            BoundaryVertex[] boundary = _patchWalkedBoundary;
            do
            {
                boundary[visited++] = new BoundaryVertex(ProjectToBoundaryPlane(vertices[current], axis), current);
                current = nextByVertex[current];
            }
            while (current != boundaryStart);
            if (visited != edgeCount
                || !TryGetStrictConvexBoundary(boundary, edgeCount, out BoundaryVertex[] strict, out int count))
                continue;

            // Every triangle is coplanar and consistently oriented, and each
            // canceled seam has exactly two opposite-side interiors. A single
            // boundary matching its convex hull has winding multiplicity one
            // inside and zero outside. It therefore proves exact filled
            // coverage; rounded projected-area tolerances are not a certificate.
            int start = 0;
            for (int i = 1; i < count; i++)
                if (strict[i].VertexIndex < strict[start].VertexIndex)
                    start = i;
            for (int i = 0; i < count; i++)
                corners[cornerCount++] = strict[(start + i) % count].VertexIndex;
        }
        offsets[^1] = cornerCount;
    }

    private static int ClassifyCoplanarSeam(Vector3d[] vertices, int[] triangles, EdgeUse first, EdgeUse second)
    {
        if (first.Direction + second.Direction != 0)
            return -1;
        Vector3d start = vertices[first.StartVertexIndex], end = vertices[first.EndVertexIndex];
        Vector3d firstOpposite = vertices[FindOppositeVertexIndex(triangles, first.TriangleIndex,
            first.StartVertexIndex, first.EndVertexIndex)];
        Vector3d secondOpposite = vertices[FindOppositeVertexIndex(triangles, second.TriangleIndex,
            first.StartVertexIndex, first.EndVertexIndex)];
        if (!Vector3d.TrySubtract(end, start, out Vector3d edge)
            || !Vector3d.TrySubtract(firstOpposite, start, out Vector3d firstOffset)
            || !Vector3d.TrySubtract(secondOpposite, start, out Vector3d secondOffset))
            return -1;
        if (Vector3d.ScalarTripleProductSign(edge, firstOffset, secondOffset) != 0)
            return 0;

        int droppedAxis = GetPatchProjectionAxis(start, end, firstOpposite);
        Vector2d projectedStart = ProjectToBoundaryPlane(start, droppedAxis);
        Vector2d projectedEnd = ProjectToBoundaryPlane(end, droppedAxis);
        int firstSide = Vector2d.OrientationSign(projectedStart, projectedEnd,
            ProjectToBoundaryPlane(firstOpposite, droppedAxis));
        int secondSide = Vector2d.OrientationSign(projectedStart, projectedEnd,
            ProjectToBoundaryPlane(secondOpposite, droppedAxis));
        return firstSide == -secondSide ? 1 : -1;
    }

    private static int GetPatchProjectionAxis(Vector3d first, Vector3d second, Vector3d third)
    {
        if (Vector2d.OrientationSign(new Vector2d(first.X, first.Y),
                new Vector2d(second.X, second.Y), new Vector2d(third.X, third.Y)) != 0)
            return 2;
        return Vector2d.OrientationSign(new Vector2d(first.X, first.Z),
            new Vector2d(second.X, second.Z), new Vector2d(third.X, third.Z)) != 0 ? 1 : 0;
    }

    private void RejectAmbiguousPatchBoundaryVertices(ReadOnlySpan<EdgeUse> edges, int count, bool[] rejected)
    {
        int vertexCount = count * 2;
        _patchBoundaryVertexUses.FastClear();
        _patchBoundaryVertexUses.EnsureCapacity(vertexCount);
        for (int i = 0; i < count; i++)
        {
            EdgeUse edge = edges[i];
            _patchBoundaryVertexUses.Add(new PatchBoundaryVertexUse(edge.TriangleIndex, edge.StartVertexIndex));
            _patchBoundaryVertexUses.Add(new PatchBoundaryVertexUse(edge.TriangleIndex, edge.EndVertexIndex));
        }
        _patchBoundaryVertexUses.SortInPlace(PatchBoundaryVertexUseComparer);
        ReadOnlySpan<PatchBoundaryVertexUse> vertices = _patchBoundaryVertexUses.AsReadOnlySpan();
        for (int start = 0; start < vertexCount;)
        {
            PatchBoundaryVertexUse first = vertices[start];
            int end = start + 1;
            while (end < vertexCount && vertices[end].Patch == first.Patch && vertices[end].Vertex == first.Vertex)
                end++;
            // Canceling opposite-wound seams preserves balanced incoming and
            // outgoing uses. Degree two therefore establishes disjoint oriented
            // cycles, including holes. Pinched vertices decline the patch.
            if (end - start != 2)
                rejected[first.Patch] = true;
            start = end;
        }
    }

    private static int ComparePatchBoundaryVertexUses(PatchBoundaryVertexUse first, PatchBoundaryVertexUse second)
    {
        int comparison = first.Patch.CompareTo(second.Patch);
        return comparison != 0 ? comparison : first.Vertex.CompareTo(second.Vertex);
    }

    private readonly struct PatchBoundaryVertexUse
    {
        internal readonly int Patch, Vertex;
        internal PatchBoundaryVertexUse(int patch, int vertex)
        {
            Patch = patch; Vertex = vertex;
        }
    }
}
