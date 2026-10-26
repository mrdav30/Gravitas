//=======================================================================
// PhysicsMesh.CoplanarPatches.cs
//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp;
using FixedMathSharp.Geometry;
using SwiftCollections;
using System;
using System.Collections.Generic;

namespace Gravitas.Colliders;

/// <content>Retains immutable authored seam ownership through positive affine mesh transforms.</content>
public partial class PhysicsMesh
{
    private int[] _coplanarPatchBoundaryOffsets = Array.Empty<int>();
    private int[] _coplanarPatchBoundaryVertexPairs = Array.Empty<int>();
    private int[] _convexCoplanarPatchCornerOffsets = Array.Empty<int>();
    private int[] _convexCoplanarPatchCornerVertexIndices = Array.Empty<int>();
    private int[] _patchIds = Array.Empty<int>();
    // Construction scratch is separate from the immutable topology views.
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
        ChronicleGuard.ThrowIfArrayIndexInvalid(triangleIndex, _triangleCount, nameof(triangleIndex));
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

    private void ReleaseTopologyPreparation()
    {
        // Positive affine transforms preserve the copied topology; construction
        // scratch must not retain mesh-sized capacity for the runtime lifetime.
        _patchIds = _patchParents = _patchCounts = Array.Empty<int>();
        _patchRejected = Array.Empty<bool>();
        _patchWalkedBoundary = Array.Empty<BoundaryVertex>();
        _topologyRepresentativeIndices = _topologyWeldedTriangles = Array.Empty<int>();
        _topologyBoundaryHull = _topologyStrictBoundary = Array.Empty<BoundaryVertex>();
        _surfaceWriteOffsets = _surfaceKeyOffsets = _surfacePreviousByVertex = Array.Empty<int>();
        _surfaceCycleVertices = _surfaceCycleOffsets = Array.Empty<int>();
        _surfacePreparationVertices = Array.Empty<Vector3d>();
        _surfaceBoundaryEdgeComparer = null;
        _surfaceOrderComparer = null;
        _patchEdgeUses.FastClear(); _patchEdgeUses.TrimExcessCapacity();
        _patchBoundaryVertexUses.FastClear(); _patchBoundaryVertexUses.TrimExcessCapacity();
        _topologyVertexUses.FastClear(); _topologyVertexUses.TrimExcessCapacity();
        _topologySortedBoundary.FastClear(); _topologySortedBoundary.TrimExcessCapacity();
        _surfaceNeighborPairs.FastClear(); _surfaceNeighborPairs.TrimExcessCapacity();
        _surfaceBoundaryEdges.FastClear(); _surfaceBoundaryEdges.TrimExcessCapacity();
        _surfaceOrder.FastClear(); _surfaceOrder.TrimExcessCapacity();
    }

    private void PrepareCoplanarPatches(Vector3d[] vertices)
    {
        // Positive affine transforms preserve source incidence and coplanarity;
        // classify once, independently of rounded inspection coordinates.
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

        // Opposite geometric interiors establish a two-sided seam regardless
        // of authored winding. Orient its surviving boundary to the root face
        // so the existing directed-chain coverage proof remains valid.
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
                        edgeUses[boundaryCount++] = CreateOrientedPatchBoundaryEdge(vertices, triangles, edge, patch);
                }
            }
            start = end;
        }

        RejectAmbiguousPatchBoundaryVertices(edgeUses, boundaryCount, rejected);
        EnsureTopologyBuffer(ref _patchIds, _triangleCount);
        EnsureTopologyBuffer(ref _coplanarPatchBoundaryOffsets, _triangleCount + 1);
        int[] ids = _patchIds;
        int[] offsets = _coplanarPatchBoundaryOffsets;
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
        EnsureTopologyBuffer(ref _coplanarPatchBoundaryVertexPairs, offsets[_triangleCount]);
        int[] boundaries = _coplanarPatchBoundaryVertexPairs;
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
        EnsureTopologyBuffer(ref _convexCoplanarPatchCornerOffsets, _triangleCount + 1);
        EnsureTopologyBuffer(ref _convexCoplanarPatchCornerVertexIndices, boundaryOffsets[_triangleCount] / 2);
        int[] offsets = _convexCoplanarPatchCornerOffsets;
        int[] corners = _convexCoplanarPatchCornerVertexIndices;
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

            // The root-oriented coplanar triangle chain is consistent, and each
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
        Vector3d start = vertices[first.StartVertexIndex], end = vertices[first.EndVertexIndex];
        Vector3d firstOpposite = vertices[FindOppositeVertexIndex(triangles, first.TriangleIndex,
            first.StartVertexIndex, first.EndVertexIndex)];
        Vector3d secondOpposite = vertices[FindOppositeVertexIndex(triangles, second.TriangleIndex,
            first.StartVertexIndex, first.EndVertexIndex)];
        // Source points may span the scalar domain even when scale admits a small mesh.
        new FixedTriangle(start, end, firstOpposite).GetExactNormal(out Signed192 nx, out Signed192 ny, out Signed192 nz, out _);
        Signed320 planeSide = WideArithmetic.AddSigned320(WideArithmetic.AddSigned320(
            WideArithmetic.MultiplySigned192(nx, WideArithmetic.Difference(secondOpposite.X, start.X)),
            WideArithmetic.MultiplySigned192(ny, WideArithmetic.Difference(secondOpposite.Y, start.Y))),
            WideArithmetic.MultiplySigned192(nz, WideArithmetic.Difference(secondOpposite.Z, start.Z)));
        if (!planeSide.IsZero)
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

    private static EdgeUse CreateOrientedPatchBoundaryEdge(Vector3d[] vertices, int[] triangles, EdgeUse edge, int patch)
    {
        int root = patch * 3, face = edge.TriangleIndex * 3;
        Vector3d a = vertices[triangles[root]], b = vertices[triangles[root + 1]], c = vertices[triangles[root + 2]];
        int axis = GetPatchProjectionAxis(a, b, c);
        int rootOrientation = Vector2d.OrientationSign(ProjectToBoundaryPlane(a, axis),
            ProjectToBoundaryPlane(b, axis), ProjectToBoundaryPlane(c, axis));
        int faceOrientation = Vector2d.OrientationSign(ProjectToBoundaryPlane(vertices[triangles[face]], axis),
            ProjectToBoundaryPlane(vertices[triangles[face + 1]], axis), ProjectToBoundaryPlane(vertices[triangles[face + 2]], axis));
        int direction = edge.Direction * rootOrientation * faceOrientation;
        return EdgeUse.Create(direction > 0 ? edge.StartVertexIndex : edge.EndVertexIndex,
            direction > 0 ? edge.EndVertexIndex : edge.StartVertexIndex, patch);
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
