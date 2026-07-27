//=======================================================================
// PhysicsMesh.ConvexTopology.cs
//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp;
using FixedMathSharp.Geometry;
using System;

namespace Gravitas.Colliders;

public partial class PhysicsMesh
{
    private static int[] CreateTopologyTriangles(Vector3d[] vertices, int[] triangles)
    {
        var vertexUses = new VertexUse[vertices.Length];
        for (int i = 0; i < vertices.Length; i++)
            vertexUses[i] = new VertexUse(vertices[i], i);

        Array.Sort(vertexUses, CompareVertexUses);
        var representativeIndices = new int[vertices.Length];
        int representativeIndex = vertexUses[0].VertexIndex;
        representativeIndices[representativeIndex] = representativeIndex;
        bool hasExactPositionSeams = false;
        for (int i = 1; i < vertexUses.Length; i++)
        {
            VertexUse current = vertexUses[i];
            if (current.Position != vertexUses[i - 1].Position)
                representativeIndex = current.VertexIndex;
            else
                hasExactPositionSeams = true;

            representativeIndices[current.VertexIndex] = representativeIndex;
        }

        if (!hasExactPositionSeams)
            return triangles;

        var topologyTriangles = new int[triangles.Length];
        for (int i = 0; i < triangles.Length; i++)
            topologyTriangles[i] = representativeIndices[triangles[i]];

        return topologyTriangles;
    }

    private static int CompareVertexUses(VertexUse first, VertexUse second)
    {
        int comparison = first.Position.X.CompareTo(second.Position.X);
        if (comparison != 0)
            return comparison;

        comparison = first.Position.Y.CompareTo(second.Position.Y);
        if (comparison != 0)
            return comparison;

        comparison = first.Position.Z.CompareTo(second.Position.Z);
        if (comparison != 0)
            return comparison;

        return first.VertexIndex.CompareTo(second.VertexIndex);
    }

    private int[] CreateConvexSatEdgeVertexPairs(
        Vector3d[] vertices,
        int[] triangles,
        int triangleCount)
    {
        var triangleUses = new TriangleUse[triangleCount];
        var edgeUses = new EdgeUse[triangleCount * 3];
        int edgeIndex = 0;
        for (int i = 0; i < triangleCount; i++)
        {
            int triangleIndex = i * 3;
            int index0 = triangles[triangleIndex];
            int index1 = triangles[triangleIndex + 1];
            int index2 = triangles[triangleIndex + 2];

            triangleUses[i] = TriangleUse.Create(index0, index1, index2);
            edgeUses[edgeIndex++] = EdgeUse.Create(index0, index1, i);
            edgeUses[edgeIndex++] = EdgeUse.Create(index1, index2, i);
            edgeUses[edgeIndex++] = EdgeUse.Create(index2, index0, i);
        }

        SwiftThrowHelper.ThrowIfArgument(
            ContainsDuplicateTriangle(triangleUses),
            nameof(triangles),
            "Convex mesh triangles must not contain duplicate faces.");

        Array.Sort(edgeUses, CompareEdgeUses);
        var parents = new int[triangleCount];
        for (int i = 0; i < parents.Length; i++)
            parents[i] = i;

        var boundaryNext = new int[vertices.Length];
        var boundaryPrevious = new int[vertices.Length];
        Array.Fill(boundaryNext, -1);
        Array.Fill(boundaryPrevious, -1);

        int boundaryEdgeCount = 0;
        int boundaryStart = -1;
        int dihedralSign = 0;
        for (int start = 0; start < edgeUses.Length;)
        {
            int end = FindEdgeUseGroupEnd(edgeUses, start);
            int useCount = end - start;
            SwiftThrowHelper.ThrowIfArgument(
                useCount > 2,
                nameof(triangles),
                "Convex mesh edges must be manifold and used by at most two triangles.");

            EdgeUse first = edgeUses[start];
            if (useCount == 1)
            {
                RegisterBoundaryEdge(
                    first,
                    boundaryNext,
                    boundaryPrevious,
                    ref boundaryEdgeCount,
                    ref boundaryStart,
                    triangles);
                start = end;
                continue;
            }

            EdgeUse second = edgeUses[start + 1];
            SwiftThrowHelper.ThrowIfArgument(
                first.Direction + second.Direction != 0,
                nameof(triangles),
                "Adjacent convex mesh triangles must use opposite shared-edge winding.");

            Union(parents, first.TriangleIndex, second.TriangleIndex);
            int currentSign = CalculateSharedEdgeSide(vertices, triangles, first, second);
            if (currentSign != 0)
            {
                SwiftThrowHelper.ThrowIfArgument(
                    dihedralSign != 0 && currentSign != dihedralSign,
                    nameof(triangles),
                    "Closed convex mesh triangles must not contain reflex edges.");
                dihedralSign = currentSign;
            }

            start = end;
        }

        int root = Find(parents, 0);
        for (int i = 1; i < parents.Length; i++)
        {
            SwiftThrowHelper.ThrowIfArgument(
                Find(parents, i) != root,
                nameof(triangles),
                "Convex mesh triangles must form one edge-connected surface.");
        }

        IsClosedSurface = boundaryEdgeCount == 0;
        _surfaceClosureValidationResult = IsClosedSurface
            ? MeshVolumeValidationResult.Valid
            : MeshVolumeValidationResult.BoundaryEdge;
        if (IsClosedSurface)
        {
            SwiftThrowHelper.ThrowIfArgument(
                dihedralSign == 0,
                nameof(triangles),
                "Closed convex mesh triangles must enclose nonzero volume.");
            SwiftThrowHelper.ThrowIfArgument(
                !HasSingleVertexLinks(triangles, edgeUses),
                nameof(triangles),
                "Closed convex mesh triangles must be manifold around every vertex.");
        }
        else
        {
            SwiftThrowHelper.ThrowIfArgument(
                dihedralSign != 0,
                nameof(triangles),
                "Open convex mesh triangles must form one coplanar surface.");
            ValidateOpenConvexBoundary(
                vertices,
                triangles,
                triangleCount,
                boundaryNext,
                boundaryEdgeCount,
                boundaryStart);
        }

        return CreateSatEdgeVertexPairs(edgeUses, vertices, triangles);
    }

    private static void RegisterBoundaryEdge(
        EdgeUse edgeUse,
        int[] boundaryNext,
        int[] boundaryPrevious,
        ref int boundaryEdgeCount,
        ref int boundaryStart,
        int[] triangles)
    {
        int start = edgeUse.Direction > 0
            ? edgeUse.StartVertexIndex
            : edgeUse.EndVertexIndex;
        int end = edgeUse.Direction > 0
            ? edgeUse.EndVertexIndex
            : edgeUse.StartVertexIndex;

        SwiftThrowHelper.ThrowIfArgument(
            boundaryNext[start] >= 0 || boundaryPrevious[end] >= 0,
            nameof(triangles),
            "Open convex mesh triangles must form one consistently wound boundary loop.");

        boundaryNext[start] = end;
        boundaryPrevious[end] = start;
        boundaryEdgeCount++;
        if (boundaryStart < 0 || start < boundaryStart)
            boundaryStart = start;
    }

    private static int CalculateSharedEdgeSide(
        Vector3d[] vertices,
        int[] triangles,
        EdgeUse firstUse,
        EdgeUse secondUse)
    {
        int startIndex = firstUse.Direction > 0
            ? firstUse.StartVertexIndex
            : firstUse.EndVertexIndex;
        int endIndex = firstUse.Direction > 0
            ? firstUse.EndVertexIndex
            : firstUse.StartVertexIndex;
        Vector3d start = vertices[startIndex];
        Vector3d end = vertices[endIndex];
        Vector3d firstOpposite = vertices[FindOppositeVertexIndex(
            triangles,
            firstUse.TriangleIndex,
            startIndex,
            endIndex)];
        Vector3d secondOpposite = vertices[FindOppositeVertexIndex(
            triangles,
            secondUse.TriangleIndex,
            startIndex,
            endIndex)];

        return Vector3d.ScalarTripleProductSign(
            end - start,
            firstOpposite - start,
            secondOpposite - start);
    }

    private static void ValidateOpenConvexBoundary(
        Vector3d[] vertices,
        int[] triangles,
        int triangleCount,
        int[] boundaryNext,
        int boundaryEdgeCount,
        int boundaryStart)
    {
        SwiftThrowHelper.ThrowIfArgument(
            boundaryStart < 0,
            nameof(triangles),
            "Open convex mesh triangles must form one boundary loop.");

        int droppedAxis = GetDominantNormalAxis(vertices, triangles);
        var boundaryVertices = new BoundaryVertex[boundaryEdgeCount];
        int current = boundaryStart;
        int visitedEdges = 0;
        do
        {
            int next = boundaryNext[current];

            boundaryVertices[visitedEdges] = new BoundaryVertex(
                ProjectToBoundaryPlane(vertices[current], droppedAxis),
                current);
            current = next;
            visitedEdges++;
            SwiftThrowHelper.ThrowIfArgument(
                visitedEdges > boundaryEdgeCount,
                nameof(triangles),
                "Open convex mesh triangles must form one boundary loop.");
        }
        while (current != boundaryStart);

        SwiftThrowHelper.ThrowIfArgument(
            visitedEdges != boundaryEdgeCount,
            nameof(triangles),
            "Open convex mesh triangles must form one boundary loop.");

        ValidateBoundaryMatchesConvexHull(boundaryVertices, triangles);
        ValidateProjectedTriangleFill(
            vertices,
            triangles,
            triangleCount,
            boundaryVertices,
            droppedAxis);
    }

    private static int GetDominantNormalAxis(Vector3d[] vertices, int[] triangles)
    {
        Vector3d first = vertices[triangles[0]];
        Vector3d second = vertices[triangles[1]];
        Vector3d third = vertices[triangles[2]];
        Vector3d absoluteNormal = Vector3d.Abs(Vector3d.Cross(second - first, third - first));
        if (absoluteNormal.X >= absoluteNormal.Y && absoluteNormal.X >= absoluteNormal.Z)
            return 0;

        return absoluteNormal.Y >= absoluteNormal.Z ? 1 : 2;
    }

    private static Vector2d ProjectToBoundaryPlane(Vector3d vertex, int droppedAxis) =>
        droppedAxis switch
        {
            0 => new Vector2d(vertex.Y, vertex.Z),
            1 => new Vector2d(vertex.X, vertex.Z),
            _ => new Vector2d(vertex.X, vertex.Y),
        };

    private static void ValidateBoundaryMatchesConvexHull(
        BoundaryVertex[] authoredBoundary,
        int[] triangles)
    {
        var sorted = new BoundaryVertex[authoredBoundary.Length];
        Array.Copy(authoredBoundary, sorted, authoredBoundary.Length);
        Array.Sort(sorted, CompareBoundaryVertices);

        for (int i = 1; i < sorted.Length; i++)
        {
            SwiftThrowHelper.ThrowIfArgument(
                sorted[i - 1].Position == sorted[i].Position,
                nameof(triangles),
                "Open convex mesh triangles must form one nondegenerate convex boundary loop.");
        }

        var hull = new BoundaryVertex[sorted.Length * 2];
        int hullCount = 0;
        for (int i = 0; i < sorted.Length; i++)
            AppendConvexHullVertex(hull, ref hullCount, 2, sorted[i]);

        int upperStart = hullCount + 1;
        for (int i = sorted.Length - 2; i >= 0; i--)
            AppendConvexHullVertex(hull, ref hullCount, upperStart, sorted[i]);

        // The upper pass repeats the lexicographically first lower-hull point.
        hullCount--;

        var strictAuthoredBoundary = new BoundaryVertex[authoredBoundary.Length];
        int strictAuthoredCount = 0;
        for (int i = 0; i < authoredBoundary.Length; i++)
        {
            BoundaryVertex previous = authoredBoundary[(i + authoredBoundary.Length - 1) % authoredBoundary.Length];
            BoundaryVertex current = authoredBoundary[i];
            BoundaryVertex next = authoredBoundary[(i + 1) % authoredBoundary.Length];
            if (Vector2d.OrientationSign(previous.Position, current.Position, next.Position) == 0)
                continue;

            strictAuthoredBoundary[strictAuthoredCount++] = current;
        }

        SwiftThrowHelper.ThrowIfArgument(
            hullCount < 3
            || strictAuthoredCount != hullCount
            || !MatchesCyclicHull(strictAuthoredBoundary, strictAuthoredCount, hull),
            nameof(triangles),
            "Open convex mesh triangles must have a convex boundary.");
    }

    private static void ValidateProjectedTriangleFill(
        Vector3d[] vertices,
        int[] triangles,
        int triangleCount,
        BoundaryVertex[] boundary,
        int droppedAxis)
    {
        Fixed64 triangleArea = Fixed64.Zero;
        int surfaceOrientation = 0;
        for (int i = 0; i < triangleCount; i++)
        {
            int index = i * 3;
            Vector2d first = ProjectToBoundaryPlane(vertices[triangles[index]], droppedAxis);
            Vector2d second = ProjectToBoundaryPlane(vertices[triangles[index + 1]], droppedAxis);
            Vector2d third = ProjectToBoundaryPlane(vertices[triangles[index + 2]], droppedAxis);
            int orientation = Vector2d.OrientationSign(first, second, third);
            if (surfaceOrientation == 0)
                surfaceOrientation = orientation;

            SwiftThrowHelper.ThrowIfArgument(
                orientation != surfaceOrientation,
                nameof(triangles),
                "Open convex mesh triangles must fill their convex boundary without holes, overlap, or folds.");
            SwiftThrowHelper.ThrowIfArgument(
                !Fixed64.TryAdd(triangleArea, new FixedTriangle2d(first, second, third).Area, out triangleArea),
                nameof(triangles),
                "Open convex mesh projected area must be representable.");
        }

        Fixed64 boundaryArea = Fixed64.Zero;
        Vector2d anchor = boundary[0].Position;
        for (int i = 1; i < boundary.Length - 1; i++)
        {
            Fixed64 area = new FixedTriangle2d(
                anchor,
                boundary[i].Position,
                boundary[i + 1].Position).Area;
            SwiftThrowHelper.ThrowIfArgument(
                !Fixed64.TryAdd(boundaryArea, area, out boundaryArea),
                nameof(triangles),
                "Open convex mesh projected boundary area must be representable.");
        }

        Fixed64 areaTolerance = Fixed64.Epsilon * (Fixed64)(triangleCount + boundary.Length);
        SwiftThrowHelper.ThrowIfArgument(
            FixedMath.Abs(triangleArea - boundaryArea) > areaTolerance,
            nameof(triangles),
            "Open convex mesh triangles must fill their convex boundary without holes, overlap, or folds.");
    }

    private static void AppendConvexHullVertex(
        BoundaryVertex[] hull,
        ref int hullCount,
        int minimumCount,
        BoundaryVertex candidate)
    {
        while (hullCount >= minimumCount
            && Vector2d.OrientationSign(
                hull[hullCount - 2].Position,
                hull[hullCount - 1].Position,
                candidate.Position) <= 0)
        {
            hullCount--;
        }

        hull[hullCount++] = candidate;
    }

    private static bool MatchesCyclicHull(
        BoundaryVertex[] authoredBoundary,
        int authoredCount,
        BoundaryVertex[] hull)
    {
        int authoredStart = 0;
        for (int i = 1; i < authoredCount; i++)
        {
            if (CompareBoundaryVertices(authoredBoundary[i], authoredBoundary[authoredStart]) < 0)
                authoredStart = i;
        }

        bool forward = true;
        bool reverse = true;
        for (int i = 0; i < authoredCount; i++)
        {
            forward &= authoredBoundary[(authoredStart + i) % authoredCount].VertexIndex
                == hull[i].VertexIndex;
            reverse &= authoredBoundary[(authoredStart - i + authoredCount) % authoredCount].VertexIndex
                == hull[i].VertexIndex;
        }

        return forward || reverse;
    }

    private static int CompareBoundaryVertices(BoundaryVertex first, BoundaryVertex second)
    {
        int comparison = first.Position.X.CompareTo(second.Position.X);
        if (comparison != 0)
            return comparison;

        return first.Position.Y.CompareTo(second.Position.Y);
    }

    private static bool HasSingleVertexLinks(
        int[] triangles,
        EdgeUse[] sortedEdgeUses)
    {
        // Edge validation has already proven that every edge has exactly two
        // oppositely wound uses. Link connectivity is the remaining local
        // manifold condition at each welded vertex.
        var linkParents = new int[triangles.Length];
        var cornerUses = new VertexCornerUse[triangles.Length];
        for (int i = 0; i < triangles.Length; i++)
        {
            linkParents[i] = i;
            cornerUses[i] = new VertexCornerUse(triangles[i], i);
        }

        for (int start = 0; start < sortedEdgeUses.Length;)
        {
            int end = FindEdgeUseGroupEnd(sortedEdgeUses, start);
            EdgeUse first = sortedEdgeUses[start];
            EdgeUse second = sortedEdgeUses[start + 1];
            Union(
                linkParents,
                FindTriangleCornerIndex(triangles, first.TriangleIndex, first.StartVertexIndex),
                FindTriangleCornerIndex(triangles, second.TriangleIndex, first.StartVertexIndex));
            Union(
                linkParents,
                FindTriangleCornerIndex(triangles, first.TriangleIndex, first.EndVertexIndex),
                FindTriangleCornerIndex(triangles, second.TriangleIndex, first.EndVertexIndex));
            start = end;
        }

        Array.Sort(cornerUses, CompareVertexCornerUses);
        for (int start = 0; start < cornerUses.Length;)
        {
            int root = Find(linkParents, cornerUses[start].CornerIndex);
            int vertexIndex = cornerUses[start].VertexIndex;
            int end = start + 1;
            while (end < cornerUses.Length && cornerUses[end].VertexIndex == vertexIndex)
            {
                if (Find(linkParents, cornerUses[end].CornerIndex) != root)
                    return false;
                end++;
            }

            start = end;
        }

        return true;
    }

    private static int FindTriangleCornerIndex(
        int[] triangles,
        int triangleIndex,
        int vertexIndex)
    {
        int cornerIndex = triangleIndex * 3;
        if (triangles[cornerIndex] == vertexIndex)
            return cornerIndex;
        if (triangles[cornerIndex + 1] == vertexIndex)
            return cornerIndex + 1;
        return cornerIndex + 2;
    }

    private static int CompareVertexCornerUses(VertexCornerUse first, VertexCornerUse second)
    {
        int comparison = first.VertexIndex.CompareTo(second.VertexIndex);
        return comparison != 0
            ? comparison
            : first.CornerIndex.CompareTo(second.CornerIndex);
    }

    private static int[] CreateSatEdgeVertexPairs(
        EdgeUse[] edgeUses,
        Vector3d[] vertices,
        int[] triangles)
    {
        int satEdgeCount = CountConvexSatEdges(edgeUses, vertices, triangles);
        var edgeVertexPairs = new int[satEdgeCount * 2];
        int pairIndex = 0;
        for (int start = 0; start < edgeUses.Length;)
        {
            int end = FindEdgeUseGroupEnd(edgeUses, start);
            if (ShouldIncludeConvexSatEdge(edgeUses, start, end, vertices, triangles))
            {
                edgeVertexPairs[pairIndex++] = edgeUses[start].StartVertexIndex;
                edgeVertexPairs[pairIndex++] = edgeUses[start].EndVertexIndex;
            }

            start = end;
        }

        return edgeVertexPairs;
    }

    private static int CountConvexSatEdges(
        EdgeUse[] edgeUses,
        Vector3d[] vertices,
        int[] triangles)
    {
        int count = 0;
        for (int start = 0; start < edgeUses.Length;)
        {
            int end = FindEdgeUseGroupEnd(edgeUses, start);
            if (ShouldIncludeConvexSatEdge(edgeUses, start, end, vertices, triangles))
                count++;

            start = end;
        }

        return count;
    }

    private static int FindEdgeUseGroupEnd(EdgeUse[] edgeUses, int start)
    {
        long key = edgeUses[start].Key;
        int end = start + 1;
        while (end < edgeUses.Length && edgeUses[end].Key == key)
            end++;

        return end;
    }

    private static bool ShouldIncludeConvexSatEdge(
        EdgeUse[] edgeUses,
        int start,
        int end,
        Vector3d[] vertices,
        int[] triangles)
    {
        if (end - start <= 1)
            return true;

        int edgeStart = edgeUses[start].StartVertexIndex;
        int edgeEnd = edgeUses[start].EndVertexIndex;
        Vector3d first = vertices[edgeStart];
        Vector3d edge = vertices[edgeEnd] - first;
        Vector3d firstOpposite = vertices[FindOppositeVertexIndex(
            triangles,
            edgeUses[start].TriangleIndex,
            edgeStart,
            edgeEnd)];
        for (int i = start + 1; i < end; i++)
        {
            Vector3d nextOpposite = vertices[FindOppositeVertexIndex(
                triangles,
                edgeUses[i].TriangleIndex,
                edgeStart,
                edgeEnd)];
            if (Vector3d.ScalarTripleProductSign(
                edge,
                firstOpposite - first,
                nextOpposite - first) != 0)
            {
                return true;
            }
        }

        return false;
    }

    private static int FindOppositeVertexIndex(
        int[] triangles,
        int triangleIndex,
        int edgeStart,
        int edgeEnd)
    {
        int index = triangleIndex * 3;
        int first = triangles[index];
        if (first != edgeStart && first != edgeEnd)
            return first;

        int second = triangles[index + 1];
        return second != edgeStart && second != edgeEnd
            ? second
            : triangles[index + 2];
    }

    private readonly struct VertexUse
    {
        public VertexUse(Vector3d position, int vertexIndex)
        {
            Position = position;
            VertexIndex = vertexIndex;
        }

        public Vector3d Position { get; }

        public int VertexIndex { get; }
    }

    private readonly struct BoundaryVertex
    {
        public BoundaryVertex(Vector2d position, int vertexIndex)
        {
            Position = position;
            VertexIndex = vertexIndex;
        }

        public Vector2d Position { get; }

        public int VertexIndex { get; }
    }

    private readonly struct VertexCornerUse
    {
        public VertexCornerUse(int vertexIndex, int cornerIndex)
        {
            VertexIndex = vertexIndex;
            CornerIndex = cornerIndex;
        }

        public int VertexIndex { get; }

        public int CornerIndex { get; }
    }
}
