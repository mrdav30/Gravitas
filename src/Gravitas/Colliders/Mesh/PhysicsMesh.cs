//=======================================================================
// PhysicsMesh.cs
//=======================================================================
// MIT License, Copyright (c) 2026–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp;
using FixedMathSharp.Bounds;
using SwiftCollections;
using SwiftCollections.Query;
using System;
using System.Collections.Generic;

namespace Gravitas.Colliders
{
    public partial class PhysicsMesh
    {
        private const int SupportTreeVertexThreshold = 32;
        private const int SupportTreeLeafVertexCount = 8;
        private const int SupportTreeStackCapacity = 64;

        /// <summary>
        /// Maximum accepted vertex count for deterministic runtime mesh construction.
        /// </summary>
        public const int MaxVertexCount = 65535;

        /// <summary>
        /// Maximum accepted triangle count for deterministic runtime mesh construction.
        /// </summary>
        public const int MaxTriangleCount = 131072;

        private static readonly Fixed64 TetrahedronVolumeDivisor = (Fixed64)6;
        private static readonly Fixed64 TetrahedronCentroidDivisor = (Fixed64)4;
        private static readonly Fixed64 SecondMomentIntegralDivisor = (Fixed64)10;
        private static readonly Fixed64 ProductMomentIntegralDivisor = (Fixed64)20;

        private readonly Vector3d[] _localVertices;

        private readonly Vector3d[] _worldVertices;
        private bool _worldVerticesValid;
        private readonly int[]? _supportVertexIndices;
        private readonly SupportTreeNode[]? _supportTreeNodes;
        private int _supportTreeNodeCount;

        /// <summary>
        /// Holds all vertices transformed to world space. Prefer point-specific helpers on hot paths.
        /// </summary>
        public ReadOnlySpan<Vector3d> Vertices
        {
            get
            {
                EnsureWorldVertices();
                return _worldVertices;
            }
        }

        /// <summary>
        /// Holds the source vertices in local mesh space.
        /// </summary>
        internal ReadOnlySpan<Vector3d> LocalVertices => _localVertices;

        /// <summary>
        /// Number of vertices in the immutable local mesh topology.
        /// </summary>
        public int VertexCount => _localVertices.Length;

        private readonly int[] _triangles;
        /// <summary>
        /// Holds all the triangles that make up the mesh in the form of indices to the vertices array.
        /// example: Triangles[0-3] = 4,2,3 means that the first triangle of the mesh is made up of the vertices _vertices[4], _vertices[2], _vertices[3]
        /// </summary>
        public ReadOnlySpan<int> Triangles => _triangles;

        private readonly int _triangleCount;
        public int TriangleCount => _triangleCount;

        private readonly int[] _convexSatEdgeVertexPairs;
        internal ReadOnlySpan<int> ConvexSatEdgeVertexPairs => _convexSatEdgeVertexPairs;

        public MeshColliderMode Mode { get; }

        private bool _faceNormalsValid;
        private readonly Vector3d[] _faceNormals;
        /// <summary>
        /// Holds triangle normals in local mesh space.
        /// </summary>
        public ReadOnlySpan<Vector3d> FaceNormals
        {
            get
            {
                if (!_faceNormalsValid)
                    CalculateFaceNormals();
                return _faceNormals;
            }
        }

        /// <summary>
        /// Gets the current triangle surface area after local scale.
        /// </summary>
        public Fixed64 TotalArea => _scaledTotalArea;

        private bool _closedVolumeMassPropertiesEvaluated;
        private MeshMassProperties _closedVolumeMassProperties;
        private MeshVolumeValidationResult _closedVolumeValidationResult;

        private bool _triangleBVHValid;
        private int _triangleBvhBuildCount;
        private readonly SwiftFixedBVH<int> _triangleBVH;

        /// <summary>
        /// Triangle acceleration structure in local mesh space.
        /// </summary>
        internal SwiftFixedBVH<int> TriangleBVH => !_triangleBVHValid
            ? UpdateTriangleBVH()
            : _triangleBVH;

        public int TriangleBvhBuildCount => _triangleBvhBuildCount;

        private FixedQuaternion _rotation = FixedQuaternion.Identity;

        private Fixed4x4 _transformationMatrix;
        public Fixed4x4 TransformationMatrix
        {
            get => _transformationMatrix;
            private set
            {
                _transformationMatrix = value;
                _inverseMatrixValid = false;
                _worldVerticesValid = false;
            }
        }

        private bool _inverseMatrixValid;
        private Fixed4x4 _inverseTransformationMatrix;
        public Fixed4x4 InverseTransformationMatrix
        {
            get
            {
                if (_inverseMatrixValid)
                    return _inverseTransformationMatrix;

                Fixed4x4 inverse = BuildInverseTransformationMatrix();
                _inverseTransformationMatrix = inverse;
                _inverseMatrixValid = true;
                return inverse;
            }
        }

        private FixedBoundBox _bounds;
        private bool _boundsInitialized;
        public FixedBoundBox Bounds => _bounds;

        private readonly FixedBoundBox _localBounds;

        /// <summary>
        /// Axis-aligned bounds of the source vertices in local mesh space.
        /// </summary>
        public FixedBoundBox LocalBounds => _localBounds;

        public PhysicsMesh(Vector3d[] vertices, int[] triangles, Vector3d position, FixedQuaternion rotation)
            : this(vertices, triangles, position, rotation, MeshColliderMode.Convex) { }

        public PhysicsMesh(
            Vector3d[] vertices,
            int[] triangles,
            Vector3d position,
            FixedQuaternion rotation,
            MeshColliderMode mode)
        {
            SwiftThrowHelper.ThrowIfArgument(
                mode != MeshColliderMode.Convex && mode != MeshColliderMode.Concave,
                nameof(mode),
                "Unsupported mesh collider mode.");

            ValidateInput(vertices, triangles);

            Mode = mode;
            _localVertices = new Vector3d[vertices.Length];
            Array.Copy(vertices, _localVertices, vertices.Length);
            _worldVertices = new Vector3d[vertices.Length];
            _triangles = new int[triangles.Length];
            Array.Copy(triangles, _triangles, triangles.Length);
            _triangleCount = triangles.Length / 3; // 3 vertices per triangle
            _triangleBVH = new SwiftFixedBVH<int>(2 * TriangleCount - 1);
            _faceNormals = new Vector3d[TriangleCount];
            _convexSatEdgeVertexPairs = Mode == MeshColliderMode.Convex
                ? CreateConvexSatEdgeVertexPairs(_localVertices, _triangles, _triangleCount)
                : Array.Empty<int>();

            _scaledFaceAreas = new Fixed64[TriangleCount];
            _surfaceValidationFaceAreas = new Fixed64[TriangleCount];
            _scaledFaceNormals = new Vector3d[TriangleCount];

            _localBounds = CalculateBounds(_localVertices);
            if (Mode == MeshColliderMode.Convex && _localVertices.Length > SupportTreeVertexThreshold)
            {
                _supportVertexIndices = CreateSupportVertexIndices(_localVertices.Length);
                _supportTreeNodes = new SupportTreeNode[(2 * _localVertices.Length) - 1];
                _supportTreeNodeCount = BuildSupportTreeNode(0, _localVertices.Length);
            }

            UpdateTransformation(position, rotation, Vector3d.One);
            UpdateBounds();
        }

        public void UpdatePosition(Vector3d position, FixedQuaternion rotation)
        {
            UpdateTransformation(position, rotation, _scale);
            UpdateBounds();
        }

        /// <summary>
        /// Updates the mesh origin, normalized rigid rotation, and strictly positive mesh-local scale.
        /// </summary>
        public void UpdateTransform(Vector3d position, FixedQuaternion rotation, Vector3d scale)
        {
            UpdateTransformation(position, rotation, scale);
            UpdateBounds();
        }

        private void EnsureWorldVertices()
        {
            if (_worldVerticesValid)
                return;

            for (int i = 0; i < _localVertices.Length; i++)
                _worldVertices[i] = TransformLocalPoint(_localVertices[i]);

            _worldVerticesValid = true;
        }

        private static void ValidateInput(Vector3d[] vertices, int[] triangles)
        {
            SwiftThrowHelper.ThrowIfNull(vertices, nameof(vertices));
            SwiftThrowHelper.ThrowIfNull(triangles, nameof(triangles));
            SwiftThrowHelper.ThrowIfArgument(vertices.Length < 3, nameof(vertices), "Mesh must contain at least three vertices.");
            SwiftThrowHelper.ThrowIfArgument(triangles.Length == 0 || triangles.Length % 3 != 0, nameof(triangles), "Triangle index count must be a positive multiple of three.");
            SwiftThrowHelper.ThrowIfArgumentOutOfRange(vertices.Length > MaxVertexCount, vertices.Length, nameof(vertices), "Mesh exceeds the deterministic vertex limit.");

            int triangleCount = triangles.Length / 3;
            SwiftThrowHelper.ThrowIfArgumentOutOfRange(triangleCount > MaxTriangleCount, triangleCount, nameof(triangles), "Mesh exceeds the deterministic triangle limit.");

            for (int i = 0; i < triangleCount; i++)
            {
                int index0 = triangles[i * 3];
                int index1 = triangles[i * 3 + 1];
                int index2 = triangles[i * 3 + 2];

                SwiftThrowHelper.ThrowIfArgumentOutOfRange((uint)index0 >= (uint)vertices.Length, vertices.Length, nameof(triangles));
                SwiftThrowHelper.ThrowIfArgumentOutOfRange((uint)index1 >= (uint)vertices.Length, vertices.Length, nameof(triangles));
                SwiftThrowHelper.ThrowIfArgumentOutOfRange((uint)index2 >= (uint)vertices.Length, vertices.Length, nameof(triangles));

                SwiftThrowHelper.ThrowIfArgument(
                    index0 == index1 || index1 == index2 || index2 == index0,
                    nameof(triangles),
                    "Triangle indices must be unique within each triangle.");

                Fixed64 area = CalculateTriangleArea(vertices[index0], vertices[index1], vertices[index0], vertices[index2]);
                SwiftThrowHelper.ThrowIfArgument(area <= Fixed64.Epsilon, nameof(triangles), "Degenerate triangles are not supported.");
            }
        }

        private static Fixed64 CalculateTriangleArea(
            Vector3d startEdgeA,
            Vector3d endEdgeA,
            Vector3d startEdgeB,
            Vector3d endEdgeB) => Vector3d.Cross(endEdgeA - startEdgeA, endEdgeB - startEdgeB).Magnitude * Fixed64.Half;

        private static int[] CreateConvexSatEdgeVertexPairs(
            Vector3d[] vertices,
            int[] triangles,
            int triangleCount)
        {
            var edgeUses = new EdgeUse[triangleCount * 3];
            int edgeIndex = 0;
            for (int i = 0; i < triangleCount; i++)
            {
                int triangleIndex = i * 3;
                int index0 = triangles[triangleIndex];
                int index1 = triangles[triangleIndex + 1];
                int index2 = triangles[triangleIndex + 2];

                edgeUses[edgeIndex++] = EdgeUse.Create(index0, index1, i);
                edgeUses[edgeIndex++] = EdgeUse.Create(index1, index2, i);
                edgeUses[edgeIndex++] = EdgeUse.Create(index2, index0, i);
            }

            Array.Sort(edgeUses, CompareEdgeUses);
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
            Vector3d firstFaceCross = Vector3d.Cross(edge, firstOpposite - first);
            for (int i = start + 1; i < end; i++)
            {
                Vector3d nextOpposite = vertices[FindOppositeVertexIndex(
                    triangles,
                    edgeUses[i].TriangleIndex,
                    edgeStart,
                    edgeEnd)];
                if (Vector3d.Dot(firstFaceCross, nextOpposite - first) != Fixed64.Zero)
                    return true;
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

        private static Vector3d CalculateLocalTriangleNormal(
            Vector3d[] vertices,
            int[] triangles,
            int triangleIndex)
        {
            int index = triangleIndex * 3;
            Vector3d first = vertices[triangles[index]];
            Vector3d second = vertices[triangles[index + 1]];
            Vector3d third = vertices[triangles[index + 2]];
            return Vector3d.Cross(second - first, third - first).Normalized;
        }

        private Vector3d[] CalculateFaceNormals()
        {
            for (int i = 0; i < _triangleCount; i++)
                _faceNormals[i] = CalculateLocalTriangleNormal(_localVertices, _triangles, i);

            _faceNormalsValid = true;
            return _faceNormals;
        }

        private SwiftFixedBVH<int> UpdateTriangleBVH()
        {
            _triangleBVH.Clear();
            for (int i = 0; i < _triangleCount; i++)
            {
                int index0 = _triangles[i * 3];
                int index1 = _triangles[i * 3 + 1];
                int index2 = _triangles[i * 3 + 2];
                Vector3d min = Vector3d.Min(Vector3d.Min(_localVertices[index0], _localVertices[index1]), _localVertices[index2]);
                Vector3d max = Vector3d.Max(Vector3d.Max(_localVertices[index0], _localVertices[index1]), _localVertices[index2]);
                _triangleBVH.Insert(i, new FixedBoundVolume(min, max));
            }

            _triangleBvhBuildCount++;
            _triangleBVHValid = true;
            return _triangleBVH;
        }

        private void UpdateBounds()
        {
            FixedBoundVolume volume = TransformBounds(_localBounds.Min, _localBounds.Max, TransformationMatrix);
            if (!_boundsInitialized)
            {
                _bounds = FixedBoundBox.FromMinMax(volume.Min, volume.Max);
                _boundsInitialized = true;
                return;
            }

            _bounds.SetMinMax(volume.Min, volume.Max);
        }

        private static FixedBoundBox CalculateBounds(Vector3d[] vertices)
        {
            Vector3d min = vertices[0];
            Vector3d max = vertices[0];
            for (int i = 1; i < vertices.Length; i++)
            {
                min = Vector3d.Min(min, vertices[i]);
                max = Vector3d.Max(max, vertices[i]);
            }

            return FixedBoundBox.FromMinMax(min, max);
        }

        private static FixedBoundVolume TransformBounds(Vector3d min, Vector3d max, Fixed4x4 transform)
        {
            Vector3d first = transform * min;
            Vector3d transformedMin = first;
            Vector3d transformedMax = first;

            IncludeTransformedPoint(transform * new Vector3d(max.X, min.Y, min.Z), ref transformedMin, ref transformedMax);
            IncludeTransformedPoint(transform * new Vector3d(min.X, max.Y, min.Z), ref transformedMin, ref transformedMax);
            IncludeTransformedPoint(transform * new Vector3d(max.X, max.Y, min.Z), ref transformedMin, ref transformedMax);
            IncludeTransformedPoint(transform * new Vector3d(min.X, min.Y, max.Z), ref transformedMin, ref transformedMax);
            IncludeTransformedPoint(transform * new Vector3d(max.X, min.Y, max.Z), ref transformedMin, ref transformedMax);
            IncludeTransformedPoint(transform * new Vector3d(min.X, max.Y, max.Z), ref transformedMin, ref transformedMax);
            IncludeTransformedPoint(transform * max, ref transformedMin, ref transformedMax);

            return new FixedBoundVolume(transformedMin, transformedMax);
        }

        private static void IncludeTransformedPoint(
            Vector3d transformed,
            ref Vector3d min,
            ref Vector3d max)
        {
            min = Vector3d.Min(min, transformed);
            max = Vector3d.Max(max, transformed);
        }

        /// <summary>
        /// Calculates solid closed-volume inertia for the supplied mass.
        /// This is a geometry/topology API; callers apply body mobility gates before requesting inertia.
        /// </summary>
        private static int CompareEdgeUses(EdgeUse first, EdgeUse second) =>
            first.Key < second.Key
                ? -1
                : first.Key > second.Key
                    ? 1
                    : 0;

        public Fixed64 GetFrontalArea(Vector3d direction)
        {
            Fixed64 directionMagnitude = direction.Magnitude;
            if (directionMagnitude <= Fixed64.Epsilon)
                return Fixed64.Zero;

            Vector3d normalizedDirection = direction / directionMagnitude;
            Fixed64 totalArea = Fixed64.Zero;
            for (int i = 0; i < _triangleCount; i++)
            {
                Fixed64 projection = Vector3d.Dot(GetFaceNormalWorld(i), normalizedDirection);
                if (projection > Fixed64.Zero)
                    totalArea += _scaledFaceAreas[i] * FixedMath.Min(projection, Fixed64.One);
            }

            return totalArea;
        }

        public void GetTriangleVertices(int index, out Vector3d first, out Vector3d second, out Vector3d third)
        {
            GetLocalTriangleVertices(index, out Vector3d localFirst, out Vector3d localSecond, out Vector3d localThird);
            first = TransformLocalPoint(localFirst);
            second = TransformLocalPoint(localSecond);
            third = TransformLocalPoint(localThird);
        }

        public void GetLocalTriangleVertices(int index, out Vector3d first, out Vector3d second, out Vector3d third)
        {
            SwiftThrowHelper.ThrowIfArrayIndexInvalid(index, _triangleCount, nameof(index));

            int triangleIndex = index * 3;
            first = _localVertices[_triangles[triangleIndex]];
            second = _localVertices[_triangles[triangleIndex + 1]];
            third = _localVertices[_triangles[triangleIndex + 2]];
        }

        public Vector3d GetVertexWorld(int index)
        {
            SwiftThrowHelper.ThrowIfArrayIndexInvalid(index, _localVertices.Length, nameof(index));
            return TransformLocalPoint(_localVertices[index]);
        }

        /// <summary>
        /// Finds the world-space vertex with the greatest projection onto
        /// <paramref name="direction"/>, preserving source vertex order for ties.
        /// </summary>
        public Vector3d GetSupportVertexWorld(Vector3d direction)
        {
            if (_supportTreeNodes != null && _supportVertexIndices != null)
                return GetAcceleratedSupportVertexWorld(direction);

            EnsureWorldVertices();

            Vector3d best = _worldVertices[0];
            Fixed64 bestProjection = Vector3d.Dot(best, direction);
            for (int i = 1; i < _worldVertices.Length; i++)
            {
                Vector3d vertex = _worldVertices[i];
                Fixed64 projection = Vector3d.Dot(vertex, direction);
                if (projection <= bestProjection)
                    continue;

                bestProjection = projection;
                best = vertex;
            }

            return best;
        }

        private Vector3d GetAcceleratedSupportVertexWorld(Vector3d direction)
        {
            Vector3d localDirection = ConvertWorldDirectionToLocal(direction);
            int supportIndex = FindSupportVertexIndex(localDirection);
            return TransformLocalPoint(_localVertices[supportIndex]);
        }

        private int FindSupportVertexIndex(Vector3d localDirection)
        {
            int bestIndex = 0;
            Fixed64 bestProjection = Vector3d.Dot(_localVertices[0], localDirection);
            Span<int> stack = stackalloc int[SupportTreeStackCapacity];
            int stackCount = 0;
            stack[stackCount++] = 0;

            while (stackCount > 0)
            {
                int nodeIndex = stack[--stackCount];
                SupportTreeNode node = _supportTreeNodes![nodeIndex];
                Fixed64 upperProjection = GetBoundsMaxProjection(node.Min, node.Max, localDirection);
                int upperComparison = upperProjection.CompareTo(bestProjection);
                if (upperComparison < 0 || (upperComparison == 0 && node.MinVertexIndex >= bestIndex))
                    continue;

                if (node.IsLeaf)
                {
                    SearchSupportLeaf(node, localDirection, ref bestIndex, ref bestProjection);
                    continue;
                }

                SupportTreeNode left = _supportTreeNodes[node.Left];
                SupportTreeNode right = _supportTreeNodes[node.Right];
                Fixed64 leftProjection = GetBoundsMaxProjection(left.Min, left.Max, localDirection);
                Fixed64 rightProjection = GetBoundsMaxProjection(right.Min, right.Max, localDirection);

                if (ComesBeforeSupportNode(left, leftProjection, right, rightProjection))
                {
                    PushSupportNode(right, rightProjection, bestIndex, bestProjection, stack, ref stackCount);
                    PushSupportNode(left, leftProjection, bestIndex, bestProjection, stack, ref stackCount);
                    continue;
                }

                PushSupportNode(left, leftProjection, bestIndex, bestProjection, stack, ref stackCount);
                PushSupportNode(right, rightProjection, bestIndex, bestProjection, stack, ref stackCount);
            }

            return bestIndex;
        }

        private void SearchSupportLeaf(
            SupportTreeNode node,
            Vector3d localDirection,
            ref int bestIndex,
            ref Fixed64 bestProjection)
        {
            for (int i = 0; i < node.Count; i++)
            {
                int vertexIndex = _supportVertexIndices![node.Start + i];
                Vector3d vertex = _localVertices[vertexIndex];
                Fixed64 projection = Vector3d.Dot(vertex, localDirection);
                int projectionComparison = projection.CompareTo(bestProjection);
                if (projectionComparison < 0 || (projectionComparison == 0 && vertexIndex >= bestIndex))
                    continue;

                bestProjection = projection;
                bestIndex = vertexIndex;
            }
        }

        private static void PushSupportNode(
            SupportTreeNode node,
            Fixed64 upperProjection,
            int bestIndex,
            Fixed64 bestProjection,
            Span<int> stack,
            ref int stackCount)
        {
            int upperComparison = upperProjection.CompareTo(bestProjection);
            if (upperComparison < 0 || (upperComparison == 0 && node.MinVertexIndex >= bestIndex))
                return;

            stack[stackCount++] = node.Index;
        }

        private static bool ComesBeforeSupportNode(
            SupportTreeNode left,
            Fixed64 leftProjection,
            SupportTreeNode right,
            Fixed64 rightProjection)
        {
            int projectionComparison = leftProjection.CompareTo(rightProjection);
            if (projectionComparison != 0)
                return projectionComparison > 0;

            return left.MinVertexIndex < right.MinVertexIndex;
        }

        private int BuildSupportTreeNode(int start, int count)
        {
            int nodeIndex = _supportTreeNodeCount;
            _supportTreeNodeCount++;

            CalculateSupportRangeBounds(start, count, out Vector3d min, out Vector3d max, out int minVertexIndex);
            if (count <= SupportTreeLeafVertexCount)
            {
                _supportTreeNodes![nodeIndex] = SupportTreeNode.CreateLeaf(
                    nodeIndex,
                    min,
                    max,
                    start,
                    count,
                    minVertexIndex);
                return nodeIndex;
            }

            int axis = GetDominantAxis(max - min);
            Array.Sort(
                _supportVertexIndices!,
                start,
                count,
                new SupportVertexIndexComparer(_localVertices, axis));

            int leftCount = count / 2;
            int rightCount = count - leftCount;
            int leftIndex = BuildSupportTreeNode(start, leftCount);
            int rightIndex = BuildSupportTreeNode(start + leftCount, rightCount);
            _supportTreeNodes![nodeIndex] = SupportTreeNode.CreateBranch(
                nodeIndex,
                min,
                max,
                leftIndex,
                rightIndex,
                minVertexIndex);
            return nodeIndex;
        }

        private void CalculateSupportRangeBounds(
            int start,
            int count,
            out Vector3d min,
            out Vector3d max,
            out int minVertexIndex)
        {
            int firstIndex = _supportVertexIndices![start];
            minVertexIndex = firstIndex;
            min = _localVertices[firstIndex];
            max = min;
            for (int i = 1; i < count; i++)
            {
                int vertexIndex = _supportVertexIndices[start + i];
                Vector3d vertex = _localVertices[vertexIndex];
                min = Vector3d.Min(min, vertex);
                max = Vector3d.Max(max, vertex);
                if (vertexIndex < minVertexIndex)
                    minVertexIndex = vertexIndex;
            }
        }

        private static int[] CreateSupportVertexIndices(int vertexCount)
        {
            var indices = new int[vertexCount];
            for (int i = 0; i < indices.Length; i++)
                indices[i] = i;

            return indices;
        }

        private static int GetDominantAxis(Vector3d extents)
        {
            if (extents.X >= extents.Y && extents.X >= extents.Z)
                return 0;

            return extents.Y >= extents.Z ? 1 : 2;
        }

        private static Fixed64 GetBoundsMaxProjection(Vector3d min, Vector3d max, Vector3d direction)
        {
            Fixed64 x = direction.X >= Fixed64.Zero ? max.X : min.X;
            Fixed64 y = direction.Y >= Fixed64.Zero ? max.Y : min.Y;
            Fixed64 z = direction.Z >= Fixed64.Zero ? max.Z : min.Z;
            return (x * direction.X) + (y * direction.Y) + (z * direction.Z);
        }

        public Vector3d GetFaceNormalWorld(int index)
        {
            SwiftThrowHelper.ThrowIfArrayIndexInvalid(index, _triangleCount, nameof(index));
            return Fixed3x3.TransformDirection(_rotationMatrix, _scaledFaceNormals[index]).Normalized;
        }

        public void GetTrianglesInWorldBounds(FixedBoundVolume worldBounds, SwiftList<int> result)
        {
            result.FastClear();
            FixedBoundVolume localBounds = TransformBounds(worldBounds.Min, worldBounds.Max, InverseTransformationMatrix);
            TriangleBVH.Query(localBounds, result);
        }

        public void GetTrianglesInLocalBounds(FixedBoundVolume localBounds, SwiftList<int> result)
        {
            result.FastClear();
            TriangleBVH.Query(localBounds, result);
        }

        public Vector3d ConvertWorldToLocal(Vector3d worldPoint) =>
            InverseTransformWorldPoint(worldPoint);

        public Vector3d ConvertLocalToWorld(Vector3d localPoint) =>
            TransformLocalPoint(localPoint);

        public Vector3d ConvertWorldDirectionToLocal(Vector3d worldDirection) =>
            Vector3d.Multiply(
                _scale,
                Fixed3x3.TransformDirection(_inverseRotationMatrix, worldDirection));

        private Vector3d TransformLocalPoint(Vector3d localPoint) =>
            _position + Fixed3x3.TransformDirection(
                _rotationMatrix,
                Vector3d.Multiply(_scale, localPoint));

        private readonly struct EdgeUse
        {
            private EdgeUse(long key, int direction, int triangleIndex)
            {
                Key = key;
                Direction = direction;
                TriangleIndex = triangleIndex;
            }

            public long Key { get; }

            public int StartVertexIndex => (int)(Key >> 32);

            public int EndVertexIndex => (int)(uint)Key;

            public int Direction { get; }

            public int TriangleIndex { get; }

            public static EdgeUse Create(int start, int end, int triangleIndex)
            {
                int min = start < end ? start : end;
                int max = start < end ? end : start;
                long key = ((long)min << 32) | (uint)max;
                int direction = start == min ? 1 : -1;
                return new EdgeUse(key, direction, triangleIndex);
            }
        }

        private readonly struct TriangleUse
        {
            private TriangleUse(int a, int b, int c)
            {
                A = a;
                B = b;
                C = c;
            }

            public int A { get; }

            public int B { get; }

            public int C { get; }

            public static TriangleUse Create(int first, int second, int third)
            {
                int a = first;
                int b = second;
                int c = third;

                if (a > b)
                    (a, b) = (b, a);
                if (b > c)
                    (b, c) = (c, b);
                if (a > b)
                    (a, b) = (b, a);

                return new TriangleUse(a, b, c);
            }
        }

        private readonly struct SupportTreeNode
        {
            private SupportTreeNode(
                int index,
                Vector3d min,
                Vector3d max,
                int left,
                int right,
                int start,
                int count,
                int minVertexIndex)
            {
                Index = index;
                Min = min;
                Max = max;
                Left = left;
                Right = right;
                Start = start;
                Count = count;
                MinVertexIndex = minVertexIndex;
            }

            public int Index { get; }

            public Vector3d Min { get; }

            public Vector3d Max { get; }

            public int Left { get; }

            public int Right { get; }

            public int Start { get; }

            public int Count { get; }

            public int MinVertexIndex { get; }

            public bool IsLeaf => Count > 0;

            public static SupportTreeNode CreateLeaf(
                int index,
                Vector3d min,
                Vector3d max,
                int start,
                int count,
                int minVertexIndex) =>
                new(index, min, max, -1, -1, start, count, minVertexIndex);

            public static SupportTreeNode CreateBranch(
                int index,
                Vector3d min,
                Vector3d max,
                int left,
                int right,
                int minVertexIndex) =>
                new(index, min, max, left, right, 0, 0, minVertexIndex);
        }

        private sealed class SupportVertexIndexComparer : IComparer<int>
        {
            private readonly Vector3d[] _vertices;
            private readonly int _axis;

            public SupportVertexIndexComparer(Vector3d[] vertices, int axis)
            {
                _vertices = vertices;
                _axis = axis;
            }

            public int Compare(int first, int second)
            {
                Fixed64 firstValue = GetAxisValue(_vertices[first], _axis);
                Fixed64 secondValue = GetAxisValue(_vertices[second], _axis);
                int valueComparison = firstValue.CompareTo(secondValue);
                if (valueComparison != 0)
                    return valueComparison;

                return first.CompareTo(second);
            }

            private static Fixed64 GetAxisValue(Vector3d vertex, int axis)
            {
                return axis switch
                {
                    0 => vertex.X,
                    1 => vertex.Y,
                    _ => vertex.Z
                };
            }
        }
    }
}
