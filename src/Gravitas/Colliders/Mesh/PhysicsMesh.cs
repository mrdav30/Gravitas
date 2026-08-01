//=======================================================================
// PhysicsMesh.cs
//=======================================================================
// MIT License, Copyright (c) 2026–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp;
using FixedMathSharp.Geometry;
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

        private Vector3d[] _scaledLocalVertices;
        private Vector3d[] _preparedScaledLocalVertices;
        private int[]? _supportVertexIndices;
        private int[]? _preparedSupportVertexIndices;
        private SupportTreeNode[]? _supportTreeNodes;
        private SupportTreeNode[]? _preparedSupportTreeNodes;
        private int _supportTreeNodeCount;
        private int _preparedSupportTreeNodeCount;
        private readonly SupportVertexIndexComparer? _supportVertexIndexComparer;

        /// <summary>
        /// Holds the source vertices in local mesh space.
        /// </summary>
        internal ReadOnlySpan<Vector3d> LocalVertices => _localVertices;

        /// <summary>
        /// Holds the committed scaled vertices as center-relative local
        /// offsets. Collision relations apply <see cref="Rotation"/> without
        /// materializing absolute world points.
        /// </summary>
        internal ReadOnlySpan<Vector3d> ScaledLocalVertices =>
            _scaledLocalVertices;

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

        /// <summary>
        /// Gets whether the complete exact-position-welded triangle topology is one
        /// connected, consistently wound, closed two-manifold surface.
        /// </summary>
        public bool IsClosedSurface { get; private set; }

        /// <summary>
        /// Gets the current triangle surface area after local scale.
        /// </summary>
        public Fixed64 TotalArea => _scaledTotalArea;

        private int _triangleBvhBuildCount;
        private SwiftFixedBVH<int> _triangleBVH;
        private SwiftFixedBVH<int> _preparedTriangleBVH;

        /// <summary>
        /// Triangle acceleration structure in local mesh space.
        /// </summary>
        internal SwiftFixedBVH<int> TriangleBVH => _triangleBVH;

        public int TriangleBvhBuildCount => _triangleBvhBuildCount;

        private FixedQuaternion _rotation = FixedQuaternion.Identity;

        /// <summary>
        /// Gets the committed world-space origin of the mesh's rigid frame.
        /// </summary>
        internal Vector3d Origin => _position;

        /// <summary>
        /// Gets the committed local-to-world mesh orientation.
        /// </summary>
        internal FixedQuaternion Rotation => _rotation;

        private FixedBoundBox _bounds;
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
            _scaledLocalVertices = new Vector3d[vertices.Length];
            _preparedScaledLocalVertices = new Vector3d[vertices.Length];
            _triangles = new int[triangles.Length];
            Array.Copy(triangles, _triangles, triangles.Length);
            _triangleCount = triangles.Length / 3; // 3 vertices per triangle
            _triangleBVH = new SwiftFixedBVH<int>(2 * TriangleCount - 1);
            _preparedTriangleBVH = new SwiftFixedBVH<int>(2 * TriangleCount - 1);
            _scaledFaceAreas = new Fixed64[TriangleCount];
            _preparedScaledFaceAreas = new Fixed64[TriangleCount];
            _scaledFaceNormals = new Vector3d[TriangleCount];
            _preparedScaledFaceNormals = new Vector3d[TriangleCount];

            _localBounds = CalculateBounds(_localVertices);

            // Scale validation proves every vertex span and triangle cross product used
            // by the exact topology predicates below is representable.
            int[] topologyTriangles = CreateTopologyTriangles(_localVertices, _triangles);
            _convexSatEdgeVertexPairs = Mode == MeshColliderMode.Convex
                ? CreateConvexSatEdgeVertexPairs(_localVertices, topologyTriangles, _triangleCount)
                : Array.Empty<int>();
            if (Mode == MeshColliderMode.Concave)
            {
                IsClosedSurface = EvaluateClosedVolumeTopology(topologyTriangles, out _surfaceClosureValidationResult);
            }

            if (Mode == MeshColliderMode.Convex && _localVertices.Length > SupportTreeVertexThreshold)
            {
                _supportVertexIndices = CreateSupportVertexIndices(_localVertices.Length);
                _preparedSupportVertexIndices = new int[_localVertices.Length];
                _supportTreeNodes = new SupportTreeNode[(2 * _localVertices.Length) - 1];
                _preparedSupportTreeNodes = new SupportTreeNode[(2 * _localVertices.Length) - 1];
                _supportVertexIndexComparer = new SupportVertexIndexComparer();
            }

            PrepareTransformation(position, rotation, Vector3d.One, Vector3d.One, null);
            PublishPreparedTransformation();
        }

        public void UpdatePosition(Vector3d position, FixedQuaternion rotation)
        {
            PrepareTransformation(position, rotation, _ownerScale, _partScale, null);
            PublishPreparedTransformation();
        }

        /// <summary>
        /// Updates the mesh center, normalized rigid rotation, and strictly positive authored scale.
        /// </summary>
        public void UpdateTransform(Vector3d position, FixedQuaternion rotation, Vector3d scale)
        {
            PrepareTransformation(position, rotation, scale, Vector3d.One, null);
            PublishPreparedTransformation();
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

            var referencedVertices = new bool[vertices.Length];
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

                Fixed64 area = new FixedTriangle(
                    vertices[index0],
                    vertices[index1],
                    vertices[index2]).Area;
                SwiftThrowHelper.ThrowIfArgument(area <= Fixed64.Epsilon, nameof(triangles), "Degenerate triangles are not supported.");

                referencedVertices[index0] = true;
                referencedVertices[index1] = true;
                referencedVertices[index2] = true;
            }

            for (int i = 0; i < referencedVertices.Length; i++)
                SwiftThrowHelper.ThrowIfArgument(!referencedVertices[i], nameof(vertices), "Every mesh vertex must be referenced by at least one triangle.");
        }

        private void BuildTriangleBVH(
            SwiftFixedBVH<int> bvh,
            ReadOnlySpan<Vector3d> vertices)
        {
            bvh.Clear();
            for (int i = 0; i < _triangleCount; i++)
            {
                int index0 = _triangles[i * 3];
                int index1 = _triangles[i * 3 + 1];
                int index2 = _triangles[i * 3 + 2];
                Vector3d min = Vector3d.Min(Vector3d.Min(vertices[index0], vertices[index1]), vertices[index2]);
                Vector3d max = Vector3d.Max(Vector3d.Max(vertices[index0], vertices[index1]), vertices[index2]);
                bvh.Insert(i, new FixedBoundVolume(min, max));
            }
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

        /// <summary>
        /// Calculates solid closed-volume inertia for the supplied mass.
        /// This is a geometry/topology API; callers apply body mobility gates before requesting inertia.
        /// </summary>
        private static int CompareEdgeUses(EdgeUse first, EdgeUse second)
        {
            if (first.Key != second.Key)
                return first.Key < second.Key ? -1 : 1;

            if (first.TriangleIndex == second.TriangleIndex)
                return 0;

            return first.TriangleIndex < second.TriangleIndex ? -1 : 1;
        }

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
            if (TryGetTriangleVertices(index, out first, out second, out third))
                return;

            throw new InvalidOperationException(
                "At least one triangle vertex lies outside the Fixed64 world-coordinate domain.");
        }

        /// <summary>
        /// Attempts to materialize one triangle's absolute world vertices.
        /// Canonical collision and query paths should consume scaled-local
        /// geometry instead.
        /// </summary>
        public bool TryGetTriangleVertices(
            int index,
            out Vector3d first,
            out Vector3d second,
            out Vector3d third)
        {
            GetLocalTriangleVertices(
                index,
                out Vector3d localFirst,
                out Vector3d localSecond,
                out Vector3d localThird);
            var firstAnchor = new FixedPointAnchor(
                _position,
                _rotation,
                localFirst);
            var secondAnchor = new FixedPointAnchor(
                _position,
                _rotation,
                localSecond);
            var thirdAnchor = new FixedPointAnchor(
                _position,
                _rotation,
                localThird);
            bool representable =
                firstAnchor.TryGetPoint(out first)
                & secondAnchor.TryGetPoint(out second)
                & thirdAnchor.TryGetPoint(out third);
            if (representable)
                return true;

            first = default;
            second = default;
            third = default;
            return false;
        }

        /// <summary>
        /// Gets one triangle's committed scaled vertices in local mesh space.
        /// </summary>
        public void GetLocalTriangleVertices(int index, out Vector3d first, out Vector3d second, out Vector3d third)
        {
            SwiftThrowHelper.ThrowIfArrayIndexInvalid(index, _triangleCount, nameof(index));

            int triangleIndex = index * 3;
            first = _scaledLocalVertices[_triangles[triangleIndex]];
            second = _scaledLocalVertices[_triangles[triangleIndex + 1]];
            third = _scaledLocalVertices[_triangles[triangleIndex + 2]];
        }

        /// <summary>
        /// Gets one triangle's committed scaled normal in local mesh space.
        /// </summary>
        internal Vector3d GetScaledLocalFaceNormal(int index)
        {
            SwiftThrowHelper.ThrowIfArrayIndexInvalid(index, _triangleCount, nameof(index));
            return _scaledFaceNormals[index];
        }

        /// <summary>
        /// Attempts to materialize one scaled vertex in world space.
        /// </summary>
        public bool TryGetVertexWorld(int index, out Vector3d vertex)
        {
            SwiftThrowHelper.ThrowIfArrayIndexInvalid(index, _localVertices.Length, nameof(index));
            return CreatePointAnchor(_scaledLocalVertices[index])
                .TryGetPoint(out vertex);
        }

        /// <summary>
        /// Materializes one scaled vertex in world space.
        /// </summary>
        /// <exception cref="InvalidOperationException">
        /// The conceptual vertex lies outside the representable world-coordinate
        /// domain.
        /// </exception>
        public Vector3d GetVertexWorld(int index)
        {
            if (TryGetVertexWorld(index, out Vector3d vertex))
                return vertex;

            throw new InvalidOperationException(
                "The selected vertex lies outside the Fixed64 world-coordinate domain.");
        }

        /// <summary>
        /// Attempts to materialize the world-space vertex with the greatest
        /// projection onto <paramref name="direction"/>, preserving source
        /// vertex order for ties.
        /// </summary>
        public bool TryGetSupportVertexWorld(
            Vector3d direction,
            out Vector3d vertex)
        {
            Vector3d localPoint = GetSupportVertexLocal(direction);
            return CreatePointAnchor(localPoint).TryGetPoint(out vertex);
        }

        /// <summary>
        /// Materializes the world-space vertex with the greatest projection
        /// onto <paramref name="direction"/>, preserving source vertex order
        /// for ties.
        /// </summary>
        /// <exception cref="InvalidOperationException">
        /// The conceptual support vertex lies outside the representable
        /// world-coordinate domain.
        /// </exception>
        public Vector3d GetSupportVertexWorld(Vector3d direction)
        {
            if (TryGetSupportVertexWorld(direction, out Vector3d vertex))
                return vertex;

            throw new InvalidOperationException(
                "The selected support vertex lies outside the Fixed64 world-coordinate domain.");
        }

        internal Vector3d GetSupportVertexLocal(Vector3d direction)
        {
            Vector3d localDirection = ConvertWorldDirectionToLocal(direction);
            localDirection = localDirection != Vector3d.Zero
                ? localDirection.Normalized
                : Vector3d.Right;

            if (_supportTreeNodes != null && _supportVertexIndices != null)
                return _scaledLocalVertices[FindSupportVertexIndex(localDirection)];

            int bestIndex = 0;
            for (int i = 1; i < _scaledLocalVertices.Length; i++)
            {
                if (Vector3d.CompareProjection(
                        _scaledLocalVertices[i],
                        _scaledLocalVertices[bestIndex],
                        localDirection) <= 0)
                    continue;

                bestIndex = i;
            }

            return _scaledLocalVertices[bestIndex];
        }

        private int FindSupportVertexIndex(Vector3d localDirection)
        {
            int bestIndex = 0;
            Span<int> stack = stackalloc int[SupportTreeStackCapacity];
            int stackCount = 0;
            stack[stackCount++] = 0;

            while (stackCount > 0)
            {
                int nodeIndex = stack[--stackCount];
                SupportTreeNode node = _supportTreeNodes![nodeIndex];
                Vector3d upperPoint = GetBoundsSupportPoint(node.Min, node.Max, localDirection);
                int upperComparison = Vector3d.CompareProjection(
                    upperPoint,
                    _scaledLocalVertices[bestIndex],
                    localDirection);
                if (upperComparison < 0 || (upperComparison == 0 && node.MinVertexIndex >= bestIndex))
                    continue;

                if (node.IsLeaf)
                {
                    SearchSupportLeaf(node, localDirection, ref bestIndex);
                    continue;
                }

                SupportTreeNode left = _supportTreeNodes[node.Left];
                SupportTreeNode right = _supportTreeNodes[node.Right];

                if (ComesBeforeSupportNode(left, right, localDirection))
                {
                    PushSupportNode(right, bestIndex, localDirection, stack, ref stackCount);
                    PushSupportNode(left, bestIndex, localDirection, stack, ref stackCount);
                    continue;
                }

                PushSupportNode(left, bestIndex, localDirection, stack, ref stackCount);
                PushSupportNode(right, bestIndex, localDirection, stack, ref stackCount);
            }

            return bestIndex;
        }

        private void SearchSupportLeaf(
            SupportTreeNode node,
            Vector3d localDirection,
            ref int bestIndex)
        {
            for (int i = 0; i < node.Count; i++)
            {
                int vertexIndex = _supportVertexIndices![node.Start + i];
                Vector3d vertex = _scaledLocalVertices[vertexIndex];
                int projectionComparison = Vector3d.CompareProjection(
                    vertex,
                    _scaledLocalVertices[bestIndex],
                    localDirection);
                if (projectionComparison < 0 || (projectionComparison == 0 && vertexIndex >= bestIndex))
                    continue;

                bestIndex = vertexIndex;
            }
        }

        private void PushSupportNode(
            SupportTreeNode node,
            int bestIndex,
            Vector3d localDirection,
            Span<int> stack,
            ref int stackCount)
        {
            Vector3d upperPoint = GetBoundsSupportPoint(node.Min, node.Max, localDirection);
            int upperComparison = Vector3d.CompareProjection(
                upperPoint,
                _scaledLocalVertices[bestIndex],
                localDirection);
            if (upperComparison < 0 || (upperComparison == 0 && node.MinVertexIndex >= bestIndex))
                return;

            stack[stackCount++] = node.Index;
        }

        private static bool ComesBeforeSupportNode(
            SupportTreeNode left,
            SupportTreeNode right,
            Vector3d localDirection)
        {
            Vector3d leftPoint = GetBoundsSupportPoint(left.Min, left.Max, localDirection);
            Vector3d rightPoint = GetBoundsSupportPoint(right.Min, right.Max, localDirection);
            int projectionComparison = Vector3d.CompareProjection(
                leftPoint,
                rightPoint,
                localDirection);
            if (projectionComparison != 0)
                return projectionComparison > 0;

            return left.MinVertexIndex < right.MinVertexIndex;
        }

        private int BuildSupportTreeNode(
            Vector3d[] vertices,
            int[] vertexIndices,
            SupportTreeNode[] nodes,
            ref int nodeCount,
            int start,
            int count)
        {
            int nodeIndex = nodeCount++;

            CalculateSupportRangeBounds(
                vertices,
                vertexIndices,
                start,
                count,
                out Vector3d min,
                out Vector3d max,
                out int minVertexIndex);
            if (count <= SupportTreeLeafVertexCount)
            {
                nodes[nodeIndex] = SupportTreeNode.CreateLeaf(
                    nodeIndex,
                    min,
                    max,
                    start,
                    count,
                    minVertexIndex);
                return nodeIndex;
            }

            int axis = GetDominantAxis(max - min);
            _supportVertexIndexComparer!.Reset(vertices, axis);
            Array.Sort(
                vertexIndices,
                start,
                count,
                _supportVertexIndexComparer);

            int leftCount = count / 2;
            int rightCount = count - leftCount;
            int leftIndex = BuildSupportTreeNode(
                vertices,
                vertexIndices,
                nodes,
                ref nodeCount,
                start,
                leftCount);
            int rightIndex = BuildSupportTreeNode(
                vertices,
                vertexIndices,
                nodes,
                ref nodeCount,
                start + leftCount,
                rightCount);
            nodes[nodeIndex] = SupportTreeNode.CreateBranch(
                nodeIndex,
                min,
                max,
                leftIndex,
                rightIndex,
                minVertexIndex);
            return nodeIndex;
        }

        private static void CalculateSupportRangeBounds(
            Vector3d[] vertices,
            int[] vertexIndices,
            int start,
            int count,
            out Vector3d min,
            out Vector3d max,
            out int minVertexIndex)
        {
            int firstIndex = vertexIndices[start];
            minVertexIndex = firstIndex;
            min = vertices[firstIndex];
            max = min;
            for (int i = 1; i < count; i++)
            {
                int vertexIndex = vertexIndices[start + i];
                Vector3d vertex = vertices[vertexIndex];
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

        private static Vector3d GetBoundsSupportPoint(Vector3d min, Vector3d max, Vector3d direction) =>
            new(
                direction.X >= Fixed64.Zero ? max.X : min.X,
                direction.Y >= Fixed64.Zero ? max.Y : min.Y,
                direction.Z >= Fixed64.Zero ? max.Z : min.Z);

        public Vector3d GetFaceNormalWorld(int index)
        {
            SwiftThrowHelper.ThrowIfArrayIndexInvalid(index, _triangleCount, nameof(index));
            _ = _rotation.TryRotate(
                _scaledFaceNormals[index],
                out Vector3d worldNormal);
            return worldNormal.Normalized;
        }

        public void GetTrianglesInWorldBounds(FixedBoundVolume worldBounds, SwiftList<int> result)
        {
            result.FastClear();
            FixedBoundBox localBounds =
                FixedBoundBox.FromRelativeRotatedBoundsClippedToDomain(
                    Vector3d.Zero,
                    FixedQuaternion.Identity,
                    worldBounds.Min,
                    worldBounds.Max,
                    _position,
                    _rotation);
            TriangleBVH.Query(
                new FixedBoundVolume(localBounds.Min, localBounds.Max),
                result);
        }

        public void GetTrianglesInLocalBounds(FixedBoundVolume localBounds, SwiftList<int> result)
        {
            result.FastClear();
            TriangleBVH.Query(localBounds, result);
        }

        /// <summary>
        /// Attempts to express a world point in the committed scaled-local
        /// mesh frame without saturating the relative displacement.
        /// </summary>
        public bool TryConvertWorldToScaledLocal(
            Vector3d worldPoint,
            out Vector3d localPoint) =>
            new FixedPointAnchor(
                worldPoint,
                FixedQuaternion.Identity,
                Vector3d.Zero)
            .TryGetLocalPointIn(
                _position,
                _rotation,
                out localPoint);

        /// <summary>
        /// Expresses a world point in the committed scaled-local mesh frame.
        /// </summary>
        /// <exception cref="InvalidOperationException">
        /// The exact relative displacement lies outside the representable
        /// coordinate domain.
        /// </exception>
        public Vector3d ConvertWorldToScaledLocal(Vector3d worldPoint)
        {
            if (TryConvertWorldToScaledLocal(worldPoint, out Vector3d localPoint))
                return localPoint;

            throw new InvalidOperationException(
                "The world point cannot be represented in the mesh's scaled-local frame.");
        }

        /// <summary>
        /// Attempts to materialize a scaled-local mesh point in world space.
        /// </summary>
        public bool TryConvertScaledLocalToWorld(
            Vector3d scaledLocalPoint,
            out Vector3d worldPoint) =>
            CreatePointAnchor(scaledLocalPoint).TryGetPoint(out worldPoint);

        /// <summary>
        /// Materializes a scaled-local mesh point in world space.
        /// </summary>
        /// <exception cref="InvalidOperationException">
        /// The conceptual world point lies outside the representable
        /// coordinate domain.
        /// </exception>
        public Vector3d ConvertScaledLocalToWorld(Vector3d scaledLocalPoint)
        {
            if (TryConvertScaledLocalToWorld(
                    scaledLocalPoint,
                    out Vector3d worldPoint))
            {
                return worldPoint;
            }

            throw new InvalidOperationException(
                "The scaled-local point lies outside the Fixed64 world-coordinate domain.");
        }

        internal FixedPointAnchor CreatePointAnchor(
            Vector3d scaledLocalPoint) =>
            new(
                _position,
                _rotation,
                scaledLocalPoint);

        /// <summary>
        /// Attempts to express a world-space direction in the committed
        /// scaled-local mesh frame.
        /// </summary>
        public bool TryConvertWorldDirectionToLocal(
            Vector3d worldDirection,
            out Vector3d localDirection) =>
            _rotation.Inverse().TryRotate(
                worldDirection,
                out localDirection);

        /// <summary>
        /// Expresses a world-space direction in the committed scaled-local
        /// mesh frame.
        /// </summary>
        public Vector3d ConvertWorldDirectionToLocal(
            Vector3d worldDirection)
        {
            if (TryConvertWorldDirectionToLocal(
                    worldDirection,
                    out Vector3d localDirection))
            {
                return localDirection;
            }

            throw new InvalidOperationException(
                "The world direction cannot be represented in the mesh's scaled-local frame.");
        }

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
            private Vector3d[] _vertices = Array.Empty<Vector3d>();
            private int _axis;

            public void Reset(Vector3d[] vertices, int axis)
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
