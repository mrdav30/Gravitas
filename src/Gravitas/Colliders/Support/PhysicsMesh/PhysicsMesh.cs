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

namespace Gravitas.Colliders
{
    public class PhysicsMesh
    {
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

        /// <summary>
        /// Holds all vertices transformed to world space. Prefer point-specific helpers on hot paths.
        /// </summary>
        public Vector3d[] Vertices
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
        internal Vector3d[] LocalVertices => _localVertices;

        /// <summary>
        /// Number of vertices in the immutable local mesh topology.
        /// </summary>
        public int VertexCount => _localVertices.Length;

        private readonly int[] _triangles;
        /// <summary>
        /// Holds all the triangles that make up the mesh in the form of indices to the vertices array.
        /// example: Triangles[0-3] = 4,2,3 means that the first triangle of the mesh is made up of the vertices _vertices[4], _vertices[2], _vertices[3]
        /// </summary>
        public int[] Triangles => _triangles;

        private readonly int _triangleCount;
        public int TriangleCount => _triangleCount;

        public MeshColliderMode Mode { get; }

        private bool _faceNormalsValid;
        private readonly Vector3d[] _faceNormals;
        /// <summary>
        /// Holds triangle normals in local mesh space.
        /// </summary>
        public Vector3d[] FaceNormals => !_faceNormalsValid
            ? CalculateFaceNormals()
            : _faceNormals;

        private readonly int[][] _edges;

        private bool _edgesNormalsValid;
        private readonly Vector3d[] _edgeNormals;
        public Vector3d[] EdgeNormals => !_edgesNormalsValid
            ? CalculateEdgeNormals()
            : _edgeNormals;

        private readonly Fixed64[] _faceAreas;
        public Fixed64[] FaceAreas => _faceAreas;

        private Fixed64 _totalArea;
        public Fixed64 TotalArea => _totalArea;

        public Fixed3x3 Tensor { get; private set; }

        private bool _closedVolumeMassPropertiesEvaluated;
        private MeshMassProperties _closedVolumeMassProperties;
        private MeshVolumeValidationResult _closedVolumeValidationResult;

        private bool _triangleBVHValid;
        private int _triangleBvhBuildCount;
        private readonly SwiftFixedBVH<int> _triangleBVH;

        /// <summary>
        /// Triangle acceleration structure in local mesh space.
        /// </summary>
        public SwiftFixedBVH<int> TriangleBVH => !_triangleBVHValid
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
                if (_inverseMatrixValid) return _inverseTransformationMatrix;

                if (!Fixed4x4.Invert(TransformationMatrix, out Fixed4x4 inverseMatrix))
                    throw new InvalidOperationException("Cannot invert the transformation matrix.");

                _inverseMatrixValid = true;
                return _inverseTransformationMatrix = inverseMatrix;
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
            _edges = new int[TriangleCount][];
            _edgeNormals = new Vector3d[TriangleCount * 3]; // 3 edges per triangle

            _faceAreas = new Fixed64[TriangleCount];
            _totalArea = Fixed64.Zero;

            _localBounds = CalculateBounds(_localVertices);

            UpdateTransformationMatrix(position, rotation);

            for (int i = 0; i < _triangleCount; i++)
            {
                int index0 = _triangles[i * 3];
                int index1 = _triangles[i * 3 + 1];
                int index2 = _triangles[i * 3 + 2];

                _edges[i] = new[] { index0, index1, index1, index2, index2, index0 };
                _faceAreas[i] = CalculateTriangleArea(
                    _localVertices[index0],
                    _localVertices[index1],
                    _localVertices[index0],
                    _localVertices[index2]);
                _totalArea += _faceAreas[i];
            }

            UpdateBounds();
        }

        public void UpdatePosition(Vector3d position, FixedQuaternion rotation)
        {
            UpdateTransformationMatrix(position, rotation);
            UpdateBounds();
        }

        private void UpdateTransformationMatrix(Vector3d position, FixedQuaternion rotation)
        {
            _rotation = rotation;
            TransformationMatrix = Fixed4x4.TranslateRotateScale(position, rotation, Vector3d.One);
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

        public Vector3d[] CalculateEdgeNormals()
        {
            for (int i = 0; i < _triangleCount; i++)
            {
                for (int n = 0; n < 3; n++)
                {
                    int edgeStart = _edges[i][n * 2];
                    int edgeEnd = _edges[i][n * 2 + 1];
                    _edgeNormals[i * 3 + n] = (_localVertices[edgeEnd] - _localVertices[edgeStart]).Normalized;
                }
            }

            _edgesNormalsValid = true;
            return _edgeNormals;
        }

        private Vector3d[] CalculateFaceNormals()
        {
            for (int i = 0; i < _triangleCount; i++)
            {
                int index0 = _triangles[i * 3];
                int index1 = _triangles[i * 3 + 1];
                int index2 = _triangles[i * 3 + 2];
                _faceNormals[i] = Vector3d.Cross(_localVertices[index1] - _localVertices[index0], _localVertices[index2] - _localVertices[index0]).Normalized;
            }

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
                _bounds = new FixedBoundBox((volume.Min + volume.Max) * Fixed64.Half, volume.Max - volume.Min);
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

            return new FixedBoundBox((min + max) * Fixed64.Half, max - min);
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
        public Fixed3x3 CalculateInertiaTensor(Fixed64 mass) =>
            CalculateInertiaTensor(mass, MeshInertiaPolicy.RequireClosedVolume);

        /// <summary>
        /// Calculates mesh inertia for the supplied mass using the requested policy.
        /// This is a geometry/topology API; callers apply body mobility gates before requesting inertia.
        /// </summary>
        public Fixed3x3 CalculateInertiaTensor(Fixed64 mass, MeshInertiaPolicy policy)
        {
            if (policy == MeshInertiaPolicy.RequireClosedVolume
                && TryGetClosedVolumeMassProperties(out MeshMassProperties properties, out _))
            {
                return CalculateInertiaTensor(mass, policy, properties.CenterOfMass);
            }

            return CalculateInertiaTensor(mass, policy, _localBounds.Center);
        }

        /// <summary>
        /// Calculates mesh inertia for the supplied mass about a specific local reference point.
        /// This is a geometry/topology API; callers apply body mobility gates before requesting inertia.
        /// </summary>
        public Fixed3x3 CalculateInertiaTensor(Fixed64 mass, MeshInertiaPolicy policy, Vector3d localReferencePoint)
        {
            switch (policy)
            {
                case MeshInertiaPolicy.RequireClosedVolume:
                    if (!TryGetClosedVolumeMassProperties(out MeshMassProperties properties, out MeshVolumeValidationResult result))
                        throw new InvalidOperationException($"Mesh inertia requires a validated closed volume. Validation result: {result}.");

                    return properties.CalculateInertiaTensor(mass, localReferencePoint);

                case MeshInertiaPolicy.SurfaceApproximation:
                    return CalculateSurfaceApproximationInertiaTensor(mass);

                default:
                    throw new ArgumentOutOfRangeException(nameof(policy), policy, "Unsupported mesh inertia policy.");
            }
        }

        /// <summary>
        /// Gets cached closed-volume mass properties when this mesh is a valid closed triangle shell.
        /// </summary>
        public bool TryGetClosedVolumeMassProperties(
            out MeshMassProperties properties,
            out MeshVolumeValidationResult result)
        {
            EnsureClosedVolumeMassProperties();
            properties = _closedVolumeMassProperties;
            result = _closedVolumeValidationResult;
            return result == MeshVolumeValidationResult.Valid;
        }

        private Fixed3x3 CalculateSurfaceApproximationInertiaTensor(Fixed64 mass)
        {
            Fixed3x3 tensor = Fixed3x3.Zero;
            if (_totalArea <= Fixed64.Zero)
                return tensor;

            for (int i = 0; i < _triangleCount; i++)
            {
                int index0 = _triangles[i * 3];
                int index1 = _triangles[i * 3 + 1];
                int index2 = _triangles[i * 3 + 2];
                Fixed3x3 triangleTensor = new(_localVertices[index0], _localVertices[index1], _localVertices[index2]);
                Fixed64 triangleMass = mass * (_faceAreas[i] / _totalArea);
                triangleTensor *= triangleMass;
                tensor += triangleTensor;
            }

            tensor /= _triangleCount;
            return tensor;
        }

        private void EnsureClosedVolumeMassProperties()
        {
            if (_closedVolumeMassPropertiesEvaluated)
                return;

            _closedVolumeMassPropertiesEvaluated = true;

            if (!ValidateClosedVolumeTopology(out MeshVolumeValidationResult topologyResult))
            {
                _closedVolumeValidationResult = topologyResult;
                _closedVolumeMassProperties = default;
                return;
            }

            if (!TryCalculateClosedVolumeMassProperties(out MeshMassProperties properties, out MeshVolumeValidationResult volumeResult))
            {
                _closedVolumeValidationResult = volumeResult;
                _closedVolumeMassProperties = default;
                return;
            }

            _closedVolumeValidationResult = MeshVolumeValidationResult.Valid;
            _closedVolumeMassProperties = properties;
        }

        private bool ValidateClosedVolumeTopology(out MeshVolumeValidationResult result)
        {
            var triangleUses = new TriangleUse[_triangleCount];
            var edgeUses = new EdgeUse[_triangleCount * 3];
            int edgeIndex = 0;
            for (int i = 0; i < _triangleCount; i++)
            {
                int triangleIndex = i * 3;
                int index0 = _triangles[triangleIndex];
                int index1 = _triangles[triangleIndex + 1];
                int index2 = _triangles[triangleIndex + 2];

                triangleUses[i] = TriangleUse.Create(index0, index1, index2);
                edgeUses[edgeIndex++] = EdgeUse.Create(index0, index1, i);
                edgeUses[edgeIndex++] = EdgeUse.Create(index1, index2, i);
                edgeUses[edgeIndex++] = EdgeUse.Create(index2, index0, i);
            }

            if (ContainsDuplicateTriangle(triangleUses))
            {
                result = MeshVolumeValidationResult.DuplicateTriangle;
                return false;
            }

            Array.Sort(edgeUses, CompareEdgeUses);

            int[] parents = new int[_triangleCount];
            for (int i = 0; i < parents.Length; i++)
                parents[i] = i;

            for (int i = 0; i < edgeUses.Length;)
            {
                int groupStart = i;
                EdgeUse first = edgeUses[i++];
                while (i < edgeUses.Length && edgeUses[i].Key == first.Key)
                    i++;

                int count = i - groupStart;
                if (count == 1)
                {
                    result = MeshVolumeValidationResult.BoundaryEdge;
                    return false;
                }

                if (count > 2)
                {
                    result = MeshVolumeValidationResult.NonManifoldEdge;
                    return false;
                }

                EdgeUse second = edgeUses[groupStart + 1];
                if (first.Direction + second.Direction != 0)
                {
                    result = MeshVolumeValidationResult.InconsistentWinding;
                    return false;
                }

                Union(parents, first.TriangleIndex, second.TriangleIndex);
            }

            int root = Find(parents, 0);
            for (int i = 1; i < parents.Length; i++)
            {
                if (Find(parents, i) == root)
                    continue;

                result = MeshVolumeValidationResult.DisconnectedShell;
                return false;
            }

            result = MeshVolumeValidationResult.Valid;
            return true;
        }

        private bool TryCalculateClosedVolumeMassProperties(
            out MeshMassProperties properties,
            out MeshVolumeValidationResult result)
        {
            Vector3d reference = _localBounds.Center;
            Fixed64 signedVolume = Fixed64.Zero;
            Vector3d firstMoment = Vector3d.Zero;
            Fixed64 integralX2 = Fixed64.Zero;
            Fixed64 integralY2 = Fixed64.Zero;
            Fixed64 integralZ2 = Fixed64.Zero;
            Fixed64 integralXY = Fixed64.Zero;
            Fixed64 integralXZ = Fixed64.Zero;
            Fixed64 integralYZ = Fixed64.Zero;

            for (int i = 0; i < _triangleCount; i++)
            {
                int triangleIndex = i * 3;
                Vector3d a = _localVertices[_triangles[triangleIndex]] - reference;
                Vector3d b = _localVertices[_triangles[triangleIndex + 1]] - reference;
                Vector3d c = _localVertices[_triangles[triangleIndex + 2]] - reference;

                Fixed64 volume = Vector3d.Dot(a, Vector3d.Cross(b, c)) / TetrahedronVolumeDivisor;
                signedVolume += volume;
                firstMoment += (a + b + c) * (volume / TetrahedronCentroidDivisor);

                Fixed3x3 productSums = Fixed3x3.CreateBarycentricProductSums(a, b, c);
                integralX2 += volume * productSums.M11 / SecondMomentIntegralDivisor;
                integralY2 += volume * productSums.M22 / SecondMomentIntegralDivisor;
                integralZ2 += volume * productSums.M33 / SecondMomentIntegralDivisor;
                integralXY += volume * productSums.M12 / ProductMomentIntegralDivisor;
                integralXZ += volume * productSums.M13 / ProductMomentIntegralDivisor;
                integralYZ += volume * productSums.M23 / ProductMomentIntegralDivisor;
            }

            Fixed64 absoluteVolume = signedVolume.Abs();
            if (absoluteVolume <= Fixed64.Epsilon)
            {
                properties = default;
                result = MeshVolumeValidationResult.ZeroVolume;
                return false;
            }

            Fixed64 orientationSign = signedVolume < Fixed64.Zero ? -Fixed64.One : Fixed64.One;
            Vector3d centerOfMass = reference + firstMoment / signedVolume;
            Fixed64 ixx = orientationSign * (integralY2 + integralZ2) / absoluteVolume;
            Fixed64 iyy = orientationSign * (integralX2 + integralZ2) / absoluteVolume;
            Fixed64 izz = orientationSign * (integralX2 + integralY2) / absoluteVolume;
            Fixed64 ixy = -orientationSign * integralXY / absoluteVolume;
            Fixed64 ixz = -orientationSign * integralXZ / absoluteVolume;
            Fixed64 iyz = -orientationSign * integralYZ / absoluteVolume;

            properties = new MeshMassProperties(
                absoluteVolume,
                centerOfMass,
                reference,
                new Fixed3x3(
                    ixx, ixy, ixz,
                    ixy, iyy, iyz,
                    ixz, iyz, izz));
            result = MeshVolumeValidationResult.Valid;
            return true;
        }

        private static int CompareEdgeUses(EdgeUse first, EdgeUse second) =>
            first.Key < second.Key
                ? -1
                : first.Key > second.Key
                    ? 1
                    : 0;

        private static bool ContainsDuplicateTriangle(TriangleUse[] triangleUses)
        {
            Array.Sort(triangleUses, CompareTriangleUses);
            for (int i = 1; i < triangleUses.Length; i++)
            {
                if (triangleUses[i].Equals(triangleUses[i - 1]))
                    return true;
            }

            return false;
        }

        private static int CompareTriangleUses(TriangleUse first, TriangleUse second)
        {
            if (first.A != second.A)
                return first.A < second.A ? -1 : 1;

            if (first.B != second.B)
                return first.B < second.B ? -1 : 1;

            if (first.C != second.C)
                return first.C < second.C ? -1 : 1;

            return 0;
        }

        private static int Find(int[] parents, int index)
        {
            while (parents[index] != index)
            {
                parents[index] = parents[parents[index]];
                index = parents[index];
            }

            return index;
        }

        private static void Union(int[] parents, int first, int second)
        {
            int firstRoot = Find(parents, first);
            int secondRoot = Find(parents, second);
            if (firstRoot == secondRoot)
                return;

            if (firstRoot < secondRoot)
                parents[secondRoot] = firstRoot;
            else
                parents[firstRoot] = secondRoot;
        }

        public Fixed64 GetFrontalArea(Vector3d direction)
        {
            Fixed64 totalArea = Fixed64.Zero;
            for (int i = 0; i < FaceNormals.Length; i++)
            {
                if (Vector3d.Dot(GetFaceNormalWorld(i), direction) > Fixed64.Zero)
                    totalArea += _faceAreas[i];
            }

            return totalArea;
        }

        public Vector3d[] GetTriangleAtIndex(int index)
        {
            GetTriangleVertices(index, out Vector3d first, out Vector3d second, out Vector3d third);

            return new[]
            {
                first,
                second,
                third
            };
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

        public int GetTriangleVertexIndex(int triangleIndex, int vertexOffset)
        {
            SwiftThrowHelper.ThrowIfArrayIndexInvalid(triangleIndex, _triangleCount, nameof(triangleIndex));
            SwiftThrowHelper.ThrowIfArrayIndexInvalid(vertexOffset, 3, nameof(vertexOffset));
            return _triangles[triangleIndex * 3 + vertexOffset];
        }

        public Vector3d GetVertexWorld(int index)
        {
            SwiftThrowHelper.ThrowIfArrayIndexInvalid(index, _localVertices.Length, nameof(index));
            return TransformLocalPoint(_localVertices[index]);
        }

        public Vector3d GetFaceNormalWorld(int index)
        {
            SwiftThrowHelper.ThrowIfArrayIndexInvalid(index, _triangleCount, nameof(index));
            return TransformLocalNormal(FaceNormals[index]);
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
            InverseTransformationMatrix * worldPoint;

        public Vector3d ConvertLocalToWorld(Vector3d localPoint) =>
            TransformLocalPoint(localPoint);

        public Vector3d ConvertWorldDirectionToLocal(Vector3d worldDirection) =>
            _rotation.Inverse() * worldDirection;

        public Vector3d ConvertLocalNormalToWorld(Vector3d localNormal) =>
            TransformLocalNormal(localNormal);

        private Vector3d TransformLocalPoint(Vector3d localPoint) =>
            TransformationMatrix * localPoint;

        private Vector3d TransformLocalNormal(Vector3d localNormal) =>
            localNormal == Vector3d.Zero ? Vector3d.Zero : (_rotation * localNormal).Normalized;

        private readonly struct EdgeUse
        {
            private EdgeUse(long key, int direction, int triangleIndex)
            {
                Key = key;
                Direction = direction;
                TriangleIndex = triangleIndex;
            }

            public long Key { get; }

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

        private readonly struct TriangleUse : IEquatable<TriangleUse>
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

            public bool Equals(TriangleUse other) =>
                A == other.A && B == other.B && C == other.C;

            public override bool Equals(object? obj) =>
                obj is TriangleUse other && Equals(other);

            public override int GetHashCode() =>
                HashCode.Combine(A, B, C);
        }
    }
}
