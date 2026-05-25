using FixedMathSharp;
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

        // Holds all the vertices in local space that make up the mesh
        private Vector3d[] _localVertices;

        private Vector3d[] _vertices;
        /// <summary>
        /// Holds all the vertices transformed to world space
        /// </summary>
        public Vector3d[] Vertices => _vertices;

        private int[] _triangles;
        /// <summary>
        /// Holds all the triangles that make up the mesh in the form of indices to the vertices array
        /// example: Triangles[0-3] = 4,2,3 means that the first triangle of the mesh is made up of the vertices _vertices[4], _vertices[2], _vertices[3]
        /// </summary>
        public int[] Triangles => _triangles;

        private int _triangleCount;
        public int TriangleCount => _triangleCount;

        // Holds all the normals for the triangles, which are tied to the triangles array at the same index
        // example: Normals[0] = (0, 1, 0) means that the normal of Triangles[0-3] is (0, 1, 0)
        private bool _faceNormalsValid;
        private readonly Vector3d[] _faceNormals;
        public Vector3d[] FaceNormals => !_faceNormalsValid
            ? CalculateFaceNormals()
            : _faceNormals;

        // Holds all the edges that make up a triangle in the form of indices to the vertices array
        // example: _edges[0] = [0,1,1,2,2,0] means that these vertices for the first triangle of the mesh make up the edges _vertices[0]-_vertices[1], _vertices[1]-_vertices[2], _vertices[2]-_vertices[0]
        private int[][] _edges;

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

        private bool _triangleBVHValid;
        private SwiftFixedBVH<int> _triangleBVH;
        public SwiftFixedBVH<int> TriangleBVH => !_triangleBVHValid
            ? UpdateTriangleBVH()
            : _triangleBVH;

        public Fixed4x4 _transformationMatrix;
        public Fixed4x4 TransformationMatrix
        {
            get => _transformationMatrix;
            private set
            {
                _transformationMatrix = value;

                // Invalidate all cached data that depends on the position of the vertices in world space
                _inverseMatrixValid = false;
                _edgesNormalsValid = false;
                _faceNormalsValid = false;
                _triangleBVHValid = false;
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

        private BoundingBox _bounds;
        public BoundingBox Bounds => _bounds;

        private BoundingBox _localBounds;

        /// <summary>
        /// Axis-aligned bounds of the source vertices in local mesh space.
        /// </summary>
        public BoundingBox LocalBounds => _localBounds;

        public PhysicsMesh(Vector3d[] vertices, int[] triangles, Vector3d position, FixedQuaternion rotation)
        {
            ValidateInput(vertices, triangles);

            _localVertices = new Vector3d[vertices.Length];
            Array.Copy(vertices, _localVertices, vertices.Length);
            _vertices = new Vector3d[vertices.Length];
            _triangles = new int[triangles.Length];
            Array.Copy(triangles, _triangles, triangles.Length);
            _triangleCount = triangles.Length / 3; // 3 vertices per triangle
            _triangleBVH = new SwiftFixedBVH<int>(2 * TriangleCount - 1);
            _faceNormals = new Vector3d[TriangleCount];
            _edges = new int[TriangleCount][];
            _edgeNormals = new Vector3d[TriangleCount * 3]; // 3 edges per triangle

            // unless the mesh changes, the total area & areas of the triangles will never change
            _faceAreas = new Fixed64[TriangleCount];
            _totalArea = Fixed64.Zero;

            _localBounds = CalculateBounds(_localVertices);

            UpdateTransformationMatrix(position, rotation);

            for (int i = 0; i < _triangleCount; i++)
            {
                int index0 = _triangles[i * 3];
                int index1 = _triangles[i * 3 + 1];
                int index2 = _triangles[i * 3 + 2];

                // Store the edges of the triangle for querying normals, and the normals of the edges
                _edges[i] = new int[] { index0, index1, index1, index2, index2, index0 };
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
            UpdateTriangleBVH();
            CalculateFaceNormals();
            UpdateBounds();
        }

        private void UpdateTransformationMatrix(Vector3d position, FixedQuaternion rotation)
        {
            TransformationMatrix = Fixed4x4.TranslateRotateScale(position, rotation, Vector3d.One);
            UpdateTransformedVertices();
        }

        private void UpdateTransformedVertices()
        {
            for (int i = 0; i < _localVertices.Length; i++)
                _vertices[i] = TransformationMatrix * _localVertices[i];
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

                ThrowIfIndexOutOfRange(index0, vertices.Length, nameof(triangles));
                ThrowIfIndexOutOfRange(index1, vertices.Length, nameof(triangles));
                ThrowIfIndexOutOfRange(index2, vertices.Length, nameof(triangles));

                SwiftThrowHelper.ThrowIfArgument(
                    index0 == index1 || index1 == index2 || index2 == index0,
                    nameof(triangles),
                    "Triangle indices must be unique within each triangle.");

                Fixed64 area = CalculateTriangleAreaStatic(vertices[index0], vertices[index1], vertices[index0], vertices[index2]);
                SwiftThrowHelper.ThrowIfArgument(area <= Fixed64.Epsilon, nameof(triangles), "Degenerate triangles are not supported.");
            }
        }

        private static void ThrowIfIndexOutOfRange(int index, int length, string paramName)
        {
            if ((uint)index >= (uint)length)
                throw new ArgumentOutOfRangeException(paramName, index, "Triangle index is outside the vertex array.");
        }

        // Calculate the area of the triangle using cross product
        private static Fixed64 CalculateTriangleAreaStatic(
            Vector3d startEdgeA,
            Vector3d endEdgeA,
            Vector3d startEdgeB,
            Vector3d endEdgeB) => Vector3d.Cross(endEdgeA - startEdgeA, endEdgeB - startEdgeB).Magnitude * Fixed64.Half;

        private Fixed64 CalculateTriangleArea(
            Vector3d startEdgeA,
            Vector3d endEdgeA,
            Vector3d startEdgeB,
            Vector3d endEdgeB) => CalculateTriangleAreaStatic(startEdgeA, endEdgeA, startEdgeB, endEdgeB);

        public Vector3d[] CalculateEdgeNormals()
        {
            for (int i = 0; i < _triangleCount; i++)
            {
                for (int n = 0; n < 3; n++)
                {
                    int edgeStart = _edges[i][n * 2];
                    int edgeEnd = _edges[i][n * 2 + 1];
                    _edgeNormals[i * 3 + n] = (_vertices[edgeEnd] - _vertices[edgeStart]).Normal;
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
                _faceNormals[i] = Vector3d.Cross(_vertices[index1] - _vertices[index0], _vertices[index2] - _vertices[index0]).Normal;
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
                Vector3d min = Vector3d.Min(Vector3d.Min(_vertices[index0], _vertices[index1]), _vertices[index2]);
                Vector3d max = Vector3d.Max(Vector3d.Max(_vertices[index0], _vertices[index1]), _vertices[index2]);
                // Store the starting index of the triangle for querying positions
                _triangleBVH.Insert(i, new FixedBoundVolume(min, max));
            }

            _triangleBVHValid = true;
            return _triangleBVH;
        }

        private void UpdateBounds()
        {
            _bounds = CalculateBounds(_vertices);
        }

        private static BoundingBox CalculateBounds(Vector3d[] vertices)
        {
            Vector3d min = vertices[0];
            Vector3d max = vertices[0];
            for (int i = 1; i < vertices.Length; i++)
            {
                min = Vector3d.Min(min, vertices[i]);
                max = Vector3d.Max(max, vertices[i]);
            }

            return new BoundingBox((min + max) * Fixed64.Half, max - min);
        }

        public Fixed3x3 CalculateInertiaTensor(Fixed64 mass)
        {
            Fixed3x3 tensor = Fixed3x3.Zero;
            if (_totalArea <= Fixed64.Zero)
                return tensor;

            for (int i = 0; i < _triangleCount; i++)
            {
                int index0 = _triangles[i * 3];
                int index1 = _triangles[i * 3 + 1];
                int index2 = _triangles[i * 3 + 2];
                Fixed3x3 triangleTensor = new(_vertices[index0], _vertices[index1], _vertices[index2]);
                Fixed64 triangleMass = mass * (_faceAreas[i] / _totalArea);
                triangleTensor *= triangleMass; // Adjust for the mass and volume of the triangle
                tensor += triangleTensor;
            }

            tensor /= _triangleCount;
            return tensor;
        }

        // make sure direction is normalized
        public Fixed64 GetFrontalArea(Vector3d direction)
        {
            Fixed64 totalArea = Fixed64.Zero;
            for (int i = 0; i < FaceNormals.Length; i++)
            {
                if (Vector3d.Dot(FaceNormals[i], direction) > Fixed64.Zero) // if the triangle faces the direction
                    totalArea += _faceAreas[i];
            }

            return totalArea;
        }

        public Vector3d[] GetTriangleAtIndex(int index)
        {
            GetTriangleVertices(index, out Vector3d first, out Vector3d second, out Vector3d third);

            return new Vector3d[3] {
                            first,
                            second,
                            third
                        };
        }

        public void GetTriangleVertices(int index, out Vector3d first, out Vector3d second, out Vector3d third)
        {
            SwiftThrowHelper.ThrowIfArrayIndexInvalid(index, _triangleCount, nameof(index));

            int triangleIndex = index * 3;
            first = _vertices[_triangles[triangleIndex]];
            second = _vertices[_triangles[triangleIndex + 1]];
            third = _vertices[_triangles[triangleIndex + 2]];
        }

        public Vector3d ConvertWorldToLocal(Vector3d worldPoint) =>
            InverseTransformationMatrix * worldPoint;
    }
}
