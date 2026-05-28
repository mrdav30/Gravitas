using FixedMathSharp;
using Gravitas.Raycasting;
using SwiftCollections;

namespace Gravitas.Colliders;

public enum CuboidState
{
    AABox,
    OOBox
}

public class LSCuboidCollider : LSCollider
{
    // Define faces with vertex indices
    public static readonly int[][] FaceDefinitions = new int[][]
    {
            new[] {0, 1, 3, 2}, // near quad
            new[] {4, 6, 7, 5}, // far quad
            new[] {2, 3, 7, 6}, // top quad
            new[] {4, 5, 1, 0}, // bottom quad
            new[] {0, 2, 6, 4}, // left quad
            new[] {5, 7, 3, 1}  // right quad
    };

    // Define edges with vertex indices
    public static readonly int[][] EdgeDefinitions = new int[][]
    {
            new[] {0, 1}, new[] {2, 3}, new[] {0, 2}, new[] {1, 3},  // near quad
            new[] {4, 5}, new[] {6, 7}, new[] {4, 6}, new[] {5, 7},  // far quad
            new[] {0, 4}, new[] {1, 5}, new[] {2, 6}, new[] {3, 7}   // lines connecting the quads
    };

    private Vector3d[] _vertices = null!;
    /// <summary>
    /// The vertices of the cuboid.
    /// </summary>
    public Vector3d[] Vertices => _vertices;

    protected BoundingBox _orientedBounds;

    protected int[][] _faceVertices = null!;
    /// <summary>
    /// The face definitions of the cuboid, represented by indices of vertices.
    /// </summary>
    public int[][] Faces => _faceVertices;

    protected Vector3d[] _faceNormals = null!;
    /// <summary>
    /// The normal vectors for each face of the cuboid.
    /// </summary>
    public Vector3d[] FaceNormals => _faceNormals;

    protected Vector3d[] _faceCentroids = null!;
    /// <summary>
    /// The centroids of each face of the cuboid.
    /// </summary>
    public Vector3d[] FaceCentroids => _faceCentroids;

    protected int[][] _edgeVertices = null!;
    /// <summary>
    /// The edge definitions of the cuboid, represented by pairs of vertex indices.
    /// </summary>
    public int[][] EdgeVertices => _edgeVertices;

    protected Vector3d[] _edgeDirections = null!;
    /// <summary>
    /// A collection of normalized edge displacement vectors.
    /// Note, the displacement is a vector that points from the start of the edge (line segment) to the end.
    /// </summary>
    public Vector3d[] EdgeDirections => _edgeDirections;

    /// <summary>
    /// Stores the length of the cuboid along the X axis.
    /// </summary>
    protected Fixed64 _xAxisLength;

    /// <summary>
    /// Stores the length of the cuboid along the Y axis.
    /// </summary>
    protected Fixed64 _yAxisLength;

    /// <summary>
    /// Stores the length of the cuboid along the Z axis.
    /// </summary>
    protected Fixed64 _zAxisLength;

    /// <summary>
    /// Stores the direction vector along the X axis of the cuboid.
    /// </summary>
    protected Vector3d _xAxisDirectionVector;

    /// <summary>
    /// Stores the direction vector along the Y axis of the cuboid.
    /// </summary>
    protected Vector3d _yAxisDirectionVector;

    /// <summary>
    /// Stores the direction vector along the Z axis of the cuboid.
    /// </summary>
    protected Vector3d _zAxisDirectionVector;

    public CuboidState CurrentState => Rotation == FixedQuaternion.Identity ? CuboidState.AABox : CuboidState.OOBox;

    public override ColliderType Shape => CurrentState == CuboidState.AABox ? ColliderType.AABox : ColliderType.OBBox;

    public override int Priority => ColliderSettings.GetPriority(Shape);

    public override Fixed64 ScaledRadius => ScaledSize.Magnitude * Fixed64.Half;

    public LSCuboidCollider()
    {
        _vertices = new Vector3d[8];
        _faceVertices = new int[FaceDefinitions.Length][];
        _faceNormals = new Vector3d[FaceDefinitions.Length];
        _faceCentroids = new Vector3d[FaceDefinitions.Length];
        _edgeVertices = new int[EdgeDefinitions.Length][];
        _edgeDirections = new Vector3d[EdgeDefinitions.Length];
        _orientedBounds = new BoundingBox(Vector3d.Zero, Vector3d.One);
    }

    protected override void OnInitialize()
    {
        _orientedBounds = new BoundingBox(Center, ScaledSize);
        base.OnInitialize();
    }

    protected override void BuildShape()
    {
        GenerateVertices();
        GenerateAxes();
        GenerateFaces();
        CalculateFaceNormals();
        CalculateFaceCentroids();
        GenerateEdges();
        CalculateEdgeDirections();
        GenerateArea();
    }

    protected void GenerateVertices()
    {
        if (Rotation != FixedQuaternion.Identity)
            _orientedBounds.Orient(Center, ScaledSize);

        for (int i = 0; i < _bounds.Vertices.Length; i++)
        {
            if (CurrentState != CuboidState.AABox)
            {
                _vertices[i] = _orientedBounds.Vertices[i].Rotate(_bounds.Center, Rotation);
                continue;
            }

            _vertices[i] = _bounds.Vertices[i];
        }
    }

    protected void GenerateAxes()
    {
        _xAxisLength = (Vertices[1] - Vertices[0]).Magnitude;
        _yAxisLength = (Vertices[2] - Vertices[0]).Magnitude;
        _zAxisLength = (Vertices[4] - Vertices[0]).Magnitude;

        _xAxisDirectionVector = (Vertices[1] - Vertices[0]) / _xAxisLength;
        _yAxisDirectionVector = (Vertices[2] - Vertices[0]) / _yAxisLength;
        _zAxisDirectionVector = (Vertices[4] - Vertices[0]) / _zAxisLength;
    }

    protected virtual void GenerateFaces()
    {
        for (int i = 0; i < FaceDefinitions.Length; i++)
        {
            if (_faceVertices[i] == null)
                _faceVertices[i] = FaceDefinitions[i];
        }
    }

    // Method to get a face by index
    public Vector3d[] GetFace(int index)
    {
        int[] faceIndices = _faceVertices[index];
        Vector3d[] faceVertices = new Vector3d[faceIndices.Length];

        for (int i = 0; i < faceIndices.Length; i++)
            faceVertices[i] = Vertices[faceIndices[i]];

        return faceVertices;
    }

    protected virtual void CalculateFaceNormals()
    {
        for (int i = 0; i < _faceVertices.Length; i++)
        {
            int[] faceVertices = _faceVertices[i];
            Vector3d first = Vertices[faceVertices[0]];

            // Get two non-parallel edges
            Vector3d edge1 = Vertices[faceVertices[1]] - first;
            Vector3d edge2 = Vertices[faceVertices[2]] - first;

            // Calculate the normal using cross product
            _faceNormals[i] = Vector3d.Cross(edge1, edge2).Normal;
        }
    }

    protected virtual void CalculateFaceCentroids()
    {
        for (int i = 0; i < _faceVertices.Length; i++)
        {
            Vector3d result = Vector3d.Zero;
            int[] faceIndices = _faceVertices[i];
            for (int j = 0; j < faceIndices.Length; j++)
                result += Vertices[faceIndices[j]];

            // Average the sum of vertices
            _faceCentroids[i] = result / faceIndices.Length;
        }
    }

    protected virtual void GenerateEdges()
    {
        for (int i = 0; i < EdgeDefinitions.Length; i++)
        {
            if (_edgeVertices[i] == null)
                _edgeVertices[i] = EdgeDefinitions[i];
        }
    }

    // Method to get an edge by index
    public (Vector3d start, Vector3d end) GetEdge(int index)
    {
        int[] edgeIndices = _edgeVertices[index];
        return (Vertices[edgeIndices[0]], Vertices[edgeIndices[1]]);
    }

    protected virtual void CalculateEdgeDirections()
    {
        for (int i = 0; i < _edgeDirections.Length; i++)
            _edgeDirections[i] = GetEdgeDirection(i);
    }

    // Method to calculate displacement of an edge
    public Vector3d GetEdgeDisplacement(int index)
    {
        var (start, end) = GetEdge(index);
        return end - start;
    }

    // Method to calculate direction of an edge
    public Vector3d GetEdgeDirection(int index) =>
         GetEdgeDisplacement(index).Normal;

    protected virtual void GenerateArea() =>
        // Area calculation: A = 2lw + 2lh + 2wh
        Area = 2 * ScaledSize.x * ScaledSize.z + 2 * ScaledSize.x * ScaledSize.y + 2 * ScaledSize.y * ScaledSize.z;

    public override Fixed3x3 CalculateInertiaTensor(Fixed64 mass)
    {
        Vector3d worldScaleSqr = ScaledSize * ScaledSize;

        // For a solid box, the inertia tensor is (m/12)*(h^2 + d^2), (m/12)*(w^2 + d^2), (m/12)*(w^2 + h^2) for the diagonal elements
        Fixed64 xx = (mass / (Fixed64)12) * (worldScaleSqr.y + worldScaleSqr.z);
        Fixed64 yy = (mass / (Fixed64)12) * (worldScaleSqr.x + worldScaleSqr.z);
        Fixed64 zz = (mass / (Fixed64)12) * (worldScaleSqr.x + worldScaleSqr.y);

        return new Fixed3x3(
            xx, Fixed64.Zero, Fixed64.Zero,
            Fixed64.Zero, yy, Fixed64.Zero,
            Fixed64.Zero, Fixed64.Zero, zz
        );
    }

    public override Fixed64 GetFrontalArea(Vector3d direction)
    {
        // Normalize the direction vector to get only the direction information
        direction.Normalize();

        // Get the absolute dot products of the direction with the local axes
        Fixed64 dotX = Vector3d.Dot(direction, _xAxisDirectionVector).Abs();
        Fixed64 dotY = Vector3d.Dot(direction, _yAxisDirectionVector).Abs();
        Fixed64 dotZ = Vector3d.Dot(direction, _zAxisDirectionVector).Abs();

        // Determine the two dimensions of the box that contribute to the frontal area
        Fixed64 area;
        if (dotX < dotY)
        {
            if (dotX < dotZ)
                // The x-axis of the box is most aligned with the direction, so the y and z axes contribute to the frontal area
                area = _yAxisLength * _zAxisLength;
            else
                // The z-axis of the box is most aligned with the direction, so the x and y axes contribute to the frontal area
                area = _xAxisLength * _yAxisLength;
        }
        else
        {
            if (dotY < dotZ)
                // The y-axis of the box is most aligned with the direction, so the x and z axes contribute to the frontal area
                area = _xAxisLength * _zAxisLength;
            else
                // The z-axis of the box is most aligned with the direction, so the x and y axes contribute to the frontal area
                area = _xAxisLength * _yAxisLength;
        }

        return area;
    }

    protected Vector3d ClosestPointOnFace(int[] faceVertices, Vector3d faceNormal, Vector3d faceCentroid, Vector3d point)
    {
        Vector3d projectedPoint = ProjectPointOntoPlane(faceNormal, faceCentroid, point);

        // Check if the projected point lies within the face
        if (IsPointInFacePlane(faceVertices, faceNormal, projectedPoint))
            return projectedPoint;

        // Find the closest point on the edges or vertices
        Vector3d closestPoint = Vertices[faceVertices[0]];
        Fixed64 minDistanceSquared = (projectedPoint - closestPoint).SqrMagnitude;

        for (int i = 0; i < faceVertices.Length; i++)
        {
            Vector3d start = Vertices[faceVertices[i]];
            Vector3d end = Vertices[faceVertices[(i + 1) % faceVertices.Length]];
            Vector3d closestPointOnEdge = ClosestPointOnLineSegment(start, end, projectedPoint);
            Fixed64 distanceSquared = (projectedPoint - closestPointOnEdge).SqrMagnitude;

            if (distanceSquared < minDistanceSquared)
            {
                minDistanceSquared = distanceSquared;
                closestPoint = closestPointOnEdge;
            }
        }

        return closestPoint;
    }

    public override Vector3d ClosestPointOnSurface(Vector3d other)
    {
        if (CurrentState == CuboidState.AABox)
            return Bounds.ClosestPointOnSurface(other);

        Fixed64 minDistance = Fixed64.MAX_VALUE;
        Vector3d closestPoint = Center;

        // Find closest point on faces
        for (int i = 0; i < _faceVertices.Length; i++)
        {
            // Calculate the closest point on the current face
            Vector3d facePoint = ClosestPointOnFace(_faceVertices[i], _faceNormals[i], _faceCentroids[i], other);
            Fixed64 distance = Vector3d.SqrDistance(facePoint, other);

            if (distance < minDistance)
            {
                minDistance = distance;
                closestPoint = facePoint;
            }
        }

        return closestPoint;
    }

    public override Vector3d GetNormalAtPoint(Vector3d point)
    {
        if (CurrentState == CuboidState.AABox)
        {
            Vector3d localToCenter = point - Center;
            Vector3d absLocal = Vector3d.Abs(localToCenter);

            if (absLocal.x > absLocal.y && absLocal.x > absLocal.z)
                return new Vector3d(localToCenter.x > Fixed64.Zero ? Fixed64.One : -Fixed64.One, Fixed64.Zero, Fixed64.Zero);

            if (absLocal.y > absLocal.z)
                return new Vector3d(Fixed64.Zero, localToCenter.y > Fixed64.Zero ? Fixed64.One : -Fixed64.One, Fixed64.Zero);

            return new Vector3d(Fixed64.Zero, Fixed64.Zero, localToCenter.z > Fixed64.Zero ? Fixed64.One : -Fixed64.One);
        }

        // Transform the point to local space
        Vector3d localPoint = (point - Position).InverseRotate(Position, Rotation);

        // Get the local normal as if it's an AABB
        Vector3d localNormal = Vector3d.Abs(localPoint - Center) - Bounds.Scope;

        Vector3d sign = new((localPoint.x - Center.x).Sign(), (localPoint.y - Center.y).Sign(), (localPoint.z - Center.z).Sign());

        // Find the component of localNormal with the greatest absolute value
        if (localNormal.x > localNormal.y)
        {
            if (localNormal.x > localNormal.z)
                localNormal = new Vector3d(sign.x, Fixed64.Zero, Fixed64.Zero);
            else
                localNormal = new Vector3d(Fixed64.Zero, Fixed64.Zero, sign.z);
        }
        else
        {
            if (localNormal.y > localNormal.z)
                localNormal = new Vector3d(Fixed64.Zero, sign.y, Fixed64.Zero);
            else
                localNormal = new Vector3d(Fixed64.Zero, Fixed64.Zero, sign.z);
        }

        // Transform the normal back to world space
        return localNormal.Rotate(Position, Rotation);
    }

    protected Vector3d ProjectPointOntoPlane(Vector3d faceNormal, Vector3d faceCentroid, Vector3d point)
    {
        // Calculate the distance from the point to the plane along the normal
        Fixed64 distance = Vector3d.Dot(faceNormal, point - faceCentroid);
        // Project the point onto the plane
        return point - faceNormal * distance;
    }

    protected bool IsPointInFacePlane(int[] faceVertices, Vector3d faceNorm, Vector3d point)
    {
        Vector3d v0 = Vertices[faceVertices[0]];
        Vector3d edge0 = Vertices[faceVertices[1]] - v0;
        Vector3d edge0_normal = Vector3d.Cross(faceNorm, edge0);
        Fixed64 dot0 = Vector3d.Dot(edge0_normal, point - v0);

        for (int i = 1; i < faceVertices.Length; i++)
        {
            Vector3d vi = Vertices[faceVertices[i]];
            Vector3d edgei = Vertices[faceVertices[(i + 1) % faceVertices.Length]] - vi;
            Vector3d edgei_normal = Vector3d.Cross(faceNorm, edgei);

            if (Vector3d.Dot(edgei_normal, point - vi) * dot0 < Fixed64.Zero)
                return false;
        }

        return true;
    }

    protected Vector3d ClosestPointOnLineSegment(Vector3d start, Vector3d end, Vector3d point)
    {
        Vector3d direction = end - start;
        Fixed64 lengthSquared = direction.SqrMagnitude;
        if (lengthSquared == Fixed64.Zero) return start;

        Fixed64 t = Vector3d.Dot(point - start, direction) / lengthSquared;
        t = FixedMath.Clamp01(t);
        return start + t * direction;
    }

    public override bool ColliderOverlapsRay(RaycastSegmentWorker worker, ref SwiftList<Vector3d> outputIntersectionPoints)
    {
        if (CurrentState == CuboidState.AABox)
            return worker.CheckAABBoxOverlaps(this, ref outputIntersectionPoints);

        return worker.CheckOBBoxOverlaps(this, ref outputIntersectionPoints);
    }
}
