using FixedMathSharp;
using Gravitas.Raycasting;
using SwiftCollections;
using SwiftCollections.Query;

namespace Gravitas.Colliders;

public class LSMeshCollider : LSCollider
{
    private readonly SwiftList<int> _triangleQueryBuffer = new();

    public override ColliderType Shape => ColliderType.Mesh;

    public override int Priority => ColliderSettings.GetPriority(Shape);

    public override Fixed64 Area => Mesh != null ? Mesh.TotalArea : Fixed64.Zero;

    public PhysicsMesh Mesh { get; private set; }

    public MeshColliderMode Mode => Mesh.Mode;

    public LSMeshCollider(Vector3d[] vertices, int[] triangles)
        : this(vertices, triangles, MeshColliderMode.Convex)
    {
    }

    public LSMeshCollider(Vector3d[] vertices, int[] triangles, MeshColliderMode mode)
    {
        Mesh = new PhysicsMesh(vertices, triangles, Vector3d.Zero, FixedQuaternion.Identity, mode);
        _offset = Mesh.LocalBounds.Center;
        _size = Mesh.LocalBounds.Proportions;
        _radius = _size.Magnitude * Fixed64.Half;
        SetBounds(Mesh.Bounds);
    }

    protected override void OnBeforeInitialize(IMatterAgent agent)
    {
        SwiftThrowHelper.ThrowIfTrue(
            MeshColliderPolicy.RequiresConvexDecomposition(Mode, Body),
            nameof(LSMeshCollider),
            "Explicit concave mesh colliders cannot be initialized as dynamic bodies. Use a convex mesh, kinematic/static concave mesh, or a decomposed convex/compound representation.");
    }

    protected override void OnInitialize()
    {
        base.OnInitialize();
    }

    public override Fixed64 ScaledRadius => Bounds.Scope.Magnitude;

    protected override void RebuildRuntimeShape()
    {
        BuildShape();
        BuildBoundingBox();
    }

    protected override void BuildBoundingBox() =>
        SetBounds(Mesh.Bounds);

    protected override void BuildShape() =>
         Mesh.UpdatePosition(Position, Rotation);

    public override Fixed3x3 CalculateInertiaTensor(Fixed64 mass) =>
        Mesh.CalculateInertiaTensor(mass);

    public override Fixed64 GetFrontalArea(Vector3d direction) =>
        Mesh.GetFrontalArea(direction);

    public void GetNearbyTriangles(Vector3d queryPoint, SwiftList<int> result)
    {
        FixedBoundVolume queryBounds = CreateQueryBounds(queryPoint, GetMeshQueryHalfExtent());
        GetTrianglesInBounds(queryBounds, result);
    }

    public void GetTrianglesInBounds(FixedBoundVolume queryBounds, SwiftList<int> result)
    {
        Mesh.GetTrianglesInWorldBounds(queryBounds, result);
    }

    public override Vector3d ClosestPointOnSurface(Vector3d queryPoint)
    {
        return TryFindClosestPointOnSurface(queryPoint, _triangleQueryBuffer, out Vector3d closest, out _)
            ? closest
            : Bounds.ClosestPointOnSurface(queryPoint);
    }

    public bool TryFindClosestPointOnSurface(
        Vector3d queryPoint,
        SwiftList<int> triangleBuffer,
        out Vector3d closest,
        out Vector3d normal)
    {
        Vector3d boundedPoint = Bounds.ClosestPointOnSurface(queryPoint);
        FixedBoundVolume queryBounds = CreateQueryBounds(boundedPoint, GetMeshQueryHalfExtent());
        Mesh.GetTrianglesInWorldBounds(queryBounds, triangleBuffer);
        return TryFindClosestPointToTriangles(triangleBuffer, queryPoint, out closest, out normal);
    }

    private Fixed64 GetMeshQueryHalfExtent() =>
        FixedMath.Max(FixedMath.Max(Bounds.Scope.x, Bounds.Scope.y), Bounds.Scope.z);

    private static FixedBoundVolume CreateQueryBounds(Vector3d center, Fixed64 halfExtent)
    {
        Vector3d extents = Vector3d.One * halfExtent;
        return new FixedBoundVolume(center - extents, center + extents);
    }

    private bool TryFindClosestPointToTriangles(
        SwiftList<int> indices,
        Vector3d point,
        out Vector3d closest,
        out Vector3d normal)
    {
        Fixed64 minDistance = Fixed64.MAX_VALUE;
        closest = point;
        normal = Vector3d.Zero;
        bool found = false;

        for (int i = 0; i < indices.Count; i++)
        {
            int index = indices[i]; // index of the triangle
            Mesh.GetTriangleVertices(index, out Vector3d first, out Vector3d second, out Vector3d third);
            Vector3d faceNormal = Mesh.GetFaceNormalWorld(index);
            Vector3d pointOnTriangle = MeshUtils.ClosestPointOnTriangle(first, second, third, faceNormal, point);
            Fixed64 distance = Vector3d.SqrDistance(point, pointOnTriangle);
            if (distance < minDistance)
            {
                minDistance = distance;
                closest = pointOnTriangle;
                normal = OrientNormalTowardPoint(faceNormal, point - pointOnTriangle);
                found = true;
                if (minDistance < Fixed64.Epsilon)
                    break;
            }
        }

        return found;
    }

    public override Vector3d GetNormalAtPoint(Vector3d point) =>
        TryFindClosestPointOnSurface(point, _triangleQueryBuffer, out _, out Vector3d normal)
            ? normal
            : (point - Center).Normal;

    public Vector3d GetSupportPoint(Vector3d point)
    {
        Vector3d closestPoint = Bounds.ClosestPointOnSurface(point);
        Fixed64 minDistance = Fixed64.MAX_VALUE;
        Vector3d nearestPoint = closestPoint;

        // Iterate through all vertices of the mesh to find the closest point
        for (int i = 0; i < Mesh.VertexCount; i++)
        {
            Vector3d worldVertex = Mesh.GetVertexWorld(i);
            Fixed64 distance = Vector3d.SqrDistance(worldVertex, point);
            if (distance >= minDistance) continue;

            minDistance = distance;
            nearestPoint = worldVertex;
        }

        return nearestPoint;
    }

    public override bool ColliderOverlapsRay(RaycastSegmentWorker worker, ref SwiftList<Vector3d> outputIntersectionPoints)
    {
        return worker.CheckMeshOverlaps(this, ref outputIntersectionPoints);
    }

    private static Vector3d OrientNormalTowardPoint(Vector3d normal, Vector3d targetDirection)
    {
        if (targetDirection.SqrMagnitude <= Fixed64.Epsilon)
            return normal;

        return Vector3d.Dot(normal, targetDirection) < Fixed64.Zero ? -normal : normal;
    }
}
