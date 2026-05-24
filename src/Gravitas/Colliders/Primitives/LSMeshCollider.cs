using FixedMathSharp;
using Gravitas.Raycasting;
using SwiftCollections;
using SwiftCollections.Query;

namespace Gravitas.Colliders;

public class LSMeshCollider : LSCollider
{
    public override ColliderType Shape => ColliderType.Mesh;

    public override int Priority => ColliderSettings.GetPriority(Shape);

    public override Fixed64 Area => Mesh != null ? Mesh.TotalArea : Fixed64.Zero;

    public PhysicsMesh Mesh { get; private set; }

    public LSMeshCollider(Vector3d[] vertices, int[] triangles)
    {
        Mesh = new PhysicsMesh(vertices, triangles, Vector3d.Zero, FixedQuaternion.Identity);
        _offset = Mesh.LocalBounds.Center;
        _size = Mesh.LocalBounds.Proportions;
        _radius = _size.Magnitude * Fixed64.Half;
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

    public SwiftList<int> GetNearbyTriangles(Vector3d queryPoint)
    {
        FixedBoundVolume queryBounds = CreateQueryBounds(queryPoint, Bounds.Scope.x);
        SwiftList<int> result = new();
        Mesh.TriangleBVH.Query(queryBounds, result);
        return result;
    }

    public override Vector3d ClosestPointOnSurface(Vector3d queryPoint)
    {
        SwiftList<int> result = new();
        Vector3d boundedPoint = Bounds.ClosestPointOnSurface(queryPoint);
        FixedBoundVolume queryBounds = CreateQueryBounds(boundedPoint, Bounds.Scope.x);
        Mesh.TriangleBVH.Query(queryBounds, result);
        Vector3d direction = (queryBounds.Center - Center).Normal;
        Vector3d closest = ClosestPointToTriangles(result, direction, queryBounds.Center);
        return closest;
    }

    private static FixedBoundVolume CreateQueryBounds(Vector3d center, Fixed64 halfExtent)
    {
        Vector3d extents = Vector3d.One * halfExtent;
        return new FixedBoundVolume(center - extents, center + extents);
    }

    private Vector3d ClosestPointToTriangles(SwiftList<int> indices, Vector3d direction, Vector3d point)
    {
        Fixed64 minDistance = Fixed64.MAX_VALUE;
        Vector3d closest = point;

        for (int i = 0; i < indices.Count; i++)
        {
            int index = indices[i]; // index of the triangle
            if (Vector3d.Dot(Mesh.FaceNormals[index], direction) <= Fixed64.Zero)
                continue;

            Vector3d[] triangle = new Vector3d[3]
            {
                    Mesh.Vertices[Mesh.Triangles[index * 3]],
                    Mesh.Vertices[Mesh.Triangles[index * 3 + 1]],
                    Mesh.Vertices[Mesh.Triangles[index * 3 + 2]],
            };

            Vector3d pointOnTriangle = MeshUtils.ClosestPointOnTriangle(triangle, Mesh.FaceNormals[index], point);
            Fixed64 distance = Vector3d.SqrDistance(point, pointOnTriangle);
            if (distance < minDistance)
            {
                minDistance = distance;
                closest = pointOnTriangle;
                if (minDistance < Fixed64.Epsilon)
                    break;
            }
        }

        return closest;

        //should we return only the triangles that include the closest point ?
        //if (outputTriangles != null && output.HasValue)
        //{
        //    for (int i = 0; triangles.Count > i; i++)
        //    {
        //        if (triangles[i].IsPointConnectedToTriangle(output.Value))
        //            outputTriangles.Add(triangles[i]);
        //    }
        //}
    }

    public override Vector3d GetNormalAtPoint(Vector3d point) =>
        ClosestPointOnSurface(point).Normal;

    public Vector3d GetSupportPoint(Vector3d point)
    {
        Vector3d closestPoint = Bounds.ClosestPointOnSurface(point);
        Fixed64 minDistance = Fixed64.MAX_VALUE;
        Vector3d nearestPoint = closestPoint;

        // Iterate through all vertices of the mesh to find the closest point
        for (int i = 0; i < Mesh.Vertices.Length; i++)
        {
            Fixed64 distance = Vector3d.SqrDistance(Mesh.Vertices[i], point);
            if (distance >= minDistance) continue;

            minDistance = distance;
            nearestPoint = Mesh.Vertices[i];
        }

        return nearestPoint;
    }

    public override bool ColliderOverlapsRay(RaycastSegmentWorker worker, ref SwiftList<Vector3d> outputIntersectionPoints)
    {
        return false;
    }
}
