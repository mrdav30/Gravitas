using FixedMathSharp;
using Gravitas.Colliders;
using SwiftCollections;

namespace Gravitas.CollisionHandling;

/// <summary>
/// Owns reusable SAT and mesh-query buffers for one world context.
/// </summary>
internal sealed class CollisionSatScratch
{
    public CollisionContext Context { get; } = new();

    public CuboidObjectInfo CuboidA { get; } = new();

    public CuboidObjectInfo CuboidB { get; } = new();

    public MeshObjectInfo MeshA { get; } = new();

    public MeshObjectInfo MeshB { get; } = new();

    public SwiftList<int> MeshCylinderTriangles { get; } = new(8);

    public SwiftList<int> MeshTriangleCandidatesA { get; } = new(16);

    public SwiftList<int> MeshTriangleCandidatesB { get; } = new(16);

    public CollisionContext PrepareCuboids(
        LSCuboidCollider cuboidA,
        Vector3d pointA,
        LSCuboidCollider cuboidB,
        Vector3d pointB)
    {
        CuboidA.Set(cuboidA, pointA);
        CuboidB.Set(cuboidB, pointB);
        Context.Prepare(CuboidA, CuboidB);
        return Context;
    }

    public bool TryPrepareMeshCuboid(
        LSMeshCollider mesh,
        Vector3d pointOnMesh,
        LSCuboidCollider cuboid,
        Vector3d pointOnCuboid,
        out CollisionContext context)
    {
        MeshA.Set(mesh, pointOnMesh);
        mesh.GetNearbyTriangles(pointOnMesh, MeshA.TriangleIndices);
        if (MeshA.TriangleIndices.Count <= 0)
        {
            context = null!;
            return false;
        }

        CuboidA.Set(cuboid, pointOnCuboid);
        Context.Prepare(MeshA, CuboidA);
        context = Context;
        return true;
    }

    public bool TryPrepareMeshes(
        LSMeshCollider meshA,
        Vector3d pointA,
        LSMeshCollider meshB,
        Vector3d pointB,
        out CollisionContext context)
    {
        MeshA.Set(meshA, pointA);
        meshA.GetNearbyTriangles(pointA, MeshA.TriangleIndices);
        if (MeshA.TriangleIndices.Count <= 0)
        {
            context = null!;
            return false;
        }

        MeshB.Set(meshB, pointB);
        meshB.GetNearbyTriangles(pointB, MeshB.TriangleIndices);
        if (MeshB.TriangleIndices.Count <= 0)
        {
            context = null!;
            return false;
        }

        Context.Prepare(MeshA, MeshB);
        context = Context;
        return true;
    }
}
