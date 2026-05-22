using FixedMathSharp;
using Gravitas.Colliders;
using SwiftCollections;
using SwiftCollections.Pool;

namespace Gravitas.CollisionHandling;

public class MeshObjectInfo : CollisionObjectInfo
{
    public LSMeshCollider Collider;

    public SwiftList<int> TriangleIndices;

    public MeshObjectInfo(LSMeshCollider collider, Vector3d poc) : base()
    {
        Collider = collider;
        PointOfContact = poc;
        TriangleIndices = new SwiftList<int>(collider.Mesh.Triangles);
    }

    // Pass in filtered triangle indices to avoid unnecessary processing.
    public MeshObjectInfo(LSMeshCollider collider, Vector3d poc, SwiftList<int> triangleIndices) : base()
    {
        Collider = collider;
        PointOfContact = poc;
        TriangleIndices = triangleIndices;
    }

    protected override void OnPrepareVertices(ref SwiftHashSet<Vector3d> axisSet)
    {
        SwiftHashSet<int> processedVertices = SwiftHashSetPool<int>.Shared.Rent();
        for (int i = 0; i < TriangleIndices.Count; i++)
        {
            int index = TriangleIndices[i];
            if (!axisSet.Contains(Collider.Mesh.FaceNormals[index]))
                axisSet.Add(Collider.Mesh.FaceNormals[index]);

            for (int n = 0; n < 3; n++)
            {
                int vertexIndex = Collider.Mesh.Triangles[index * 3 + n];
                if (processedVertices.Add(vertexIndex))
                    UniqueVertices.Add(Collider.Mesh.Vertices[vertexIndex]);
            }
        }

        SwiftHashSetPool<int>.Shared.Release(processedVertices);
    }
}
