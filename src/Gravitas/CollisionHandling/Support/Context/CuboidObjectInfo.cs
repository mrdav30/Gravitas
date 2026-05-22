using FixedMathSharp;
using Gravitas.Colliders;
using SwiftCollections;

namespace Gravitas.CollisionHandling;

public class CuboidObjectInfo : CollisionObjectInfo
{
    public LSCuboidCollider Collider;

    public CuboidObjectInfo(LSCuboidCollider collider, Vector3d poc) : base()
    {
        Collider = collider;
        PointOfContact = poc;
    }

    protected override void OnPrepareVertices(ref SwiftHashSet<Vector3d> axisSet)
    {
        for (int i = 0; i < Collider.FaceNormals.Length; i++)
        {
            if (!axisSet.Contains(Collider.FaceNormals[i]))
                axisSet.Add(Collider.FaceNormals[i]);
        }

        for (int i = 0; i < Collider.Vertices.Length; i++)
            UniqueVertices.Add(Collider.Vertices[i]);
    }
}
