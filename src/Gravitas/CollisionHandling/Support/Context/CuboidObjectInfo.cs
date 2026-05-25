using FixedMathSharp;
using Gravitas.Colliders;
using SwiftCollections;

namespace Gravitas.CollisionHandling;

internal sealed class CuboidObjectInfo : CollisionObjectInfo
{
    public LSCuboidCollider Collider = null!;

    public CuboidObjectInfo() : base(8) { }

    public void Set(LSCuboidCollider collider, Vector3d pointOfContact)
    {
        Collider = collider;
        PointOfContact = pointOfContact;
        UniqueVertices.FastClear();
    }

    protected override void OnPrepareVertices(SwiftHashSet<Vector3d> axisSet)
    {
        for (int i = 0; i < Collider.FaceNormals.Length; i++)
            axisSet.Add(Collider.FaceNormals[i]);

        for (int i = 0; i < Collider.Vertices.Length; i++)
            UniqueVertices.Add(Collider.Vertices[i]);
    }
}
