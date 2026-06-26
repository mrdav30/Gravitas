//=======================================================================
// MeshObjectInfo.cs
//=======================================================================
// MIT License, Copyright (c) 2026–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp;
using Gravitas.Colliders;
using SwiftCollections;

namespace Gravitas.CollisionHandling;

internal sealed class MeshObjectInfo : CollisionObjectInfo
{
    private readonly SwiftHashSet<int> _processedVertices = new(16);

    public LSMeshCollider Collider = null!;

    public SwiftList<int> TriangleIndices;

    public MeshObjectInfo() : base(16) =>
        TriangleIndices = new SwiftList<int>(8);

    public void Set(LSMeshCollider collider, Vector3d pointOfContact)
    {
        Collider = collider;
        PointOfContact = pointOfContact;
        UniqueVertices.FastClear();
        TriangleIndices.FastClear();
    }

    protected override void OnPrepareVertices(SwiftHashSet<Vector3d> axisSet)
    {
        _processedVertices.Clear();
        for (int i = 0; i < TriangleIndices.Count; i++)
        {
            int index = TriangleIndices[i];
            axisSet.Add(Collider.Mesh.GetFaceNormalWorld(index));

            for (int n = 0; n < 3; n++)
            {
                int vertexIndex = Collider.Mesh.GetTriangleVertexIndex(index, n);
                if (_processedVertices.Add(vertexIndex))
                    UniqueVertices.Add(Collider.Mesh.GetVertexWorld(vertexIndex));
            }
        }
    }
}
