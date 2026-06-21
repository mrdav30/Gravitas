//=======================================================================
// CollisionObjectInfo.cs
//=======================================================================
// MIT License, Copyright (c) 2026–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp;
using SwiftCollections;

namespace Gravitas.CollisionHandling;

internal abstract class CollisionObjectInfo
{
    public Vector3d PointOfContact;
    public SwiftList<Vector3d> UniqueVertices;

    protected CollisionObjectInfo(int vertexCapacity = 16) =>
        UniqueVertices = new SwiftList<Vector3d>(vertexCapacity);

    public void PrepareVertices(SwiftHashSet<Vector3d> axisSet)
    {
        UniqueVertices.FastClear();
        OnPrepareVertices(axisSet);
    }

    protected virtual void OnPrepareVertices(SwiftHashSet<Vector3d> axisSet) { }
}
