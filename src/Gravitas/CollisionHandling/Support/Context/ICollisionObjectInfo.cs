using FixedMathSharp;
using SwiftCollections;
using System;

namespace Gravitas.CollisionHandling;

public interface ICollisionObjectInfo : IDisposable
{
    void PrepareVertices(ref SwiftHashSet<Vector3d> axisSet);
}
