using FixedMathSharp;
using SwiftCollections;
using SwiftCollections.Pool;

namespace Gravitas.CollisionHandling;

public abstract class CollisionObjectInfo : ICollisionObjectInfo
{
    public Vector3d PointOfContact;
    public SwiftList<Vector3d> UniqueVertices;

    public CollisionObjectInfo() =>
        UniqueVertices = SwiftListPool<Vector3d>.Shared.Rent();

    public void PrepareVertices(ref SwiftHashSet<Vector3d> axisSet)
    {
        UniqueVertices.FastClear();
        OnPrepareVertices(ref axisSet);
    }

    protected virtual void OnPrepareVertices(ref SwiftHashSet<Vector3d> axisSet) { }

    public void Dispose()
    {
        if (UniqueVertices != null)
            SwiftListPool<Vector3d>.Shared.Release(UniqueVertices);
        OnDispose();
    }

    protected virtual void OnDispose() { }
}
