using FixedMathSharp;
using SwiftCollections;
using SwiftCollections.Pool;
using System;

namespace Gravitas.CollisionHandling;

public class CollisionContext : IDisposable
{
    private static readonly Fixed64 AngleThresholdDegrees = new(2); // 2 degrees
    private static readonly Fixed64 AngleThresholdRadians = FixedMath.DegToRad(AngleThresholdDegrees);
    private static readonly Fixed64 CosThreshold = FixedMath.Cos(AngleThresholdRadians);

    public CollisionObjectInfo CollisionInfoA { get; private set; }

    public CollisionObjectInfo CollisionInfoB { get; private set; }

    public Vector3d Displacement { get; private set; }

    public (Vector3d Point1, Vector3d Point2) PointsOfContact => (CollisionInfoA.PointOfContact, CollisionInfoB.PointOfContact);

    public SwiftHashSet<Vector3d> AxisVectors;

    private SwiftHashSet<Vector3d>? _potentialNewAxes;

    public CollisionContext(CollisionObjectInfo collisionInfoA, CollisionObjectInfo collisionInfoB)
    {
        AxisVectors ??= SwiftHashSetPool<Vector3d>.Shared.Rent();
        CollisionInfoA = collisionInfoA;
        CollisionInfoB = collisionInfoB;
    }

    public void PrepareDataForSAT()
    {
        //  Instead of adding to AxisVectors directly, use PotentialNewAxes to check uniqueness.
        _potentialNewAxes = SwiftHashSetPool<Vector3d>.Shared.Rent();
        CollisionInfoA.PrepareVertices(ref _potentialNewAxes);
        CollisionInfoB.PrepareVertices(ref _potentialNewAxes);
        ProcessAndAddAxes();
        Displacement = CollisionInfoB.PointOfContact - CollisionInfoA.PointOfContact;

        SwiftHashSetPool<Vector3d>.Shared.Release(_potentialNewAxes);
        _potentialNewAxes = null;
    }

    private void ProcessAndAddAxes()
    {
        AxisVectors.Clear();
        foreach (Vector3d newAxis in _potentialNewAxes!)
            TryAddAxis(newAxis);
    }

    private void TryAddAxis(Vector3d newAxis)
    {
        foreach (Vector3d existingAxis in AxisVectors!)
            if (Vector3d.AreAlmostParallel(existingAxis, newAxis, CosThreshold))
                return; // It's nearly parallel to an existing axis, don't add it.

        // If it gets here, it's unique enough to be added.
        AxisVectors.Add(newAxis);
    }

    public void Dispose()
    {
        CollisionInfoA.Dispose();
        CollisionInfoB.Dispose();
        SwiftHashSetPool<Vector3d>.Shared.Release(AxisVectors);
    }
}
