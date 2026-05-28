using FixedMathSharp;
using SwiftCollections;

namespace Gravitas.CollisionHandling;

internal sealed class CollisionContext
{
    /// <summary>
    /// Cosine of the angle threshold. Used to determine if two axes are nearly parallel.
    /// </summary>
    private static readonly Fixed64 AngleThresholdCos = FixedMath.Cos(FixedMath.DegToRad(new(2)));

    private readonly SwiftHashSet<Vector3d> _potentialNewAxes = new(32);

    public CollisionObjectInfo CollisionInfoA { get; private set; } = null!;

    public CollisionObjectInfo CollisionInfoB { get; private set; } = null!;

    public Vector3d Displacement { get; private set; }

    public (Vector3d Point1, Vector3d Point2) PointsOfContact => (CollisionInfoA.PointOfContact, CollisionInfoB.PointOfContact);

    public SwiftHashSet<Vector3d> AxisVectors { get; } = new(32);

    public CollisionContext() { }

    public void Set(CollisionObjectInfo collisionInfoA, CollisionObjectInfo collisionInfoB)
    {
        CollisionInfoA = collisionInfoA;
        CollisionInfoB = collisionInfoB;
        AxisVectors.Clear();
        _potentialNewAxes.Clear();
        Displacement = Vector3d.Zero;
    }

    public void Prepare(CollisionObjectInfo collisionInfoA, CollisionObjectInfo collisionInfoB)
    {
        Set(collisionInfoA, collisionInfoB);
        PrepareDataForSAT();
    }

    public void PrepareDataForSAT()
    {
        _potentialNewAxes.Clear();
        CollisionInfoA.PrepareVertices(_potentialNewAxes);
        CollisionInfoB.PrepareVertices(_potentialNewAxes);
        ProcessAndAddAxes();
        Displacement = CollisionInfoB.PointOfContact - CollisionInfoA.PointOfContact;
    }

    private void ProcessAndAddAxes()
    {
        AxisVectors.Clear();
        foreach (Vector3d newAxis in _potentialNewAxes)
            TryAddAxis(newAxis);
    }

    private void TryAddAxis(Vector3d newAxis)
    {
        foreach (Vector3d existingAxis in AxisVectors)
            if (Vector3d.AreAlmostParallel(existingAxis, newAxis, AngleThresholdCos))
                return; // It's nearly parallel to an existing axis, don't add it.

        AxisVectors.Add(newAxis);
    }
}
