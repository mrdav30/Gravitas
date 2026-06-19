using FixedMathSharp;
using Gravitas.Queries;
using SwiftCollections;

namespace Gravitas.Colliders;

public class LSSphereCollider : LSCollider
{
    public LSSphereCollider() { }

    public LSSphereCollider(ColliderShapeDefinition definition)
    {
        definition.EnsureKind(ColliderShapeDefinitionKind.Sphere);
        Radius = definition.Radius;
    }

    public override ColliderType Shape => ColliderType.Sphere;

    public override int Priority => ColliderSettings.GetPriority(Shape);

    protected override void OnInitialize()
    {
        Fixed64 diameter = _radius * 2;
        _size = new Vector3d(diameter, diameter, diameter);
        base.OnInitialize();
    }

    protected override void OnRadiusChanged()
    {
        Fixed64 diameter = _radius * 2;
        _size = new Vector3d(diameter, diameter, diameter);
    }

    protected override void BuildShape() =>
        Area = Fixed64.Pi * ScaledRadiusSqr;  // The area of a circle is pi times the radius squared (A = π r²)

    public override Fixed3x3 CalculateInertiaTensor(Fixed64 mass, Vector3d localCenterOfMassOffset)
    {
        // For a solid sphere, the inertia tensor is (2/5)*m*r^2 for the diagonal elements
        Fixed64 diagonalElement = Fixed64.FromFraction(2, 5) * mass * ScaledRadiusSqr;

        Fixed3x3 tensor = new(
            diagonalElement, Fixed64.Zero, Fixed64.Zero,
            Fixed64.Zero, diagonalElement, Fixed64.Zero,
            Fixed64.Zero, Fixed64.Zero, diagonalElement
        );
        return ShiftInertiaTensorFromLocalCenterOfMass(tensor, mass, localCenterOfMassOffset);
    }

    public override Vector3d ClosestPointOnSurface(Vector3d other)
    {
        Vector3d direction = other - Center;
        Fixed64 distance = direction.Magnitude;
        direction /= distance; // normalize
        direction *= ScaledRadius;
        return Center + direction;
    }

    public override Vector3d GetNormalAtPoint(Vector3d point) => (point - Center).Normalized;

    public override bool ColliderOverlapsRay(RaycastSegmentWorker worker, ref SwiftList<Vector3d> outputIntersectionPoints) =>
        worker.CheckSphereOverlaps(this, ref outputIntersectionPoints);
}
