using FixedMathSharp;
using Gravitas.Raycasting;
using SwiftCollections;

namespace Gravitas.Colliders;

public class LSSphereCollider : LSCollider
{
    public override ColliderType Shape => ColliderType.Sphere;

    public override int Priority => ColliderSettings.GetPriority(Shape);

    protected override void OnInitialize()
    {
        Fixed64 diameter = ScaledRadius * 2;
        _size = new Vector3d(diameter, diameter, diameter);
        base.OnInitialize();
    }


    protected override void BuildShape() =>
        Area = FixedMath.PI * ScaledRadiusSqr;  // The area of a circle is pi times the radius squared (A = π r²)

    public override Fixed3x3 CalculateInertiaTensor(Fixed64 mass)
    {
        // For a solid sphere, the inertia tensor is (2/5)*m*r^2 for the diagonal elements
        Fixed64 diagonalElement = Fixed64.Fraction(2, 5) * mass * ScaledRadiusSqr;

        return new Fixed3x3(
            diagonalElement, Fixed64.Zero, Fixed64.Zero,
            Fixed64.Zero, diagonalElement, Fixed64.Zero,
            Fixed64.Zero, Fixed64.Zero, diagonalElement
        );
    }

    public override Vector3d ClosestPointOnSurface(Vector3d other)
    {
        Vector3d direction = other - Center;
        Fixed64 distance = direction.Magnitude;
        direction /= distance; // normalize
        direction *= ScaledRadius;
        return Center + direction;
    }

    public override Vector3d GetNormalAtPoint(Vector3d point) => (point - Center).Normal;

    public override bool ColliderOverlapsRay(RaycastSegmentWorker worker, ref SwiftList<Vector3d> outputIntersectionPoints) =>
        worker.CheckSphereOverlaps(this, ref outputIntersectionPoints);
}
