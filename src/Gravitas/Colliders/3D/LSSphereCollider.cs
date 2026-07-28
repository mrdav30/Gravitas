//=======================================================================
// LSSphereCollider.cs
//=======================================================================
// MIT License, Copyright (c) 2026–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp;
using FixedMathSharp.Geometry;
using Gravitas.Queries;
using SwiftCollections;

namespace Gravitas.Colliders;

public class LSSphereCollider : LSCollider
{
    private Fixed64 _scaledRadius = Fixed64.Half;
    private Fixed64 _preparedRadius;
    private Fixed64 _preparedArea;

    public LSSphereCollider() { }

    public LSSphereCollider(ColliderShapeDefinition definition)
    {
        definition.EnsureKind(ColliderShapeDefinitionKind.Sphere);
        Material = definition.Material;
        Radius = definition.Radius;
    }

    public override ColliderType Shape => ColliderType.Sphere;

    public override int Priority => ColliderSettings.GetPriority(Shape);

    protected override void OnRadiusChanged()
    {
        Fixed64 diameter = _radius * 2;
        _size = new Vector3d(diameter, diameter, diameter);
    }

    public override Fixed64 ScaledRadius => _scaledRadius;

    private protected override void PrepareShape(in ColliderShapeSnapshot snapshot)
    {
        Fixed64 radiusX = ColliderScalePolicy.ScalePositive(
            snapshot.Radius,
            snapshot.OwnerScale.X,
            snapshot.PartScale.X);
        Fixed64 radiusY = ColliderScalePolicy.ScalePositive(
            snapshot.Radius,
            snapshot.OwnerScale.Y,
            snapshot.PartScale.Y);
        Fixed64 radiusZ = ColliderScalePolicy.ScalePositive(
            snapshot.Radius,
            snapshot.OwnerScale.Z,
            snapshot.PartScale.Z);
        _preparedRadius = FixedMath.Max(radiusX, FixedMath.Max(radiusY, radiusZ));
        _preparedArea = Fixed64.Pi * _preparedRadius * _preparedRadius;
        SetPreparedBounds(FixedBoundBox.FromCenterAndScopeClippedToDomain(
            snapshot.Center,
            Vector3d.One * _preparedRadius));
    }

    private protected override void PublishShape()
    {
        _scaledRadius = _preparedRadius;
        Area = _preparedArea;
    }

    protected internal override FixedMassWeight CalculateMassPropertyWeight() =>
        FixedMassWeight.FromProduct(
            Fixed64.FromFraction(4, 3) * Fixed64.Pi,
            ScaledRadius,
            ScaledRadius,
            ScaledRadius);

    internal override FixedMassWeight CalculatePreparedMassPropertyWeight() =>
        FixedMassWeight.FromProduct(
            Fixed64.FromFraction(4, 3) * Fixed64.Pi,
            _preparedRadius,
            _preparedRadius,
            _preparedRadius);

    internal override Fixed3x3 CalculateCenterOfMassInertiaTensor(Fixed64 mass)
    {
        // For a solid sphere, the inertia tensor is (2/5)*m*r^2 for the diagonal elements
        Fixed64 diagonalElement = Fixed64.FromFraction(2, 5) * mass * ScaledRadiusSqr;

        Fixed3x3 tensor = new(
            diagonalElement, Fixed64.Zero, Fixed64.Zero,
            Fixed64.Zero, diagonalElement, Fixed64.Zero,
            Fixed64.Zero, Fixed64.Zero, diagonalElement
        );
        return tensor;
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
