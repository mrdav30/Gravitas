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
using System;

namespace Gravitas.Colliders;

/// <summary>Represents a runtime 3D sphere collider.</summary>
public sealed class LSSphereCollider : LSCollider
{
    private Fixed64 _scaledRadius = Fixed64.Half;
    private Fixed64 _preparedRadius;
    private Fixed64 _preparedArea;

    /// <summary>Creates a sphere collider with default dimensions.</summary>
    public LSSphereCollider() { }

    /// <summary>Creates a runtime sphere from an authored shape definition.</summary>
    public LSSphereCollider(ColliderShapeDefinition definition)
    {
        definition.EnsureKind(ColliderShapeDefinitionKind.Sphere);
        Material = definition.Material;
        Radius = definition.Radius;
    }

    /// <inheritdoc/>
    public override ColliderType Shape => ColliderType.Sphere;

    /// <inheritdoc/>
    public override int Priority => ColliderSettings.GetPriority(Shape);

    /// <inheritdoc/>
    protected override void OnRadiusChanged()
    {
        Fixed64 diameter = _radius * 2;
        _size = new Vector3d(diameter, diameter, diameter);
    }

    /// <inheritdoc/>
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
        SetPreparedGeometryReach(_preparedRadius);
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

    internal override ExactMassWeight CalculateMassPropertyWeight()
    {
        Fixed64 radius = GetMassPropertyRadius();
        return ExactMassWeight.FromProduct(
            Fixed64.FromFraction(4, 3) * Fixed64.Pi,
            radius,
            radius,
            radius);
    }

    internal Fixed64 GetMassPropertyRadius() =>
        HasCommittedShape ? _scaledRadius : GetCurrentScaledRadius();

    internal override ExactMassWeight CalculatePreparedMassPropertyWeight() =>
        ExactMassWeight.FromProduct(
            Fixed64.FromFraction(4, 3) * Fixed64.Pi,
            _preparedRadius,
            _preparedRadius,
            _preparedRadius);

    internal override Fixed3x3 CalculateCenterOfMassInertiaTensor(Fixed64 mass)
    {
        // For a solid sphere, the inertia tensor is (2/5)*m*r^2 for the diagonal elements
        Fixed64 radius = GetMassPropertyRadius();
        Fixed64 diagonalElement = ExactMassProperties.InertiaMoment(
            mass, ExactMassProperties.InertiaGeometryTerm(Fixed64.FromFraction(2, 5), radius, radius));

        Fixed3x3 tensor = new(
            diagonalElement, Fixed64.Zero, Fixed64.Zero,
            Fixed64.Zero, diagonalElement, Fixed64.Zero,
            Fixed64.Zero, Fixed64.Zero, diagonalElement
        );
        return tensor;
    }

    /// <inheritdoc/>
    public override Vector3d ClosestPointOnSurface(Vector3d other)
    {
        FixedPointAnchor anchor =
            GetClosestSurfaceAnchor(other, out _);
        if (anchor.TryGetPoint(out Vector3d point))
            return point;

        throw new InvalidOperationException(
            "The closest sphere surface point is outside the representable coordinate domain.");
    }

    /// <inheritdoc/>
    public override Vector3d GetNormalAtPoint(Vector3d point)
    {
        if (point == Center)
            return Vector3d.Zero;

        _ = GetClosestSurfaceAnchor(
            point,
            out Vector3d normal);
        return normal;
    }

    internal override FixedPointAnchor GetClosestSurfaceAnchor(
        Vector3d point,
        out Vector3d normal) =>
        GetClosestSurfaceAnchor(point, out normal, out _);

    internal override FixedPointAnchor GetClosestSurfaceAnchor(
        Vector3d point, out Vector3d normal, out bool contained) =>
        WideFiniteAxisIntersection
            .GetClosestCenteredCapsuleSurfaceAnchor(
                point,
                Center,
                FixedQuaternion.Identity,
                Vector3d.Up,
                Fixed64.Zero,
                ScaledRadius,
                Vector3d.Right,
                out normal,
                out contained);

    /// <inheritdoc/>
    public override bool ColliderOverlapsRay(RaycastSegmentWorker worker, ref SwiftList<Vector3d> outputIntersectionPoints) =>
        worker.CheckSphereOverlaps(this, ref outputIntersectionPoints);
}
