using System;
using FixedMathSharp;
using FixedMathSharp.Geometry;
using Gravitas.Colliders;
using Gravitas.Queries;
using SwiftCollections;

namespace Gravitas.Tests.Support;

internal sealed class UnsupportedTestCollider3D : LSCollider
{
    internal bool OmitGeometryReach { get; set; }

    internal bool ThrowOnShapePreparation { get; set; }

    internal Fixed3x3 InertiaTensor { get; set; } = Fixed3x3.Zero;

    internal bool ThrowOnInertiaCalculation { get; set; }

    internal ExactMassWeight MassPropertyWeight { get; set; } =
        ExactMassWeight.Zero;

    internal bool ReportRayOverlapWithoutIntersection { get; set; }

    internal bool DeactivateOnInitialize { get; set; }

    internal Vector3d? ClosestPointOverride { get; set; }

    internal Vector3d? NormalOverride { get; set; }

    internal void OverrideDerivedBoundsForReplayTest(Vector3d min, Vector3d max) =>
        _bounds = FixedBoundBox.FromMinMax(min, max);

    public override ColliderType Shape => (ColliderType)byte.MaxValue;

    public override int Priority => 0;

    private protected override void PrepareShape(in ColliderShapeSnapshot snapshot)
    {
        if (ThrowOnShapePreparation)
            throw new InvalidOperationException("Configured shape preparation failure.");
        if (!OmitGeometryReach)
            SetPreparedGeometryReach(ColliderGeometryPolicy.Magnitude(Vector3d.One));
        SetPreparedBounds(FixedBoundBox.FromMinMax(
            snapshot.Center - Vector3d.One,
            snapshot.Center + Vector3d.One));
    }

    private protected override void PublishShape() => Area = Fixed64.One;

    protected override void OnInitialize()
    {
        if (DeactivateOnInitialize)
            IsActive = false;

        base.OnInitialize();
    }

    public override Fixed3x3 CalculateInertiaTensor(Fixed64 mass, Vector3d localCenterOfMassOffset)
    {
        if (ThrowOnInertiaCalculation)
            throw new InvalidOperationException("Configured test inertia calculation failure.");

        return InertiaTensor;
    }

    internal override Fixed3x3 CalculateCenterOfMassInertiaTensor(
        Fixed64 mass) =>
        InertiaTensor;

    internal override ExactMassWeight CalculateMassPropertyWeight() =>
        MassPropertyWeight;

    internal override ExactMassWeight CalculatePreparedMassPropertyWeight() =>
        MassPropertyWeight;

    public override Vector3d ClosestPointOnSurface(Vector3d other) =>
        ClosestPointOverride ?? Center;

    public override Vector3d GetNormalAtPoint(Vector3d point) =>
        NormalOverride ?? Vector3d.Up;

    internal override FixedPointAnchor GetClosestSurfaceAnchor(
        Vector3d point,
        out Vector3d normal)
    {
        Vector3d closest = ClosestPointOnSurface(point);
        normal = GetNormalAtPoint(closest);
        return new FixedPointAnchor(
            closest,
            FixedQuaternion.Identity,
            Vector3d.Zero);
    }

    public override bool ColliderOverlapsRay(RaycastSegmentWorker worker, ref SwiftList<Vector3d> outputIntersectionPoints) =>
        ReportRayOverlapWithoutIntersection;
}
