//=======================================================================
// UnsupportedTestColliders.cs
//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp;
using FixedMathSharp.Geometry;
using Gravitas.Colliders;
using Gravitas.Queries;
using SwiftCollections;

namespace Gravitas.Tests.Support;

internal sealed class UnsupportedTestCollider2D : LSCollider2D
{
    public override ColliderType2D Shape => (ColliderType2D)byte.MaxValue;

    public override bool ContainsPoint(Vector2d point) => false;

    public override Vector2d GetClosestPoint(Vector2d point) =>
        new(
            FixedMath.Clamp(point.X, Center.X - Fixed64.One, Center.X + Fixed64.One),
            FixedMath.Clamp(point.Y, Center.Y - Fixed64.One, Center.Y + Fixed64.One));

    public override Vector2d GetSupportPoint(Vector2d direction) => Center;

    internal override Fixed64 CalculateCenterOfMassMoment(Fixed64 mass) =>
        Fixed64.Zero;

    internal override ExactMassWeight CalculateAreaForMassProperties() =>
        ExactMassWeight.Zero;

    internal override ExactMassWeight CalculatePreparedAreaForMassProperties() =>
        ExactMassWeight.Zero;

    private protected override void PrepareShape(in ColliderShapeSnapshot2D snapshot) =>
        SetPreparedBounds(FixedBoundArea.FromMinMax(
            snapshot.Center - Vector2d.One,
            snapshot.Center + Vector2d.One));

    private protected override void PublishShape() { }
}

internal sealed class UnsupportedTestCollider3D : LSCollider
{
    internal Fixed3x3 InertiaTensor { get; set; } = Fixed3x3.Zero;

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

    private protected override void PrepareShape(in ColliderShapeSnapshot snapshot) =>
        SetPreparedBounds(FixedBoundBox.FromMinMax(
            snapshot.Center - Vector3d.One,
            snapshot.Center + Vector3d.One));

    private protected override void PublishShape() => Area = Fixed64.One;

    protected override void OnInitialize()
    {
        if (DeactivateOnInitialize)
            IsActive = false;

        base.OnInitialize();
    }

    public override Fixed3x3 CalculateInertiaTensor(Fixed64 mass, Vector3d localCenterOfMassOffset) =>
        InertiaTensor;

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
