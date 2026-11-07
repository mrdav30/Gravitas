using System;
using FixedMathSharp;
using FixedMathSharp.Geometry;
using Gravitas.Colliders;

namespace Gravitas.Tests.Support;

internal sealed class UnsupportedTestCollider2D : LSCollider2D
{
    internal bool OmitGeometryReach { get; set; }

    internal bool ContainsQueryPoint { get; set; }
    internal Fixed64 MomentOfInertia { get; set; } = Fixed64.Zero;

    internal bool ThrowOnInertiaCalculation { get; set; }

    public override Fixed64 CalculateMomentOfInertia(Fixed64 mass, Vector2d localReferencePoint)
    {
        if (ThrowOnInertiaCalculation)
            throw new InvalidOperationException("Configured test moment calculation failure.");

        return MomentOfInertia;
    }

    public override ColliderType2D Shape => (ColliderType2D)byte.MaxValue;

    public override bool ContainsPoint(Vector2d point) => ContainsQueryPoint;

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

    private protected override void PrepareShape(in ColliderShapeSnapshot2D snapshot)
    {
        if (!OmitGeometryReach)
            SetPreparedGeometryReach(ColliderGeometryPolicy.Magnitude(Vector2d.One));
        SetPreparedBounds(FixedBoundArea.FromMinMax(
            snapshot.Center - Vector2d.One,
            snapshot.Center + Vector2d.One));
    }

    private protected override void PublishShape() { }
}
