//=======================================================================
// ColliderBodyPoseTransaction2DTests.cs
//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp;
using FixedMathSharp.Geometry;
using FluentAssertions;
using Gravitas.Colliders;
using Gravitas.Tests.Support;
using System;
using Xunit;

namespace Gravitas.Tests.Colliders;

public sealed class ColliderBodyPoseTransaction2DTests
{
    [Fact]
    public void SetPosition_WhenShapePreparationFails_ShouldNotPublishPartialPose()
    {
        using GravitasWorldContext context = Physics2DTestWorld.CreateContext();
        var collider = new FailingPoseCollider2D();
        var body = new SolidBody2D(new TestMatterAgent(context), collider);
        body.Initialize(Vector2d.Zero);
        FixedBoundArea bounds = collider.Bounds;
        uint runtimeShapeVersion = collider.RuntimeShapeVersion;
        uint broadPhaseVersion = collider.BroadPhaseVersion;
        collider.FailPreparation = true;

        Action setPosition = () => body.SetPosition(Vector2d.Right);

        setPosition.Should().Throw<InvalidOperationException>();
        body.Position.Should().Be(Vector2d.Zero);
        collider.Center.Should().Be(Vector2d.Zero);
        collider.Bounds.Should().Be(bounds);
        collider.RuntimeShapeVersion.Should().Be(runtimeShapeVersion);
        collider.BroadPhaseVersion.Should().Be(broadPhaseVersion);

        collider.FailPreparation = false;
        body.SetPosition(Vector2d.Right);
        body.Position.Should().Be(Vector2d.Right);
        collider.Center.Should().Be(Vector2d.Right);
        collider.RuntimeShapeVersion.Should().Be(runtimeShapeVersion + 1);
    }

    private sealed class FailingPoseCollider2D : LSCollider2D
    {
        internal bool FailPreparation { get; set; }

        public override ColliderType2D Shape => (ColliderType2D)byte.MaxValue;

        public override bool ContainsPoint(Vector2d point) => false;

        public override Vector2d GetClosestPoint(Vector2d point) => Center;

        public override Vector2d GetSupportPoint(Vector2d direction) => Center;

        internal override Fixed64 CalculateCenterOfMassMoment(
            Fixed64 mass) =>
            Fixed64.Zero;

        internal override ExactMassWeight CalculateAreaForMassProperties() =>
            ExactMassWeight.Zero;

        internal override ExactMassWeight CalculatePreparedAreaForMassProperties() =>
            ExactMassWeight.Zero;

        private protected override void PrepareShape(
            in ColliderShapeSnapshot2D snapshot)
        {
            if (FailPreparation)
                throw new InvalidOperationException("Simulated shape-admission failure.");

            SetPreparedBounds(FixedBoundArea.FromMinMax(
                snapshot.Center - Vector2d.One,
                snapshot.Center + Vector2d.One));
        }

        private protected override void PublishShape()
        {
        }
    }
}
