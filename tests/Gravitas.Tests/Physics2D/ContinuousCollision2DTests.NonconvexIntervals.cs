using FixedMathSharp;
using FluentAssertions;
using Gravitas.Colliders;
using Gravitas.CollisionHandling;
using Gravitas.Materials;
using Xunit;

namespace Gravitas.Tests.Physics2D;

public sealed partial class ContinuousCollision2DTests
{
    [Theory]
    [InlineData(false, 2)]
    [InlineData(true, 2)]
    [InlineData(false, 4)]
    [InlineData(true, 4)]
    public void FullLoop_NonconvexIntervals2D_ShouldAdvanceThroughEmptyCompoundBounds(
        bool compoundSource, int wallCenter)
    {
        using GravitasWorldContext context = CreateContext(frameRate: 1);
        context.Environment.DampingFactor = Fixed64.Zero;
        Vector2d[] vertices =
        {
            new((Fixed64)(-3), Fixed64.FromFraction(-1, 10)),
            new((Fixed64)3, Fixed64.FromFraction(-1, 10)),
            new((Fixed64)3, Fixed64.FromFraction(1, 10)),
            new((Fixed64)(-3), Fixed64.FromFraction(1, 10))
        };
        LSCollider2D sourceCollider = compoundSource
            ? new LSCompoundCollider2D(CompoundColliderPart2D.ConvexPolygon(vertices, Vector2d.Zero))
            : new LSPolygonCollider2D(vertices);
        SolidBody2D source = CreateBody(context, sourceCollider, Vector2d.Zero,
            immovable: false, isKinematic: true);
        source.Collider.Material = PhysicsMaterial.Frictionless;
        source.UseManualGrounding();
        source.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        Vector2d wallSize = new((Fixed64)8, Fixed64.FromFraction(1, 5));
        SolidBody2D target = CreateBody(context, new LSCompoundCollider2D(
            CompoundColliderPart2D.AABBox(wallSize, new Vector2d(Fixed64.Zero, (Fixed64)wallCenter)),
            CompoundColliderPart2D.AABBox(wallSize, new Vector2d(Fixed64.Zero, (Fixed64)(-wallCenter)))),
            Vector2d.Zero, immovable: true);
        target.Collider.Material = PhysicsMaterial.Frictionless;
        source.Collider.Bounds.Intersects(target.Collider.Bounds).Should().BeTrue();
        CollisionDetection2D.TryCollide(source.Collider, target.Collider, out _).Should().BeFalse();

        source.SetRotation(FixedMath.DegToRad((Fixed64)30));
        CollisionDetection2D.TryCollide(source.Collider, target.Collider, out _).Should().BeFalse();
        source.SetRotation(FixedMath.DegToRad((Fixed64)45));
        CollisionDetection2D.TryCollide(source.Collider, target.Collider, out _).Should().Be(wallCenter == 2);
        source.SetRotation(Fixed64.Zero);
        Fixed64 endRotation = FixedMath.DegToRad((Fixed64)90);
        source.Agent.Transform.LocalRotationXZRadians = endRotation;

        context.Simulate();
        context.LateSimulate();

        source.Position.Should().Be(Vector2d.Zero);
        if (wallCenter == 2)
        {
            // The blade's Z support is 3*sin(angle) + 1/10*cos(angle), reaching the wall between these probes.
            source.Rotation.Should().BeInRange(
                FixedMath.DegToRad((Fixed64)30), FixedMath.DegToRad((Fixed64)45));
            source.LastContinuousCollisionToiIterationCount.Should().BeGreaterThan(0);
        }
        else
        {
            source.Rotation.Should().Be(endRotation);
            source.LastContinuousCollisionToiIterationCount.Should().Be(0);
        }
    }
}
