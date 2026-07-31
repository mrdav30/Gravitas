using FixedMathSharp;
using FluentAssertions;
using Gravitas.Colliders;
using Gravitas.Tests.Support;
using System;
using Xunit;

namespace Gravitas.Tests.Core;

public sealed class SolidBody2DPointTransformTests
{
    [Fact]
    public void PointConversion_ShouldUseAuthoritativePoseAndCommittedOwnerScale()
    {
        using GravitasWorldContext context = CreateContext();
        SolidBody2D body = CreateBody(
            context,
            new Vector2d(10, 20),
            Fixed64.HalfPi,
            new Vector2d(2, 3));
        Vector2d localPoint = new(3, -4);
        Vector2d expectedWorldPoint = new(22, 26);

        body.TryGetWorldPoint(localPoint, out Vector2d attemptedWorldPoint).Should().BeTrue();
        Vector2d worldPoint = body.GetWorldPoint(localPoint);
        body.TryGetLocalPoint(worldPoint, out Vector2d attemptedLocalPoint).Should().BeTrue();
        Vector2d roundTrip = body.GetLocalPoint(worldPoint);

        attemptedWorldPoint.Should().Be(expectedWorldPoint);
        worldPoint.Should().Be(expectedWorldPoint);
        attemptedLocalPoint.Should().Be(localPoint);
        roundTrip.Should().Be(localPoint);
    }

    [Fact]
    public void PointConversion_WhenHostSnapshotChanges_ShouldRemainAuthoritative()
    {
        using GravitasWorldContext context = CreateContext();
        SolidBody2D body = CreateBody(
            context,
            new Vector2d(4, 5),
            Fixed64.Zero,
            new Vector2d(2, 3));
        Vector2d localPoint = Vector2d.One;
        Vector2d expectedWorldPoint = new(6, 8);

        body.Agent.Transform.LocalPosition = new Vector3d(100, 7, 200);
        body.Agent.Transform.LocalScale = new Vector3d(5, 1, 5);

        body.GetWorldPoint(localPoint).Should().Be(expectedWorldPoint);
        body.GetLocalPoint(expectedWorldPoint).Should().Be(localPoint);
        body.Agent.Transform.TransformPointXZ(localPoint).Should().Be(new Vector2d(105, 205));
    }

    [Fact]
    public void PointConversion_ShouldFailAtomicallyBeforeCommitOrAtTrueOverflow()
    {
        using GravitasWorldContext context = CreateContext();
        var uninitialized = new SolidBody2D(
            new TestMatterAgent(context),
            new LSCircleCollider2D(Fixed64.One));

        uninitialized.TryGetWorldPoint(Vector2d.One, out Vector2d unavailableWorld).Should().BeFalse();
        uninitialized.TryGetLocalPoint(Vector2d.One, out Vector2d unavailableLocal).Should().BeFalse();
        unavailableWorld.Should().Be(Vector2d.Zero);
        unavailableLocal.Should().Be(Vector2d.Zero);
        ((Action)(() => uninitialized.GetWorldPoint(Vector2d.One)))
            .Should().Throw<InvalidOperationException>();
        ((Action)(() => uninitialized.GetLocalPoint(Vector2d.One)))
            .Should().Throw<InvalidOperationException>();

        SolidBody2D initialized = CreateBody(
            context,
            new Vector2d((Fixed64)2_000_000_000, Fixed64.Zero),
            Fixed64.Zero,
            Vector2d.One);
        initialized.TryGetWorldPoint(
            new Vector2d((Fixed64)1_000_000_000, Fixed64.Zero),
            out Vector2d overflowWorld).Should().BeFalse();
        initialized.TryGetLocalPoint(
            new Vector2d(Fixed64.MinValue, Fixed64.Zero),
            out Vector2d overflowLocal).Should().BeFalse();
        overflowWorld.Should().Be(Vector2d.Zero);
        overflowLocal.Should().Be(Vector2d.Zero);
    }

    [Fact]
    public void PointConversion_AfterWarmup_ShouldNotAllocate()
    {
        using GravitasWorldContext context = CreateContext();
        SolidBody2D body = CreateBody(
            context,
            new Vector2d(5, -2),
            Fixed64.PiOver6,
            new Vector2d(3, 2));
        Vector2d localPoint = new(Fixed64.Half, -Fixed64.One);
        Vector2d worldPoint = body.GetWorldPoint(localPoint);

        void ConvertRoundTrip()
        {
            body.TryGetWorldPoint(localPoint, out _);
            body.TryGetLocalPoint(worldPoint, out _);
        }

        AllocationTestHelper.MeasureSteadyState(ConvertRoundTrip).Should().Be(0);
    }

    private static GravitasWorldContext CreateContext()
    {
        GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        context.Settings.RuntimeMode = PhysicsRuntimeMode.TwoD;
        return context;
    }

    private static SolidBody2D CreateBody(
        GravitasWorldContext context,
        Vector2d position,
        Fixed64 rotation,
        Vector2d scale)
    {
        var transform = new FixedTransform(
            position.ToVector3d(Fixed64.Zero),
            FixedQuaternion.Identity,
            new Vector3d(scale.X, Fixed64.One, scale.Y));
        var body = new SolidBody2D(
            new TestMatterAgent(context, transform),
            new LSCircleCollider2D(Fixed64.One))
        {
            Mass = Fixed64.One
        };
        body.Initialize(position, rotation);
        return body;
    }
}
