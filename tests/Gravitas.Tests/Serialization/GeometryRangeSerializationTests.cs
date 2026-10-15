using FixedMathSharp;
using FluentAssertions;
using Gravitas.Colliders;
using Gravitas.Tests.Support;
using System;
using Xunit;

namespace Gravitas.Tests.Serialization;

public sealed class GeometryRangeSerializationTests
{
    public static TheoryData<GravitasSerializationTransport> Transports => GravitasSerializationTransportCases.All();

    [Theory]
    [MemberData(nameof(Transports))]
    public void PopulateCompensatedPlanarGeometry_ShouldPreserveSavedSleepAndRejectUncompensatedTarget(
        GravitasSerializationTransport transport)
    {
        using GravitasWorldContext sourceContext = GravitasWorldContext.CreateOwned();
        using GravitasWorldContext targetContext = GravitasWorldContext.CreateOwned();
        sourceContext.Settings.RuntimeMode = targetContext.Settings.RuntimeMode = PhysicsRuntimeMode.TwoD;
        var source = new SolidBody2D(new TestMatterAgent(sourceContext, CreateTransform(Fixed64.Quarter)),
            new LSCircleCollider2D((Fixed64)6000)) { Mass = (Fixed64)2 };
        source.Initialize(Vector2d.Zero);
        source.Sleep();
        object payload = GravitasSerializationHarness.Serialize(source, transport);
        var target = new SolidBody2D(new TestMatterAgent(targetContext, CreateTransform(Fixed64.Quarter)),
            new LSCircleCollider2D(Fixed64.One)) { Mass = (Fixed64)2 };
        target.Initialize(Vector2d.Zero);

        GravitasSerializationHarness.Populate(target, payload, transport);

        target.IsSleeping.Should().BeTrue();
        ((LSCircleCollider2D)target.Collider).ScaledRadius.Should().Be((Fixed64)1500);
        targetContext.ComputeReplayHash().Should().Be(sourceContext.ComputeReplayHash());
        using GravitasWorldContext rejectedContext = GravitasWorldContext.CreateOwned();
        rejectedContext.Settings.RuntimeMode = PhysicsRuntimeMode.TwoD;
        var rejected = new SolidBody2D(new TestMatterAgent(rejectedContext), new LSCircleCollider2D(Fixed64.One));
        rejected.Initialize(Vector2d.Zero);
        var bounds = rejected.Collider.Bounds;
        uint version = rejected.Collider.RuntimeShapeVersion;
        Action load = () => GravitasSerializationHarness.Populate(rejected, payload, transport);
        load.Should().Throw<ArgumentException>();
        rejected.Collider.Bounds.Should().Be(bounds);
        rejected.Collider.RuntimeShapeVersion.Should().Be(version);
    }

    [Theory]
    [MemberData(nameof(Transports))]
    public void PopulateCompensatedSpatialGeometry_ShouldPreserveSavedSleepAndRejectUncompensatedTarget(
        GravitasSerializationTransport transport)
    {
        using GravitasWorldContext sourceContext = GravitasWorldContext.CreateOwned();
        using GravitasWorldContext targetContext = GravitasWorldContext.CreateOwned();
        var source = new SolidBody(new TestMatterAgent(sourceContext, CreateTransform(Fixed64.Quarter)),
            new LSSphereCollider { Radius = (Fixed64)6000 }) { Mass = (Fixed64)2 };
        source.Initialize(Vector3d.Zero, FixedQuaternion.Identity);
        source.Sleep();
        object payload = GravitasSerializationHarness.Serialize(source, transport);
        var target = new SolidBody(new TestMatterAgent(targetContext, CreateTransform(Fixed64.Quarter)),
            new LSSphereCollider { Radius = Fixed64.One }) { Mass = (Fixed64)2 };
        target.Initialize(Vector3d.Zero, FixedQuaternion.Identity);

        GravitasSerializationHarness.Populate(target, payload, transport);

        target.IsSleeping.Should().BeTrue();
        target.Collider.ScaledRadius.Should().Be((Fixed64)1500);
        targetContext.ComputeReplayHash().Should().Be(sourceContext.ComputeReplayHash());
        using GravitasWorldContext rejectedContext = GravitasWorldContext.CreateOwned();
        var rejected = new SolidBody(new TestMatterAgent(rejectedContext), new LSSphereCollider { Radius = Fixed64.One });
        rejected.Initialize(Vector3d.Zero, FixedQuaternion.Identity);
        var bounds = rejected.Collider.Bounds;
        uint version = rejected.Collider.RuntimeShapeVersion;
        Action load = () => GravitasSerializationHarness.Populate(rejected, payload, transport);
        load.Should().Throw<ArgumentException>();
        rejected.Collider.Bounds.Should().Be(bounds);
        rejected.Collider.RuntimeShapeVersion.Should().Be(version);
    }

    private static FixedTransform CreateTransform(Fixed64 scale) =>
        new(Vector3d.Zero, FixedQuaternion.Identity, Vector3d.One * scale);
}
