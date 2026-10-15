using FixedMathSharp;
using FluentAssertions;
using Gravitas.Colliders;
using Gravitas.Queries;
using Gravitas.Support;
using Gravitas.Tests.Support;
using SwiftCollections;
using System;
using System.Collections.Generic;
using Xunit;

namespace Gravitas.Tests.Queries;

public sealed class RuntimeQueryGeometryTests
{
    [Fact]
    public void FailedPreparation_ShouldNotReplaceTheCommittedMixedAdmissionMetric()
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        context.Settings.RuntimeMode = PhysicsRuntimeMode.TwoD;
        var transform = new FixedTransform(Vector3d.Zero, FixedQuaternion.Identity, Vector3d.One);
        var circle = new LSCircleCollider2D(Fixed64.One);
        circle.InitializeWithNoBody(new TestMatterAgent(context, transform));
        uint version = circle.RuntimeShapeVersion;
        transform.LocalScale = Vector3d.One * (Fixed64)8192;
        Action query = () => context.QueryMixed.SweepSphereAgainst2D(-Vector3d.Right, Vector3d.Right,
            Fixed64.One, PhysicsLayerMask.All, out _);

        query.Should().Throw<ArgumentException>();
        circle.RuntimeShapeVersion.Should().Be(version);
        transform.LocalScale = Vector3d.One;
        query.Should().NotThrow();
        circle.RuntimeShapeVersion.Should().Be(version);
    }
    [Fact]
    public void ReconfiguredAttachment_ShouldRefreshTheProxyAndCommittedMixedAdmissionMetric()
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        context.Settings.RuntimeMode = PhysicsRuntimeMode.TwoD;
        var collider = new LSCircleCollider2D(Fixed64.One);
        var body = new SolidBody2D(new TestMatterAgent(context), collider) { Mass = Fixed64.One };
        body.Initialize(Vector2d.Zero);

        body.TryReconfigureCollider(ColliderShapeDefinition2D.Circle(Fixed64.One),
            new Vector2d(4095, 0), Vector2d.Zero, out _, out _).Should().Be(ColliderReconfigurationStatus.Applied);

        body.ResolveContinuousCollisionProxyRadius().Should().BeGreaterThan((Fixed64)4096);
        Action mixed = () => context.QueryMixed.SweepSphereAgainst2D(-Vector3d.Right, Vector3d.Right,
            Fixed64.One, PhysicsLayerMask.All, out _);
        mixed.Should().Throw<ArgumentException>();
    }
    [Theory]
    [InlineData(false, false)]
    [InlineData(false, true)]
    [InlineData(true, false)]
    [InlineData(true, true)]
    public void GroundProbeRadius_ShouldRejectOversizedConfigurationAndRecords(bool planar, bool loading)
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        Fixed64 radius = (Fixed64)4096 + Fixed64.MinIncrement;
        var payload = new InvalidRecordPayloadChronicler(new Dictionary<string, object>
        {
            ["GroundProbeRadius"] = radius,
            ["BodySchemaVersion"] = 1
        });
        Action change;
        if (planar)
        {
            var body = new SolidBody2D(new TestMatterAgent(context), new LSCircleCollider2D(Fixed64.One));
            body.Initialize(Vector2d.Zero);
            change = loading ? () => body.RecordData(payload) : () => body.GroundProbeRadius = radius;
        }
        else
        {
            var body = new SolidBody(new TestMatterAgent(context), new LSSphereCollider());
            body.Initialize(Vector3d.Zero, FixedQuaternion.Identity);
            change = loading ? () => body.RecordData(payload) : () => body.GroundProbeRadius = radius;
        }
        var hash = context.ComputeReplayHash();
        change.Should().Throw<ArgumentException>().WithMessage("*4,096*");
        context.ComputeReplayHash().Should().Be(hash);
    }

    [Theory]
    [InlineData(false, false)]
    [InlineData(false, true)]
    [InlineData(true, false)]
    [InlineData(true, true)]
    public void AdmittedCcdProxy_ShouldRetainItsConservativePaddingInInternalSearches(bool planar, bool mixed)
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        context.Settings.RuntimeMode = mixed ? PhysicsRuntimeMode.Mixed
            : planar ? PhysicsRuntimeMode.TwoD : PhysicsRuntimeMode.ThreeD;
        if (planar)
        {
            Fixed64 offset = (Fixed64)4095 - (mixed ? Fixed64.MinIncrement : Fixed64.Zero);
            var collider = new LSCircleCollider2D(Fixed64.One)
            {
                LocalOffset = new Vector2d(offset, Fixed64.Zero),
                MixedHalfThicknessOverride = Fixed64.MinIncrement
            };
            var body = new SolidBody2D(new TestMatterAgent(context), collider);
            body.Initialize(Vector2d.Zero, motionType: BodyMotionType.Kinematic);
            Fixed64 proxy = body.ResolveContinuousCollisionProxyRadius();
            proxy.Should().BeGreaterThan((Fixed64)4096);
            var hits = new SwiftList<Physics2DHit>();
            context.Query2D.OverlapCircleAgainstStaticAll(Vector2d.Zero, proxy, PhysicsLayerMask.All,
                hits, collider).Should().Be(0);
            context.Query2D.SweepCircleAgainstStaticAll(Vector2d.Zero, Vector2d.Right, proxy,
                PhysicsLayerMask.All, hits, collider).Should().Be(0);
            if (mixed)
                context.QueryMixed.SweepCircleAgainstStatic3DAll(Vector2d.Zero, Vector2d.Right, proxy,
                    Fixed64.Zero, Fixed64.MinIncrement, PhysicsLayerMask.All,
                    new SwiftList<PhysicsMixedHit>(), collider).Should().Be(0);
        }
        else
        {
            var collider = new LSSphereCollider { Radius = Fixed64.One,
                LocalOffset = new Vector3d(4095, 0, 0) };
            var body = new SolidBody(new TestMatterAgent(context), collider);
            body.Initialize(Vector3d.Zero, FixedQuaternion.Identity, BodyMotionType.Kinematic);
            Fixed64 proxy = body.ResolveContinuousCollisionProxyRadius();
            proxy.Should().BeGreaterThan((Fixed64)4096);
            context.Query3D.SweepSphereAgainstStaticAll(Vector3d.Zero, Vector3d.Right, proxy,
                PhysicsLayerMask.All, new SwiftList<Physics3DHit>(), collider).Should().Be(0);
            if (mixed)
                context.QueryMixed.SweepSphereAgainstStatic2DAll(Vector3d.Zero, Vector3d.Right, proxy,
                    PhysicsLayerMask.All, new SwiftList<PhysicsMixedHit>(), collider).Should().Be(0);
        }
    }
}
