using FixedMathSharp;
using FluentAssertions;
using Gravitas.Colliders;
using Gravitas.CollisionHandling;
using Gravitas.Queries;
using Gravitas.Tests.Support;
using GridForge.Configuration;
using Xunit;

namespace Gravitas.Tests.CollisionHandlingTests;

public sealed partial class ContinuousCollisionDetectionTests
{
    [Fact]
    public void DynamicRelativeSphereHit_ShouldNotConstructSyntheticEndpoint()
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        context.SetFrameRate(1);
        context.Environment.Gravity = Fixed64.Zero;
        context.Environment.MaxSpeed = (Fixed64)16;
        context.Environment.MaxFallSpeed = (Fixed64)16;
        Fixed64 targetX = Fixed64.MaxValue - Fixed64.FromFraction(3, 2);
        Fixed64 sourceX = targetX - Fixed64.Two;
        context.World.TryAddGrid(
                new GridConfiguration(
                    new Vector3d(targetX - (Fixed64)16, (Fixed64)(-4), (Fixed64)(-4)),
                    new Vector3d(Fixed64.MaxValue, (Fixed64)4, (Fixed64)4)),
                out _)
            .Should()
            .BeTrue();
        var source = new SolidBody(
            new TestMatterAgent(
                context,
                new FixedTransform(
                    new Vector3d(sourceX, Fixed64.Zero, Fixed64.Zero),
                    FixedQuaternion.Identity,
                    Vector3d.One)),
            new LSSphereCollider { Radius = Fixed64.Half });
        var target = new SolidBody(
            new TestMatterAgent(
                context,
                new FixedTransform(
                    new Vector3d(targetX, Fixed64.Zero, Fixed64.Zero),
                    FixedQuaternion.Identity,
                    Vector3d.One)),
            new LSSphereCollider { Radius = Fixed64.Half });
        source.Initialize(
            new Vector3d(sourceX, Fixed64.Zero, Fixed64.Zero),
            FixedQuaternion.Identity,
            BodyMotionType.Dynamic);
        target.Initialize(
            new Vector3d(targetX, Fixed64.Zero, Fixed64.Zero),
            FixedQuaternion.Identity,
            BodyMotionType.Dynamic);
        target.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        target.ApplyCollisionVelocityState(
            Vector3d.Left * (Fixed64)3,
            Vector3d.Zero);
        context.AdvanceLateSimulateToken();
        target.EnsureContinuousCollisionFramePrepared(context.LateSimulateToken);
        target.SampleContinuousCollisionPosition(Fixed64.One).X
            .Should()
            .Be(targetX - (Fixed64)3);

        source.TryGetDynamicRelativeContinuousCollisionHit(
                target,
                source.Position3d,
                Vector3d.Right,
                Fixed64.Half,
                Fixed64.One,
                Fixed64.Zero,
                out Physics3DHit hit,
                out _)
            .Should()
            .BeTrue();
        hit.Distance.Should().Be(Fixed64.One / (Fixed64)4);
        hit.Normal.Should().Be(Vector3d.Left);
    }

    [Fact]
    public void DynamicRelativeHits_ShouldPreserveOneRawPhysicalDistanceOrdering()
    {
        using PhysicsScenarioBuilder scenario = CreateCcdScenario();
        ScenarioBody<LSSphereCollider> source = scenario.CreateSphere(Vector3d.Zero);
        ScenarioBody<LSSphereCollider> farther = scenario.CreateSphere(
            new Vector3d((Fixed64)11 + Fixed64.MinIncrement, Fixed64.Zero, Fixed64.Zero));
        ScenarioBody<LSSphereCollider> nearer = scenario.CreateSphere(
            new Vector3d((Fixed64)11, Fixed64.Zero, Fixed64.Zero));
        Vector3d displacement = Vector3d.Right * (Fixed64)100_000;
        scenario.Context.AdvanceLateSimulateToken();
        scenario.Context.Physics.PrepareContinuousCollisionFrame();

        source.Body.TryGetDynamicRelativeContinuousCollisionHit(
                farther.Body,
                source.Body.Position3d,
                displacement,
                Fixed64.Half,
                (Fixed64)100_000,
                Fixed64.Zero,
                out Physics3DHit fartherHit,
                out Fixed64 fartherClosingSpeed)
            .Should()
            .BeTrue();
        source.Body.TryGetDynamicRelativeContinuousCollisionHit(
                nearer.Body,
                source.Body.Position3d,
                displacement,
                Fixed64.Half,
                (Fixed64)100_000,
                Fixed64.Zero,
                out Physics3DHit nearerHit,
                out Fixed64 nearerClosingSpeed)
            .Should()
            .BeTrue();

        (fartherHit.Distance - nearerHit.Distance)
            .Should()
            .Be(Fixed64.MinIncrement);
        farther.Collider.Id.Should().BeLessThan(nearer.Collider.Id);
        ContinuousCollisionCandidateOrdering.ShouldReplaceHit(
                nearerHit,
                nearerClosingSpeed,
                true,
                true,
                fartherHit,
                fartherClosingSpeed)
            .Should()
            .BeTrue();
    }

    [Fact]
    public void ContinuousMode_PiecewiseOutAndReturn3DTarget_ShouldBlockTranslationalSource()
    {
        using PhysicsScenarioBuilder scenario = CreateCcdScenario();
        scenario.Context.Settings.ContinuousCollisionMaxToiIterations = 4;
        ScenarioBody<LSSphereCollider> source = scenario.CreateSphere(
            new Vector3d((Fixed64)(-5), Fixed64.Zero, Fixed64.Zero));
        ScenarioBody<LSSphereCollider> target = scenario.CreateSphere(
            new Vector3d(Fixed64.Zero, (Fixed64)3, Fixed64.Zero));
        source.Body.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        target.Body.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        source.Body.ApplyCollisionLinearVelocityDelta(Vector3d.Right * (Fixed64)10);
        target.Body.ApplyCollisionLinearVelocityDelta(Vector3d.Down * (Fixed64)6);
        scenario.Context.AdvanceLateSimulateToken();
        scenario.Context.Physics.PrepareContinuousCollisionFrame();
        target.Body.ApplyContinuousCollisionHandoff(
                Vector3d.Zero,
                FixedQuaternion.Identity,
                Vector3d.Up * (Fixed64)6,
                Vector3d.Zero,
                Fixed64.Half)
            .Should()
            .BeTrue();

        source.Body.LateSimulate(updateSleepState: false, updateColliderState: true);

        source.Body.LastContinuousCollisionToiIterationCount.Should().BeGreaterThan(0);
        source.Body.Position3d.X.Should().BeLessThan((Fixed64)5);
    }

    [Fact]
    public void ContinuousMode_PiecewiseOutAndReturn3DTarget_ShouldReceiveKinematicHandoff()
    {
        using PhysicsScenarioBuilder scenario = CreateCcdScenario();
        ScenarioBody<LSSphereCollider> target = scenario.CreateSphere(
            new Vector3d(Fixed64.Zero, (Fixed64)3, Fixed64.Zero));
        ScenarioBody<LSSphereCollider> source = scenario.CreateSphere(
            new Vector3d((Fixed64)(-5), Fixed64.Zero, Fixed64.Zero),
            isKinematic: true);
        source.Body.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        source.Body.Agent.Transform.LocalPosition = new Vector3d(
            (Fixed64)5,
            Fixed64.Zero,
            Fixed64.Zero);
        target.Body.ApplyCollisionLinearVelocityDelta(Vector3d.Down * (Fixed64)6);
        scenario.Context.AdvanceLateSimulateToken();
        scenario.Context.Physics.PrepareContinuousCollisionFrame();
        target.Body.ApplyContinuousCollisionHandoff(
                Vector3d.Zero,
                FixedQuaternion.Identity,
                Vector3d.Up * (Fixed64)6,
                Vector3d.Zero,
                Fixed64.Half)
            .Should()
            .BeTrue();

        source.Body.LateSimulate(updateSleepState: false, updateColliderState: true);

        source.Body.LastContinuousCollisionToiIterationCount.Should().Be(1);
        target.Body.LinearVelocity.X.Should().BeGreaterThan(Fixed64.Zero);
    }

    [Fact]
    public void ContinuousMode_TargetReversingAtTouchBoundary_ShouldUseSeparatingSegment()
    {
        using PhysicsScenarioBuilder scenario = CreateCcdScenario();
        ScenarioBody<LSSphereCollider> source = scenario.CreateSphere(
            new Vector3d((Fixed64)(-5), Fixed64.Zero, Fixed64.Zero));
        ScenarioBody<LSSphereCollider> target = scenario.CreateSphere(
            new Vector3d(Fixed64.Zero, (Fixed64)4, Fixed64.Zero));
        source.Body.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        source.Body.ApplyCollisionLinearVelocityDelta(Vector3d.Right * (Fixed64)10);
        target.Body.ApplyCollisionLinearVelocityDelta(Vector3d.Down * (Fixed64)6);
        scenario.Context.AdvanceLateSimulateToken();
        scenario.Context.Physics.PrepareContinuousCollisionFrame();
        target.Body.ApplyContinuousCollisionHandoff(
                Vector3d.Up,
                FixedQuaternion.Identity,
                Vector3d.Up * (Fixed64)6,
                Vector3d.Zero,
                Fixed64.Half)
            .Should()
            .BeTrue();

        source.Body.LateSimulate(updateSleepState: false, updateColliderState: true);

        source.Body.LastContinuousCollisionToiIterationCount.Should().Be(0);
        source.Body.Position3d.X.Should().Be((Fixed64)5);
    }
}
