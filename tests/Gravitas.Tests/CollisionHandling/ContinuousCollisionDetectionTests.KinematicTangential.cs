using FixedMathSharp;
using FluentAssertions;
using Gravitas.Colliders;
using Gravitas.CollisionHandling;
using Gravitas.Support;
using Gravitas.Tests.Support;
using GridForge.Configuration;
using GridForge.Grids;
using GridForge.Grids.Topology;
using Xunit;

namespace Gravitas.Tests.CollisionHandlingTests;

public sealed partial class ContinuousCollisionDetectionTests
{
    [Fact]
    public void ContinuousMode_KinematicCylinderMovingAcrossSupportingTop_ShouldReachHostPose()
    {
        using PhysicsScenarioBuilder scenario = CreateCcdScenario();
        var support = new LSCuboidCollider
        {
            Size = new Vector3d(Fixed64.One, (Fixed64)2, Fixed64.One)
        };
        scenario.InitializeStaticCollider(
            support,
            new Vector3d(Fixed64.Zero, -Fixed64.One, Fixed64.Zero));
        ScenarioBody<LSCylinderCollider> source = scenario.CreateBody(
            new LSCylinderCollider
            {
                Radius = Fixed64.Quarter,
                Size = Vector3d.One * Fixed64.Half
            },
            Vector3d.Up * Fixed64.Quarter,
            FixedQuaternion.Identity,
            isKinematic: true);
        source.Body.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        DisableGroundQueries(source.Body);
        var hostTarget = new Vector3d(
            Fixed64.FromFraction(1, 64),
            Fixed64.Quarter,
            Fixed64.Zero);

        source.Body.Agent.Transform.LocalPosition = hostTarget;
        scenario.Context.LateSimulate();

        source.Body.Position3d.Should().Be(hostTarget,
            "motion tangent to a supporting face is not a closing collision");
        source.Body.LastContinuousCollisionToiIterationCount.Should().Be(0);
    }

    [Fact]
    public void ContinuousMode_KinematicCylinderSeparatingFromDynamicSupport_ShouldLeaveSupportUnchanged()
    {
        using PhysicsScenarioBuilder scenario = CreateCcdScenario();
        var supportStart = new Vector3d(
            Fixed64.Zero,
            -Fixed64.One,
            Fixed64.Zero);
        ScenarioBody<LSCuboidCollider> support = scenario.CreateBody(
            new LSCuboidCollider
            {
                Size = new Vector3d(Fixed64.One, (Fixed64)2, Fixed64.One)
            },
            supportStart,
            FixedQuaternion.Identity);
        ScenarioBody<LSCylinderCollider> source = scenario.CreateBody(
            new LSCylinderCollider
            {
                Radius = Fixed64.Quarter,
                Size = Vector3d.One * Fixed64.Half
            },
            Vector3d.Up * Fixed64.Quarter,
            FixedQuaternion.Identity,
            isKinematic: true);
        source.Body.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        DisableGroundQueries(source.Body);
        DisableGroundQueries(support.Body);
        support.Body.Sleep();
        var sourceTarget = new Vector3d(
            Fixed64.FromFraction(1, 64),
            Fixed64.Half,
            Fixed64.Zero);

        source.Body.Agent.Transform.LocalPosition = sourceTarget;
        scenario.Context.LateSimulate();

        source.Body.Position3d.Should().Be(sourceTarget,
            "relative motion away from a dynamic support is not a closing collision");
        source.Body.LastContinuousCollisionToiIterationCount.Should().Be(0);
        support.Body.Position3d.Should().Be(supportStart);
        support.Body.LinearVelocity.Should().Be(Vector3d.Zero);
        support.Body.IsSleeping.Should().BeTrue();
    }

    [Fact]
    public void ContinuousMode_KinematicCylinderBeforeAdjacentQuarterStep_ShouldReachHostPose()
    {
        using PhysicsScenarioBuilder scenario = CreateCcdScenario();
        var lowerTread = new LSCuboidCollider
        {
            Size = new Vector3d(Fixed64.One, Fixed64.One, Fixed64.One)
        };
        scenario.InitializeStaticCollider(
            lowerTread,
            new Vector3d(Fixed64.Zero, Fixed64.FromFraction(-3, 2), Fixed64.Zero));
        var upperTread = new LSCuboidCollider
        {
            Size = new Vector3d(Fixed64.One, Fixed64.FromFraction(5, 4), Fixed64.One)
        };
        scenario.InitializeStaticCollider(
            upperTread,
            new Vector3d(Fixed64.One, Fixed64.FromFraction(-11, 8), Fixed64.Zero));
        var thirdTread = new LSCuboidCollider
        {
            Size = new Vector3d(Fixed64.One, Fixed64.FromFraction(3, 2), Fixed64.One)
        };
        scenario.InitializeStaticCollider(
            thirdTread,
            new Vector3d((Fixed64)2, Fixed64.FromFraction(-5, 4), Fixed64.Zero));
        var landing = new LSCuboidCollider
        {
            Size = new Vector3d(Fixed64.One, Fixed64.FromFraction(7, 4), Fixed64.One)
        };
        scenario.InitializeStaticCollider(
            landing,
            new Vector3d((Fixed64)3, Fixed64.FromFraction(-9, 8), Fixed64.Zero));
        ScenarioBody<LSCylinderCollider> source = scenario.CreateBody(
            new LSCylinderCollider
            {
                Radius = Fixed64.Quarter,
                Size = Vector3d.One * Fixed64.Half
            },
            new Vector3d(Fixed64.Zero, Fixed64.FromFraction(-3, 4), Fixed64.Zero),
            FixedQuaternion.Identity,
            isKinematic: true);
        source.Body.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        DisableGroundQueries(source.Body);
        var hostTarget = new Vector3d(
            Fixed64.FromFraction(1, 64),
            Fixed64.FromFraction(-3, 4),
            Fixed64.Zero);

        source.Body.Agent.Transform.LocalPosition = hostTarget;
        scenario.Context.LateSimulate();

        source.Body.Position3d.Should().Be(hostTarget,
            "the body has not reached the adjacent riser yet");
        source.Body.LastContinuousCollisionToiIterationCount.Should().Be(0);
    }

    [Fact]
    public void ContinuousMode_KinematicCylinderSeparatingFromSupportNearRiser_ShouldReachHostPose()
    {
        using PhysicsScenarioBuilder scenario = CreateCcdScenario();
        var lowerTread = new LSCuboidCollider
        {
            Size = new Vector3d(Fixed64.One, Fixed64.One, Fixed64.One)
        };
        scenario.InitializeStaticCollider(
            lowerTread,
            new Vector3d(Fixed64.Zero, Fixed64.FromFraction(-3, 2), Fixed64.Zero));
        var upperTread = new LSCuboidCollider
        {
            Size = new Vector3d(Fixed64.One, Fixed64.FromFraction(5, 4), Fixed64.One)
        };
        scenario.InitializeStaticCollider(
            upperTread,
            new Vector3d(Fixed64.One, Fixed64.FromFraction(-11, 8), Fixed64.Zero));
        var start = new Vector3d(
            Fixed64.FromFraction(27, 128),
            Fixed64.FromFraction(-3, 4),
            Fixed64.Zero);
        ScenarioBody<LSCylinderCollider> source = scenario.CreateBody(
            new LSCylinderCollider
            {
                Radius = Fixed64.Quarter,
                Size = Vector3d.One * Fixed64.Half
            },
            start,
            FixedQuaternion.Identity,
            isKinematic: true);
        source.Body.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        DisableGroundQueries(source.Body);
        Vector3d hostTarget = start + Vector3d.Up * Fixed64.Quarter;

        source.Body.Agent.Transform.LocalPosition = hostTarget;
        scenario.Context.LateSimulate();

        source.Body.Position3d.Should().Be(hostTarget,
            "motion away from the supporting face must not be clamped by a separated adjacent riser");
        source.Body.LastContinuousCollisionToiIterationCount.Should().Be(0);
    }

    [Fact]
    public void ContinuousMode_KinematicCylinderAcrossMultiGridSupport_ShouldReachHostPose()
    {
        using var world = new GridWorld();
        using GravitasWorldContext context = GravitasWorldContext.Attach(world);
        Fixed64[] layerHeights =
        {
            (Fixed64)2,
            Fixed64.FromFraction(3, 2),
            Fixed64.One,
            Fixed64.Half
        };
        for (int step = 0; step < layerHeights.Length; step++)
        {
            var center = new Vector3d((Fixed64)step, Fixed64.Zero, Fixed64.Zero);
            var configuration = new GridConfiguration(
                center,
                center,
                topologyKind: GridTopologyKind.RectangularPrism,
                topologyMetrics: GridTopologyMetrics.Rectangular(
                    Fixed64.One,
                    layerHeights[step],
                    Fixed64.One));
            world.TryAddGrid(configuration, out _).Should().BeTrue();
        }

        Fixed64[] surfaceLevels =
        {
            -Fixed64.One,
            Fixed64.FromFraction(-3, 4),
            -Fixed64.Half,
            Fixed64.FromFraction(-1, 4)
        };
        var treads = new LSCuboidCollider[surfaceLevels.Length];
        for (int step = 0; step < surfaceLevels.Length; step++)
        {
            Fixed64 height = surfaceLevels[step] + (Fixed64)2;
            treads[step] = new LSCuboidCollider
            {
                Size = new Vector3d(Fixed64.One, height, Fixed64.One)
            };
            treads[step].InitializeWithNoBody(new TestMatterAgent(
                context,
                new FixedTransform(
                    new Vector3d(
                        (Fixed64)step,
                        (surfaceLevels[step] - (Fixed64)2) / (Fixed64)2,
                        Fixed64.Zero),
                    FixedQuaternion.Identity,
                    Vector3d.One)));
        }

        var collider = new LSCylinderCollider
        {
            Radius = Fixed64.Quarter,
            Size = Vector3d.One * Fixed64.Half,
            Layer = new PhysicsLayer(2),
            IgnoredCollisionLayers = PhysicsLayerMask.None
        };
        var start = new Vector3d(
            Fixed64.Zero,
            Fixed64.FromFraction(-3, 4),
            Fixed64.Zero);
        FixedQuaternion rotation = FixedQuaternion.FromDirection(Vector3d.Right);
        var agent = new TestMatterAgent(
            context,
            new FixedTransform(start, rotation, Vector3d.One));
        var body = new SolidBody(agent, collider);
        body.UseManualGrounding();
        body.Initialize(start, rotation, BodyMotionType.Kinematic);
        body.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        var hostTarget = start + Vector3d.Right * Fixed64.FromFraction(1, 64);

        context.Simulate();
        agent.Transform.LocalPosition = hostTarget;
        context.LateSimulate();

        body.Position3d.Should().Be(hostTarget,
            "multi-grid broad-phase membership must not turn remote steps into a closing hit");
        body.LastContinuousCollisionToiIterationCount.Should().Be(0);

        body.Deactivate();
        for (int i = 0; i < treads.Length; i++)
            treads[i].Deactivate();
    }
}
