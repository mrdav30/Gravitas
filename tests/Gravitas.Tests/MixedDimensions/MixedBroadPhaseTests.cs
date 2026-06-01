using FixedMathSharp;
using FluentAssertions;
using Gravitas.Colliders;
using Gravitas.Support;
using Gravitas.Tests.Support;
using GridForge.Configuration;
using SwiftCollections;
using Xunit;

namespace Gravitas.Tests.MixedDimensions;

public sealed class MixedBroadPhaseTests
{
    [Fact]
    public void Simulate_WithSparseMixedOverlap_ShouldEmitStableCandidateKey()
    {
        using GravitasWorldContext context = CreateMixedContext();
        ScenarioBody<LSSphereCollider> body3D = CreateSphere3D(context, Vector3d.Zero, immovable: false);
        StiffBody2D body2D = CreateCircle2D(context, Vector2d.Zero, immovable: true);

        context.Simulate();

        context.MixedCollisions.LastBroadPhaseCandidateCount.Should().Be(1);
        MixedColliderKey candidate = context.MixedCollisions.GetCandidate(0);
        candidate.Collider3DId.Should().Be(body3D.Collider.Id);
        candidate.Collider2DId.Should().Be(body2D.Collider.Id);
        context.MixedCollisions.ActivePartitionCount.Should().BeGreaterThan(0);
    }

    [Fact]
    public void Simulate_WithDenseMixedOverlap_ShouldEmitEachPairOnceInDeterministicOrder()
    {
        using GravitasWorldContext context = CreateMixedContext();
        var bodies3D = new SwiftList<ScenarioBody<LSSphereCollider>>();
        var bodies2D = new SwiftList<StiffBody2D>();

        for (int i = 0; i < 4; i++)
        {
            bodies3D.Add(CreateSphere3D(context, Vector3d.Zero, immovable: false));
            bodies2D.Add(CreateCircle2D(context, Vector2d.Zero, immovable: false));
        }

        context.Simulate();

        context.MixedCollisions.LastBroadPhaseCandidateCount.Should().Be(16);
        ulong previousKey = 0;
        for (int i = 0; i < context.MixedCollisions.LastBroadPhaseCandidateCount; i++)
        {
            MixedColliderKey candidate = context.MixedCollisions.GetCandidate(i);
            if (i > 0)
                candidate.Key.Should().BeGreaterThan(previousKey);

            previousKey = candidate.Key;
        }
    }

    [Fact]
    public void Simulate_WithLargeSparseMixedWorld_ShouldCullFarCrossDimensionPairs()
    {
        using GravitasWorldContext context = CreateMixedContext(extent: 128);
        _ = CreateSphere3D(context, Vector3d.Zero, immovable: false);
        _ = CreateCircle2D(context, Vector2d.Zero, immovable: true);

        for (int i = 0; i < 32; i++)
        {
            Fixed64 offset = (Fixed64)(8 + (i * 2));
            _ = CreateSphere3D(context, new Vector3d(offset, Fixed64.Zero, (Fixed64)48), immovable: false);
            _ = CreateCircle2D(context, new Vector2d(offset, (Fixed64)(-48)), immovable: true);
        }

        context.Simulate();

        context.MixedCollisions.LastBroadPhaseCandidateCount.Should().Be(1);
    }

    [Fact]
    public void Simulate_WithSleepingMixedDynamics_ShouldSkipUntilOneParticipantWakes()
    {
        using GravitasWorldContext context = CreateMixedContext();
        ScenarioBody<LSSphereCollider> body3D = CreateSphere3D(context, Vector3d.Zero, immovable: false);
        StiffBody2D body2D = CreateCircle2D(context, Vector2d.Zero, immovable: false);
        body3D.Body.Sleep();
        body2D.Sleep();

        context.Simulate();

        context.MixedCollisions.LastBroadPhaseCandidateCount.Should().Be(0);

        body3D.Body.Wake();
        context.Simulate();

        context.MixedCollisions.LastBroadPhaseCandidateCount.Should().Be(1);
    }

    [Fact]
    public void Simulate_WithTriggerLayerAndSameAgentFilters_ShouldApplyMixedBroadPhaseRules()
    {
        using GravitasWorldContext context = CreateMixedContext();
        var allowed3D = CreateSphere3D(context, Vector3d.Zero, immovable: false);
        StiffBody2D trigger2D = CreateCircle2D(context, Vector2d.Zero, immovable: true);
        trigger2D.Collider.IsTrigger = true;

        var blocked3D = CreateSphere3D(context, new Vector3d((Fixed64)4, Fixed64.Zero, Fixed64.Zero), immovable: false, layer: new PhysicsLayer(1));
        _ = CreateCircle2D(context, new Vector2d((Fixed64)4, Fixed64.Zero), immovable: true, layer: new PhysicsLayer(2));

        IMatterAgent sharedAgent = new TestMatterAgent(
            context,
            new FixedTransform(new Vector3d((Fixed64)8, Fixed64.Zero, Fixed64.Zero), FixedQuaternion.Identity, Vector3d.One));
        var sameAgent3D = CreateSphere3D(context, sharedAgent, immovable: false);
        _ = CreateCircle2D(context, sharedAgent, immovable: true);

        context.Simulate();

        context.MixedCollisions.LastBroadPhaseCandidateCount.Should().Be(1);
        MixedColliderKey candidate = context.MixedCollisions.GetCandidate(0);
        candidate.Collider3DId.Should().Be(allowed3D.Collider.Id);
        candidate.Collider2DId.Should().Be(trigger2D.Collider.Id);
        candidate.Collider3DId.Should().NotBe(blocked3D.Collider.Id);
        candidate.Collider3DId.Should().NotBe(sameAgent3D.Collider.Id);
    }

    [Fact]
    public void Simulate_WithRetainedMixedPartitions_ShouldRetireAndPoolAfterTtk()
    {
        using GravitasWorldContext context = CreateMixedContext();
        context.Settings.RetainedPartitionTimeToKillFrames = 1;
        context.Settings.RetainedPartitionRetirementSweepBudget = 1024;
        ScenarioBody<LSSphereCollider> body3D = CreateSphere3D(context, Vector3d.Zero, immovable: false);
        StiffBody2D body2D = CreateCircle2D(context, Vector2d.Zero, immovable: false);

        context.Simulate();
        int retainedBeforeDeactivate = context.MixedCollisions.RetainedPartitionCount;

        body3D.Collider.Deactivate();
        body2D.Collider.Deactivate();
        context.Simulate();

        context.MixedCollisions.ActivePartitionCount.Should().Be(0);
        context.MixedCollisions.RetainedPartitionCount.Should().BeLessThan(retainedBeforeDeactivate);
        context.MixedCollisions.InactivePartitionCount.Should().BeGreaterThan(0);
    }

    private static GravitasWorldContext CreateMixedContext(int extent = 32)
    {
        GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        context.ApplySettings(new PhysicsSettings(
            4,
            new[,]
            {
                { true, true, true },
                    { true, true, false },
                    { true, false, true }
                }));
        context.Settings.RuntimeMode = PhysicsRuntimeMode.Mixed;
        context.World.TryAddGrid(
            new GridConfiguration(
                new Vector3d((Fixed64)(-extent), (Fixed64)(-4), (Fixed64)(-extent)),
                new Vector3d((Fixed64)extent, (Fixed64)4, (Fixed64)extent)),
            out _).Should().BeTrue();
        return context;
    }

    private static ScenarioBody<LSSphereCollider> CreateSphere3D(
        GravitasWorldContext context,
        Vector3d position,
        bool immovable,
        PhysicsLayer? layer = null)
    {
        var transform = new FixedTransform(position, FixedQuaternion.Identity, Vector3d.One);
        var agent = new TestMatterAgent(context, transform);
        return CreateSphere3D(context, agent, immovable, layer);
    }

    private static ScenarioBody<LSSphereCollider> CreateSphere3D(
        GravitasWorldContext context,
        IMatterAgent agent,
        bool immovable,
        PhysicsLayer? layer = null)
    {
        var collider = new LSSphereCollider();
        if (layer.HasValue)
            collider.Layer = layer.Value;

        var body = new StiffBody(agent, collider)
        {
            Mass = Fixed64.One,
            Immovable = immovable
        };
        body.Initialize(agent.Transform.Position, agent.Transform.Rotation);
        return new ScenarioBody<LSSphereCollider>(body, collider);
    }

    private static StiffBody2D CreateCircle2D(
        GravitasWorldContext context,
        Vector2d position,
        bool immovable,
        PhysicsLayer? layer = null)
    {
        var transform = new FixedTransform(
            new Vector3d(position.x, Fixed64.Zero, position.y),
            FixedQuaternion.Identity,
            Vector3d.One);
        var agent = new TestMatterAgent(context, transform);
        return CreateCircle2D(context, agent, immovable, layer);
    }

    private static StiffBody2D CreateCircle2D(
        GravitasWorldContext context,
        IMatterAgent agent,
        bool immovable,
        PhysicsLayer? layer = null)
    {
        var collider = new LSCircleCollider2D(Fixed64.Half);
        if (layer.HasValue)
            collider.Layer = layer.Value;

        var body = new StiffBody2D(agent, collider)
        {
            Mass = Fixed64.One,
            Immovable = immovable
        };
        body.Initialize(agent.Transform.Position.ToVector2d());
        return body;
    }
}
