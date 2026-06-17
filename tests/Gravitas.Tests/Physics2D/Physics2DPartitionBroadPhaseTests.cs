using FixedMathSharp;
using FluentAssertions;
using Gravitas.Colliders;
using Gravitas.Queries;
using Gravitas.Support;
using Gravitas.Tests.Support;
using GridForge.Configuration;
using SwiftCollections;
using System;
using Xunit;

namespace Gravitas.Tests.Physics2D;

public sealed class Physics2DPartitionBroadPhaseTests
{
    [Fact]
    public void Simulate_WithSparseScene_ShouldProcessOnlyPartitionCandidates()
    {
        using GravitasWorldContext context = CreateContext(extent: 512);
        StiffBody2D dynamicBody = CreateCircle(context, Vector2d.Zero, immovable: false);
        _ = CreateCircle(context, new Vector2d(Fixed64.Half, Fixed64.Zero), immovable: true);

        for (int i = 0; i < 160; i++)
            _ = CreateCircle(context, new Vector2d((Fixed64)(16 + (i * 3)), (Fixed64)32), immovable: true);

        context.Simulate();

        dynamicBody.Position.X.Should().BeLessThan(Fixed64.Zero);
        context.Physics2D.LastBroadPhaseCandidateCount.Should().Be(1);
        context.Collisions2D.ActivePartitionCount.Should().BeGreaterThan(0);
    }

    [Fact]
    public void OverlapCircleAll_WithColliderSpanningMultipleVoxels_ShouldReturnOneHit()
    {
        using GravitasWorldContext context = CreateContext(extent: 32);
        StiffBody2D body = CreateBody(
            context,
            new LSAABBoxCollider2D(new Vector2d((Fixed64)8, (Fixed64)8)),
            Vector2d.Zero,
            immovable: true);
        var hits = new SwiftList<Physics2DHit>();

        int count = context.Query2D.OverlapCircleAll(Vector2d.Zero, (Fixed64)6, hits);

        count.Should().Be(1);
        hits[0].Collider.Should().BeSameAs(body.Collider);
        context.Query2D.LastQueryCandidateCount.Should().Be(1);
    }

    [Fact]
    public void MovingBody_ShouldLeaveOldPartitionsAndEnterNewPartitions()
    {
        using GravitasWorldContext context = CreateContext(extent: 32);
        StiffBody2D body = CreateCircle(context, Vector2d.Zero, immovable: false);
        var hits = new SwiftList<Physics2DHit>();

        context.Query2D.OverlapCircleAll(Vector2d.Zero, Fixed64.One, hits).Should().Be(1);

        body.SetPosition(new Vector2d((Fixed64)10, Fixed64.Zero));

        context.Query2D.OverlapCircleAll(Vector2d.Zero, Fixed64.One, hits).Should().Be(0);
        context.Query2D.OverlapCircleAll(new Vector2d((Fixed64)10, Fixed64.Zero), Fixed64.One, hits).Should().Be(1);
        hits[0].Collider.Should().BeSameAs(body.Collider);
    }

    [Fact]
    public void Deactivate_WithRetainedPartitionTtk_ShouldRetireAndPoolEmpty2DPartitions()
    {
        using GravitasWorldContext context = CreateContext(extent: 16);
        context.Settings.RetainedPartitionTimeToKillFrames = 1;
        context.Settings.RetainedPartitionRetirementSweepBudget = 1024;
        StiffBody2D body = CreateCircle(context, Vector2d.Zero, immovable: false);
        int retainedBeforeDeactivate = context.Collisions2D.RetainedPartitionCount;

        body.Deactivate();
        context.Simulate();

        context.Collisions2D.ActivePartitionCount.Should().Be(0);
        context.Collisions2D.RetainedPartitionCount.Should().BeLessThan(retainedBeforeDeactivate);
        context.Collisions2D.InactivePartitionCount.Should().BeGreaterThan(0);
    }

    [Fact]
    public void Simulate_WithOnlySleepingDynamicAndStaticObjects_ShouldSkipPartitionWork()
    {
        using GravitasWorldContext context = CreateContext(extent: 16);
        StiffBody2D sleeper = CreateCircle(context, Vector2d.Zero, immovable: false);
        _ = CreateCircle(context, new Vector2d(Fixed64.Half, Fixed64.Zero), immovable: true);
        sleeper.Sleep();

        context.Simulate();

        sleeper.IsSleeping.Should().BeTrue();
        context.Physics2D.LastBroadPhaseCandidateCount.Should().Be(0);
    }

    [Fact]
    public void Simulate_WhenExistingSolidPairFallsAsleep_ShouldRetainRestingContactWithoutExit()
    {
        using GravitasWorldContext context = CreateContext(extent: 16);
        StiffBody2D sleeper = CreateCircle(context, Vector2d.Zero, immovable: false);
        _ = CreateCircle(context, new Vector2d((Fixed64)0.75f, Fixed64.Zero), immovable: true);
        int exited = 0;
        sleeper.Collider.OnContactExit += _ => exited++;

        context.Simulate();
        sleeper.Sleep();
        context.Simulate();

        sleeper.IsSleeping.Should().BeTrue();
        exited.Should().Be(0);
        context.Physics2D.LastBroadPhaseCandidateCount.Should().Be(0);
    }

    [Fact]
    public void Simulate_WithAwakeDynamicTouchingSleepingDynamic_ShouldWakeSleeper()
    {
        using GravitasWorldContext context = CreateContext(extent: 16);
        StiffBody2D sleeper = CreateCircle(context, Vector2d.Zero, immovable: false);
        _ = CreateCircle(context, new Vector2d(Fixed64.Half, Fixed64.Zero), immovable: false);
        sleeper.Sleep();

        context.Simulate();

        sleeper.IsSleeping.Should().BeFalse();
        context.Physics2D.LastBroadPhaseCandidateCount.Should().Be(1);
    }

    [Fact]
    public void ReplayedMovingPartitionScenario_ShouldProduceSame2DStateAndCandidateCounts()
    {
        ReplayResult first = RunReplayScenario();
        ReplayResult second = RunReplayScenario();

        second.Should().Be(first);
    }

    [Fact]
    public void Simulate_WithDenseOverlappingPairs_ShouldNotAllocateAfterWarmup()
    {
        using GravitasWorldContext context = CreateContext(extent: 128);
        var bodies = new SwiftList<StiffBody2D>();
        for (int i = 0; i < 64; i++)
        {
            Vector2d position = PositionForIndex(i, spacing: (Fixed64)2);
            bodies.Add(CreateCircle(context, position, immovable: false));
            _ = CreateCircle(context, position + new Vector2d(Fixed64.Half, Fixed64.Zero), immovable: true);
        }

        for (int i = 0; i < 3; i++)
        {
            ResetBodyPositions(bodies);
            context.Simulate();
        }

        long allocatedBytes = MeasureAllocatedBytes(() =>
        {
            ResetBodyPositions(bodies);
            context.Simulate();
        });

        allocatedBytes.Should().Be(0);
    }

    private static ReplayResult RunReplayScenario()
    {
        using GravitasWorldContext context = CreateContext(extent: 256, frameRate: 8);
        var bodies = new SwiftList<StiffBody2D>();
        for (int i = 0; i < 48; i++)
            bodies.Add(CreateCircle(context, new Vector2d((Fixed64)(i * 3), Fixed64.Zero), immovable: false));

        _ = CreateCircle(context, new Vector2d((Fixed64)12, Fixed64.Half), immovable: true);
        int candidateTotal = 0;
        for (int frame = 0; frame < 8; frame++)
        {
            StiffBody2D moved = bodies[frame % bodies.Count];
            moved.SetPosition(moved.Position + new Vector2d(Fixed64.Half, Fixed64.Zero));
            context.Simulate();
            context.LateSimulate();
            candidateTotal += context.Physics2D.LastBroadPhaseCandidateCount;
        }

        StiffBody2D body = bodies[4];
        return new ReplayResult(body.Position, body.LinearVelocity, candidateTotal, context.Collisions2D.RetainedPartitionCount);
    }

    private static GravitasWorldContext CreateContext(int extent, int frameRate = 4)
    {
        GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        context.SetFrameRate(frameRate);
        context.Settings.RuntimeMode = PhysicsRuntimeMode.TwoD;
        context.World.TryAddGrid(
            new GridConfiguration(
                new Vector3d((Fixed64)(-16), Fixed64.Zero, (Fixed64)(-16)),
                new Vector3d((Fixed64)extent, Fixed64.Zero, (Fixed64)extent)),
            out _).Should().BeTrue();
        return context;
    }

    private static StiffBody2D CreateCircle(GravitasWorldContext context, Vector2d position, bool immovable)
    {
        return CreateBody(context, new LSCircleCollider2D(Fixed64.Half), position, immovable);
    }

    private static StiffBody2D CreateBody(
        GravitasWorldContext context,
        LSCollider2D collider,
        Vector2d position,
        bool immovable)
    {
        var transform = new FixedTransform(
            new Vector3d(position.X, Fixed64.Zero, position.Y),
            FixedQuaternion.Identity,
            Vector3d.One);
        var agent = new TestMatterAgent(context, transform);
        var body = new StiffBody2D(agent, collider)
        {
            Mass = Fixed64.One,
            Immovable = immovable
        };
        body.Initialize(position);
        return body;
    }

    private static Vector2d PositionForIndex(int index, Fixed64 spacing)
    {
        int width = 8;
        int x = index % width;
        int y = index / width;
        return new Vector2d((Fixed64)x * spacing, (Fixed64)y * spacing);
    }

    private static void ResetBodyPositions(SwiftList<StiffBody2D> bodies)
    {
        for (int i = 0; i < bodies.Count; i++)
            bodies[i].SetPosition(PositionForIndex(i, spacing: (Fixed64)2));
    }

    private static long MeasureAllocatedBytes(Action action)
    {
        GC.Collect();
        GC.WaitForPendingFinalizers();
        GC.Collect();

        long before = GC.GetAllocatedBytesForCurrentThread();
        action();
        return GC.GetAllocatedBytesForCurrentThread() - before;
    }

    private readonly record struct ReplayResult(
        Vector2d Position,
        Vector2d Velocity,
        int CandidateTotal,
        int RetainedPartitionCount);
}
