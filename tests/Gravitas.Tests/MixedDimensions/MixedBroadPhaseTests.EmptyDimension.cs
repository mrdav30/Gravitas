using FixedMathSharp;
using FluentAssertions;
using Gravitas.Colliders;
using Gravitas.Support;
using Gravitas.Tests.Support;
using SwiftCollections;
using Xunit;

namespace Gravitas.Tests.MixedDimensions;

public sealed partial class MixedBroadPhaseTests
{
    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void Simulate_WithAnEmptyDimension_ShouldKeepQueriesFreshAndRetirePartitions(bool planar)
    {
        using GravitasWorldContext context = CreateMixedContext();
        context.Settings.RetainedPartitionTimeToKillFrames = 1;
        context.Settings.RetainedPartitionRetirementSweepBudget = 1024;
        SolidBody? body3D = planar ? null : CreateSphere3D(context, Vector3d.Zero, immovable: true).Body;
        SolidBody2D? body2D = planar ? CreateCircle2D(context, Vector2d.Zero, immovable: true) : null;
        var hits3D = new SwiftList<LSCollider>();
        var hits2D = new SwiftList<LSCollider2D>();
        Step(context);
        uint version = context.MixedCollisions.Version;

        if (planar) body2D!.SetPosition(new Vector2d(12, 0));
        else body3D!.SetPosition(new Vector3d(12, 0, 0));
        Step(context);

        context.MixedCollisions.Version.Should().BeGreaterThan(version);
        context.MixedCollisions.LastBroadPhaseCandidateCount.Should().Be(0);
        context.MixedCollisions.ActivePairCount.Should().Be(0);
        Collect(Vector3d.Zero);
        hits3D.Count.Should().Be(0);
        hits2D.Count.Should().Be(0);
        Collect(new Vector3d(12, 0, 0));
        if (planar)
        {
            hits2D.Count.Should().Be(1);
            hits2D[0].Should().BeSameAs(body2D!.Collider);
        }
        else
        {
            hits3D.Count.Should().Be(1);
            hits3D[0].Should().BeSameAs(body3D!.Collider);
        }

        if (planar) body2D!.Collider.Deactivate();
        else body3D!.Collider.Deactivate();
        Step(context);
        context.MixedCollisions.ActivePartitionCount.Should().Be(0);
        context.MixedCollisions.RetainedPartitionCount.Should().Be(0);
        context.MixedCollisions.InactivePartitionCount.Should().BeGreaterThan(0);

        void Collect(Vector3d center)
        {
            context.MixedCollisions.Collect3DCandidatesInMixedBounds(center - Vector3d.One, center + Vector3d.One,
                PhysicsLayerMask.FromLayer(0), hits3D, staticStyleOnly: false, cachePartitionRefresh: true);
            context.MixedCollisions.Collect2DCandidatesInMixedBounds(center - Vector3d.One, center + Vector3d.One,
                PhysicsLayerMask.FromLayer(0), hits2D, staticStyleOnly: false, cachePartitionRefresh: true);
        }
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void Simulate_WhenTheLastOppositeColliderIsRemovedInACallback_ShouldResumeAfterRespawn(bool removePlanar)
    {
        using GravitasWorldContext context = CreateMixedContext();
        LSSphereCollider collider3D = CreateBodylessSphere3D(context, Vector3d.Zero);
        IMatterAgent agent3D = collider3D.AgentOrNull!;
        SolidBody2D body2D = CreateCircle2D(context, Vector2d.Zero, immovable: false);
        collider3D.IsTrigger = true;
        int enters = 0, exits = 0;
        collider3D.OnMixedTriggerExit += _ => exits++;
        collider3D.OnMixedTriggerEnter += _ =>
        {
            enters++;
            if (removePlanar) body2D.Collider.Deactivate();
            else collider3D.Deactivate();
        };

        for (int cycle = 0; cycle < 3; cycle++)
        {
            if (cycle > 0)
            {
                if (removePlanar) body2D.Initialize(Vector2d.Zero);
                else collider3D.InitializeWithNoBody(agent3D);
            }
            Step(context);
            context.MixedCollisions.ActivePairCount.Should().Be(0);
            context.MixedCollisions.LastBroadPhaseCandidateCount.Should().Be(1);
            enters.Should().Be(cycle + 1);
            exits.Should().Be(cycle + 1);
            Step(context);
            context.MixedCollisions.LastBroadPhaseCandidateCount.Should().Be(0);
            context.MixedCollisions.ActivePairCount.Should().Be(0);
        }
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void Simulate_WithAnEmptyOppositeDimension_ShouldNotAllocateAfterWarmup(bool planar)
    {
        using GravitasWorldContext context = CreateMixedContext();
        if (planar) CreateCircle2D(context, Vector2d.Zero, immovable: false);
        else CreateSphere3D(context, Vector3d.Zero, immovable: false);
        AllocationTestHelper.MeasureSteadyState(() => Step(context)).Should().Be(0);
    }
}
