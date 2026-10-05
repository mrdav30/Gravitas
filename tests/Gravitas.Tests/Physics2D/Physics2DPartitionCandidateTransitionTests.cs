using FixedMathSharp;
using FluentAssertions;
using Gravitas.Colliders;
using Gravitas.CollisionHandling;
using Gravitas.Support;
using Gravitas.Tests.Support;
using GridForge.Configuration;
using GridForge.Grids.Topology;
using Xunit;

namespace Gravitas.Tests.Physics2D;

public sealed class Physics2DPartitionCandidateTransitionTests
{
    [Fact]
    public void PartitionCandidate_RejectedThenTouchingInSamePass_ShouldAcceptOnlyOnce()
    {
        using GravitasWorldContext context = Physics2DTestWorld.CreateContext();
        context.World.TryAddGrid(new GridConfiguration(
            new Vector3d(96, 0, 96), new Vector3d(128, 0, 128),
            topologyMetrics: GridTopologyMetrics.Rectangular((Fixed64)16, Fixed64.One, (Fixed64)16)), out _)
            .Should().BeTrue();
        SolidBody2D moving = CreateCircle(context, new Vector2d(113, 116), BodyMotionType.Dynamic);
        SolidBody2D target = CreateCircle(context, new Vector2d(119, 116), BodyMotionType.Static);
        var coordinate = moving.Collider.PartitionCoordinates![0];
        int enters = 0;
        moving.Collider.OnContactEnter += _ => enters++;

        // A rejected link must not reserve its key: later collider changes in
        // this same distribution pass can make the original pair eligible.
        context.Physics2D.ProcessPartitionCandidate(moving.Collider.Id, target.Collider.Id, coordinate);
        context.Physics2D.LastBroadPhaseCandidateCount.Should().Be(0);
        moving.ResetPosition(new Vector2d(118, 116), Fixed64.Zero);
        context.Physics2D.ProcessPartitionCandidate(moving.Collider.Id, target.Collider.Id, coordinate);
        context.Physics2D.ProcessPartitionCandidate(moving.Collider.Id, target.Collider.Id, coordinate);

        context.Physics2D.LastBroadPhaseCandidateCount.Should().Be(1);
        enters.Should().Be(1);
    }

    [Fact]
    public void SharedVoxelPair_MovingFromSeparatedBoundsThroughContactAndBack_ShouldPreserveCandidatesAndEvents()
    {
        using GravitasWorldContext context = Physics2DTestWorld.CreateContext();
        context.World.TryAddGrid(new GridConfiguration(
            new Vector3d(96, 0, 96), new Vector3d(128, 0, 128),
            topologyMetrics: GridTopologyMetrics.Rectangular((Fixed64)16, Fixed64.One, (Fixed64)16)), out _)
            .Should().BeTrue();
        SolidBody2D moving = CreateCircle(context, new Vector2d(113, 116), BodyMotionType.Dynamic);
        SolidBody2D target = CreateCircle(context, new Vector2d(119, 116), BodyMotionType.Static);
        moving.Collider.PartitionCoordinates!.Count.Should().Be(1);
        target.Collider.PartitionCoordinates!.Count.Should().Be(1);
        moving.Collider.PartitionCoordinates[0].Should().Be(target.Collider.PartitionCoordinates[0]);
        int enters = 0;
        int exits = 0;
        moving.Collider.OnContactEnter += other =>
        {
            other.Should().BeSameAs(target);
            enters++;
        };
        moving.Collider.OnContactExit += other =>
        {
            other.Should().BeSameAs(target);
            exits++;
        };

        Step(context);

        context.Physics2D.LastBroadPhaseCandidateCount.Should().Be(0);
        enters.Should().Be(0);
        exits.Should().Be(0);

        moving.ResetPosition(new Vector2d(118, 116), Fixed64.Zero);
        CollisionDetection2D.BoundsOverlap(moving.Collider, target.Collider).Should().BeTrue();
        CollisionDetection2D.TryCollide(moving.Collider, target.Collider, out Contact2D touching).Should().BeTrue();
        touching.Depth.Should().Be(Fixed64.Zero);

        Step(context);

        context.Physics2D.LastBroadPhaseCandidateCount.Should().Be(1);
        enters.Should().Be(1);
        exits.Should().Be(0);

        moving.ResetPosition(new Vector2d((Fixed64)118 + Fixed64.Half, (Fixed64)116), Fixed64.Zero);

        Step(context);

        context.Physics2D.LastBroadPhaseCandidateCount.Should().Be(1);
        enters.Should().Be(1);
        exits.Should().Be(0);

        moving.ResetPosition(new Vector2d(113, 116), Fixed64.Zero);

        Step(context);

        context.Physics2D.LastBroadPhaseCandidateCount.Should().Be(0);
        enters.Should().Be(1);
        exits.Should().Be(1);
        moving.Collider.PartitionCoordinates![0].Should().Be(target.Collider.PartitionCoordinates![0]);

        moving.Collider.IgnoredCollisionLayers = PhysicsLayerMask.FromLayer(target.Collider.Layer);
        moving.ResetPosition(new Vector2d(118, 116), Fixed64.Zero);

        Step(context);

        context.Physics2D.LastBroadPhaseCandidateCount.Should().Be(0);
        enters.Should().Be(1);
        exits.Should().Be(1);
    }

    private static SolidBody2D CreateCircle(GravitasWorldContext context, Vector2d position, BodyMotionType motionType)
    {
        var transform = new FixedTransform(new Vector3d(position.X, Fixed64.Zero, position.Y),
            FixedQuaternion.Identity, Vector3d.One);
        var body = new SolidBody2D(new TestMatterAgent(context, transform), new LSCircleCollider2D(Fixed64.Half))
        {
            Mass = Fixed64.One,
            SleepEnabled = false
        };
        body.Initialize(position, motionType: motionType);
        body.UseManualGrounding();
        return body;
    }

    private static void Step(GravitasWorldContext context)
    {
        context.Simulate();
        context.LateSimulate();
    }
}
