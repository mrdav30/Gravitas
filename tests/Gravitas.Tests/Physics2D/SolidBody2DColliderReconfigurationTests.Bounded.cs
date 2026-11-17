using System;
using FixedMathSharp;
using FluentAssertions;
using Gravitas.Colliders;
using Gravitas.Constraints;
using Gravitas.Queries;
using Gravitas.Tests.Support;
using Xunit;

namespace Gravitas.Tests.Physics2D;

public sealed partial class SolidBody2DColliderReconfigurationTests
{
    [Theory]
    [InlineData(-1_000_000_000, 2, 500_000_000, 1_500_000_000, 0)]
    [InlineData(1_000_000_000, 1, -1_000_000_000, -1_500_000_000, 0)]
    [InlineData(1_000_000_000, 1, 0, 1_500_000_000, -1_000_000_000)]
    public void UnrepresentableHostPoseLeavesTheCommittedPlanarPostureUnchanged(
        int parentX, int scale, int localX, int positionX, int grandparentX)
    {
        using GravitasWorldContext context = CreateContext(withGrid: false);
        // Opposing ancestors can compose on their own while the requested child overflows in between.
        FixedTransform? grandparent = grandparentX == 0 ? null :
            new FixedTransform(new Vector3d(grandparentX, 0, 0), FixedQuaternion.Identity, Vector3d.One);
        var parent = new FixedTransform(new Vector3d(parentX, 0, 0),
            FixedQuaternion.Identity, Vector3d.One * (Fixed64)scale, grandparent);
        var localPosition = new Vector3d(localX, 0, 0);
        var transform = new FixedTransform(localPosition, FixedQuaternion.Identity, Vector3d.One, parent);
        var body = new SolidBody2D(new TestMatterAgent(context, transform), new LSCircleCollider2D(Fixed64.Half))
            { Mass = Fixed64.One };
        body.Initialize(Vector2d.Zero, motionType: PhysicsMobilityType.Kinematic);
        var before = context.ComputeReplayHash();
        var position = new Vector2d(positionX, 0);
        var budget = new PhysicsQueryBudget(2, 32, 32);
        int publications = 0;

        body.TryReconfigureCollider(ColliderShapeDefinition2D.Circle(Fixed64.One), Vector2d.Zero, position, budget,
            new PhysicsQueryScratch(0, 0), out var result, out var blocker, out var failure, out _, () => publications++)
            .Should().Be(PhysicsQueryExecutionStatus.Unrepresentable);

        result.Should().BeNull();
        blocker.Should().BeNull();
        failure.Should().BeNull();
        publications.Should().Be(0);
        body.Position.Should().Be(Vector2d.Zero);
        transform.LocalPosition.Should().Be(localPosition);
        context.ComputeReplayHash().Should().Be(before);
    }

    [Fact]
    public void BoundedPostureRequiresPureRuntimeAndPublishedPhysics()
    {
        using GravitasWorldContext context = CreateContext();
        var body = CreateBody(context, new LSCircleCollider2D(Fixed64.Half));
        var scratch = new PhysicsQueryScratch(0, 0);
        var posture = ColliderShapeDefinition2D.Circle(Fixed64.One);
        var budget = new PhysicsQueryBudget(2, 8, 8);

        context.Settings.RuntimeMode = PhysicsRuntimeMode.Both;
        Reject(PhysicsQueryExecutionStatus.UnsupportedRuntimeMode);
        context.Settings.RuntimeMode = PhysicsRuntimeMode.TwoD;
        context.Simulate();
        Reject(PhysicsQueryExecutionStatus.WorldNotReady);
        context.LateSimulate();

        body.TryReconfigureCollider(posture, Vector2d.Zero, body.Position, budget, scratch,
            out var result, out _, out _, out _).Should().Be(PhysicsQueryExecutionStatus.Completed);
        result.Should().Be(ColliderReconfigurationStatus.Applied);

        void Reject(PhysicsQueryExecutionStatus expected)
        {
            var before = context.ComputeReplayHash();
            body.TryReconfigureCollider(posture, Vector2d.Zero, body.Position, budget, scratch,
                out var outcome, out var blocker, out var failure, out var work)
                .Should().Be(expected);
            outcome.Should().BeNull();
            blocker.Should().BeNull();
            failure.Should().BeNull();
            work.Should().Be(default(PhysicsQueryWork));
            context.ComputeReplayHash().Should().Be(before);
        }
    }

    [Fact]
    public void BoundedPostureRejectsUnrepresentableAttachmentAndReusesScratch()
    {
        using GravitasWorldContext context = CreateContext(withGrid: false);
        var body = CreateBody(context, new LSCircleCollider2D(Fixed64.Half));
        var scratch = new PhysicsQueryScratch(0, 0);
        var posture = ColliderShapeDefinition2D.Circle(Fixed64.One);
        var budget = new PhysicsQueryBudget(2, 8, 8);
        var before = context.ComputeReplayHash();

        body.TryReconfigureCollider(posture, new Vector2d(Fixed64.MaxValue, Fixed64.Zero), Vector2d.Right,
            budget, scratch, out var result, out var blocker, out var failure, out _)
            .Should().Be(PhysicsQueryExecutionStatus.Unrepresentable);
        result.Should().BeNull();
        blocker.Should().BeNull();
        failure.Should().BeNull();
        context.ComputeReplayHash().Should().Be(before);

        body.TryReconfigureCollider(posture, Vector2d.Zero, Vector2d.Right, budget, scratch,
            out result, out _, out _, out _).Should().Be(PhysicsQueryExecutionStatus.Completed);
        result.Should().Be(ColliderReconfigurationStatus.Applied);
    }

    [Fact]
    public void BoundedPostureChargesTheLinkedPairProbeBeforeSkippingItsBlocker()
    {
        using GravitasWorldContext context = CreateContext(withGrid: false);
        var body = CreateBody(context, new LSCircleCollider2D(Fixed64.Half));
        var linked = CreateBody(context, new LSCircleCollider2D(Fixed64.Half), Vector2d.Right);
        context.Constraints2D.RegisterJoint(new JointDefinition2D(body, linked,
            JointFrame2D.Identity, JointFrame2D.Identity, JointType2D.Pin, JointLimit2D.Unrestricted,
            JointMotor2D.Disabled, JointCollisionPolicy.SuppressLinked));
        var scratch = new PhysicsQueryScratch(0, 0);
        var posture = ColliderShapeDefinition2D.Circle(Fixed64.One);
        var before = context.ComputeReplayHash();

        body.TryReconfigureCollider(posture, Vector2d.Zero, Vector2d.Zero, new PhysicsQueryBudget(2, 2, 1),
            scratch, out var result, out var blocker, out var failure, out var work)
            .Should().Be(PhysicsQueryExecutionStatus.WorkBudgetExceeded);
        result.Should().BeNull();
        blocker.Should().BeNull();
        failure.Should().BeNull();
        work.Should().Be(new PhysicsQueryWork(1, 2, 1));
        context.ComputeReplayHash().Should().Be(before);

        body.TryReconfigureCollider(posture, Vector2d.Zero, Vector2d.Zero, new PhysicsQueryBudget(2, 3, 1),
            scratch, out result, out blocker, out failure, out work).Should().Be(PhysicsQueryExecutionStatus.Completed);
        result.Should().Be(ColliderReconfigurationStatus.Applied);
        blocker.Should().BeNull();
        failure.Should().BeNull();
        work.Should().Be(new PhysicsQueryWork(2, 3, 1));
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void BoundedPosture_ShouldKeepFirstBlockerAndWorkAcrossRegistrationReuse(bool capsuleBlockers)
    {
        using GravitasWorldContext context = CreateContext(withGrid: false);
        SolidBody2D body = CreateBody(context, new LSCircleCollider2D(Fixed64.Half));
        var removed = CreateObstacle(context, new LSCircleCollider2D(Fixed64.Half), new Vector2d(8, 0));
        var older = CreateObstacle(context, BlockerShape(), Vector2d.Right);
        CreateObstacle(context, BlockerShape(), -Vector2d.Right);
        int retiredId = removed.Id;
        removed.Deactivate();
        var replacement = CreateObstacle(context, BlockerShape(), Vector2d.Right);
        replacement.Id.Should().Be(retiredId);

        using GravitasWorldContext rebuilt = CreateContext(withGrid: false);
        SolidBody2D rebuiltBody = CreateBody(rebuilt, new LSCircleCollider2D(Fixed64.Half));
        var rebuiltOlder = CreateObstacle(rebuilt, BlockerShape(), Vector2d.Right);
        CreateObstacle(rebuilt, BlockerShape(), -Vector2d.Right);
        CreateObstacle(rebuilt, BlockerShape(), Vector2d.Right);
        var replay = rebuilt.ComputeReplayHash();
        context.ComputeReplayHash().Should().Be(replay);

        var definition = ColliderShapeDefinition2D.Circle(Fixed64.One);
        var scratch = new PhysicsQueryScratch(0, 0);
        var ample = new PhysicsQueryBudget(2, 32, 32);
        rebuiltBody.TryReconfigureCollider(definition, Vector2d.Zero, Vector2d.Zero, ample, scratch,
            out var expected, out var blocker, out _, out var work).Should().Be(PhysicsQueryExecutionStatus.Completed);
        expected.Should().Be(ColliderReconfigurationStatus.Blocked);
        blocker.Should().BeSameAs(rebuiltOlder);

        var exact = new PhysicsQueryBudget(work.Discovery, work.Candidates, work.Geometry);
        body.TryReconfigureCollider(definition, Vector2d.Zero, Vector2d.Zero, exact, scratch,
            out var result, out blocker, out _, out var actual).Should().Be(PhysicsQueryExecutionStatus.Completed);
        result.Should().Be(expected);
        blocker.Should().BeSameAs(older);
        actual.Should().Be(work);

        var insufficient = new PhysicsQueryBudget(work.Discovery, work.Candidates - 1, work.Geometry);
        body.TryReconfigureCollider(definition, Vector2d.Zero, Vector2d.Zero, insufficient, scratch,
            out result, out blocker, out _, out _).Should().Be(PhysicsQueryExecutionStatus.WorkBudgetExceeded);
        result.Should().BeNull();
        blocker.Should().BeNull();
        context.ComputeReplayHash().Should().Be(replay);

        LSCollider2D BlockerShape() => capsuleBlockers
            ? new LSCapsuleCollider2D(Fixed64.Half, Fixed64.Two)
            : new LSCircleCollider2D(Fixed64.Half);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void BoundedPosture_ShouldHonorExactBudgetsAndPublishOnlyAfterClearance(bool box)
    {
        using GravitasWorldContext context = CreateContext(withGrid: false);
        SolidBody2D body = CreateBody(context, new LSCapsuleCollider2D(Fixed64.Half, Fixed64.One));
        Vector2d[] vertices = { new(-2, -2), new(0, -2), new(0, 0), new(-2, 0) };
        LSCollider2D obstacle = box
            ? CreateObstacle(context, new LSAABBoxCollider2D(new Vector2d(2, 2)), new Vector2d(-1, -1))
            : CreateObstacle(context, new LSCompoundCollider2D(
                new CompoundColliderPart2D(ColliderShapeDefinition2D.ConvexPolygon(vertices), Vector2d.Zero)), Vector2d.Zero);
        obstacle.IsPartitioned.Should().BeFalse();

        var scratch = new PhysicsQueryScratch(0, 0);
        var budget = new PhysicsQueryBudget(2, 32, 128);
        var posture = ColliderShapeDefinition2D.Capsule(Fixed64.One, Fixed64.Two);
        Vector2d touching = Vector2d.Right;
        Vector2d overlapping = touching - Vector2d.Right * Fixed64.FromRaw(1);
        var before = context.ComputeReplayHash();
        int publications = 0;
        Action publish = () => publications++;

        body.TryReconfigureCollider(posture, Vector2d.Zero, overlapping, budget, scratch,
            out var result, out var blocker, out var failure, out var work, publish)
            .Should().Be(PhysicsQueryExecutionStatus.Completed);
        result.Should().Be(ColliderReconfigurationStatus.Blocked);
        blocker.Should().BeSameAs(obstacle);
        failure.Should().BeNull();
        work.Discovery.Should().Be(2);
        context.ComputeReplayHash().Should().Be(before);

        foreach (var insufficient in new[]
        {
            new PhysicsQueryBudget(work.Discovery - 1, work.Candidates, work.Geometry),
            new PhysicsQueryBudget(work.Discovery, work.Candidates - 1, work.Geometry),
            new PhysicsQueryBudget(work.Discovery, work.Candidates, 0),
            new PhysicsQueryBudget(work.Discovery, work.Candidates, 1),
            new PhysicsQueryBudget(work.Discovery, work.Candidates, 2),
            new PhysicsQueryBudget(work.Discovery, work.Candidates, work.Geometry - 1)
        })
        {
            body.TryReconfigureCollider(posture, Vector2d.Zero, overlapping, insufficient, scratch,
                out result, out blocker, out failure, out var charged, publish)
                .Should().Be(PhysicsQueryExecutionStatus.WorkBudgetExceeded);
            charged.Discovery.Should().BeLessThanOrEqualTo(insufficient.DiscoveryLimit);
            charged.Candidates.Should().BeLessThanOrEqualTo(insufficient.CandidateLimit);
            charged.Geometry.Should().BeLessThanOrEqualTo(insufficient.GeometryLimit);
            result.Should().BeNull();
            blocker.Should().BeNull();
            failure.Should().BeNull();
            context.ComputeReplayHash().Should().Be(before);
        }

        var exact = new PhysicsQueryBudget(work.Discovery, work.Candidates, work.Geometry);
        long allocated = GC.GetAllocatedBytesForCurrentThread();
        var execution = body.TryReconfigureCollider(posture, Vector2d.Zero, overlapping, exact, scratch,
            out result, out blocker, out failure, out var repeated, publish);
        long rejectedAllocation = GC.GetAllocatedBytesForCurrentThread() - allocated;
        execution.Should().Be(PhysicsQueryExecutionStatus.Completed);
        result.Should().Be(ColliderReconfigurationStatus.Blocked);
        blocker.Should().BeSameAs(obstacle);
        repeated.Should().Be(work);
        rejectedAllocation.Should().Be(0);
        publications.Should().Be(0);

        body.TryReconfigureCollider(posture, Vector2d.Zero, touching, exact, scratch,
            out result, out blocker, out failure, out _, () =>
            {
                publications++;
                body.Position.Should().Be(touching);
                body.Agent.Transform.WorldPosition.Should().Be(new Vector3d(1, 7, 0));
                body.TryReconfigureCollider(posture, Vector2d.Zero, touching, new PhysicsQueryBudget(2, 0, 1), scratch,
                    out var nested, out _, out _, out _).Should().Be(PhysicsQueryExecutionStatus.Completed);
                nested.Should().Be(ColliderReconfigurationStatus.Unchanged);
            }).Should().Be(PhysicsQueryExecutionStatus.Completed);
        result.Should().Be(ColliderReconfigurationStatus.Applied);
        blocker.Should().BeNull();
        failure.Should().BeNull();
        publications.Should().Be(1);
    }
}
