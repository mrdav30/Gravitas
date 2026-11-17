using System;
using FixedMathSharp;
using FluentAssertions;
using Gravitas.Colliders;
using Gravitas.Constraints;
using Gravitas.Queries;
using Gravitas.Tests.Support;
using Xunit;

namespace Gravitas.Tests.Core;

public sealed partial class SolidBodyColliderReconfigurationClearanceTests
{
    [Theory]
    [InlineData(1)]
    [InlineData(2)]
    public void BoundedPostureChargesTheCachedHostPoseAncestry(int geometryLimit)
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        var parent = new FixedTransform(Vector3d.Zero, FixedQuaternion.Identity, Vector3d.One);
        var transform = new FixedTransform(Vector3d.Zero, FixedQuaternion.Identity, Vector3d.One, parent);
        var agent = new ReplaceableTransformAgent(context, transform);
        var body = new SolidBody(agent, SourceShape(1, Fixed64.Half).CreateCollider())
            { Mass = Fixed64.One };
        body.Initialize(Vector3d.Zero, FixedQuaternion.Identity, PhysicsMobilityType.Kinematic);
        agent.Transform = new FixedTransform(Vector3d.Zero, FixedQuaternion.Identity, Vector3d.One);

        var shortBudget = new PhysicsQueryBudget(2, 2, (ulong)geometryLimit);
        body.TryReconfigureCollider(SourceShape(1, Fixed64.One), Vector3d.Zero, Vector3d.Up,
            shortBudget, new PhysicsQueryScratch(0, 0), out var result, out var blocker, out var failure, out var work)
            .Should().Be(PhysicsQueryExecutionStatus.WorkBudgetExceeded);
        result.Should().BeNull();
        blocker.Should().BeNull();
        failure.Should().BeNull();
        work.Geometry.Should().Be((ulong)geometryLimit);
        transform.WorldPosition.Should().Be(Vector3d.Zero);
    }

    [Theory]
    [InlineData(-1_000_000_000, 2, 500_000_000, 1_500_000_000, 0)]
    [InlineData(1_000_000_000, 1, -1_000_000_000, -1_500_000_000, 0)]
    [InlineData(1_000_000_000, 1, 0, 1_500_000_000, -1_000_000_000)]
    public void UnrepresentableHostPoseLeavesTheCommittedPostureUnchanged(
        int parentX, int scale, int localX, int positionX, int grandparentX)
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        // Opposing ancestors can compose on their own while the requested child overflows in between.
        FixedTransform? grandparent = grandparentX == 0 ? null :
            new FixedTransform(new Vector3d(grandparentX, 0, 0), FixedQuaternion.Identity, Vector3d.One);
        var parent = new FixedTransform(new Vector3d(parentX, 0, 0),
            FixedQuaternion.Identity, Vector3d.One * (Fixed64)scale, grandparent);
        var localPosition = new Vector3d(localX, 0, 0);
        var transform = new FixedTransform(localPosition, FixedQuaternion.Identity, Vector3d.One, parent);
        var body = new SolidBody(new TestMatterAgent(context, transform), SourceShape(1, Fixed64.Half).CreateCollider())
            { Mass = Fixed64.One };
        body.Initialize(Vector3d.Zero, FixedQuaternion.Identity, PhysicsMobilityType.Kinematic);
        var before = context.ComputeReplayHash();
        var position = new Vector3d(positionX, 0, 0);
        var replacement = SourceShape(1, Fixed64.One);
        var budget = new PhysicsQueryBudget(2, 32, 32);
        int publications = 0;

        body.TryReconfigureCollider(replacement, Vector3d.Zero, position, budget,
            new PhysicsQueryScratch(0, 0), out var result, out var blocker, out var failure, out _,
            () => publications++).Should().Be(PhysicsQueryExecutionStatus.Unrepresentable);

        result.Should().BeNull();
        blocker.Should().BeNull();
        failure.Should().BeNull();
        publications.Should().Be(0);
        body.Position3d.Should().Be(Vector3d.Zero);
        transform.LocalPosition.Should().Be(localPosition);
        context.ComputeReplayHash().Should().Be(before);
    }

    [Fact]
    public void UnrepresentableDependentHostPoseLeavesTheCommittedPostureUnchanged()
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        var transform = new FixedTransform(Vector3d.Zero, FixedQuaternion.Identity, Vector3d.One);
        var agent = new ReplaceableTransformAgent(context, transform);
        var body = new SolidBody(agent, SourceShape(1, Fixed64.Half).CreateCollider()) { Mass = Fixed64.One };
        body.Initialize(Vector3d.Zero, FixedQuaternion.Identity, PhysicsMobilityType.Kinematic);
        var childPosition = new Vector3d(1_000_000_000, 0, 0);
        agent.Transform = new FixedTransform(childPosition, FixedQuaternion.Identity, Vector3d.One, transform);
        var before = context.ComputeReplayHash();
        var position = new Vector3d(1_500_000_000, 0, 0);
        var budget = new PhysicsQueryBudget(2, 32, 32);
        int publications = 0;

        body.TryReconfigureCollider(SourceShape(1, Fixed64.One), Vector3d.Zero, position, budget,
            new PhysicsQueryScratch(0, 0), out var result, out var blocker, out var failure, out _,
            () => publications++).Should().Be(PhysicsQueryExecutionStatus.Unrepresentable);

        result.Should().BeNull();
        blocker.Should().BeNull();
        failure.Should().BeNull();
        publications.Should().Be(0);
        body.Position3d.Should().Be(Vector3d.Zero);
        transform.LocalPosition.Should().Be(Vector3d.Zero);
        agent.Transform.WorldPosition.Should().Be(childPosition);
        context.ComputeReplayHash().Should().Be(before);
    }

    [Theory]
    [InlineData(PhysicsRuntimeMode.Both, PhysicsQueryExecutionStatus.UnsupportedRuntimeMode)]
    [InlineData(PhysicsRuntimeMode.ThreeD, PhysicsQueryExecutionStatus.UnsupportedGeometry)]
    public void BoundedCylinderReplacementRejectsBeforeClearance(
        PhysicsRuntimeMode mode, PhysicsQueryExecutionStatus expected)
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        var body = new SolidBody(new TestMatterAgent(context), SourceShape(2, Fixed64.Half).CreateCollider())
            { Mass = Fixed64.One };
        body.Initialize(Vector3d.Zero, FixedQuaternion.Identity, PhysicsMobilityType.Kinematic);
        context.Settings.RuntimeMode = mode;
        var before = context.ComputeReplayHash();

        body.TryReconfigureCollider(SourceShape(2, Fixed64.One), Vector3d.Zero, Vector3d.Up,
            new PhysicsQueryBudget(2, 8, 8), new PhysicsQueryScratch(0, 0),
            out var result, out var blocker, out var failure, out var work).Should().Be(expected);
        result.Should().BeNull();
        blocker.Should().BeNull();
        failure.Should().BeNull();
        work.Should().Be(default(PhysicsQueryWork));
        context.ComputeReplayHash().Should().Be(before);
    }

    [Fact]
    public void BoundedSpherePostureWaitsForPublicationAndRejectsUnrepresentableAttachment()
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        var body = new SolidBody(new TestMatterAgent(context), SourceShape(0, Fixed64.Half).CreateCollider())
            { Mass = Fixed64.One };
        body.Initialize(Vector3d.Zero, FixedQuaternion.Identity, PhysicsMobilityType.Kinematic);
        var scratch = new PhysicsQueryScratch(0, 0);
        var posture = SourceShape(0, Fixed64.One);
        var budget = new PhysicsQueryBudget(2, 8, 8);

        context.Simulate();
        Reject(Vector3d.Zero, PhysicsQueryExecutionStatus.WorldNotReady, default);
        context.LateSimulate();
        Reject(new Vector3d(Fixed64.MaxValue, Fixed64.Zero, Fixed64.Zero),
            PhysicsQueryExecutionStatus.Unrepresentable, new PhysicsQueryWork(1, 0, 1));

        body.TryReconfigureCollider(posture, Vector3d.Zero, Vector3d.Right, budget, scratch,
            out var result, out _, out _, out _).Should().Be(PhysicsQueryExecutionStatus.Completed);
        result.Should().Be(ColliderReconfigurationStatus.Applied);

        void Reject(Vector3d offset, PhysicsQueryExecutionStatus expected, PhysicsQueryWork expectedWork)
        {
            var before = context.ComputeReplayHash();
            body.TryReconfigureCollider(posture, offset, Vector3d.Right, budget, scratch,
                out var outcome, out var blocker, out var failure, out var work).Should().Be(expected);
            outcome.Should().BeNull();
            blocker.Should().BeNull();
            failure.Should().BeNull();
            work.Should().Be(expectedWork);
            context.ComputeReplayHash().Should().Be(before);
        }
    }

    [Fact]
    public void BoundedPostureChargesTheLinkedPairProbeBeforeSkippingItsBlocker()
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        var definition = SourceShape(0, Fixed64.Half);
        var body = new SolidBody(new TestMatterAgent(context), definition.CreateCollider()) { Mass = Fixed64.One };
        body.Initialize(Vector3d.Zero, FixedQuaternion.Identity, PhysicsMobilityType.Kinematic);
        var linked = new SolidBody(new TestMatterAgent(context), definition.CreateCollider()) { Mass = Fixed64.One };
        linked.Initialize(Vector3d.Right, FixedQuaternion.Identity, PhysicsMobilityType.Kinematic);
        var frame = new FixedTransform(Vector3d.Zero, FixedQuaternion.Identity, Vector3d.One);
        context.Constraints3D.RegisterJoint(new JointDefinition3D(body, linked, frame, frame,
            JointType3D.BallSocket, JointLimit3D.Unrestricted, JointMotor3D.Disabled, JointCollisionPolicy.SuppressLinked));
        var scratch = new PhysicsQueryScratch(0, 0);
        var posture = SourceShape(0, Fixed64.One);
        var before = context.ComputeReplayHash();

        body.TryReconfigureCollider(posture, Vector3d.Zero, Vector3d.Zero, new PhysicsQueryBudget(2, 2, 1),
            scratch, out var result, out var blocker, out var failure, out var work)
            .Should().Be(PhysicsQueryExecutionStatus.WorkBudgetExceeded);
        result.Should().BeNull();
        blocker.Should().BeNull();
        failure.Should().BeNull();
        work.Should().Be(new PhysicsQueryWork(1, 2, 1));
        context.ComputeReplayHash().Should().Be(before);

        body.TryReconfigureCollider(posture, Vector3d.Zero, Vector3d.Zero, new PhysicsQueryBudget(2, 3, 1),
            scratch, out result, out blocker, out failure, out work).Should().Be(PhysicsQueryExecutionStatus.Completed);
        result.Should().Be(ColliderReconfigurationStatus.Applied);
        blocker.Should().BeNull();
        failure.Should().BeNull();
        work.Should().Be(new PhysicsQueryWork(2, 3, 1));
    }

    [Theory]
    [InlineData(3, true)]  // Finite cylinder compound leaf
    [InlineData(5, true)]  // Closed convex mesh compound leaf
    [InlineData(7, false)] // Concave surface mesh
    public void BoundedPosture_ShouldHonorExactBudgetsAndPublishOnlyAfterClearance(int target, bool compound)
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        var transform = new FixedTransform(Vector3d.Zero, FixedQuaternion.Identity, Vector3d.One);
        var body = new SolidBody(new TestMatterAgent(context, transform), SourceShape(1, Fixed64.Half).CreateCollider())
        { Mass = Fixed64.One };
        body.Initialize(Vector3d.Zero, FixedQuaternion.Identity, PhysicsMobilityType.Kinematic);
        LSCollider obstacle = CreateTarget(target, compound);
        obstacle.InitializeWithNoBody(new TestMatterAgent(context,
            new FixedTransform(Vector3d.Zero, FixedQuaternion.Identity, Vector3d.One)));
        (obstacle.PartitionCoordinates?.Count ?? 0).Should().Be(0);

        var scratch = new PhysicsQueryScratch(0, 0);
        var budget = new PhysicsQueryBudget(2, 32, 1024);
        ColliderShapeDefinition posture = SourceShape(1, Fixed64.One);
        Vector3d touching = Vector3d.Up * (target == 7 ? Fixed64.Two : (Fixed64)3);
        Vector3d overlapping = touching - Vector3d.Up * Fixed64.FromRaw(1);
        var before = context.ComputeReplayHash();
        int publications = 0;
        Action publish = () => publications++;

        body.TryReconfigureCollider(posture, Vector3d.Zero, overlapping, budget, scratch,
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
            body.TryReconfigureCollider(posture, Vector3d.Zero, overlapping, insufficient, scratch,
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
        var execution = body.TryReconfigureCollider(posture, Vector3d.Zero, overlapping, exact, scratch,
            out result, out blocker, out failure, out var repeated, publish);
        long rejectedAllocation = GC.GetAllocatedBytesForCurrentThread() - allocated;
        execution.Should().Be(PhysicsQueryExecutionStatus.Completed);
        result.Should().Be(ColliderReconfigurationStatus.Blocked);
        blocker.Should().BeSameAs(obstacle);
        repeated.Should().Be(work);
        rejectedAllocation.Should().Be(0);
        publications.Should().Be(0);

        body.TryReconfigureCollider(posture, Vector3d.Zero, touching, exact, scratch,
            out result, out blocker, out failure, out _, () =>
            {
                publications++;
                body.Position3d.Should().Be(touching);
                transform.WorldPosition.Should().Be(touching);
                body.TryReconfigureCollider(posture, Vector3d.Zero, touching, new PhysicsQueryBudget(2, 0, 1), scratch,
                    out var nested, out _, out _, out _).Should().Be(PhysicsQueryExecutionStatus.Completed);
                nested.Should().Be(ColliderReconfigurationStatus.Unchanged);
            }).Should().Be(PhysicsQueryExecutionStatus.Completed);
        result.Should().Be(ColliderReconfigurationStatus.Applied);
        blocker.Should().BeNull();
        failure.Should().BeNull();
        publications.Should().Be(1);
    }

    private sealed class ReplaceableTransformAgent(GravitasWorldContext context, FixedTransform transform) : IMatterAgent
    {
        public GravitasWorldContext Context { get; } = context;
        public FixedTransform Transform { get; set; } = transform;
        public bool IsParent => true;
        public bool IsInteracting => false;
    }
}
