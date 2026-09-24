using FixedMathSharp;
using FluentAssertions;
using Gravitas.Colliders;
using Gravitas.Constraints;
using Gravitas.Queries;
using Gravitas.Support;
using Gravitas.Tests.Support;
using System;
using Xunit;

namespace Gravitas.Tests.Physics2D;

public sealed class Physics2DSupportQueryTests
{
    [Theory]
    [InlineData(1)]
    [InlineData(256)]
    [InlineData(257)]
    [InlineData(4294967296L)]
    public void TouchingFloor_PreservesWitnessForEveryPositiveDistance(long rawDistance)
    {
        using var context = Physics2DTestWorld.CreateContext();
        SolidBody2D source = Source(context);
        var floor = new LSAABBoxCollider2D(new Vector2d(12, 1));
        Initialize(context, floor, new Vector2d(Fixed64.Zero, -Fixed64.Half));
        var request = Request(source, new Vector2d(Fixed64.Zero, Fixed64.Half), Fixed64.FromRaw(rawDistance));

        context.Query2D.QuerySupport(request, out var support).Should().Be(Physics2DSupportQueryStatus.Found);

        support.Hit.Collider.Should().BeSameAs(floor);
        support.Hit.Point.Should().Be(Vector2d.Zero);
        support.Hit.Normal.Should().Be(Vector2d.Forward);
        support.Hit.Distance.Should().Be(Fixed64.Zero);
        support.SampledFrame.Should().Be(context.FrameCount);
        support.IsCurrentLifetime.Should().BeTrue();
        context.Reset();
        support.IsCurrentLifetime.Should().BeFalse();
    }

    [Fact]
    public void CompoundWallBeforeFloor_DoesNotHideSupport()
    {
        using var context = Physics2DTestWorld.CreateContext();
        SolidBody2D source = Source(context);
        var compound = new LSCompoundCollider2D(
            CompoundColliderPart2D.AABBox(new Vector2d(1, 4), new Vector2d(-Fixed64.Half, Fixed64.Zero)),
            CompoundColliderPart2D.AABBox(new Vector2d(4, 1), new Vector2d(Fixed64.Zero, -Fixed64.Half)));
        Initialize(context, compound, Vector2d.Zero);
        var request = Request(source, new Vector2d(Fixed64.Half, Fixed64.Half), Fixed64.One);

        context.Query2D.QuerySupport(request, out var support).Should().Be(Physics2DSupportQueryStatus.Found);

        support.Hit.Collider.Should().BeSameAs(compound);
        support.Hit.Point.Should().Be(new Vector2d(Fixed64.Half, Fixed64.Zero));
        support.Hit.Normal.Should().Be(Vector2d.Forward);
    }

    [Fact]
    public void DownwardProbe_ReportsTravelAndTargetPoint()
    {
        using var context = Physics2DTestWorld.CreateContext();
        SolidBody2D source = Source(context);
        var floor = new LSAABBoxCollider2D(new Vector2d(12, 1));
        Initialize(context, floor, new Vector2d(Fixed64.Zero, -Fixed64.Half));
        var request = Request(source, new Vector2d(Fixed64.Zero, Fixed64.Two), (Fixed64)3);

        context.Query2D.QuerySupport(request, out var support).Should().Be(Physics2DSupportQueryStatus.Found);

        support.Hit.Distance.Should().Be(Fixed64.One + Fixed64.Half);
        support.Hit.Point.Should().Be(Vector2d.Zero);
    }

    private static Physics2DSupportQuery Request(SolidBody2D source, Vector2d center, Fixed64 distance) =>
        new(source.Collider, center, Fixed64.Half, Vector2d.Forward, distance, Fixed64.Half, PhysicsLayerMask.All);

    [Theory]
    [InlineData(1, ColliderType2D.AABox)]
    [InlineData(256, ColliderType2D.AABox)]
    [InlineData(257, ColliderType2D.AABox)]
    [InlineData(1, ColliderType2D.Circle)]
    [InlineData(256, ColliderType2D.Circle)]
    [InlineData(257, ColliderType2D.Circle)]
    [InlineData(1, ColliderType2D.Capsule)]
    [InlineData(256, ColliderType2D.Capsule)]
    [InlineData(257, ColliderType2D.Capsule)]
    public void SeparatedTinyProbe_ReachesSurfaceAtEndpoint(long rawDistance, ColliderType2D shape)
    {
        using var context = Physics2DTestWorld.CreateContext();
        SolidBody2D source = Source(context);
        LSCollider2D target = shape switch
        {
            ColliderType2D.Circle => new LSCircleCollider2D(Fixed64.Half),
            ColliderType2D.Capsule => new LSCapsuleCollider2D(Fixed64.Half, Fixed64.Two),
            _ => new LSAABBoxCollider2D(new Vector2d(12, 1))
        };
        Initialize(context, target, Vector2d.Zero);
        Fixed64 top = shape == ColliderType2D.Capsule ? Fixed64.One : Fixed64.Half;
        Fixed64 distance = Fixed64.FromRaw(rawDistance);
        var request = Request(source, new Vector2d(Fixed64.Zero, top + Fixed64.Half + distance), distance);

        context.Query2D.QuerySupport(request, out var support).Should().Be(Physics2DSupportQueryStatus.Found);
        support.Hit.Distance.Should().Be(distance);
        support.Hit.Point.Should().Be(new Vector2d(Fixed64.Zero, top));
        support.Hit.Normal.Should().Be(Vector2d.Forward);
    }

    [Theory]
    [InlineData(0)] // Trigger.
    [InlineData(1)] // Dynamic body.
    [InlineData(2)] // Source's ignored layers.
    [InlineData(3)] // Target's ignored layers.
    [InlineData(4)] // Query's include mask.
    [InlineData(5)] // Layer matrix.
    [InlineData(6)] // Hierarchy sibling.
    public void IneligibleNearSurface_DoesNotHideFarSupport(int exclusion)
    {
        using var context = Physics2DTestWorld.CreateContext();
        if (exclusion == 5)
            context.ApplySettings(new PhysicsSettings(4, new[,]
            {
                { true, true, false }, { true, true, true }, { false, true, true }
            }) { RuntimeMode = PhysicsRuntimeMode.TwoD });
        SolidBody2D source = Source(context);
        var near = new LSAABBoxCollider2D(new Vector2d(12, 1));
        near.Layer = new PhysicsLayer(2);
        if (exclusion == 0)
            near.IsTrigger = true;
        if (exclusion == 1)
            new SolidBody2D(new TestMatterAgent(context: context), near).Initialize(Vector2d.Zero);
        else
            Initialize(context, near, Vector2d.Zero);
        if (exclusion == 2)
            source.Collider.IgnoredCollisionLayers = PhysicsLayerMask.FromLayer(near.Layer);
        if (exclusion == 3)
            near.IgnoredCollisionLayers = PhysicsLayerMask.FromLayer(source.Collider.Layer);
        if (exclusion == 6)
            near.SetParent(source.Collider);
        var floor = new LSAABBoxCollider2D(new Vector2d(12, 1));
        Initialize(context, floor, new Vector2d(Fixed64.Zero, -Fixed64.Two));
        var request = new Physics2DSupportQuery(source.Collider, Vector2d.Forward * (Fixed64)2,
            Fixed64.Half, Vector2d.Forward, (Fixed64)4, Fixed64.Half,
            exclusion == 4 ? PhysicsLayerMask.FromLayer(floor.Layer) : PhysicsLayerMask.All);

        context.Query2D.QuerySupport(request, out var support).Should().Be(Physics2DSupportQueryStatus.Found);
        support.Hit.Collider.Should().BeSameAs(floor);
        support.Hit.Distance.Should().Be((Fixed64)3);
    }

    [Theory]
    [InlineData(BodyMotionType.Static)]
    [InlineData(BodyMotionType.Kinematic)]
    public void BodyBackedSupport_PreservesRegistrationIdentity(BodyMotionType motionType)
    {
        using var context = Physics2DTestWorld.CreateContext();
        SolidBody2D source = Source(context);
        var floor = new SolidBody2D(new TestMatterAgent(context: context), new LSAABBoxCollider2D(new Vector2d(12, 1)));
        floor.Initialize(Vector2d.Zero, motionType: motionType);
        var request = Request(source, Vector2d.Forward, Fixed64.One);
        context.Query2D.QuerySupport(request, out var support).Should().Be(Physics2DSupportQueryStatus.Found);
        support.Hit.Body.Should().BeSameAs(floor);
        support.IsCurrentLifetime.Should().BeTrue();
        floor.Deactivate();
        support.IsCurrentLifetime.Should().BeFalse();
        floor.Initialize(Vector2d.Zero, motionType: motionType);
        support.IsCurrentLifetime.Should().BeFalse();
        context.Query2D.QuerySupport(request, out var fresh).Should().Be(Physics2DSupportQueryStatus.Found);
        fresh.IsCurrentLifetime.Should().BeTrue();
    }

    [Fact]
    public void OpenStep_IsNotACompleteNoSupportResult()
    {
        using var context = Physics2DTestWorld.CreateContext();
        SolidBody2D source = Source(context);
        var request = Request(source, Vector2d.Forward, Fixed64.One);
        context.Simulate();
        context.Query2D.QuerySupport(request, out var pending).Should().Be(Physics2DSupportQueryStatus.WorldNotReady);
        pending.IsCurrentLifetime.Should().BeFalse();
        context.LateSimulate();
        context.Query2D.QuerySupport(request, out var absent).Should().Be(Physics2DSupportQueryStatus.NoSupport);
        absent.IsCurrentLifetime.Should().BeFalse();
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void UnrepresentableProbe_ClearsResult(bool endpoint)
    {
        using var context = Physics2DTestWorld.CreateContext();
        SolidBody2D source = Source(context);
        var request = Request(source,
            endpoint ? new Vector2d(Fixed64.Zero, Fixed64.MinValue) : new Vector2d(Fixed64.MaxValue, Fixed64.Zero), Fixed64.One);
        context.Query2D.QuerySupport(request, out var support).Should().Be(Physics2DSupportQueryStatus.Unrepresentable);
        support.IsCurrentLifetime.Should().BeFalse();
        support.Hit.Collider.Should().BeNull();
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void NegativeScalarFace_RejectsEndpointOrRadiusOverflow(bool endpoint)
    {
        using var context = Physics2DTestWorld.CreateContext();
        SolidBody2D source = Source(context);
        var request = new Physics2DSupportQuery(source.Collider,
            new Vector2d(Fixed64.MinValue, Fixed64.Zero), Fixed64.Half,
            endpoint ? Vector2d.Right : Vector2d.Forward, Fixed64.One, Fixed64.Half, PhysicsLayerMask.All);
        context.Query2D.QuerySupport(request, out var support).Should().Be(Physics2DSupportQueryStatus.Unrepresentable);
        support.Hit.Collider.Should().BeNull();
    }

    [Fact]
    public void CompoundWithoutEligibleNormal_ClearsHit()
    {
        using var context = Physics2DTestWorld.CreateContext();
        SolidBody2D source = Source(context);
        var wall = new LSCompoundCollider2D(
            CompoundColliderPart2D.AABBox(new Vector2d(1, 4), new Vector2d(-Fixed64.Half, Fixed64.Zero)));
        Initialize(context, wall, Vector2d.Zero);
        var request = Request(source, new Vector2d(Fixed64.Half, Fixed64.Half), Fixed64.One);
        context.Query2D.QuerySupport(request, out var support).Should().Be(Physics2DSupportQueryStatus.NoSupport);
        support.Hit.Collider.Should().BeNull();
    }

    [Fact]
    public void Request_RejectsInvalidInputAndNormalizesUp()
    {
        using var context = Physics2DTestWorld.CreateContext();
        SolidBody2D source = Source(context);
        Assert.Throws<ArgumentNullException>(() => new Physics2DSupportQuery(null!, Vector2d.Zero, Fixed64.One,
            Vector2d.Forward, Fixed64.One, Fixed64.Half, PhysicsLayerMask.All));
        foreach (Fixed64 invalid in new[] { Fixed64.Zero, -Fixed64.One })
        {
            Assert.Throws<ArgumentException>(() => new Physics2DSupportQuery(source.Collider, Vector2d.Zero, invalid,
                Vector2d.Forward, Fixed64.One, Fixed64.Half, PhysicsLayerMask.All));
            Assert.Throws<ArgumentException>(() => new Physics2DSupportQuery(source.Collider, Vector2d.Zero, Fixed64.One,
                Vector2d.Forward, invalid, Fixed64.Half, PhysicsLayerMask.All));
        }
        foreach (Fixed64 invalid in new[] { Fixed64.Zero, Fixed64.Two })
            Assert.Throws<ArgumentException>(() => new Physics2DSupportQuery(source.Collider, Vector2d.Zero, Fixed64.One,
                Vector2d.Forward, Fixed64.One, invalid, PhysicsLayerMask.All));
        Assert.Throws<ArgumentException>(() => new Physics2DSupportQuery(source.Collider, Vector2d.Zero, Fixed64.One,
            Vector2d.Zero, Fixed64.One, Fixed64.Half, PhysicsLayerMask.All));
        var request = new Physics2DSupportQuery(source.Collider, Vector2d.Zero, Fixed64.One,
            Vector2d.Forward * (Fixed64)7, Fixed64.One, Fixed64.Half, PhysicsLayerMask.All);
        request.Up.Should().Be(Vector2d.Forward);
        Assert.Throws<ArgumentNullException>(() => context.Query2D.QuerySupport(default, out _));
        source.Deactivate();
        Assert.Throws<ArgumentException>(() => context.Query2D.QuerySupport(request, out _));
    }

    [Fact]
    public void LinkedSupport_UsesPhysicalJointExclusion()
    {
        using var context = Physics2DTestWorld.CreateContext();
        SolidBody2D source = Source(context);
        source.SetMotionType(BodyMotionType.Dynamic);
        var floor = new SolidBody2D(new TestMatterAgent(context: context), new LSAABBoxCollider2D(new Vector2d(12, 1)));
        floor.Initialize(Vector2d.Zero, motionType: BodyMotionType.Kinematic);
        var request = Request(source, Vector2d.Forward, Fixed64.One);
        context.Query2D.QuerySupport(request, out _).Should().Be(Physics2DSupportQueryStatus.Found);
        Joint2D joint = context.Constraints2D.RegisterJoint(new JointDefinition2D(source, floor,
            JointFrame2D.Identity, JointFrame2D.Identity, JointType2D.Pin, JointLimit2D.Unrestricted,
            JointMotor2D.Disabled, JointCollisionPolicy.SuppressLinked));
        context.Query2D.QuerySupport(request, out _).Should().Be(Physics2DSupportQueryStatus.NoSupport);
        context.Constraints2D.RemoveJoint(joint.Id).Should().BeTrue();
        context.Query2D.QuerySupport(request, out _).Should().Be(Physics2DSupportQueryStatus.Found);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void SupportRanking_UsesDistanceThenOwnerId(bool closerSecond)
    {
        using var context = Physics2DTestWorld.CreateContext();
        SolidBody2D source = Source(context);
        var first = new LSAABBoxCollider2D(new Vector2d(12, 1));
        var second = new LSAABBoxCollider2D(new Vector2d(12, 1));
        Initialize(context, first, Vector2d.Zero);
        Initialize(context, second, closerSecond ? Vector2d.Forward : Vector2d.Zero);
        var request = Request(source, Vector2d.Forward * (Fixed64)3, (Fixed64)4);
        context.Query2D.QuerySupport(request, out var support).Should().Be(Physics2DSupportQueryStatus.Found);
        support.Hit.Collider.Should().BeSameAs(closerSecond ? second : first);
        support.Hit.Distance.Should().Be(closerSecond ? Fixed64.One : Fixed64.Two);
    }

    [Fact]
    public void ArbitraryUp_SelectsSurfaceAndRejectsTooSteepNormal()
    {
        using var context = Physics2DTestWorld.CreateContext();
        SolidBody2D source = Source(context);
        var wall = new LSAABBoxCollider2D(new Vector2d(1, 12));
        Initialize(context, wall, Vector2d.Zero);
        // Probe down-left at 45 degrees toward the wall's right side.
        var up = new Vector2d(Fixed64.One, Fixed64.One);
        var admitted = new Physics2DSupportQuery(source.Collider, Vector2d.Right * Fixed64.Two,
            Fixed64.Half, up, (Fixed64)3, Fixed64.Half, PhysicsLayerMask.All);
        context.Query2D.QuerySupport(admitted, out var support).Should().Be(Physics2DSupportQueryStatus.Found);
        support.Hit.Normal.Should().Be(Vector2d.Right);
        var rejected = new Physics2DSupportQuery(source.Collider, admitted.Center, admitted.Radius,
            up, admitted.Distance, Fixed64.One, PhysicsLayerMask.All);
        context.Query2D.QuerySupport(rejected, out var absent).Should().Be(Physics2DSupportQueryStatus.NoSupport);
        absent.IsCurrentLifetime.Should().BeFalse();
    }

    [Fact]
    public void WarmQueries_ReuseStorageAndKeepObservableHits()
    {
        using var context = Physics2DTestWorld.CreateContext();
        SolidBody2D source = Source(context);
        var floor = new LSAABBoxCollider2D(new Vector2d(12, 1));
        Initialize(context, floor, Vector2d.Zero);
        var request = Request(source, Vector2d.Forward, Fixed64.One);
        int hits = 0;
        long allocated = AllocationTestHelper.MeasureSteadyState(() =>
        {
            if (context.Query2D.QuerySupport(request, out var hit) == Physics2DSupportQueryStatus.Found
                && ReferenceEquals(hit.Hit.Collider, floor))
                hits++;
        });
        hits.Should().Be(128 + 16 + 64);
        allocated.Should().Be(0);
    }

    private static SolidBody2D Source(GravitasWorldContext context)
    {
        var agent = new TestMatterAgent(context: context);
        var body = new SolidBody2D(agent, new LSCircleCollider2D(Fixed64.Half));
        body.Initialize(new Vector2d(10, 10), motionType: BodyMotionType.Kinematic);
        return body;
    }

    private static void Initialize(GravitasWorldContext context, LSCollider2D collider, Vector2d position)
    {
        var agent = new TestMatterAgent(context: context);
        agent.Transform.TrySetWorldPosition(new Vector3d(position.X, Fixed64.Zero, position.Y)).Should().BeTrue();
        collider.InitializeWithNoBody(agent);
    }
}
