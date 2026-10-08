using FixedMathSharp;
using FluentAssertions;
using Gravitas.Colliders;
using Gravitas.CollisionHandling;
using Gravitas.Materials;
using Gravitas.Tests.Support;
using System.Reflection;
using Xunit;

namespace Gravitas.Tests.CollisionHandlingTests;

public sealed class DiscreteIslandSolverTests
{
    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void MultipleGroupsInOnePair_ShouldUseConfiguredIterations(bool includeUnrelatedPair)
    {
        Fixed64 oneIterationResidual = SolveObliqueGroups(1, includeUnrelatedPair);
        Fixed64 fourIterationResidual = SolveObliqueGroups(4, includeUnrelatedPair);

        oneIterationResidual.Should().BeGreaterThan(Fixed64.FromFraction(1, 3));
        fourIterationResidual.Should().BeLessThan(Fixed64.FromFraction(1, 10),
            "the later oblique row reintroduces closing speed at the first surface, requiring additional iterations");
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void SingleGroupFace_ShouldUseConfiguredIterations(bool includeUnrelatedPair)
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        scenario.Context.Settings.DiscreteSolverIterations = 32;
        var floor = scenario.CreateCuboid(Vector3d.Zero, immovable: true);
        var box = scenario.CreateCuboid(new Vector3d(Fixed64.Zero, Fixed64.FromFraction(3, 4), Fixed64.Zero));
        CollisionPair pair = scenario.CreatePair(floor.Collider, box.Collider);
        CollisionDetection.DoCollisionCheck(pair).Should().BeTrue();
        pair.Manifold.GroupCount.Should().Be(1);
        pair.Manifold.Count.Should().Be(ContactManifold.MaxContactsPerGroup);
        box.Body.AddLinearImpulse(Vector3d.Down * (Fixed64)2 * box.Body.Mass);
        scenario.Context.Physics.QueueDiscreteResponsePair(pair);
        if (includeUnrelatedPair)
            QueueUnrelatedPair(scenario);

        SolveQueuedResponses(scenario);

        Fixed64 tolerance = Fixed64.FromFraction(1, 1_000_000);
        box.Body.LinearVelocity.X.Abs().Should().BeLessThan(tolerance);
        (box.Body.LinearVelocity.Y - Fixed64.One).Abs().Should().BeLessThan(tolerance);
        box.Body.LinearVelocity.Z.Abs().Should().BeLessThan(tolerance);
        box.Body.AngularVelocity.X.Abs().Should().BeLessThan(tolerance);
        box.Body.AngularVelocity.Y.Abs().Should().BeLessThan(tolerance);
        box.Body.AngularVelocity.Z.Abs().Should().BeLessThan(tolerance);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void SharedBodyRestitution_ShouldUseIncomingMotionBeforeAnyPairSolves(bool cached)
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        scenario.Context.Settings.DiscreteSolverIterations = 6;
        scenario.Context.Settings.RestitutionVelocityThreshold = Fixed64.Zero;
        var firstWall = scenario.CreateCuboid(Vector3d.Zero, immovable: true);
        var secondWall = scenario.CreateCuboid(new Vector3d(0, 0, -1), immovable: true);
        var actor = scenario.CreateCuboid(new Vector3d(1, 0, 1), preventAngularForces: true);
        PhysicsMaterial material = PhysicsMaterialTestHelper.WithFrictionAndRestitution(Fixed64.Zero, Fixed64.One);
        firstWall.Collider.Material = secondWall.Collider.Material = actor.Collider.Material = material;
        CollisionPair firstPair = scenario.CreatePair(firstWall.Collider, actor.Collider);
        CollisionPair secondPair = scenario.CreatePair(secondWall.Collider, actor.Collider);
        firstPair.Manifold.SetContact(Vector3d.Zero, Vector3d.Zero, Fixed64.Zero, Vector3d.Right);
        Vector3d secondNormal = new(Fixed64.FromFraction(3, 5), Fixed64.Zero, Fixed64.FromFraction(4, 5));
        secondPair.Manifold.SetContact(Vector3d.Zero, Vector3d.Zero, Fixed64.Zero, secondNormal);
        if (cached)
            secondPair.StoreWarmStartImpulse(secondPair.Manifold[0].ContactId, secondNormal, Fixed64.Half, Fixed64.Zero);
        actor.Body.AddLinearImpulse(new Vector3d(-1, 0, -1) * actor.Body.Mass);
        scenario.Context.Physics.QueueDiscreteResponsePair(firstPair);
        scenario.Context.Physics.QueueDiscreteResponsePair(secondPair);

        SolveQueuedResponses(scenario);

        // These unit normals couple X and Z. Their incoming restitution targets
        // are 1 and 7/5; capturing the second after the first solve instead gives
        // it only 1/5 and produces a different rebound with negative Z velocity.
        Fixed64 tolerance = Fixed64.FromFraction(1, 100);
        (actor.Body.LinearVelocity.X - Fixed64.One).Abs().Should().BeLessThan(tolerance);
        (actor.Body.LinearVelocity.Z - Fixed64.One).Abs().Should().BeLessThan(tolerance);
        actor.Body.LinearVelocity.Y.Should().Be(Fixed64.Zero);
        actor.Body.AngularVelocity.Should().Be(Vector3d.Zero);
        actor.Body.Position3d.Should().Be(new Vector3d(1, 0, 1));
        firstWall.Body.LinearVelocity.Should().Be(Vector3d.Zero);
        secondWall.Body.LinearVelocity.Should().Be(Vector3d.Zero);
        firstWall.Body.Position3d.Should().Be(Vector3d.Zero);
        secondWall.Body.Position3d.Should().Be(new Vector3d(0, 0, -1));
    }

    [Fact]
    public void Simulate_WithRestingBodyUnderGravity_ShouldSettleAfterPostSolve()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSCuboidCollider> floor = scenario.CreateCuboid(PhysicsScenarioBuilder.Vector(0, 0, 0), immovable: true);
        ScenarioBody<LSCuboidCollider> body = scenario.CreateCuboid(
            PhysicsScenarioBuilder.Vector(0, 1, 0),
            preventAngularForces: true);
        floor.Collider.Material = PhysicsMaterialTestHelper.WithRestitution(Fixed64.Zero);
        body.Collider.Material = PhysicsMaterialTestHelper.WithRestitution(Fixed64.Zero);
        body.Body.UseManualGrounding();
        body.Body.SleepFrameThreshold = 4;

        for (int i = 0; i < 16; i++)
        {
            scenario.Context.Simulate();
            scenario.Context.LateSimulate();
        }

        CollisionPair pair = GetPair(floor.Collider, body.Collider);
        body.Body.IsSleeping.Should().BeTrue(
            $"body should settle; position={body.Body.Position3d}, velocity={body.Body.LinearVelocity}, speed={body.Body.LinearSpeed}, contacts={pair.Manifold.Count}, depth={pair.Manifold.PrimaryContact.Depth}");
        body.Body.Position3d.Y.Should().BeGreaterThan(Fixed64.FromFraction(63, 64));
    }

    [Fact]
    public void Simulate_WithRestingStackUnderGravity_ShouldSettleAsIsland()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSCuboidCollider> floor = scenario.CreateCuboid(PhysicsScenarioBuilder.Vector(0, 0, 0), immovable: true);
        ScenarioBody<LSCuboidCollider> lower = scenario.CreateCuboid(
            PhysicsScenarioBuilder.Vector(0, 1, 0),
            preventAngularForces: true);
        ScenarioBody<LSCuboidCollider> upper = scenario.CreateCuboid(
            PhysicsScenarioBuilder.Vector(0, 2, 0),
            preventAngularForces: true);
        floor.Collider.Material = PhysicsMaterialTestHelper.WithRestitution(Fixed64.Zero);
        lower.Collider.Material = PhysicsMaterialTestHelper.WithRestitution(Fixed64.Zero);
        upper.Collider.Material = PhysicsMaterialTestHelper.WithRestitution(Fixed64.Zero);
        lower.Body.UseManualGrounding();
        upper.Body.UseManualGrounding();
        lower.Body.SleepFrameThreshold = 4;
        upper.Body.SleepFrameThreshold = 4;
        lower.Body.SleepLinearSpeedThreshold = Fixed64.FromFraction(1, 100);
        upper.Body.SleepLinearSpeedThreshold = Fixed64.FromFraction(1, 100);

        for (int i = 0; i < 64; i++)
        {
            scenario.Context.Simulate();
            scenario.Context.LateSimulate();
        }

        lower.Body.IsSleeping.Should().BeTrue(
            $"lower stack body should settle; position={lower.Body.Position3d}, velocity={lower.Body.LinearVelocity}, speed={lower.Body.LinearSpeed}");
        upper.Body.IsSleeping.Should().BeTrue(
            $"upper stack body should settle; position={upper.Body.Position3d}, velocity={upper.Body.LinearVelocity}, speed={upper.Body.LinearSpeed}");
        lower.Body.Position3d.Y.Should().BeGreaterThan(Fixed64.FromFraction(31, 32));
        upper.Body.Position3d.Y.Should().BeGreaterThan(Fixed64.FromFraction(125, 64));
    }

    [Fact]
    public void Simulate_WithConnectedSleepingContactIsland_ShouldWakeWholeIsland()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        scenario.Context.Environment.Gravity = Fixed64.Zero;

        ScenarioBody<LSCuboidCollider> far = scenario.CreateCuboid(Vector3d.Zero);
        ScenarioBody<LSCuboidCollider> middle = scenario.CreateCuboid(new Vector3d(
            Fixed64.FromFraction(3, 4),
            Fixed64.Zero,
            Fixed64.Zero));
        ScenarioBody<LSCuboidCollider> driver = scenario.CreateCuboid(new Vector3d(
            Fixed64.FromFraction(3, 2),
            Fixed64.Zero,
            Fixed64.Zero));
        far.Body.Sleep();
        middle.Body.Sleep();

        scenario.Context.Simulate();
        scenario.Context.LateSimulate();

        driver.Body.IsSleeping.Should().BeFalse();
        middle.Body.IsSleeping.Should().BeFalse();
        far.Body.IsSleeping.Should().BeFalse();
    }

    [Fact]
    public void Simulate_WithUnconnectedSleepingIslandInAwakePartition_ShouldNotSolveSleepingIsland()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        scenario.Context.Environment.Gravity = Fixed64.Zero;

        ScenarioBody<LSSphereCollider> sleepingA = scenario.CreateSphere(Vector3d.Zero);
        ScenarioBody<LSSphereCollider> sleepingB = scenario.CreateSphere(new Vector3d(
            Fixed64.FromFraction(3, 4),
            Fixed64.Zero,
            Fixed64.Zero));
        ScenarioBody<LSSphereCollider> awake = scenario.CreateSphere(new Vector3d(
            (Fixed64)2,
            Fixed64.Zero,
            Fixed64.Zero));
        Vector3d sleepingAPosition = sleepingA.Body.Position3d;
        Vector3d sleepingBPosition = sleepingB.Body.Position3d;

        sleepingA.Body.Sleep();
        sleepingB.Body.Sleep();

        scenario.Context.Simulate();
        scenario.Context.LateSimulate();

        awake.Body.IsSleeping.Should().BeFalse();
        GetPair(sleepingA.Collider, sleepingB.Collider).Manifold.HasContact.Should().BeTrue();
        sleepingA.Body.IsSleeping.Should().BeTrue();
        sleepingB.Body.IsSleeping.Should().BeTrue();
        sleepingA.Body.Position3d.Should().Be(sleepingAPosition);
        sleepingB.Body.Position3d.Should().Be(sleepingBPosition);
    }

    private static CollisionPair GetPair(LSCollider first, LSCollider second)
    {
        if (first.TryGetCollisionPair(second.Id, out CollisionPair? firstPair) && firstPair != null)
            return firstPair;

        second.TryGetCollisionPair(first.Id, out CollisionPair? secondPair).Should().BeTrue();
        return secondPair!;
    }

    private static Fixed64 SolveObliqueGroups(int iterations, bool includeUnrelatedPair)
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        scenario.Context.Settings.DiscreteSolverIterations = iterations;
        var wall = scenario.CreateCuboid(Vector3d.Zero, immovable: true);
        var actor = scenario.CreateCuboid(new Vector3d(1, 0, 1), preventAngularForces: true);
        wall.Collider.Material = actor.Collider.Material = PhysicsMaterial.Frictionless;
        CollisionPair pair = scenario.CreatePair(wall.Collider, actor.Collider);
        pair.Manifold.AddContact(ContactAnchor.FromWorldPoint(Vector3d.Zero),
            ContactAnchor.FromWorldPoint(Vector3d.Zero), Fixed64.Zero, Vector3d.Right,
            PhysicsMaterial.Frictionless, PhysicsMaterial.Frictionless, featureNamespaceA: 0);
        pair.Manifold.AddContact(ContactAnchor.FromWorldPoint(Vector3d.Zero),
            ContactAnchor.FromWorldPoint(Vector3d.Zero), Fixed64.Zero,
            new Vector3d(Fixed64.FromFraction(-3, 5), Fixed64.Zero, Fixed64.FromFraction(4, 5)),
            PhysicsMaterial.Frictionless, PhysicsMaterial.Frictionless, featureNamespaceA: 1);
        actor.Body.AddLinearImpulse(new Vector3d(-1, 0, -1) * actor.Body.Mass);
        scenario.Context.Physics.QueueDiscreteResponsePair(pair);

        if (includeUnrelatedPair)
            QueueUnrelatedPair(scenario);

        SolveQueuedResponses(scenario);
        return -actor.Body.LinearVelocity.X;
    }

    private static void QueueUnrelatedPair(PhysicsScenarioBuilder scenario)
    {
        // A second island bypasses the queued-one-pair shortcut while retaining
        // a lone contact constraint in each island.
        var otherWall = scenario.CreateCuboid(new Vector3d(8, 0, 0), immovable: true);
        var otherActor = scenario.CreateCuboid(new Vector3d(9, 0, 0), preventAngularForces: true);
        CollisionPair otherPair = scenario.CreatePair(otherWall.Collider, otherActor.Collider);
        otherPair.Manifold.SetContact(Vector3d.Zero, Vector3d.Zero, Fixed64.Zero, Vector3d.Right);
        scenario.Context.Physics.QueueDiscreteResponsePair(otherPair);
    }

    private static void SolveQueuedResponses(PhysicsScenarioBuilder scenario) =>
        typeof(GravitasPhysicsService).GetMethod("SolveDiscreteResponsePairs",
            BindingFlags.Instance | BindingFlags.NonPublic)!.Invoke(scenario.Context.Physics, null);
}
