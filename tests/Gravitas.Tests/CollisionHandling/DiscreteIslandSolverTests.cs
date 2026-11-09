using System.Reflection;
using FixedMathSharp;
using FluentAssertions;
using Gravitas.Colliders;
using Gravitas.CollisionHandling;
using Gravitas.Diagnostics;
using Gravitas.Materials;
using Gravitas.Tests.Support;
using Xunit;

namespace Gravitas.Tests.CollisionHandlingTests;

public sealed class DiscreteIslandSolverTests
{
    public enum SeedFallback { MissingSnapshot, Restitution, FrozenLoad, EqualMass, OtherShape }

    [Fact]
    public void HeavyLoadOnSupportedBody_ShouldResolveBothNormalVelocitiesTogether()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        var floor = new LSCuboidCollider();
        scenario.InitializeStaticCollider(floor, Vector3d.Zero);
        var lower = scenario.CreateCuboid(Vector3d.Up, mass: Fixed64.One);
        var upper = scenario.CreateCuboid(Vector3d.Up * (Fixed64)2, mass: (Fixed64)100);
        PhysicsMaterial material = PhysicsMaterialTestHelper.WithFrictionAndRestitution(Fixed64.Zero, Fixed64.Zero);
        floor.Material = lower.Collider.Material = upper.Collider.Material = material;
        CollisionPair support = scenario.CreatePair(floor, lower.Collider);
        CollisionPair load = scenario.CreatePair(lower.Collider, upper.Collider);
        support.Manifold.SetContact(Vector3d.Up * Fixed64.Half, Vector3d.Up * Fixed64.Half,
            Fixed64.Zero, Vector3d.Up);
        load.Manifold.SetContact(Vector3d.Up * Fixed64.FromFraction(3, 2),
            Vector3d.Up * Fixed64.FromFraction(3, 2), Fixed64.Zero, Vector3d.Up);
        upper.Body.AddLinearImpulse(Vector3d.Down * upper.Body.Mass);
        scenario.Context.Diagnostics.Enable();
        scenario.Context.Physics.QueueDiscreteResponsePair(load);
        scenario.Context.Physics.QueueDiscreteResponsePair(support);

        SolveQueuedResponses(scenario);

        Fixed64 tolerance = Fixed64.FromFraction(1, 10000);
        lower.Body.LinearVelocity.Y.Abs().Should().BeLessThan(tolerance);
        upper.Body.LinearVelocity.Y.Abs().Should().BeLessThan(tolerance);
        lower.Body.AngularVelocity.Should().Be(Vector3d.Zero);
        upper.Body.AngularVelocity.Should().Be(Vector3d.Zero);

        int floorResponse = -1, loadResponse = -1, firstVelocity = int.MaxValue;
        foreach (GravitasDiagnosticEvent item in scenario.Context.Diagnostics.Events)
        {
            if (item.TryAsResponseImpulse(out GravitasResponseImpulseDiagnosticView response)
                && response.Impulse.Y > (Fixed64)99)
            {
                if (response.ColliderAId == floor.Id) floorResponse = item.Sequence;
                if (response.ColliderAId == lower.Collider.Id) loadResponse = item.Sequence;
            }
            if (item.Kind == GravitasDiagnosticEventKind.LinearVelocityDelta)
                firstVelocity = System.Math.Min(firstVelocity, item.Sequence);
        }
        floorResponse.Should().BeGreaterThanOrEqualTo(0).And.BeLessThan(firstVelocity);
        loadResponse.Should().BeGreaterThanOrEqualTo(0).And.BeLessThan(firstVelocity);
        scenario.Context.Diagnostics.Clear();
        CollisionResponse.PrepareSolve(support);
        CollisionResponse.PrepareSolve(load);
        CollisionResponse.TrySeedSupportedIslandNormals(support, load, out _).Should().BeTrue();
        scenario.Context.Diagnostics.EventCount.Should().Be(0);
        scenario.Context.Diagnostics.Disable();

        bool seeded = true;
        long allocated = AllocationTestHelper.MeasureSteadyState(() =>
        {
            upper.Body.AddLinearImpulse(Vector3d.Down * upper.Body.Mass);
            CollisionResponse.PrepareSolve(support);
            CollisionResponse.PrepareSolve(load);
            seeded &= CollisionResponse.TrySeedSupportedIslandNormals(support, load, out _);
        });
        seeded.Should().BeTrue();
        allocated.Should().Be(0);
    }

    [Theory]
    [InlineData(false, 14_268_994)]
    [InlineData(true, 100)]
    [InlineData(false, 100_000_000)]
    public void CoupledSeed_ShouldLeaveBodiesUnchangedWhenPreparedResponseFails(bool exceedsSpeedLimit, int loadMass)
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        Vector3d normal = exceedsSpeedLimit ? Vector3d.Right
            : new Vector3d(Fixed64.FromFraction(3, 5), Fixed64.FromFraction(4, 5), Fixed64.Zero);
        if (exceedsSpeedLimit)
            scenario.Context.Environment.MaxSpeed = Fixed64.One;
        var floor = new LSCuboidCollider();
        scenario.InitializeStaticCollider(floor, Vector3d.Zero);
        var lower = scenario.CreateCuboid(normal);
        var upper = scenario.CreateCuboid(normal * (Fixed64)2, mass: (Fixed64)loadMass);
        PhysicsMaterial material = PhysicsMaterialTestHelper.WithFrictionAndRestitution(Fixed64.Zero, Fixed64.Zero);
        floor.Material = lower.Collider.Material = upper.Collider.Material = material;
        CollisionPair support = scenario.CreatePair(floor, lower.Collider);
        CollisionPair load = scenario.CreatePair(lower.Collider, upper.Collider);
        support.Manifold.SetContact(Vector3d.Zero, lower.Body.Position3d, Fixed64.Zero,
            exceedsSpeedLimit ? Vector3d.Up : normal);
        load.Manifold.SetContact(lower.Body.Position3d, upper.Body.Position3d, Fixed64.Zero, normal);
        lower.Body.AddLinearImpulse(Vector3d.Forward);
        upper.Body.AddLinearImpulse(-normal * upper.Body.Mass);
        Vector3d beforeLower = lower.Body.LinearVelocity;
        Vector3d beforeUpper = upper.Body.LinearVelocity;
        CollisionResponse.PrepareSolve(support);
        CollisionResponse.PrepareSolve(load);

        CollisionResponse.TrySeedSupportedIslandNormals(support, load, out bool corrected).Should().BeFalse();

        corrected.Should().BeFalse();
        lower.Body.LinearVelocity.Should().Be(beforeLower);
        upper.Body.LinearVelocity.Should().Be(beforeUpper);
        support.TryGetWarmStartImpulse(support.Manifold[0].ContactId, out _).Should().BeFalse();
        load.TryGetWarmStartImpulse(load.Manifold[0].ContactId, out _).Should().BeFalse();
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void CoupledSeed_ShouldRejectOverflowingCachedFrictionWithoutPublishing(bool angularOverflow)
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        var floor = new LSCuboidCollider();
        scenario.InitializeStaticCollider(floor, Vector3d.Zero);
        var lower = scenario.CreateBody(new LSCuboidCollider { Size = new Vector3d(4, 1, 1) },
            Vector3d.Up, FixedQuaternion.Identity);
        var upper = scenario.CreateCuboid(Vector3d.Up * (Fixed64)2, mass: (Fixed64)100);
        var material = new PhysicsMaterial(Fixed64.One, Fixed64.One, Fixed64.Zero);
        floor.Material = lower.Collider.Material = upper.Collider.Material = material;
        CollisionPair support = scenario.CreatePair(floor, lower.Collider);
        CollisionPair load = scenario.CreatePair(lower.Collider, upper.Collider);
        if (angularOverflow)
            support.Manifold.AddContact(Vector3d.Right * (Fixed64)2,
                lower.Body.Position3d + Vector3d.Right * (Fixed64)2, Fixed64.Zero, Vector3d.Up);
        else
        {
            support.Manifold.AddContact(Vector3d.Zero, lower.Body.Position3d, Fixed64.Zero, Vector3d.Up);
            support.Manifold.AddContact(Vector3d.Right, lower.Body.Position3d, Fixed64.Zero, Vector3d.Up);
        }
        load.Manifold.SetContact(lower.Body.Position3d, upper.Body.Position3d, Fixed64.Zero, Vector3d.Up);
        support.Manifold.Count.Should().Be(angularOverflow ? 1 : 2);
        ContactGroupKey group = support.Manifold.GetGroup(0).Key;
        Fixed64 impulse = (Fixed64)1_500_000_000;
        foreach (ManifoldContact row in support.Manifold)
            support.StoreWarmStartImpulse(group, row.ContactId, row.Normal, impulse, impulse, Fixed64.Zero);
        CollisionResponse.PrepareSolve(support);
        CollisionResponse.PrepareSolve(load);

        CollisionResponse.TrySeedSupportedIslandNormals(support, load, out bool corrected).Should().BeFalse();

        corrected.Should().BeFalse();
        lower.Body.LinearVelocity.Should().Be(Vector3d.Zero);
        upper.Body.LinearVelocity.Should().Be(Vector3d.Zero);
        foreach (ManifoldContact row in support.Manifold)
        {
            support.TryGetWarmStartImpulse(group, row.ContactId, out ContactWarmStartImpulse cached).Should().BeTrue();
            cached.TangentImpulse.Should().Be(impulse);
        }
    }

    [Fact]
    public void CoupledSeed_ShouldLeaveMixedDirectionTranslationToPairCorrection()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        var floor = new LSCuboidCollider();
        scenario.InitializeStaticCollider(floor, Vector3d.Zero);
        var lower = scenario.CreateCuboid(Vector3d.Up);
        var upper = scenario.CreateCuboid(Vector3d.Up * (Fixed64)2, mass: (Fixed64)100);
        PhysicsMaterial material = PhysicsMaterialTestHelper.WithFrictionAndRestitution(Fixed64.Zero, Fixed64.Zero);
        floor.Material = lower.Collider.Material = upper.Collider.Material = material;
        CollisionPair support = scenario.CreatePair(floor, lower.Collider);
        CollisionPair load = scenario.CreatePair(upper.Collider, lower.Collider);
        Fixed64 depth = Fixed64.FromFraction(1, 32);
        support.Manifold.AddContact(Vector3d.Zero, lower.Body.Position3d, depth, Vector3d.Up);
        support.Manifold.AddContact(Vector3d.Right, lower.Body.Position3d, depth, Vector3d.Right);
        load.Manifold.SetContact(upper.Body.Position3d, lower.Body.Position3d, depth, Vector3d.Down);
        CollisionResponse.PrepareSolve(support);
        CollisionResponse.PrepareSolve(load);

        // Reverse the input order to exercise the same island when the load pair sorts first.
        CollisionResponse.TrySeedSupportedIslandNormals(load, support, out bool corrected).Should().BeTrue();

        corrected.Should().BeFalse();
        lower.Body.Position3d.Should().Be(Vector3d.Up);
        upper.Body.Position3d.Should().Be(Vector3d.Up * (Fixed64)2);
    }

    [Theory]
    [InlineData(SeedFallback.MissingSnapshot)]
    [InlineData(SeedFallback.Restitution)]
    [InlineData(SeedFallback.FrozenLoad)]
    [InlineData(SeedFallback.EqualMass)]
    [InlineData(SeedFallback.OtherShape)]
    public void CoupledSeed_ShouldLeaveUnsupportedInputsToThePairSolver(SeedFallback input)
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        var floor = new LSCuboidCollider();
        scenario.InitializeStaticCollider(floor, Vector3d.Zero);
        var lower = scenario.CreateCuboid(Vector3d.Up);
        LSCollider shape = input == SeedFallback.OtherShape ? new LSCapsuleCollider() : new LSCuboidCollider();
        var upper = scenario.CreateBody(shape, Vector3d.Up * (Fixed64)2, FixedQuaternion.Identity,
            mass: input == SeedFallback.EqualMass ? Fixed64.One : (Fixed64)100,
            preventAngularForces: input == SeedFallback.FrozenLoad);
        PhysicsMaterial material = PhysicsMaterialTestHelper.WithFrictionAndRestitution(Fixed64.Zero,
            input == SeedFallback.Restitution ? Fixed64.Half : Fixed64.Zero);
        floor.Material = lower.Collider.Material = upper.Collider.Material = material;
        CollisionPair support = scenario.CreatePair(floor, lower.Collider);
        CollisionPair load = scenario.CreatePair(lower.Collider, upper.Collider);
        support.Manifold.SetContact(Vector3d.Zero, lower.Body.Position3d, Fixed64.Zero, Vector3d.Up);
        load.Manifold.SetContact(lower.Body.Position3d, upper.Body.Position3d, Fixed64.Zero, Vector3d.Up);
        upper.Body.AddLinearImpulse(Vector3d.Down * upper.Body.Mass);
        Vector3d incoming = upper.Body.LinearVelocity;
        if (input != SeedFallback.MissingSnapshot)
            CollisionResponse.PrepareSolve(support);
        CollisionResponse.PrepareSolve(load);

        CollisionResponse.TrySeedSupportedIslandNormals(support, load, out bool corrected).Should().BeFalse();

        corrected.Should().BeFalse();
        lower.Body.LinearVelocity.Should().Be(Vector3d.Zero);
        upper.Body.LinearVelocity.Should().Be(incoming);
    }

    [Fact]
    public void CoupledTranslation_ShouldKeepBothPosesWhenEitherCorrectionOverflows()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        Vector3d upperPosition = new(Fixed64.Zero, Fixed64.MaxValue - Fixed64.FromFraction(1, 100), Fixed64.Zero);
        Vector3d lowerPosition = upperPosition - Vector3d.Up;
        var floor = new LSCuboidCollider();
        scenario.InitializeStaticCollider(floor, lowerPosition - Vector3d.Up);
        var lower = scenario.CreateCuboid(lowerPosition);
        var upper = scenario.CreateCuboid(upperPosition, mass: (Fixed64)100);
        PhysicsMaterial material = PhysicsMaterialTestHelper.WithFrictionAndRestitution(Fixed64.Zero, Fixed64.Zero);
        floor.Material = lower.Collider.Material = upper.Collider.Material = material;
        CollisionPair support = scenario.CreatePair(floor, lower.Collider);
        CollisionPair load = scenario.CreatePair(lower.Collider, upper.Collider);
        Fixed64 depth = Fixed64.FromFraction(1, 8);
        support.Manifold.SetContact(lowerPosition, lowerPosition, depth, Vector3d.Up);
        load.Manifold.SetContact(lowerPosition, upperPosition, depth, Vector3d.Up);
        CollisionResponse.PrepareSolve(support);
        CollisionResponse.PrepareSolve(load);

        CollisionResponse.TrySeedSupportedIslandNormals(support, load, out bool corrected).Should().BeTrue();

        corrected.Should().BeFalse();
        lower.Body.Position3d.Should().Be(lowerPosition);
        upper.Body.Position3d.Should().Be(upperPosition);
    }

    [Fact]
    public void CoupledSeed_ShouldKeepTwoFixedSupportsOnThePairPath()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        var left = new LSCuboidCollider();
        var right = new LSCuboidCollider();
        scenario.InitializeStaticCollider(left, -Vector3d.Right * Fixed64.Half);
        scenario.InitializeStaticCollider(right, Vector3d.Right * Fixed64.Half);
        var body = scenario.CreateCuboid(Vector3d.Up);
        CollisionPair first = scenario.CreatePair(left, body.Collider);
        CollisionPair second = scenario.CreatePair(right, body.Collider);
        CollisionDetection.DoCollisionCheck(first).Should().BeTrue();
        CollisionDetection.DoCollisionCheck(second).Should().BeTrue();
        CollisionResponse.PrepareSolve(first);
        CollisionResponse.PrepareSolve(second);

        CollisionResponse.TrySeedSupportedIslandNormals(first, second, out bool corrected).Should().BeFalse();

        corrected.Should().BeFalse();
        body.Body.Position3d.Should().Be(Vector3d.Up);
    }

    [Fact]
    public void Simulate_HeavyLoadOnFreelyRotatingCuboid_ShouldKeepBothSupportInterfaces()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        scenario.Context.SetFrameRate(60);
        scenario.Context.Settings.DiscreteSolverIterations = 6;
        scenario.Context.Environment.Gravity = Fixed64.FromFraction(49, 5);
        scenario.Context.Environment.AirDensity = Fixed64.Zero;
        scenario.Context.Environment.DampingFactor = Fixed64.Zero;
        var floor = new LSCuboidCollider { Size = new Vector3d(16, 1, 16) };
        scenario.InitializeStaticCollider(floor, -Vector3d.Up * Fixed64.Half);
        var lower = scenario.CreateBody(new LSCuboidCollider { Size = new Vector3d(2, 1, 2) },
            Vector3d.Up * Fixed64.FromFraction(17, 32), FixedQuaternion.Identity);
        var upper = scenario.CreateBody(new LSCuboidCollider { Size = new Vector3d(2, 1, 2) },
            new Vector3d(Fixed64.FromFraction(1, 64), Fixed64.FromFraction(25, 16), Fixed64.Zero),
            FixedQuaternion.Identity, mass: (Fixed64)100);
        var material = new PhysicsMaterial(Fixed64.Half, Fixed64.FromFraction(1, 3), Fixed64.Zero);
        floor.Material = lower.Collider.Material = upper.Collider.Material = material;
        lower.Body.LinearDragCoefficient = upper.Body.LinearDragCoefficient = Fixed64.Zero;
        lower.Body.UseManualGrounding();
        upper.Body.UseManualGrounding();

        for (int frame = 0; frame < 32; frame++)
        {
            scenario.Context.Simulate();
            scenario.Context.LateSimulate();
        }

        lower.Body.WorldCenterOfMass.Y.Should().BeGreaterThan(Fixed64.FromFraction(9, 20));
        upper.Body.WorldCenterOfMass.Y.Should().BeGreaterThan(Fixed64.FromFraction(7, 5));
        GetPair(floor, lower.Collider).Manifold.HasContact.Should().BeTrue();
        GetPair(lower.Collider, upper.Collider).Manifold.HasContact.Should().BeTrue();
        CollisionDetection.DoCollisionCheck(scenario.CreatePair(floor, upper.Collider)).Should().BeFalse();
    }

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
