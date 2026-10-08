using FixedMathSharp;
using FluentAssertions;
using Gravitas.Colliders;
using Gravitas.CollisionHandling;
using Gravitas.Materials;
using Gravitas.Tests.Support;
using Xunit;

namespace Gravitas.Tests.CollisionHandlingTests;

public sealed class ContactGroupConstraintTests
{
    private static readonly Fixed64 Tolerance = Fixed64.FromFraction(1, 1_000_000);

    [Fact]
    public void FailureAfterEighthContact_ShouldRemoveOnlyFailedRowAndContinueSolving()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        scenario.Context.Environment.MaxSpeed = scenario.Context.Environment.MaxFallSpeed = Fixed64.MaxValue;
        var first = scenario.CreateSphere(Vector3d.Zero);
        var second = scenario.CreateSphere(Vector3d.Right);
        first.Collider.Material = second.Collider.Material = PhysicsMaterial.Frictionless;
        first.Body.ApplyCollisionLinearVelocityDelta(Vector3d.Right * Fixed64.MaxValue);
        second.Body.ApplyCollisionLinearVelocityDelta(Vector3d.Right * Fixed64.MaxValue);
        CollisionPair pair = scenario.CreatePair(first.Collider, second.Collider);
        SolidBody bodyA = pair.ColliderA.Body!;
        SolidBody bodyB = pair.ColliderB.Body!;
        bodyB.ApplyCollisionAngularVelocityDelta(Vector3d.Forward);
        for (int group = 0; group < 10; group++)
        {
            ContactGroupKey key = new(0, 0, group);
            // The ninth row sees closing speed from B's angular motion. Its
            // admitted +X linear delta cannot be added to an already maximal
            // velocity. Healthy rows before and after it have no closing speed.
            bool fails = group == 8;
            Add(pair, key, (ulong)(group + 1),
                bodyA.WorldCenterOfMass,
                bodyB.WorldCenterOfMass + (fails ? Vector3d.Up : Vector3d.Zero),
                Fixed64.Zero, fails ? Vector3d.Right : Vector3d.Up);
            pair.StoreWarmStartImpulse(key, (ulong)(group + 1),
                fails ? Vector3d.Right : Vector3d.Up,
                fails ? Fixed64.Half : Fixed64.Zero, Fixed64.Zero, Fixed64.Zero);
        }
        pair.Manifold.GetGroupStartIndex(8).Should().Be(8);
        Vector3d initialA = bodyA.LinearVelocity;
        Vector3d initialB = bodyB.LinearVelocity;
        Vector3d initialAngularB = bodyB.AngularVelocity;

        CollisionResponse.CalculateImpulse(pair, applyCachedImpulse: false, applyPositionCorrection: false);

        bodyA.LinearVelocity.Should().Be(initialA);
        bodyB.LinearVelocity.Should().Be(initialB);
        bodyA.AngularVelocity.Should().Be(Vector3d.Zero);
        bodyB.AngularVelocity.Should().Be(initialAngularB);
        for (int group = 0; group < 10; group++)
        {
            bool retained = pair.TryGetWarmStartImpulse(new ContactGroupKey(0, 0, group),
                (ulong)(group + 1), out ContactWarmStartImpulse cached);
            retained.Should().Be(group != 8);
            if (retained) cached.NormalImpulse.Should().Be(Fixed64.Zero);
        }
    }

    [Theory]
    [InlineData(1, false)]
    [InlineData(4, false)]
    [InlineData(4, true)]
    [InlineData(9, true)]
    public void RedundantNormalSamples_WithFrozenRotation_ShouldNotMultiplyLinearResponse(int samples, bool independentGroups)
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        var wall = scenario.CreateCuboid(Vector3d.Zero, immovable: true);
        var actor = scenario.CreateCuboid(Vector3d.Right, preventAngularForces: true);
        wall.Collider.Material = actor.Collider.Material = PhysicsMaterial.Frictionless;
        CollisionPair pair = scenario.CreatePair(wall.Collider, actor.Collider);
        for (int sample = 0; sample < samples; sample++)
        {
            Vector3d point = Vector3d.Right * Fixed64.Half + Vector3d.Up * (Fixed64)sample;
            Add(pair, new ContactGroupKey(0, 0, independentGroups ? sample : 0),
                (ulong)(sample + 1), point, point, Fixed64.Zero, Vector3d.Right);
        }
        actor.Body.AddLinearImpulse(Vector3d.Left * Fixed64.Two);

        CollisionResponse.CalculateImpulse(pair, applyCachedImpulse: false, applyPositionCorrection: false);

        actor.Body.LinearVelocity.Should().Be(Vector3d.Zero);
        actor.Body.AngularVelocity.Should().Be(Vector3d.Zero);
        Fixed64 totalImpulse = Fixed64.Zero;
        for (int group = 0; group < pair.Manifold.GroupCount; group++)
        {
            ref ContactGroup current = ref pair.Manifold.GetGroup(group);
            for (int point = 0; point < current.Count; point++)
            {
                pair.TryGetWarmStartImpulse(current.Key, current[point].ContactId,
                    out ContactWarmStartImpulse cached).Should().BeTrue();
                totalImpulse += cached.NormalImpulse;
            }
        }
        totalImpulse.Should().Be(Fixed64.Two);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void DuplicateCorrectionDirection_ShouldApplyOnlyDeepestDepthWithStableTies(bool deeperGroup)
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        var wall = scenario.CreateCuboid(Vector3d.Zero, immovable: true);
        var actor = scenario.CreateCuboid(Vector3d.Right, preventAngularForces: true);
        wall.Collider.Material = actor.Collider.Material = PhysicsMaterial.Frictionless;
        CollisionPair pair = scenario.CreatePair(wall.Collider, actor.Collider);
        // Equal-depth samples have distinct features and anchors inside one
        // group. The second sample must not repeat its predecessor's correction.
        Add(pair, default, 1, Vector3d.Down, Vector3d.Down, Fixed64.Half, Vector3d.Right);
        Add(pair, default, 2, Vector3d.Up, Vector3d.Up, Fixed64.Half, Vector3d.Right);
        Fixed64 deepest = deeperGroup ? Fixed64.One : Fixed64.Half;
        Add(pair, new ContactGroupKey(0, 0, 1), 3, Vector3d.Zero, Vector3d.Zero, deepest, Vector3d.Right);
        Add(pair, new ContactGroupKey(0, 0, 2), 4, Vector3d.Forward, Vector3d.Forward,
            Fixed64.Quarter, Vector3d.Right);
        pair.Manifold.GetGroupContactCount(0).Should().Be(2);
        Vector3d initial = actor.Body.Position3d;

        CollisionResponse.CalculateImpulse(pair);

        Fixed64 expected = (deepest - CollisionResponse.PenetrationSlop)
            * CollisionResponse.PenetrationCorrectionPercent;
        ((actor.Body.Position3d - initial) - Vector3d.Right * expected)
            .Magnitude.Should().BeLessThan(Tolerance);
        actor.Body.LinearVelocity.Should().Be(Vector3d.Zero);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void DistinctCorrectionDirections_ShouldRemainIndependent(bool opposing)
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        var wall = scenario.CreateCuboid(Vector3d.Zero, immovable: true);
        var actor = scenario.CreateCuboid(Vector3d.Right, preventAngularForces: true);
        wall.Collider.Material = actor.Collider.Material = PhysicsMaterial.Frictionless;
        CollisionPair pair = scenario.CreatePair(wall.Collider, actor.Collider);
        Vector3d secondNormal = opposing ? Vector3d.Left
            : new Vector3d(Fixed64.One, Fixed64.FromFraction(1, 1_000), Fixed64.Zero).Normalized;
        Add(pair, default, 1, Vector3d.Zero, Vector3d.Zero, Fixed64.Half, Vector3d.Right);
        Add(pair, new ContactGroupKey(0, 0, 1), 2, Vector3d.Zero, Vector3d.Zero, Fixed64.Half, secondNormal);
        Vector3d initial = actor.Body.Position3d;
        Fixed64 correction = (Fixed64.Half - CollisionResponse.PenetrationSlop)
            * CollisionResponse.PenetrationCorrectionPercent;
        Vector3d expected = (pair.Manifold[0].Normal + pair.Manifold[1].Normal) * correction;

        CollisionResponse.CalculateImpulse(pair);

        ((actor.Body.Position3d - initial) - expected).Magnitude.Should().BeLessThan(Tolerance);
        if (!opposing)
            (actor.Body.Position3d.X - initial.X).Should().BeGreaterThan(correction);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void DisconnectedSupports_ShouldPreserveSeparateLeversAndRotatedInertia(bool rotated)
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        var wall = scenario.CreateCuboid(Vector3d.Zero, immovable: true);
        var actor = scenario.CreateBody(new LSCuboidCollider { Size = new Vector3d(1, 2, 3) },
            Vector3d.Up, rotated ? PhysicsScenarioBuilder.Yaw(45) : FixedQuaternion.Identity);
        wall.Collider.Material = actor.Collider.Material = PhysicsMaterial.Frictionless;
        CollisionPair pair = scenario.CreatePair(wall.Collider, actor.Collider);
        for (int group = 0; group < 2; group++)
        {
            Vector3d point = actor.Body.WorldCenterOfMass
                + Vector3d.Right * (group == 0 ? -Fixed64.Half : Fixed64.Half);
            Add(pair, new ContactGroupKey(0, 0, group), (ulong)(group + 1),
                point, point, Fixed64.Zero, Vector3d.Up);
        }
        actor.Body.AddLinearImpulse(Vector3d.Down * Fixed64.Two);

        CollisionResponse.CalculateImpulse(pair, applyCachedImpulse: false, applyPositionCorrection: false);

        actor.Body.LinearVelocity.Y.Should().BeGreaterThan(-Fixed64.Two);
        actor.Body.AngularVelocity.Z.Abs().Should().BeGreaterThan(Fixed64.Zero);
        if (rotated) actor.Body.AngularVelocity.X.Abs().Should().BeGreaterThan(Fixed64.Zero);
        else actor.Body.AngularVelocity.X.Should().Be(Fixed64.Zero);
        for (int group = 0; group < 2; group++)
        {
            pair.TryGetWarmStartImpulse(new ContactGroupKey(0, 0, group), (ulong)(group + 1),
                out ContactWarmStartImpulse cached).Should().BeTrue();
            cached.NormalImpulse.Should().BeGreaterThan(Fixed64.Zero);
        }
    }

    private static void Add(CollisionPair pair, ContactGroupKey key, ulong id,
        Vector3d first, Vector3d second, Fixed64 depth, Vector3d normal) =>
        pair.Manifold.AddContact(key, new ManifoldContact(id, first, second, depth, normal));
}
