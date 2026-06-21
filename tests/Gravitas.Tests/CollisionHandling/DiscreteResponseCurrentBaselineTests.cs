using FixedMathSharp;
using FluentAssertions;
using Gravitas.Colliders;
using Gravitas.CollisionHandling;
using Gravitas.Tests.Support;
using Xunit;

namespace Gravitas.Tests.CollisionHandlingTests;

public sealed class DiscreteResponseCurrentBaselineTests
{
    [Fact]
    public void CurrentBaseline_Stored3DWarmStartImpulse_ShouldNotAffectFreshSinglePairSolve()
    {
        ResponseState withoutCachedImpulse = RunSinglePairResponse(preloadWarmStart: false);
        ResponseState withCachedImpulse = RunSinglePairResponse(preloadWarmStart: true);

        withCachedImpulse.Should().Be(withoutCachedImpulse);
    }

    [Fact]
    public void CurrentBaseline_RestingStackUnderGravity_ShouldRemainAwakeOverShortWindow()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        _ = scenario.CreateCuboid(PhysicsScenarioBuilder.Vector(0, 0, 0), immovable: true);
        ScenarioBody<LSCuboidCollider> lower = scenario.CreateCuboid(PhysicsScenarioBuilder.Vector(0, 1, 0));
        ScenarioBody<LSCuboidCollider> upper = scenario.CreateCuboid(PhysicsScenarioBuilder.Vector(0, 2, 0));
        lower.Body.SleepFrameThreshold = 100;
        upper.Body.SleepFrameThreshold = 100;

        for (int i = 0; i < 8; i++)
        {
            scenario.Context.Simulate();
            scenario.Context.LateSimulate();
        }

        lower.Body.IsSleeping.Should().BeFalse();
        upper.Body.IsSleeping.Should().BeFalse();
        lower.Body.Position3d.Y.Should().NotBe((Fixed64)1);
        upper.Body.Position3d.Y.Should().NotBe((Fixed64)2);
    }

    [Fact]
    public void CurrentBaseline_CylinderRimContact_ShouldReduceToSingleContact()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        Fixed64 rimTouchOffset = Fixed64.Half * FixedMath.Sqrt(Fixed64.Half);
        ScenarioBody<LSCylinderCollider> cylinder = scenario.CreateCylinder(PhysicsScenarioBuilder.Vector(0, 0, 0));
        ScenarioBody<LSSphereCollider> sphere = scenario.CreateSphere(new Vector3d(
            Fixed64.Half + rimTouchOffset,
            Fixed64.Half + rimTouchOffset,
            Fixed64.Zero));
        CollisionPair pair = scenario.CreatePair(cylinder.Collider, sphere.Collider);

        CollisionDetection.DoCollisionCheck(pair).Should().BeTrue();

        pair.Manifold.Count.Should().Be(1);
        pair.Manifold.PrimaryContact.Depth.Should().BeLessThan(Fixed64.FromFraction(1, 10_000));
    }

    [Fact]
    public void CurrentBaseline_MeshCuboidFaceContact_ShouldReduceToSingleTriangleContact()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSMeshCollider> floor = scenario.CreateBody(
            MeshTestFixtures.CreateConvexQuadFloor(),
            PhysicsScenarioBuilder.Vector(0, 0, 0),
            FixedQuaternion.Identity,
            immovable: true);
        ScenarioBody<LSCuboidCollider> cuboid = scenario.CreateCuboid(new Vector3d(
            Fixed64.Zero,
            Fixed64.FromFraction(1, 4),
            Fixed64.Zero));
        CollisionPair pair = scenario.CreatePair(floor.Collider, cuboid.Collider);

        CollisionDetection.DoCollisionCheck(pair).Should().BeTrue();

        pair.Manifold.Count.Should().Be(1);
        pair.Manifold.PrimaryContact.Normal.Y.Should().BeGreaterThan(Fixed64.Zero);
    }

    private static ResponseState RunSinglePairResponse(bool preloadWarmStart)
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSCuboidCollider> wall = scenario.CreateCuboid(
            PhysicsScenarioBuilder.Vector(0, 0, 0),
            immovable: true);
        ScenarioBody<LSSphereCollider> mover = scenario.CreateSphere(new Vector3d(
            Fixed64.FromFraction(3, 4),
            Fixed64.Zero,
            Fixed64.Zero));
        mover.Body.AddLinearImpulse(new Vector3d((Fixed64)(-60), Fixed64.Zero, (Fixed64)30));
        CollisionPair pair = scenario.CreatePair(wall.Collider, mover.Collider);
        CollisionDetection.DoCollisionCheck(pair).Should().BeTrue();

        if (preloadWarmStart)
            pair.StoreWarmStartImpulse(pair.Manifold.PrimaryContact.ContactId, (Fixed64)16, (Fixed64)8);

        CollisionResponse.CalculateImpulse(pair);

        return new ResponseState(
            wall.Body.Position3d,
            mover.Body.Position3d,
            wall.Body.LinearVelocity,
            mover.Body.LinearVelocity,
            wall.Body.AngularVelocity,
            mover.Body.AngularVelocity);
    }

    private readonly record struct ResponseState(
        Vector3d WallPosition,
        Vector3d MoverPosition,
        Vector3d WallVelocity,
        Vector3d MoverVelocity,
        Vector3d WallAngularVelocity,
        Vector3d MoverAngularVelocity);
}
