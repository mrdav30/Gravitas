using FixedMathSharp;
using FluentAssertions;
using Gravitas.Colliders;
using Gravitas.CollisionHandling;
using Gravitas.Tests.Support;
using Xunit;

namespace Gravitas.Tests.CollisionHandlingTests;

public sealed class CuboidCollisionDetectionCoverageTests
{
    [Fact]
    public void RotatedCuboids_SeparatedOnFirstCuboidForwardFace_ShouldNotCollide()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        FixedQuaternion rotation = PhysicsScenarioBuilder.Yaw(45);
        ScenarioBody<LSCuboidCollider> first = scenario.CreateCuboid(Vector3d.Zero, rotation);
        ScenarioBody<LSCuboidCollider> second = scenario.CreateCuboid(
            (rotation * Vector3d.Forward) * Fixed64.FromFraction(5, 4));
        CollisionPair pair = scenario.CreatePair(first.Collider, second.Collider);

        first.Collider.Shape.Should().Be(ColliderType.OBBox);
        first.Collider.Bounds.Intersects(second.Collider.Bounds).Should().BeTrue();
        pair.CollisionType.Should().Be(CollisionType.Cuboid_Cuboid);
        CollisionDetection.DoCollisionCheck(pair).Should().BeFalse();
        pair.Manifold.Count.Should().Be(0);
    }

    [Fact]
    public void AxisAlignedCuboids_SeparatedOnlyOnY_ShouldNotCollide()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSCuboidCollider> first = scenario.CreateCuboid(Vector3d.Zero);
        ScenarioBody<LSCuboidCollider> second = scenario.CreateCuboid((Fixed64)2 * Vector3d.Up);
        CollisionPair pair = scenario.CreatePair(first.Collider, second.Collider);

        pair.CollisionType.Should().Be(CollisionType.Cuboid_Cuboid);
        CollisionDetection.DoCollisionCheck(pair).Should().BeFalse();
        pair.Manifold.Count.Should().Be(0);
    }

    [Fact]
    public void AxisAlignedCuboids_OverlappingTowardNegativeY_ShouldBuildNegativeYFaceManifold()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSCuboidCollider> first = scenario.CreateCuboid(Vector3d.Zero);
        ScenarioBody<LSCuboidCollider> second = scenario.CreateCuboid(
            -Fixed64.FromFraction(3, 4) * Vector3d.Up);
        CollisionPair pair = scenario.CreatePair(first.Collider, second.Collider);

        CollisionDetection.DoCollisionCheck(pair).Should().BeTrue();
        pair.Manifold.Count.Should().Be(ContactManifold.MaxContactCount);
        for (int i = 0; i < pair.Manifold.Count; i++)
        {
            ManifoldContact contact = pair.Manifold[i];
            contact.Depth.Should().Be(Fixed64.FromFraction(1, 4));
            contact.Normal.Should().Be(-Vector3d.Up);
            contact.PointA.Y.Should().Be(-Fixed64.Half);
            contact.PointB.Y.Should().Be(-Fixed64.FromFraction(1, 4));
        }
    }
}
