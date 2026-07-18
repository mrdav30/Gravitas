using FixedMathSharp;
using FluentAssertions;
using Gravitas.Colliders;
using Gravitas.CollisionHandling;
using Gravitas.Tests.Support;
using SwiftCollections.Pool;
using Xunit;

namespace Gravitas.Tests.CollisionHandlingTests;

[CollectionDefinition(Name, DisableParallelization = true)]
public sealed class PooledAxisHistoryCollection
{
    public const string Name = nameof(PooledAxisHistoryCollection);
}

[Collection(PooledAxisHistoryCollection.Name)]
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

    [Fact]
    public void ObbCapsule_EqualDepthAxes_ShouldKeepFirstFaceNormalAcrossPooledCapacityHistory()
    {
        SwiftHashSetPool<Vector3d>.Shared.Clear();
        SwiftListPool<Vector3d>.Shared.Clear();

        try
        {
            using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
            FixedQuaternion rotation = PhysicsScenarioBuilder.Yaw(45);
            ScenarioBody<LSCuboidCollider> cuboid = scenario.CreateCuboid(Vector3d.Zero, rotation);
            ScenarioBody<LSCapsuleCollider> capsule = scenario.CreateCapsule(Vector3d.Zero);
            CollisionPair pair = scenario.CreatePair(cuboid.Collider, capsule.Collider);
            Vector3d expectedNormal = (rotation * Vector3d.Forward).Normalized;

            foreach (int retainedCapacity in new[] { 8, 64 })
            {
                PrimeAxisPools(retainedCapacity);

                pair.CollisionType.Should().Be(CollisionType.OBBox_Capsule);
                CollisionDetection.DoCollisionCheck(pair).Should().BeTrue();
                pair.Manifold.Count.Should().Be(1);
                (pair.Manifold.PrimaryContact.Depth - Fixed64.One)
                    .Abs().Should().BeLessThanOrEqualTo(Fixed64.Epsilon);
                Vector3d.Distance(pair.Manifold.PrimaryContact.Normal, expectedNormal)
                    .Should().BeLessThanOrEqualTo(Fixed64.Epsilon);
            }
        }
        finally
        {
            SwiftHashSetPool<Vector3d>.Shared.Clear();
            SwiftListPool<Vector3d>.Shared.Clear();
        }
    }

    private static void PrimeAxisPools(int retainedCapacity)
    {
        var hashAxes = SwiftHashSetPool<Vector3d>.Shared.Rent();
        hashAxes.EnsureCapacity(retainedCapacity);
        SwiftHashSetPool<Vector3d>.Shared.Release(hashAxes);

        var orderedAxes = SwiftListPool<Vector3d>.Shared.Rent();
        orderedAxes.EnsureCapacity(retainedCapacity);
        SwiftListPool<Vector3d>.Shared.Release(orderedAxes);
    }
}
