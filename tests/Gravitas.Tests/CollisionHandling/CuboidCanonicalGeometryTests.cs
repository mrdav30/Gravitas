using FixedMathSharp;
using FluentAssertions;
using Gravitas.Colliders;
using Gravitas.CollisionHandling;
using Gravitas.Tests.Support;
using Xunit;

namespace Gravitas.Tests.CollisionHandlingTests;

public sealed class CuboidCanonicalGeometryTests
{
    [Fact]
    public void ParallelRotatedCuboids_ShouldUseMatchedFaceCenterAnchors()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        FixedQuaternion rotation = PhysicsScenarioBuilder.Yaw(45);
        Vector3d normal = rotation * Vector3d.Right;
        ScenarioBody<LSCuboidCollider> first = scenario.CreateCuboid(
            Vector3d.Zero,
            rotation);
        ScenarioBody<LSCuboidCollider> second = scenario.CreateBody(
            new LSCuboidCollider
            {
                Size = Vector3d.One * Fixed64.Half
            },
            normal * Fixed64.FromFraction(3, 5),
            rotation);
        CollisionPair pair = scenario.CreatePair(first.Collider, second.Collider);

        CollisionDetection.DoCollisionCheck(pair).Should().BeTrue();
        pair.Manifold.Count.Should().Be(1);
        ManifoldContact contact = pair.Manifold.PrimaryContact;
        Vector3d.Distance(contact.Normal, normal)
            .Should().BeLessThanOrEqualTo(Fixed64.Epsilon);
        contact.AnchorA.TryGetOffset(out Vector3d firstOffset)
            .Should().BeTrue();
        contact.AnchorB.TryGetOffset(out Vector3d secondOffset)
            .Should().BeTrue();
        Vector3d.Distance(firstOffset, normal * Fixed64.Half)
            .Should().BeLessThanOrEqualTo(Fixed64.Epsilon);
        Vector3d.Distance(secondOffset, -normal * Fixed64.FromFraction(1, 4))
            .Should().BeLessThanOrEqualTo(Fixed64.Epsilon);
        (contact.Depth - Fixed64.FromFraction(3, 20))
            .Abs().Should().BeLessThanOrEqualTo(Fixed64.Epsilon);
    }

    [Theory]
    [InlineData(true, ColliderType.Sphere)]
    [InlineData(true, ColliderType.Capsule)]
    [InlineData(true, ColliderType.Cylinder)]
    [InlineData(true, ColliderType.AABox)]
    [InlineData(false, ColliderType.Sphere)]
    [InlineData(false, ColliderType.Capsule)]
    [InlineData(false, ColliderType.Cylinder)]
    [InlineData(false, ColliderType.AABox)]
    public void RotatedCuboidNearScalarFace_ShouldRetainContainedTargetContacts(
        bool positiveFace,
        ColliderType targetType)
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        Vector3d center = positiveFace
            ? new Vector3d(
                Fixed64.MaxValue - Fixed64.One,
                Fixed64.Zero,
                Fixed64.Zero)
            : new Vector3d(
                Fixed64.MinValue + Fixed64.One,
                Fixed64.Zero,
                Fixed64.Zero);
        ScenarioBody<LSCuboidCollider> rotated = scenario.CreateBody(
            new LSCuboidCollider
            {
                Size = new Vector3d((Fixed64)4, Fixed64.One, Fixed64.One)
            },
            center,
            PhysicsScenarioBuilder.Yaw(45));
        Vector3d targetCenter = positiveFace
            ? new Vector3d(
                Fixed64.MaxValue - Fixed64.FromFraction(1, 200),
                Fixed64.Zero,
                -Fixed64.One)
            : new Vector3d(
                Fixed64.MinValue + Fixed64.FromFraction(1, 200),
                Fixed64.Zero,
                Fixed64.One);
        LSCollider target = CreateTarget(targetType);
        _ = scenario.CreateBody(
            target,
            targetCenter,
            FixedQuaternion.Identity);
        CollisionPair pair = scenario.CreatePair(rotated.Collider, target);

        rotated.Collider.OrientedBox.Contains(targetCenter).Should().BeTrue();
        CollisionDetection.DoCollisionCheck(pair).Should().BeTrue();
        pair.Manifold.Count.Should().BeGreaterThan(0);
        pair.Manifold[0].AnchorA.Origin.Should().Be(pair.ColliderA.Center);
        pair.Manifold[0].AnchorB.Origin.Should().Be(pair.ColliderB.Center);
        pair.Manifold[0].Depth.Should().BeGreaterThan(Fixed64.Zero);
    }

    [Theory]
    [InlineData(true)]
    [InlineData(false)]
    public void RotatedCuboidNearScalarFace_ShouldNotUseSaturatedCornerHull(
        bool positiveFace)
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        Vector3d center = positiveFace
            ? new Vector3d(
                Fixed64.MaxValue - Fixed64.One,
                Fixed64.Zero,
                Fixed64.Zero)
            : new Vector3d(
                Fixed64.MinValue + Fixed64.One,
                Fixed64.Zero,
                Fixed64.Zero);
        ScenarioBody<LSCuboidCollider> rotated = scenario.CreateBody(
            new LSCuboidCollider
            {
                Size = new Vector3d((Fixed64)4, Fixed64.One, Fixed64.One)
            },
            center,
            PhysicsScenarioBuilder.Yaw(45));
        Fixed64 probeSize = Fixed64.FromFraction(1, 100);
        Vector3d probeCenter = positiveFace
            ? new Vector3d(
                Fixed64.MaxValue - Fixed64.FromFraction(1, 200),
                Fixed64.Zero,
                -Fixed64.FromFraction(44, 25))
            : new Vector3d(
                Fixed64.MinValue + Fixed64.FromFraction(1, 200),
                Fixed64.Zero,
                Fixed64.FromFraction(44, 25));
        ScenarioBody<LSCuboidCollider> probe = scenario.CreateBody(
            new LSCuboidCollider
            {
                Size = Vector3d.One * probeSize
            },
            probeCenter,
            FixedQuaternion.Identity);
        CollisionPair pair = scenario.CreatePair(rotated.Collider, probe.Collider);

        rotated.Collider.Bounds.Intersects(probe.Collider.Bounds).Should().BeTrue();
        CollisionDetection.DoCollisionCheck(pair).Should().BeFalse();
        pair.Manifold.Count.Should().Be(0);
    }

    private static LSCollider CreateTarget(ColliderType targetType) =>
        targetType switch
        {
            ColliderType.Sphere => new LSSphereCollider
            {
                Radius = Fixed64.FromFraction(1, 100)
            },
            ColliderType.Capsule => new LSCapsuleCollider
            {
                Radius = Fixed64.FromFraction(1, 100),
                Size = new Vector3d(
                    Fixed64.FromFraction(1, 50),
                    Fixed64.FromFraction(1, 10),
                    Fixed64.FromFraction(1, 50))
            },
            ColliderType.Cylinder => new LSCylinderCollider
            {
                Radius = Fixed64.FromFraction(1, 100),
                Size = new Vector3d(
                    Fixed64.FromFraction(1, 50),
                    Fixed64.FromFraction(1, 10),
                    Fixed64.FromFraction(1, 50))
            },
            _ => new LSCuboidCollider
            {
                Size = Vector3d.One * Fixed64.FromFraction(1, 50)
            }
        };
}
