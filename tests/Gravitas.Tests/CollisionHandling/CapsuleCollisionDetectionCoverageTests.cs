using FixedMathSharp;
using FluentAssertions;
using Gravitas.Colliders;
using Gravitas.CollisionHandling;
using Gravitas.Tests.Support;
using Xunit;

namespace Gravitas.Tests.CollisionHandlingTests;

public sealed class CapsuleCollisionDetectionCoverageTests
{
    [Fact]
    public void CoincidentSphereLimitCapsules_ShouldUseDeterministicRightFallbackManifold()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSCapsuleCollider> first = scenario.CreateCapsule(Vector3d.Zero);
        ScenarioBody<LSCapsuleCollider> second = scenario.CreateCapsule(Vector3d.Zero);
        CollisionPair pair = scenario.CreatePair(first.Collider, second.Collider);

        first.Collider.LineSegmentStart.Should().Be(first.Collider.LineSegmentEnd);
        second.Collider.LineSegmentStart.Should().Be(second.Collider.LineSegmentEnd);
        pair.CollisionType.Should().Be(CollisionType.Capsule_Capsule);
        CollisionDetection.DoCollisionCheck(pair).Should().BeTrue();

        pair.Manifold.Count.Should().Be(1);
        ManifoldContact contact = pair.Manifold.PrimaryContact;
        contact.Depth.Should().Be(Fixed64.One);
        contact.Normal.Should().Be(Vector3d.Right);
        contact.PointA.Should().Be(Fixed64.Half * Vector3d.Right);
        contact.PointB.Should().Be(-Fixed64.Half * Vector3d.Right);
    }

    [Fact]
    public void SphereLimitCapsuleFirstAgainstTallCapsule_ShouldProjectOntoSecondCenterLine()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSCapsuleCollider> sphereLimit = scenario.CreateCapsule(
            Fixed64.FromFraction(3, 4) * Vector3d.Right);
        ScenarioBody<LSCapsuleCollider> tall = scenario.CreateBody(
            new LSCapsuleCollider
            {
                Size = new Vector3d(Fixed64.One, (Fixed64)3, Fixed64.One)
            },
            Vector3d.Zero,
            FixedQuaternion.Identity);
        CollisionPair forward = scenario.CreatePair(sphereLimit.Collider, tall.Collider);
        CollisionPair reversed = scenario.CreatePair(tall.Collider, sphereLimit.Collider);

        sphereLimit.Collider.LineSegmentStart.Should().Be(sphereLimit.Collider.LineSegmentEnd);
        tall.Collider.LineSegmentStart.Should().Be(-Vector3d.Up);
        tall.Collider.LineSegmentEnd.Should().Be(Vector3d.Up);
        forward.CollisionType.Should().Be(CollisionType.Capsule_Capsule);
        CollisionDetection.DoCollisionCheck(forward).Should().BeTrue();

        forward.Manifold.Count.Should().Be(1);
        ManifoldContact contact = forward.Manifold.PrimaryContact;
        contact.Depth.Should().Be(Fixed64.FromFraction(1, 4));
        contact.Normal.Should().Be(-Vector3d.Right);
        contact.PointA.Should().Be(Fixed64.FromFraction(1, 4) * Vector3d.Right);
        contact.PointB.Should().Be(Fixed64.Half * Vector3d.Right);

        CollisionDetection.DoCollisionCheck(reversed).Should().BeTrue();
        reversed.Manifold.Count.Should().Be(1);
        contact = reversed.Manifold.PrimaryContact;
        contact.Depth.Should().Be(Fixed64.FromFraction(1, 4));
        contact.Normal.Should().Be(Vector3d.Right);
        contact.PointA.Should().Be(Fixed64.Half * Vector3d.Right);
        contact.PointB.Should().Be(Fixed64.FromFraction(1, 4) * Vector3d.Right);
    }

    [Fact]
    public void EpsilonLengthCapsuleFirstAgainstTallCapsule_ShouldPreserveOrderInvariantManifold()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        Fixed64 segmentLength = FixedMath.Sqrt(Fixed64.Epsilon);
        Fixed64 halfSegmentLength = segmentLength * Fixed64.Half;
        FixedQuaternion rotateAxisOntoX = FixedQuaternion.FromEulerAnglesInDegrees(
            Fixed64.Zero,
            Fixed64.Zero,
            (Fixed64)(-90));
        ScenarioBody<LSCapsuleCollider> epsilonLength = scenario.CreateBody(
            new LSCapsuleCollider
            {
                Size = new Vector3d(Fixed64.One, Fixed64.One + segmentLength, Fixed64.One)
            },
            Vector3d.Zero,
            rotateAxisOntoX);
        ScenarioBody<LSCapsuleCollider> tall = scenario.CreateBody(
            new LSCapsuleCollider
            {
                Size = new Vector3d(Fixed64.One, (Fixed64)3, Fixed64.One)
            },
            halfSegmentLength * Vector3d.Right,
            FixedQuaternion.Identity);
        CollisionPair forward = scenario.CreatePair(epsilonLength.Collider, tall.Collider);
        CollisionPair reversed = scenario.CreatePair(tall.Collider, epsilonLength.Collider);
        Fixed64 expectedDepth = Fixed64.One - segmentLength;
        Vector3d expectedPositivePoint = (Fixed64.Half - halfSegmentLength) * Vector3d.Right;

        (epsilonLength.Collider.LineSegmentEnd - epsilonLength.Collider.LineSegmentStart)
            .MagnitudeSquared.Should().Be(Fixed64.Epsilon);
        epsilonLength.Collider.LineSegmentStart.Should().Be(-halfSegmentLength * Vector3d.Right);
        epsilonLength.Collider.LineSegmentEnd.Should().Be(halfSegmentLength * Vector3d.Right);

        CollisionDetection.DoCollisionCheck(forward).Should().BeTrue();
        CollisionDetection.DoCollisionCheck(reversed).Should().BeTrue();

        ManifoldContact forwardContact = forward.Manifold.PrimaryContact;
        forwardContact.Depth.Should().Be(expectedDepth);
        forwardContact.Normal.Should().Be(Vector3d.Right);
        forwardContact.PointA.Should().Be(expectedPositivePoint);
        forwardContact.PointB.Should().Be(-expectedPositivePoint);
        ManifoldContact reversedContact = reversed.Manifold.PrimaryContact;
        reversedContact.Depth.Should().Be(expectedDepth);
        reversedContact.Normal.Should().Be(-Vector3d.Right);
        reversedContact.PointA.Should().Be(-expectedPositivePoint);
        reversedContact.PointB.Should().Be(expectedPositivePoint);
    }
}
