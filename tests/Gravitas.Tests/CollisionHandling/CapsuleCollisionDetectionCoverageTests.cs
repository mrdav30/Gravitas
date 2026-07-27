using FixedMathSharp;
using FixedMathSharp.Geometry;
using FluentAssertions;
using Gravitas.Colliders;
using Gravitas.CollisionHandling;
using Gravitas.Tests.Support;
using Xunit;

namespace Gravitas.Tests.CollisionHandlingTests;

public sealed class CapsuleCollisionDetectionCoverageTests
{
    [Theory]
    [InlineData(-1, 0, 0)]
    [InlineData(0, 1, 0)]
    [InlineData(0, -1, 0)]
    [InlineData(0, 0, 1)]
    [InlineData(0, 0, -1)]
    public void CoincidentAxisCapsules_ShouldUseCenterLexicographicFallback(
        int x,
        int y,
        int z)
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSCapsuleCollider> first =
            scenario.CreateCapsule(Vector3d.Zero);
        var offset = new Vector3d((Fixed64)x, (Fixed64)y, (Fixed64)z)
            * Fixed64.FromFraction(3, 4);
        ScenarioBody<LSCapsuleCollider> second =
            scenario.CreateCapsule(offset);
        CollisionPair pair =
            scenario.CreatePair(first.Collider, second.Collider);

        CollisionDetection.DoCollisionCheck(pair).Should().BeTrue();
        pair.Manifold.HasContact.Should().BeTrue();
        pair.Manifold.PrimaryContact.Normal.Should().Be(
            new Vector3d((Fixed64)x, (Fixed64)y, (Fixed64)z));
    }

    [Fact]
    public void ScalarFaceCapsules_WhenWorldWitnessIsUnrepresentable_ShouldPreserveAuthoritativeContact()
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        LSCapsuleCollider first = CreateScalarFaceCollider(
            context,
            new LSCapsuleCollider(),
            new Vector3d(Fixed64.MaxValue, Fixed64.Zero, Fixed64.Zero),
            Fixed64.One);
        LSCapsuleCollider second = CreateScalarFaceCollider(
            context,
            new LSCapsuleCollider(),
            new Vector3d(Fixed64.MaxValue, Fixed64.Zero, Fixed64.Zero),
            Fixed64.One);
        var pair = new CollisionPair(first, second);

        FixedSegment.DoCenteredCapsulesOverlap(
            first.Center,
            first.WorldAxis,
            first.AxisLength,
            first.ScaledRadius,
            second.Center,
            second.WorldAxis,
            second.AxisLength,
            second.ScaledRadius).Should().BeTrue();
        FixedSegment.TryGetCenteredCapsulesContact(
            first.Center,
            first.Rotation,
            Vector3d.Up,
            first.AxisLength,
            first.ScaledRadius,
            second.Center,
            second.Rotation,
            Vector3d.Up,
            second.AxisLength,
            second.ScaledRadius,
            Vector3d.Right,
            out FixedContactAnchors relation).Should().BeTrue();
        relation.FirstAnchor.TryGetPoint(out _).Should().BeFalse();

        CollisionDetection.DoCollisionCheck(pair).Should().BeTrue();
        pair.Manifold.HasContact.Should().BeTrue();
        ManifoldContact contact = pair.Manifold.PrimaryContact;
        contact.AnchorA.Origin.Should().Be(first.Center);
        contact.AnchorB.Origin.Should().Be(second.Center);
        contact.AnchorA.TryGetOffsetFrom(
            contact.AnchorB,
            out Vector3d witnessSeparation).Should().BeTrue();
        witnessSeparation.Should().Be(Vector3d.Right * Fixed64.Two);
        contact.Normal.Should().Be(Vector3d.Right);
        contact.Depth.Should().Be(Fixed64.Two);
        contact.TryGetPointA(out _).Should().BeFalse();
        contact.TryGetPointB(out Vector3d pointB).Should().BeTrue();
        pointB.Should().Be(second.Center + Vector3d.Left);
    }

    [Fact]
    public void ScalarFaceCapsuleAndSphere_WhenWorldWitnessIsUnrepresentable_ShouldPreserveAuthoritativeContact()
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        LSCapsuleCollider capsule = CreateScalarFaceCollider(
            context,
            new LSCapsuleCollider(),
            new Vector3d(Fixed64.MaxValue, Fixed64.Zero, Fixed64.Zero),
            Fixed64.One);
        LSSphereCollider sphere = CreateScalarFaceCollider(
            context,
            new LSSphereCollider(),
            new Vector3d(Fixed64.MaxValue, Fixed64.Zero, Fixed64.Zero),
            Fixed64.One);
        var pair = new CollisionPair(capsule, sphere);

        pair.CollisionType.Should().Be(CollisionType.Capsule_Sphere);
        CollisionDetection.DoCollisionCheck(pair).Should().BeTrue();
        pair.Manifold.HasContact.Should().BeTrue();
        ManifoldContact contact = pair.Manifold.PrimaryContact;
        contact.AnchorA.Origin.Should().Be(capsule.Center);
        contact.AnchorB.Origin.Should().Be(sphere.Center);
        contact.TryGetPointA(out _).Should().BeFalse();
        contact.TryGetPointB(out _).Should().BeTrue();
    }

    private static TCollider CreateScalarFaceCollider<TCollider>(
        GravitasWorldContext context,
        TCollider collider,
        Vector3d center,
        Fixed64 radius)
        where TCollider : LSCollider
    {
        var transform = new FixedTransform(
            Vector3d.Zero,
            FixedQuaternion.Identity,
            Vector3d.One);
        collider.Radius = radius;
        if (collider is LSCapsuleCollider capsule)
        {
            capsule.Size = new Vector3d(
                capsule.Size.X,
                radius * Fixed64.Two,
                capsule.Size.Z);
        }
        collider.InitializeWithNoBody(new TestMatterAgent(context, transform));
        transform.TrySetWorldPosition(center).Should().BeTrue();
        collider.RebuildRuntimeShapeOnly(refreshMassProperties: false).Should().BeTrue();
        return collider;
    }

    [Fact]
    public void CoincidentSphereLimitCapsules_ShouldUseDeterministicRightFallbackManifold()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSCapsuleCollider> first = scenario.CreateCapsule(Vector3d.Zero);
        ScenarioBody<LSCapsuleCollider> second = scenario.CreateCapsule(Vector3d.Zero);
        CollisionPair pair = scenario.CreatePair(first.Collider, second.Collider);

        first.Collider.AxisLength.Should().Be(Fixed64.Zero);
        second.Collider.AxisLength.Should().Be(Fixed64.Zero);
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

        sphereLimit.Collider.AxisLength.Should().Be(Fixed64.Zero);
        FixedSegment.TryGetCenteredAxisEndpoint(
            tall.Collider.Center,
            tall.Collider.WorldAxis,
            tall.Collider.AxisLength,
            positive: false,
            out Vector3d tallStart).Should().BeTrue();
        FixedSegment.TryGetCenteredAxisEndpoint(
            tall.Collider.Center,
            tall.Collider.WorldAxis,
            tall.Collider.AxisLength,
            positive: true,
            out Vector3d tallEnd).Should().BeTrue();
        tallStart.Should().Be(-Vector3d.Up);
        tallEnd.Should().Be(Vector3d.Up);
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
        Vector3d expectedPositivePoint = (Fixed64.Half + halfSegmentLength) * Vector3d.Right;
        Vector3d expectedNegativePoint = (-Fixed64.Half + halfSegmentLength) * Vector3d.Right;

        epsilonLength.Collider.AxisLength.Should().Be(segmentLength);
        FixedSegment.TryGetCenteredAxisEndpoint(
            epsilonLength.Collider.Center,
            epsilonLength.Collider.WorldAxis,
            epsilonLength.Collider.AxisLength,
            positive: false,
            out Vector3d epsilonStart).Should().BeTrue();
        FixedSegment.TryGetCenteredAxisEndpoint(
            epsilonLength.Collider.Center,
            epsilonLength.Collider.WorldAxis,
            epsilonLength.Collider.AxisLength,
            positive: true,
            out Vector3d epsilonEnd).Should().BeTrue();
        epsilonStart.Should().Be(-halfSegmentLength * Vector3d.Right);
        epsilonEnd.Should().Be(halfSegmentLength * Vector3d.Right);

        CollisionDetection.DoCollisionCheck(forward).Should().BeTrue();
        CollisionDetection.DoCollisionCheck(reversed).Should().BeTrue();

        ManifoldContact forwardContact = forward.Manifold.PrimaryContact;
        forwardContact.Depth.Should().Be(Fixed64.One);
        forwardContact.Normal.Should().Be(Vector3d.Right);
        forwardContact.PointA.Should().Be(expectedPositivePoint);
        forwardContact.PointB.Should().Be(expectedNegativePoint);
        ManifoldContact reversedContact = reversed.Manifold.PrimaryContact;
        reversedContact.Depth.Should().Be(Fixed64.One);
        reversedContact.Normal.Should().Be(-Vector3d.Right);
        reversedContact.PointA.Should().Be(expectedNegativePoint);
        reversedContact.PointB.Should().Be(expectedPositivePoint);
    }
}
