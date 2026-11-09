using FixedMathSharp;
using FluentAssertions;
using Gravitas.Colliders;
using Gravitas.CollisionHandling;
using Gravitas.Tests.Support;
using Xunit;

namespace Gravitas.Tests.CollisionHandlingTests;

public sealed class CuboidCanonicalGeometryTests
{
    [Theory]
    [InlineData(16, 0, 1)]
    [InlineData(2, 1, 1)]
    [InlineData(16, 0, -1)]
    public void ShallowTiltedCuboidFace_ShouldCoverThePenetratingFaceInBothPairOrders(
        int supportWidth, int halfUnitOffset, int direction)
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        Fixed64 faceSign = (Fixed64)direction;
        Vector3d normal = Vector3d.Up * faceSign;
        var support = scenario.CreateBody(new LSCuboidCollider
        {
            Size = new Vector3d((Fixed64)supportWidth, Fixed64.One, (Fixed64)supportWidth)
        }, -normal * Fixed64.Half, FixedQuaternion.Identity);
        var tilted = scenario.CreateBody(new LSCuboidCollider
        {
            Size = new Vector3d(Fixed64.FromFraction(7, 4), Fixed64.One, Fixed64.FromFraction(7, 4))
        }, new Vector3d(Fixed64.FromFraction(halfUnitOffset, 2), Fixed64.FromFraction(49, 100) * faceSign, Fixed64.Zero),
            FixedQuaternion.FromEulerAnglesInDegrees(Fixed64.FromFraction(1, 16), Fixed64.Zero, Fixed64.FromFraction(1, 16)));
        CollisionPair forward = scenario.CreatePair(support.Collider, tilted.Collider);
        CollisionPair reverse = scenario.CreatePair(tilted.Collider, support.Collider);

        CollisionDetection.DoCollisionCheck(forward).Should().BeTrue();
        CollisionDetection.DoCollisionCheck(reverse).Should().BeTrue();
        forward.Manifold.Count.Should().Be(4);
        reverse.Manifold.Count.Should().Be(4);
        int clippedPoints = 0;
        for (int i = 0; i < forward.Manifold.Count; i++)
        {
            ManifoldContact contact = forward.Manifold[i];
            ManifoldContact reversed = reverse.Manifold[i];
            contact.Normal.Should().Be(normal);
            contact.PointA.Y.Should().Be(Fixed64.Zero);
            contact.PointA.X.Abs().Should().BeLessThanOrEqualTo(Fixed64.FromFraction(supportWidth, 2));
            contact.PointA.Z.Abs().Should().BeLessThanOrEqualTo(Fixed64.FromFraction(supportWidth, 2));
            contact.AnchorB.LocalPoint.Y.Should().Be(-faceSign * Fixed64.Half);
            (contact.PointB.Y * faceSign).Should().BeLessThan(Fixed64.Zero);
            contact.Depth.Should().Be(-contact.PointB.Y * faceSign);
            Vector3d.Distance(contact.PointA, contact.PointB + normal * contact.Depth)
                .Should().BeLessThanOrEqualTo(Fixed64.Epsilon);
            reversed.ContactId.Should().Be(contact.ContactId);
            reversed.PointA.Should().Be(contact.PointB);
            reversed.PointB.Should().Be(contact.PointA);
            reversed.Normal.Should().Be(-contact.Normal);
            if (contact.PointA.X == Fixed64.One)
                clippedPoints++;
        }
        clippedPoints.Should().Be(halfUnitOffset == 0 ? 0 : 2);
        AllocationTestHelper.MeasureSteadyState(() => CollisionDetection.DoCollisionCheck(forward)).Should().Be(0);
    }

    [Fact]
    public void RotatedCuboidFace_ShouldDiscardRoundedClipWitnessesOutsideTheReferenceFace()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        var support = scenario.CreateBody(new LSCuboidCollider { Size = new Vector3d(16, 1, 16) },
            -Vector3d.Up * Fixed64.Half, FixedQuaternion.Identity);

        // The X:W ratio of 1:3 gives the exact rotation cos=4/5, sin=3/5.
        // The two depth-plane intersections rematerialize at +3/5 of one raw
        // unit in Y. Only the two corners below the plane are valid witnesses.
        Fixed64 quaternionX = Fixed64.FromRaw(1_358_187_913);
        var rotation = new FixedQuaternion(quaternionX, Fixed64.Zero, Fixed64.Zero, quaternionX * (Fixed64)3);
        Fixed64 size = Fixed64.One + Fixed64.FromRaw(2);
        var tilted = scenario.CreateBody(new LSCuboidCollider { Size = new Vector3d(Fixed64.One, size, size) },
            Vector3d.Up * Fixed64.FromRaw(1_717_986_921), rotation);
        CollisionPair pair = scenario.CreatePair(support.Collider, tilted.Collider);

        CollisionDetection.DoCollisionCheck(pair).Should().BeTrue();
        pair.Manifold.Count.Should().Be(2);
        foreach (ManifoldContact contact in pair.Manifold)
        {
            contact.Normal.Should().Be(Vector3d.Up);
            contact.PointA.Y.Should().Be(Fixed64.Zero);
            contact.AnchorB.LocalPoint.Z.Should().Be(size * Fixed64.Half);
            contact.Depth.Should().Be(Fixed64.FromRaw(1_288_490_188));
        }
    }

    [Fact]
    public void RotatedCuboidFace_WhenRoundedClippingLosesExactTouch_ShouldRetainTheSingleWitness()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        var support = scenario.CreateBody(new LSCuboidCollider { Size = Vector3d.One },
            -Vector3d.Up * Fixed64.Half, FixedQuaternion.Identity);

        // The exact yaw (cos=4/5, sin=3/5) places the reference corner on
        // the incident edge. Rounding the corners before clipping loses it.
        Fixed64 quaternionY = Fixed64.FromRaw(1_358_187_913);
        var rotation = new FixedQuaternion(Fixed64.Zero, quaternionY, Fixed64.Zero, quaternionY * (Fixed64)3);
        var tilted = scenario.CreateBody(new LSCuboidCollider
        {
            Size = new Vector3d(Fixed64.One + Fixed64.FromRaw(2), Fixed64.One, Fixed64.One - Fixed64.FromRaw(4))
        }, new Vector3d(Fixed64.FromRaw(3_435_973_834), Fixed64.Half, Fixed64.FromRaw(3_865_470_566)), rotation);
        CollisionPair pair = scenario.CreatePair(support.Collider, tilted.Collider);

        CollisionDetection.DoCollisionCheck(pair).Should().BeTrue();
        pair.Manifold.Count.Should().Be(1);
        ManifoldContact contact = pair.Manifold.PrimaryContact;
        var corner = new Vector3d(Fixed64.Half, Fixed64.Zero, Fixed64.Half);
        contact.Normal.Should().Be(Vector3d.Up);
        contact.Depth.Should().Be(Fixed64.Zero);
        contact.PointA.Should().Be(corner);
        contact.PointB.Should().Be(corner);
    }

    [Fact]
    public void RotatedCompoundCuboidPart_ShouldRetainItsNamespacedSingleWitness()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        var compound = new LSCompoundCollider(CompoundColliderPart.Cuboid(
            Vector3d.One, Vector3d.Zero, PhysicsScenarioBuilder.Yaw(20), Vector3d.One));
        scenario.CreateBody(compound, Vector3d.Up * Fixed64.Quarter, FixedQuaternion.Identity);
        var support = scenario.CreateBody(new LSCuboidCollider { Size = new Vector3d(4, 1, 4) },
            -Vector3d.Up * Fixed64.Half, FixedQuaternion.Identity);
        CollisionPair pair = scenario.CreatePair(support.Collider, compound);
        var part = (LSCuboidCollider)compound.GetPartCollider(0);

        part.OrientedBox.TryGetContact(support.Collider.OrientedBox, out var single).Should().BeTrue();
        CollisionDetection.DoCollisionCheck(pair).Should().BeTrue();
        pair.ColliderA.Should().BeSameAs(compound);
        pair.Manifold.Count.Should().Be(1);
        ManifoldContact contact = pair.Manifold.PrimaryContact;
        contact.FeatureNamespaceA.Should().Be(1);
        contact.AnchorA.Should().Be(new ContactAnchor(single.FirstAnchor));
        contact.AnchorB.Should().Be(new ContactAnchor(single.SecondAnchor));
        contact.Depth.Should().Be(single.Depth);
        contact.Normal.Should().Be(single.Normal);
    }

    [Fact]
    public void ParallelRotatedCuboids_ShouldUseFiniteMatchedFaceAnchors()
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
        pair.Manifold.Count.Should().Be(4);
        Fixed64 quarter = Fixed64.FromFraction(1, 4);
        Fixed64 tolerance = Fixed64.FromRaw(4);
        foreach (ManifoldContact contact in pair.Manifold)
        {
            Vector3d.Distance(contact.Normal, normal).Should().BeLessThanOrEqualTo(tolerance);
            contact.AnchorA.LocalPoint.X.Should().Be(Fixed64.Half);
            contact.AnchorB.LocalPoint.X.Should().Be(-quarter);
            contact.AnchorB.LocalPoint.Y.Abs().Should().Be(quarter);
            contact.AnchorB.LocalPoint.Z.Abs().Should().Be(quarter);
            contact.AnchorA.TryGetOffsetFrom(contact.AnchorB, out Vector3d separation).Should().BeTrue();
            Vector3d.Distance(separation, contact.Normal * contact.Depth).Should().BeLessThanOrEqualTo(tolerance);
            (contact.Depth - Fixed64.FromFraction(3, 20)).Abs().Should().BeLessThanOrEqualTo(tolerance);
        }
    }

    [Fact]
    public void RotatedCuboids_WithEdgeAxisContact_ShouldRetainTheSingleWitness()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        var first = scenario.CreateBody(new LSCuboidCollider { Size = new Vector3d(4, 2, 2) },
            Vector3d.Zero, FixedQuaternion.FromEulerAnglesInDegrees((Fixed64)15, (Fixed64)25, (Fixed64)5));
        var second = scenario.CreateBody(new LSCuboidCollider { Size = new Vector3d(2, 4, 2) },
            new Vector3d(2, 0, 1), FixedQuaternion.FromEulerAnglesInDegrees((Fixed64)(-10), (Fixed64)40, (Fixed64)20));
        CollisionPair forward = scenario.CreatePair(first.Collider, second.Collider);
        CollisionPair reverse = scenario.CreatePair(second.Collider, first.Collider);

        CollisionDetection.DoCollisionCheck(forward).Should().BeTrue();
        CollisionDetection.DoCollisionCheck(reverse).Should().BeTrue();
        forward.Manifold.Count.Should().Be(1);
        reverse.Manifold.Count.Should().Be(1);
        ManifoldContact contact = forward.Manifold.PrimaryContact;
        ManifoldContact reversed = reverse.Manifold.PrimaryContact;
        first.Collider.OrientedBox.TryGetContact(second.Collider.OrientedBox, out var single).Should().BeTrue();
        contact.Depth.Should().Be(single.Depth);
        contact.AnchorA.Should().Be(new ContactAnchor(single.FirstAnchor));
        contact.AnchorB.Should().Be(new ContactAnchor(single.SecondAnchor));
        reversed.Depth.Should().Be(contact.Depth);
        reversed.AnchorA.Should().Be(contact.AnchorB);
        reversed.AnchorB.Should().Be(contact.AnchorA);
        reversed.Normal.Should().Be(-contact.Normal);
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
