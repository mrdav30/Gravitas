using FixedMathSharp;
using FixedMathSharp.Geometry;
using FluentAssertions;
using Gravitas.Colliders;
using Gravitas.CollisionHandling;
using Gravitas.Materials;
using Gravitas.Tests.Support;
using Xunit;

namespace Gravitas.Tests.Physics2D;

public sealed class CollisionDetection2DGapTests
{
    [Fact]
    public void ConvexPairSeparatedOnlyOnFirstPolygonAxis_ShouldRejectContact()
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        context.Settings.RuntimeMode = PhysicsRuntimeMode.TwoD;
        var diamond = new LSPolygonCollider2D(
            new Vector2d(Fixed64.Zero, -Fixed64.One),
            new Vector2d(Fixed64.One, Fixed64.Zero),
            new Vector2d(Fixed64.Zero, Fixed64.One),
            new Vector2d(-Fixed64.One, Fixed64.Zero));
        var square = new LSAABBoxCollider2D(new Vector2d((Fixed64)2, (Fixed64)2));
        _ = CreateBody(
            context,
            diamond,
            new Vector2d(Fixed64.FromFraction(17, 10), Fixed64.FromFraction(17, 10)));
        _ = CreateBody(context, square, Vector2d.Zero);

        CollisionDetection2D.BoundsOverlap(diamond, square).Should().BeTrue();

        bool collided = CollisionDetection2D.TryCollide(diamond, square, out Contact2D contact);

        collided.Should().BeFalse();
        contact.Should().Be(default(Contact2D));
    }

    [Theory]
    [InlineData(true)]
    [InlineData(false)]
    public void PolygonAgainstBox_ShouldBuildWindingIndependentOwnerOrderedFaceManifold(bool clockwise)
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        context.Settings.RuntimeMode = PhysicsRuntimeMode.TwoD;
        Vector2d[] vertices = clockwise
            ? new[]
            {
                new Vector2d(-Fixed64.One, -Fixed64.One),
                new Vector2d(-Fixed64.One, Fixed64.One),
                new Vector2d(Fixed64.One, Fixed64.One),
                new Vector2d(Fixed64.One, -Fixed64.One)
            }
            : new[]
            {
                new Vector2d(-Fixed64.One, -Fixed64.One),
                new Vector2d(Fixed64.One, -Fixed64.One),
                new Vector2d(Fixed64.One, Fixed64.One),
                new Vector2d(-Fixed64.One, Fixed64.One)
            };
        var polygon = new LSPolygonCollider2D(vertices);
        var box = new LSAABBoxCollider2D(new Vector2d((Fixed64)2, (Fixed64)2));
        _ = CreateBody(context, polygon, Vector2d.Zero);
        _ = CreateBody(context, box, new Vector2d(Fixed64.FromFraction(3, 2), Fixed64.Zero));
        var manifold = new ContactManifold2D();

        bool collided = CollisionDetection2D.TryCollide(
            new CollisionWorkItem2D(polygon, box, CollisionType2D.Convex_Convex),
            manifold,
            frame: 17);
        var reverse = new ContactManifold2D();
        bool reverseCollided = CollisionDetection2D.TryCollide(
            new CollisionWorkItem2D(box, polygon, CollisionType2D.Convex_Convex),
            reverse,
            frame: 17);

        collided.Should().BeTrue();
        reverseCollided.Should().BeTrue();
        manifold.Count.Should().Be(2);
        reverse.Count.Should().Be(2);
        manifold[0].Normal.Should().Be(Vector2d.Right);
        manifold[1].Normal.Should().Be(Vector2d.Right);
        manifold[0].Depth.Should().Be(Fixed64.Half);
        manifold[1].Depth.Should().Be(Fixed64.Half);
        manifold[0].PointA.X.Should().Be(Fixed64.One);
        manifold[1].PointA.X.Should().Be(Fixed64.One);
        manifold[0].PointB.X.Should().Be(Fixed64.Half);
        manifold[1].PointB.X.Should().Be(Fixed64.Half);
        new[] { manifold[0].PointA.Y, manifold[1].PointA.Y }
            .Should().BeEquivalentTo(new[] { Fixed64.One, -Fixed64.One });
        new[] { manifold[0].PointB.Y, manifold[1].PointB.Y }
            .Should().BeEquivalentTo(new[] { Fixed64.One, -Fixed64.One });
        for (int i = 0; i < manifold.Count; i++)
        {
            reverse[i].ContactId.Should().Be(manifold[i].ContactId);
            reverse[i].Normal.Should().Be(-manifold[i].Normal);
            reverse[i].PointA.Should().Be(manifold[i].PointB);
            reverse[i].PointB.Should().Be(manifold[i].PointA);
        }
    }

    [Theory]
    [InlineData(true)]
    [InlineData(false)]
    public void CapsuleEndCapAgainstBox_ShouldUseSingleOwnerOrderedContact(bool capsuleFirst)
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        context.Settings.RuntimeMode = PhysicsRuntimeMode.TwoD;
        var capsule = new LSCapsuleCollider2D(Fixed64.Half, (Fixed64)3)
        {
            Material = PhysicsMaterial.Bouncy
        };
        var box = new LSAABBoxCollider2D(Vector2d.One)
        {
            Material = PhysicsMaterial.Frictionless
        };
        _ = CreateBody(context, capsule, Vector2d.Zero);
        _ = CreateBody(context, box, new Vector2d(Fixed64.Zero, Fixed64.FromFraction(5, 4)));
        LSCollider2D colliderA = capsuleFirst ? capsule : box;
        LSCollider2D colliderB = capsuleFirst ? box : capsule;
        var manifold = new ContactManifold2D();

        bool collided = CollisionDetection2D.TryCollide(
            new CollisionWorkItem2D(
                colliderA,
                colliderB,
                ColliderSettings2D.GetCollisionType(colliderA.Shape, colliderB.Shape)),
            manifold,
            frame: 19);

        collided.Should().BeTrue();
        manifold.Count.Should().Be(1);
        ManifoldContact2D contact = manifold[0];
        contact.Depth.Should().Be(Fixed64.FromFraction(3, 4));
        contact.Normal.Should().Be(capsuleFirst ? Vector2d.Forward : -Vector2d.Forward);
        contact.PointA.Should().Be(capsuleFirst
            ? new Vector2d(Fixed64.Zero, Fixed64.FromFraction(3, 2))
            : new Vector2d(Fixed64.Zero, Fixed64.FromFraction(3, 4)));
        contact.PointB.Should().Be(capsuleFirst
            ? new Vector2d(Fixed64.Zero, Fixed64.FromFraction(3, 4))
            : new Vector2d(Fixed64.Zero, Fixed64.FromFraction(3, 2)));
        contact.MaterialA.Should().Be(capsuleFirst ? PhysicsMaterial.Bouncy : PhysicsMaterial.Frictionless);
        contact.MaterialB.Should().Be(capsuleFirst ? PhysicsMaterial.Frictionless : PhysicsMaterial.Bouncy);
    }

    [Theory]
    [InlineData(1)]
    [InlineData(-1)]
    public void RotatedContainedPolygon_ShouldUseExactMinimumAxisManifold(int verticalSign)
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        context.Settings.RuntimeMode = PhysicsRuntimeMode.TwoD;
        var outer = new LSAABBoxCollider2D(new Vector2d((Fixed64)2, (Fixed64)2));
        var inner = new LSPolygonCollider2D(
            new Vector2d(-Fixed64.Half, -Fixed64.Half),
            new Vector2d(Fixed64.Half, -Fixed64.Half),
            new Vector2d(Fixed64.Half, Fixed64.Half),
            new Vector2d(-Fixed64.Half, Fixed64.Half));
        Fixed64 sign = (Fixed64)verticalSign;
        _ = CreateBody(context, outer, Vector2d.Zero);
        _ = CreateBody(
            context,
            inner,
            new Vector2d(-Fixed64.Half, sign * Fixed64.Half),
            FixedMath.DegToRad((Fixed64)45));
        var manifold = new ContactManifold2D();

        bool direct = CollisionDetection2D.TryCollide(outer, inner, out Contact2D contact);
        bool collided = CollisionDetection2D.TryCollide(
            new CollisionWorkItem2D(outer, inner, CollisionType2D.Convex_Convex),
            manifold,
            frame: 23);

        Vector2d expectedNormal = sign * Vector2d.Forward;
        direct.Should().BeTrue();
        collided.Should().BeTrue();
        contact.Normal.Should().Be(expectedNormal);
        contact.Depth.Should().BeGreaterThan(Fixed64.Zero);
        contact.PointA.Should().Be(outer.GetSupportPoint(expectedNormal));
        contact.PointB.Should().Be(inner.GetSupportPoint(-expectedNormal));
        manifold.Count.Should().Be(1);
        Fixed64 reconstructedDepthTolerance = Fixed64.FromRaw(4);
        for (int i = 0; i < manifold.Count; i++)
        {
            ManifoldContact2D manifoldContact = manifold[i];
            manifoldContact.Normal.Should().Be(expectedNormal);
            manifoldContact.Depth.Should().BeGreaterThan(Fixed64.Zero);
            manifoldContact.Depth.Should().BeLessThanOrEqualTo(contact.Depth);
            outer.ContainsPoint(manifoldContact.PointA).Should().BeTrue();
            inner.ContainsPoint(manifoldContact.PointB).Should().BeTrue();
            Fixed64 reconstructedDepth = Vector2d.Dot(
                manifoldContact.PointA - manifoldContact.PointB,
                manifoldContact.Normal);
            (reconstructedDepth - manifoldContact.Depth)
                .Abs()
                .Should().BeLessThanOrEqualTo(reconstructedDepthTolerance);
        }
    }

    [Fact]
    public void UltraThinPositiveBoxes_ShouldPreserveMinimumRepresentablePenetration()
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        context.Settings.RuntimeMode = PhysicsRuntimeMode.TwoD;
        var first = new LSAABBoxCollider2D(new Vector2d(Fixed64.Epsilon, (Fixed64)2));
        var second = new LSAABBoxCollider2D(new Vector2d(Fixed64.Epsilon, (Fixed64)2));
        _ = CreateBody(context, first, Vector2d.Zero);
        _ = CreateBody(context, second, Vector2d.Zero);

        bool collided = CollisionDetection2D.TryCollide(first, second, out Contact2D contact);

        collided.Should().BeTrue();
        contact.Depth.Should().Be(Fixed64.Epsilon);
        contact.Normal.Should().Be(-Vector2d.Right);
        Fixed64 halfWidth = Fixed64.Epsilon * Fixed64.Half;
        contact.PointA.Should().Be(new Vector2d(-halfWidth, Fixed64.One));
        contact.PointB.Should().Be(new Vector2d(halfWidth, Fixed64.One));
    }

    [Fact]
    public void OffOriginContainedPolygon_ShouldUseShortestSignedAxisForBothOwners()
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        context.Settings.RuntimeMode = PhysicsRuntimeMode.TwoD;
        var polygon = new LSPolygonCollider2D(
            new Vector2d((Fixed64)10, -Fixed64.One),
            new Vector2d((Fixed64)12, -Fixed64.One),
            new Vector2d((Fixed64)12, Fixed64.One),
            new Vector2d((Fixed64)10, Fixed64.One));
        var box = new LSAABBoxCollider2D(new Vector2d(Fixed64.FromFraction(7, 2), (Fixed64)4));
        _ = CreateBody(context, polygon, Vector2d.Zero);
        _ = CreateBody(context, box, new Vector2d(Fixed64.FromFraction(43, 4), Fixed64.Zero));

        bool direct = CollisionDetection2D.TryCollide(polygon, box, out Contact2D contact);
        ContactManifold2D forward = BuildManifold(polygon, box, frame: 29, out bool forwardHit);
        ContactManifold2D reverse = BuildManifold(box, polygon, frame: 29, out bool reverseHit);

        direct.Should().BeTrue();
        contact.Normal.Should().Be(-Vector2d.Right);
        contact.Depth.Should().Be(Fixed64.FromFraction(5, 2));
        contact.PointA.Should().Be(new Vector2d((Fixed64)10, -Fixed64.One));
        contact.PointB.Should().Be(new Vector2d(Fixed64.FromFraction(25, 2), (Fixed64)2));
        forwardHit.Should().BeTrue();
        reverseHit.Should().BeTrue();
        forward.Count.Should().Be(2);
        reverse.Count.Should().Be(2);
        forward[0].ContactId.Should().BeLessThan(forward[1].ContactId);
        Fixed64 clippedBottom = -Fixed64.One;
        for (int i = 0; i < forward.Count; i++)
        {
            forward[i].Normal.Should().Be(-Vector2d.Right);
            forward[i].Depth.Should().Be(Fixed64.FromFraction(5, 2));
        }
        new[] { forward[0].PointA, forward[1].PointA }
            .Should().BeEquivalentTo(new[]
            {
                new Vector2d((Fixed64)10, Fixed64.One),
                new Vector2d((Fixed64)10, clippedBottom)
            });
        new[] { forward[0].PointB, forward[1].PointB }
            .Should().BeEquivalentTo(new[]
            {
                new Vector2d(Fixed64.FromFraction(25, 2), Fixed64.One),
                new Vector2d(Fixed64.FromFraction(25, 2), clippedBottom)
            });
        reverse[0].ContactId.Should().BeLessThan(reverse[1].ContactId);
        for (int i = 0; i < reverse.Count; i++)
        {
            reverse[i].Normal.Should().Be(Vector2d.Right);
            reverse[i].Depth.Should().Be(Fixed64.FromFraction(5, 2));
        }
        new[] { reverse[0].PointA, reverse[1].PointA }
            .Should().BeEquivalentTo(new[]
            {
                new Vector2d(Fixed64.FromFraction(25, 2), -Fixed64.One),
                new Vector2d(Fixed64.FromFraction(25, 2), Fixed64.One)
            });
        new[] { reverse[0].PointB, reverse[1].PointB }
            .Should().BeEquivalentTo(new[]
            {
                new Vector2d((Fixed64)10, -Fixed64.One),
                new Vector2d((Fixed64)10, Fixed64.One)
            });
    }

    [Fact]
    public void OffOriginSymmetricContainmentTie_ShouldKeepAuthoredAxis()
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        context.Settings.RuntimeMode = PhysicsRuntimeMode.TwoD;
        var polygon = new LSPolygonCollider2D(
            new Vector2d((Fixed64)10, -Fixed64.One),
            new Vector2d((Fixed64)12, -Fixed64.One),
            new Vector2d((Fixed64)12, Fixed64.One),
            new Vector2d((Fixed64)10, Fixed64.One));
        var box = new LSAABBoxCollider2D(new Vector2d((Fixed64)4, (Fixed64)6));
        _ = CreateBody(context, polygon, Vector2d.Zero);
        _ = CreateBody(context, box, new Vector2d((Fixed64)11, Fixed64.Zero));

        bool collided = CollisionDetection2D.TryCollide(polygon, box, out Contact2D contact);

        collided.Should().BeTrue();
        contact.Normal.Should().Be(-Vector2d.Right);
        contact.Depth.Should().Be((Fixed64)3);
        contact.PointA.Should().Be(new Vector2d((Fixed64)10, -Fixed64.One));
        contact.PointB.Should().Be(new Vector2d((Fixed64)13, (Fixed64)3));
    }

    [Theory]
    [InlineData(-1)]
    [InlineData(1)]
    public void OffsetIntersectingCapsules_ShouldOrientCoincidentFeatureFallback(int offset)
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        context.Settings.RuntimeMode = PhysicsRuntimeMode.TwoD;
        var horizontal = new LSCapsuleCollider2D(Fixed64.Half, (Fixed64)4);
        var vertical = new LSCapsuleCollider2D(Fixed64.Half, (Fixed64)4);
        _ = CreateBody(context, horizontal, Vector2d.Zero, FixedMath.DegToRad((Fixed64)90));
        _ = CreateBody(context, vertical, (Fixed64)offset * Vector2d.Right);

        bool collided = CollisionDetection2D.TryCollide(horizontal, vertical, out Contact2D contact);

        collided.Should().BeTrue();
        Vector2d normal = (Fixed64)offset * Vector2d.Right;
        contact.Normal.Should().Be(normal);
        contact.Depth.Should().Be(Fixed64.One);
        contact.PointA.Should().Be((Fixed64)offset * Fixed64.FromFraction(3, 2) * Vector2d.Right);
        contact.PointB.Should().Be((Fixed64)offset * Fixed64.Half * Vector2d.Right);
    }

    [Theory]
    [InlineData(false, false)]
    [InlineData(false, true)]
    [InlineData(true, false)]
    [InlineData(true, true)]
    public void RotatedCapsuleAtScalarFace_ShouldCollideAlongConceptualAxis(
        bool positiveFace,
        bool targetCapsule)
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        context.Settings.RuntimeMode = PhysicsRuntimeMode.TwoD;
        Fixed64 radius = Fixed64.FromFraction(1, 10);
        Fixed64 centerX = positiveFace
            ? Fixed64.MaxValue - (Fixed64)5
            : Fixed64.MinValue + (Fixed64)5;
        var capsule = new LSCapsuleCollider2D(radius, (Fixed64)20 + radius * Fixed64.Two);
        _ = CreateBody(
            context,
            capsule,
            new Vector2d(centerX, Fixed64.Zero),
            FixedMath.DegToRad((Fixed64)45));
        Fixed64 axialDistance = positiveFace ? (Fixed64)6 : (Fixed64)(-6);
        Fixed64.TryMultiplyAdd(capsule.WorldAxis.X, axialDistance, centerX, out Fixed64 circleX)
            .Should()
            .BeTrue();
        Fixed64 circleY = capsule.WorldAxis.Y * axialDistance;
        LSCollider2D target = targetCapsule
            ? new LSCapsuleCollider2D(radius, radius * Fixed64.Two)
            : new LSCircleCollider2D(radius);
        _ = CreateBody(context, target, new Vector2d(circleX, circleY));

        bool negativeEndpointRepresentable = FixedSegment2d.TryGetCenteredAxisEndpoint(
            capsule.Center,
            capsule.WorldAxis,
            capsule.AxisLength,
            positive: false,
            out _);
        bool positiveEndpointRepresentable = FixedSegment2d.TryGetCenteredAxisEndpoint(
            capsule.Center,
            capsule.WorldAxis,
            capsule.AxisLength,
            positive: true,
            out _);
        Vector2d tieSupport = capsule.GetSupportPoint(capsule.WorldAxis.RightHandNormal);
        bool collided = CollisionDetection2D.TryCollide(capsule, target, out Contact2D contact);

        (negativeEndpointRepresentable ^ positiveEndpointRepresentable).Should().BeTrue();
        capsule.Bounds.Contains(tieSupport).Should().BeTrue();
        collided.Should().BeTrue();
        contact.Depth.Should().BeGreaterThan(radius);
        contact.Normal.IsNormalized().Should().BeTrue();
        Vector2d.Dot(contact.Normal, capsule.WorldAxis).Abs()
            .Should()
            .BeLessThanOrEqualTo(Fixed64.Epsilon * (Fixed64)4);
    }

    [Theory]
    [InlineData(false, false)]
    [InlineData(false, true)]
    [InlineData(true, false)]
    [InlineData(true, true)]
    public void CapsuleContactAtScalarFace_ShouldNotRequireAnAbsoluteWorldWitness(
        bool targetCapsule,
        bool reverseOrder)
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        context.Settings.RuntimeMode = PhysicsRuntimeMode.TwoD;
        Fixed64 capsuleX = Fixed64.MaxValue - Fixed64.FromFraction(1, 4);
        var capsule = new LSCapsuleCollider2D(Fixed64.One, (Fixed64)4);
        LSCollider2D target = targetCapsule
            ? new LSCapsuleCollider2D(Fixed64.Half, Fixed64.One)
            : new LSCircleCollider2D(Fixed64.Half);
        _ = CreateBody(context, capsule, new Vector2d(capsuleX, Fixed64.Zero));
        _ = CreateBody(context, target, new Vector2d(Fixed64.MaxValue, Fixed64.Zero));
        LSCollider2D colliderA = reverseOrder ? target : capsule;
        LSCollider2D colliderB = reverseOrder ? capsule : target;

        bool collided = CollisionDetection2D.TryCollide(colliderA, colliderB, out Contact2D contact);

        collided.Should().BeTrue();
        contact.Depth.Should().Be(Fixed64.FromFraction(5, 4));
        contact.Normal.Should().Be(reverseOrder ? Vector2d.Left : Vector2d.Right);
        ContactAnchor2D capsuleAnchor = reverseOrder ? contact.AnchorB : contact.AnchorA;
        ContactAnchor2D targetAnchor = reverseOrder ? contact.AnchorA : contact.AnchorB;
        capsuleAnchor.Origin.Should().Be(capsule.Center);
        capsuleAnchor.Offset.Should().Be(Vector2d.Right);
        capsuleAnchor.TryGetWorldPoint(out _).Should().BeFalse();
        targetAnchor.Origin.Should().Be(target.Center);
        targetAnchor.Offset.Should().Be(Vector2d.Left * Fixed64.Half);
        targetAnchor.TryGetWorldPoint(out _).Should().BeTrue();
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void CapsuleContactIdentity_ShouldBeStableUnderRigidTranslation(bool targetCapsule)
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        context.Settings.RuntimeMode = PhysicsRuntimeMode.TwoD;
        var firstCapsule = new LSCapsuleCollider2D(Fixed64.One, (Fixed64)4);
        LSCollider2D firstTarget = targetCapsule
            ? new LSCapsuleCollider2D(Fixed64.Half, Fixed64.One)
            : new LSCircleCollider2D(Fixed64.Half);
        var translatedCapsule = new LSCapsuleCollider2D(Fixed64.One, (Fixed64)4);
        LSCollider2D translatedTarget = targetCapsule
            ? new LSCapsuleCollider2D(Fixed64.Half, Fixed64.One)
            : new LSCircleCollider2D(Fixed64.Half);
        _ = CreateBody(context, firstCapsule, Vector2d.Zero);
        _ = CreateBody(context, firstTarget, Vector2d.Right * Fixed64.FromFraction(1, 4));
        Vector2d translation = new((Fixed64)1000, (Fixed64)(-2000));
        _ = CreateBody(context, translatedCapsule, translation);
        _ = CreateBody(
            context,
            translatedTarget,
            translation + Vector2d.Right * Fixed64.FromFraction(1, 4));

        ContactManifold2D first = BuildManifold(firstCapsule, firstTarget, frame: 31, out bool firstHit);
        ContactManifold2D translated = BuildManifold(
            translatedCapsule,
            translatedTarget,
            frame: 31,
            out bool translatedHit);

        firstHit.Should().BeTrue();
        translatedHit.Should().BeTrue();
        first.PrimaryContact.ContactId.Should().Be(translated.PrimaryContact.ContactId);
        first.PrimaryContact.AnchorA.Offset.Should().Be(translated.PrimaryContact.AnchorA.Offset);
        first.PrimaryContact.AnchorB.Offset.Should().Be(translated.PrimaryContact.AnchorB.Offset);
        first.PrimaryContact.AnchorA.Origin.Should().Be(firstCapsule.Center);
        translated.PrimaryContact.AnchorA.Origin.Should().Be(translatedCapsule.Center);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void CapsuleContactIdentity_ShouldBeStableUnderCommonRigidRotation(
        bool targetCapsule)
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        context.Settings.RuntimeMode = PhysicsRuntimeMode.TwoD;
        var firstCapsule = new LSCapsuleCollider2D(Fixed64.One, (Fixed64)4);
        LSCollider2D firstTarget = targetCapsule
            ? new LSCapsuleCollider2D(Fixed64.Half, Fixed64.One)
            : new LSCircleCollider2D(Fixed64.Half);
        var rotatedCapsule = new LSCapsuleCollider2D(Fixed64.One, (Fixed64)4);
        LSCollider2D rotatedTarget = targetCapsule
            ? new LSCapsuleCollider2D(Fixed64.Half, Fixed64.One)
            : new LSCircleCollider2D(Fixed64.Half);
        Vector2d targetOffset =
            Vector2d.Right * Fixed64.FromFraction(1, 4);
        _ = CreateBody(context, firstCapsule, Vector2d.Zero);
        _ = CreateBody(context, firstTarget, targetOffset);
        Fixed64 rotation = Fixed64.HalfPi;
        _ = CreateBody(
            context,
            rotatedCapsule,
            Vector2d.Zero,
            rotation);
        _ = CreateBody(
            context,
            rotatedTarget,
            Vector2d.Rotate(targetOffset, rotation),
            rotation);

        ContactManifold2D first =
            BuildManifold(firstCapsule, firstTarget, frame: 32, out bool firstHit);
        ContactManifold2D rotated = BuildManifold(
            rotatedCapsule,
            rotatedTarget,
            frame: 32,
            out bool rotatedHit);

        firstHit.Should().BeTrue();
        rotatedHit.Should().BeTrue();
        rotated.PrimaryContact.ContactId.Should().Be(first.PrimaryContact.ContactId);
        rotated.PrimaryContact.AnchorA.LocalPoint
            .Should()
            .Be(first.PrimaryContact.AnchorA.LocalPoint);
        rotated.PrimaryContact.AnchorA.LocalDisplacement
            .Should()
            .Be(first.PrimaryContact.AnchorA.LocalDisplacement);
        rotated.PrimaryContact.AnchorB.LocalPoint
            .Should()
            .Be(first.PrimaryContact.AnchorB.LocalPoint);
        rotated.PrimaryContact.AnchorB.LocalDisplacement
            .Should()
            .Be(first.PrimaryContact.AnchorB.LocalDisplacement);
    }

    [Fact]
    public void CapsuleSideManifold_ShouldPreserveOwnerOrderAndIdentityWhenReversed()
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        context.Settings.RuntimeMode = PhysicsRuntimeMode.TwoD;
        var capsule = new LSCapsuleCollider2D(Fixed64.One, (Fixed64)4)
        {
            Material = PhysicsMaterial.Bouncy,
        };
        var box = new LSAABBoxCollider2D(new Vector2d(Fixed64.One, (Fixed64)6))
        {
            Material = PhysicsMaterial.Frictionless,
        };
        _ = CreateBody(context, capsule, Vector2d.Zero);
        _ = CreateBody(context, box, Vector2d.Right * Fixed64.FromFraction(1, 4));

        ContactManifold2D forward = BuildManifold(capsule, box, frame: 37, out bool forwardHit);
        ContactManifold2D reversed = BuildManifold(box, capsule, frame: 37, out bool reversedHit);

        forwardHit.Should().BeTrue();
        reversedHit.Should().BeTrue();
        forward.Count.Should().Be(2);
        reversed.Count.Should().Be(2);
        for (int i = 0; i < forward.Count; i++)
        {
            ManifoldContact2D direct = forward[i];
            ManifoldContact2D reverse = reversed[i];
            direct.ContactId.Should().Be(reverse.ContactId);
            direct.AnchorA.Origin.Should().Be(capsule.Center);
            direct.AnchorB.Origin.Should().Be(box.Center);
            reverse.AnchorA.Origin.Should().Be(box.Center);
            reverse.AnchorB.Origin.Should().Be(capsule.Center);
            direct.AnchorA.Offset.Should().Be(reverse.AnchorB.Offset);
            direct.AnchorB.Offset.Should().Be(reverse.AnchorA.Offset);
            direct.Normal.Should().Be(Vector2d.Right);
            reverse.Normal.Should().Be(Vector2d.Left);
            direct.MaterialA.Should().Be(PhysicsMaterial.Bouncy);
            direct.MaterialB.Should().Be(PhysicsMaterial.Frictionless);
            reverse.MaterialA.Should().Be(PhysicsMaterial.Frictionless);
            reverse.MaterialB.Should().Be(PhysicsMaterial.Bouncy);
            direct.TryGetPointA(out _).Should().BeTrue();
            direct.TryGetPointB(out _).Should().BeTrue();
            reverse.TryGetPointA(out _).Should().BeTrue();
            reverse.TryGetPointB(out _).Should().BeTrue();
        }
    }

    [Fact]
    public void CapsuleSideManifoldAtScalarFace_ShouldMatchTranslatedRelativeAnchors()
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        context.Settings.RuntimeMode = PhysicsRuntimeMode.TwoD;
        Fixed64 boxWidth = Fixed64.FromFraction(1, 10);
        var baselineCapsule = new LSCapsuleCollider2D(Fixed64.One, (Fixed64)4);
        var baselineBox = new LSAABBoxCollider2D(new Vector2d(boxWidth, (Fixed64)6));
        var scalarCapsule = new LSCapsuleCollider2D(Fixed64.One, (Fixed64)4);
        var scalarBox = new LSAABBoxCollider2D(new Vector2d(boxWidth, (Fixed64)6));
        _ = CreateBody(context, baselineCapsule, Vector2d.Zero);
        _ = CreateBody(context, baselineBox, Vector2d.Right * Fixed64.FromFraction(1, 5));
        _ = CreateBody(
            context,
            scalarCapsule,
            new Vector2d(Fixed64.MaxValue - Fixed64.FromFraction(1, 4), Fixed64.Zero));
        _ = CreateBody(
            context,
            scalarBox,
            new Vector2d(Fixed64.MaxValue - Fixed64.FromFraction(1, 20), Fixed64.Zero));

        ContactManifold2D baseline = BuildManifold(
            baselineCapsule,
            baselineBox,
            frame: 39,
            out bool baselineHit);
        ContactManifold2D scalar = BuildManifold(
            scalarCapsule,
            scalarBox,
            frame: 39,
            out bool scalarHit);

        baselineHit.Should().BeTrue();
        scalarHit.Should().BeTrue();
        baseline.Count.Should().Be(2);
        scalar.Count.Should().Be(2);
        for (int i = 0; i < scalar.Count; i++)
        {
            scalar[i].ContactId.Should().Be(baseline[i].ContactId);
            scalar[i].AnchorA.Offset.Should().Be(baseline[i].AnchorA.Offset);
            scalar[i].AnchorB.Offset.Should().Be(baseline[i].AnchorB.Offset);
            scalar[i].AnchorA.Origin.Should().Be(scalarCapsule.Center);
            scalar[i].AnchorB.Origin.Should().Be(scalarBox.Center);
            scalar[i].TryGetPointA(out _).Should().BeFalse();
            scalar[i].TryGetPointB(out _).Should().BeTrue();
        }
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void CompoundCapsuleContact_ShouldRetainExactPartFrame(bool compoundIsA)
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        context.Settings.RuntimeMode = PhysicsRuntimeMode.TwoD;
        var compound = new LSCompoundCollider2D(
            CompoundColliderPart2D.Capsule(
                Fixed64.One,
                (Fixed64)4,
                Vector2d.Right * (Fixed64)2));
        var circle = new LSCircleCollider2D(Fixed64.Half);
        _ = CreateBody(context, compound, Vector2d.Right * (Fixed64)10);
        _ = CreateBody(context, circle, Vector2d.Right * Fixed64.FromFraction(49, 4));
        LSCollider2D colliderA = compoundIsA ? compound : circle;
        LSCollider2D colliderB = compoundIsA ? circle : compound;

        bool collided = CollisionDetection2D.TryCollide(colliderA, colliderB, out Contact2D contact);
        ContactManifold2D manifold = BuildManifold(colliderA, colliderB, frame: 41, out bool manifoldHit);

        collided.Should().BeTrue();
        manifoldHit.Should().BeTrue();
        ContactAnchor2D directCompoundAnchor = compoundIsA ? contact.AnchorA : contact.AnchorB;
        ContactAnchor2D manifoldCompoundAnchor = compoundIsA
            ? manifold.PrimaryContact.AnchorA
            : manifold.PrimaryContact.AnchorB;
        Vector2d partCenter = Vector2d.Right * (Fixed64)12;
        directCompoundAnchor.Origin.Should().Be(partCenter);
        directCompoundAnchor.Offset.Should().Be(Vector2d.Right);
        manifoldCompoundAnchor.Origin.Should().Be(partCenter);
        manifoldCompoundAnchor.Offset.Should().Be(directCompoundAnchor.Offset);
        manifoldCompoundAnchor.GetLocalFeatureHash64()
            .Should()
            .Be(directCompoundAnchor.GetLocalFeatureHash64());
    }

    [Fact]
    public void CompoundPairManifold_ShouldRetainBothExactPartFrames()
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        context.Settings.RuntimeMode = PhysicsRuntimeMode.TwoD;
        var first = new LSCompoundCollider2D(
            CompoundColliderPart2D.Circle(Fixed64.One, Vector2d.Right * (Fixed64)2));
        var second = new LSCompoundCollider2D(
            CompoundColliderPart2D.Circle(Fixed64.One, Vector2d.Left * (Fixed64)2));
        _ = CreateBody(context, first, Vector2d.Right * (Fixed64)10);
        _ = CreateBody(context, second, Vector2d.Right * (Fixed64)15);

        ContactManifold2D manifold = BuildManifold(first, second, frame: 43, out bool collided);

        collided.Should().BeTrue();
        manifold.Count.Should().Be(1);
        manifold.PrimaryContact.AnchorA.Origin.Should().Be(Vector2d.Right * (Fixed64)12);
        manifold.PrimaryContact.AnchorA.Offset.Should().Be(Vector2d.Right);
        manifold.PrimaryContact.AnchorB.Origin.Should().Be(Vector2d.Right * (Fixed64)13);
        manifold.PrimaryContact.AnchorB.Offset.Should().Be(Vector2d.Left);
    }

    [Fact]
    public void RotatingCompoundManifold_ShouldKeepOwnerLocalContactIdentity()
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        context.Settings.RuntimeMode = PhysicsRuntimeMode.TwoD;
        Vector2d[] square =
        {
            new(-Fixed64.One, -Fixed64.One),
            new(Fixed64.One, -Fixed64.One),
            new(Fixed64.One, Fixed64.One),
            new(-Fixed64.One, Fixed64.One),
        };
        var compound = new LSCompoundCollider2D(
            CompoundColliderPart2D.ConvexPolygon(
                square,
                Vector2d.Right * Fixed64.Two));
        var target = new LSPolygonCollider2D(square);
        SolidBody2D compoundBody = CreateBody(
            context,
            compound,
            Vector2d.Zero);
        SolidBody2D targetBody = CreateBody(
            context,
            target,
            Vector2d.Right * Fixed64.FromFraction(7, 2));

        ContactManifold2D baseline = BuildManifold(
            compound,
            target,
            frame: 47,
            out bool baselineHit);

        compoundBody.SetRotation(Fixed64.HalfPi);
        targetBody.SetPosition(
            Vector2d.Forward * Fixed64.FromFraction(7, 2));
        targetBody.SetRotation(Fixed64.HalfPi);
        ContactManifold2D rotated = BuildManifold(
            compound,
            target,
            frame: 48,
            out bool rotatedHit);

        baselineHit.Should().BeTrue();
        rotatedHit.Should().BeTrue();
        rotated.Count.Should().Be(baseline.Count);
        for (int i = 0; i < baseline.Count; i++)
            rotated[i].ContactId.Should().Be(baseline[i].ContactId);
    }

    private static ContactManifold2D BuildManifold(
        LSCollider2D colliderA,
        LSCollider2D colliderB,
        int frame,
        out bool collided)
    {
        var manifold = new ContactManifold2D();
        collided = CollisionDetection2D.TryCollide(
            new CollisionWorkItem2D(
                colliderA,
                colliderB,
                ColliderSettings2D.GetCollisionType(colliderA.Shape, colliderB.Shape)),
            manifold,
            frame);
        return manifold;
    }

    private static SolidBody2D CreateBody(
        GravitasWorldContext context,
        LSCollider2D collider,
        Vector2d position,
        Fixed64 rotation = default)
    {
        var transform = new FixedTransform(
            new Vector3d(position.X, Fixed64.Zero, position.Y),
            FixedQuaternion.Identity,
            Vector3d.One);
        var body = new SolidBody2D(new TestMatterAgent(context, transform), collider)
        {
            Mass = Fixed64.One
        };
        body.Initialize(position, rotation, BodyMotionType.Static);
        return body;
    }
}
