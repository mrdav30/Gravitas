using FixedMathSharp;
using FluentAssertions;
using Gravitas.Colliders;
using Gravitas.CollisionHandling;
using Gravitas.Tests.Support;
using System.Linq;
using Xunit;

namespace Gravitas.Tests.Physics2D;

public sealed class CollisionDetection2DTests
{
    public static TheoryData<ColliderType2D, ColliderType2D, Vector2d, Fixed64, Vector2d, Fixed64> OverlappingBoundsSeparatedPairs =>
        new()
        {
            {
                ColliderType2D.Circle,
                ColliderType2D.AABox,
                Vector2d.Zero,
                Fixed64.Zero,
                new Vector2d(Fixed64.FromFraction(19, 10), Fixed64.FromFraction(19, 10)),
                Fixed64.Zero
            },
            {
                ColliderType2D.AABox,
                ColliderType2D.Circle,
                Vector2d.Zero,
                Fixed64.Zero,
                new Vector2d(Fixed64.FromFraction(19, 10), Fixed64.FromFraction(19, 10)),
                Fixed64.Zero
            },
            {
                ColliderType2D.Capsule,
                ColliderType2D.Circle,
                Vector2d.Zero,
                Fixed64.Zero,
                new Vector2d(Fixed64.FromFraction(3, 2), (Fixed64)2),
                Fixed64.Zero
            },
            {
                ColliderType2D.Circle,
                ColliderType2D.Capsule,
                new Vector2d(Fixed64.FromFraction(3, 2), (Fixed64)2),
                Fixed64.Zero,
                Vector2d.Zero,
                Fixed64.Zero
            },
            {
                ColliderType2D.Capsule,
                ColliderType2D.Capsule,
                Vector2d.Zero,
                Fixed64.Zero,
                new Vector2d(Fixed64.FromFraction(3, 2), Fixed64.FromFraction(3, 2)),
                FixedMath.DegToRad((Fixed64)90)
            }
        };

    [Theory]
    [InlineData(ColliderType2D.Circle, ColliderType2D.Circle)]
    [InlineData(ColliderType2D.Circle, ColliderType2D.AABox)]
    [InlineData(ColliderType2D.AABox, ColliderType2D.AABox)]
    [InlineData(ColliderType2D.Circle, ColliderType2D.ConvexPolygon)]
    [InlineData(ColliderType2D.AABox, ColliderType2D.ConvexPolygon)]
    [InlineData(ColliderType2D.ConvexPolygon, ColliderType2D.ConvexPolygon)]
    [InlineData(ColliderType2D.Capsule, ColliderType2D.Circle)]
    [InlineData(ColliderType2D.Capsule, ColliderType2D.AABox)]
    [InlineData(ColliderType2D.Capsule, ColliderType2D.ConvexPolygon)]
    [InlineData(ColliderType2D.Capsule, ColliderType2D.Capsule)]
    public void TryCollideManifold_ShouldSupportRequiredShapePairs(ColliderType2D firstType, ColliderType2D secondType)
    {
        using GravitasWorldContext context = Create2DContext();
        LSCollider2D first = CreateCollider(firstType);
        LSCollider2D second = CreateCollider(secondType);
        _ = CreateBody(context, first, new Vector2d(Fixed64.Zero, Fixed64.Zero));
        _ = CreateBody(context, second, new Vector2d(Fixed64.Half, Fixed64.Zero));

        (bool result, ContactManifold2D manifold) = BuildManifold(first, second);

        result.Should().BeTrue();
        manifold.HasContact.Should().BeTrue();
        manifold.PrimaryContact.Depth.Should().BeGreaterThan(Fixed64.Zero);
        manifold.PrimaryContact.Normal.X.Should().BeGreaterThan(Fixed64.Zero);
    }

    [Fact]
    public void TryCollide_WithSeparatedPolygons_ShouldReturnFalse()
    {
        using GravitasWorldContext context = Create2DContext();
        LSCollider2D first = CreateCollider(ColliderType2D.ConvexPolygon);
        LSCollider2D second = CreateCollider(ColliderType2D.ConvexPolygon);
        _ = CreateBody(context, first, new Vector2d(Fixed64.Zero, Fixed64.Zero));
        _ = CreateBody(context, second, new Vector2d((Fixed64)5, Fixed64.Zero));

        (bool collided, ContactManifold2D manifold) = BuildManifold(first, second);

        collided.Should().BeFalse();
        manifold.HasContact.Should().BeFalse();
    }

    [Fact]
    public void TryCollide_WithDiagonalSeparationOnSecondPolygonAxis_ShouldReturnFalse()
    {
        using GravitasWorldContext context = Create2DContext();
        LSCollider2D square = CreateCollider(ColliderType2D.ConvexPolygon);
        LSCollider2D diamond = CreateDiamondPolygon();
        _ = CreateBody(context, square, Vector2d.Zero);
        _ = CreateBody(
            context,
            diamond,
            new Vector2d(Fixed64.FromFraction(17, 10), Fixed64.FromFraction(17, 10)));

        bool collided = CollisionDetection2D.TryCollide(square, diamond, out Contact2D contact);
        (bool manifoldCollided, ContactManifold2D manifold) = BuildManifold(square, diamond);

        collided.Should().BeFalse();
        contact.Should().Be(default(Contact2D));
        manifoldCollided.Should().BeFalse();
        manifold.HasContact.Should().BeFalse();
    }

    [Fact]
    public void TryCollideManifold_WhenSecondPolygonProvidesMinimumAxis_ShouldOrientOwnerPoints()
    {
        using GravitasWorldContext context = Create2DContext();
        LSCollider2D square = CreateCollider(ColliderType2D.ConvexPolygon);
        LSCollider2D diamond = CreateDiamondPolygon();
        _ = CreateBody(context, square, Vector2d.Zero);
        _ = CreateBody(
            context,
            diamond,
            new Vector2d(Fixed64.FromFraction(6, 5), Fixed64.FromFraction(6, 5)));

        (bool collided, ContactManifold2D manifold) = BuildManifold(square, diamond);

        collided.Should().BeTrue();
        manifold.Count.Should().BeGreaterThan(0);
        manifold.PrimaryContact.Normal.X.Should().BeGreaterThan(Fixed64.Zero);
        manifold.PrimaryContact.Normal.Y.Should().BeGreaterThan(Fixed64.Zero);
        manifold.PrimaryContact.PointA.X.Should().BeLessThanOrEqualTo(Fixed64.One);
        manifold.PrimaryContact.PointA.Y.Should().BeLessThanOrEqualTo(Fixed64.One);
        Fixed64 projectedDepth = Vector2d.Dot(
            manifold.PrimaryContact.PointA - manifold.PrimaryContact.PointB,
            manifold.PrimaryContact.Normal);
        (projectedDepth - manifold.PrimaryContact.Depth).Abs()
            .Should()
            .BeLessThan(Fixed64.FromFraction(1, 1_000_000));
    }

    [Theory]
    [InlineData(ColliderType2D.Circle, ColliderType2D.Circle)]
    [InlineData(ColliderType2D.Circle, ColliderType2D.AABox)]
    [InlineData(ColliderType2D.AABox, ColliderType2D.Circle)]
    [InlineData(ColliderType2D.AABox, ColliderType2D.AABox)]
    [InlineData(ColliderType2D.Circle, ColliderType2D.ConvexPolygon)]
    [InlineData(ColliderType2D.ConvexPolygon, ColliderType2D.Circle)]
    [InlineData(ColliderType2D.Capsule, ColliderType2D.Circle)]
    [InlineData(ColliderType2D.Circle, ColliderType2D.Capsule)]
    [InlineData(ColliderType2D.Capsule, ColliderType2D.AABox)]
    [InlineData(ColliderType2D.AABox, ColliderType2D.Capsule)]
    [InlineData(ColliderType2D.Capsule, ColliderType2D.ConvexPolygon)]
    [InlineData(ColliderType2D.ConvexPolygon, ColliderType2D.Capsule)]
    [InlineData(ColliderType2D.Capsule, ColliderType2D.Capsule)]
    public void TryCollideSingleContact_ShouldSupportRequiredDispatchPairs(
        ColliderType2D firstType,
        ColliderType2D secondType)
    {
        using GravitasWorldContext context = Create2DContext();
        LSCollider2D first = CreateCollider(firstType);
        LSCollider2D second = CreateCollider(secondType);
        _ = CreateBody(context, first, Vector2d.Zero);
        _ = CreateBody(context, second, new Vector2d(Fixed64.Half, Fixed64.Zero));

        bool collided = CollisionDetection2D.TryCollide(first, second, out Contact2D contact);

        collided.Should().BeTrue();
        contact.Depth.Should().BeGreaterThanOrEqualTo(Fixed64.Zero);
        contact.Normal.MagnitudeSquared.Should().BeGreaterThan(Fixed64.Zero);
    }

    [Theory]
    [InlineData(ColliderType2D.ConvexPolygon, ColliderType2D.Circle)]
    [InlineData(ColliderType2D.Circle, ColliderType2D.Capsule)]
    [InlineData(ColliderType2D.ConvexPolygon, ColliderType2D.Capsule)]
    public void TryCollideSingleContact_WithSeparatedReversedDispatchPairs_ShouldReturnFalse(
        ColliderType2D firstType,
        ColliderType2D secondType)
    {
        using GravitasWorldContext context = Create2DContext();
        LSCollider2D first = CreateCollider(firstType);
        LSCollider2D second = CreateCollider(secondType);
        _ = CreateBody(context, first, Vector2d.Zero);
        _ = CreateBody(context, second, new Vector2d((Fixed64)5, Fixed64.Zero));

        bool collided = CollisionDetection2D.TryCollide(first, second, out Contact2D contact);

        collided.Should().BeFalse();
        contact.Should().Be(default(Contact2D));
    }

    [Theory]
    [InlineData(ColliderType2D.ConvexPolygon, ColliderType2D.Circle)]
    [InlineData(ColliderType2D.Circle, ColliderType2D.Capsule)]
    [InlineData(ColliderType2D.ConvexPolygon, ColliderType2D.Capsule)]
    public void TryCollideManifold_WithSeparatedReversedDispatchPairs_ShouldReturnFalse(
        ColliderType2D firstType,
        ColliderType2D secondType)
    {
        using GravitasWorldContext context = Create2DContext();
        LSCollider2D first = CreateCollider(firstType);
        LSCollider2D second = CreateCollider(secondType);
        _ = CreateBody(context, first, Vector2d.Zero);
        _ = CreateBody(context, second, new Vector2d((Fixed64)5, Fixed64.Zero));

        (bool collided, ContactManifold2D manifold) = BuildManifold(first, second);

        collided.Should().BeFalse();
        manifold.HasContact.Should().BeFalse();
    }

    [Fact]
    public void TryCollide_WithNonCompoundWorkItemMarkedCompound_ShouldReturnFalse()
    {
        using GravitasWorldContext context = Create2DContext();
        var first = new LSCircleCollider2D(Fixed64.One);
        var second = new LSCircleCollider2D(Fixed64.One);
        _ = CreateBody(context, first, Vector2d.Zero);
        _ = CreateBody(context, second, Vector2d.Zero);
        var manifold = new ContactManifold2D();

        bool single = CollisionDetection2D.TryCollide(
            new CollisionWorkItem2D(first, second, CollisionType2D.Compound),
            out Contact2D contact);
        bool multi = CollisionDetection2D.TryCollide(
            new CollisionWorkItem2D(first, second, CollisionType2D.Compound),
            manifold,
            frame: 4);

        single.Should().BeFalse();
        contact.Should().Be(default(Contact2D));
        multi.Should().BeFalse();
        manifold.HasContact.Should().BeFalse();
    }

    [Fact]
    public void TryCollide_WithUnsupportedCollisionType_ShouldReturnFalse()
    {
        using GravitasWorldContext context = Create2DContext();
        var first = new LSCircleCollider2D(Fixed64.One);
        var second = new LSCircleCollider2D(Fixed64.One);
        _ = CreateBody(context, first, Vector2d.Zero);
        _ = CreateBody(context, second, Vector2d.Zero);
        var manifold = new ContactManifold2D();

        bool single = CollisionDetection2D.TryCollide(
            new CollisionWorkItem2D(first, second, CollisionType2D.None),
            out Contact2D contact);
        bool multi = CollisionDetection2D.TryCollide(
            new CollisionWorkItem2D(first, second, CollisionType2D.None),
            manifold,
            frame: 4);

        single.Should().BeFalse();
        contact.Should().Be(default(Contact2D));
        multi.Should().BeFalse();
        manifold.HasContact.Should().BeFalse();
    }

    [Theory]
    [MemberData(nameof(OverlappingBoundsSeparatedPairs))]
    public void TryCollide_WithOverlappingBoundsButExactSeparation_ShouldReturnFalse(
        ColliderType2D firstType,
        ColliderType2D secondType,
        Vector2d firstPosition,
        Fixed64 firstRotation,
        Vector2d secondPosition,
        Fixed64 secondRotation)
    {
        using GravitasWorldContext context = Create2DContext();
        LSCollider2D first = CreateCollider(firstType);
        LSCollider2D second = CreateCollider(secondType);
        _ = CreateBody(context, first, firstPosition, firstRotation);
        _ = CreateBody(context, second, secondPosition, secondRotation);

        CollisionDetection2D.BoundsOverlap(first, second).Should().BeTrue();

        bool single = CollisionDetection2D.TryCollide(first, second, out Contact2D contact);
        (bool manifoldCollided, ContactManifold2D manifold) = BuildManifold(first, second);

        single.Should().BeFalse();
        contact.Should().Be(default(Contact2D));
        manifoldCollided.Should().BeFalse();
        manifold.HasContact.Should().BeFalse();
    }

    [Fact]
    public void TryCollide_WithSmallBoxTouchingCapsuleBoundsButOutsideCapsuleRadius_ShouldReturnFalse()
    {
        using GravitasWorldContext context = Create2DContext();
        var capsule = new LSCapsuleCollider2D(Fixed64.Half, (Fixed64)2);
        var box = new LSAABBoxCollider2D(Vector2d.One);
        _ = CreateBody(context, capsule, Vector2d.Zero);
        _ = CreateBody(context, box, new Vector2d(Fixed64.One, Fixed64.FromFraction(3, 2)));

        CollisionDetection2D.BoundsOverlap(capsule, box).Should().BeTrue();

        bool capsuleFirst = CollisionDetection2D.TryCollide(capsule, box, out Contact2D firstContact);
        bool boxFirst = CollisionDetection2D.TryCollide(box, capsule, out Contact2D secondContact);
        (bool manifoldCollided, ContactManifold2D manifold) = BuildManifold(capsule, box);

        capsuleFirst.Should().BeFalse();
        boxFirst.Should().BeFalse();
        firstContact.Should().Be(default(Contact2D));
        secondContact.Should().Be(default(Contact2D));
        manifoldCollided.Should().BeFalse();
        manifold.HasContact.Should().BeFalse();
    }

    [Fact]
    public void TryCollide_WithCapsuleConvexSeparatedOnConvexFaceAxis_ShouldReturnFalse()
    {
        using GravitasWorldContext context = Create2DContext();
        var capsule = new LSCapsuleCollider2D(Fixed64.Half, (Fixed64)3);
        LSPolygonCollider2D diamond = CreateDiamondPolygon();
        _ = CreateBody(context, capsule, Vector2d.Zero, FixedMath.DegToRad((Fixed64)90));
        _ = CreateBody(
            context,
            diamond,
            new Vector2d(Fixed64.FromFraction(3, 2), Fixed64.FromFraction(3, 2)),
            Fixed64.Zero);

        CollisionDetection2D.BoundsOverlap(capsule, diamond).Should().BeTrue();

        bool collided = CollisionDetection2D.TryCollide(capsule, diamond, out Contact2D contact);
        (bool manifoldCollided, ContactManifold2D manifold) = BuildManifold(diamond, capsule);

        collided.Should().BeFalse();
        contact.Should().Be(default(Contact2D));
        manifoldCollided.Should().BeFalse();
        manifold.HasContact.Should().BeFalse();
    }

    [Fact]
    public void TryCollideManifold_WithCircleCircleOverlap_ShouldProduceOneContact()
    {
        using GravitasWorldContext context = Create2DContext();
        var first = new LSCircleCollider2D(Fixed64.One);
        var second = new LSCircleCollider2D(Fixed64.One);
        _ = CreateBody(context, first, Vector2d.Zero);
        _ = CreateBody(context, second, new Vector2d(Fixed64.One, Fixed64.Zero));

        (bool collided, ContactManifold2D manifold) = BuildManifold(first, second);

        collided.Should().BeTrue();
        manifold.Count.Should().Be(1);
        manifold[0].Depth.Should().Be(Fixed64.One);
        manifold[0].Normal.Should().Be(Vector2d.Right);
        manifold[0].PointA.Should().Be(new Vector2d(Fixed64.One, Fixed64.Zero));
        manifold[0].PointB.Should().Be(Vector2d.Zero);
    }

    [Fact]
    public void TryCollide_WithCoincidentCircles_ShouldUseDeterministicFallbackNormal()
    {
        using GravitasWorldContext context = Create2DContext();
        var first = new LSCircleCollider2D(Fixed64.One);
        var second = new LSCircleCollider2D(Fixed64.One);
        _ = CreateBody(context, first, Vector2d.Zero);
        _ = CreateBody(context, second, Vector2d.Zero);

        bool collided = CollisionDetection2D.TryCollide(first, second, out Contact2D contact);

        collided.Should().BeTrue();
        contact.Depth.Should().Be((Fixed64)2);
        contact.Normal.Should().Be(Vector2d.Right);
        contact.PointA.Should().Be(Vector2d.Right);
        contact.PointB.Should().Be(-Vector2d.Right);
    }

    [Fact]
    public void TryCollideManifold_WithCircleConvexOverlap_ShouldProduceOneContact()
    {
        using GravitasWorldContext context = Create2DContext();
        var circle = new LSCircleCollider2D(Fixed64.One);
        LSCollider2D box = CreateCollider(ColliderType2D.AABox);
        _ = CreateBody(context, circle, Vector2d.Zero);
        _ = CreateBody(context, box, new Vector2d(Fixed64.FromFraction(3, 2), Fixed64.Zero));

        (bool collided, ContactManifold2D manifold) = BuildManifold(circle, box);

        collided.Should().BeTrue();
        manifold.Count.Should().Be(1);
        manifold[0].Depth.Should().Be(Fixed64.Half);
        manifold[0].Normal.Should().Be(Vector2d.Right);
    }

    [Fact]
    public void TryCollideManifold_WithCapsuleCircleEndCapOverlap_ShouldProduceStableContact()
    {
        using GravitasWorldContext context = Create2DContext();
        var capsule = new LSCapsuleCollider2D(Fixed64.Half, (Fixed64)3);
        var circle = new LSCircleCollider2D(Fixed64.Half);
        _ = CreateBody(context, capsule, Vector2d.Zero);
        _ = CreateBody(context, circle, new Vector2d(Fixed64.Zero, Fixed64.FromFraction(3, 2)));

        (bool collided, ContactManifold2D manifold) = BuildManifold(capsule, circle);

        collided.Should().BeTrue();
        manifold.Count.Should().Be(1);
        manifold[0].Normal.Should().Be(Vector2d.Forward);
        manifold[0].Depth.Should().Be(Fixed64.Half);
    }

    [Fact]
    public void TryCollideManifold_WithCircleCapsuleEndCapOverlap_ShouldReverseStableContact()
    {
        using GravitasWorldContext context = Create2DContext();
        var capsule = new LSCapsuleCollider2D(Fixed64.Half, (Fixed64)3);
        var circle = new LSCircleCollider2D(Fixed64.Half);
        _ = CreateBody(context, capsule, Vector2d.Zero);
        _ = CreateBody(context, circle, new Vector2d(Fixed64.Zero, Fixed64.FromFraction(3, 2)));

        (bool forwardCollided, ContactManifold2D forward) = BuildManifold(capsule, circle);
        (bool reversedCollided, ContactManifold2D reversed) = BuildManifold(circle, capsule);

        forwardCollided.Should().BeTrue();
        reversedCollided.Should().BeTrue();
        reversed.Count.Should().Be(1);
        reversed[0].PointA.Should().Be(forward[0].PointB);
        reversed[0].PointB.Should().Be(forward[0].PointA);
        reversed[0].Depth.Should().Be(forward[0].Depth);
        reversed[0].Normal.Should().Be(-forward[0].Normal);
        reversed[0].MaterialA.Should().Be(circle.Material);
        reversed[0].MaterialB.Should().Be(capsule.Material);
    }

    [Fact]
    public void TryCollide_WithCircleCenteredOnCapsuleSegment_ShouldUseDeterministicFallbackNormal()
    {
        using GravitasWorldContext context = Create2DContext();
        var capsule = new LSCapsuleCollider2D(Fixed64.Half, (Fixed64)3);
        var circle = new LSCircleCollider2D(Fixed64.Half);
        _ = CreateBody(context, capsule, Vector2d.Zero);
        _ = CreateBody(context, circle, Vector2d.Zero);

        bool collided = CollisionDetection2D.TryCollide(capsule, circle, out Contact2D contact);

        collided.Should().BeTrue();
        contact.Depth.Should().Be(Fixed64.One);
        contact.Normal.Should().Be(Vector2d.Right);
        contact.PointA.Should().Be(Vector2d.Right * Fixed64.Half);
        contact.PointB.Should().Be(-Vector2d.Right * Fixed64.Half);
    }

    [Fact]
    public void TryCollideManifold_WithHorizontalCapsuleOnFlatBox_ShouldProduceTwoSideContacts()
    {
        using GravitasWorldContext context = Create2DContext();
        var capsule = new LSCapsuleCollider2D(Fixed64.Half, (Fixed64)4);
        var box = new LSAABBoxCollider2D(new Vector2d((Fixed64)5, Fixed64.One));
        _ = CreateBody(
            context,
            capsule,
            new Vector2d(Fixed64.Zero, Fixed64.Zero),
            FixedMath.DegToRad((Fixed64)90));
        _ = CreateBody(context, box, new Vector2d(Fixed64.Zero, -Fixed64.Half));

        (bool collided, ContactManifold2D manifold) = BuildManifold(capsule, box);

        collided.Should().BeTrue();
        manifold.Count.Should().Be(2);
        manifold.Select(static contact => contact.Normal).Should().AllBeEquivalentTo(-Vector2d.Forward);
        manifold.Select(static contact => contact.Depth).Should().AllBeEquivalentTo(Fixed64.Half);
        manifold.Select(static contact => contact.PointA.Y).Should().AllBeEquivalentTo(-Fixed64.Half);
        manifold.Select(static contact => contact.PointB.Y).Should().AllBeEquivalentTo(Fixed64.Zero);
    }

    [Fact]
    public void TryCollideManifold_WithFlatBoxOnHorizontalCapsule_ShouldProduceReversedSideContacts()
    {
        using GravitasWorldContext context = Create2DContext();
        var box = new LSAABBoxCollider2D(new Vector2d((Fixed64)5, Fixed64.One));
        var capsule = new LSCapsuleCollider2D(Fixed64.Half, (Fixed64)4);
        _ = CreateBody(context, box, new Vector2d(Fixed64.Zero, -Fixed64.Half));
        _ = CreateBody(
            context,
            capsule,
            new Vector2d(Fixed64.Zero, Fixed64.Zero),
            FixedMath.DegToRad((Fixed64)90));

        (bool collided, ContactManifold2D manifold) = BuildManifold(box, capsule);

        collided.Should().BeTrue();
        manifold.Count.Should().Be(2);
        manifold.Select(static contact => contact.Normal).Should().AllBeEquivalentTo(Vector2d.Forward);
        manifold.Select(static contact => contact.Depth).Should().AllBeEquivalentTo(Fixed64.Half);
        manifold.Select(static contact => contact.PointA.Y).Should().AllBeEquivalentTo(Fixed64.Zero);
        manifold.Select(static contact => contact.PointB.Y).Should().AllBeEquivalentTo(-Fixed64.Half);
    }

    [Fact]
    public void TryCollideManifold_WithDegenerateCapsuleAgainstBox_ShouldUseSingleContactFallback()
    {
        using GravitasWorldContext context = Create2DContext();
        var capsule = new LSCapsuleCollider2D(Fixed64.Half, Fixed64.One);
        var box = new LSAABBoxCollider2D(new Vector2d((Fixed64)2, (Fixed64)2));
        _ = CreateBody(context, capsule, Vector2d.Zero);
        _ = CreateBody(context, box, new Vector2d(Fixed64.Half, Fixed64.Zero));

        (bool collided, ContactManifold2D manifold) = BuildManifold(capsule, box);

        collided.Should().BeTrue();
        manifold.Count.Should().Be(1);
        manifold.PrimaryContact.Depth.Should().BeGreaterThan(Fixed64.Zero);
        manifold.PrimaryContact.Normal.MagnitudeSquared.Should().BeGreaterThan(Fixed64.Zero);
    }

    [Fact]
    public void TryCollideManifold_WithBoxAgainstDegenerateCapsule_ShouldUseReversedSingleContactFallback()
    {
        using GravitasWorldContext context = Create2DContext();
        var box = new LSAABBoxCollider2D(new Vector2d((Fixed64)2, (Fixed64)2));
        var capsule = new LSCapsuleCollider2D(Fixed64.Half, Fixed64.One);
        _ = CreateBody(context, box, new Vector2d(Fixed64.Half, Fixed64.Zero));
        _ = CreateBody(context, capsule, Vector2d.Zero);

        (bool collided, ContactManifold2D manifold) = BuildManifold(box, capsule);

        collided.Should().BeTrue();
        manifold.Count.Should().Be(1);
        manifold.PrimaryContact.Depth.Should().BeGreaterThan(Fixed64.Zero);
        manifold.PrimaryContact.Normal.MagnitudeSquared.Should().BeGreaterThan(Fixed64.Zero);
    }

    [Fact]
    public void TryCollideManifold_WithCapsuleCircleTangentAndSeparatedBoundary_ShouldStayDeterministic()
    {
        using GravitasWorldContext context = Create2DContext();
        var capsule = new LSCapsuleCollider2D(Fixed64.Half, (Fixed64)3);
        var tangentCircle = new LSCircleCollider2D(Fixed64.Half);
        var separatedCircle = new LSCircleCollider2D(Fixed64.Half);
        _ = CreateBody(context, capsule, Vector2d.Zero);
        _ = CreateBody(context, tangentCircle, new Vector2d(Fixed64.Zero, (Fixed64)2));
        _ = CreateBody(context, separatedCircle, new Vector2d(Fixed64.Zero, Fixed64.FromFraction(33, 16)));

        (bool tangentCollided, ContactManifold2D tangent) = BuildManifold(capsule, tangentCircle);
        (bool separatedCollided, ContactManifold2D separated) = BuildManifold(capsule, separatedCircle);

        tangentCollided.Should().BeTrue();
        tangent.Count.Should().Be(1);
        tangent[0].Depth.Should().Be(Fixed64.Zero);
        tangent[0].Normal.Should().Be(Vector2d.Forward);
        separatedCollided.Should().BeFalse();
        separated.HasContact.Should().BeFalse();
    }

    [Fact]
    public void TryCollideManifold_WithRotatedCapsuleConvexOverlap_ShouldReportContact()
    {
        using GravitasWorldContext context = Create2DContext();
        var capsule = new LSCapsuleCollider2D(Fixed64.Half, (Fixed64)3);
        var box = new LSAABBoxCollider2D(Vector2d.One);
        _ = CreateBody(context, capsule, Vector2d.Zero, FixedMath.DegToRad((Fixed64)45));
        _ = CreateBody(context, box, new Vector2d(Fixed64.FromFraction(3, 4), Fixed64.FromFraction(3, 4)));

        (bool collided, ContactManifold2D manifold) = BuildManifold(capsule, box);

        collided.Should().BeTrue();
        manifold.HasContact.Should().BeTrue();
        manifold.PrimaryContact.Depth.Should().BeGreaterThan(Fixed64.Zero);
        manifold.PrimaryContact.Normal.MagnitudeSquared.Should().BeGreaterThan(Fixed64.Zero);
    }

    [Fact]
    public void TryCollideManifold_WithConvexRotatedCapsuleOverlap_ShouldUseReversedSingleContactFallback()
    {
        using GravitasWorldContext context = Create2DContext();
        var box = new LSAABBoxCollider2D(Vector2d.One);
        var capsule = new LSCapsuleCollider2D(Fixed64.Half, (Fixed64)3);
        _ = CreateBody(context, box, new Vector2d(Fixed64.FromFraction(3, 4), Fixed64.FromFraction(3, 4)));
        _ = CreateBody(context, capsule, Vector2d.Zero, FixedMath.DegToRad((Fixed64)45));

        (bool collided, ContactManifold2D manifold) = BuildManifold(box, capsule);

        collided.Should().BeTrue();
        manifold.Count.Should().Be(1);
        manifold.PrimaryContact.Depth.Should().BeGreaterThan(Fixed64.Zero);
        manifold.PrimaryContact.Normal.MagnitudeSquared.Should().BeGreaterThan(Fixed64.Zero);
    }

    [Fact]
    public void TryCollideManifold_WithParallelCapsules_ShouldReportStableSideContact()
    {
        using GravitasWorldContext context = Create2DContext();
        var first = new LSCapsuleCollider2D(Fixed64.Half, (Fixed64)4);
        var second = new LSCapsuleCollider2D(Fixed64.Half, (Fixed64)4);
        Fixed64 horizontal = FixedMath.DegToRad((Fixed64)90);
        _ = CreateBody(context, first, Vector2d.Zero, horizontal);
        _ = CreateBody(context, second, new Vector2d(Fixed64.Zero, Fixed64.FromFraction(3, 4)), horizontal);

        (bool collided, ContactManifold2D manifold) = BuildManifold(first, second);

        collided.Should().BeTrue();
        manifold.Count.Should().Be(1);
        manifold[0].Normal.Should().Be(Vector2d.Forward);
        manifold[0].Depth.Should().Be(Fixed64.FromFraction(1, 4));
    }

    [Fact]
    public void TryCollide_WithCrossingCapsules_ShouldUseSegmentIntersectionContact()
    {
        using GravitasWorldContext context = Create2DContext();
        var horizontal = new LSCapsuleCollider2D(Fixed64.Half, (Fixed64)4);
        var vertical = new LSCapsuleCollider2D(Fixed64.Half, (Fixed64)4);
        _ = CreateBody(context, horizontal, Vector2d.Zero, FixedMath.DegToRad((Fixed64)90));
        _ = CreateBody(context, vertical, Vector2d.Zero);

        bool collided = CollisionDetection2D.TryCollide(horizontal, vertical, out Contact2D contact);

        collided.Should().BeTrue();
        contact.Depth.Should().Be(Fixed64.One);
        contact.Normal.Should().Be(Vector2d.Right);
        contact.PointA.Should().Be(new Vector2d(Fixed64.Half, Fixed64.Zero));
        contact.PointB.Should().Be(new Vector2d(-Fixed64.Half, Fixed64.Zero));
    }

    [Fact]
    public void TryCollide_WithCoincidentCapsules_ShouldUseDeterministicFallbackNormal()
    {
        using GravitasWorldContext context = Create2DContext();
        var first = new LSCapsuleCollider2D(Fixed64.Half, (Fixed64)4);
        var second = new LSCapsuleCollider2D(Fixed64.Half, (Fixed64)4);
        _ = CreateBody(context, first, Vector2d.Zero);
        _ = CreateBody(context, second, Vector2d.Zero);

        bool collided = CollisionDetection2D.TryCollide(first, second, out Contact2D contact);

        collided.Should().BeTrue();
        contact.Depth.Should().Be(Fixed64.One);
        contact.Normal.Should().Be(Vector2d.Right);
    }

    [Fact]
    public void TryCollideManifold_WithConvexFaceOverlap_ShouldProduceTwoIncidentEdgeContacts()
    {
        using GravitasWorldContext context = Create2DContext();
        LSCollider2D first = CreateCollider(ColliderType2D.AABox);
        LSCollider2D second = CreateCollider(ColliderType2D.AABox);
        _ = CreateBody(context, first, Vector2d.Zero);
        _ = CreateBody(context, second, new Vector2d(Fixed64.FromFraction(3, 2), Fixed64.Zero));

        (bool collided, ContactManifold2D manifold) = BuildManifold(first, second);

        collided.Should().BeTrue();
        manifold.Count.Should().Be(2);
        manifold.Select(static contact => contact.Depth).Should().AllBeEquivalentTo(Fixed64.Half);
        manifold.Select(static contact => contact.Normal).Should().AllBeEquivalentTo(Vector2d.Right);
        manifold.Select(static contact => contact.PointA.X).Should().AllBeEquivalentTo(Fixed64.One);
        manifold.Select(static contact => contact.PointB.X).Should().AllBeEquivalentTo(Fixed64.Half);
        manifold.Select(static contact => contact.PointA.Y).Should().BeEquivalentTo(new[] { Fixed64.One, -Fixed64.One });
        manifold.Select(static contact => contact.PointB.Y).Should().BeEquivalentTo(new[] { Fixed64.One, -Fixed64.One });
    }

    [Fact]
    public void TryCollideManifold_WithRotatedIncidentPolygon_ShouldClipAgainstSecondReferenceEdge()
    {
        using GravitasWorldContext context = Create2DContext();
        var wideBox = new LSAABBoxCollider2D(new Vector2d((Fixed64)4, Fixed64.One));
        var diamond = new LSPolygonCollider2D(
            new Vector2d(Fixed64.Zero, (Fixed64)(-2)),
            new Vector2d((Fixed64)2, Fixed64.Zero),
            new Vector2d(Fixed64.Zero, (Fixed64)2),
            new Vector2d((Fixed64)(-2), Fixed64.Zero));
        _ = CreateBody(context, wideBox, Vector2d.Zero);
        _ = CreateBody(context, diamond, new Vector2d(Fixed64.FromFraction(7, 2), Fixed64.Zero));

        (bool collided, ContactManifold2D manifold) = BuildManifold(wideBox, diamond);

        collided.Should().BeTrue();
        manifold.Count.Should().BeGreaterThan(0);
        manifold.PrimaryContact.Normal.X.Should().BeGreaterThan(Fixed64.Zero);
        manifold.PrimaryContact.Depth.Should().BeGreaterThan(Fixed64.Zero);
    }

    [Fact]
    public void TryCollideManifold_WithConvexCornerTouch_ShouldProduceOneContact()
    {
        using GravitasWorldContext context = Create2DContext();
        LSCollider2D first = CreateCollider(ColliderType2D.AABox);
        LSCollider2D second = CreateCollider(ColliderType2D.AABox);
        _ = CreateBody(context, first, Vector2d.Zero);
        _ = CreateBody(context, second, new Vector2d((Fixed64)2, (Fixed64)2));

        (bool collided, ContactManifold2D manifold) = BuildManifold(first, second);

        collided.Should().BeTrue();
        manifold.Count.Should().Be(1);
        manifold[0].Depth.Should().Be(Fixed64.Zero);
        manifold[0].PointA.Should().Be(new Vector2d(Fixed64.One, Fixed64.One));
        manifold[0].PointB.Should().Be(new Vector2d(Fixed64.One, Fixed64.One));
    }

    [Fact]
    public void TryCollideManifold_WithReversedPairOrder_ShouldReverseNormalsAndOwnerPoints()
    {
        using GravitasWorldContext context = Create2DContext();
        LSCollider2D first = CreateCollider(ColliderType2D.AABox);
        LSCollider2D second = CreateCollider(ColliderType2D.AABox);
        _ = CreateBody(context, first, Vector2d.Zero);
        _ = CreateBody(context, second, new Vector2d(Fixed64.FromFraction(3, 2), Fixed64.Zero));

        (bool collided, ContactManifold2D forward) = BuildManifold(first, second);
        (bool reversedCollided, ContactManifold2D reverse) = BuildManifold(second, first);

        collided.Should().BeTrue();
        reversedCollided.Should().BeTrue();
        reverse.Count.Should().Be(forward.Count);
        for (int i = 0; i < forward.Count; i++)
        {
            reverse[i].ContactId.Should().Be(forward[i].ContactId);
            reverse[i].Depth.Should().Be(forward[i].Depth);
            reverse[i].Normal.Should().Be(-forward[i].Normal);
            reverse[i].PointA.Should().Be(forward[i].PointB);
            reverse[i].PointB.Should().Be(forward[i].PointA);
        }
    }

    [Fact]
    public void TryCollideManifold_WithCompoundPrimitive_ShouldReduceToDeepestTwoOwnerContacts()
    {
        using GravitasWorldContext context = Create2DContext();
        var compound = new LSCompoundCollider2D(
            CompoundColliderPart2D.Circle(Fixed64.Half, new Vector2d(-Fixed64.One, Fixed64.Zero)),
            CompoundColliderPart2D.Circle(Fixed64.Half, Vector2d.Zero),
            CompoundColliderPart2D.Circle(Fixed64.Half, new Vector2d(Fixed64.FromFraction(1, 4), Fixed64.Zero)));
        var circle = new LSCircleCollider2D(Fixed64.Half);
        _ = CreateBody(context, compound, Vector2d.Zero);
        _ = CreateBody(context, circle, Vector2d.Zero);

        (bool collided, ContactManifold2D manifold) = BuildManifold(compound, circle);

        collided.Should().BeTrue();
        manifold.Count.Should().Be(2);
        manifold.Select(static contact => contact.Depth).Should().BeEquivalentTo(new[]
        {
            Fixed64.One,
            Fixed64.FromFraction(3, 4)
        });
    }

    [Fact]
    public void TryCollide_WithPrimitiveCompound_ShouldOrientContactFromPrimitiveToMatchingPart()
    {
        using GravitasWorldContext context = Create2DContext();
        var circle = new LSCircleCollider2D(Fixed64.Half);
        var compound = new LSCompoundCollider2D(
            CompoundColliderPart2D.Circle(Fixed64.Half, new Vector2d((Fixed64)(-4), Fixed64.Zero)),
            CompoundColliderPart2D.Circle(Fixed64.Half, new Vector2d(Fixed64.FromFraction(3, 4), Fixed64.Zero)));
        _ = CreateBody(context, circle, Vector2d.Zero);
        _ = CreateBody(context, compound, Vector2d.Zero);

        bool collided = CollisionDetection2D.TryCollide(circle, compound, out Contact2D contact);

        collided.Should().BeTrue();
        contact.Depth.Should().Be(Fixed64.FromFraction(1, 4));
        contact.Normal.Should().Be(Vector2d.Right);
        contact.PointA.Should().Be(new Vector2d(Fixed64.Half, Fixed64.Zero));
        contact.PointB.Should().Be(new Vector2d(Fixed64.FromFraction(1, 4), Fixed64.Zero));
    }

    [Fact]
    public void TryCollideManifold_WithPrimitiveCompound_ShouldPopulateOwnerOrderedContacts()
    {
        using GravitasWorldContext context = Create2DContext();
        var circle = new LSCircleCollider2D(Fixed64.Half);
        var compound = new LSCompoundCollider2D(
            CompoundColliderPart2D.Circle(Fixed64.Half, new Vector2d((Fixed64)(-4), Fixed64.Zero)),
            CompoundColliderPart2D.Circle(Fixed64.Half, Vector2d.Zero),
            CompoundColliderPart2D.Circle(Fixed64.Half, new Vector2d(Fixed64.FromFraction(1, 4), Fixed64.Zero)));
        _ = CreateBody(context, circle, Vector2d.Zero);
        _ = CreateBody(context, compound, Vector2d.Zero);

        (bool collided, ContactManifold2D manifold) = BuildManifold(circle, compound);

        collided.Should().BeTrue();
        manifold.Count.Should().Be(2);
        manifold.Select(static contact => contact.Depth).Should().BeEquivalentTo(new[]
        {
            Fixed64.One,
            Fixed64.FromFraction(3, 4)
        });
        manifold.PrimaryContact.Normal.Should().Be(Vector2d.Right);
    }

    [Fact]
    public void TryCollide_WithCompoundCompound_ShouldUseDeepestPartContact()
    {
        using GravitasWorldContext context = Create2DContext();
        var first = new LSCompoundCollider2D(
            CompoundColliderPart2D.Circle(Fixed64.Half, new Vector2d((Fixed64)(-4), Fixed64.Zero)),
            CompoundColliderPart2D.Circle(Fixed64.Half, Vector2d.Zero));
        var second = new LSCompoundCollider2D(
            CompoundColliderPart2D.Circle(Fixed64.Half, new Vector2d((Fixed64)4, Fixed64.Zero)),
            CompoundColliderPart2D.Circle(Fixed64.Half, new Vector2d(Fixed64.FromFraction(1, 4), Fixed64.Zero)));
        _ = CreateBody(context, first, Vector2d.Zero);
        _ = CreateBody(context, second, Vector2d.Zero);

        bool collided = CollisionDetection2D.TryCollide(first, second, out Contact2D contact);

        collided.Should().BeTrue();
        contact.Depth.Should().Be(Fixed64.FromFraction(3, 4));
        contact.Normal.Should().Be(Vector2d.Right);
    }

    [Fact]
    public void TryCollide_WithCompoundPrimitive_ShouldKeepEarlierDeeperPartContact()
    {
        using GravitasWorldContext context = Create2DContext();
        var compound = new LSCompoundCollider2D(
            CompoundColliderPart2D.Circle(Fixed64.Half, Vector2d.Zero),
            CompoundColliderPart2D.Circle(Fixed64.Half, new Vector2d(Fixed64.FromFraction(1, 4), Fixed64.Zero)));
        var circle = new LSCircleCollider2D(Fixed64.Half);
        _ = CreateBody(context, compound, Vector2d.Zero);
        _ = CreateBody(context, circle, Vector2d.Zero);

        bool collided = CollisionDetection2D.TryCollide(compound, circle, out Contact2D contact);

        collided.Should().BeTrue();
        contact.Depth.Should().Be(Fixed64.One);
    }

    [Fact]
    public void TryCollide_WithCompoundCompound_ShouldKeepEarlierDeeperPartContact()
    {
        using GravitasWorldContext context = Create2DContext();
        var first = new LSCompoundCollider2D(
            CompoundColliderPart2D.Circle(Fixed64.Half, Vector2d.Zero));
        var second = new LSCompoundCollider2D(
            CompoundColliderPart2D.Circle(Fixed64.Half, Vector2d.Zero),
            CompoundColliderPart2D.Circle(Fixed64.Half, new Vector2d(Fixed64.FromFraction(1, 4), Fixed64.Zero)));
        _ = CreateBody(context, first, Vector2d.Zero);
        _ = CreateBody(context, second, Vector2d.Zero);

        bool collided = CollisionDetection2D.TryCollide(first, second, out Contact2D contact);

        collided.Should().BeTrue();
        contact.Depth.Should().Be(Fixed64.One);
    }

    [Fact]
    public void TryCollide_WithSeparatedCompoundCompound_ShouldReturnFalse()
    {
        using GravitasWorldContext context = Create2DContext();
        var first = new LSCompoundCollider2D(
            CompoundColliderPart2D.Circle(Fixed64.Half, new Vector2d((Fixed64)(-4), Fixed64.Zero)),
            CompoundColliderPart2D.Circle(Fixed64.Half, new Vector2d((Fixed64)(-2), Fixed64.Zero)));
        var second = new LSCompoundCollider2D(
            CompoundColliderPart2D.Circle(Fixed64.Half, new Vector2d((Fixed64)2, Fixed64.Zero)),
            CompoundColliderPart2D.Circle(Fixed64.Half, new Vector2d((Fixed64)4, Fixed64.Zero)));
        _ = CreateBody(context, first, Vector2d.Zero);
        _ = CreateBody(context, second, Vector2d.Zero);

        bool collided = CollisionDetection2D.TryCollide(first, second, out Contact2D contact);

        collided.Should().BeFalse();
        contact.Should().Be(default(Contact2D));
    }

    [Fact]
    public void TryCollide_WithOverlappingCompoundBoundsButNoPartContact_ShouldReturnFalse()
    {
        using GravitasWorldContext context = Create2DContext();
        var compound = new LSCompoundCollider2D(
            CompoundColliderPart2D.Circle(Fixed64.Half, new Vector2d(Fixed64.FromFraction(-3, 2), Fixed64.Zero)),
            CompoundColliderPart2D.Circle(Fixed64.Half, new Vector2d(Fixed64.FromFraction(3, 2), Fixed64.Zero)));
        var circle = new LSCircleCollider2D(Fixed64.Half);
        _ = CreateBody(context, compound, Vector2d.Zero);
        _ = CreateBody(context, circle, new Vector2d(Fixed64.Zero, Fixed64.FromFraction(9, 10)));

        CollisionDetection2D.BoundsOverlap(compound, circle).Should().BeTrue();

        bool compoundFirst = CollisionDetection2D.TryCollide(compound, circle, out Contact2D firstContact);
        bool primitiveFirst = CollisionDetection2D.TryCollide(circle, compound, out Contact2D secondContact);

        compoundFirst.Should().BeFalse();
        primitiveFirst.Should().BeFalse();
        firstContact.Should().Be(default(Contact2D));
        secondContact.Should().Be(default(Contact2D));
    }

    [Fact]
    public void TryCollide_WithOverlappingCompoundCompoundBoundsButNoPartContact_ShouldReturnFalse()
    {
        using GravitasWorldContext context = Create2DContext();
        var first = new LSCompoundCollider2D(
            CompoundColliderPart2D.Circle(Fixed64.Half, new Vector2d(Fixed64.FromFraction(-3, 2), Fixed64.Zero)),
            CompoundColliderPart2D.Circle(Fixed64.Half, new Vector2d(Fixed64.FromFraction(3, 2), Fixed64.Zero)));
        var second = new LSCompoundCollider2D(
            CompoundColliderPart2D.Circle(Fixed64.Half, new Vector2d(Fixed64.Zero, Fixed64.One)));
        _ = CreateBody(context, first, Vector2d.Zero);
        _ = CreateBody(context, second, Vector2d.Zero);

        CollisionDetection2D.BoundsOverlap(first, second).Should().BeTrue();

        bool collided = CollisionDetection2D.TryCollide(first, second, out Contact2D contact);
        (bool manifoldCollided, ContactManifold2D manifold) = BuildManifold(first, second);

        collided.Should().BeFalse();
        contact.Should().Be(default(Contact2D));
        manifoldCollided.Should().BeFalse();
        manifold.HasContact.Should().BeFalse();
    }

    private static SolidBody2D CreateBody(
        GravitasWorldContext context,
        LSCollider2D collider,
        Vector2d position,
        Fixed64 rotation = default)
    {
        var transform = new FixedTransform(new Vector3d(position.X, Fixed64.Zero, position.Y), FixedQuaternion.Identity, Vector3d.One);
        var agent = new TestMatterAgent(context, transform);
        var body = new SolidBody2D(agent, collider)
        {
            Mass = Fixed64.One,
            FreezeAxes = BodyFreezeAxes2D.Position
        };
        body.Initialize(position, rotation);
        return body;
    }

    private static GravitasWorldContext Create2DContext()
    {
        GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        context.Settings.RuntimeMode = PhysicsRuntimeMode.TwoD;
        return context;
    }

    private static (bool Collided, ContactManifold2D Manifold) BuildManifold(
        LSCollider2D colliderA,
        LSCollider2D colliderB,
        int frame = 11)
    {
        var manifold = new ContactManifold2D();
        CollisionType2D collisionType = ColliderSettings2D.GetCollisionType(colliderA.Shape, colliderB.Shape);
        bool collided = CollisionDetection2D.TryCollide(
            new CollisionWorkItem2D(colliderA, colliderB, collisionType),
            manifold,
            frame);
        return (collided, manifold);
    }

    private static LSCollider2D CreateCollider(ColliderType2D type) =>
        type switch
        {
            ColliderType2D.Circle => new LSCircleCollider2D(Fixed64.One),
            ColliderType2D.AABox => new LSAABBoxCollider2D(new Vector2d((Fixed64)2, (Fixed64)2)),
            ColliderType2D.Capsule => new LSCapsuleCollider2D(Fixed64.Half, (Fixed64)2),
            ColliderType2D.ConvexPolygon => new LSPolygonCollider2D(
                new Vector2d(-Fixed64.One, -Fixed64.One),
                new Vector2d(Fixed64.One, -Fixed64.One),
                new Vector2d(Fixed64.One, Fixed64.One),
                new Vector2d(-Fixed64.One, Fixed64.One)),
            _ => throw new Xunit.Sdk.XunitException("Unsupported test collider type.")
        };

    private static LSPolygonCollider2D CreateDiamondPolygon() =>
        new(
            new Vector2d(Fixed64.Zero, -Fixed64.One),
            new Vector2d(Fixed64.One, Fixed64.Zero),
            new Vector2d(Fixed64.Zero, Fixed64.One),
            new Vector2d(-Fixed64.One, Fixed64.Zero));
}
