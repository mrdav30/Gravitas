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
    [Theory]
    [InlineData(ColliderType2D.Circle, ColliderType2D.Circle)]
    [InlineData(ColliderType2D.Circle, ColliderType2D.AABox)]
    [InlineData(ColliderType2D.AABox, ColliderType2D.AABox)]
    [InlineData(ColliderType2D.Circle, ColliderType2D.ConvexPolygon)]
    [InlineData(ColliderType2D.AABox, ColliderType2D.ConvexPolygon)]
    [InlineData(ColliderType2D.ConvexPolygon, ColliderType2D.ConvexPolygon)]
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

    private static SolidBody2D CreateBody(GravitasWorldContext context, LSCollider2D collider, Vector2d position)
    {
        var transform = new FixedTransform(new Vector3d(position.X, Fixed64.Zero, position.Y), FixedQuaternion.Identity, Vector3d.One);
        var agent = new TestMatterAgent(context, transform);
        var body = new SolidBody2D(agent, collider)
        {
            Mass = Fixed64.One,
            FreezeAxes = BodyFreezeAxes2D.Position
        };
        body.Initialize(position);
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
            ColliderType2D.ConvexPolygon => new LSPolygonCollider2D(
                new Vector2d(-Fixed64.One, -Fixed64.One),
                new Vector2d(Fixed64.One, -Fixed64.One),
                new Vector2d(Fixed64.One, Fixed64.One),
                new Vector2d(-Fixed64.One, Fixed64.One)),
            _ => throw new Xunit.Sdk.XunitException("Unsupported test collider type.")
        };
}
