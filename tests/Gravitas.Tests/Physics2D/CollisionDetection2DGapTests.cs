using FixedMathSharp;
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
        manifold[0].PointA.Y.Should().Be(Fixed64.One);
        manifold[0].PointB.Y.Should().Be(Fixed64.One);
        manifold[1].PointA.Y.Should().Be(-Fixed64.One);
        manifold[1].PointB.Y.Should().Be(-Fixed64.One);
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
            : new Vector2d(Fixed64.Half, Fixed64.FromFraction(3, 4)));
        contact.PointB.Should().Be(capsuleFirst
            ? new Vector2d(Fixed64.Half, Fixed64.FromFraction(3, 4))
            : new Vector2d(Fixed64.Zero, Fixed64.FromFraction(3, 2)));
        contact.MaterialA.Should().Be(capsuleFirst ? PhysicsMaterial.Bouncy : PhysicsMaterial.Frictionless);
        contact.MaterialB.Should().Be(capsuleFirst ? PhysicsMaterial.Frictionless : PhysicsMaterial.Bouncy);
    }

    [Theory]
    [InlineData(1)]
    [InlineData(-1)]
    public void RotatedContainedPolygon_ShouldUseContainingBoxFaceManifold(int verticalSign)
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

        Fixed64 diagonal = FixedMath.Sin(FixedMath.DegToRad((Fixed64)45));
        direct.Should().BeTrue();
        collided.Should().BeTrue();
        contact.Normal.Should().Be(sign * Vector2d.Forward);
        contact.Depth.Should().Be(Fixed64.Half + diagonal);
        contact.PointA.Should().Be(new Vector2d(Fixed64.One, sign));
        contact.PointB.Should().Be(new Vector2d(-Fixed64.Half, sign * (Fixed64.Half - diagonal)));
        manifold.Count.Should().Be(2);
        manifold[0].ContactId.Should().BeLessThan(manifold[1].ContactId);
        manifold[0].Normal.Should().Be(sign * Vector2d.Forward);
        manifold[0].Depth.Should().Be(Fixed64.Half + diagonal);
        manifold[0].PointA.Should().Be(new Vector2d(-Fixed64.Half, sign));
        manifold[0].PointB.Should().Be(new Vector2d(-Fixed64.Half, sign * (Fixed64.Half - diagonal)));
        manifold[1].Normal.Should().Be(sign * Vector2d.Forward);
        manifold[1].Depth.Should().Be(Fixed64.Half);
        manifold[1].PointA.Should().Be(new Vector2d(-Fixed64.Half + diagonal, sign));
        manifold[1].PointB.Should().Be(new Vector2d(-Fixed64.Half + diagonal, sign * Fixed64.Half));
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
        forward[0].Normal.Should().Be(-Vector2d.Right);
        forward[0].Depth.Should().Be(Fixed64.FromFraction(5, 2));
        forward[0].PointA.Should().Be(new Vector2d((Fixed64)10, Fixed64.One));
        forward[0].PointB.Should().Be(new Vector2d(Fixed64.FromFraction(25, 2), Fixed64.One));
        forward[1].Normal.Should().Be(-Vector2d.Right);
        forward[1].Depth.Should().Be(Fixed64.FromFraction(5, 2));
        Fixed64 clippedBottom = -Fixed64.One - Fixed64.MinIncrement;
        forward[1].PointA.Should().Be(new Vector2d((Fixed64)10, clippedBottom));
        forward[1].PointB.Should().Be(new Vector2d(Fixed64.FromFraction(25, 2), clippedBottom));
        reverse[0].ContactId.Should().BeLessThan(reverse[1].ContactId);
        reverse[0].Normal.Should().Be(Vector2d.Right);
        reverse[0].Depth.Should().Be(Fixed64.FromFraction(5, 2));
        reverse[0].PointA.Should().Be(new Vector2d(Fixed64.FromFraction(25, 2), -Fixed64.One));
        reverse[0].PointB.Should().Be(new Vector2d((Fixed64)10, -Fixed64.One));
        reverse[1].Normal.Should().Be(Vector2d.Right);
        reverse[1].Depth.Should().Be(Fixed64.FromFraction(5, 2));
        reverse[1].PointA.Should().Be(new Vector2d(Fixed64.FromFraction(25, 2), Fixed64.One));
        reverse[1].PointB.Should().Be(new Vector2d((Fixed64)10, Fixed64.One));
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
        Vector2d intersection = (Fixed64)offset * (Fixed64.One - Fixed64.MinIncrement) * Vector2d.Right;
        contact.Normal.Should().Be(normal);
        contact.Depth.Should().Be(Fixed64.One);
        contact.PointA.Should().Be(intersection + normal * Fixed64.Half);
        contact.PointB.Should().Be(intersection - normal * Fixed64.Half);
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
            Mass = Fixed64.One,
            FreezeAxes = BodyFreezeAxes2D.Position
        };
        body.Initialize(position, rotation);
        return body;
    }
}
