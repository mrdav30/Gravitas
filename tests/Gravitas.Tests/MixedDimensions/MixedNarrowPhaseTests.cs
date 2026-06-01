using FixedMathSharp;
using FluentAssertions;
using Gravitas.Colliders;
using Gravitas.Support;
using Gravitas.Tests.Support;
using GridForge.Configuration;
using Xunit;

namespace Gravitas.Tests.MixedDimensions;

public sealed class MixedNarrowPhaseTests
{
    [Fact]
    public void SphereCircleSlab_WithPlanarOverlap_ShouldReportDeterministicContact()
    {
        using GravitasWorldContext context = CreateMixedContext();
        ScenarioBody<LSSphereCollider> sphere = CreateSphere3D(
            context,
            new Vector3d(Fixed64.Fraction(3, 4), Fixed64.Zero, Fixed64.Zero));
        StiffBody2D circle = CreateBody2D(context, new LSCircleCollider2D(Fixed64.Half), Vector2d.Zero);

        bool collided = CollisionDetectionMixed.TryCollide(sphere.Collider, circle.Collider, out MixedContact contact);

        collided.Should().BeTrue();
        contact.HasContact.Should().BeTrue();
        contact.Normal3DTo2D.Should().Be(-Vector3d.Right);
        contact.Depth.Should().Be(Fixed64.Fraction(1, 4));
        contact.Point3D.Should().Be(new Vector3d(Fixed64.Fraction(1, 4), Fixed64.Zero, Fixed64.Zero));
        contact.Point2D.Should().Be(new Vector3d(Fixed64.Half, Fixed64.Zero, Fixed64.Zero));
    }

    [Fact]
    public void SphereCircleSlab_WithSeparatedYSlab_ShouldNotCollide()
    {
        using GravitasWorldContext context = CreateMixedContext();
        ScenarioBody<LSSphereCollider> sphere = CreateSphere3D(
            context,
            new Vector3d(Fixed64.Zero, (Fixed64)2, Fixed64.Zero));
        StiffBody2D circle = CreateBody2D(context, new LSCircleCollider2D(Fixed64.Half), Vector2d.Zero);

        CollisionDetectionMixed.TryCollide(sphere.Collider, circle.Collider, out MixedContact contact).Should().BeFalse();
        contact.HasContact.Should().BeFalse();
    }

    [Fact]
    public void SphereAABoxSlab_WithTouchingFace_ShouldReportZeroDepthContact()
    {
        using GravitasWorldContext context = CreateMixedContext();
        ScenarioBody<LSSphereCollider> sphere = CreateSphere3D(
            context,
            new Vector3d(Fixed64.Fraction(3, 2), Fixed64.Zero, Fixed64.Zero));
        StiffBody2D box = CreateBody2D(context, new LSAABBoxCollider2D(new Vector2d((Fixed64)2, (Fixed64)2)), Vector2d.Zero);

        bool collided = CollisionDetectionMixed.TryCollide(sphere.Collider, box.Collider, out MixedContact contact);

        collided.Should().BeTrue();
        contact.Depth.Should().Be(Fixed64.Zero);
        contact.Normal3DTo2D.Should().Be(-Vector3d.Right);
        contact.Point3D.Should().Be(new Vector3d(Fixed64.One, Fixed64.Zero, Fixed64.Zero));
        contact.Point2D.Should().Be(new Vector3d(Fixed64.One, Fixed64.Zero, Fixed64.Zero));
    }

    [Fact]
    public void SphereConvexPolygonSlab_WithCornerOverlap_ShouldReportContact()
    {
        using GravitasWorldContext context = CreateMixedContext();
        ScenarioBody<LSSphereCollider> sphere = CreateSphere3D(
            context,
            new Vector3d((Fixed64)2, Fixed64.Zero, (Fixed64)2));
        var polygon = new LSPolygonCollider2D(
            new Vector2d(-Fixed64.One, -Fixed64.One),
            new Vector2d(Fixed64.One, -Fixed64.One),
            new Vector2d(Fixed64.One, Fixed64.One),
            new Vector2d(-Fixed64.One, Fixed64.One));
        StiffBody2D polygonBody = CreateBody2D(context, polygon, Vector2d.Zero);

        bool collided = CollisionDetectionMixed.TryCollide(sphere.Collider, polygonBody.Collider, out MixedContact contact);

        collided.Should().BeFalse();

        ScenarioBody<LSSphereCollider> overlappingSphere = CreateSphere3D(
            context,
            new Vector3d(Fixed64.Fraction(13, 10), Fixed64.Zero, Fixed64.Fraction(13, 10)));

        CollisionDetectionMixed.TryCollide(overlappingSphere.Collider, polygonBody.Collider, out contact).Should().BeTrue();
        contact.Depth.Should().BeGreaterThan(Fixed64.Zero);
        contact.Point2D.x.Should().Be(Fixed64.One);
        contact.Point2D.z.Should().Be(Fixed64.One);
    }

    private static GravitasWorldContext CreateMixedContext()
    {
        GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        context.Settings.RuntimeMode = PhysicsRuntimeMode.Mixed;
        context.World.TryAddGrid(
            new GridConfiguration(
                new Vector3d((Fixed64)(-8), (Fixed64)(-4), (Fixed64)(-8)),
                new Vector3d((Fixed64)8, (Fixed64)4, (Fixed64)8)),
            out _).Should().BeTrue();
        return context;
    }

    private static ScenarioBody<LSSphereCollider> CreateSphere3D(GravitasWorldContext context, Vector3d position)
    {
        var agent = new TestMatterAgent(context, new FixedTransform(position, FixedQuaternion.Identity, Vector3d.One));
        var collider = new LSSphereCollider();
        var body = new StiffBody(agent, collider)
        {
            Mass = Fixed64.One
        };
        body.Initialize(position, FixedQuaternion.Identity);
        return new ScenarioBody<LSSphereCollider>(body, collider);
    }

    private static StiffBody2D CreateBody2D(GravitasWorldContext context, LSCollider2D collider, Vector2d position)
    {
        var agent = new TestMatterAgent(
            context,
            new FixedTransform(new Vector3d(position.x, Fixed64.Zero, position.y), FixedQuaternion.Identity, Vector3d.One));
        var body = new StiffBody2D(agent, collider)
        {
            Mass = Fixed64.One
        };
        body.Initialize(position);
        return body;
    }
}
