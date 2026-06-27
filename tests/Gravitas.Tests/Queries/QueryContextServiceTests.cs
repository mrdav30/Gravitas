using FixedMathSharp;
using FluentAssertions;
using Gravitas.Colliders;
using Gravitas.Queries;
using Gravitas.Support;
using Gravitas.Tests.Support;
using GridForge.Configuration;
using SwiftCollections;
using Xunit;

namespace Gravitas.Tests.Queries;

public sealed class QueryContextServiceTests
{
    [Fact]
    public void WorldContext_ShouldExposeConsolidatedQueryServices()
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();

        context.Query2D.Should().NotBeNull();
        context.Query3D.Should().NotBeNull();
    }

    [Fact]
    public void ConsolidatedQueryServices_ShouldPreserve2DAnd3DQueryEntryPoints()
    {
        using GravitasWorldContext context3D = GravitasWorldContext.CreateOwned();
        EnsureGrid(context3D);
        LSSphereCollider sphere = CreateDynamicSphere(context3D, Vector3d.Zero);
        var hits3D = new SwiftList<Physics3DHit>();

        int count3D = context3D.Query3D.RaycastAll(
            new Vector3d((Fixed64)(-2), Fixed64.Zero, Fixed64.Zero),
            new Vector3d((Fixed64)2, Fixed64.Zero, Fixed64.Zero),
            PhysicsLayerMask.All,
            hits3D);

        using GravitasWorldContext context2D = Physics2DTestWorld.CreateContext();
        SolidBody2D circle = CreateStaticCircle2D(context2D, Vector2d.Zero);
        var hits2D = new SwiftList<Physics2DHit>();

        int count2D = context2D.Query2D.RaycastAll(
            new Vector2d((Fixed64)(-2), Fixed64.Zero),
            new Vector2d((Fixed64)2, Fixed64.Zero),
            hits2D);

        count3D.Should().Be(1);
        hits3D[0].Collider.Should().BeSameAs(sphere);
        count2D.Should().Be(1);
        hits2D[0].Collider.Should().BeSameAs(circle.Collider);
    }

    private static LSSphereCollider CreateDynamicSphere(GravitasWorldContext context, Vector3d position)
    {
        var agent = new TestMatterAgent(context);
        var collider = new LSSphereCollider();
        var body = new SolidBody(agent, collider)
        {
            Mass = Fixed64.One
        };

        body.Initialize(position, FixedQuaternion.Identity);
        return collider;
    }

    private static SolidBody2D CreateStaticCircle2D(GravitasWorldContext context, Vector2d position)
    {
        var transform = new FixedTransform(
            new Vector3d(position.X, Fixed64.Zero, position.Y),
            FixedQuaternion.Identity,
            Vector3d.One);
        var agent = new TestMatterAgent(context, transform);
        var body = new SolidBody2D(agent, new LSCircleCollider2D(Fixed64.Half))
        {
            FreezeAxes = BodyFreezeAxes2D.Position,
            Mass = Fixed64.One
        };

        body.Initialize(position);
        return body;
    }

    private static void EnsureGrid(GravitasWorldContext context)
    {
        GridConfiguration configuration = new(
            new Vector3d(-4, -4, -4),
            new Vector3d(6, 6, 6));

        context.World.TryAddGrid(configuration, out _).Should().BeTrue();
    }
}
