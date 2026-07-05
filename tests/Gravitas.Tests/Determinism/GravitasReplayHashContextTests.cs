using Chronicler;
using FixedMathSharp;
using FluentAssertions;
using Gravitas.Colliders;
using Gravitas.Support;
using Gravitas.Tests.Support;
using GridForge.Configuration;
using Xunit;

namespace Gravitas.Tests.Determinism;

public sealed class GravitasReplayHashContextTests
{
    [Fact]
    public void ComputeReplayHash_ShouldMatchForRepeatedEquivalent3DRuns()
    {
        ChronicleHash[] first = Run3DTrace();
        ChronicleHash[] second = Run3DTrace();

        second.Should().Equal(first);
    }

    [Fact]
    public void ComputeReplayHash_ShouldChangeWhenAuthoritativeBodyStateChanges()
    {
        using PhysicsScenarioBuilder scenario = Create3DScenario();
        ScenarioBody<LSSphereCollider> body = scenario.CreateSphere(Vector3d.Zero);

        ChronicleHash before = scenario.Context.ComputeReplayHash();

        body.Body.AddForce(Vector3d.Right);
        scenario.Context.LateSimulate();

        scenario.Context.ComputeReplayHash().Should().NotBe(before);
    }

    [Fact]
    public void ComputeReplayHash_ShouldIgnoreQueryCacheMutationInAuthoritativeMode()
    {
        using PhysicsScenarioBuilder first = Create3DScenario();
        first.CreateSphere(Vector3d.Zero);
        ChronicleHash beforeQuery = first.Context.ComputeReplayHash();

        using PhysicsScenarioBuilder second = Create3DScenario();
        second.CreateSphere(Vector3d.Zero);
        second.Context.Query3D.Raycast(
            Vector3d.Left,
            Vector3d.Right,
            (Fixed64)8,
            out _,
            PhysicsLayerMask.All);

        second.Context.ComputeReplayHash(GravitasReplayHashMode.Authoritative).Should().Be(beforeQuery);
    }

    [Fact]
    public void ComputeReplayHash_Authoritative_ShouldIgnoreDeleted3DColliderIdChurn()
    {
        using GravitasWorldContext compact = Create3DStaticColliderHashContext(churnBeforeLive: 0, churnAfterLive: 0);
        using GravitasWorldContext churned = Create3DStaticColliderHashContext(churnBeforeLive: 8, churnAfterLive: 8);

        churned.ComputeReplayHash().Should().Be(compact.ComputeReplayHash());
    }

    [Fact]
    public void ComputeReplayHash_Authoritative_ShouldIgnoreDeleted3DFreeListOrderChurn()
    {
        using GravitasWorldContext compact = Create3DStaticColliderHashContext(churnBeforeLive: 0, churnAfterLive: 0);
        using GravitasWorldContext churned = Create3DStaticColliderHashContextWithBatchDeletedChurn();

        churned.ComputeReplayHash().Should().Be(compact.ComputeReplayHash());
    }

    [Fact]
    public void ComputeReplayHash_Authoritative_ShouldIgnoreDeleted2DColliderIdChurn()
    {
        using GravitasWorldContext compact = Create2DStaticColliderHashContext(churnBeforeLive: 0, churnAfterLive: 0);
        using GravitasWorldContext churned = Create2DStaticColliderHashContext(churnBeforeLive: 8, churnAfterLive: 8);

        churned.ComputeReplayHash().Should().Be(compact.ComputeReplayHash());
    }

    [Fact]
    public void ComputeReplayHash_Authoritative_ShouldIgnoreDeleted2DFreeListOrderChurn()
    {
        using GravitasWorldContext compact = Create2DStaticColliderHashContext(churnBeforeLive: 0, churnAfterLive: 0);
        using GravitasWorldContext churned = Create2DStaticColliderHashContextWithBatchDeletedChurn();

        churned.ComputeReplayHash().Should().Be(compact.ComputeReplayHash());
    }

    [Fact]
    public void ComputeReplayHash_Authoritative_ShouldIgnoreDeletedMixedColliderIdChurn()
    {
        using GravitasWorldContext compact = CreateMixedStaticColliderHashContext(churnBeforeLive: 0, churnAfterLive: 0);
        using GravitasWorldContext churned = CreateMixedStaticColliderHashContext(churnBeforeLive: 8, churnAfterLive: 8);

        churned.ComputeReplayHash().Should().Be(compact.ComputeReplayHash());
    }

    [Fact]
    public void ComputeReplayHash_Authoritative_ShouldStillChangeWhenLiveColliderIdOrderChanges()
    {
        using GravitasWorldContext first = CreateReplayHashContext(PhysicsRuntimeMode.ThreeD);
        _ = CreateBodylessSphere3D(first, Vector3d.Zero);
        _ = CreateBodylessSphere3D(first, new Vector3d((Fixed64)2, Fixed64.Zero, Fixed64.Zero));

        using GravitasWorldContext second = CreateReplayHashContext(PhysicsRuntimeMode.ThreeD);
        _ = CreateBodylessSphere3D(second, new Vector3d((Fixed64)2, Fixed64.Zero, Fixed64.Zero));
        _ = CreateBodylessSphere3D(second, Vector3d.Zero);

        second.ComputeReplayHash().Should().NotBe(first.ComputeReplayHash());
    }

    [Fact]
    public void ComputeReplayHash_Authoritative_ShouldNotAllocateAfterColliderIdChurn()
    {
        using GravitasWorldContext context3D = Create3DStaticColliderHashContext(churnBeforeLive: 16, churnAfterLive: 16);
        using GravitasWorldContext context2D = Create2DStaticColliderHashContext(churnBeforeLive: 16, churnAfterLive: 16);
        using GravitasWorldContext mixedContext = CreateMixedStaticColliderHashContext(churnBeforeLive: 16, churnAfterLive: 16);

        long allocatedBytes = AllocationTestHelper.MeasureSteadyState(
            () =>
            {
                _ = context3D.ComputeReplayHash();
                _ = context2D.ComputeReplayHash();
                _ = mixedContext.ComputeReplayHash();
            },
            warmupIterations: 16,
            stabilizationIterations: 8,
            measurementIterations: 16);

        allocatedBytes.Should().Be(0);
    }

    private static ChronicleHash[] Run3DTrace()
    {
        using PhysicsScenarioBuilder scenario = Create3DScenario();
        ScenarioBody<LSSphereCollider> body = scenario.CreateSphere(Vector3d.Zero);
        body.Body.AddForce(new Vector3d((Fixed64)4, Fixed64.Zero, Fixed64.Zero));

        var hashes = new ChronicleHash[8];
        for (int frame = 0; frame < hashes.Length; frame++)
        {
            scenario.Context.LateSimulate();
            hashes[frame] = scenario.Context.ComputeReplayHash();
        }

        return hashes;
    }

    private static PhysicsScenarioBuilder Create3DScenario()
    {
        PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        scenario.Context.SetFrameRate(8);
        scenario.Context.Settings.RuntimeMode = PhysicsRuntimeMode.ThreeD;
        return scenario;
    }

    private static GravitasWorldContext Create3DStaticColliderHashContext(
        int churnBeforeLive,
        int churnAfterLive)
    {
        GravitasWorldContext context = CreateReplayHashContext(PhysicsRuntimeMode.ThreeD);
        ChurnDeleted3DColliders(context, churnBeforeLive);
        _ = CreateBodylessSphere3D(context, Vector3d.Zero);
        _ = CreateBodylessSphere3D(context, new Vector3d((Fixed64)2, Fixed64.Zero, Fixed64.Zero));
        ChurnDeleted3DColliders(context, churnAfterLive);
        return context;
    }

    private static GravitasWorldContext Create3DStaticColliderHashContextWithBatchDeletedChurn()
    {
        GravitasWorldContext context = CreateReplayHashContext(PhysicsRuntimeMode.ThreeD);
        LSSphereCollider firstDeleted = CreateBodylessSphere3D(
            context,
            new Vector3d((Fixed64)4, Fixed64.Zero, (Fixed64)8));
        LSSphereCollider secondDeleted = CreateBodylessSphere3D(
            context,
            new Vector3d((Fixed64)5, Fixed64.Zero, (Fixed64)8));
        firstDeleted.Deactivate();
        secondDeleted.Deactivate();

        _ = CreateBodylessSphere3D(context, Vector3d.Zero);
        _ = CreateBodylessSphere3D(context, new Vector3d((Fixed64)2, Fixed64.Zero, Fixed64.Zero));
        return context;
    }

    private static GravitasWorldContext Create2DStaticColliderHashContext(
        int churnBeforeLive,
        int churnAfterLive)
    {
        GravitasWorldContext context = Physics2DTestWorld.CreateContext(frameRate: 8);
        ChurnDeleted2DColliders(context, churnBeforeLive);
        _ = CreateBodylessCircle2D(context, Vector2d.Zero);
        _ = CreateBodylessCircle2D(context, new Vector2d((Fixed64)2, Fixed64.Zero));
        ChurnDeleted2DColliders(context, churnAfterLive);
        return context;
    }

    private static GravitasWorldContext Create2DStaticColliderHashContextWithBatchDeletedChurn()
    {
        GravitasWorldContext context = Physics2DTestWorld.CreateContext(frameRate: 8);
        LSCircleCollider2D firstDeleted = CreateBodylessCircle2D(
            context,
            new Vector2d((Fixed64)4, (Fixed64)8));
        LSCircleCollider2D secondDeleted = CreateBodylessCircle2D(
            context,
            new Vector2d((Fixed64)5, (Fixed64)8));
        firstDeleted.Deactivate();
        secondDeleted.Deactivate();

        _ = CreateBodylessCircle2D(context, Vector2d.Zero);
        _ = CreateBodylessCircle2D(context, new Vector2d((Fixed64)2, Fixed64.Zero));
        return context;
    }

    private static GravitasWorldContext CreateMixedStaticColliderHashContext(
        int churnBeforeLive,
        int churnAfterLive)
    {
        GravitasWorldContext context = CreateReplayHashContext(PhysicsRuntimeMode.Mixed);
        ChurnDeleted3DColliders(context, churnBeforeLive);
        ChurnDeleted2DColliders(context, churnBeforeLive);
        _ = CreateBodylessSphere3D(context, Vector3d.Zero);
        _ = CreateBodylessCircle2D(context, new Vector2d((Fixed64)2, Fixed64.Zero));
        ChurnDeleted3DColliders(context, churnAfterLive);
        ChurnDeleted2DColliders(context, churnAfterLive);
        return context;
    }

    private static GravitasWorldContext CreateReplayHashContext(PhysicsRuntimeMode runtimeMode)
    {
        GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        context.SetFrameRate(8);
        context.Settings.RuntimeMode = runtimeMode;
        context.World.TryAddGrid(
            new GridConfiguration(
                new Vector3d((Fixed64)(-32), (Fixed64)(-8), (Fixed64)(-32)),
                new Vector3d((Fixed64)32, (Fixed64)8, (Fixed64)32)),
            out _).Should().BeTrue();
        return context;
    }

    private static LSSphereCollider CreateBodylessSphere3D(GravitasWorldContext context, Vector3d position)
    {
        var collider = new LSSphereCollider();
        var transform = new FixedTransform(position, FixedQuaternion.Identity, Vector3d.One);
        collider.InitializeWithNoBody(new TestMatterAgent(context, transform));
        return collider;
    }

    private static LSCircleCollider2D CreateBodylessCircle2D(GravitasWorldContext context, Vector2d position)
    {
        var collider = new LSCircleCollider2D(Fixed64.Half);
        var transform = new FixedTransform(
            new Vector3d(position.X, Fixed64.Zero, position.Y),
            FixedQuaternion.Identity,
            Vector3d.One);
        collider.InitializeWithNoBody(new TestMatterAgent(context, transform));
        return collider;
    }

    private static void ChurnDeleted3DColliders(GravitasWorldContext context, int count)
    {
        for (int i = 0; i < count; i++)
        {
            LSSphereCollider collider = CreateBodylessSphere3D(
                context,
                new Vector3d((Fixed64)(4 + i), Fixed64.Zero, (Fixed64)8));
            collider.Deactivate();
        }
    }

    private static void ChurnDeleted2DColliders(GravitasWorldContext context, int count)
    {
        for (int i = 0; i < count; i++)
        {
            LSCircleCollider2D collider = CreateBodylessCircle2D(
                context,
                new Vector2d((Fixed64)(4 + i), (Fixed64)8));
            collider.Deactivate();
        }
    }
}
