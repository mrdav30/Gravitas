using FixedMathSharp;
using FluentAssertions;
using Gravitas.Colliders;
using Gravitas.Tests.Support;
using Xunit;

namespace Gravitas.Tests.CollisionHandlingTests;

public sealed class UntouchedPairDimensionParityTests
{
    [Theory]
    [InlineData(false, false)]
    [InlineData(false, true)]
    [InlineData(true, false)]
    [InlineData(true, true)]
    public void Pure2D_FarSeparatedPair_ShouldExitOnTheNextCollisionPass(bool fullLoop, bool trigger)
    {
        using PhysicsScenarioBuilder scene = CreateScene(PhysicsRuntimeMode.TwoD);
        GravitasWorldContext context = scene.Context;
        LSCircleCollider2D target = CreateTarget(context, trigger);
        var moverCollider = new LSCircleCollider2D(Fixed64.Half);
        var mover = new SolidBody2D(new TestMatterAgent(context,
            new FixedTransform(Vector3d.Zero, FixedQuaternion.Identity, Vector3d.One)), moverCollider)
            { Mass = Fixed64.One };
        mover.Initialize(new Vector2d(Fixed64.FromFraction(3, 4), Fixed64.Zero));
        mover.UseManualGrounding();
        mover.SleepEnabled = false;
        int enters = 0, stays = 0, exits = 0;
        if (trigger)
        {
            target.OnTriggerEnter += _ => enters++;
            target.OnTriggerStay += _ => stays++;
            target.OnTriggerExit += _ => exits++;
        }
        else
        {
            target.OnContactEnter += _ => enters++;
            target.OnContact += _ => stays++;
            target.OnContactExit += _ => exits++;
        }

        Step(context, fullLoop);
        enters.Should().Be(1);
        stays.Should().Be(1);
        mover.SetPosition(Vector2d.Right * (Fixed64)8);

        Step(context, fullLoop);

        exits.Should().Be(1);
        stays.Should().Be(1);
        (target.CollisionPairCount + target.CollisionPairHolderCount).Should().Be(0);
        (moverCollider.CollisionPairCount + moverCollider.CollisionPairHolderCount).Should().Be(0);
        Step(context, fullLoop);
        exits.Should().Be(1);
    }

    [Theory]
    [InlineData(false, false, false)]
    [InlineData(false, true, false)]
    [InlineData(true, false, false)]
    [InlineData(true, true, false)]
    [InlineData(false, false, true)]
    [InlineData(true, true, true)]
    public void Mixed_FarSeparatedPair_ShouldExitOnTheNextCollisionPass(
        bool fullLoop, bool trigger, bool disablePureServices)
    {
        using PhysicsScenarioBuilder scene = CreateScene(PhysicsRuntimeMode.Mixed);
        GravitasWorldContext context = scene.Context;
        if (disablePureServices)
        {
            context.Physics.SimulatePhysics = false;
            context.Physics2D.SimulatePhysics = false;
        }
        LSCircleCollider2D target = CreateTarget(context, trigger);
        SolidBody mover = scene.CreateSphere(Vector3d.Right * Fixed64.FromFraction(3, 4),
            preventAngularForces: true).Body;
        mover.UseManualGrounding();
        mover.SleepEnabled = false;
        int enters = 0, stays = 0, exits = 0;
        if (trigger)
        {
            target.OnMixedTriggerEnter += _ => enters++;
            target.OnMixedTriggerStay += _ => stays++;
            target.OnMixedTriggerExit += _ => exits++;
        }
        else
        {
            target.OnMixedContactEnter += _ => enters++;
            target.OnMixedContact += _ => stays++;
            target.OnMixedContactExit += _ => exits++;
        }

        Step(context, fullLoop);
        enters.Should().Be(1);
        stays.Should().Be(1);
        context.MixedCollisions.ActivePairCount.Should().Be(1);
        mover.SetPosition(Vector3d.Right * (Fixed64)8);
        mover.Collider.Simulate();

        Step(context, fullLoop);

        exits.Should().Be(1);
        stays.Should().Be(1);
        context.MixedCollisions.ActivePairCount.Should().Be(0);
        Step(context, fullLoop);
        exits.Should().Be(1);
    }

    private static PhysicsScenarioBuilder CreateScene(PhysicsRuntimeMode mode)
    {
        PhysicsScenarioBuilder scene = PhysicsScenarioBuilder.Create();
        scene.Context.Settings.RuntimeMode = mode;
        scene.Context.Environment.Gravity = Fixed64.Zero;
        scene.Context.Environment.AirDensity = Fixed64.Zero;
        scene.Context.Environment.DampingFactor = Fixed64.Zero;
        scene.Context.Environment.MinSpeed = Fixed64.Zero;
        return scene;
    }

    private static LSCircleCollider2D CreateTarget(GravitasWorldContext context, bool trigger)
    {
        var collider = new LSCircleCollider2D(Fixed64.Half) { IsTrigger = trigger };
        collider.InitializeWithNoBody(new TestMatterAgent(context,
            new FixedTransform(Vector3d.Zero, FixedQuaternion.Identity, Vector3d.One)));
        return collider;
    }

    private static void Step(GravitasWorldContext context, bool fullLoop)
    {
        if (fullLoop) context.Simulate();
        context.LateSimulate();
    }
}
