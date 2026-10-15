using FixedMathSharp;
using FluentAssertions;
using Gravitas.Colliders;
using Gravitas.CollisionHandling;
using Gravitas.Materials;
using Gravitas.Tests.Support;
using Xunit;

namespace Gravitas.Tests.CollisionHandlingTests;

public sealed class UntouchedCollisionPairTests
{
    [Theory]
    [InlineData(0, false, false)]
    [InlineData(0, true, false)]
    [InlineData(1, false, false)]
    [InlineData(1, true, false)]
    [InlineData(2, false, false)]
    [InlineData(2, true, false)]
    [InlineData(0, false, true)]
    [InlineData(1, false, true)]
    [InlineData(2, true, true)]
    public void LeavingSharedPartitions_ShouldExitOnTheNextPassWithoutStaleContact(
        int targetKind, bool lateOnly, bool precheck)
    {
        using PhysicsScenarioBuilder scene = PhysicsScenarioBuilder.Create();
        scene.Context.Environment.Gravity = Fixed64.Zero;
        var target = new LSSphereCollider { Material = PhysicsMaterial.Frictionless, IsTrigger = targetKind == 2 };
        if (targetKind == 0)
            scene.CreateBody(target, Vector3d.Zero, FixedQuaternion.Identity, immovable: true);
        else
            scene.InitializeStaticCollider(target, Vector3d.Zero);
        SolidBody mover = scene.CreateSphere(Vector3d.Right * Fixed64.FromFraction(3, 4),
            preventAngularForces: true).Body;
        mover.UseManualGrounding();
        mover.SleepEnabled = false;
        mover.Collider.Material = PhysicsMaterial.Frictionless;
        int enters = 0, stays = 0, exits = 0;
        if (target.IsTrigger)
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
        Step(scene.Context, lateOnly);
        enters.Should().Be(1);
        CollisionPair pair = scene.Context.Physics.GetCollisionPair(target.Id, mover.Collider.Id)!;
        pair.Manifold.HasContact.Should().BeTrue();
        int previousStays = stays;
        mover.SetPosition(Vector3d.Right * (Fixed64)8);
        if (precheck)
        {
            mover.Collider.Simulate();
            CollisionDetection.DoCollisionCheck(pair).Should().BeFalse();
        }

        Step(scene.Context, lateOnly);

        exits.Should().Be(1);
        stays.Should().Be(previousStays);
        pair.Manifold.HasContact.Should().BeFalse();
        pair.Active.Should().BeFalse();
        (target.CollisionPairCount + target.CollisionPairHolderCount).Should().Be(0);
        Step(scene.Context, lateOnly);
        exits.Should().Be(1);
        stays.Should().Be(previousStays);
    }

    private static void Step(GravitasWorldContext context, bool lateOnly = false)
    {
        if (!lateOnly) context.Simulate();
        context.LateSimulate();
    }
}
