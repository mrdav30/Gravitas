using FixedMathSharp;
using FluentAssertions;
using Gravitas.Colliders;
using Gravitas.Materials;
using Gravitas.Tests.Serialization;
using Gravitas.Tests.Support;
using Xunit;

namespace Gravitas.Tests.CollisionHandlingTests;

public sealed class SleepingContactInvalidationTests
{
    public static TheoryData<bool, bool, int> LostSupports
    {
        get
        {
            var cases = new TheoryData<bool, bool, int>();
            foreach (bool bodyless in new[] { false, true })
            foreach (bool automatic in new[] { false, true })
            foreach (int change in new[] { 0, 1, 2 })
                cases.Add(bodyless, automatic, change);
            return cases;
        }
    }

    [Theory]
    [MemberData(nameof(LostSupports))]
    public void LostSolidSupport_ShouldWakeTheSleeperAndResumeGravity(bool bodyless, bool automatic, int change)
    {
        using PhysicsScenarioBuilder scene = CreateScene();
        var floor = new LSCuboidCollider { Size = new Vector3d(8, 1, 8), Material = PhysicsMaterial.Frictionless };
        Vector3d floorPosition = Vector3d.Down * Fixed64.Half;
        if (bodyless) scene.InitializeStaticCollider(floor, floorPosition);
        else scene.CreateBody(floor, floorPosition, FixedQuaternion.Identity, immovable: true);
        var sphere = new LSSphereCollider { LocalOffset = Vector3d.Up * Fixed64.Half, Material = PhysicsMaterial.Frictionless };
        SolidBody body = scene.CreateBody(sphere, Vector3d.Zero, FixedQuaternion.Identity,
            preventAngularForces: true).Body;
        body.SleepEnabled = false;
        if (!automatic) body.UseManualGrounding();
        for (int frame = 0; frame < 16; frame++) Step(scene.Context);
        body.SleepEnabled = true;
        body.Sleep();
        Step(scene.Context);
        body.IsSleeping.Should().BeTrue();
        body.IsGrounded.Should().Be(automatic);
        scene.Context.Physics.GetCollisionPair(floor.Id, sphere.Id)!.Manifold.HasContact.Should().BeTrue();
        Vector3d supportedPosition = body.Position3d;

        if (change == 0)
        {
            if (floor.Body != null) floor.Body.SetPosition(floorPosition + Vector3d.Right * (Fixed64)12);
            else floor.Position = floorPosition + Vector3d.Right * (Fixed64)12;
            floor.Simulate();
        }
        else if (change == 1)
        {
            floor.Size = new Vector3d((Fixed64)8, Fixed64.FromFraction(1, 16), (Fixed64)8);
            floor.Simulate();
        }
        else if (floor.Body != null) floor.Body.Deactivate();
        else floor.Deactivate();

        Step(scene.Context);
        body.IsSleeping.Should().BeFalse();
        for (int frame = 0; frame < 7; frame++) Step(scene.Context);
        body.Position3d.Y.Should().BeLessThan(supportedPosition.Y);
        // The automatic swept probe can find the newly lowered surface and snap to it.
        if (automatic && change == 1)
        {
            body.IsGrounded.Should().BeTrue();
            body.Position3d.Y.Should().Be(floor.BoundsMax.Y);
        }
        else
        {
            body.LinearVelocity.Y.Should().BeLessThan(Fixed64.Zero);
            body.IsGrounded.Should().BeFalse();
        }
        body.GroundingMode.Should().Be(automatic ? GroundingMode.Automatic : GroundingMode.Manual);
    }

    [Fact]
    public void TriggerOnlySeparation_ShouldNotWakeAnUnrelatedSleeper()
    {
        using PhysicsScenarioBuilder scene = CreateScene();
        var trigger = new LSSphereCollider { IsTrigger = true };
        scene.InitializeStaticCollider(trigger, Vector3d.Zero);
        SolidBody body = scene.CreateSphere(Vector3d.Right * Fixed64.FromFraction(3, 4)).Body;
        body.UseManualGrounding();
        scene.Context.Environment.Gravity = Fixed64.Zero;
        Step(scene.Context);
        body.Sleep();
        int exits = 0;
        trigger.OnTriggerExit += _ => exits++;
        trigger.Position = Vector3d.Left * (Fixed64)8;
        trigger.Simulate();

        Step(scene.Context);

        body.IsSleeping.Should().BeTrue();
        exits.Should().Be(1);
    }

    [Theory]
    [InlineData(0)]
    [InlineData(1)]
    [InlineData(2)]
    public void AcceptedSolverPose_ShouldPreserveSleepButAnExplicitPoseShouldInvalidateItsNeighbor(int poseEdit)
    {
        using PhysicsScenarioBuilder scene = CreateScene();
        scene.Context.Environment.Gravity = Fixed64.Zero;
        SolidBody first = scene.CreateSphere(Vector3d.Zero).Body;
        SolidBody second = scene.CreateSphere(Vector3d.Right * Fixed64.FromFraction(3, 4)).Body;
        first.UseManualGrounding();
        second.UseManualGrounding();
        Step(scene.Context);
        first.Sleep();
        second.Sleep();
        Vector3d firstPosition = first.Position3d, secondPosition = second.Position3d;
        if (poseEdit == 1)
        {
            first.SetPosition(firstPosition + Vector3d.Right * Fixed64.FromFraction(1, 64));
            first.Sleep();
        }
        else if (poseEdit == 2)
        {
            first.SetPosition(first.Position3d);
            first.SetHeight(first.HeightPos);
            first.SetRotation(first.Rotation);
            first.UpdateRotation(first.Rotation, Fixed64.One);
        }

        Step(scene.Context);

        second.IsSleeping.Should().Be(poseEdit != 1);
        if (poseEdit != 1)
        {
            first.IsSleeping.Should().BeTrue();
            first.Position3d.Should().Be(firstPosition);
            second.Position3d.Should().Be(secondPosition);
        }
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void PhysicalKindOrMaterialChange_ShouldWakeAnExistingSleepingContact(bool trigger)
    {
        using PhysicsScenarioBuilder scene = CreateScene();
        scene.Context.Environment.Gravity = Fixed64.Zero;
        var target = new LSSphereCollider { Material = PhysicsMaterial.Frictionless };
        scene.InitializeStaticCollider(target, Vector3d.Zero);
        SolidBody body = scene.CreateSphere(Vector3d.Right * Fixed64.FromFraction(3, 4)).Body;
        body.UseManualGrounding();
        Step(scene.Context);
        body.Sleep();
        Step(scene.Context);
        target.Material = target.Material;
        Step(scene.Context);
        body.IsSleeping.Should().BeTrue();

        if (trigger) target.IsTrigger = true;
        else target.Material = PhysicsMaterial.Default;
        Step(scene.Context);

        body.IsSleeping.Should().BeFalse();
        if (trigger)
        {
            body.Sleep();
            target.IsTrigger = false;
            Step(scene.Context);
            body.IsSleeping.Should().BeFalse();
        }
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void IntegratedDynamicSeparation_ShouldWakeBeforeDiscardingTheOldContact(bool boundsMiss)
    {
        using PhysicsScenarioBuilder scene = CreateScene();
        scene.Context.Environment.MaxSpeed = (Fixed64)64;
        var floor = new LSCuboidCollider { Material = PhysicsMaterial.Frictionless };
        SolidBody support = scene.CreateBody(floor, Vector3d.Down * Fixed64.Half,
            FixedQuaternion.Identity).Body;
        support.FreezeAxes = BodyFreezeAxes3D.PositionY | BodyFreezeAxes3D.PositionZ | BodyFreezeAxes3D.Rotation;
        support.SleepEnabled = false;
        support.UseManualGrounding();
        SolidBody body = scene.CreateSphere(Vector3d.Up * Fixed64.Half, preventAngularForces: true).Body;
        body.Collider.Material = PhysicsMaterial.Frictionless;
        body.UseManualGrounding();
        body.SleepEnabled = false;
        for (int frame = 0; frame < 16; frame++) Step(scene.Context);
        var pair = scene.Context.Physics.GetCollisionPair(floor.Id, body.Collider.Id)!;
        pair.Manifold.HasContact.Should().BeTrue();
        body.SleepEnabled = true;
        body.Sleep();
        Fixed64 displacement = boundsMiss ? Fixed64.FromFraction(5, 4) : Fixed64.FromFraction(3, 4);
        support.AddLinearImpulse(Vector3d.Right * displacement * (Fixed64)scene.Context.FrameRate);

        Step(scene.Context);

        pair.LastFrame.Should().Be(scene.Context.FrameCount, "the shared voxel must visit the separating pair");
        pair.Manifold.HasContact.Should().BeFalse();
        body.IsSleeping.Should().BeFalse();
    }

    [Fact]
    public void SavingAStaticSupport_ShouldPreserveItsSleepingContact()
    {
        using PhysicsScenarioBuilder scene = CreateScene();
        scene.Context.Environment.Gravity = Fixed64.Zero;
        var target = new LSSphereCollider();
        scene.InitializeStaticCollider(target, Vector3d.Zero);
        SolidBody body = scene.CreateSphere(Vector3d.Right * Fixed64.FromFraction(3, 4)).Body;
        body.UseManualGrounding();
        Step(scene.Context);
        body.Sleep();
        Step(scene.Context);
        body.IsSleeping.Should().BeTrue();
        var before = scene.Context.ComputeReplayHash();

        GravitasSerializationHarness.Serialize(target, GravitasSerializationTransport.Json);
        target.Simulate();

        scene.Context.ComputeReplayHash().Should().Be(before);
        Step(scene.Context);
        body.IsSleeping.Should().BeTrue();
    }

    private static PhysicsScenarioBuilder CreateScene()
    {
        PhysicsScenarioBuilder scene = PhysicsScenarioBuilder.Create();
        scene.Context.SetFrameRate(16);
        scene.Context.Environment.Gravity = Fixed64.One;
        scene.Context.Environment.AirDensity = Fixed64.Zero;
        scene.Context.Environment.DampingFactor = Fixed64.Zero;
        scene.Context.Environment.MinSpeed = Fixed64.Zero;
        return scene;
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void BodylessContactRemoval_ShouldWakeTheSurvivingBodyInEitherRegistrationOrder(bool bodylessFirst)
    {
        using PhysicsScenarioBuilder scene = CreateScene();
        scene.Context.Environment.Gravity = Fixed64.Zero;
        var target = new LSSphereCollider();
        if (bodylessFirst) scene.InitializeStaticCollider(target, Vector3d.Zero);
        SolidBody body = scene.CreateSphere(Vector3d.Right * Fixed64.FromFraction(3, 4)).Body;
        body.UseManualGrounding();
        if (!bodylessFirst) scene.InitializeStaticCollider(target, Vector3d.Zero);
        Step(scene.Context);
        body.Sleep();
        Step(scene.Context);
        body.IsSleeping.Should().BeTrue();
        scene.Context.Physics.GetCollisionPair(target.Id, body.Collider.Id)!.Manifold.HasContact.Should().BeTrue();

        target.Deactivate();
        Step(scene.Context);

        body.IsSleeping.Should().BeFalse();
        (body.Collider.CollisionPairCount + body.Collider.CollisionPairHolderCount).Should().Be(0);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void TriggerSeparationWithinSharedVoxel_ShouldNotifyWithoutPhysicalWake(bool boundsMiss)
    {
        using PhysicsScenarioBuilder scene = CreateScene();
        scene.Context.Environment.Gravity = Fixed64.Zero;
        var trigger = new LSSphereCollider { IsTrigger = true };
        scene.InitializeStaticCollider(trigger, Vector3d.Zero);
        SolidBody body = scene.CreateSphere(Vector3d.Right * Fixed64.FromFraction(3, 4)).Body;
        body.UseManualGrounding();
        body.SleepEnabled = false;
        SolidBody unrelated = scene.CreateSphere(Vector3d.Left * (Fixed64)8).Body;
        unrelated.UseManualGrounding();
        unrelated.Sleep();
        Step(scene.Context);
        var pair = scene.Context.Physics.GetCollisionPair(trigger.Id, body.Collider.Id)!;
        pair.Manifold.HasContact.Should().BeTrue();
        int exits = 0;
        trigger.OnTriggerExit += _ => exits++;
        body.SetPosition(boundsMiss ? Vector3d.Right * Fixed64.FromFraction(5, 4)
            : new Vector3d(Fixed64.FromFraction(3, 4), Fixed64.FromFraction(3, 4), Fixed64.Zero));

        Step(scene.Context);

        pair.LastFrame.Should().Be(scene.Context.FrameCount);
        pair.Manifold.HasContact.Should().BeFalse();
        exits.Should().Be(1);
        unrelated.IsSleeping.Should().BeTrue();
    }

    private static void Step(GravitasWorldContext context) { context.Simulate(); context.LateSimulate(); }
}
