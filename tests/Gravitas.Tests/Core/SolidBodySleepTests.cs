using System;
using Chronicler;
using FixedMathSharp;
using FluentAssertions;
using Gravitas.Colliders;
using Gravitas.Tests.Serialization;
using Gravitas.Tests.Support;
using Xunit;

namespace Gravitas.Tests.Core;

public sealed class SolidBodySleepTests
{
    [Theory]
    [InlineData(false, false)]
    [InlineData(true, false)]
    [InlineData(true, true)]
    public void QuietSpatialStack_ShouldSleepTogetherAfterStaggeredEligibility(bool enableUpper, bool restore)
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        GravitasWorldContext context = scenario.Context;
        context.SetFrameRate(32);
        context.Environment.Gravity = context.Environment.AirDensity = context.Environment.DampingFactor = Fixed64.Zero;
        scenario.InitializeStaticCollider(new LSCuboidCollider { Size = new Vector3d(16, 1, 16) },
            Vector3d.Down * Fixed64.Half);
        var lower = scenario.CreateBody(new LSCuboidCollider { Size = new Vector3d(2, 1, 2) },
            Vector3d.Up * Fixed64.Half, FixedQuaternion.Identity);
        var upper = scenario.CreateBody(new LSCuboidCollider { Size = new Vector3d(2, 1, 2) },
            Vector3d.Up * Fixed64.FromFraction(3, 2), FixedQuaternion.Identity, mass: (Fixed64)100);
        lower.Body.UseManualGrounding();
        upper.Body.UseManualGrounding();
        lower.Body.SleepFrameThreshold = upper.Body.SleepFrameThreshold = 4;
        upper.Body.SleepEnabled = false;

        AssertSleepSequence(context, enableUpper, () => upper.Body.SleepEnabled = true,
            () => lower.Body.IsSleeping, () => upper.Body.IsSleeping, upper.Body.Wake,
            restore ? () => RestoreQuietCounter(lower.Body, lower.Body.Wake) : null);

        Assert.Equal(Vector3d.Up * Fixed64.Half, lower.Body.Position3d);
        Assert.Equal(Vector3d.Up * Fixed64.FromFraction(3, 2), upper.Body.Position3d);
    }

    [Theory]
    [InlineData(false, false)]
    [InlineData(true, false)]
    [InlineData(true, true)]
    public void QuietPlanarStack_ShouldSleepTogetherAfterStaggeredEligibility(bool enableUpper, bool restore)
    {
        using GravitasWorldContext context = Physics2DTestWorld.CreateContext(frameRate: 32);
        SolidBody2D lower = CreateBox(Fixed64.Half, Fixed64.One, PhysicsMobilityType.Dynamic);
        SolidBody2D upper = CreateBox(Fixed64.FromFraction(3, 2), (Fixed64)100, PhysicsMobilityType.Dynamic);
        _ = CreateBox(-Fixed64.Half, Fixed64.One, PhysicsMobilityType.Static);
        upper.SleepEnabled = false;

        AssertSleepSequence(context, enableUpper, () => upper.SleepEnabled = true,
            () => lower.IsSleeping, () => upper.IsSleeping, upper.Wake,
            restore ? () => RestoreQuietCounter(lower, lower.Wake) : null);

        Assert.Equal(Fixed64.Half, lower.Position.Y);
        Assert.Equal(Fixed64.FromFraction(3, 2), upper.Position.Y);

        SolidBody2D CreateBox(Fixed64 height, Fixed64 mass, PhysicsMobilityType mobility)
        {
            var position = new Vector2d(Fixed64.Zero, height);
            var body = new SolidBody2D(new TestMatterAgent(context,
                new FixedTransform(position.ToVector3d(Fixed64.Zero), FixedQuaternion.Identity, Vector3d.One)),
                new LSAABBoxCollider2D(new Vector2d(mobility == PhysicsMobilityType.Static ? (Fixed64)16 : Fixed64.Two, Fixed64.One)))
            {
                Mass = mass,
                SleepFrameThreshold = 4
            };
            body.UseManualGrounding();
            body.Initialize(position, motionType: mobility);
            return body;
        }
    }

    private static void AssertSleepSequence(GravitasWorldContext context, bool enableUpper, Action enable,
        Func<bool> lowerAsleep, Func<bool> upperAsleep, Action wakeUpper, Action? restore)
    {
        Step();
        if (enableUpper)
            enable();

        for (int frame = 2; frame <= 20; frame++)
        {
            Step();
            bool expected = enableUpper && frame >= 5;
            Assert.Equal(expected, lowerAsleep());
            Assert.Equal(expected, upperAsleep());
            if (frame == 4)
                restore?.Invoke();
        }

        if (enableUpper)
        {
            wakeUpper();
            Step();
            Assert.False(lowerAsleep());
            Assert.False(upperAsleep());
        }
        else
        {
            // The lower body remains eligible while its neighbor vetoes sleep,
            // so each measured step exercises the component decision.
            Assert.Equal(0, AllocationTestHelper.MeasureSteadyState(Step));
            Assert.False(lowerAsleep());
            Assert.False(upperAsleep());
        }

        void Step()
        {
            context.Simulate();
            context.LateSimulate();
        }
    }

    private static void RestoreQuietCounter(IRecordable body, Action wake)
    {
        object payload = GravitasSerializationHarness.Serialize(body, GravitasSerializationTransport.Json);
        wake();
        GravitasSerializationHarness.Populate(body, payload, GravitasSerializationTransport.Json);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void LateSimulate_AtRestForSleepWindow_ShouldPutBodyToSleep(bool bodyOnly)
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        scenario.Context.Environment.Gravity = Fixed64.Zero;
        ScenarioBody<LSSphereCollider> body = scenario.CreateSphere(PhysicsScenarioBuilder.Vector(0, 0, 0));
        body.Body.SleepFrameThreshold = 2;

        Step();
        body.Body.IsSleeping.Should().BeFalse();

        Step();
        body.Body.IsSleeping.Should().BeTrue();

        Step();
        body.Body.IsSleeping.Should().BeTrue();

        void Step()
        {
            if (bodyOnly)
                body.Body.LateSimulate();
            else
                scenario.Context.LateSimulate();
        }
    }

    [Fact]
    public void SleepingBody_ShouldWakeFromDeterministicStimuli()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSSphereCollider> body = scenario.CreateSphere(PhysicsScenarioBuilder.Vector(0, 0, 0));

        body.Body.Sleep();
        body.Body.IsSleeping.Should().BeTrue();

        body.Body.AddForce(Vector3d.Right);

        body.Body.IsSleeping.Should().BeFalse();

        body.Body.Sleep();
        body.Body.SetPosition(Vector3d.Right);

        body.Body.IsSleeping.Should().BeFalse();

        body.Body.Sleep();
        body.Body.AddLinearImpulse(Vector3d.Right);

        body.Body.IsSleeping.Should().BeFalse();

        body.Body.Sleep();
        body.Body.AddAngularImpulse(Vector3d.Up);

        body.Body.IsSleeping.Should().BeFalse();
    }

    [Fact]
    public void ShapeMutation_ShouldWakeSleepingBody()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSSphereCollider> body = scenario.CreateSphere(PhysicsScenarioBuilder.Vector(0, 0, 0));
        body.Body.Sleep();

        body.Collider.Radius = Fixed64.One;

        body.Body.IsSleeping.Should().BeFalse();
    }

    [Fact]
    public void SleepDeactivateAndLateSimulate_ShouldRemainIdempotent()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSSphereCollider> body = scenario.CreateSphere(Vector3d.Zero);

        body.Body.Sleep();
        body.Body.Sleep();
        body.Body.Deactivate();
        body.Body.Sleep();
        body.Body.Deactivate();
        body.Body.LateSimulate();

        body.Body.IsSleeping.Should().BeTrue();
        body.Body.Active.Should().BeFalse();
        body.Body.DynamicId.Should().Be(-1);
        body.Collider.IsActive.Should().BeFalse();
        scenario.Context.Physics.BodyCount.Should().Be(0);
    }

    [Fact]
    public void LateSimulate_StackedRestingBodies_ShouldSleepDeterministically()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        scenario.Context.Environment.Gravity = Fixed64.Zero;
        ScenarioBody<LSCuboidCollider> lower = scenario.CreateCuboid(PhysicsScenarioBuilder.Vector(0, 0, 0));
        ScenarioBody<LSCuboidCollider> upper = scenario.CreateCuboid(PhysicsScenarioBuilder.Vector(0, 1, 0));
        lower.Body.SleepFrameThreshold = 1;
        upper.Body.SleepFrameThreshold = 1;

        scenario.Context.Simulate();
        scenario.Context.LateSimulate();

        lower.Body.IsSleeping.Should().BeTrue();
        upper.Body.IsSleeping.Should().BeTrue();
    }
}
