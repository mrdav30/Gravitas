using System;
using FixedMathSharp;
using FluentAssertions;
using Gravitas.Colliders;
using Gravitas.Constraints;
using Gravitas.Tests.Support;
using GridForge.Configuration;
using GridForge.Grids.Topology;
using Xunit;

namespace Gravitas.Tests.Constraints;

public sealed class JointCenterOfMassBoundaryTests
{
    [Fact]
    public void Joint3D_WhenAggregateRepairDistanceOverflows_ShouldKeepTheIslandBounded()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        scenario.Context.Environment.Gravity = Fixed64.Zero;
        var first = scenario.CreateSphere(Vector3d.Zero);
        var second = scenario.CreateSphere(Vector3d.Right * (Fixed64)4);
        var third = scenario.CreateSphere(Vector3d.Right * (Fixed64)8);
        foreach (SolidBody body in new[] { first.Body, second.Body, third.Body })
            body.UseManualGrounding();
        foreach (SolidBody other in new[] { second.Body, third.Body })
            scenario.Context.Constraints3D.RegisterJoint(new JointDefinition3D(
                first.Body, other,
                new FixedTransform(Vector3d.Zero, FixedQuaternion.Identity, Vector3d.One),
                new FixedTransform(Vector3d.Right * (Fixed64)1_500_000_000, FixedQuaternion.Identity, Vector3d.One),
                JointType3D.BallSocket, JointLimit3D.Unrestricted, JointMotor3D.Disabled,
                JointCollisionPolicy.SuppressLinked));

        scenario.Context.Simulate();
        scenario.Context.LateSimulate();

        Assert.Equal(Vector3d.Zero, first.Body.LinearVelocity);
        Assert.Equal(Vector3d.Zero, second.Body.LinearVelocity);
        Assert.Equal(Vector3d.Zero, third.Body.LinearVelocity);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void Joint3D_WithUnrepresentableWarmImpulse_ShouldRejectItBeforePublishingEitherEndpoint(bool minimum)
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        scenario.Context.Environment.Gravity = Fixed64.Zero;
        var first = scenario.CreateSphere(Vector3d.Zero, mass: Fixed64.Quarter);
        var second = scenario.CreateSphere(Vector3d.Right * (Fixed64)3, mass: Fixed64.Quarter);
        first.Body.UseManualGrounding();
        second.Body.UseManualGrounding();
        Joint3D joint = scenario.Context.Constraints3D.RegisterJoint(new JointDefinition3D(
            first.Body, second.Body,
            new FixedTransform(Vector3d.Zero, FixedQuaternion.Identity, Vector3d.One),
            new FixedTransform(Vector3d.Left * (Fixed64)3, FixedQuaternion.Identity, Vector3d.One),
            JointType3D.BallSocket, JointLimit3D.Unrestricted, JointMotor3D.Disabled,
            JointCollisionPolicy.SuppressLinked));
        joint.SetCachedImpulse(0, minimum ? Fixed64.MinValue : Fixed64.MaxValue);

        scenario.Context.Simulate();
        scenario.Context.LateSimulate();

        Assert.Equal(Vector3d.Zero, first.Body.LinearVelocity);
        Assert.Equal(Vector3d.Zero, second.Body.LinearVelocity);
        Assert.Equal(Vector3d.Zero, first.Body.AngularVelocity);
        Assert.Equal(Vector3d.Zero, second.Body.AngularVelocity);
        Assert.Equal(Fixed64.Zero, joint.GetCachedImpulse(0));
    }

    [Fact]
    public void Joint2D_WithOverflowingFrozenAxisError_ShouldPreserveFeasibleCorrection()
    {
        Fixed64 control = Run(includeFrozenError: false);
        Fixed64 withFrozenError = Run(includeFrozenError: true);

        Assert.True(control < Fixed64.Zero);
        Assert.Equal(control, withFrozenError);

        static Fixed64 Run(bool includeFrozenError)
        {
            using GravitasWorldContext context = Physics2DTestWorld.CreateContext();
            context.Environment.Gravity = Fixed64.Zero;
            context.SetFrameRate(PhysicsSettings.MaxResolvableFrameRate);
            SolidBody2D first = CreateBody2D(context, Vector2d.Zero);
            SolidBody2D second = CreateBody2D(context, Vector2d.Right * (Fixed64)3);
            first.UseManualGrounding();
            second.UseManualGrounding();
            first.FreezeAxes = BodyFreezeAxes2D.All;
            second.FreezeAxes = BodyFreezeAxes2D.PositionX | BodyFreezeAxes2D.Rotation;
            Joint2D joint = context.Constraints2D.RegisterJoint(new JointDefinition2D(
                first, second, JointFrame2D.Identity,
                new JointFrame2D(new Vector2d((Fixed64)(includeFrozenError ? 1021 : -3),
                    Fixed64.FromRaw(256)), Fixed64.Zero),
                JointType2D.Pin, JointLimit2D.Unrestricted, JointMotor2D.Disabled,
                JointCollisionPolicy.SuppressLinked));

            context.Simulate();
            context.LateSimulate();

            Assert.Equal(2, joint.LastSolvedRowCount);
            Assert.Equal(Fixed64.Zero, second.LinearVelocity.X);
            Assert.Equal(Fixed64.Zero, second.AngularVelocity);
            return second.LinearVelocity.Y;
        }
    }

    [Fact]
    public void Joint3D_WithOverflowingFrozenAxisError_ShouldPreserveFeasibleCorrection()
    {
        Fixed64 control = Run(includeFrozenError: false);
        Fixed64 withFrozenError = Run(includeFrozenError: true);

        Assert.True(control < Fixed64.Zero);
        Assert.Equal(control, withFrozenError);

        static Fixed64 Run(bool includeFrozenError)
        {
            using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
            scenario.Context.Environment.Gravity = Fixed64.Zero;
            scenario.Context.SetFrameRate(PhysicsSettings.MaxResolvableFrameRate);
            var first = scenario.CreateSphere(Vector3d.Zero, immovable: true);
            var second = scenario.CreateSphere(Vector3d.Right * (Fixed64)3);
            first.Body.UseManualGrounding();
            second.Body.UseManualGrounding();
            second.Body.FreezeAxes = BodyFreezeAxes3D.PositionX | BodyFreezeAxes3D.Rotation;
            Joint3D joint = scenario.Context.Constraints3D.RegisterJoint(new JointDefinition3D(
                first.Body, second.Body,
                new FixedTransform(Vector3d.Zero, FixedQuaternion.Identity, Vector3d.One),
                new FixedTransform(new Vector3d((Fixed64)(includeFrozenError ? 1021 : -3),
                    Fixed64.FromRaw(256), Fixed64.Zero), FixedQuaternion.Identity, Vector3d.One),
                JointType3D.BallSocket, JointLimit3D.Unrestricted, JointMotor3D.Disabled,
                JointCollisionPolicy.SuppressLinked));

            scenario.Context.Simulate();
            scenario.Context.LateSimulate();

            Assert.Equal(3, joint.LastSolvedRowCount);
            Assert.Equal(Fixed64.Zero, second.Body.LinearVelocity.X);
            Assert.Equal(Vector3d.Zero, second.Body.AngularVelocity);
            return second.Body.LinearVelocity.Y;
        }
    }

    [Fact]
    public void Joint3D_WithUnrepresentableAnchors_ShouldPreserveConnectedJointCorrection()
    {
        (Fixed64 first, Fixed64 second) control = Run(includeInvalidJoint: false);
        (Fixed64 first, Fixed64 second) withInvalid = Run(includeInvalidJoint: true);

        Assert.True(control.first > Fixed64.Zero);
        Assert.True(control.second < Fixed64.Zero);
        Assert.Equal(control, withInvalid);

        static (Fixed64 first, Fixed64 second) Run(bool includeInvalidJoint)
        {
            using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
            scenario.Context.Environment.Gravity = Fixed64.Zero;
            var first = scenario.CreateSphere(Vector3d.Zero);
            var invalidOther = scenario.CreateSphere(Vector3d.Right * (Fixed64)8);
            var validOther = scenario.CreateSphere(Vector3d.Right * (Fixed64)3);
            first.Body.UseManualGrounding();
            invalidOther.Body.UseManualGrounding();
            validOther.Body.UseManualGrounding();
            Joint3D? invalid = includeInvalidJoint
                ? scenario.Context.Constraints3D.RegisterJoint(new JointDefinition3D(
                    first.Body, invalidOther.Body,
                    new FixedTransform(new Vector3d(Fixed64.MinValue, Fixed64.Zero, Fixed64.Zero), FixedQuaternion.Identity, Vector3d.One),
                    new FixedTransform(new Vector3d(Fixed64.MaxValue, Fixed64.Zero, Fixed64.Zero), FixedQuaternion.Identity, Vector3d.One),
                    JointType3D.BallSocket, JointLimit3D.Unrestricted, JointMotor3D.Disabled, JointCollisionPolicy.SuppressLinked))
                : null;
            Joint3D valid = scenario.Context.Constraints3D.RegisterJoint(new JointDefinition3D(
                first.Body, validOther.Body,
                new FixedTransform(Vector3d.Zero, FixedQuaternion.Identity, Vector3d.One),
                new FixedTransform(new Vector3d(-(Fixed64)3 + Fixed64.Eighth, Fixed64.Zero, Fixed64.Zero), FixedQuaternion.Identity, Vector3d.One),
                JointType3D.BallSocket, JointLimit3D.Unrestricted, JointMotor3D.Disabled, JointCollisionPolicy.SuppressLinked));

            scenario.Context.Simulate();
            scenario.Context.LateSimulate();

            if (invalid != null)
                Assert.Equal(0, invalid.LastSolvedRowCount);
            Assert.Equal(3, valid.LastSolvedRowCount);
            return (first.Body.LinearVelocity.X, validOther.Body.LinearVelocity.X);
        }
    }

    [Fact]
    public void Joint3D_WhenContactCorrectionExceedsIncomingBudget_ShouldRejectRowsBeforeWarmStart()
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        context.SetFrameRate(PhysicsSettings.MaxResolvableFrameRate);
        context.Environment.Gravity = Fixed64.Zero;
        Assert.True(context.World.TryAddGrid(new GridConfiguration(
            new Vector3d(-4096, -4096, -4096), new Vector3d(4096, 4096, 4096),
            topologyMetrics: GridTopologyMetrics.Rectangular((Fixed64)1024)), out _));
        var support = new LSSphereCollider { Radius = (Fixed64)512 };
        support.InitializeWithNoBody(new TestMatterAgent(context,
            new FixedTransform(Vector3d.Up, FixedQuaternion.Identity, Vector3d.One)));
        SolidBody first = CreateSpatialBody(Vector3d.Zero, (Fixed64)512);
        SolidBody second = CreateSpatialBody(Vector3d.Right * (Fixed64)2000, Fixed64.Half);
        Joint3D joint = context.Constraints3D.RegisterJoint(new JointDefinition3D(
            first, second,
            new FixedTransform(Vector3d.Zero, FixedQuaternion.Identity, Vector3d.One),
            new FixedTransform(Vector3d.Left * (Fixed64)2000, FixedQuaternion.Identity, Vector3d.One),
            JointType3D.BallSocket, JointLimit3D.Unrestricted, JointMotor3D.Disabled,
            JointCollisionPolicy.SuppressLinked));
        joint.SetCachedImpulse(0, Fixed64.One);

        context.Simulate();
        context.LateSimulate();

        Assert.True(first.Position3d.Y < -(Fixed64)512,
            "Contact correction must separate the initially coincident joint anchors after budget preparation.");
        Assert.Equal(0, joint.LastSolvedRowCount);
        Assert.Equal(Fixed64.One, joint.GetCachedImpulse(0));
        Assert.Equal(Vector3d.Zero, first.LinearVelocity);
        Assert.Equal(Vector3d.Zero, second.LinearVelocity);
        Assert.Equal(Vector3d.Zero, first.AngularVelocity);
        Assert.Equal(Vector3d.Zero, second.AngularVelocity);

        SolidBody CreateSpatialBody(Vector3d position, Fixed64 radius)
        {
            var body = new SolidBody(new TestMatterAgent(context,
                new FixedTransform(position, FixedQuaternion.Identity, Vector3d.One)),
                new LSSphereCollider { Radius = radius }) { Mass = Fixed64.One };
            body.UseManualGrounding();
            body.Initialize(position, FixedQuaternion.Identity, PhysicsMobilityType.Dynamic);
            return body;
        }
    }

    [Fact]
    public void Joint3D_WithLargeAnchorErrorAtMaximumRate_ShouldBudgetCorrectionWithinSpeedLimit()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        scenario.Context.SetFrameRate(PhysicsSettings.MaxResolvableFrameRate);
        scenario.Context.Environment.Gravity = Fixed64.Zero;
        var first = scenario.CreateSphere(Vector3d.Zero);
        var second = scenario.CreateSphere(Vector3d.Right * (Fixed64)3);
        Joint3D joint = scenario.Context.Constraints3D.RegisterJoint(new JointDefinition3D(
            first.Body, second.Body,
            new FixedTransform(Vector3d.Zero, FixedQuaternion.Identity, Vector3d.One),
            new FixedTransform(new Vector3d(-3, 1024, 0), FixedQuaternion.Identity, Vector3d.One),
            JointType3D.BallSocket, JointLimit3D.Unrestricted, JointMotor3D.Disabled,
            JointCollisionPolicy.SuppressLinked));
        scenario.Context.Simulate();
        scenario.Context.LateSimulate();

        Assert.Equal(3, joint.LastSolvedRowCount);
        Assert.True(joint.AccumulatedImpulseMagnitude > Fixed64.Zero);
        foreach (SolidBody body in new[] { first.Body, second.Body })
        {
            Assert.True(body.LinearVelocity.Magnitude <= scenario.Context.Environment.MaxSpeed);
            Assert.True(body.AngularVelocity.Magnitude <= scenario.Context.Environment.MaxSpeed);
        }
    }

    [Fact]
    public void Joint2D_WhenCorrectionVelocityOverflows_ShouldRejectBeforeApplyingCachedRows()
    {
        using GravitasWorldContext context = Physics2DTestWorld.CreateContext();
        context.SetFrameRate(PhysicsSettings.MaxResolvableFrameRate);
        context.Environment.Gravity = Fixed64.Zero;
        SolidBody2D first = CreateBody2D(context, Vector2d.Zero);
        SolidBody2D second = CreateBody2D(context, Vector2d.Right * (Fixed64)3);
        Joint2D joint = context.Constraints2D.RegisterJoint(new JointDefinition2D(
            first, second, JointFrame2D.Identity,
            new JointFrame2D(new Vector2d(-3, 1024), Fixed64.Zero),
            JointType2D.Pin, JointLimit2D.Unrestricted, JointMotor2D.Disabled,
            JointCollisionPolicy.SuppressLinked));
        joint.SetCachedImpulse(0, Fixed64.One);

        context.Simulate();
        context.LateSimulate();

        Assert.Equal(0, joint.LastSolvedRowCount);
        Assert.Equal(Fixed64.One, joint.GetCachedImpulse(0));
        Assert.Equal(Vector2d.Zero, first.LinearVelocity);
        Assert.Equal(Vector2d.Zero, second.LinearVelocity);
        Assert.Equal(Fixed64.Zero, first.AngularVelocity);
        Assert.Equal(Fixed64.Zero, second.AngularVelocity);
    }

    [Fact]
    public void Joint3D_WhenWorldCenterOfMassIsUnrepresentable_ShouldUseRelativeLeverArm()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        scenario.Context.Environment.Gravity = Fixed64.Zero;
        ScenarioBody<LSSphereCollider> first = scenario.CreateSphere(Vector3d.Right);
        ScenarioBody<LSSphereCollider> second = scenario.CreateSphere(Vector3d.Right * Fixed64.Two);
        first.Body.LocalCenterOfMassOffset = new Vector3d(
            Fixed64.MaxValue,
            Fixed64.Zero,
            Fixed64.Zero);
        var localFrame = new FixedTransform(
            Vector3d.Zero,
            FixedQuaternion.Identity,
            Vector3d.One);
        Joint3D joint = scenario.Context.Constraints3D.RegisterJoint(new JointDefinition3D(
            first.Body,
            second.Body,
            localFrame,
            localFrame,
            JointType3D.BallSocket,
            JointLimit3D.Unrestricted,
            JointMotor3D.Disabled,
            JointCollisionPolicy.SuppressLinked));

        Action simulate = () =>
        {
            scenario.Context.Simulate();
            scenario.Context.LateSimulate();
        };

        simulate.Should().NotThrow();
        joint.LastSolvedRowCount.Should().Be(3);
    }

    [Fact]
    public void Joint3D_WhenAnchorErrorIsUnrepresentable_ShouldRejectRowsAtomically()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        scenario.Context.Environment.Gravity = Fixed64.Zero;
        ScenarioBody<LSSphereCollider> first = scenario.CreateSphere(Vector3d.Zero);
        ScenarioBody<LSSphereCollider> second = scenario.CreateSphere(Vector3d.Zero);
        Joint3D joint = scenario.Context.Constraints3D.RegisterJoint(new JointDefinition3D(
            first.Body,
            second.Body,
            new FixedTransform(
                new Vector3d(Fixed64.MinValue, Fixed64.Zero, Fixed64.Zero),
                FixedQuaternion.Identity,
                Vector3d.One),
            new FixedTransform(
                new Vector3d(Fixed64.MaxValue, Fixed64.Zero, Fixed64.Zero),
                FixedQuaternion.Identity,
                Vector3d.One),
            JointType3D.BallSocket,
            JointLimit3D.Unrestricted,
            JointMotor3D.Disabled,
            JointCollisionPolicy.SuppressLinked));

        scenario.Context.Simulate();
        scenario.Context.LateSimulate();

        joint.LastSolvedRowCount.Should().Be(0);
        joint.AccumulatedImpulseMagnitude.Should().Be(Fixed64.Zero);
    }

    [Theory]
    [InlineData(true, true)]
    [InlineData(false, true)]
    [InlineData(true, false)]
    [InlineData(false, false)]
    public void Joint3D_WhenOneParticipantCannotRotateItsAnchorOrCenterOfMass_ShouldRejectRowsAtomically(
        bool failFirst,
        bool failAnchorRotation)
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        scenario.Context.Environment.Gravity = Fixed64.Zero;
        ScenarioBody<LSSphereCollider> first = scenario.CreateSphere(Vector3d.Zero);
        ScenarioBody<LSSphereCollider> second = scenario.CreateSphere(Vector3d.Zero);
        Vector3d extreme = new(Fixed64.MaxValue, Fixed64.Zero, Fixed64.MaxValue);
        SolidBody failingBody = failFirst ? first.Body : second.Body;
        failingBody.SetRotation(FixedQuaternion.FromEulerAnglesInDegrees(
            Fixed64.Zero,
            (Fixed64)45,
            Fixed64.Zero));
        if (!failAnchorRotation)
            failingBody.LocalCenterOfMassOffset = extreme;
        Vector3d frameAOffset = failAnchorRotation && failFirst ? extreme : Vector3d.Zero;
        Vector3d frameBOffset = failAnchorRotation && !failFirst ? extreme : Vector3d.Zero;
        Joint3D joint = scenario.Context.Constraints3D.RegisterJoint(new JointDefinition3D(
            first.Body,
            second.Body,
            new FixedTransform(frameAOffset, FixedQuaternion.Identity, Vector3d.One),
            new FixedTransform(frameBOffset, FixedQuaternion.Identity, Vector3d.One),
            JointType3D.BallSocket,
            JointLimit3D.Unrestricted,
            JointMotor3D.Disabled,
            JointCollisionPolicy.SuppressLinked));

        scenario.Context.Simulate();
        scenario.Context.LateSimulate();

        joint.LastSolvedRowCount.Should().Be(0);
        joint.AccumulatedImpulseMagnitude.Should().Be(Fixed64.Zero);
    }

    [Fact]
    public void Joint2D_WhenWorldCenterOfMassIsUnrepresentable_ShouldUseRelativeLeverArm()
    {
        using GravitasWorldContext context = Physics2DTestWorld.CreateContext();
        context.Environment.Gravity = Fixed64.Zero;
        SolidBody2D first = CreateBody2D(context, Vector2d.Right);
        SolidBody2D second = CreateBody2D(context, Vector2d.Right * Fixed64.Two);
        first.LocalCenterOfMassOffset = new Vector2d(Fixed64.MaxValue, Fixed64.Zero);
        Joint2D joint = context.Constraints2D.RegisterJoint(new JointDefinition2D(
            first,
            second,
            JointFrame2D.Identity,
            JointFrame2D.Identity,
            JointType2D.Pin,
            JointLimit2D.Unrestricted,
            JointMotor2D.Disabled,
            JointCollisionPolicy.SuppressLinked));

        Action simulate = () =>
        {
            context.Simulate();
            context.LateSimulate();
        };

        simulate.Should().NotThrow();
        joint.LastSolvedRowCount.Should().Be(2);
    }

    [Fact]
    public void Joint2D_WhenAnchorErrorIsUnrepresentable_ShouldRejectRowsAtomically()
    {
        using GravitasWorldContext context = Physics2DTestWorld.CreateContext();
        context.Environment.Gravity = Fixed64.Zero;
        SolidBody2D first = CreateBody2D(context, Vector2d.Zero);
        SolidBody2D second = CreateBody2D(context, Vector2d.Zero);
        Joint2D joint = context.Constraints2D.RegisterJoint(new JointDefinition2D(
            first,
            second,
            new JointFrame2D(
                new Vector2d(Fixed64.MinValue, Fixed64.Zero),
                Fixed64.Zero),
            new JointFrame2D(
                new Vector2d(Fixed64.MaxValue, Fixed64.Zero),
                Fixed64.Zero),
            JointType2D.Pin,
            JointLimit2D.Unrestricted,
            JointMotor2D.Disabled,
            JointCollisionPolicy.SuppressLinked));

        context.Simulate();
        context.LateSimulate();

        joint.LastSolvedRowCount.Should().Be(0);
        joint.AccumulatedImpulseMagnitude.Should().Be(Fixed64.Zero);
    }

    [Theory]
    [InlineData(true, true)]
    [InlineData(false, true)]
    [InlineData(true, false)]
    [InlineData(false, false)]
    public void Joint2D_WhenOneParticipantCannotRotateItsAnchorOrCenterOfMass_ShouldRejectRowsAtomically(
        bool failFirst,
        bool failAnchorRotation)
    {
        using GravitasWorldContext context = Physics2DTestWorld.CreateContext();
        context.Environment.Gravity = Fixed64.Zero;
        SolidBody2D first = CreateBody2D(context, Vector2d.Zero);
        SolidBody2D second = CreateBody2D(context, Vector2d.Zero);
        Vector2d extreme = new(Fixed64.MaxValue, Fixed64.MaxValue);
        SolidBody2D failingBody = failFirst ? first : second;
        failingBody.SetRotation(Fixed64.PiOver4);
        if (!failAnchorRotation)
            failingBody.LocalCenterOfMassOffset = extreme;
        Vector2d frameAOffset = failAnchorRotation && failFirst ? extreme : Vector2d.Zero;
        Vector2d frameBOffset = failAnchorRotation && !failFirst ? extreme : Vector2d.Zero;
        Joint2D joint = context.Constraints2D.RegisterJoint(new JointDefinition2D(
            first,
            second,
            new JointFrame2D(frameAOffset, Fixed64.Zero),
            new JointFrame2D(frameBOffset, Fixed64.Zero),
            JointType2D.Pin,
            JointLimit2D.Unrestricted,
            JointMotor2D.Disabled,
            JointCollisionPolicy.SuppressLinked));

        context.Simulate();
        context.LateSimulate();

        joint.LastSolvedRowCount.Should().Be(0);
        joint.AccumulatedImpulseMagnitude.Should().Be(Fixed64.Zero);
    }

    private static SolidBody2D CreateBody2D(
        GravitasWorldContext context,
        Vector2d position)
    {
        var transform = new FixedTransform(
            position.ToVector3d(Fixed64.Zero),
            FixedQuaternion.Identity,
            Vector3d.One);
        var body = new SolidBody2D(
            new TestMatterAgent(context, transform),
            new LSCircleCollider2D(Fixed64.Half))
        {
            Mass = Fixed64.One
        };
        body.Initialize(position, Fixed64.Zero, PhysicsMobilityType.Dynamic);
        return body;
    }
}
