using FixedMathSharp;
using FluentAssertions;
using Gravitas.Colliders;
using Gravitas.Constraints;
using Gravitas.Tests.Support;
using System;
using Xunit;

namespace Gravitas.Tests.Constraints;

public sealed class JointCenterOfMassBoundaryTests
{
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
        body.Initialize(position, Fixed64.Zero, BodyMotionType.Dynamic);
        return body;
    }
}
