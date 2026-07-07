using Chronicler;
using FixedMathSharp;
using FluentAssertions;
using Gravitas.Colliders;
using Gravitas.Constraints;
using Gravitas.Diagnostics;
using Gravitas.Tests.Serialization;
using Gravitas.Tests.Support;
using System;
using System.Collections.Generic;
using Xunit;

namespace Gravitas.Tests.Constraints;

public sealed class Constraint2DServiceTests
{
    public static TheoryData<GravitasSerializationTransport> Transports => GravitasSerializationTransportCases.All();

    [Fact]
    public void NewContext_ShouldOwnEmpty2DConstraintService()
    {
        using GravitasWorldContext context = CreateConstraintContext();

        context.Constraints2D.Should().NotBeNull();
        context.Constraints2D.RegisteredJointCount.Should().Be(0);
        context.Constraints2D.PeakJointCount.Should().Be(0);
    }

    [Fact]
    public void RegisterJoint_ShouldAssignDeterministicMonotonicIdsAndAllowDuplicateBodyPairs()
    {
        using GravitasWorldContext context = CreateConstraintContext();
        SolidBody2D first = CreateBody(context, Vector2d.Zero);
        SolidBody2D second = CreateBody(context, Vector2d.Right * (Fixed64)2);
        JointDefinition2D definition = CreatePin(first, second);

        Joint2D firstJoint = context.Constraints2D.RegisterJoint(definition);
        Joint2D secondJoint = context.Constraints2D.RegisterJoint(definition);

        firstJoint.Id.Should().Be(1);
        secondJoint.Id.Should().Be(2);
        firstJoint.Should().NotBeSameAs(secondJoint);
        context.Constraints2D.RegisteredJointCount.Should().Be(2);
        context.Constraints2D.PeakJointCount.Should().Be(2);
        context.Constraints2D.TryGetJoint(1, out Joint2D? resolved).Should().BeTrue();
        resolved.Should().BeSameAs(firstJoint);
    }

    [Fact]
    public void RegisterJoint_WithInvalidDefinition_ShouldFailBeforeSolverStateIsCreated()
    {
        using GravitasWorldContext context = CreateConstraintContext();
        SolidBody2D body = CreateBody(context, Vector2d.Zero);

        Action sameBody = () => context.Constraints2D.RegisterJoint(CreatePin(body, body));
        Action invalidType = () => context.Constraints2D.RegisterJoint(new JointDefinition2D(
            body,
            CreateBody(context, Vector2d.Right * (Fixed64)2),
            JointFrame2D.Identity,
            JointFrame2D.Identity,
            (JointType2D)255,
            JointLimit2D.Unrestricted,
            JointMotor2D.Disabled,
            JointCollisionPolicy.SuppressLinked));
        Action invalidLimit = () => context.Constraints2D.RegisterJoint(new JointDefinition2D(
            body,
            CreateBody(context, Vector2d.Right * (Fixed64)4),
            JointFrame2D.Identity,
            JointFrame2D.Identity,
            JointType2D.Prismatic,
            JointLimit2D.Slider(Fixed64.One, -Fixed64.One),
            JointMotor2D.Disabled,
            JointCollisionPolicy.SuppressLinked));

        sameBody.Should().Throw<ArgumentException>();
        invalidType.Should().Throw<ArgumentException>();
        invalidLimit.Should().Throw<ArgumentException>();
        context.Constraints2D.RegisteredJointCount.Should().Be(0);
    }

    [Fact]
    public void RegisterJoint_WithBodiesFromDifferentContexts_ShouldFail()
    {
        using GravitasWorldContext firstContext = CreateConstraintContext();
        using GravitasWorldContext secondContext = CreateConstraintContext();
        SolidBody2D first = CreateBody(firstContext, Vector2d.Zero);
        SolidBody2D second = CreateBody(secondContext, Vector2d.Right * (Fixed64)2);

        Action act = () => firstContext.Constraints2D.RegisterJoint(CreatePin(first, second));

        act.Should().Throw<ArgumentException>();
    }

    [Fact]
    public void DirectJoint_ShouldSuppressAdjacentLinked2DCollisionByDefault()
    {
        using GravitasWorldContext context = CreateConstraintContext();
        SolidBody2D first = CreateBody(context, Vector2d.Zero);
        SolidBody2D second = CreateBody(context, Vector2d.Right * Fixed64.Half);

        context.Constraints2D.RegisterJoint(CreatePin(first, second));
        Step(context);

        first.Collider.TryGetCollisionPair(second.Collider.Id, out _).Should().BeFalse();
        context.Constraints2D.ShouldExcludeLinkedCollision(first.Collider, second.Collider).Should().BeTrue();
    }

    [Fact]
    public void DirectJoint_WithCollidePolicy_ShouldAllowLinked2DCollision()
    {
        using GravitasWorldContext context = CreateConstraintContext();
        SolidBody2D first = CreateBody(context, Vector2d.Zero);
        SolidBody2D second = CreateBody(context, Vector2d.Right * Fixed64.Half);

        context.Constraints2D.RegisterJoint(CreatePin(first, second, JointCollisionPolicy.Collide));
        Step(context);

        first.Collider.TryGetCollisionPair(second.Collider.Id, out CollisionPair2D? pair).Should().BeTrue();
        pair!.Manifold.HasContact.Should().BeTrue();
    }

    [Fact]
    public void LinkedCollisionSuppression_ShouldNotFollowReusedColliderIdsAfterColliderRemoval()
    {
        using GravitasWorldContext context = CreateConstraintContext();
        SolidBody2D first = CreateBody(context, Vector2d.Zero);
        SolidBody2D second = CreateBody(context, Vector2d.Right * (Fixed64)2);
        context.Constraints2D.RegisterJoint(CreatePin(first, second));

        first.Collider.Deactivate();
        second.Collider.Deactivate();
        SolidBody2D replacementA = CreateBody(context, Vector2d.Forward * (Fixed64)4);
        SolidBody2D replacementB = CreateBody(context, Vector2d.Forward * (Fixed64)6);

        context.Constraints2D.ShouldExcludeLinkedCollision(
            replacementA.Collider,
            replacementB.Collider).Should().BeFalse();
    }

    [Fact]
    public void DistanceJoint_ShouldReduceAnchorSeparationThroughPlanarImpulses()
    {
        using GravitasWorldContext context = CreateConstraintContext();
        SolidBody2D first = CreateBody(context, Vector2d.Zero);
        SolidBody2D second = CreateBody(context, Vector2d.Right * (Fixed64)4);
        Joint2D joint = context.Constraints2D.RegisterJoint(new JointDefinition2D(
            first,
            second,
            new JointFrame2D(Vector2d.Right * Fixed64.Half, Fixed64.Zero),
            new JointFrame2D(-Vector2d.Right * Fixed64.Half, Fixed64.Zero),
            JointType2D.Distance,
            JointLimit2D.Distance(Fixed64.One),
            JointMotor2D.Disabled,
            JointCollisionPolicy.SuppressLinked));
        Fixed64 before = AnchorDistanceSquared(first, second, joint);

        Step(context, 12);

        AnchorDistanceSquared(first, second, joint).Should().BeLessThan(before);
        joint.LastSolvedRowCount.Should().BeGreaterThan(0);
        joint.AccumulatedImpulseMagnitude.Should().BeGreaterThan(Fixed64.Zero);
    }

    [Fact]
    public void WeldJoint_ShouldReduceScalarAngularFrameError()
    {
        using GravitasWorldContext context = CreateConstraintContext();
        SolidBody2D first = CreateBody(context, Vector2d.Zero);
        SolidBody2D second = CreateBody(context, Vector2d.Right * (Fixed64)2, rotation: FixedMath.DegToRad((Fixed64)45));
        Joint2D joint = context.Constraints2D.RegisterJoint(new JointDefinition2D(
            first,
            second,
            JointFrame2D.Identity,
            JointFrame2D.Identity,
            JointType2D.Weld,
            JointLimit2D.Unrestricted,
            JointMotor2D.Disabled,
            JointCollisionPolicy.SuppressLinked));
        Fixed64 before = joint.LastSolveMetrics.AngularErrorMagnitude;
        before.Should().Be(Fixed64.Zero);

        Fixed64 initialError = (second.Rotation - first.Rotation).Abs();
        Step(context, 16);

        (second.Rotation - first.Rotation).Abs().Should().BeLessThan(initialError);
        joint.LastSolveMetrics.AngularErrorMagnitude.Should().BeGreaterThan(Fixed64.Zero);
    }

    [Fact]
    public void PrismaticJoint_ShouldPreserveSliderAxisAndRespectFrozenAxes()
    {
        using GravitasWorldContext context = CreateConstraintContext();
        SolidBody2D first = CreateBody(context, Vector2d.Zero);
        SolidBody2D second = CreateBody(context, new Vector2d((Fixed64)3, Fixed64.One));
        second.FreezeAxes = BodyFreezeAxes2D.PositionY;
        Joint2D joint = context.Constraints2D.RegisterJoint(new JointDefinition2D(
            first,
            second,
            new JointFrame2D(Vector2d.Zero, Fixed64.Zero),
            new JointFrame2D(Vector2d.Zero, Fixed64.Zero),
            JointType2D.Prismatic,
            JointLimit2D.Slider(Fixed64.Zero, (Fixed64)2),
            JointMotor2D.Disabled,
            JointCollisionPolicy.SuppressLinked));
        Fixed64 frozenY = second.Position.Y;

        Step(context, 12);

        second.Position.Y.Should().Be(frozenY);
        joint.LastSolvedRowCount.Should().BeGreaterThan(0);
        joint.LastSolveMetrics.ClampedRowCount.Should().BeGreaterThan(0);
    }

    [Fact]
    public void PrismaticJoint_WithLowerSliderLimit_ShouldReportLimitError()
    {
        using GravitasWorldContext context = CreateConstraintContext();
        SolidBody2D first = CreateBody(context, Vector2d.Zero, immovable: true);
        SolidBody2D second = CreateBody(context, new Vector2d((Fixed64)(-3), Fixed64.Zero));
        Joint2D joint = context.Constraints2D.RegisterJoint(new JointDefinition2D(
            first,
            second,
            JointFrame2D.Identity,
            JointFrame2D.Identity,
            JointType2D.Prismatic,
            JointLimit2D.Slider((Fixed64)(-2), Fixed64.Zero),
            JointMotor2D.Disabled,
            JointCollisionPolicy.SuppressLinked));

        Step(context, 2);

        joint.LastSolveMetrics.LimitErrorMagnitude.Should().BeGreaterThan(Fixed64.Zero);
        joint.LastSolveMetrics.ClampedRowCount.Should().BeGreaterThan(0);
    }

    [Fact]
    public void ConstraintIsland_ShouldWakeSleepingLinked2DBodiesAsOneIsland()
    {
        using GravitasWorldContext context = CreateConstraintContext();
        SolidBody2D sleeping = CreateBody(context, Vector2d.Zero);
        SolidBody2D driver = CreateBody(context, Vector2d.Right * (Fixed64)3);
        context.Constraints2D.RegisterJoint(CreatePin(sleeping, driver));
        sleeping.Sleep();
        driver.AddForce(-Vector2d.Right * (Fixed64)16);

        Step(context);

        sleeping.IsSleeping.Should().BeFalse();
        driver.IsSleeping.Should().BeFalse();
    }

    [Fact]
    public void ConstraintSolveOrder_ShouldBeDeterministicAcross2DJointRegistrationOrder()
    {
        ConstraintState first = RunConstraintChain(registerForward: true);
        ConstraintState second = RunConstraintChain(registerForward: false);

        second.Should().Be(first);
    }

    [Fact]
    public void RagdollFiltering_ShouldSuppressAdjacentLinksButAllowNonAdjacentByDefault()
    {
        using GravitasWorldContext context = CreateConstraintContext();
        SolidBody2D root = CreateBody(context, Vector2d.Zero);
        SolidBody2D middle = CreateBody(context, Vector2d.Right * (Fixed64)2);
        SolidBody2D end = CreateBody(context, Vector2d.Right * Fixed64.Half);
        RagdollRuntime2D runtime = context.Constraints2D.RegisterRagdoll(new RagdollDefinition2D(
            new[]
            {
                new RagdollLinkDefinition2D(0, root),
                new RagdollLinkDefinition2D(1, middle),
                new RagdollLinkDefinition2D(2, end)
            },
            new[]
            {
                new RagdollJointDefinition2D(0, 1, JointType2D.Pin, JointFrame2D.Identity, JointFrame2D.Identity),
                new RagdollJointDefinition2D(1, 2, JointType2D.Pin, JointFrame2D.Identity, JointFrame2D.Identity)
            }));

        runtime.LinkCount.Should().Be(3);
        runtime.JointCount.Should().Be(2);
        runtime.IsActive.Should().BeTrue();
        context.Constraints2D.EnabledJointCount.Should().Be(2);
        context.Constraints2D.ShouldExcludeLinkedCollision(root.Collider, middle.Collider).Should().BeTrue();
        context.Constraints2D.ShouldExcludeLinkedCollision(middle.Collider, end.Collider).Should().BeTrue();
        context.Constraints2D.ShouldExcludeLinkedCollision(root.Collider, end.Collider).Should().BeFalse();
    }

    [Fact]
    public void RagdollFiltering_WithSuppressAllPolicy_ShouldSuppressNonAdjacentLinks()
    {
        using GravitasWorldContext context = CreateConstraintContext();
        SolidBody2D root = CreateBody(context, Vector2d.Zero);
        SolidBody2D middle = CreateBody(context, Vector2d.Right * (Fixed64)2);
        SolidBody2D end = CreateBody(context, Vector2d.Right * Fixed64.Half);

        context.Constraints2D.RegisterRagdoll(new RagdollDefinition2D(
            new[]
            {
                new RagdollLinkDefinition2D(0, root),
                new RagdollLinkDefinition2D(1, middle),
                new RagdollLinkDefinition2D(2, end)
            },
            new[]
            {
                new RagdollJointDefinition2D(0, 1, JointType2D.Pin, JointFrame2D.Identity, JointFrame2D.Identity),
                new RagdollJointDefinition2D(1, 2, JointType2D.Pin, JointFrame2D.Identity, JointFrame2D.Identity)
            },
            RagdollSelfCollisionPolicy.SuppressAllLinks));

        context.Constraints2D.ShouldExcludeLinkedCollision(root.Collider, end.Collider).Should().BeTrue();
    }

    [Fact]
    public void RagdollLinkDefinition_ShouldDeriveColliderFromBodyAndRejectNullBody()
    {
        using GravitasWorldContext context = CreateConstraintContext();
        SolidBody2D body = CreateBody(context, Vector2d.Zero);

        var link = new RagdollLinkDefinition2D(7, body);
        Action nullBody = () => _ = new RagdollLinkDefinition2D(0, null!);

        link.LinkId.Should().Be(7);
        link.Body.Should().BeSameAs(body);
        link.Collider.Should().BeSameAs(body.Collider);
        nullBody.Should().Throw<ArgumentNullException>().WithParameterName("body");
    }

    [Fact]
    public void RegisterRagdoll_WithInvalidJointPayload_ShouldFailAtomically()
    {
        using GravitasWorldContext context = CreateConstraintContext();
        SolidBody2D root = CreateBody(context, Vector2d.Zero);
        SolidBody2D middle = CreateBody(context, Vector2d.Right * (Fixed64)2);
        SolidBody2D end = CreateBody(context, Vector2d.Right * (Fixed64)4);
        var definition = new RagdollDefinition2D(
            new[]
            {
                new RagdollLinkDefinition2D(0, root),
                new RagdollLinkDefinition2D(1, middle),
                new RagdollLinkDefinition2D(2, end)
            },
            new[]
            {
                new RagdollJointDefinition2D(0, 1, JointType2D.Pin, JointFrame2D.Identity, JointFrame2D.Identity),
                new RagdollJointDefinition2D(
                    1,
                    2,
                    JointType2D.Pin,
                    JointFrame2D.Identity,
                    JointFrame2D.Identity,
                    JointLimit2D.Unrestricted,
                    JointMotor2D.Linear(Fixed64.Zero, Fixed64.One, Fixed64.Zero, Fixed64.One),
                    JointCollisionPolicy.SuppressLinked)
            });

        Action act = () => context.Constraints2D.RegisterRagdoll(definition);

        act.Should().Throw<ArgumentException>();
        context.Constraints2D.RegisteredJointCount.Should().Be(0);
        context.Constraints2D.EnabledJointCount.Should().Be(0);
        context.Constraints2D.PeakJointCount.Should().Be(0);
        context.Constraints2D.RegisteredRagdollCount.Should().Be(0);
        context.Constraints2D.ShouldExcludeLinkedCollision(root.Collider, middle.Collider).Should().BeFalse();
        context.Constraints2D.ShouldExcludeLinkedCollision(middle.Collider, end.Collider).Should().BeFalse();
    }

    [Fact]
    public void RagdollRuntime_ShouldActivateDynamicAndDeactivateToKinematicDeterministically()
    {
        using GravitasWorldContext context = CreateConstraintContext();
        SolidBody2D root = CreateBody(context, Vector2d.Zero, isKinematic: true);
        SolidBody2D child = CreateBody(context, Vector2d.Right * (Fixed64)2, isKinematic: true);
        RagdollRuntime2D runtime = context.Constraints2D.RegisterRagdoll(CreateTwoLinkRagdoll(root, child));

        runtime.IsActive.Should().BeFalse();
        runtime.GetJoint(0).IsEnabled.Should().BeFalse();
        context.Constraints2D.EnabledJointCount.Should().Be(0);

        runtime.ActivateDynamic();

        runtime.IsActive.Should().BeTrue();
        root.IsKinematic.Should().BeFalse();
        child.IsKinematic.Should().BeFalse();
        runtime.GetJoint(0).IsEnabled.Should().BeTrue();
        context.Constraints2D.EnabledJointCount.Should().Be(1);

        runtime.DeactivateToKinematic();

        runtime.IsActive.Should().BeFalse();
        root.IsKinematic.Should().BeTrue();
        child.IsKinematic.Should().BeTrue();
        runtime.GetJoint(0).IsEnabled.Should().BeFalse();
        context.Constraints2D.EnabledJointCount.Should().Be(0);
    }

    [Fact]
    public void ConstraintServiceMotorHelpers_ShouldUpdateJointAndRagdollTargets()
    {
        using GravitasWorldContext context = CreateConstraintContext();
        SolidBody2D root = CreateBody(context, Vector2d.Zero, isKinematic: true);
        SolidBody2D child = CreateBody(context, Vector2d.Right * (Fixed64)2, isKinematic: true);
        RagdollRuntime2D ragdoll = context.Constraints2D.RegisterRagdoll(CreateTwoLinkRagdoll(root, child));
        Joint2D joint = ragdoll.GetJoint(0);
        JointMotor2D motor = JointMotor2D.Angular(Fixed64.FromFraction(1, 4), (Fixed64)2, Fixed64.Half, Fixed64.One);

        context.Constraints2D.SetRagdollPoseTargets(ragdoll, new[] { motor });
        context.Constraints2D.SetJointMotorTarget(joint.Id, Fixed64.FromFraction(1, 3)).Should().BeTrue();

        joint.Motor.Kind.Should().Be(JointMotorKind2D.Angular);
        joint.Motor.DriveStrength.Should().Be((Fixed64)2);
        joint.Motor.Damping.Should().Be(Fixed64.Half);
        joint.Motor.MaximumMotorImpulse.Should().Be(Fixed64.One);
        joint.Motor.Target.Should().Be(Fixed64.FromFraction(1, 3));

        context.Constraints2D.ClearJointMotorTarget(joint.Id).Should().BeTrue();
        joint.Motor.Kind.Should().Be(JointMotorKind2D.Disabled);
    }

    [Fact]
    public void SetRagdollPoseTargets_WithForeignRuntime_ShouldReject()
    {
        using GravitasWorldContext sourceContext = CreateConstraintContext();
        using GravitasWorldContext targetContext = CreateConstraintContext();
        SolidBody2D root = CreateBody(sourceContext, Vector2d.Zero, isKinematic: true);
        SolidBody2D child = CreateBody(sourceContext, Vector2d.Right * (Fixed64)2, isKinematic: true);
        RagdollRuntime2D ragdoll = sourceContext.Constraints2D.RegisterRagdoll(CreateTwoLinkRagdoll(root, child));
        JointMotor2D motor = JointMotor2D.Angular(Fixed64.Zero, Fixed64.One, Fixed64.Zero, Fixed64.One);

        Action act = () => targetContext.Constraints2D.SetRagdollPoseTargets(ragdoll, new[] { motor });

        act.Should().Throw<ArgumentException>()
            .WithParameterName("ragdoll");
    }

    [Theory]
    [MemberData(nameof(Transports))]
    public void JointRecordData_ShouldRoundTripAuthoritative2DState(GravitasSerializationTransport transport)
    {
        using GravitasWorldContext context = CreateConstraintContext();
        SolidBody2D first = CreateBody(context, Vector2d.Zero);
        SolidBody2D second = CreateBody(context, Vector2d.Right * (Fixed64)2);
        Joint2D source = context.Constraints2D.RegisterJoint(new JointDefinition2D(
            first,
            second,
            new JointFrame2D(Vector2d.Right, Fixed64.Half),
            new JointFrame2D(-Vector2d.Right, -Fixed64.Half),
            JointType2D.Prismatic,
            JointLimit2D.Slider(-Fixed64.One, Fixed64.One),
            JointMotor2D.Linear(Fixed64.Half, (Fixed64)3, Fixed64.Half, Fixed64.One),
            JointCollisionPolicy.Collide));
        source.IsEnabled = false;

        object payload = GravitasSerializationHarness.Serialize(source, transport);

        Joint2D target = context.Constraints2D.RegisterJoint(CreatePin(first, second));
        GravitasSerializationHarness.Populate(target, payload, transport);

        target.IsEnabled.Should().BeFalse();
        target.Type.Should().Be(JointType2D.Prismatic);
        target.LocalFrameA.Anchor.Should().Be(Vector2d.Right);
        target.LocalFrameB.Anchor.Should().Be(-Vector2d.Right);
        target.LocalFrameA.Angle.Should().Be(Fixed64.Half);
        target.Limits.Should().Be(source.Limits);
        target.Motor.Should().Be(source.Motor);
        target.CollisionPolicy.Should().Be(JointCollisionPolicy.Collide);
    }

    [Fact]
    public void JointRecordData_WithInvalidLoadedTypePayload_ShouldReject()
    {
        using GravitasWorldContext context = CreateConstraintContext();
        SolidBody2D first = CreateBody(context, Vector2d.Zero);
        SolidBody2D second = CreateBody(context, Vector2d.Right * (Fixed64)2);
        Joint2D target = context.Constraints2D.RegisterJoint(CreatePin(first, second));
        var chronicler = new InvalidJointPayloadChronicler(new Dictionary<string, object>
        {
            [nameof(Joint2D.IsEnabled)] = true,
            ["Type"] = JointType2D.Pin,
            ["LimitKind"] = JointLimitKind2D.Unrestricted,
            ["MotorKind"] = JointMotorKind2D.Linear,
            ["MotorTarget"] = Fixed64.Zero,
            ["MotorStrength"] = Fixed64.One,
            ["MotorDamping"] = Fixed64.Zero,
            ["MaxMotorImpulse"] = Fixed64.One,
            ["CollisionPolicy"] = JointCollisionPolicy.SuppressLinked,
            ["LocalFrameAAnchor"] = Vector2d.Zero,
            ["LocalFrameAAngle"] = Fixed64.Zero,
            ["LocalFrameBAnchor"] = Vector2d.Zero,
            ["LocalFrameBAngle"] = Fixed64.Zero
        });

        Action act = () => target.RecordData(chronicler);

        act.Should().Throw<ArgumentException>();
        target.Type.Should().Be(JointType2D.Pin);
        target.Motor.Kind.Should().Be(JointMotorKind2D.Disabled);
    }

    [Theory]
    [MemberData(nameof(Transports))]
    public void RagdollRecordData_ShouldApplyInactiveStateToActiveRuntime(GravitasSerializationTransport transport)
    {
        using GravitasWorldContext sourceContext = CreateConstraintContext();
        SolidBody2D sourceRoot = CreateBody(sourceContext, Vector2d.Zero);
        SolidBody2D sourceChild = CreateBody(sourceContext, Vector2d.Right * (Fixed64)2);
        RagdollRuntime2D source = sourceContext.Constraints2D.RegisterRagdoll(CreateTwoLinkRagdoll(sourceRoot, sourceChild));
        source.IsActive.Should().BeTrue();
        source.DeactivateToKinematic();
        object payload = GravitasSerializationHarness.Serialize(source, transport);

        using GravitasWorldContext targetContext = CreateConstraintContext();
        SolidBody2D targetRoot = CreateBody(targetContext, Vector2d.Zero);
        SolidBody2D targetChild = CreateBody(targetContext, Vector2d.Right * (Fixed64)2);
        RagdollRuntime2D target = targetContext.Constraints2D.RegisterRagdoll(CreateTwoLinkRagdoll(targetRoot, targetChild));
        target.IsActive.Should().BeTrue();

        GravitasSerializationHarness.Populate(target, payload, transport);

        target.IsActive.Should().BeFalse();
        targetRoot.IsKinematic.Should().BeTrue();
        targetChild.IsKinematic.Should().BeTrue();
        target.GetJoint(0).IsEnabled.Should().BeFalse();
        targetContext.Constraints2D.EnabledJointCount.Should().Be(0);
    }

    [Fact]
    public void DistanceJoint_WithCoincidentAnchorsAndPositiveTarget_ShouldSeparateDeterministically()
    {
        using GravitasWorldContext context = CreateConstraintContext();
        SolidBody2D first = CreateBody(context, Vector2d.Zero);
        SolidBody2D second = CreateBody(context, Vector2d.Zero);
        Joint2D joint = context.Constraints2D.RegisterJoint(new JointDefinition2D(
            first,
            second,
            JointFrame2D.Identity,
            JointFrame2D.Identity,
            JointType2D.Distance,
            JointLimit2D.Distance(Fixed64.One),
            JointMotor2D.Disabled,
            JointCollisionPolicy.SuppressLinked));

        Step(context, 8);

        (second.Position - first.Position).Magnitude.Should().BeGreaterThan(Fixed64.Zero);
        joint.LastSolvedRowCount.Should().BeGreaterThan(0);
        joint.AccumulatedImpulseMagnitude.Should().BeGreaterThan(Fixed64.Zero);
    }

    [Fact]
    public void WeldJoint_ShouldDampRelativeAngularVelocityAtZeroAngleError()
    {
        using GravitasWorldContext context = CreateConstraintContext();
        SolidBody2D anchor = CreateBody(context, Vector2d.Zero, immovable: true);
        SolidBody2D dynamic = CreateBody(context, Vector2d.Right * (Fixed64)2);
        Joint2D joint = context.Constraints2D.RegisterJoint(new JointDefinition2D(
            anchor,
            dynamic,
            JointFrame2D.Identity,
            JointFrame2D.Identity,
            JointType2D.Weld,
            JointLimit2D.Unrestricted,
            JointMotor2D.Disabled,
            JointCollisionPolicy.SuppressLinked));
        dynamic.AddAngularImpulse((Fixed64)4);
        Fixed64 before = dynamic.AngularVelocity.Abs();

        Step(context, 4);

        dynamic.AngularVelocity.Abs().Should().BeLessThan(before);
        joint.LastSolvedRowCount.Should().BeGreaterThan(0);
    }

    [Fact]
    public void AngularMotor_ShouldDampRelativeAngularVelocityAtTarget()
    {
        using GravitasWorldContext context = CreateConstraintContext();
        SolidBody2D anchor = CreateBody(context, Vector2d.Zero, immovable: true);
        SolidBody2D dynamic = CreateBody(context, Vector2d.Right * (Fixed64)2);
        Joint2D joint = context.Constraints2D.RegisterJoint(new JointDefinition2D(
            anchor,
            dynamic,
            JointFrame2D.Identity,
            JointFrame2D.Identity,
            JointType2D.Pin,
            JointLimit2D.Unrestricted,
            JointMotor2D.Angular(Fixed64.Zero, (Fixed64)2, Fixed64.One, Fixed64.One),
            JointCollisionPolicy.SuppressLinked));
        dynamic.AddAngularImpulse((Fixed64)4);
        Fixed64 before = dynamic.AngularVelocity.Abs();

        Step(context, 4);

        dynamic.AngularVelocity.Abs().Should().BeLessThan(before);
        joint.LastSolveMetrics.MotorImpulseMagnitude.Should().BeGreaterThan(Fixed64.Zero);
    }

    [Theory]
    [InlineData(-90)]
    [InlineData(90)]
    public void AngularLimit_ShouldReportLowerAndUpperViolations(int degrees)
    {
        using GravitasWorldContext context = CreateConstraintContext();
        SolidBody2D anchor = CreateBody(context, Vector2d.Zero, immovable: true);
        SolidBody2D dynamic = CreateBody(
            context,
            Vector2d.Right * (Fixed64)2,
            rotation: FixedMath.DegToRad((Fixed64)degrees));
        Joint2D joint = context.Constraints2D.RegisterJoint(new JointDefinition2D(
            anchor,
            dynamic,
            JointFrame2D.Identity,
            JointFrame2D.Identity,
            JointType2D.Pin,
            JointLimit2D.Angular(-Fixed64.Half, Fixed64.Half),
            JointMotor2D.Disabled,
            JointCollisionPolicy.SuppressLinked));

        Step(context, 2);

        joint.LastSolveMetrics.LimitErrorMagnitude.Should().BeGreaterThan(Fixed64.Zero);
        joint.LastSolveMetrics.ClampedRowCount.Should().BeGreaterThan(0);
    }

    [Fact]
    public void PrismaticLinearMotor_ShouldDriveAlongSliderAxis()
    {
        using GravitasWorldContext context = CreateConstraintContext();
        SolidBody2D anchor = CreateBody(context, Vector2d.Zero, immovable: true);
        SolidBody2D dynamic = CreateBody(context, Vector2d.Right * Fixed64.Half);
        Joint2D joint = context.Constraints2D.RegisterJoint(new JointDefinition2D(
            anchor,
            dynamic,
            JointFrame2D.Identity,
            JointFrame2D.Identity,
            JointType2D.Prismatic,
            JointLimit2D.Unrestricted,
            JointMotor2D.Linear((Fixed64)2, (Fixed64)2, Fixed64.Half, Fixed64.One),
            JointCollisionPolicy.SuppressLinked));
        Fixed64 before = dynamic.Position.X;

        Step(context, 6);

        dynamic.Position.X.Should().BeGreaterThan(before);
        joint.LastSolveMetrics.MotorErrorMagnitude.Should().BeGreaterThan(Fixed64.Zero);
        joint.LastSolveMetrics.MotorImpulseMagnitude.Should().BeGreaterThan(Fixed64.Zero);
    }

    [Fact]
    public void ContinuousCollision_ShouldSuppressAdjacentLinked2DSelfHitButClipExternalBlockers()
    {
        using GravitasWorldContext context = CreateConstraintContext(frameRate: 1);
        SolidBody2D source = CreateBody(context, new Vector2d((Fixed64)(-4), Fixed64.Zero));
        SolidBody2D linked = CreateBody(context, Vector2d.Zero);
        SolidBody2D blocker = CreateBody(context, Vector2d.Right * (Fixed64)4, immovable: true);
        context.Constraints2D.RegisterJoint(CreatePin(source, linked));
        source.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;

        source.AddForce(Vector2d.Right * (Fixed64)10);
        context.LateSimulate();

        source.Position.X.Should().BeLessThan(blocker.Position.X);
        source.Collider.TryGetCollisionPair(linked.Collider.Id, out _).Should().BeFalse();
    }

    [Fact]
    public void EnabledDiagnostics_ShouldRecord2DJointLifecycleAndDrawEvents()
    {
        using GravitasWorldContext context = CreateConstraintContext();
        SolidBody2D first = CreateBody(context, Vector2d.Zero);
        SolidBody2D second = CreateBody(context, Vector2d.Right * (Fixed64)3);
        context.Diagnostics.Enable(eventCapacity: 16, drawCommandCapacity: 16);

        Joint2D joint = context.Constraints2D.RegisterJoint(CreatePin(first, second));
        context.Diagnostics.CaptureJoint(joint, GravitasDiagnosticColor.Cyan);
        Step(context, 4);
        context.Constraints2D.RemoveJoint(joint.Id);

        ReadOnlySpan<GravitasDiagnosticEvent> events = context.Diagnostics.Events;
        events[0].Kind.Should().Be(GravitasDiagnosticEventKind.JointRegistered);
        events.Should().Contain(e => e.Kind == GravitasDiagnosticEventKind.JointImpulse && e.JointId == joint.Id);
        events[^1].Kind.Should().Be(GravitasDiagnosticEventKind.JointRemoved);
        context.Diagnostics.DrawCommandCount.Should().BeGreaterThan(0);
    }

    [Fact]
    public void ConstraintFilteringAndDisabledDiagnostics_ShouldNotAllocateAfterWarmup()
    {
        using GravitasWorldContext context = CreateConstraintContext();
        SolidBody2D first = CreateBody(context, Vector2d.Zero);
        SolidBody2D second = CreateBody(context, Vector2d.Right * (Fixed64)3);
        context.Constraints2D.RegisterJoint(CreatePin(first, second));
        Step(context, 8);
        bool linkedFilterResult = false;

        long allocatedBytes = AllocationTestHelper.MeasureSteadyState(
            () =>
            {
                linkedFilterResult = context.Constraints2D.ShouldExcludeLinkedCollision(first.Collider, second.Collider);
                context.Simulate();
                context.LateSimulate();
            },
            warmupIterations: 8,
            stabilizationIterations: 4,
            measurementIterations: 8);

        linkedFilterResult.Should().BeTrue();
        allocatedBytes.Should().Be(0);
    }

    private static GravitasWorldContext CreateConstraintContext(int frameRate = 4)
    {
        GravitasWorldContext context = Physics2DTestWorld.CreateContext(frameRate);
        context.Environment.Gravity = Fixed64.Zero;
        context.Environment.AirDensity = Fixed64.Zero;
        context.Environment.DampingFactor = Fixed64.Zero;
        context.Settings.DiscreteSolverIterations = 8;
        return context;
    }

    private static SolidBody2D CreateBody(
        GravitasWorldContext context,
        Vector2d position,
        Fixed64 rotation = default,
        bool immovable = false,
        bool isKinematic = false)
    {
        var transform = new FixedTransform(
            new Vector3d(position.X, Fixed64.Zero, position.Y),
            FixedQuaternion.FromEulerAnglesInDegrees(Fixed64.Zero, FixedMath.RadToDeg(rotation), Fixed64.Zero),
            Vector3d.One);
        var agent = new TestMatterAgent(context, transform);
        var body = new SolidBody2D(agent, new LSCircleCollider2D(Fixed64.Half))
        {
            Mass = Fixed64.One,
            FreezeAxes = immovable ? BodyFreezeAxes2D.Position : BodyFreezeAxes2D.None,
            IsKinematic = isKinematic
        };
        body.Initialize(position, rotation);
        return body;
    }

    private static JointDefinition2D CreatePin(
        SolidBody2D first,
        SolidBody2D second,
        JointCollisionPolicy collisionPolicy = JointCollisionPolicy.SuppressLinked) =>
        new(
            first,
            second,
            JointFrame2D.Identity,
            JointFrame2D.Identity,
            JointType2D.Pin,
            JointLimit2D.Unrestricted,
            JointMotor2D.Disabled,
            collisionPolicy);

    private static RagdollDefinition2D CreateTwoLinkRagdoll(SolidBody2D root, SolidBody2D child) =>
        new(
            new[]
            {
                new RagdollLinkDefinition2D(0, root),
                new RagdollLinkDefinition2D(1, child)
            },
            new[]
            {
                new RagdollJointDefinition2D(0, 1, JointType2D.Pin, JointFrame2D.Identity, JointFrame2D.Identity)
            });

    private static void Step(GravitasWorldContext context, int frames = 1)
    {
        for (int i = 0; i < frames; i++)
        {
            context.Simulate();
            context.LateSimulate();
        }
    }

    private static Fixed64 AnchorDistanceSquared(SolidBody2D first, SolidBody2D second, Joint2D joint)
    {
        Vector2d anchorA = first.Position + Vector2d.Rotate(joint.LocalFrameA.Anchor, first.Rotation);
        Vector2d anchorB = second.Position + Vector2d.Rotate(joint.LocalFrameB.Anchor, second.Rotation);
        return (anchorB - anchorA).MagnitudeSquared;
    }

    private static ConstraintState RunConstraintChain(bool registerForward)
    {
        using GravitasWorldContext context = CreateConstraintContext();
        SolidBody2D first = CreateBody(context, Vector2d.Zero);
        SolidBody2D second = CreateBody(context, Vector2d.Right * (Fixed64)3);
        SolidBody2D third = CreateBody(context, Vector2d.Right * (Fixed64)6);
        if (registerForward)
        {
            context.Constraints2D.RegisterJoint(CreatePin(first, second));
            context.Constraints2D.RegisterJoint(CreatePin(second, third));
        }
        else
        {
            context.Constraints2D.RegisterJoint(CreatePin(second, third));
            context.Constraints2D.RegisterJoint(CreatePin(first, second));
        }

        Step(context, 12);
        Joint2D firstJoint = context.Constraints2D.GetJoint(1);
        Joint2D secondJoint = context.Constraints2D.GetJoint(2);
        return new ConstraintState(
            first.Position,
            third.Position,
            first.LinearVelocity,
            third.LinearVelocity,
            (third.Position - first.Position).MagnitudeSquared,
            firstJoint.LastSolvedRowCount + secondJoint.LastSolvedRowCount,
            firstJoint.AccumulatedImpulseMagnitude + secondJoint.AccumulatedImpulseMagnitude);
    }

    private readonly record struct ConstraintState(
        Vector2d FirstPosition,
        Vector2d SecondPosition,
        Vector2d FirstVelocity,
        Vector2d SecondVelocity,
        Fixed64 AnchorDistanceSquared,
        int JointSolvedRowCount,
        Fixed64 AccumulatedImpulseMagnitude);

    private sealed class InvalidJointPayloadChronicler : IChronicler
    {
        private readonly IReadOnlyDictionary<string, object> _values;

        public InvalidJointPayloadChronicler(IReadOnlyDictionary<string, object> values)
        {
            _values = values;
            Context = new ChronicleContext();
        }

        public ChronicleContext Context { get; }

        public SerializationMode Mode => SerializationMode.Loading;

        public void LookValue<T>(ref T value, string name, T? defaultValue = default)
        {
            if (_values.TryGetValue(name, out object? loadedValue))
                value = (T)loadedValue;
        }

        public void LookDeep<T>(ref T value, string name) where T : class, IRecordable
        {
        }

        public void LookDeepStruct<T>(ref T value, string name) where T : struct, IRecordable
        {
        }

        public void LookNullableDeep<T>(ref T? value, string name) where T : struct, IRecordable
        {
        }

        public void LookLink<T>(
            ref T value,
            string name,
            string? slot = null,
            RecordLinkResolveMode resolveMode = RecordLinkResolveMode.Immediate,
            Action<T>? assignLoadedValue = null)
        {
        }
    }
}
