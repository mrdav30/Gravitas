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
    public void Joint2D_IsSolverBody_ShouldRequireActiveRegisteredSolverMobility()
    {
        using GravitasWorldContext context = CreateConstraintContext();
        SolidBody2D active = CreateBody(context, Vector2d.Zero);
        SolidBody2D frozen = CreateBody(context, Vector2d.Right * (Fixed64)2);
        SolidBody2D locked = CreateBody(context, Vector2d.Right * (Fixed64)3);
        SolidBody2D inactive = CreateBody(context, Vector2d.Right * (Fixed64)4);
        frozen.FreezeAxes = BodyFreezeAxes2D.Position;
        locked.FreezeAxes = BodyFreezeAxes2D.All;

        inactive.Deactivate();

        active.Active.Should().BeTrue();
        active.DynamicId.Should().BeGreaterThanOrEqualTo(0);
        active.IsDynamic.Should().BeTrue();
        active.CanTranslate.Should().BeTrue();
        Joint2D.IsSolverBody(active).Should().BeTrue();
        Joint2D.IsSolverBody(frozen).Should().BeTrue();
        Joint2D.IsSolverBody(locked).Should().BeFalse();
        Joint2D.IsSolverBody(inactive).Should().BeFalse();
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
    public void MotionTypeChange_ShouldClearAttachedJointSolverCacheWithoutRemovingJoint()
    {
        using GravitasWorldContext context = CreateConstraintContext();
        SolidBody2D first = CreateBody(context, Vector2d.Zero);
        SolidBody2D second = CreateBody(context, Vector2d.Right * (Fixed64)2);
        Joint2D joint = context.Constraints2D.RegisterJoint(CreatePin(first, second));
        joint.AccumulatedImpulseMagnitude = Fixed64.One;

        first.SetMotionType(BodyMotionType.Kinematic);

        joint.AccumulatedImpulseMagnitude.Should().Be(Fixed64.Zero);
        context.Constraints2D.RegisteredJointCount.Should().Be(1);
        context.Constraints2D.TryGetJoint(joint.Id, out Joint2D? retained).Should().BeTrue();
        retained.Should().BeSameAs(joint);
    }

    [Fact]
    public void RegisterJoint_BeyondDefaultCapacity_ShouldGrowAndResetDeterministically()
    {
        using GravitasWorldContext context = CreateConstraintContext();
        SolidBody2D first = CreateBody(context, Vector2d.Zero);
        SolidBody2D second = CreateBody(context, Vector2d.Right * (Fixed64)2);

        Joint2D? lastJoint = null;
        for (int i = 0; i < 70; i++)
            lastJoint = context.Constraints2D.RegisterJoint(CreatePin(first, second));

        context.Constraints2D.RemoveJoint(2).Should().BeTrue();
        context.Constraints2D.RegisteredJointCount.Should().Be(69);
        context.Constraints2D.PeakJointCount.Should().Be(70);
        context.Constraints2D.TryGetJoint(lastJoint!.Id, out Joint2D? resolved).Should().BeTrue();
        resolved.Should().BeSameAs(lastJoint);

        context.Reset();

        context.Constraints2D.RegisteredJointCount.Should().Be(0);
        context.Constraints2D.PeakJointCount.Should().Be(0);
        context.Constraints2D.TryGetJoint(lastJoint.Id, out _).Should().BeFalse();
        context.Constraints2D.ShouldExcludeLinkedCollision(first.Collider, second.Collider).Should().BeFalse();
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
        SolidBody2D inactive = CreateBody(context, Vector2d.Right * (Fixed64)6);
        inactive.Deactivate();
        Action inactiveBody = () => context.Constraints2D.RegisterJoint(CreatePin(body, inactive));
        SolidBody2D inactiveFirst = CreateBody(context, Vector2d.Right * (Fixed64)7);
        inactiveFirst.Deactivate();
        Action inactiveFirstBody = () => context.Constraints2D.RegisterJoint(CreatePin(
            inactiveFirst,
            CreateBody(context, Vector2d.Right * (Fixed64)7 + Vector2d.Forward)));
        Action invalidCollisionPolicy = () => context.Constraints2D.RegisterJoint(CreatePin(
            body,
            CreateBody(context, Vector2d.Right * (Fixed64)8),
            (JointCollisionPolicy)255));

        sameBody.Should().Throw<ArgumentException>();
        invalidType.Should().Throw<ArgumentException>();
        invalidLimit.Should().Throw<ArgumentException>();
        inactiveBody.Should().Throw<ArgumentException>();
        inactiveFirstBody.Should().Throw<ArgumentException>();
        invalidCollisionPolicy.Should().Throw<ArgumentException>();
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
        Action reversed = () => firstContext.Constraints2D.RegisterJoint(CreatePin(second, first));

        act.Should().Throw<ArgumentException>();
        reversed.Should().Throw<ArgumentException>();
    }

    [Fact]
    public void RemoveJoint_WithDisabledJoint_ShouldUpdateEnabledCountOnlyOnce()
    {
        using GravitasWorldContext context = CreateConstraintContext();
        SolidBody2D first = CreateBody(context, Vector2d.Zero);
        SolidBody2D second = CreateBody(context, Vector2d.Right * (Fixed64)2);
        Joint2D joint = context.Constraints2D.RegisterJoint(CreatePin(first, second));

        joint.IsEnabled = false;
        bool removed = context.Constraints2D.RemoveJoint(joint.Id);
        bool removedAgain = context.Constraints2D.RemoveJoint(joint.Id);

        removed.Should().BeTrue();
        removedAgain.Should().BeFalse();
        context.Constraints2D.RegisteredJointCount.Should().Be(0);
        context.Constraints2D.EnabledJointCount.Should().Be(0);
    }

    [Fact]
    public void ConstraintServicePolicyHelpers_ShouldRejectInvalid2DFilterInputsAndInactivePolicyChanges()
    {
        using GravitasWorldContext context = CreateConstraintContext();
        SolidBody2D first = CreateBody(context, Vector2d.Zero);
        SolidBody2D second = CreateBody(context, Vector2d.Right * (Fixed64)2);
        Joint2D joint = context.Constraints2D.RegisterJoint(CreatePin(first, second));
        var unregistered = new LSCircleCollider2D(Fixed64.Half);

        context.Constraints2D.ShouldExcludeLinkedCollision(null!, second.Collider).Should().BeFalse();
        context.Constraints2D.ShouldExcludeLinkedCollision(first.Collider, null!).Should().BeFalse();
        context.Constraints2D.ShouldExcludeLinkedCollision(first.Collider, first.Collider).Should().BeFalse();
        context.Constraints2D.ShouldExcludeLinkedCollision(unregistered, second.Collider).Should().BeFalse();
        context.Constraints2D.ShouldExcludeLinkedCollision(first.Collider, unregistered).Should().BeFalse();

        context.Constraints2D.UpdateJointCollisionPolicy(
            joint,
            JointCollisionPolicy.SuppressLinked,
            JointCollisionPolicy.SuppressLinked);
        context.Constraints2D.ShouldExcludeLinkedCollision(first.Collider, second.Collider).Should().BeTrue();

        context.Constraints2D.RemoveJoint(joint.Id).Should().BeTrue();
        joint.SetCollisionPolicyFromRecord(JointCollisionPolicy.Collide);
        joint.SetCollisionPolicyFromRecord(JointCollisionPolicy.SuppressLinked);
        context.Constraints2D.UpdateJointEnabledState(joint, true, false);

        context.Constraints2D.ShouldExcludeLinkedCollision(first.Collider, second.Collider).Should().BeFalse();
    }

    [Fact]
    public void ConstraintServiceSolverLookup_ShouldHonor2DEnabledAndMobilityState()
    {
        using GravitasWorldContext context = CreateConstraintContext();
        SolidBody2D first = CreateBody(context, Vector2d.Zero);
        SolidBody2D second = CreateBody(context, Vector2d.Right * (Fixed64)2);
        Joint2D joint = context.Constraints2D.RegisterJoint(CreatePin(first, second));
        Action missingGet = () => context.Constraints2D.GetJoint(99);

        missingGet.Should().Throw<ArgumentOutOfRangeException>().WithParameterName("jointId");
        context.Constraints2D.TryGetJointForSolver(99, out Joint2D? missing).Should().BeFalse();
        missing.Should().BeNull();

        joint.IsEnabled = false;
        context.Constraints2D.TryGetJointForSolver(joint.Id, out Joint2D? disabled).Should().BeFalse();
        disabled.Should().BeNull();

        joint.IsEnabled = true;
        context.Constraints2D.TryGetJointForSolver(joint.Id, out Joint2D? enabled).Should().BeTrue();
        enabled.Should().BeSameAs(joint);

        first.FreezeAxes = BodyFreezeAxes2D.All;
        second.FreezeAxes = BodyFreezeAxes2D.All;
        context.Constraints2D.TryGetJointForSolver(joint.Id, out Joint2D? frozen).Should().BeFalse();
        frozen.Should().BeNull();
    }

    [Fact]
    public void CollisionPolicyRecordUpdate_ShouldAddAndRemoveLinked2DCollisionSuppression()
    {
        using GravitasWorldContext context = CreateConstraintContext();
        SolidBody2D first = CreateBody(context, Vector2d.Zero);
        SolidBody2D second = CreateBody(context, Vector2d.Right * (Fixed64)2);
        Joint2D joint = context.Constraints2D.RegisterJoint(CreatePin(first, second));

        context.Constraints2D.ShouldExcludeLinkedCollision(first.Collider, second.Collider).Should().BeTrue();

        joint.SetCollisionPolicyFromRecord(JointCollisionPolicy.Collide);

        joint.CollisionPolicy.Should().Be(JointCollisionPolicy.Collide);
        context.Constraints2D.ShouldExcludeLinkedCollision(first.Collider, second.Collider).Should().BeFalse();

        joint.SetCollisionPolicyFromRecord(JointCollisionPolicy.SuppressLinked);

        joint.CollisionPolicy.Should().Be(JointCollisionPolicy.SuppressLinked);
        context.Constraints2D.ShouldExcludeLinkedCollision(first.Collider, second.Collider).Should().BeTrue();
    }

    [Fact]
    public void RemoveJoint_WithCollidePolicy_ShouldLeave2DSuppressionStateUntouched()
    {
        using GravitasWorldContext context = CreateConstraintContext();
        SolidBody2D first = CreateBody(context, Vector2d.Zero);
        SolidBody2D second = CreateBody(context, Vector2d.Right * (Fixed64)2);
        Joint2D joint = context.Constraints2D.RegisterJoint(CreatePin(
            first,
            second,
            JointCollisionPolicy.Collide));

        context.Constraints2D.ShouldExcludeLinkedCollision(first.Collider, second.Collider).Should().BeFalse();
        context.Constraints2D.UpdateJointCollisionPolicy(
            joint,
            JointCollisionPolicy.SuppressLinked,
            JointCollisionPolicy.Collide);

        context.Constraints2D.RemoveJoint(joint.Id).Should().BeTrue();

        context.Constraints2D.ShouldExcludeLinkedCollision(first.Collider, second.Collider).Should().BeFalse();
    }

    [Fact]
    public void RemoveJoint_WithDuplicateSuppressingJoints_ShouldKeep2DSuppressionUntilLastJointIsRemoved()
    {
        using GravitasWorldContext context = CreateConstraintContext();
        SolidBody2D first = CreateBody(context, Vector2d.Zero);
        SolidBody2D second = CreateBody(context, Vector2d.Right * (Fixed64)2);
        JointDefinition2D definition = CreatePin(first, second);
        Joint2D firstJoint = context.Constraints2D.RegisterJoint(definition);
        Joint2D secondJoint = context.Constraints2D.RegisterJoint(definition);

        context.Constraints2D.ShouldExcludeLinkedCollision(first.Collider, second.Collider).Should().BeTrue();

        context.Constraints2D.RemoveJoint(firstJoint.Id).Should().BeTrue();

        context.Constraints2D.ShouldExcludeLinkedCollision(first.Collider, second.Collider).Should().BeTrue();

        context.Constraints2D.RemoveJoint(secondJoint.Id).Should().BeTrue();

        context.Constraints2D.ShouldExcludeLinkedCollision(first.Collider, second.Collider).Should().BeFalse();
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
    public void LinkedCollisionSuppression_ShouldBeRemovedWhenLargerColliderIdIsReused()
    {
        using GravitasWorldContext context = CreateConstraintContext();
        SolidBody2D first = CreateBody(context, Vector2d.Zero);
        SolidBody2D second = CreateBody(context, Vector2d.Right * (Fixed64)2);
        context.Constraints2D.RegisterJoint(CreatePin(first, second));
        int secondColliderId = second.Collider.Id;

        first.Collider.Id.Should().BeLessThan(secondColliderId);
        second.Deactivate();
        SolidBody2D replacement = CreateBody(context, Vector2d.Forward * (Fixed64)4);

        replacement.Collider.Id.Should().Be(secondColliderId);
        context.Constraints2D.ShouldExcludeLinkedCollision(first.Collider, replacement.Collider).Should().BeFalse();
    }

    [Theory]
    [InlineData(true)]
    [InlineData(false)]
    public void DeactivateEndpoint_ShouldRemoveEveryAttached2DJointBeforeSameShellReinitialization(bool deactivateFirst)
    {
        using GravitasWorldContext context = CreateConstraintContext();
        SolidBody2D first = CreateBody(context, Vector2d.Zero);
        SolidBody2D second = CreateBody(context, Vector2d.Right * (Fixed64)2);
        JointDefinition2D definition = CreatePin(first, second);
        Joint2D firstJoint = context.Constraints2D.RegisterJoint(definition);
        Joint2D secondJoint = context.Constraints2D.RegisterJoint(definition);
        SolidBody2D endpoint = deactivateFirst ? first : second;

        endpoint.Deactivate();

        firstJoint.IsActive.Should().BeFalse();
        secondJoint.IsActive.Should().BeFalse();
        context.Constraints2D.RegisteredJointCount.Should().Be(0);
        context.Constraints2D.EnabledJointCount.Should().Be(0);
        context.Constraints2D.TryGetJointForSolver(firstJoint.Id, out _).Should().BeFalse();
        context.Constraints2D.TryGetJointForSolver(secondJoint.Id, out _).Should().BeFalse();
        Action mutateRemovedJoint = () => firstJoint.IsEnabled = false;
        Action clearRemovedMotor = firstJoint.ClearMotor;
        Action serializeRemovedJoint = () => GravitasSerializationHarness.Serialize(
            firstJoint,
            GravitasSerializationTransport.Json);

        mutateRemovedJoint.Should().Throw<InvalidOperationException>();
        clearRemovedMotor.Should().Throw<InvalidOperationException>();
        serializeRemovedJoint.Should().Throw<InvalidOperationException>();

        endpoint.Initialize(Vector2d.Forward * (Fixed64)4);

        context.Constraints2D.RegisteredJointCount.Should().Be(0);
        context.Constraints2D.ShouldExcludeLinkedCollision(first.Collider, second.Collider).Should().BeFalse();
    }

    [Fact]
    public void Deactivated2DCollider_WhenReboundToDifferentBody_ShouldNotRetargetRemovedJoint()
    {
        using GravitasWorldContext context = CreateConstraintContext();
        SolidBody2D first = CreateBody(context, Vector2d.Zero);
        SolidBody2D second = CreateBody(context, Vector2d.Right * (Fixed64)2);
        Joint2D joint = context.Constraints2D.RegisterJoint(CreatePin(first, second));
        LSCircleCollider2D reboundCollider = (LSCircleCollider2D)second.Collider;

        second.Deactivate();
        Vector2d reboundPosition = Vector2d.Forward * (Fixed64)4;
        var reboundBody = new SolidBody2D(
            new TestMatterAgent(
                context,
                new FixedTransform(
                    new Vector3d(reboundPosition.X, Fixed64.Zero, reboundPosition.Y),
                    FixedQuaternion.Identity,
                    Vector3d.One)),
            reboundCollider);
        reboundBody.Initialize(reboundPosition);

        joint.IsActive.Should().BeFalse();
        joint.BodyB.Should().BeSameAs(second);
        reboundCollider.Body.Should().BeSameAs(reboundBody);
        context.Constraints2D.RegisteredJointCount.Should().Be(0);
        context.Constraints2D.TryGetJointForSolver(joint.Id, out _).Should().BeFalse();
        context.Constraints2D.ShouldExcludeLinkedCollision(first.Collider, reboundCollider).Should().BeFalse();
    }

    [Fact]
    public void EndpointTeardown_ShouldMatchExplicit2DJointRemovalReplayState()
    {
        using GravitasWorldContext automatic = CreateConstraintContext();
        SolidBody2D automaticFirst = CreateBody(automatic, Vector2d.Zero);
        SolidBody2D automaticSecond = CreateBody(automatic, Vector2d.Right * (Fixed64)2);
        automatic.Constraints2D.RegisterJoint(CreatePin(automaticFirst, automaticSecond));

        using GravitasWorldContext explicitRemoval = CreateConstraintContext();
        SolidBody2D explicitFirst = CreateBody(explicitRemoval, Vector2d.Zero);
        SolidBody2D explicitSecond = CreateBody(explicitRemoval, Vector2d.Right * (Fixed64)2);
        Joint2D explicitJoint = explicitRemoval.Constraints2D.RegisterJoint(CreatePin(explicitFirst, explicitSecond));

        automaticSecond.Deactivate();
        explicitRemoval.Constraints2D.RemoveJoint(explicitJoint.Id).Should().BeTrue();
        explicitSecond.Deactivate();

        automatic.ComputeReplayHash().Should().Be(explicitRemoval.ComputeReplayHash());
        automatic.ComputeReplayHash(GravitasReplayHashMode.AuthoritativeWithSolverCaches)
            .Should()
            .Be(explicitRemoval.ComputeReplayHash(GravitasReplayHashMode.AuthoritativeWithSolverCaches));
    }

    [Fact]
    public void EndpointTeardown_AfterUnlinkingEarlier2DJoint_ShouldRemoveRemainingJointsInReverseRegistrationOrder()
    {
        using GravitasWorldContext context = CreateConstraintContext();
        SolidBody2D center = CreateBody(context, Vector2d.Zero);
        var spokes = new[]
        {
            CreateBody(context, Vector2d.Right * (Fixed64)2),
            CreateBody(context, Vector2d.Forward * (Fixed64)2),
            CreateBody(context, Vector2d.Left * (Fixed64)2),
            CreateBody(context, -Vector2d.Forward * (Fixed64)2)
        };
        var joints = new Joint2D[spokes.Length];
        context.Diagnostics.Enable(eventCapacity: 32, drawCommandCapacity: 0);
        for (int i = 0; i < spokes.Length; i++)
            joints[i] = context.Constraints2D.RegisterJoint(CreatePin(center, spokes[i]));

        spokes[0].Deactivate();
        context.Diagnostics.Clear();

        center.Deactivate();

        var removedJointIds = new List<int>();
        foreach (GravitasDiagnosticEvent diagnosticEvent in context.Diagnostics.Events)
        {
            if (diagnosticEvent.Kind == GravitasDiagnosticEventKind.JointRemoved)
                removedJointIds.Add(diagnosticEvent.JointId);
        }

        removedJointIds.Should().Equal(joints[3].Id, joints[2].Id, joints[1].Id);
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
    public void PrismaticJoint_WithinSliderLimits_ShouldSkipLimitCorrection()
    {
        using GravitasWorldContext context = CreateConstraintContext();
        SolidBody2D first = CreateBody(context, Vector2d.Zero, immovable: true);
        SolidBody2D second = CreateBody(context, Vector2d.Right);
        Joint2D joint = context.Constraints2D.RegisterJoint(new JointDefinition2D(
            first,
            second,
            JointFrame2D.Identity,
            JointFrame2D.Identity,
            JointType2D.Prismatic,
            JointLimit2D.Slider(Fixed64.Zero, (Fixed64)2),
            JointMotor2D.Disabled,
            JointCollisionPolicy.SuppressLinked));

        Step(context, 2);

        joint.LastSolveMetrics.LimitErrorMagnitude.Should().Be(Fixed64.Zero);
        second.Position.X.Should().Be(Fixed64.One);
    }

    [Fact]
    public void AngularLimit_WithinRange_ShouldSkipLimitCorrection()
    {
        using GravitasWorldContext context = CreateConstraintContext();
        SolidBody2D first = CreateBody(context, Vector2d.Zero, immovable: true);
        SolidBody2D second = CreateBody(
            context,
            Vector2d.Right * (Fixed64)2,
            rotation: Fixed64.FromFraction(1, 4));
        Joint2D joint = context.Constraints2D.RegisterJoint(new JointDefinition2D(
            first,
            second,
            JointFrame2D.Identity,
            JointFrame2D.Identity,
            JointType2D.Pin,
            JointLimit2D.Angular(-Fixed64.Half, Fixed64.Half),
            JointMotor2D.Disabled,
            JointCollisionPolicy.SuppressLinked));

        Step(context, 2);

        joint.LastSolveMetrics.LimitErrorMagnitude.Should().Be(Fixed64.Zero);
        joint.LastSolvedRowCount.Should().BeGreaterThan(0);
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
    public void ConstraintIsland_WithOnlySleepingBodies_ShouldSkipSolvingAndRemainUnchanged()
    {
        using GravitasWorldContext context = CreateConstraintContext();
        SolidBody2D first = CreateBody(context, Vector2d.Zero);
        SolidBody2D second = CreateBody(context, Vector2d.Right * (Fixed64)3);
        Joint2D joint = context.Constraints2D.RegisterJoint(CreatePin(first, second));
        first.Sleep();
        second.Sleep();
        Vector2d firstPosition = first.Position;
        Vector2d secondPosition = second.Position;

        Step(context);

        first.IsSleeping.Should().BeTrue();
        second.IsSleeping.Should().BeTrue();
        first.Position.Should().Be(firstPosition);
        second.Position.Should().Be(secondPosition);
        first.LinearVelocity.Should().Be(Vector2d.Zero);
        second.LinearVelocity.Should().Be(Vector2d.Zero);
        joint.LastSolvedRowCount.Should().Be(0);
        joint.AccumulatedImpulseMagnitude.Should().Be(Fixed64.Zero);
    }

    [Fact]
    public void ConstraintIsland_WithSparseJointIdsAndMovableBodyA_ShouldSolveSurvivingJoint()
    {
        using GravitasWorldContext context = CreateConstraintContext();
        SolidBody2D removedFirst = CreateBody(context, new Vector2d((Fixed64)(-8), Fixed64.Zero));
        SolidBody2D removedSecond = CreateBody(context, new Vector2d((Fixed64)(-6), Fixed64.Zero));
        Joint2D removed = context.Constraints2D.RegisterJoint(CreatePin(removedFirst, removedSecond));
        SolidBody2D movable = CreateBody(context, Vector2d.Zero);
        SolidBody2D anchor = CreateBody(context, Vector2d.Right * (Fixed64)3, immovable: true);
        Joint2D surviving = context.Constraints2D.RegisterJoint(CreatePin(movable, anchor));
        Vector2d anchorPosition = anchor.Position;
        Vector2d movablePosition = movable.Position;

        context.Constraints2D.RemoveJoint(removed.Id).Should().BeTrue();
        Step(context, 2);

        context.Constraints2D.RegisteredJointCount.Should().Be(1);
        context.Constraints2D.GetJoint(surviving.Id).Should().BeSameAs(surviving);
        movable.Position.Should().NotBe(movablePosition);
        anchor.Position.Should().Be(anchorPosition);
        surviving.LastSolvedRowCount.Should().BeGreaterThan(0);
        surviving.AccumulatedImpulseMagnitude.Should().BeGreaterThan(Fixed64.Zero);
    }

    [Fact]
    public void ConstraintSolver_WithOnlyFrozen2DBodies_ShouldKeepJointRegisteredWithoutRows()
    {
        using GravitasWorldContext context = CreateConstraintContext();
        SolidBody2D first = CreateBody(context, Vector2d.Zero);
        SolidBody2D second = CreateBody(context, Vector2d.Right * (Fixed64)2);
        first.FreezeAxes = BodyFreezeAxes2D.All;
        second.FreezeAxes = BodyFreezeAxes2D.All;
        Joint2D joint = context.Constraints2D.RegisterJoint(CreatePin(first, second));

        Step(context);

        joint.IsEnabled.Should().BeTrue();
        context.Constraints2D.EnabledJointCount.Should().Be(1);
        joint.LastSolvedRowCount.Should().Be(0);
        joint.AccumulatedImpulseMagnitude.Should().Be(Fixed64.Zero);
    }

    [Fact]
    public void ConstraintSolveOrder_ShouldBeDeterministicAcross2DJointRegistrationOrder()
    {
        ConstraintState first = RunConstraintChain(registerForward: true);
        ConstraintState second = RunConstraintChain(registerForward: false);

        second.Should().Be(first);
    }

    [Fact]
    public void ConstraintIsland_WithReversedDuplicateEndpoints_ShouldEmitNonzeroImpulsesInAscendingJointIdOrder()
    {
        using GravitasWorldContext context = CreateConstraintContext();
        SolidBody2D first = CreateBody(context, Vector2d.Zero);
        SolidBody2D second = CreateBody(context, Vector2d.Right * (Fixed64)3);
        // Cross SwiftCollections' 16-item insertion-sort threshold so equal keys cannot pass by stable input order.
        const int jointCount = 17;
        var joints = new Joint2D[jointCount];
        joints[0] = context.Constraints2D.RegisterJoint(CreatePin(
            second,
            first,
            JointCollisionPolicy.Collide));
        for (int i = 1; i < joints.Length; i++)
        {
            joints[i] = context.Constraints2D.RegisterJoint(CreatePin(
                first,
                second,
                JointCollisionPolicy.Collide));
        }

        context.Diagnostics.Enable(eventCapacity: 128, drawCommandCapacity: 0);

        Step(context);

        var firstSequences = new int[jointCount];
        Array.Fill(firstSequences, -1);
        ReadOnlySpan<GravitasDiagnosticEvent> events = context.Diagnostics.Events;
        for (int i = 0; i < events.Length; i++)
        {
            GravitasDiagnosticEvent diagnosticEvent = events[i];
            if (diagnosticEvent.Kind != GravitasDiagnosticEventKind.JointImpulse)
                continue;

            int jointIndex = diagnosticEvent.JointId - 1;
            if ((uint)jointIndex < (uint)firstSequences.Length && firstSequences[jointIndex] < 0)
                firstSequences[jointIndex] = diagnosticEvent.Sequence;
        }

        int previousSequence = -1;
        for (int i = 0; i < joints.Length; i++)
        {
            joints[i].LastSolvedRowCount.Should().BeGreaterThan(0);
            if (firstSequences[i] < 0)
                continue;

            joints[i].AccumulatedImpulseMagnitude.Should().BeGreaterThan(Fixed64.Zero);
            firstSequences[i].Should().BeGreaterThan(previousSequence);
            previousSequence = firstSequences[i];
        }

        previousSequence.Should().BeGreaterThanOrEqualTo(0);
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

    [Theory]
    [InlineData(true)]
    [InlineData(false)]
    public void DeactivateRagdollLink_ShouldRemoveAtomic2DRuntimeBeforeSameShellReinitialization(bool deactivateRoot)
    {
        using GravitasWorldContext context = CreateConstraintContext();
        SolidBody2D root = CreateBody(context, Vector2d.Zero);
        SolidBody2D child = CreateBody(context, Vector2d.Right * (Fixed64)2);
        RagdollRuntime2D runtime = context.Constraints2D.RegisterRagdoll(CreateTwoLinkRagdoll(root, child));
        Joint2D joint = runtime.GetJoint(0);
        SolidBody2D endpoint = deactivateRoot ? root : child;

        endpoint.Deactivate();

        runtime.IsRegistered.Should().BeFalse();
        runtime.IsActive.Should().BeFalse();
        joint.IsActive.Should().BeFalse();
        context.Constraints2D.RegisteredRagdollCount.Should().Be(0);
        context.Constraints2D.RegisteredJointCount.Should().Be(0);
        context.Constraints2D.EnabledJointCount.Should().Be(0);
        context.Constraints2D.RemoveRagdoll(runtime.Id).Should().BeFalse();
        Action reactivate = runtime.ActivateDynamic;
        Action setPose = () => context.Constraints2D.SetRagdollPoseTargets(
            runtime,
            new[] { JointMotor2D.Disabled });
        Action serializeRemovedRagdoll = () => GravitasSerializationHarness.Serialize(
            runtime,
            GravitasSerializationTransport.Json);
        reactivate.Should().Throw<InvalidOperationException>();
        setPose.Should().Throw<InvalidOperationException>();
        serializeRemovedRagdoll.Should().Throw<InvalidOperationException>();

        endpoint.Initialize(Vector2d.Forward * (Fixed64)4);

        context.Constraints2D.ShouldExcludeLinkedCollision(root.Collider, child.Collider).Should().BeFalse();
    }

    [Fact]
    public void RemoveRagdoll_ShouldReleaseOwned2DJointsAndEverySelfCollisionSuppression()
    {
        using GravitasWorldContext context = CreateConstraintContext();
        SolidBody2D root = CreateBody(context, Vector2d.Zero);
        SolidBody2D middle = CreateBody(context, Vector2d.Right * (Fixed64)2);
        SolidBody2D end = CreateBody(context, Vector2d.Right * (Fixed64)4);
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
            },
            RagdollSelfCollisionPolicy.SuppressAllLinks));
        Action removeOwnedJoint = () => context.Constraints2D.RemoveJoint(runtime.GetJoint(0).Id);

        removeOwnedJoint.Should().Throw<InvalidOperationException>();
        context.Constraints2D.RemoveRagdoll(runtime.Id).Should().BeTrue();

        runtime.IsRegistered.Should().BeFalse();
        runtime.IsActive.Should().BeFalse();
        context.Constraints2D.RegisteredRagdollCount.Should().Be(0);
        context.Constraints2D.RegisteredJointCount.Should().Be(0);
        context.Constraints2D.EnabledJointCount.Should().Be(0);
        context.Constraints2D.ShouldExcludeLinkedCollision(root.Collider, middle.Collider).Should().BeFalse();
        context.Constraints2D.ShouldExcludeLinkedCollision(middle.Collider, end.Collider).Should().BeFalse();
        context.Constraints2D.ShouldExcludeLinkedCollision(root.Collider, end.Collider).Should().BeFalse();
    }

    [Fact]
    public void DeactivateSingleLinkRagdoll_ShouldRemoveZeroJoint2DRuntime()
    {
        using GravitasWorldContext context = CreateConstraintContext();
        SolidBody2D body = CreateBody(context, Vector2d.Zero);
        RagdollRuntime2D runtime = context.Constraints2D.RegisterRagdoll(new RagdollDefinition2D(
            new[] { new RagdollLinkDefinition2D(0, body) },
            Array.Empty<RagdollJointDefinition2D>()));

        body.Deactivate();

        runtime.IsRegistered.Should().BeFalse();
        context.Constraints2D.RegisteredRagdollCount.Should().Be(0);
    }

    [Fact]
    public void Remove2DRagdolls_OutOfOrder_ShouldKeepMovedRuntimeIndexed()
    {
        using GravitasWorldContext context = CreateConstraintContext();
        SolidBody2D firstRoot = CreateBody(context, Vector2d.Zero);
        SolidBody2D firstChild = CreateBody(context, Vector2d.Right * (Fixed64)2);
        SolidBody2D secondRoot = CreateBody(context, Vector2d.Forward * (Fixed64)4);
        SolidBody2D secondChild = CreateBody(
            context,
            Vector2d.Forward * (Fixed64)4 + Vector2d.Right * (Fixed64)2);
        RagdollRuntime2D first = context.Constraints2D.RegisterRagdoll(CreateTwoLinkRagdoll(firstRoot, firstChild));
        RagdollRuntime2D second = context.Constraints2D.RegisterRagdoll(CreateTwoLinkRagdoll(secondRoot, secondChild));

        context.Constraints2D.RemoveRagdoll(first.Id).Should().BeTrue();
        context.Constraints2D.RemoveRagdoll(second.Id).Should().BeTrue();

        first.IsRegistered.Should().BeFalse();
        second.IsRegistered.Should().BeFalse();
        context.Constraints2D.RegisteredRagdollCount.Should().Be(0);
        context.Constraints2D.RegisteredJointCount.Should().Be(0);
    }

    [Fact]
    public void Ragdoll2DReplayHash_ShouldNotDependOnRemovalOrder()
    {
        using GravitasWorldContext first = CreateConstraintContext();
        using GravitasWorldContext second = CreateConstraintContext();
        var firstRagdolls = new RagdollRuntime2D[4];
        var secondRagdolls = new RagdollRuntime2D[4];
        for (int i = 0; i < firstRagdolls.Length; i++)
        {
            SolidBody2D firstBody = CreateBody(first, Vector2d.Right * (Fixed64)(i * 2));
            SolidBody2D secondBody = CreateBody(second, Vector2d.Right * (Fixed64)(i * 2));
            firstRagdolls[i] = first.Constraints2D.RegisterRagdoll(new RagdollDefinition2D(
                new[] { new RagdollLinkDefinition2D(i, firstBody) },
                Array.Empty<RagdollJointDefinition2D>()));
            secondRagdolls[i] = second.Constraints2D.RegisterRagdoll(new RagdollDefinition2D(
                new[] { new RagdollLinkDefinition2D(i, secondBody) },
                Array.Empty<RagdollJointDefinition2D>()));
        }

        first.Constraints2D.RemoveRagdoll(firstRagdolls[0].Id).Should().BeTrue();
        first.Constraints2D.RemoveRagdoll(firstRagdolls[1].Id).Should().BeTrue();
        second.Constraints2D.RemoveRagdoll(secondRagdolls[1].Id).Should().BeTrue();
        second.Constraints2D.RemoveRagdoll(secondRagdolls[0].Id).Should().BeTrue();

        first.ComputeReplayHash().Should().Be(second.ComputeReplayHash());
        first.ComputeReplayHash(GravitasReplayHashMode.AuthoritativeWithSolverCaches)
            .Should()
            .Be(second.ComputeReplayHash(GravitasReplayHashMode.AuthoritativeWithSolverCaches));
    }

    [Fact]
    public void RegisterRagdoll_WithBodyInExisting2DRagdoll_ShouldFailBeforeRuntimeMutation()
    {
        using GravitasWorldContext context = CreateConstraintContext();
        SolidBody2D root = CreateBody(context, Vector2d.Zero);
        SolidBody2D child = CreateBody(context, Vector2d.Right * (Fixed64)2);
        SolidBody2D other = CreateBody(context, Vector2d.Right * (Fixed64)4);
        context.Constraints2D.RegisterRagdoll(CreateTwoLinkRagdoll(root, child));

        Action registerOverlap = () => context.Constraints2D.RegisterRagdoll(CreateTwoLinkRagdoll(root, other));

        registerOverlap.Should().Throw<ArgumentException>();
        context.Constraints2D.RegisteredRagdollCount.Should().Be(1);
        context.Constraints2D.RegisteredJointCount.Should().Be(1);
        context.Constraints2D.EnabledJointCount.Should().Be(1);
    }

    [Fact]
    public void RegisterRagdoll_WithDuplicate2DBodyLinks_ShouldFailBeforeRuntimeMutation()
    {
        using GravitasWorldContext context = CreateConstraintContext();
        SolidBody2D body = CreateBody(context, Vector2d.Zero);
        var definition = new RagdollDefinition2D(
            new[]
            {
                new RagdollLinkDefinition2D(0, body),
                new RagdollLinkDefinition2D(1, body)
            },
            Array.Empty<RagdollJointDefinition2D>());

        Action registerDuplicate = () => context.Constraints2D.RegisterRagdoll(definition);

        registerDuplicate.Should().Throw<ArgumentException>();
        context.Constraints2D.RegisteredRagdollCount.Should().Be(0);
        context.Constraints2D.RegisteredJointCount.Should().Be(0);
    }

    [Fact]
    public void Reset_WithRegistered2DRagdoll_ShouldInvalidateRuntimeAndOwnedJoints()
    {
        using GravitasWorldContext context = CreateConstraintContext();
        SolidBody2D root = CreateBody(context, Vector2d.Zero);
        SolidBody2D child = CreateBody(context, Vector2d.Right * (Fixed64)2);
        RagdollRuntime2D runtime = context.Constraints2D.RegisterRagdoll(CreateTwoLinkRagdoll(root, child));
        Joint2D joint = runtime.GetJoint(0);

        context.Reset();

        runtime.IsRegistered.Should().BeFalse();
        joint.IsActive.Should().BeFalse();
        context.Constraints2D.RegisteredRagdollCount.Should().Be(0);
        context.Constraints2D.RegisteredJointCount.Should().Be(0);
        Action reactivate = runtime.ActivateDynamic;
        reactivate.Should().Throw<InvalidOperationException>();
    }

    [Fact]
    public void Dispose_WithRegistered2DConstraints_ShouldInvalidateHandlesAndRejectFurtherMutation()
    {
        GravitasWorldContext context = CreateConstraintContext();
        SolidBody2D first = CreateBody(context, Vector2d.Zero);
        SolidBody2D second = CreateBody(context, Vector2d.Right * (Fixed64)2);
        SolidBody2D root = CreateBody(context, Vector2d.Forward * (Fixed64)4);
        SolidBody2D child = CreateBody(context, Vector2d.Forward * (Fixed64)6);
        Joint2D joint = context.Constraints2D.RegisterJoint(CreatePin(first, second));
        RagdollRuntime2D ragdoll = context.Constraints2D.RegisterRagdoll(CreateTwoLinkRagdoll(root, child));

        context.Dispose();

        joint.IsActive.Should().BeFalse();
        ragdoll.IsRegistered.Should().BeFalse();
        context.Constraints2D.RegisteredJointCount.Should().Be(0);
        context.Constraints2D.RegisteredRagdollCount.Should().Be(0);
        Action register = () => context.Constraints2D.RegisterJoint(CreatePin(first, second));
        Action remove = () => context.Constraints2D.RemoveJoint(joint.Id);
        Action reactivate = ragdoll.ActivateDynamic;
        register.Should().Throw<ObjectDisposedException>();
        remove.Should().Throw<ObjectDisposedException>();
        reactivate.Should().Throw<InvalidOperationException>();
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
    public void RagdollFiltering_WithCollideAllPolicy_ShouldAllowAll2DLinkCollisions()
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
            RagdollSelfCollisionPolicy.CollideAllLinks));

        context.Constraints2D.ShouldExcludeLinkedCollision(root.Collider, middle.Collider).Should().BeFalse();
        context.Constraints2D.ShouldExcludeLinkedCollision(middle.Collider, end.Collider).Should().BeFalse();
        context.Constraints2D.ShouldExcludeLinkedCollision(root.Collider, end.Collider).Should().BeFalse();
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
    public void RegisterRagdoll_WithUnknownLinkReference_ShouldFailBefore2DRuntimeStateIsCreated()
    {
        using GravitasWorldContext context = CreateConstraintContext();
        SolidBody2D root = CreateBody(context, Vector2d.Zero);
        SolidBody2D child = CreateBody(context, Vector2d.Right * (Fixed64)2);
        var definition = new RagdollDefinition2D(
            new[]
            {
                new RagdollLinkDefinition2D(0, root),
                new RagdollLinkDefinition2D(1, child)
            },
            new[]
            {
                new RagdollJointDefinition2D(0, 99, JointType2D.Pin, JointFrame2D.Identity, JointFrame2D.Identity)
            });

        Action act = () => context.Constraints2D.RegisterRagdoll(definition);

        act.Should().Throw<ArgumentException>().WithParameterName("linkId");
        context.Constraints2D.RegisteredJointCount.Should().Be(0);
        context.Constraints2D.RegisteredRagdollCount.Should().Be(0);
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
    public void RagdollRuntime_WhenALaterLinkHasInvalidRuntimeScale_ShouldNotPartiallyDeactivate()
    {
        using GravitasWorldContext context = CreateConstraintContext();
        SolidBody2D root = CreateBody(context, Vector2d.Zero);
        SolidBody2D child = CreateBody(context, Vector2d.Right * (Fixed64)2);
        RagdollRuntime2D runtime = context.Constraints2D.RegisterRagdoll(CreateTwoLinkRagdoll(root, child));
        var singularParent = new FixedTransform(
            Vector3d.Zero,
            FixedQuaternion.Identity,
            new Vector3d(Fixed64.Zero, Fixed64.One, Fixed64.One));
        child.Agent.Transform.SetParentKeepingLocal(singularParent);

        Action deactivate = runtime.DeactivateToKinematic;

        deactivate.Should().Throw<ArgumentException>();
        runtime.IsActive.Should().BeTrue();
        root.MotionType.Should().Be(BodyMotionType.Dynamic);
        child.MotionType.Should().Be(BodyMotionType.Dynamic);
        runtime.GetJoint(0).IsEnabled.Should().BeTrue();
        context.Constraints2D.EnabledJointCount.Should().Be(1);
    }

    [Fact]
    public void RagdollRuntime_WithSingleLinkAndNoJoints_ShouldEmitActivationDiagnostic()
    {
        using GravitasWorldContext context = CreateConstraintContext();
        SolidBody2D link = CreateBody(context, Vector2d.Zero, isKinematic: true);
        RagdollRuntime2D runtime = context.Constraints2D.RegisterRagdoll(new RagdollDefinition2D(
            new[] { new RagdollLinkDefinition2D(7, link) },
            Array.Empty<RagdollJointDefinition2D>()));
        context.Diagnostics.Enable(eventCapacity: 1, drawCommandCapacity: 0);

        runtime.ActivateDynamic();

        runtime.GetLink(0).Should().BeSameAs(link);
        runtime.IsActive.Should().BeTrue();
        link.IsKinematic.Should().BeFalse();
        context.Diagnostics.Events.Should().ContainSingle();
        GravitasDiagnosticEvent diagnosticEvent = context.Diagnostics.Events[0];
        diagnosticEvent.Kind.Should().Be(GravitasDiagnosticEventKind.RagdollActivated);
        diagnosticEvent.BodyId.Should().Be(runtime.Id);
        diagnosticEvent.DataA.Should().Be(1);
        diagnosticEvent.DataB.Should().Be(0);
        diagnosticEvent.Hit.Should().BeTrue();
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
        context.Constraints2D.SetJointMotorTarget(99, Fixed64.Zero).Should().BeFalse();

        joint.Motor.Kind.Should().Be(JointMotorKind2D.Angular);
        joint.Motor.DriveStrength.Should().Be((Fixed64)2);
        joint.Motor.Damping.Should().Be(Fixed64.Half);
        joint.Motor.MaximumMotorImpulse.Should().Be(Fixed64.One);
        joint.Motor.Target.Should().Be(Fixed64.FromFraction(1, 3));

        context.Constraints2D.ClearJointMotorTarget(joint.Id).Should().BeTrue();
        context.Constraints2D.ClearJointMotorTarget(99).Should().BeFalse();
        joint.Motor.Kind.Should().Be(JointMotorKind2D.Disabled);
    }

    [Fact]
    public void SetJointMotorTarget_WithLinearMotor_ShouldPreserve2DMotorPayload()
    {
        using GravitasWorldContext context = CreateConstraintContext();
        SolidBody2D first = CreateBody(context, Vector2d.Zero);
        SolidBody2D second = CreateBody(context, Vector2d.Right * (Fixed64)2);
        Joint2D joint = context.Constraints2D.RegisterJoint(new JointDefinition2D(
            first,
            second,
            JointFrame2D.Identity,
            JointFrame2D.Identity,
            JointType2D.Prismatic,
            JointLimit2D.Unrestricted,
            JointMotor2D.Linear(Fixed64.Zero, (Fixed64)2, Fixed64.Half, Fixed64.One),
            JointCollisionPolicy.SuppressLinked));

        context.Constraints2D.SetJointMotorTarget(joint.Id, Fixed64.One).Should().BeTrue();

        joint.Motor.Kind.Should().Be(JointMotorKind2D.Linear);
        joint.Motor.Target.Should().Be(Fixed64.One);
        joint.Motor.DriveStrength.Should().Be((Fixed64)2);
        joint.Motor.Damping.Should().Be(Fixed64.Half);
        joint.Motor.MaximumMotorImpulse.Should().Be(Fixed64.One);
    }

    [Fact]
    public void SetMotor_WithLinearMotorOnNonPrismaticJoint_ShouldReject()
    {
        using GravitasWorldContext context = CreateConstraintContext();
        SolidBody2D first = CreateBody(context, Vector2d.Zero);
        SolidBody2D second = CreateBody(context, Vector2d.Right * (Fixed64)2);
        Joint2D joint = context.Constraints2D.RegisterJoint(CreatePin(first, second));

        Action act = () => joint.SetMotor(JointMotor2D.Linear(Fixed64.Zero, Fixed64.One, Fixed64.Zero, Fixed64.One));

        act.Should().Throw<ArgumentException>();
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
        context.Constraints2D.EnabledJointCount.Should().Be(0);
        target.Type.Should().Be(JointType2D.Prismatic);
        target.LocalFrameA.Anchor.Should().Be(Vector2d.Right);
        target.LocalFrameB.Anchor.Should().Be(-Vector2d.Right);
        target.LocalFrameA.Angle.Should().Be(Fixed64.Half);
        target.Limits.Should().Be(source.Limits);
        target.Motor.Should().Be(source.Motor);
        target.CollisionPolicy.Should().Be(JointCollisionPolicy.Collide);
    }

    [Theory]
    [MemberData(nameof(Transports))]
    public void JointRecordData_ShouldRoundTripAngularLimitAndMotorState(GravitasSerializationTransport transport)
    {
        using GravitasWorldContext context = CreateConstraintContext();
        SolidBody2D first = CreateBody(context, Vector2d.Zero);
        SolidBody2D second = CreateBody(context, Vector2d.Right * (Fixed64)2);
        Joint2D source = context.Constraints2D.RegisterJoint(new JointDefinition2D(
            first,
            second,
            JointFrame2D.Identity,
            JointFrame2D.Identity,
            JointType2D.Pin,
            JointLimit2D.Angular(-Fixed64.Half, Fixed64.Half),
            JointMotor2D.Angular(Fixed64.Half, (Fixed64)3, Fixed64.Half, Fixed64.One),
            JointCollisionPolicy.SuppressLinked));

        object payload = GravitasSerializationHarness.Serialize(source, transport);

        Joint2D target = context.Constraints2D.RegisterJoint(CreatePin(first, second));
        GravitasSerializationHarness.Populate(target, payload, transport);

        target.IsEnabled.Should().BeTrue();
        target.Type.Should().Be(JointType2D.Pin);
        target.Limits.Should().Be(source.Limits);
        target.Motor.Should().Be(source.Motor);
        target.CollisionPolicy.Should().Be(JointCollisionPolicy.SuppressLinked);
    }

    [Theory]
    [MemberData(nameof(Transports))]
    public void JointRecordData_ShouldRoundTripExplicitDistanceLimitAndResetSolverState(
        GravitasSerializationTransport transport)
    {
        using GravitasWorldContext sourceContext = CreateConstraintContext();
        SolidBody2D sourceFirst = CreateBody(sourceContext, Vector2d.Zero);
        SolidBody2D sourceSecond = CreateBody(sourceContext, Vector2d.Right * (Fixed64)5);
        Joint2D source = sourceContext.Constraints2D.RegisterJoint(new JointDefinition2D(
            sourceFirst,
            sourceSecond,
            new JointFrame2D(Vector2d.Right, Fixed64.Half),
            new JointFrame2D(-Vector2d.Right, -Fixed64.Half),
            JointType2D.Distance,
            JointLimit2D.Distance((Fixed64)3),
            JointMotor2D.Disabled,
            JointCollisionPolicy.Collide));
        object payload = GravitasSerializationHarness.Serialize(source, transport);

        using GravitasWorldContext targetContext = CreateConstraintContext();
        SolidBody2D targetFirst = CreateBody(targetContext, Vector2d.Zero);
        SolidBody2D targetSecond = CreateBody(targetContext, Vector2d.Right * (Fixed64)2);
        Joint2D target = targetContext.Constraints2D.RegisterJoint(CreatePin(targetFirst, targetSecond));
        targetContext.Constraints2D.ShouldExcludeLinkedCollision(
            targetFirst.Collider,
            targetSecond.Collider).Should().BeTrue();
        Step(targetContext);
        target.LastSolvedRowCount.Should().BeGreaterThan(0);
        target.AccumulatedImpulseMagnitude.Should().BeGreaterThan(Fixed64.Zero);
        target.LastSolveMetrics.PreparedRowCount.Should().BeGreaterThan(0);

        GravitasSerializationHarness.Populate(target, payload, transport);

        target.Type.Should().Be(JointType2D.Distance);
        target.Limits.Should().Be(JointLimit2D.Distance((Fixed64)3));
        target.LocalFrameA.Should().Be(source.LocalFrameA);
        target.LocalFrameB.Should().Be(source.LocalFrameB);
        target.CollisionPolicy.Should().Be(JointCollisionPolicy.Collide);
        targetContext.Constraints2D.ShouldExcludeLinkedCollision(
            targetFirst.Collider,
            targetSecond.Collider).Should().BeFalse();
        target.LastSolvedRowCount.Should().Be(0);
        target.AccumulatedImpulseMagnitude.Should().Be(Fixed64.Zero);
        target.LastSolveMetrics.Should().Be(default(JointSolveMetrics2D));
        target.BodyA.Should().BeSameAs(targetFirst);
        target.BodyB.Should().BeSameAs(targetSecond);
        targetContext.Constraints2D.GetJoint(target.Id).Should().BeSameAs(target);
        targetContext.Constraints2D.EnabledJointCount.Should().Be(1);
    }

    [Fact]
    public void JointRecordData_WithInvalidLoadedTypePayload_ShouldReject()
    {
        using GravitasWorldContext context = CreateConstraintContext();
        SolidBody2D first = CreateBody(context, Vector2d.Zero);
        SolidBody2D second = CreateBody(context, Vector2d.Right * (Fixed64)2);
        Joint2D target = context.Constraints2D.RegisterJoint(CreatePin(first, second));
        var chronicler = new InvalidRecordPayloadChronicler(new Dictionary<string, object>
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

    [Fact]
    public void JointRecordData_WithUnrestrictedDistancePayload_ShouldResolveExplicitTargetDistance()
    {
        using GravitasWorldContext context = CreateConstraintContext();
        SolidBody2D first = CreateBody(context, Vector2d.Zero);
        SolidBody2D second = CreateBody(context, Vector2d.Right * (Fixed64)2);
        Joint2D target = context.Constraints2D.RegisterJoint(CreatePin(first, second));
        Step(context);
        target.LastSolvedRowCount.Should().BeGreaterThan(0);
        target.AccumulatedImpulseMagnitude.Should().BeGreaterThan(Fixed64.Zero);
        target.LastSolveMetrics.PreparedRowCount.Should().BeGreaterThan(0);
        first.SetPosition(Vector2d.Zero);
        second.SetPosition(Vector2d.Right * (Fixed64)5);
        var chronicler = new InvalidRecordPayloadChronicler(new Dictionary<string, object>
        {
            [nameof(Joint2D.IsEnabled)] = true,
            ["Type"] = JointType2D.Distance,
            ["LimitKind"] = JointLimitKind2D.Unrestricted,
            ["MotorKind"] = JointMotorKind2D.Disabled,
            ["CollisionPolicy"] = JointCollisionPolicy.SuppressLinked,
            ["LocalFrameAAnchor"] = Vector2d.Right,
            ["LocalFrameAAngle"] = Fixed64.Half,
            ["LocalFrameBAnchor"] = -Vector2d.Right,
            ["LocalFrameBAngle"] = -Fixed64.Half
        });

        target.RecordData(chronicler);

        target.Type.Should().Be(JointType2D.Distance);
        target.Limits.Kind.Should().Be(JointLimitKind2D.Distance);
        target.Limits.TargetDistance.Should().Be((Fixed64)3);
        target.LocalFrameA.Should().Be(new JointFrame2D(Vector2d.Right, Fixed64.Half));
        target.LocalFrameB.Should().Be(new JointFrame2D(-Vector2d.Right, -Fixed64.Half));
        target.Motor.Kind.Should().Be(JointMotorKind2D.Disabled);
        target.LastSolvedRowCount.Should().Be(0);
        target.AccumulatedImpulseMagnitude.Should().Be(Fixed64.Zero);
        target.LastSolveMetrics.Should().Be(default(JointSolveMetrics2D));
        target.BodyA.Should().BeSameAs(first);
        target.BodyB.Should().BeSameAs(second);
        context.Constraints2D.GetJoint(target.Id).Should().BeSameAs(target);
        context.Constraints2D.EnabledJointCount.Should().Be(1);
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
    public void DistanceJoint_WithCoincidentAnchorsAndZeroTarget_ShouldEmitNoRows()
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
            JointLimit2D.Distance(Fixed64.Zero),
            JointMotor2D.Disabled,
            JointCollisionPolicy.SuppressLinked));

        Step(context);

        joint.LastSolvedRowCount.Should().Be(0);
        joint.LastSolveMetrics.PreparedRowCount.Should().Be(0);
        joint.AccumulatedImpulseMagnitude.Should().Be(Fixed64.Zero);
        first.Position.Should().Be(Vector2d.Zero);
        second.Position.Should().Be(Vector2d.Zero);
    }

    [Fact]
    public void DistanceJoint_WithCoincidentAnchorsAndSeparatedBodies_ShouldUseBodyDeltaAxis()
    {
        using GravitasWorldContext context = CreateConstraintContext();
        SolidBody2D first = CreateBody(context, Vector2d.Zero);
        SolidBody2D second = CreateBody(context, Vector2d.Right * (Fixed64)2);
        Joint2D joint = context.Constraints2D.RegisterJoint(new JointDefinition2D(
            first,
            second,
            new JointFrame2D(Vector2d.Zero, Fixed64.HalfPi),
            new JointFrame2D(-Vector2d.Right * (Fixed64)2, Fixed64.Zero),
            JointType2D.Distance,
            JointLimit2D.Distance(Fixed64.One),
            JointMotor2D.Disabled,
            JointCollisionPolicy.SuppressLinked));
        Step(context);

        joint.LastSolvedRowCount.Should().Be(1);
        joint.AccumulatedImpulseMagnitude.Should().BeGreaterThan(Fixed64.Zero);
        first.LinearVelocity.X.Abs().Should().BeGreaterThan(Fixed64.Zero);
        second.LinearVelocity.X.Abs().Should().BeGreaterThan(Fixed64.Zero);
        first.LinearVelocity.Y.Should().Be(Fixed64.Zero);
        second.LinearVelocity.Y.Should().Be(Fixed64.Zero);
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
        SolidBody2D dynamic = CreateBody(context, Vector2d.Right * (Fixed64)2);
        Fixed64 relativeFrameAngle = FixedMath.DegToRad((Fixed64)degrees);
        Joint2D joint = context.Constraints2D.RegisterJoint(new JointDefinition2D(
            anchor,
            dynamic,
            JointFrame2D.Identity,
            new JointFrame2D(Vector2d.Zero, relativeFrameAngle),
            JointType2D.Pin,
            JointLimit2D.Angular(-Fixed64.Half, Fixed64.Half),
            JointMotor2D.Disabled,
            JointCollisionPolicy.SuppressLinked));

        Step(context, 2);

        joint.LastSolveMetrics.LimitErrorMagnitude.Should().BeGreaterThan(Fixed64.Zero);
        joint.LastSolveMetrics.ClampedRowCount.Should().BeGreaterThan(0);
    }

    [Theory]
    [InlineData(-270)]
    [InlineData(270)]
    public void AngularLimit_ShouldNormalizeAnglesAcrossPiBoundary(int degrees)
    {
        using GravitasWorldContext context = CreateConstraintContext();
        SolidBody2D anchor = CreateBody(context, Vector2d.Zero, immovable: true);
        SolidBody2D dynamic = CreateBody(context, Vector2d.Right * (Fixed64)2);
        Fixed64 relativeFrameAngle = FixedMath.DegToRad((Fixed64)degrees);
        Joint2D joint = context.Constraints2D.RegisterJoint(new JointDefinition2D(
            anchor,
            dynamic,
            new JointFrame2D(Vector2d.Zero, relativeFrameAngle),
            JointFrame2D.Identity,
            JointType2D.Pin,
            JointLimit2D.Angular(-Fixed64.Half, Fixed64.Half),
            JointMotor2D.Disabled,
            JointCollisionPolicy.SuppressLinked));

        Step(context, 2);

        joint.LastSolveMetrics.LimitErrorMagnitude.Should().BeGreaterThan(Fixed64.Zero);
    }

    [Fact]
    public void WeldJoint_WithFrozenRotation_ShouldLeaveAngularStateUntouched()
    {
        using GravitasWorldContext context = CreateConstraintContext();
        SolidBody2D anchor = CreateBody(context, Vector2d.Zero, immovable: true);
        SolidBody2D dynamic = CreateBody(
            context,
            Vector2d.Right * (Fixed64)2,
            rotation: FixedMath.DegToRad((Fixed64)45));
        dynamic.FreezeAxes = BodyFreezeAxes2D.Rotation;
        Fixed64 initialRotation = dynamic.Rotation;
        Joint2D joint = context.Constraints2D.RegisterJoint(new JointDefinition2D(
            anchor,
            dynamic,
            JointFrame2D.Identity,
            JointFrame2D.Identity,
            JointType2D.Weld,
            JointLimit2D.Unrestricted,
            JointMotor2D.Disabled,
            JointCollisionPolicy.SuppressLinked));

        Step(context, 4);

        dynamic.Rotation.Should().Be(initialRotation);
        joint.LastSolvedRowCount.Should().BeGreaterThan(0);
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
            FixedQuaternion.FromAxisAngle(Vector3d.Up, -rotation),
            Vector3d.One);
        var agent = new TestMatterAgent(context, transform);
        var body = new SolidBody2D(agent, new LSCircleCollider2D(Fixed64.Half))
        {
            Mass = Fixed64.One
        };
        body.Initialize(
            position,
            rotation,
            immovable
                ? BodyMotionType.Static
                : isKinematic
                    ? BodyMotionType.Kinematic
                    : BodyMotionType.Dynamic);
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
}
