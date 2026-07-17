using FixedMathSharp;
using FluentAssertions;
using Gravitas.Colliders;
using Gravitas.CollisionHandling;
using Gravitas.Constraints;
using Gravitas.Diagnostics;
using Gravitas.Tests.Serialization;
using Gravitas.Tests.Support;
using System;
using System.Collections.Generic;
using Xunit;

namespace Gravitas.Tests.Constraints;

public sealed class Constraint3DServiceTests
{
    public static TheoryData<GravitasSerializationTransport> Transports => GravitasSerializationTransportCases.All();

    [Fact]
    public void NewContext_ShouldOwnEmptyConstraintService()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();

        scenario.Context.Constraints3D.Should().NotBeNull();
        scenario.Context.Constraints3D.RegisteredJointCount.Should().Be(0);
        scenario.Context.Constraints3D.PeakJointCount.Should().Be(0);
    }

    [Fact]
    public void Joint3D_IsSolverBody_ShouldRequireActiveRegisteredTranslatableBody()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSSphereCollider> active = scenario.CreateSphere(Vector3d.Zero);
        ScenarioBody<LSSphereCollider> frozen = scenario.CreateSphere(Vector3d.Right * (Fixed64)2, immovable: true);
        ScenarioBody<LSSphereCollider> inactive = scenario.CreateSphere(Vector3d.Right * (Fixed64)4);

        inactive.Body.Deactivate();

        Joint3D.IsSolverBody(active.Body).Should().BeTrue();
        Joint3D.IsSolverBody(frozen.Body).Should().BeFalse();
        Joint3D.IsSolverBody(inactive.Body).Should().BeFalse();
    }

    [Fact]
    public void RegisterJoint_ShouldAssignDeterministicMonotonicIdsAndAllowDuplicateBodyPairs()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        scenario.Context.Environment.Gravity = Fixed64.Zero;
        ScenarioBody<LSSphereCollider> first = scenario.CreateSphere(Vector3d.Zero);
        ScenarioBody<LSSphereCollider> second = scenario.CreateSphere(Vector3d.Right * (Fixed64)2);
        JointDefinition3D definition = CreateBallSocket(first.Body, second.Body);

        Joint3D firstJoint = scenario.Context.Constraints3D.RegisterJoint(definition);
        Joint3D secondJoint = scenario.Context.Constraints3D.RegisterJoint(definition);

        firstJoint.Id.Should().Be(1);
        secondJoint.Id.Should().Be(2);
        firstJoint.Should().NotBeSameAs(secondJoint);
        scenario.Context.Constraints3D.RegisteredJointCount.Should().Be(2);
        scenario.Context.Constraints3D.PeakJointCount.Should().Be(2);
        scenario.Context.Constraints3D.TryGetJoint(1, out Joint3D? resolved).Should().BeTrue();
        resolved.Should().BeSameAs(firstJoint);
    }

    [Fact]
    public void RegisterJoint_BeyondDefaultCapacity_ShouldGrowAndResetDeterministically()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        scenario.Context.Environment.Gravity = Fixed64.Zero;
        ScenarioBody<LSSphereCollider> first = scenario.CreateSphere(Vector3d.Zero);
        ScenarioBody<LSSphereCollider> second = scenario.CreateSphere(Vector3d.Right * (Fixed64)2);

        Joint3D? lastJoint = null;
        for (int i = 0; i < 70; i++)
            lastJoint = scenario.Context.Constraints3D.RegisterJoint(CreateBallSocket(first.Body, second.Body));

        scenario.Context.Constraints3D.RemoveJoint(2).Should().BeTrue();
        scenario.Context.Constraints3D.RegisteredJointCount.Should().Be(69);
        scenario.Context.Constraints3D.PeakJointCount.Should().Be(70);
        scenario.Context.Constraints3D.TryGetJoint(lastJoint!.Id, out Joint3D? resolved).Should().BeTrue();
        resolved.Should().BeSameAs(lastJoint);

        scenario.Context.Reset();

        scenario.Context.Constraints3D.RegisteredJointCount.Should().Be(0);
        scenario.Context.Constraints3D.PeakJointCount.Should().Be(0);
        scenario.Context.Constraints3D.TryGetJoint(lastJoint.Id, out _).Should().BeFalse();
        scenario.Context.Constraints3D.ShouldExcludeLinkedCollision(first.Collider, second.Collider).Should().BeFalse();
    }

    [Fact]
    public void RemoveJoint_ShouldReleaseRuntimeStateAndPreventLookup()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSSphereCollider> first = scenario.CreateSphere(Vector3d.Zero);
        ScenarioBody<LSSphereCollider> second = scenario.CreateSphere(Vector3d.Right * (Fixed64)2);
        Joint3D joint = scenario.Context.Constraints3D.RegisterJoint(CreateBallSocket(first.Body, second.Body));

        scenario.Context.Constraints3D.RemoveJoint(joint.Id).Should().BeTrue();

        joint.IsActive.Should().BeFalse();
        scenario.Context.Constraints3D.RegisteredJointCount.Should().Be(0);
        scenario.Context.Constraints3D.TryGetJoint(joint.Id, out _).Should().BeFalse();
    }

    [Fact]
    public void RemoveJoint_WithDisabledJoint_ShouldUpdateEnabledCountOnlyOnce()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSSphereCollider> first = scenario.CreateSphere(Vector3d.Zero);
        ScenarioBody<LSSphereCollider> second = scenario.CreateSphere(Vector3d.Right * (Fixed64)2);
        Joint3D joint = scenario.Context.Constraints3D.RegisterJoint(CreateBallSocket(first.Body, second.Body));

        joint.IsEnabled = false;
        bool removed = scenario.Context.Constraints3D.RemoveJoint(joint.Id);
        bool removedAgain = scenario.Context.Constraints3D.RemoveJoint(joint.Id);

        removed.Should().BeTrue();
        removedAgain.Should().BeFalse();
        scenario.Context.Constraints3D.RegisteredJointCount.Should().Be(0);
        scenario.Context.Constraints3D.EnabledJointCount.Should().Be(0);
    }

    [Fact]
    public void ConstraintServicePolicyHelpers_ShouldRejectInvalidFilterInputsAndInactivePolicyChanges()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSSphereCollider> first = scenario.CreateSphere(Vector3d.Zero);
        ScenarioBody<LSSphereCollider> second = scenario.CreateSphere(Vector3d.Right * (Fixed64)2);
        Joint3D joint = scenario.Context.Constraints3D.RegisterJoint(CreateBallSocket(first.Body, second.Body));
        var unregistered = new LSSphereCollider();

        scenario.Context.Constraints3D.ShouldExcludeLinkedCollision(null!, second.Collider).Should().BeFalse();
        scenario.Context.Constraints3D.ShouldExcludeLinkedCollision(first.Collider, null!).Should().BeFalse();
        scenario.Context.Constraints3D.ShouldExcludeLinkedCollision(first.Collider, first.Collider).Should().BeFalse();
        scenario.Context.Constraints3D.ShouldExcludeLinkedCollision(unregistered, second.Collider).Should().BeFalse();
        scenario.Context.Constraints3D.ShouldExcludeLinkedCollision(first.Collider, unregistered).Should().BeFalse();

        scenario.Context.Constraints3D.UpdateJointCollisionPolicy(
            joint,
            JointCollisionPolicy.SuppressLinked,
            JointCollisionPolicy.SuppressLinked);
        scenario.Context.Constraints3D.ShouldExcludeLinkedCollision(first.Collider, second.Collider).Should().BeTrue();
        scenario.Context.Constraints3D.RemoveSuppressionsForCollider(unregistered.Id);

        scenario.Context.Constraints3D.RemoveJoint(joint.Id).Should().BeTrue();
        joint.SetCollisionPolicyFromRecord(JointCollisionPolicy.Collide);
        joint.SetCollisionPolicyFromRecord(JointCollisionPolicy.SuppressLinked);
        scenario.Context.Constraints3D.UpdateJointEnabledState(joint, true, false);

        scenario.Context.Constraints3D.ShouldExcludeLinkedCollision(first.Collider, second.Collider).Should().BeFalse();
    }

    [Fact]
    public void ConstraintServiceSolverLookup_ShouldHonorEnabledAndMobilityState()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSSphereCollider> first = scenario.CreateSphere(Vector3d.Zero);
        ScenarioBody<LSSphereCollider> second = scenario.CreateSphere(Vector3d.Right * (Fixed64)2);
        Joint3D joint = scenario.Context.Constraints3D.RegisterJoint(CreateBallSocket(first.Body, second.Body));
        Action missingGet = () => scenario.Context.Constraints3D.GetJoint(99);

        missingGet.Should().Throw<ArgumentOutOfRangeException>().WithParameterName("jointId");
        scenario.Context.Constraints3D.TryGetJointForSolver(99, out Joint3D? missing).Should().BeFalse();
        missing.Should().BeNull();

        joint.IsEnabled = false;
        scenario.Context.Constraints3D.TryGetJointForSolver(joint.Id, out Joint3D? disabled).Should().BeFalse();
        disabled.Should().BeNull();

        joint.IsEnabled = true;
        scenario.Context.Constraints3D.TryGetJointForSolver(joint.Id, out Joint3D? enabled).Should().BeTrue();
        enabled.Should().BeSameAs(joint);

        first.Body.FreezeAxes = BodyFreezeAxes3D.Position;
        second.Body.FreezeAxes = BodyFreezeAxes3D.Position;
        scenario.Context.Constraints3D.TryGetJointForSolver(joint.Id, out Joint3D? frozen).Should().BeFalse();
        frozen.Should().BeNull();
    }

    [Fact]
    public void CollisionPolicyRecordUpdate_ShouldAddAndRemoveLinkedCollisionSuppression()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSSphereCollider> first = scenario.CreateSphere(Vector3d.Zero);
        ScenarioBody<LSSphereCollider> second = scenario.CreateSphere(Vector3d.Right * (Fixed64)2);
        Joint3D joint = scenario.Context.Constraints3D.RegisterJoint(CreateBallSocket(first.Body, second.Body));

        scenario.Context.Constraints3D.ShouldExcludeLinkedCollision(first.Collider, second.Collider).Should().BeTrue();

        joint.SetCollisionPolicyFromRecord(JointCollisionPolicy.Collide);

        joint.CollisionPolicy.Should().Be(JointCollisionPolicy.Collide);
        scenario.Context.Constraints3D.ShouldExcludeLinkedCollision(first.Collider, second.Collider).Should().BeFalse();

        joint.SetCollisionPolicyFromRecord(JointCollisionPolicy.SuppressLinked);

        joint.CollisionPolicy.Should().Be(JointCollisionPolicy.SuppressLinked);
        scenario.Context.Constraints3D.ShouldExcludeLinkedCollision(first.Collider, second.Collider).Should().BeTrue();
    }

    [Fact]
    public void RemoveJoint_WithCollidePolicy_ShouldLeaveSuppressionStateUntouched()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSSphereCollider> first = scenario.CreateSphere(Vector3d.Zero);
        ScenarioBody<LSSphereCollider> second = scenario.CreateSphere(Vector3d.Right * (Fixed64)2);
        Joint3D joint = scenario.Context.Constraints3D.RegisterJoint(CreateBallSocket(
            first.Body,
            second.Body,
            JointCollisionPolicy.Collide));

        scenario.Context.Constraints3D.ShouldExcludeLinkedCollision(first.Collider, second.Collider).Should().BeFalse();
        scenario.Context.Constraints3D.UpdateJointCollisionPolicy(
            joint,
            JointCollisionPolicy.SuppressLinked,
            JointCollisionPolicy.Collide);

        scenario.Context.Constraints3D.RemoveJoint(joint.Id).Should().BeTrue();

        scenario.Context.Constraints3D.ShouldExcludeLinkedCollision(first.Collider, second.Collider).Should().BeFalse();
    }

    [Fact]
    public void RemoveJoint_WithDuplicateSuppressingJoints_ShouldKeepSuppressionUntilLastJointIsRemoved()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSSphereCollider> first = scenario.CreateSphere(Vector3d.Zero);
        ScenarioBody<LSSphereCollider> second = scenario.CreateSphere(Vector3d.Right * (Fixed64)2);
        JointDefinition3D definition = CreateBallSocket(first.Body, second.Body);
        Joint3D firstJoint = scenario.Context.Constraints3D.RegisterJoint(definition);
        Joint3D secondJoint = scenario.Context.Constraints3D.RegisterJoint(definition);

        scenario.Context.Constraints3D.ShouldExcludeLinkedCollision(first.Collider, second.Collider).Should().BeTrue();

        scenario.Context.Constraints3D.RemoveJoint(firstJoint.Id).Should().BeTrue();

        scenario.Context.Constraints3D.ShouldExcludeLinkedCollision(first.Collider, second.Collider).Should().BeTrue();

        scenario.Context.Constraints3D.RemoveJoint(secondJoint.Id).Should().BeTrue();

        scenario.Context.Constraints3D.ShouldExcludeLinkedCollision(first.Collider, second.Collider).Should().BeFalse();
    }

    [Fact]
    public void RegisterJoint_WithInvalidDefinition_ShouldFailBeforeSolverStateIsCreated()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSSphereCollider> body = scenario.CreateSphere(Vector3d.Zero);

        Action sameBody = () => scenario.Context.Constraints3D.RegisterJoint(CreateBallSocket(body.Body, body.Body));
        Action nullFrame = () => scenario.Context.Constraints3D.RegisterJoint(new JointDefinition3D(
            body.Body,
            scenario.CreateSphere(Vector3d.Right * (Fixed64)2).Body,
            localFrameA: null!,
            localFrameB: LocalFrame(Vector3d.Zero),
            JointType3D.BallSocket,
            JointLimit3D.Unrestricted,
            JointMotor3D.Disabled,
            JointCollisionPolicy.SuppressLinked));
        ScenarioBody<LSSphereCollider> inactive = scenario.CreateSphere(Vector3d.Right * (Fixed64)4);
        inactive.Body.Deactivate();
        Action inactiveBody = () => scenario.Context.Constraints3D.RegisterJoint(CreateBallSocket(body.Body, inactive.Body));
        ScenarioBody<LSSphereCollider> inactiveFirst = scenario.CreateSphere(Vector3d.Right * (Fixed64)5);
        inactiveFirst.Body.Deactivate();
        Action inactiveFirstBody = () => scenario.Context.Constraints3D.RegisterJoint(CreateBallSocket(
            inactiveFirst.Body,
            scenario.CreateSphere(new Vector3d((Fixed64)5, Fixed64.Zero, Fixed64.One)).Body));
        Action invalidCollisionPolicy = () => scenario.Context.Constraints3D.RegisterJoint(CreateBallSocket(
            body.Body,
            scenario.CreateSphere(Vector3d.Right * (Fixed64)6).Body,
            (JointCollisionPolicy)255));

        sameBody.Should().Throw<ArgumentException>();
        nullFrame.Should().Throw<ArgumentNullException>();
        inactiveBody.Should().Throw<ArgumentException>();
        inactiveFirstBody.Should().Throw<ArgumentException>();
        invalidCollisionPolicy.Should().Throw<ArgumentException>();
        scenario.Context.Constraints3D.RegisteredJointCount.Should().Be(0);
    }

    [Fact]
    public void RegisterJoint_WithIncompatibleLimitPayload_ShouldReject()
    {
        using PhysicsScenarioBuilder scenario = CreateConstraintScenario();
        ScenarioBody<LSSphereCollider> first = scenario.CreateSphere(Vector3d.Zero);
        ScenarioBody<LSSphereCollider> second = scenario.CreateSphere(Vector3d.Right * (Fixed64)2);

        Action act = () => scenario.Context.Constraints3D.RegisterJoint(new JointDefinition3D(
            first.Body,
            second.Body,
            LocalFrame(Vector3d.Zero),
            LocalFrame(Vector3d.Zero),
            JointType3D.BallSocket,
            JointLimit3D.Hinge(Fixed64.HalfPi),
            JointMotor3D.Disabled,
            JointCollisionPolicy.SuppressLinked));

        act.Should().Throw<ArgumentException>();
        scenario.Context.Constraints3D.RegisteredJointCount.Should().Be(0);
    }

    [Fact]
    public void RegisterJoint_WithBodiesFromDifferentContexts_ShouldFail()
    {
        using PhysicsScenarioBuilder firstScenario = PhysicsScenarioBuilder.Create();
        using PhysicsScenarioBuilder secondScenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSSphereCollider> first = firstScenario.CreateSphere(Vector3d.Zero);
        ScenarioBody<LSSphereCollider> second = secondScenario.CreateSphere(Vector3d.Right * (Fixed64)2);

        Action act = () => firstScenario.Context.Constraints3D.RegisterJoint(CreateBallSocket(first.Body, second.Body));
        Action reversed = () => firstScenario.Context.Constraints3D.RegisterJoint(CreateBallSocket(second.Body, first.Body));

        act.Should().Throw<ArgumentException>();
        reversed.Should().Throw<ArgumentException>();
    }

    [Fact]
    public void DirectJoint_ShouldSuppressAdjacentLinkedCollisionByDefault()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        scenario.Context.Environment.Gravity = Fixed64.Zero;
        ScenarioBody<LSSphereCollider> first = scenario.CreateSphere(Vector3d.Zero);
        ScenarioBody<LSSphereCollider> second = scenario.CreateSphere(Vector3d.Right * Fixed64.Half);

        scenario.Context.Constraints3D.RegisterJoint(CreateBallSocket(first.Body, second.Body));
        scenario.Context.Simulate();
        scenario.Context.LateSimulate();

        first.Collider.TryGetCollisionPair(second.Collider.Id, out _).Should().BeFalse();
        scenario.Context.Constraints3D.ShouldExcludeLinkedCollision(first.Collider, second.Collider).Should().BeTrue();
    }

    [Fact]
    public void DirectJoint_WithCollidePolicy_ShouldAllowLinkedCollision()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        scenario.Context.Environment.Gravity = Fixed64.Zero;
        ScenarioBody<LSSphereCollider> first = scenario.CreateSphere(Vector3d.Zero);
        ScenarioBody<LSSphereCollider> second = scenario.CreateSphere(Vector3d.Right * Fixed64.Half);

        scenario.Context.Constraints3D.RegisterJoint(CreateBallSocket(
            first.Body,
            second.Body,
            collisionPolicy: JointCollisionPolicy.Collide));
        scenario.Context.Simulate();
        scenario.Context.LateSimulate();

        first.Collider.TryGetCollisionPair(second.Collider.Id, out CollisionPair? pair).Should().BeTrue();
        pair!.Manifold.HasContact.Should().BeTrue();
    }

    [Fact]
    public void LinkedCollisionSuppression_ShouldNotFollowReusedColliderIdsAfterColliderRemoval()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        scenario.Context.Environment.Gravity = Fixed64.Zero;
        ScenarioBody<LSSphereCollider> first = scenario.CreateSphere(Vector3d.Zero);
        ScenarioBody<LSSphereCollider> second = scenario.CreateSphere(Vector3d.Right * (Fixed64)2);
        scenario.Context.Constraints3D.RegisterJoint(CreateBallSocket(first.Body, second.Body));

        first.Collider.Deactivate();
        second.Collider.Deactivate();
        ScenarioBody<LSSphereCollider> replacementA = scenario.CreateSphere(Vector3d.Up * (Fixed64)4);
        ScenarioBody<LSSphereCollider> replacementB = scenario.CreateSphere(Vector3d.Up * (Fixed64)6);

        scenario.Context.Constraints3D.ShouldExcludeLinkedCollision(
            replacementA.Collider,
            replacementB.Collider).Should().BeFalse();
    }

    [Fact]
    public void LinkedCollisionSuppression_ShouldClearWhenHigherIdColliderIsRemoved()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSSphereCollider> first = scenario.CreateSphere(Vector3d.Zero);
        ScenarioBody<LSSphereCollider> second = scenario.CreateSphere(Vector3d.Right * (Fixed64)2);
        scenario.Context.Constraints3D.RegisterJoint(CreateBallSocket(first.Body, second.Body));
        int removedId = second.Collider.Id;

        second.Collider.Deactivate();
        ScenarioBody<LSSphereCollider> replacement = scenario.CreateSphere(Vector3d.Up * (Fixed64)4);

        replacement.Collider.Id.Should().Be(removedId);
        scenario.Context.Constraints3D.ShouldExcludeLinkedCollision(first.Collider, replacement.Collider).Should().BeFalse();
    }

    [Fact]
    public void RagdollFiltering_ShouldSuppressAdjacentLinksButAllowNonAdjacentByDefault()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSSphereCollider> root = scenario.CreateSphere(Vector3d.Zero);
        ScenarioBody<LSSphereCollider> middle = scenario.CreateSphere(Vector3d.Right * (Fixed64)2);
        ScenarioBody<LSSphereCollider> end = scenario.CreateSphere(Vector3d.Right * Fixed64.Half);
        RagdollRuntime3D runtime = scenario.Context.Constraints3D.RegisterRagdoll(new RagdollDefinition3D(
            new[]
            {
                new RagdollLinkDefinition3D(0, root.Body),
                new RagdollLinkDefinition3D(1, middle.Body),
                new RagdollLinkDefinition3D(2, end.Body)
            },
            new[]
            {
                new RagdollJointDefinition3D(0, 1, JointType3D.BallSocket, LocalFrame(Vector3d.Zero), LocalFrame(Vector3d.Zero)),
                new RagdollJointDefinition3D(1, 2, JointType3D.BallSocket, LocalFrame(Vector3d.Zero), LocalFrame(Vector3d.Zero))
            }));

        runtime.LinkCount.Should().Be(3);
        runtime.JointCount.Should().Be(2);
        scenario.Context.Constraints3D.ShouldExcludeLinkedCollision(root.Collider, middle.Collider).Should().BeTrue();
        scenario.Context.Constraints3D.ShouldExcludeLinkedCollision(middle.Collider, end.Collider).Should().BeTrue();
        scenario.Context.Constraints3D.ShouldExcludeLinkedCollision(root.Collider, end.Collider).Should().BeFalse();
    }

    [Fact]
    public void RagdollLinkDefinition_ShouldDeriveColliderFromBodyAndRejectNullBody()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSSphereCollider> body = scenario.CreateSphere(Vector3d.Zero);

        var link = new RagdollLinkDefinition3D(7, body.Body);
        Action nullBody = () => _ = new RagdollLinkDefinition3D(0, null!);

        link.LinkId.Should().Be(7);
        link.Body.Should().BeSameAs(body.Body);
        link.Collider.Should().BeSameAs(body.Collider);
        nullBody.Should().Throw<ArgumentNullException>().WithParameterName("body");
    }

    [Fact]
    public void RagdollFiltering_WithSuppressAllPolicy_ShouldSuppressNonAdjacentLinks()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSSphereCollider> root = scenario.CreateSphere(Vector3d.Zero);
        ScenarioBody<LSSphereCollider> middle = scenario.CreateSphere(Vector3d.Right * (Fixed64)2);
        ScenarioBody<LSSphereCollider> end = scenario.CreateSphere(Vector3d.Right * Fixed64.Half);

        scenario.Context.Constraints3D.RegisterRagdoll(new RagdollDefinition3D(
            new[]
            {
                new RagdollLinkDefinition3D(0, root.Body),
                new RagdollLinkDefinition3D(1, middle.Body),
                new RagdollLinkDefinition3D(2, end.Body)
            },
            new[]
            {
                new RagdollJointDefinition3D(0, 1, JointType3D.BallSocket, LocalFrame(Vector3d.Zero), LocalFrame(Vector3d.Zero)),
                new RagdollJointDefinition3D(1, 2, JointType3D.BallSocket, LocalFrame(Vector3d.Zero), LocalFrame(Vector3d.Zero))
            },
            RagdollSelfCollisionPolicy.SuppressAllLinks));

        scenario.Context.Constraints3D.ShouldExcludeLinkedCollision(root.Collider, end.Collider).Should().BeTrue();
    }

    [Fact]
    public void RagdollFiltering_WithCollideAllPolicy_ShouldAllowAllLinkCollisions()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSSphereCollider> root = scenario.CreateSphere(Vector3d.Zero);
        ScenarioBody<LSSphereCollider> middle = scenario.CreateSphere(Vector3d.Right * (Fixed64)2);
        ScenarioBody<LSSphereCollider> end = scenario.CreateSphere(Vector3d.Right * Fixed64.Half);

        scenario.Context.Constraints3D.RegisterRagdoll(new RagdollDefinition3D(
            new[]
            {
                new RagdollLinkDefinition3D(0, root.Body),
                new RagdollLinkDefinition3D(1, middle.Body),
                new RagdollLinkDefinition3D(2, end.Body)
            },
            new[]
            {
                new RagdollJointDefinition3D(0, 1, JointType3D.BallSocket, LocalFrame(Vector3d.Zero), LocalFrame(Vector3d.Zero)),
                new RagdollJointDefinition3D(1, 2, JointType3D.BallSocket, LocalFrame(Vector3d.Zero), LocalFrame(Vector3d.Zero))
            },
            RagdollSelfCollisionPolicy.CollideAllLinks));

        scenario.Context.Constraints3D.ShouldExcludeLinkedCollision(root.Collider, middle.Collider).Should().BeFalse();
        scenario.Context.Constraints3D.ShouldExcludeLinkedCollision(middle.Collider, end.Collider).Should().BeFalse();
        scenario.Context.Constraints3D.ShouldExcludeLinkedCollision(root.Collider, end.Collider).Should().BeFalse();
    }

    [Fact]
    public void RegisterRagdoll_WithInvalidJointPayload_ShouldFailAtomically()
    {
        using PhysicsScenarioBuilder scenario = CreateConstraintScenario();
        ScenarioBody<LSSphereCollider> root = scenario.CreateSphere(Vector3d.Zero);
        ScenarioBody<LSSphereCollider> middle = scenario.CreateSphere(Vector3d.Right * (Fixed64)2);
        ScenarioBody<LSSphereCollider> end = scenario.CreateSphere(Vector3d.Right * (Fixed64)4);
        var definition = new RagdollDefinition3D(
            new[]
            {
                new RagdollLinkDefinition3D(0, root.Body),
                new RagdollLinkDefinition3D(1, middle.Body),
                new RagdollLinkDefinition3D(2, end.Body)
            },
            new[]
            {
                new RagdollJointDefinition3D(0, 1, JointType3D.BallSocket, LocalFrame(Vector3d.Zero), LocalFrame(Vector3d.Zero)),
                new RagdollJointDefinition3D(
                    1,
                    2,
                    JointType3D.BallSocket,
                    LocalFrame(Vector3d.Zero),
                    LocalFrame(Vector3d.Zero),
                    JointLimit3D.Hinge(Fixed64.HalfPi),
                    JointMotor3D.Disabled,
                    JointCollisionPolicy.SuppressLinked)
            });

        Action act = () => scenario.Context.Constraints3D.RegisterRagdoll(definition);

        act.Should().Throw<ArgumentException>();
        scenario.Context.Constraints3D.RegisteredJointCount.Should().Be(0);
        scenario.Context.Constraints3D.EnabledJointCount.Should().Be(0);
        scenario.Context.Constraints3D.PeakJointCount.Should().Be(0);
        scenario.Context.Constraints3D.RegisteredRagdollCount.Should().Be(0);
        scenario.Context.Constraints3D.ShouldExcludeLinkedCollision(root.Collider, middle.Collider).Should().BeFalse();
        scenario.Context.Constraints3D.ShouldExcludeLinkedCollision(middle.Collider, end.Collider).Should().BeFalse();
    }

    [Fact]
    public void RegisterRagdoll_WithUnknownLinkReference_ShouldFailBeforeRuntimeStateIsCreated()
    {
        using PhysicsScenarioBuilder scenario = CreateConstraintScenario();
        ScenarioBody<LSSphereCollider> root = scenario.CreateSphere(Vector3d.Zero);
        ScenarioBody<LSSphereCollider> child = scenario.CreateSphere(Vector3d.Right * (Fixed64)2);
        var definition = new RagdollDefinition3D(
            new[]
            {
                new RagdollLinkDefinition3D(0, root.Body),
                new RagdollLinkDefinition3D(1, child.Body)
            },
            new[]
            {
                new RagdollJointDefinition3D(0, 99, JointType3D.BallSocket, LocalFrame(Vector3d.Zero), LocalFrame(Vector3d.Zero))
            });

        Action act = () => scenario.Context.Constraints3D.RegisterRagdoll(definition);

        act.Should().Throw<ArgumentException>().WithParameterName("linkId");
        scenario.Context.Constraints3D.RegisteredJointCount.Should().Be(0);
        scenario.Context.Constraints3D.RegisteredRagdollCount.Should().Be(0);
    }

    [Fact]
    public void BallSocketJoint_ShouldReduceAnchorSeparationThroughImpulses()
    {
        ConstraintState before;
        ConstraintState after;

        using (PhysicsScenarioBuilder scenario = CreateConstraintScenario())
        {
            ScenarioBody<LSSphereCollider> first = scenario.CreateSphere(Vector3d.Zero, preventAngularForces: true);
            ScenarioBody<LSSphereCollider> second = scenario.CreateSphere(Vector3d.Right * (Fixed64)3, preventAngularForces: true);
            Joint3D joint = scenario.Context.Constraints3D.RegisterJoint(new JointDefinition3D(
                first.Body,
                second.Body,
                LocalFrame(Vector3d.Right * Fixed64.Half),
                LocalFrame(-Vector3d.Right * Fixed64.Half),
                JointType3D.BallSocket,
                JointLimit3D.Unrestricted,
                JointMotor3D.Disabled,
                JointCollisionPolicy.SuppressLinked));

            before = CaptureConstraintState(first.Body, second.Body, joint);
            Step(scenario.Context, 12);
            after = CaptureConstraintState(first.Body, second.Body, joint);
        }

        after.AnchorDistanceSquared.Should().BeLessThan(before.AnchorDistanceSquared);
        after.JointSolvedRowCount.Should().BeGreaterThan(0);
        after.AccumulatedImpulseMagnitude.Should().BeGreaterThan(Fixed64.Zero);
    }

    [Fact]
    public void FixedJoint_ShouldReduceAngularFrameError()
    {
        Fixed64 beforeError;
        Fixed64 afterError;

        using (PhysicsScenarioBuilder scenario = CreateConstraintScenario())
        {
            ScenarioBody<LSSphereCollider> first = scenario.CreateSphere(Vector3d.Zero);
            ScenarioBody<LSSphereCollider> second = scenario.CreateSphere(
                Vector3d.Right * (Fixed64)2,
                FixedQuaternion.FromAxisAngle(Vector3d.Up, Fixed64.FromFraction(1, 3)));
            scenario.Context.Constraints3D.RegisterJoint(new JointDefinition3D(
                first.Body,
                second.Body,
                LocalFrame(Vector3d.Zero),
                LocalFrame(Vector3d.Zero),
                JointType3D.Fixed,
                JointLimit3D.Unrestricted,
                JointMotor3D.Disabled,
                JointCollisionPolicy.SuppressLinked));

            beforeError = FixedQuaternion.Angle(first.Body.Rotation, second.Body.Rotation);
            Step(scenario.Context, 16);
            afterError = FixedQuaternion.Angle(first.Body.Rotation, second.Body.Rotation);
        }

        afterError.Should().BeLessThan(beforeError);
    }

    [Fact]
    public void FixedJoint_WithQuaternionLogEndpointDrift_ShouldSolveWithoutDomainFailure()
    {
        using PhysicsScenarioBuilder scenario = CreateConstraintScenario();
        ScenarioBody<LSSphereCollider> first = scenario.CreateSphere(Vector3d.Zero);
        ScenarioBody<LSSphereCollider> second = scenario.CreateSphere(Vector3d.Right * (Fixed64)2);
        second.Body.SetRotation(new FixedQuaternion(
            Fixed64.FromRaw(4_096),
            Fixed64.Zero,
            Fixed64.Zero,
            Fixed64.One + Fixed64.MinIncrement));
        Joint3D joint = scenario.Context.Constraints3D.RegisterJoint(new JointDefinition3D(
            first.Body,
            second.Body,
            LocalFrame(Vector3d.Zero),
            LocalFrame(Vector3d.Zero),
            JointType3D.Fixed,
            JointLimit3D.Unrestricted,
            JointMotor3D.Disabled,
            JointCollisionPolicy.SuppressLinked));

        Action solve = () => Step(scenario.Context, 1);

        solve.Should().NotThrow();
        joint.LastSolvedRowCount.Should().BeGreaterThan(0);
    }

    [Fact]
    public void HingeJoint_ShouldAlignHingeAxesWithoutLockingHingeRotation()
    {
        using PhysicsScenarioBuilder scenario = CreateConstraintScenario();
        ScenarioBody<LSSphereCollider> first = scenario.CreateSphere(Vector3d.Zero);
        ScenarioBody<LSSphereCollider> second = scenario.CreateSphere(
            Vector3d.Right * (Fixed64)2,
            FixedQuaternion.FromAxisAngle(Vector3d.Forward, Fixed64.FromFraction(1, 4)));
        Joint3D joint = scenario.Context.Constraints3D.RegisterJoint(new JointDefinition3D(
            first.Body,
            second.Body,
            LocalFrame(Vector3d.Zero),
            LocalFrame(Vector3d.Zero),
            JointType3D.Hinge,
            JointLimit3D.Hinge(Fixed64.FromFraction(1, 2)),
            JointMotor3D.Disabled,
            JointCollisionPolicy.SuppressLinked));
        Fixed64 before = Vector3d.Cross(first.Body.Right, second.Body.Right).MagnitudeSquared;

        Step(scenario.Context, 16);

        Fixed64 after = Vector3d.Cross(first.Body.Right, second.Body.Right).MagnitudeSquared;
        after.Should().BeLessThan(before);
        joint.LastSolvedRowCount.Should().BeGreaterThanOrEqualTo(4);
    }

    [Fact]
    public void HingeJoint_WithinLimit_ShouldSkipLimitCorrection()
    {
        using PhysicsScenarioBuilder scenario = CreateConstraintScenario();
        ScenarioBody<LSSphereCollider> first = scenario.CreateSphere(Vector3d.Zero, immovable: true);
        ScenarioBody<LSSphereCollider> second = scenario.CreateSphere(
            Vector3d.Right * (Fixed64)2,
            FixedQuaternion.FromAxisAngle(Vector3d.Right, Fixed64.FromFraction(1, 8)));
        Joint3D joint = scenario.Context.Constraints3D.RegisterJoint(new JointDefinition3D(
            first.Body,
            second.Body,
            LocalFrame(Vector3d.Zero),
            LocalFrame(Vector3d.Zero),
            JointType3D.Hinge,
            JointLimit3D.Hinge(Fixed64.Half),
            JointMotor3D.Disabled,
            JointCollisionPolicy.SuppressLinked));

        Step(scenario.Context, 2);

        joint.LastSolveMetrics.AngularLimitErrorMagnitude.Should().Be(Fixed64.Zero);
        joint.LastSolvedRowCount.Should().BeGreaterThan(0);
    }

    [Theory]
    [InlineData(JointType3D.Hinge)]
    [InlineData(JointType3D.ConeTwist)]
    public void UnrestrictedAngularJoint_ShouldAdmitOnlyAnchorRows(JointType3D type)
    {
        using PhysicsScenarioBuilder scenario = CreateConstraintScenario();
        ScenarioBody<LSSphereCollider> first = scenario.CreateSphere(Vector3d.Zero, immovable: true);
        Vector3d rotationAxis = type == JointType3D.Hinge ? Vector3d.Right : Vector3d.Up;
        ScenarioBody<LSSphereCollider> second = scenario.CreateSphere(
            Vector3d.Right * (Fixed64)2,
            FixedQuaternion.FromAxisAngle(rotationAxis, Fixed64.HalfPi));
        FixedQuaternion initialRotation = second.Body.Rotation;
        Joint3D joint = scenario.Context.Constraints3D.RegisterJoint(new JointDefinition3D(
            first.Body,
            second.Body,
            LocalFrame(Vector3d.Zero),
            LocalFrame(Vector3d.Zero),
            type,
            JointLimit3D.Unrestricted,
            JointMotor3D.Disabled,
            JointCollisionPolicy.SuppressLinked));
        Step(scenario.Context, 1);

        joint.LastSolvedRowCount.Should().Be(3);
        joint.LastSolveMetrics.AngularLimitErrorMagnitude.Should().Be(Fixed64.Zero);
        second.Body.AngularVelocity.Should().Be(Vector3d.Zero);
        second.Body.Rotation.Should().Be(initialRotation);
    }

    [Theory]
    [InlineData(-90)]
    [InlineData(90)]
    public void HingeJoint_ShouldReportSignedLimitViolations(int degrees)
    {
        using PhysicsScenarioBuilder scenario = CreateConstraintScenario();
        ScenarioBody<LSSphereCollider> first = scenario.CreateSphere(Vector3d.Zero, immovable: true);
        ScenarioBody<LSSphereCollider> second = scenario.CreateSphere(
            Vector3d.Right * (Fixed64)2,
            FixedQuaternion.FromAxisAngle(Vector3d.Right, FixedMath.DegToRad((Fixed64)degrees)));
        Joint3D joint = scenario.Context.Constraints3D.RegisterJoint(new JointDefinition3D(
            first.Body,
            second.Body,
            LocalFrame(Vector3d.Zero),
            LocalFrame(Vector3d.Zero),
            JointType3D.Hinge,
            JointLimit3D.Hinge(Fixed64.FromFraction(1, 8)),
            JointMotor3D.Disabled,
            JointCollisionPolicy.SuppressLinked));

        Step(scenario.Context, 2);

        joint.LastSolveMetrics.AngularLimitErrorMagnitude.Should().BeGreaterThan(Fixed64.Zero);
        joint.LastSolvedRowCount.Should().BeGreaterThan(3);
    }

    [Fact]
    public void FixedJoint_WithFrozenRotation_ShouldLeaveAngularStateUntouched()
    {
        using PhysicsScenarioBuilder scenario = CreateConstraintScenario();
        ScenarioBody<LSSphereCollider> first = scenario.CreateSphere(Vector3d.Zero, immovable: true);
        ScenarioBody<LSSphereCollider> second = scenario.CreateSphere(
            Vector3d.Right * (Fixed64)2,
            FixedQuaternion.FromAxisAngle(Vector3d.Up, Fixed64.HalfPi));
        second.Body.FreezeAxes = BodyFreezeAxes3D.Rotation;
        FixedQuaternion initialRotation = second.Body.Rotation;
        Joint3D joint = scenario.Context.Constraints3D.RegisterJoint(new JointDefinition3D(
            first.Body,
            second.Body,
            LocalFrame(Vector3d.Zero),
            LocalFrame(Vector3d.Zero),
            JointType3D.Fixed,
            JointLimit3D.Unrestricted,
            JointMotor3D.Disabled,
            JointCollisionPolicy.SuppressLinked));

        Step(scenario.Context, 4);

        second.Body.Rotation.Should().Be(initialRotation);
        joint.LastSolvedRowCount.Should().BeGreaterThan(0);
    }

    [Fact]
    public void ConeTwistJoint_ShouldReportConeSwingLimitViolations()
    {
        using PhysicsScenarioBuilder scenario = CreateConstraintScenario();
        ScenarioBody<LSSphereCollider> first = scenario.CreateSphere(Vector3d.Zero, immovable: true);
        ScenarioBody<LSSphereCollider> second = scenario.CreateSphere(
            Vector3d.Right * (Fixed64)2,
            FixedQuaternion.FromAxisAngle(Vector3d.Up, Fixed64.HalfPi));
        Joint3D joint = scenario.Context.Constraints3D.RegisterJoint(new JointDefinition3D(
            first.Body,
            second.Body,
            LocalFrame(Vector3d.Zero),
            LocalFrame(Vector3d.Zero),
            JointType3D.ConeTwist,
            JointLimit3D.ConeTwist(Fixed64.FromFraction(1, 8), Fixed64.HalfPi),
            JointMotor3D.Disabled,
            JointCollisionPolicy.SuppressLinked));

        Step(scenario.Context, 2);

        joint.LastSolveMetrics.AngularLimitErrorMagnitude.Should().BeGreaterThan(Fixed64.Zero);
    }

    [Fact]
    public void ConeTwistJoint_WithinLimits_ShouldSkipLimitCorrection()
    {
        using PhysicsScenarioBuilder scenario = CreateConstraintScenario();
        ScenarioBody<LSSphereCollider> first = scenario.CreateSphere(Vector3d.Zero, immovable: true);
        ScenarioBody<LSSphereCollider> second = scenario.CreateSphere(
            Vector3d.Right * (Fixed64)2,
            FixedQuaternion.FromAxisAngle(Vector3d.Up, Fixed64.FromFraction(1, 8)));
        FixedQuaternion initialRotation = second.Body.Rotation;
        Joint3D joint = scenario.Context.Constraints3D.RegisterJoint(new JointDefinition3D(
            first.Body,
            second.Body,
            LocalFrame(Vector3d.Zero),
            LocalFrame(Vector3d.Zero),
            JointType3D.ConeTwist,
            JointLimit3D.ConeTwist(Fixed64.HalfPi, Fixed64.Half),
            JointMotor3D.Disabled,
            JointCollisionPolicy.SuppressLinked));

        Step(scenario.Context, 1);

        joint.LastSolveMetrics.AngularLimitErrorMagnitude.Should().Be(Fixed64.Zero);
        joint.LastSolvedRowCount.Should().Be(3);
        second.Body.AngularVelocity.Should().Be(Vector3d.Zero);
        second.Body.Rotation.Should().Be(initialRotation);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void ConeTwistJoint_WithAntiparallelAxes_ShouldUseDeterministicSwingFallback(bool verticalAxis)
    {
        using PhysicsScenarioBuilder scenario = CreateConstraintScenario();
        scenario.Context.Settings.DiscreteSolverIterations = 1;
        ScenarioBody<LSSphereCollider> first = scenario.CreateSphere(Vector3d.Zero, immovable: true);
        FixedQuaternion frameRotation = verticalAxis
            ? FixedQuaternion.FromAxisAngle(Vector3d.Right, -Fixed64.HalfPi)
            : FixedQuaternion.Identity;
        FixedQuaternion bodyRotation = FixedQuaternion.FromAxisAngle(
            verticalAxis ? Vector3d.Right : Vector3d.Up,
            Fixed64.Pi);
        ScenarioBody<LSSphereCollider> second = scenario.CreateSphere(
            Vector3d.Right * (Fixed64)2,
            bodyRotation);
        var localFrame = new FixedTransform(Vector3d.Zero, frameRotation, Vector3d.One);
        Joint3D joint = scenario.Context.Constraints3D.RegisterJoint(new JointDefinition3D(
            first.Body,
            second.Body,
            localFrame,
            localFrame,
            JointType3D.ConeTwist,
            JointLimit3D.ConeTwist(Fixed64.HalfPi, Fixed64.Pi),
            JointMotor3D.Disabled,
            JointCollisionPolicy.SuppressLinked));
        Vector3d forwardA = (first.Body.Rotation * joint.LocalFrameA.Rotation).Normalized * Vector3d.Forward;
        Vector3d forwardB = (second.Body.Rotation * joint.LocalFrameB.Rotation).Normalized * Vector3d.Forward;
        Vector3d.Dot(forwardA, forwardB).Should().BeLessThan(-Fixed64.FromFraction(99, 100));

        Step(scenario.Context, 1);

        joint.LastSolveMetrics.AngularLimitErrorMagnitude.Should().BeGreaterThan(Fixed64.Zero);
        joint.LastSolvedRowCount.Should().Be(4);
        joint.GetCachedImpulse(3).Should().BeLessThan(Fixed64.Zero);
        if (verticalAxis)
        {
            second.Body.AngularVelocity.X.Should().Be(Fixed64.Zero);
            second.Body.AngularVelocity.Y.Should().Be(Fixed64.Zero);
            second.Body.AngularVelocity.Z.Abs().Should().BeGreaterThan(Fixed64.Zero);
        }
        else
        {
            second.Body.AngularVelocity.X.Should().BeGreaterThan(Fixed64.Zero);
            second.Body.AngularVelocity.Y.Should().Be(Fixed64.Zero);
            second.Body.AngularVelocity.Z.Should().Be(Fixed64.Zero);
        }
    }

    [Fact]
    public void ConeTwistJoint_ShouldReportTwistLimitViolations()
    {
        using PhysicsScenarioBuilder scenario = CreateConstraintScenario();
        ScenarioBody<LSSphereCollider> first = scenario.CreateSphere(Vector3d.Zero, immovable: true);
        ScenarioBody<LSSphereCollider> second = scenario.CreateSphere(
            Vector3d.Right * (Fixed64)2,
            FixedQuaternion.FromAxisAngle(Vector3d.Forward, Fixed64.HalfPi));
        Joint3D joint = scenario.Context.Constraints3D.RegisterJoint(new JointDefinition3D(
            first.Body,
            second.Body,
            LocalFrame(Vector3d.Zero),
            LocalFrame(Vector3d.Zero),
            JointType3D.ConeTwist,
            JointLimit3D.ConeTwist(Fixed64.HalfPi, Fixed64.FromFraction(1, 8)),
            JointMotor3D.Disabled,
            JointCollisionPolicy.SuppressLinked));

        Step(scenario.Context, 2);

        joint.LastSolveMetrics.AngularLimitErrorMagnitude.Should().BeGreaterThan(Fixed64.Zero);
    }

    [Fact]
    public void ConeTwistJoint_ShouldReportNegativeTwistLimitViolations()
    {
        using PhysicsScenarioBuilder scenario = CreateConstraintScenario();
        ScenarioBody<LSSphereCollider> first = scenario.CreateSphere(Vector3d.Zero, immovable: true);
        ScenarioBody<LSSphereCollider> second = scenario.CreateSphere(
            Vector3d.Right * (Fixed64)2,
            FixedQuaternion.FromAxisAngle(Vector3d.Forward, -Fixed64.HalfPi));
        Joint3D joint = scenario.Context.Constraints3D.RegisterJoint(new JointDefinition3D(
            first.Body,
            second.Body,
            LocalFrame(Vector3d.Zero),
            LocalFrame(Vector3d.Zero),
            JointType3D.ConeTwist,
            JointLimit3D.ConeTwist(Fixed64.HalfPi, Fixed64.FromFraction(1, 8)),
            JointMotor3D.Disabled,
            JointCollisionPolicy.SuppressLinked));

        Step(scenario.Context, 2);

        joint.LastSolveMetrics.AngularLimitErrorMagnitude.Should().BeGreaterThan(Fixed64.Zero);
    }

    [Theory]
    [InlineData(1, false)]
    [InlineData(1, true)]
    [InlineData(-1, false)]
    [InlineData(-1, true)]
    public void ConeTwistJoint_WithCombinedSwingAndTwist_ShouldEnforceDecomposedTwistLimit(
        int twistSign,
        bool negateQuaternion)
    {
        using PhysicsScenarioBuilder scenario = CreateConstraintScenario();
        scenario.Context.Settings.DiscreteSolverIterations = 1;
        ScenarioBody<LSSphereCollider> first = scenario.CreateSphere(Vector3d.Zero, immovable: true);
        FixedQuaternion swing = FixedQuaternion.FromAxisAngle(Vector3d.Up, Fixed64.HalfPi);
        FixedQuaternion twist = FixedQuaternion.FromAxisAngle(Vector3d.Forward, twistSign * Fixed64.HalfPi);
        FixedQuaternion combined = (swing * twist).Normalized;
        ScenarioBody<LSSphereCollider> second = scenario.CreateSphere(
            Vector3d.Right * (Fixed64)2,
            negateQuaternion ? -combined : combined);
        Joint3D joint = scenario.Context.Constraints3D.RegisterJoint(new JointDefinition3D(
            first.Body,
            second.Body,
            LocalFrame(Vector3d.Zero),
            LocalFrame(Vector3d.Zero),
            JointType3D.ConeTwist,
            JointLimit3D.ConeTwist(Fixed64.Pi, Fixed64.FromFraction(13, 10)),
            JointMotor3D.Disabled,
            JointCollisionPolicy.SuppressLinked));

        Step(scenario.Context, 1);

        joint.LastSolveMetrics.AngularLimitErrorMagnitude.Should().BeGreaterThan(Fixed64.Zero);
        joint.LastSolvedRowCount.Should().Be(4);
        (joint.GetCachedImpulse(3) * twistSign).Should().BeLessThan(Fixed64.Zero);
        second.Body.AngularVelocity.X.Should().Be(Fixed64.Zero);
        second.Body.AngularVelocity.Y.Should().Be(Fixed64.Zero);
        (second.Body.AngularVelocity.Z * twistSign).Should().BeLessThan(Fixed64.Zero);
    }

    [Fact]
    public void ConeTwistJoint_WithPurePiTwist_ShouldCanonicalizeQuaternionSign()
    {
        (Fixed64 CachedImpulse, Vector3d AngularVelocity, Fixed64 LimitError, int RowCount) canonical =
            RunPurePiTwist(negateQuaternion: false);
        (Fixed64 CachedImpulse, Vector3d AngularVelocity, Fixed64 LimitError, int RowCount) negated =
            RunPurePiTwist(negateQuaternion: true);

        canonical.Should().Be(negated);
        canonical.CachedImpulse.Should().BeGreaterThan(Fixed64.Zero);
        canonical.AngularVelocity.X.Should().Be(Fixed64.Zero);
        canonical.AngularVelocity.Y.Should().Be(Fixed64.Zero);
        canonical.AngularVelocity.Z.Should().BeGreaterThan(Fixed64.Zero);
        canonical.LimitError.Should().BeGreaterThan(Fixed64.Zero);
        canonical.RowCount.Should().Be(4);
    }

    [Fact]
    public void ConstraintSolver_ShouldRespectFrozenBodyAxes()
    {
        using PhysicsScenarioBuilder scenario = CreateConstraintScenario();
        ScenarioBody<LSSphereCollider> first = scenario.CreateSphere(Vector3d.Zero, preventAngularForces: true);
        ScenarioBody<LSSphereCollider> second = scenario.CreateSphere(Vector3d.Right * (Fixed64)3, preventAngularForces: true);
        second.Body.FreezeAxes = BodyFreezeAxes3D.PositionX | BodyFreezeAxes3D.Rotation;
        scenario.Context.Constraints3D.RegisterJoint(CreateBallSocket(first.Body, second.Body));
        Vector3d frozenStart = second.Body.Position3d;

        Step(scenario.Context, 12);

        second.Body.Position3d.X.Should().Be(frozenStart.X);
        first.Body.Position3d.X.Should().BeGreaterThan(Fixed64.Zero);
    }

    [Fact]
    public void ConstraintIsland_ShouldWakeSleepingLinkedBodiesAsOneIsland()
    {
        using PhysicsScenarioBuilder scenario = CreateConstraintScenario();
        ScenarioBody<LSSphereCollider> sleeping = scenario.CreateSphere(Vector3d.Zero);
        ScenarioBody<LSSphereCollider> driver = scenario.CreateSphere(Vector3d.Right * (Fixed64)3);
        scenario.Context.Constraints3D.RegisterJoint(CreateBallSocket(sleeping.Body, driver.Body));
        sleeping.Body.Sleep();
        driver.Body.AddLinearImpulse(-Vector3d.Right * (Fixed64)16);

        scenario.Context.Simulate();
        scenario.Context.LateSimulate();

        sleeping.Body.IsSleeping.Should().BeFalse();
        driver.Body.IsSleeping.Should().BeFalse();
    }

    [Fact]
    public void ConstraintSolver_WithOnlyFrozenBodies_ShouldKeepJointRegisteredWithoutRows()
    {
        using PhysicsScenarioBuilder scenario = CreateConstraintScenario();
        ScenarioBody<LSSphereCollider> first = scenario.CreateSphere(Vector3d.Zero, immovable: true);
        ScenarioBody<LSSphereCollider> second = scenario.CreateSphere(Vector3d.Right * (Fixed64)2, immovable: true);
        Joint3D joint = scenario.Context.Constraints3D.RegisterJoint(CreateBallSocket(first.Body, second.Body));

        Step(scenario.Context, 1);

        joint.IsEnabled.Should().BeTrue();
        scenario.Context.Constraints3D.EnabledJointCount.Should().Be(1);
        joint.LastSolvedRowCount.Should().Be(0);
        joint.AccumulatedImpulseMagnitude.Should().Be(Fixed64.Zero);
    }

    [Fact]
    public void ConstraintSolveOrder_ShouldBeDeterministicAcrossJointRegistrationOrder()
    {
        ConstraintState first = RunConstraintChain(registerForward: true);
        ConstraintState second = RunConstraintChain(registerForward: false);

        second.Should().Be(first);
    }

    [Fact]
    public void MotorTarget_ShouldPullTowardTargetDeterministicallyAndRespectDisabledStrength()
    {
        MotorState disabled = RunMotorScenario(strength: Fixed64.Zero);
        MotorState enabledA = RunMotorScenario(strength: (Fixed64)4);
        MotorState enabledB = RunMotorScenario(strength: (Fixed64)4);

        disabled.AngularErrorAfter.Should().Be(disabled.AngularErrorBefore);
        enabledA.AngularErrorAfter.Should().BeLessThan(enabledA.AngularErrorBefore);
        enabledB.Should().Be(enabledA);
    }

    [Fact]
    public void ConstraintServiceMotorHelpers_ShouldUpdateJointAndRagdollTargets()
    {
        using PhysicsScenarioBuilder scenario = CreateConstraintScenario();
        ScenarioBody<LSSphereCollider> root = scenario.CreateSphere(Vector3d.Zero, isKinematic: true);
        ScenarioBody<LSSphereCollider> child = scenario.CreateSphere(Vector3d.Right * (Fixed64)2, isKinematic: true);
        RagdollRuntime3D ragdoll = scenario.Context.Constraints3D.RegisterRagdoll(CreateTwoLinkRagdoll(root, child));
        Joint3D joint = ragdoll.GetJoint(0);
        var motor = new JointMotor3D(
            FixedQuaternion.Identity,
            (Fixed64)2,
            Fixed64.Half,
            Fixed64.One);

        scenario.Context.Constraints3D.SetRagdollPoseTargets(ragdoll, new[] { motor });
        scenario.Context.Constraints3D.SetJointMotorTarget(joint.Id, FixedQuaternion.FromAxisAngle(Vector3d.Up, Fixed64.FromFraction(1, 5))).Should().BeTrue();
        scenario.Context.Constraints3D.SetJointMotorTarget(99, FixedQuaternion.Identity).Should().BeFalse();

        joint.Motor.AngularDriveStrength.Should().Be((Fixed64)2);
        joint.Motor.TargetLocalRotation.Should().Be(FixedQuaternion.FromAxisAngle(Vector3d.Up, Fixed64.FromFraction(1, 5)).Normalized);

        scenario.Context.Constraints3D.ClearJointMotorTarget(joint.Id).Should().BeTrue();
        scenario.Context.Constraints3D.ClearJointMotorTarget(99).Should().BeFalse();

        joint.Motor.IsEnabled.Should().BeFalse();
    }

    [Fact]
    public void SetRagdollPoseTargets_WithForeignRuntime_ShouldReject()
    {
        using PhysicsScenarioBuilder sourceScenario = CreateConstraintScenario();
        using PhysicsScenarioBuilder targetScenario = CreateConstraintScenario();
        ScenarioBody<LSSphereCollider> root = sourceScenario.CreateSphere(Vector3d.Zero, isKinematic: true);
        ScenarioBody<LSSphereCollider> child = sourceScenario.CreateSphere(Vector3d.Right * (Fixed64)2, isKinematic: true);
        RagdollRuntime3D ragdoll = sourceScenario.Context.Constraints3D.RegisterRagdoll(CreateTwoLinkRagdoll(root, child));
        var motor = new JointMotor3D(FixedQuaternion.Identity, Fixed64.One, Fixed64.Zero, Fixed64.One);

        Action act = () => targetScenario.Context.Constraints3D.SetRagdollPoseTargets(ragdoll, new[] { motor });

        act.Should().Throw<ArgumentException>()
            .WithParameterName("ragdoll");
    }

    [Fact]
    public void RagdollRuntime_ShouldActivateDynamicAndDeactivateToKinematicDeterministically()
    {
        using PhysicsScenarioBuilder scenario = CreateConstraintScenario();
        ScenarioBody<LSSphereCollider> root = scenario.CreateSphere(Vector3d.Zero, isKinematic: true);
        ScenarioBody<LSSphereCollider> child = scenario.CreateSphere(Vector3d.Right * (Fixed64)2, isKinematic: true);
        RagdollRuntime3D runtime = scenario.Context.Constraints3D.RegisterRagdoll(CreateTwoLinkRagdoll(root, child));

        runtime.GetLink(0).Should().BeSameAs(root.Body);
        runtime.GetLink(1).Should().BeSameAs(child.Body);

        runtime.ActivateDynamic();

        runtime.IsActive.Should().BeTrue();
        root.Body.IsKinematic.Should().BeFalse();
        child.Body.IsKinematic.Should().BeFalse();
        runtime.GetJoint(0).IsEnabled.Should().BeTrue();

        runtime.DeactivateToKinematic();

        runtime.IsActive.Should().BeFalse();
        root.Body.IsKinematic.Should().BeTrue();
        child.Body.IsKinematic.Should().BeTrue();
        runtime.GetJoint(0).IsEnabled.Should().BeFalse();
    }

    [Fact]
    public void RagdollRuntime_WithSingleLinkAndNoJoints_ShouldEmitActivationDiagnostic()
    {
        using PhysicsScenarioBuilder scenario = CreateConstraintScenario();
        ScenarioBody<LSSphereCollider> link = scenario.CreateSphere(Vector3d.Zero, isKinematic: true);
        RagdollRuntime3D runtime = scenario.Context.Constraints3D.RegisterRagdoll(new RagdollDefinition3D(
            new[] { new RagdollLinkDefinition3D(7, link.Body) },
            Array.Empty<RagdollJointDefinition3D>()));
        scenario.Context.Diagnostics.Enable(eventCapacity: 1, drawCommandCapacity: 0);

        runtime.ActivateDynamic();

        runtime.GetLink(0).Should().BeSameAs(link.Body);
        runtime.IsActive.Should().BeTrue();
        link.Body.IsKinematic.Should().BeFalse();
        scenario.Context.Diagnostics.Events.Should().ContainSingle();
        GravitasDiagnosticEvent diagnosticEvent = scenario.Context.Diagnostics.Events[0];
        diagnosticEvent.Kind.Should().Be(GravitasDiagnosticEventKind.RagdollActivated);
        diagnosticEvent.BodyId.Should().Be(runtime.Id);
        diagnosticEvent.DataA.Should().Be(1);
        diagnosticEvent.DataB.Should().Be(0);
        diagnosticEvent.Hit.Should().BeTrue();
    }

    [Theory]
    [MemberData(nameof(Transports))]
    public void RagdollRecordData_ShouldApplyInactiveStateToActiveRuntime(GravitasSerializationTransport transport)
    {
        using PhysicsScenarioBuilder sourceScenario = CreateConstraintScenario();
        ScenarioBody<LSSphereCollider> sourceRoot = sourceScenario.CreateSphere(Vector3d.Zero);
        ScenarioBody<LSSphereCollider> sourceChild = sourceScenario.CreateSphere(Vector3d.Right * (Fixed64)2);
        RagdollRuntime3D source = sourceScenario.Context.Constraints3D.RegisterRagdoll(CreateTwoLinkRagdoll(sourceRoot, sourceChild));
        source.DeactivateToKinematic();
        object payload = GravitasSerializationHarness.Serialize(source, transport);

        using PhysicsScenarioBuilder targetScenario = CreateConstraintScenario();
        ScenarioBody<LSSphereCollider> targetRoot = targetScenario.CreateSphere(Vector3d.Zero);
        ScenarioBody<LSSphereCollider> targetChild = targetScenario.CreateSphere(Vector3d.Right * (Fixed64)2);
        RagdollRuntime3D target = targetScenario.Context.Constraints3D.RegisterRagdoll(CreateTwoLinkRagdoll(targetRoot, targetChild));
        targetScenario.Context.Diagnostics.Enable(eventCapacity: 1, drawCommandCapacity: 0);

        GravitasSerializationHarness.Populate(target, payload, transport);

        target.IsActive.Should().BeFalse();
        targetRoot.Body.IsKinematic.Should().BeTrue();
        targetChild.Body.IsKinematic.Should().BeTrue();
        target.GetJoint(0).IsEnabled.Should().BeFalse();
        targetScenario.Context.Constraints3D.EnabledJointCount.Should().Be(0);
        targetScenario.Context.Diagnostics.Events.Should().ContainSingle();
        targetScenario.Context.Diagnostics.Events[0].Hit.Should().BeFalse();
    }

    [Theory]
    [MemberData(nameof(Transports))]
    public void JointRecordData_ShouldRoundTripAuthoritativeState(GravitasSerializationTransport transport)
    {
        using PhysicsScenarioBuilder scenario = CreateConstraintScenario();
        ScenarioBody<LSSphereCollider> first = scenario.CreateSphere(Vector3d.Zero);
        ScenarioBody<LSSphereCollider> second = scenario.CreateSphere(Vector3d.Right * (Fixed64)2);
        Joint3D source = scenario.Context.Constraints3D.RegisterJoint(new JointDefinition3D(
            first.Body,
            second.Body,
            LocalFrame(Vector3d.Right),
            LocalFrame(-Vector3d.Right),
            JointType3D.ConeTwist,
            JointLimit3D.ConeTwist(Fixed64.FromFraction(1, 3), Fixed64.FromFraction(1, 4)),
            new JointMotor3D(FixedQuaternion.FromAxisAngle(Vector3d.Up, Fixed64.FromFraction(1, 5)), (Fixed64)3, Fixed64.Half, Fixed64.One),
            JointCollisionPolicy.Collide));
        source.IsEnabled = false;

        object payload = GravitasSerializationHarness.Serialize(source, transport);

        Joint3D target = scenario.Context.Constraints3D.RegisterJoint(CreateBallSocket(first.Body, second.Body));
        GravitasSerializationHarness.Populate(target, payload, transport);

        target.IsEnabled.Should().BeFalse();
        target.Type.Should().Be(JointType3D.ConeTwist);
        target.LocalFrameA.Position.Should().Be(Vector3d.Right);
        target.LocalFrameB.Position.Should().Be(-Vector3d.Right);
        target.Limits.Should().Be(source.Limits);
        target.Motor.Should().Be(source.Motor);
        target.CollisionPolicy.Should().Be(JointCollisionPolicy.Collide);
    }

    [Theory]
    [MemberData(nameof(Transports))]
    public void JointRecordData_ShouldRoundTripHingeLimitState(GravitasSerializationTransport transport)
    {
        using PhysicsScenarioBuilder scenario = CreateConstraintScenario();
        ScenarioBody<LSSphereCollider> first = scenario.CreateSphere(Vector3d.Zero);
        ScenarioBody<LSSphereCollider> second = scenario.CreateSphere(Vector3d.Right * (Fixed64)2);
        Joint3D source = scenario.Context.Constraints3D.RegisterJoint(new JointDefinition3D(
            first.Body,
            second.Body,
            LocalFrame(Vector3d.Zero),
            LocalFrame(Vector3d.Zero),
            JointType3D.Hinge,
            JointLimit3D.Hinge(Fixed64.HalfPi),
            JointMotor3D.Disabled,
            JointCollisionPolicy.SuppressLinked));

        object payload = GravitasSerializationHarness.Serialize(source, transport);

        Joint3D target = scenario.Context.Constraints3D.RegisterJoint(CreateBallSocket(first.Body, second.Body));
        GravitasSerializationHarness.Populate(target, payload, transport);

        target.IsEnabled.Should().BeTrue();
        target.Type.Should().Be(JointType3D.Hinge);
        target.Limits.Should().Be(source.Limits);
        target.Motor.IsEnabled.Should().BeFalse();
        target.Motor.AngularDriveStrength.Should().Be(Fixed64.Zero);
        target.Motor.MaximumMotorImpulse.Should().Be(Fixed64.Zero);
        target.CollisionPolicy.Should().Be(JointCollisionPolicy.SuppressLinked);
    }

    [Fact]
    public void JointRecordData_ShouldSynchronizeEnabledCountWhenLoadingDisabledState()
    {
        using PhysicsScenarioBuilder scenario = CreateConstraintScenario();
        ScenarioBody<LSSphereCollider> first = scenario.CreateSphere(Vector3d.Zero);
        ScenarioBody<LSSphereCollider> second = scenario.CreateSphere(Vector3d.Right * (Fixed64)2);
        Joint3D source = scenario.Context.Constraints3D.RegisterJoint(CreateBallSocket(first.Body, second.Body));
        source.IsEnabled = false;
        object payload = GravitasSerializationHarness.Serialize(source, GravitasSerializationTransport.Json);

        Joint3D target = scenario.Context.Constraints3D.RegisterJoint(CreateBallSocket(first.Body, second.Body));

        GravitasSerializationHarness.Populate(target, payload, GravitasSerializationTransport.Json);

        target.IsEnabled.Should().BeFalse();
        scenario.Context.Constraints3D.EnabledJointCount.Should().Be(0);
    }

    [Fact]
    public void JointRecordData_WithIncompatibleLoadedLimitPayload_ShouldReject()
    {
        using PhysicsScenarioBuilder scenario = CreateConstraintScenario();
        ScenarioBody<LSSphereCollider> first = scenario.CreateSphere(Vector3d.Zero);
        ScenarioBody<LSSphereCollider> second = scenario.CreateSphere(Vector3d.Right * (Fixed64)2);
        Joint3D target = scenario.Context.Constraints3D.RegisterJoint(CreateBallSocket(first.Body, second.Body));
        var chronicler = new InvalidRecordPayloadChronicler(new Dictionary<string, object>
        {
            [nameof(Joint3D.IsEnabled)] = true,
            ["Type"] = JointType3D.BallSocket,
            ["LimitKind"] = JointLimitKind3D.Hinge,
            ["MaxHingeAngle"] = Fixed64.HalfPi,
            ["MaxConeAngle"] = Fixed64.Zero,
            ["MaxTwistAngle"] = Fixed64.Zero,
            ["MotorTarget"] = FixedQuaternion.Identity,
            ["MotorStrength"] = Fixed64.Zero,
            ["MotorDamping"] = Fixed64.Zero,
            ["MaxMotorImpulse"] = Fixed64.Zero,
            ["CollisionPolicy"] = JointCollisionPolicy.SuppressLinked,
            ["LocalFrameAPosition"] = Vector3d.Zero,
            ["LocalFrameARotation"] = FixedQuaternion.Identity,
            ["LocalFrameBPosition"] = Vector3d.Zero,
            ["LocalFrameBRotation"] = FixedQuaternion.Identity
        });

        Action act = () => target.RecordData(chronicler);

        act.Should().Throw<ArgumentException>();
        target.Type.Should().Be(JointType3D.BallSocket);
        target.Limits.Kind.Should().Be(JointLimitKind3D.Unrestricted);
    }

    [Fact]
    public void EnabledDiagnostics_ShouldRecordJointLifecycleAndImpulseEvents()
    {
        using PhysicsScenarioBuilder scenario = CreateConstraintScenario();
        ScenarioBody<LSSphereCollider> first = scenario.CreateSphere(Vector3d.Zero);
        ScenarioBody<LSSphereCollider> second = scenario.CreateSphere(Vector3d.Right * (Fixed64)3);
        scenario.Context.Diagnostics.Enable(eventCapacity: 16, drawCommandCapacity: 16);

        Joint3D joint = scenario.Context.Constraints3D.RegisterJoint(CreateBallSocket(first.Body, second.Body));
        scenario.Context.Diagnostics.CaptureJoint(joint, GravitasDiagnosticColor.Cyan);
        Step(scenario.Context, 4);
        scenario.Context.Constraints3D.RemoveJoint(joint.Id);

        ReadOnlySpan<GravitasDiagnosticEvent> events = scenario.Context.Diagnostics.Events;
        events[0].Kind.Should().Be(GravitasDiagnosticEventKind.JointRegistered);
        events.Should().Contain(e => e.Kind == GravitasDiagnosticEventKind.JointImpulse && e.JointId == joint.Id);
        events[^1].Kind.Should().Be(GravitasDiagnosticEventKind.JointRemoved);
        scenario.Context.Diagnostics.DrawCommandCount.Should().BeGreaterThan(0);
    }

    [Fact]
    public void ConstraintFilteringAndDisabledDiagnostics_ShouldNotAllocateAfterWarmup()
    {
        using PhysicsScenarioBuilder scenario = CreateConstraintScenario();
        ScenarioBody<LSSphereCollider> first = scenario.CreateSphere(Vector3d.Zero);
        ScenarioBody<LSSphereCollider> second = scenario.CreateSphere(Vector3d.Right * (Fixed64)3);
        scenario.Context.Constraints3D.RegisterJoint(CreateBallSocket(first.Body, second.Body));
        Step(scenario.Context, 8);
        bool linkedFilterResult = false;

        long allocatedBytes = AllocationTestHelper.MeasureSteadyState(
            () =>
            {
                linkedFilterResult = scenario.Context.Constraints3D.ShouldExcludeLinkedCollision(first.Collider, second.Collider);
                scenario.Context.Simulate();
                scenario.Context.LateSimulate();
            },
            warmupIterations: 8,
            stabilizationIterations: 4,
            measurementIterations: 8);

        linkedFilterResult.Should().BeTrue();
        allocatedBytes.Should().Be(0);
    }

    private static PhysicsScenarioBuilder CreateConstraintScenario()
    {
        PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        scenario.Context.Environment.Gravity = Fixed64.Zero;
        scenario.Context.Environment.AirDensity = Fixed64.Zero;
        scenario.Context.Environment.DampingFactor = Fixed64.Zero;
        scenario.Context.Settings.DiscreteSolverIterations = 8;
        return scenario;
    }

    private static JointDefinition3D CreateBallSocket(
        SolidBody first,
        SolidBody second,
        JointCollisionPolicy collisionPolicy = JointCollisionPolicy.SuppressLinked)
    {
        return new JointDefinition3D(
            first,
            second,
            LocalFrame(Vector3d.Zero),
            LocalFrame(Vector3d.Zero),
            JointType3D.BallSocket,
            JointLimit3D.Unrestricted,
            JointMotor3D.Disabled,
            collisionPolicy);
    }

    private static FixedTransform LocalFrame(Vector3d position) =>
        new(position, FixedQuaternion.Identity, Vector3d.One);

    private static void Step(GravitasWorldContext context, int frames)
    {
        for (int i = 0; i < frames; i++)
        {
            context.Simulate();
            context.LateSimulate();
        }
    }

    private static ConstraintState CaptureConstraintState(SolidBody first, SolidBody second, Joint3D joint)
    {
        Vector3d anchorA = first.Position3d + first.Rotation * joint.LocalFrameA.Position;
        Vector3d anchorB = second.Position3d + second.Rotation * joint.LocalFrameB.Position;
        return new ConstraintState(
            first.Position3d,
            second.Position3d,
            first.LinearVelocity,
            second.LinearVelocity,
            (anchorB - anchorA).MagnitudeSquared,
            joint.LastSolvedRowCount,
            joint.AccumulatedImpulseMagnitude);
    }

    private static ConstraintState RunConstraintChain(bool registerForward)
    {
        using PhysicsScenarioBuilder scenario = CreateConstraintScenario();
        ScenarioBody<LSSphereCollider> first = scenario.CreateSphere(Vector3d.Zero, preventAngularForces: true);
        ScenarioBody<LSSphereCollider> second = scenario.CreateSphere(Vector3d.Right * (Fixed64)3, preventAngularForces: true);
        ScenarioBody<LSSphereCollider> third = scenario.CreateSphere(Vector3d.Right * (Fixed64)6, preventAngularForces: true);
        if (registerForward)
        {
            scenario.Context.Constraints3D.RegisterJoint(CreateBallSocket(first.Body, second.Body));
            scenario.Context.Constraints3D.RegisterJoint(CreateBallSocket(second.Body, third.Body));
        }
        else
        {
            scenario.Context.Constraints3D.RegisterJoint(CreateBallSocket(second.Body, third.Body));
            scenario.Context.Constraints3D.RegisterJoint(CreateBallSocket(first.Body, second.Body));
        }

        Step(scenario.Context, 12);
        Joint3D firstJoint = scenario.Context.Constraints3D.GetJoint(1);
        Joint3D secondJoint = scenario.Context.Constraints3D.GetJoint(2);
        return new ConstraintState(
            first.Body.Position3d,
            third.Body.Position3d,
            first.Body.LinearVelocity,
            third.Body.LinearVelocity,
            (third.Body.Position3d - first.Body.Position3d).MagnitudeSquared,
            firstJoint.LastSolvedRowCount + secondJoint.LastSolvedRowCount,
            firstJoint.AccumulatedImpulseMagnitude + secondJoint.AccumulatedImpulseMagnitude);
    }

    private static MotorState RunMotorScenario(Fixed64 strength)
    {
        using PhysicsScenarioBuilder scenario = CreateConstraintScenario();
        ScenarioBody<LSSphereCollider> first = scenario.CreateSphere(Vector3d.Zero, immovable: true);
        ScenarioBody<LSSphereCollider> second = scenario.CreateSphere(
            Vector3d.Right * (Fixed64)2,
            FixedQuaternion.FromAxisAngle(Vector3d.Up, Fixed64.FromFraction(1, 3)));
        Joint3D joint = scenario.Context.Constraints3D.RegisterJoint(new JointDefinition3D(
            first.Body,
            second.Body,
            LocalFrame(Vector3d.Zero),
            LocalFrame(Vector3d.Zero),
            JointType3D.BallSocket,
            JointLimit3D.Unrestricted,
            new JointMotor3D(FixedQuaternion.Identity, strength, Fixed64.Half, (Fixed64)2),
            JointCollisionPolicy.SuppressLinked));
        Fixed64 before = FixedQuaternion.Angle(FixedQuaternion.Identity, second.Body.Rotation);

        Step(scenario.Context, 16);

        return new MotorState(
            before,
            FixedQuaternion.Angle(FixedQuaternion.Identity, second.Body.Rotation),
            second.Body.AngularVelocity,
            joint.AccumulatedImpulseMagnitude);
    }

    private static (Fixed64 CachedImpulse, Vector3d AngularVelocity, Fixed64 LimitError, int RowCount)
        RunPurePiTwist(bool negateQuaternion)
    {
        using PhysicsScenarioBuilder scenario = CreateConstraintScenario();
        scenario.Context.Settings.DiscreteSolverIterations = 1;
        ScenarioBody<LSSphereCollider> first = scenario.CreateSphere(Vector3d.Zero, immovable: true);
        FixedQuaternion rotation = FixedQuaternion.FromAxisAngle(Vector3d.Forward, Fixed64.Pi);
        ScenarioBody<LSSphereCollider> second = scenario.CreateSphere(
            Vector3d.Right * (Fixed64)2,
            negateQuaternion ? -rotation : rotation);
        Joint3D joint = scenario.Context.Constraints3D.RegisterJoint(new JointDefinition3D(
            first.Body,
            second.Body,
            LocalFrame(Vector3d.Zero),
            LocalFrame(Vector3d.Zero),
            JointType3D.ConeTwist,
            JointLimit3D.ConeTwist(Fixed64.Pi, Fixed64.HalfPi),
            JointMotor3D.Disabled,
            JointCollisionPolicy.SuppressLinked));

        Step(scenario.Context, 1);

        return (
            joint.GetCachedImpulse(3),
            second.Body.AngularVelocity,
            joint.LastSolveMetrics.AngularLimitErrorMagnitude,
            joint.LastSolvedRowCount);
    }

    private static RagdollDefinition3D CreateTwoLinkRagdoll(
        ScenarioBody<LSSphereCollider> root,
        ScenarioBody<LSSphereCollider> child)
    {
        return new RagdollDefinition3D(
            new[]
            {
                new RagdollLinkDefinition3D(0, root.Body),
                new RagdollLinkDefinition3D(1, child.Body)
            },
            new[]
            {
                new RagdollJointDefinition3D(0, 1, JointType3D.BallSocket, LocalFrame(Vector3d.Zero), LocalFrame(Vector3d.Zero))
            });
    }

    private readonly record struct ConstraintState(
        Vector3d FirstPosition,
        Vector3d SecondPosition,
        Vector3d FirstVelocity,
        Vector3d SecondVelocity,
        Fixed64 AnchorDistanceSquared,
        int JointSolvedRowCount,
        Fixed64 AccumulatedImpulseMagnitude);

    private readonly record struct MotorState(
        Fixed64 AngularErrorBefore,
        Fixed64 AngularErrorAfter,
        Vector3d AngularVelocity,
        Fixed64 AccumulatedImpulseMagnitude);
}
