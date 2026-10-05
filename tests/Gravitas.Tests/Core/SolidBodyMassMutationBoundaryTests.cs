using FixedMathSharp;
using FluentAssertions;
using Gravitas.Colliders;
using Gravitas.CollisionHandling;
using Gravitas.Constraints;
using Gravitas.Tests.Serialization;
using Gravitas.Tests.Support;
using System;
using Xunit;

namespace Gravitas.Tests.Core;

public sealed class SolidBodyMassMutationBoundaryTests
{
    public static TheoryData<GravitasSerializationTransport> Transports => GravitasSerializationTransportCases.All();

    [Fact]
    public void MassChange_DuringOpenFixedStep_ShouldRejectChangesButAllowNoOpsInBothDimensions()
    {
        using GravitasWorldContext context3D = GravitasWorldContext.CreateOwned();
        SolidBody body3D = Create3DBody(context3D);
        using GravitasWorldContext context2D = Physics2DTestWorld.CreateContext();
        SolidBody2D body2D = Create2DBody(context2D);
        context3D.Simulate();
        context2D.Simulate();
        Fixed3x3 inertia3D = body3D.InverseInertiaTensor;
        Fixed64 inertia2D = body2D.InverseMomentOfInertia;

        Action noOp3D = () => body3D.Mass = Fixed64.One;
        Action noOp2D = () => body2D.Mass = Fixed64.One;
        Action change3D = () => body3D.Mass = (Fixed64)2;
        Action change2D = () => body2D.Mass = (Fixed64)2;

        noOp3D.Should().NotThrow();
        noOp2D.Should().NotThrow();
        change3D.Should().Throw<InvalidOperationException>();
        change2D.Should().Throw<InvalidOperationException>();
        body3D.Mass.Should().Be(Fixed64.One);
        body2D.Mass.Should().Be(Fixed64.One);
        body3D.InverseInertiaTensor.Should().Be(inertia3D);
        body2D.InverseMomentOfInertia.Should().Be(inertia2D);

        context3D.LateSimulate();
        context2D.LateSimulate();
        change3D.Should().NotThrow();
        change2D.Should().NotThrow();
        body3D.Mass.Should().Be((Fixed64)2);
        body2D.Mass.Should().Be((Fixed64)2);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void MassChange_FromFixedStepCallback_ShouldRejectChangesAndReleaseGuardInBothDimensions(bool latePhase)
    {
        using GravitasWorldContext context3D = GravitasWorldContext.CreateOwned();
        SolidBody body3D = Create3DBody(context3D);
        using GravitasWorldContext context2D = Physics2DTestWorld.CreateContext();
        SolidBody2D body2D = Create2DBody(context2D);
        bool noOpCompleted3D = false;
        bool noOpCompleted2D = false;
        Action callback3D = () =>
        {
            body3D.Mass = Fixed64.One;
            noOpCompleted3D = true;
            body3D.Mass = (Fixed64)2;
        };
        Action callback2D = () =>
        {
            body2D.Mass = Fixed64.One;
            noOpCompleted2D = true;
            body2D.Mass = (Fixed64)2;
        };
        using IDisposable hook3D = latePhase
            ? context3D.RegisterOnLateSimulate("mass-mutation", 0, callback3D)
            : context3D.RegisterOnSimulate("mass-mutation", 0, callback3D);
        using IDisposable hook2D = latePhase
            ? context2D.RegisterOnLateSimulate("mass-mutation", 0, callback2D)
            : context2D.RegisterOnSimulate("mass-mutation", 0, callback2D);
        Action step3D = latePhase ? context3D.LateSimulate : context3D.Simulate;
        Action step2D = latePhase ? context2D.LateSimulate : context2D.Simulate;

        step3D.Should().Throw<InvalidOperationException>();
        step2D.Should().Throw<InvalidOperationException>();
        noOpCompleted3D.Should().BeTrue();
        noOpCompleted2D.Should().BeTrue();
        body3D.Mass.Should().Be(Fixed64.One);
        body2D.Mass.Should().Be(Fixed64.One);

        hook3D.Dispose();
        hook2D.Dispose();
        body3D.Mass = (Fixed64)2;
        body2D.Mass = (Fixed64)2;
        body3D.Mass.Should().Be((Fixed64)2);
        body2D.Mass.Should().Be((Fixed64)2);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void MassChange_AfterContextReset_ShouldRejectStaleBodiesEvenWithReusedRegistration(bool replaceRegistration)
    {
        using GravitasWorldContext context3D = GravitasWorldContext.CreateOwned();
        SolidBody body3D = Create3DBody(context3D);
        using GravitasWorldContext context2D = Physics2DTestWorld.CreateContext();
        SolidBody2D body2D = Create2DBody(context2D);
        int id3D = body3D.DynamicId;
        int id2D = body2D.DynamicId;
        context3D.Reset();
        context2D.Reset();
        SolidBody? replacement3D = null;
        SolidBody2D? replacement2D = null;
        if (replaceRegistration)
        {
            replacement3D = Create3DBody(context3D);
            replacement2D = Create2DBody(context2D);
            replacement3D.DynamicId.Should().Be(id3D);
            replacement2D.DynamicId.Should().Be(id2D);
            replacement3D.Mass = (Fixed64)3;
            replacement2D.Mass = (Fixed64)3;
        }
        Fixed3x3 inertia3D = body3D.InverseInertiaTensor;
        Fixed64 inertia2D = body2D.InverseMomentOfInertia;
        Action noOp3D = () => body3D.Mass = Fixed64.One;
        Action noOp2D = () => body2D.Mass = Fixed64.One;
        Action change3D = () => body3D.Mass = (Fixed64)2;
        Action change2D = () => body2D.Mass = (Fixed64)2;

        noOp3D.Should().NotThrow();
        noOp2D.Should().NotThrow();
        change3D.Should().Throw<InvalidOperationException>();
        change2D.Should().Throw<InvalidOperationException>();
        body3D.Mass.Should().Be(Fixed64.One);
        body2D.Mass.Should().Be(Fixed64.One);
        body3D.InverseInertiaTensor.Should().Be(inertia3D);
        body2D.InverseMomentOfInertia.Should().Be(inertia2D);
        context3D.Physics.BodyCount.Should().Be(replaceRegistration ? 1 : 0);
        context2D.Physics2D.BodyCount.Should().Be(replaceRegistration ? 1 : 0);
        if (replaceRegistration)
        {
            replacement3D!.Mass.Should().Be((Fixed64)3);
            replacement2D!.Mass.Should().Be((Fixed64)3);
        }
    }

    [Theory]
    [InlineData(false, false)]
    [InlineData(false, true)]
    [InlineData(true, false)]
    [InlineData(true, true)]
    public void SolidBody_MassAssignment_ShouldInvalidateConnectedCachesOnlyForChanges(bool pairOwner, bool noOp)
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        SolidBody first = Create3DBody(context);
        SolidBody second = Create3DBody(context);
        CollisionPair pair = CreateContact(first, second);
        Joint3D joint = CreateJoint(first, second);
        SolidBody changedBody = pairOwner ? first : second;
        changedBody.Sleep();
        pair.StoreWarmStartImpulse(7, Vector3d.Right, Fixed64.One, Fixed64.Half);
        joint.SetCachedImpulse(0, Fixed64.One);
        joint.SetCachedImpulse(1, Fixed64.Half);
        joint.AccumulatedImpulseMagnitude = Fixed64.One;

        changedBody.Mass = noOp ? Fixed64.One : (Fixed64)2;

        pair.TryGetWarmStartImpulse(7, out _).Should().Be(noOp);
        joint.GetCachedImpulse(0).Should().Be(noOp ? Fixed64.One : Fixed64.Zero);
        joint.GetCachedImpulse(1).Should().Be(noOp ? Fixed64.Half : Fixed64.Zero);
        joint.AccumulatedImpulseMagnitude.Should().Be(noOp ? Fixed64.One : Fixed64.Zero);
        changedBody.IsSleeping.Should().Be(noOp);
        first.Collider.TryGetCollisionPair(second.Collider.Id, out CollisionPair? retainedPair).Should().BeTrue();
        retainedPair.Should().BeSameAs(pair);
        context.Constraints3D.TryGetJoint(joint.Id, out Joint3D? retainedJoint).Should().BeTrue();
        retainedJoint.Should().BeSameAs(joint);
        context.Constraints3D.RegisteredJointCount.Should().Be(1);
    }

    [Theory]
    [InlineData(false, false)]
    [InlineData(false, true)]
    [InlineData(true, false)]
    [InlineData(true, true)]
    public void SolidBody2D_MassAssignment_ShouldInvalidateConnectedCachesOnlyForChanges(bool pairOwner, bool noOp)
    {
        using GravitasWorldContext context = Physics2DTestWorld.CreateContext();
        SolidBody2D first = Create2DBody(context);
        SolidBody2D second = Create2DBody(context);
        CollisionPair2D pair = CreateContact(first, second);
        Joint2D joint = CreateJoint(first, second);
        SolidBody2D changedBody = pairOwner ? first : second;
        changedBody.Sleep();
        pair.StoreWarmStartImpulse(7, Fixed64.One, Fixed64.Half);
        joint.SetCachedImpulse(0, Fixed64.One);
        joint.SetCachedImpulse(1, Fixed64.Half);
        joint.AccumulatedImpulseMagnitude = Fixed64.One;

        changedBody.Mass = noOp ? Fixed64.One : (Fixed64)2;

        pair.TryGetWarmStartImpulse(7, out _).Should().Be(noOp);
        joint.GetCachedImpulse(0).Should().Be(noOp ? Fixed64.One : Fixed64.Zero);
        joint.GetCachedImpulse(1).Should().Be(noOp ? Fixed64.Half : Fixed64.Zero);
        joint.AccumulatedImpulseMagnitude.Should().Be(noOp ? Fixed64.One : Fixed64.Zero);
        changedBody.IsSleeping.Should().Be(noOp);
        first.Collider.TryGetCollisionPair(second.Collider.Id, out CollisionPair2D? retainedPair).Should().BeTrue();
        retainedPair.Should().BeSameAs(pair);
        context.Constraints2D.TryGetJoint(joint.Id, out Joint2D? retainedJoint).Should().BeTrue();
        retainedJoint.Should().BeSameAs(joint);
        context.Constraints2D.RegisteredJointCount.Should().Be(1);
    }

    [Theory]
    [MemberData(nameof(Transports))]
    public void SolidBody_PopulateChangedMass_ShouldPreserveRecordedSleepAndClearConnectedCaches(GravitasSerializationTransport transport)
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        SolidBody body = Create3DBody(context);
        SolidBody linkedBody = Create3DBody(context);
        CollisionPair pair = CreateContact(body, linkedBody);
        Joint3D joint = CreateJoint(body, linkedBody);
        body.Mass = (Fixed64)2;
        body.Sleep();
        Fixed3x3 recordedInertia = body.InverseInertiaTensor;
        object payload = GravitasSerializationHarness.Serialize(body, transport);
        body.Mass = Fixed64.One;
        body.IsSleeping.Should().BeFalse();
        pair.StoreWarmStartImpulse(7, Vector3d.Right, Fixed64.One, Fixed64.Half);
        joint.SetCachedImpulse(0, Fixed64.One);

        GravitasSerializationHarness.Populate(body, payload, transport);

        body.Mass.Should().Be((Fixed64)2);
        body.InverseInertiaTensor.Should().Be(recordedInertia);
        body.IsSleeping.Should().BeTrue();
        pair.TryGetWarmStartImpulse(7, out _).Should().BeFalse();
        joint.GetCachedImpulse(0).Should().Be(Fixed64.Zero);
        context.Constraints3D.TryGetJoint(joint.Id, out Joint3D? retainedJoint).Should().BeTrue();
        retainedJoint.Should().BeSameAs(joint);
        body.Collider.TryGetCollisionPair(linkedBody.Collider.Id, out CollisionPair? retainedPair).Should().BeTrue();
        retainedPair.Should().BeSameAs(pair);
    }

    [Theory]
    [MemberData(nameof(Transports))]
    public void SolidBody2D_PopulateChangedMass_ShouldPreserveRecordedSleepAndClearConnectedCaches(GravitasSerializationTransport transport)
    {
        using GravitasWorldContext context = Physics2DTestWorld.CreateContext();
        SolidBody2D body = Create2DBody(context);
        SolidBody2D linkedBody = Create2DBody(context);
        CollisionPair2D pair = CreateContact(body, linkedBody);
        Joint2D joint = CreateJoint(body, linkedBody);
        body.Mass = (Fixed64)2;
        body.Sleep();
        Fixed64 recordedInertia = body.InverseMomentOfInertia;
        object payload = GravitasSerializationHarness.Serialize(body, transport);
        body.Mass = Fixed64.One;
        body.IsSleeping.Should().BeFalse();
        pair.StoreWarmStartImpulse(7, Fixed64.One, Fixed64.Half);
        joint.SetCachedImpulse(0, Fixed64.One);

        GravitasSerializationHarness.Populate(body, payload, transport);

        body.Mass.Should().Be((Fixed64)2);
        body.InverseMomentOfInertia.Should().Be(recordedInertia);
        body.IsSleeping.Should().BeTrue();
        pair.TryGetWarmStartImpulse(7, out _).Should().BeFalse();
        joint.GetCachedImpulse(0).Should().Be(Fixed64.Zero);
        context.Constraints2D.TryGetJoint(joint.Id, out Joint2D? retainedJoint).Should().BeTrue();
        retainedJoint.Should().BeSameAs(joint);
        body.Collider.TryGetCollisionPair(linkedBody.Collider.Id, out CollisionPair2D? retainedPair).Should().BeTrue();
        retainedPair.Should().BeSameAs(pair);
    }

    private static SolidBody Create3DBody(GravitasWorldContext context)
    {
        var body = new SolidBody(
            new TestMatterAgent(context, new FixedTransform(Vector3d.Zero, FixedQuaternion.Identity, Vector3d.One)),
            new LSSphereCollider())
        {
            Mass = Fixed64.One
        };
        body.Initialize(Vector3d.Zero, FixedQuaternion.Identity);
        body.UseManualGrounding();
        return body;
    }

    private static SolidBody2D Create2DBody(GravitasWorldContext context)
    {
        var body = new SolidBody2D(
            new TestMatterAgent(context, new FixedTransform(Vector3d.Zero, FixedQuaternion.Identity, Vector3d.One)),
            new LSCircleCollider2D(Fixed64.One))
        {
            Mass = Fixed64.One
        };
        body.Initialize(Vector2d.Zero);
        body.UseManualGrounding();
        return body;
    }

    private static CollisionPair CreateContact(SolidBody first, SolidBody second)
    {
        var pair = new CollisionPair(first.Collider, second.Collider);
        first.Collider.TryAddCollisionPair(second.Collider.Id, pair).Should().BeTrue();
        second.Collider.TryAddCollisionPairHolder(first.Collider.Id).Should().BeTrue();
        return pair;
    }

    private static CollisionPair2D CreateContact(SolidBody2D first, SolidBody2D second)
    {
        var pair = new CollisionPair2D(first.Collider, second.Collider);
        first.Collider.TryAddCollisionPair(second.Collider.Id, pair).Should().BeTrue();
        second.Collider.TryAddCollisionPairHolder(first.Collider.Id).Should().BeTrue();
        return pair;
    }

    private static Joint3D CreateJoint(SolidBody first, SolidBody second) =>
        first.Context.Constraints3D.RegisterJoint(new JointDefinition3D(
            first, second,
            new FixedTransform(Vector3d.Zero, FixedQuaternion.Identity, Vector3d.One),
            new FixedTransform(Vector3d.Zero, FixedQuaternion.Identity, Vector3d.One),
            JointType3D.BallSocket, JointLimit3D.Unrestricted, JointMotor3D.Disabled, JointCollisionPolicy.Collide));

    private static Joint2D CreateJoint(SolidBody2D first, SolidBody2D second) =>
        first.Context.Constraints2D.RegisterJoint(new JointDefinition2D(
            first, second, JointFrame2D.Identity, JointFrame2D.Identity,
            JointType2D.Pin, JointLimit2D.Unrestricted, JointMotor2D.Disabled, JointCollisionPolicy.Collide));
}
