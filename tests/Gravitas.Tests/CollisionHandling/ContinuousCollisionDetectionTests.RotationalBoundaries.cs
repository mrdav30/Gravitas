using Chronicler.Hashing;
using FixedMathSharp;
using FluentAssertions;
using Gravitas.Colliders;
using Gravitas.CollisionHandling;
using Gravitas.Materials;
using Gravitas.Tests.Support;
using GridForge.Configuration;
using System;
using Xunit;

namespace Gravitas.Tests.CollisionHandlingTests;

public sealed partial class ContinuousCollisionDetectionTests
{
    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void FullLoop_RotatingConeAgainstMeshGroups_ShouldClampAtTheEarlierFace(bool includeLaterPlane)
    {
        using PhysicsScenarioBuilder scenario = CreateCcdScenario();
        scenario.Context.Environment.DampingFactor = Fixed64.Zero;
        Fixed64 upperY = Fixed64.FromFraction(1,10);
        Vector3d[] vertices = { new((Fixed64)(-2),upperY,(Fixed64)(-2)), new((Fixed64)2,upperY,(Fixed64)(-2)),
            new((Fixed64)2,upperY,(Fixed64)2), new((Fixed64)(-2),upperY,(Fixed64)2),
            new(-2,0,-2), new(2,0,-2), new(2,0,2), new(-2,0,2) };
        int[] triangles = { 0,3,1, 1,3,2, 4,7,5, 5,7,6 };
        if (!includeLaterPlane)
        {
            Array.Resize(ref vertices, 4);
            Array.Resize(ref triangles, 6);
        }
        var mesh = new LSMeshCollider(vertices, triangles, MeshColliderMode.Concave,
            MeshInertiaPolicy.SurfaceApproximation) { Material = PhysicsMaterial.Frictionless };
        scenario.CreateBody(mesh, Vector3d.Zero, FixedQuaternion.Identity, immovable: true);
        ScenarioBody<LSConeCollider> source = scenario.CreateBody(new LSConeCollider(),
            new Vector3d(Fixed64.Zero, Fixed64.FromFraction(13,20), Fixed64.Zero),
            FixedQuaternion.Identity, isKinematic: true);
        source.Collider.Material = PhysicsMaterial.Frictionless;
        source.Body.UseManualGrounding();
        source.Body.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        CollisionPair pair = scenario.CreatePair(mesh, source.Collider);
        Assert.False(CollisionDetection.DoCollisionCheck(pair));
        // Clockwise tilt gives the unit cone's lowest X/Y support
        // -1/2*(cos(theta)+sin(theta)). The upper plane, 0.55 below the
        // center, is hit between 5 and 8 degrees; the lower plane is later.
        // Their aggregate bounds also certify the initially separated interval.
        source.Body.SetRotation(FixedQuaternion.FromAxisAngle(Vector3d.Forward,
            -FixedMath.DegToRad((Fixed64)8)));
        Assert.True(CollisionDetection.DoCollisionCheck(pair));
        Assert.Equal(1, pair.Manifold.GroupCount);
        Assert.All(pair.Manifold, contact => Assert.Equal(Vector3d.Up, contact.Normal));
        source.Body.SetRotation(FixedQuaternion.FromAxisAngle(Vector3d.Forward, -Fixed64.Pi / 4));
        Assert.True(CollisionDetection.DoCollisionCheck(pair));
        Assert.Contains(pair.Manifold, contact => contact.Normal == Vector3d.Up);
        if (includeLaterPlane)
        {
            Assert.True(pair.Manifold.GroupCount >= 2);
            Assert.Contains(pair.Manifold, contact => contact.PointA.Y == Fixed64.Zero);
            Assert.Contains(pair.Manifold, contact => contact.PointA.Y == upperY);
        }
        source.Body.SetRotation(FixedQuaternion.Identity);
        Assert.False(CollisionDetection.DoCollisionCheck(pair));
        Vector3d initialPosition = source.Body.Position3d;
        source.Body.Agent.Transform.LocalRotation = FixedQuaternion.FromAxisAngle(Vector3d.Forward, -Fixed64.Pi / 2);

        scenario.Context.Simulate();
        scenario.Context.LateSimulate();

        Assert.True(source.Body.LastContinuousCollisionToiIterationCount > 0);
        Fixed64 angle = FixedQuaternion.Angle(FixedQuaternion.Identity, source.Body.Rotation);
        Assert.InRange(angle.m_rawValue, ((Fixed64)5).m_rawValue, ((Fixed64)8).m_rawValue);
        Assert.Equal(initialPosition, source.Body.Position3d);
    }

    [Theory]
    [InlineData(false, false)]
    [InlineData(false, true)]
    [InlineData(true, false)]
    [InlineData(true, true)]
    public void RotationalClosingWitness_CompetingGroups_ShouldIgnoreDepthAndFollowMotion(
        bool sourceIsA, bool reverseMotion)
    {
        using PhysicsScenarioBuilder scenario = CreateCcdScenario();
        SolidBody source = sourceIsA
            ? scenario.CreateCuboid(Vector3d.Zero).Body
            : scenario.CreateSphere(Vector3d.Zero).Body;
        SolidBody target = sourceIsA
            ? scenario.CreateSphere(Vector3d.Zero, immovable: true).Body
            : scenario.CreateCuboid(Vector3d.Zero, immovable: true).Body;
        source.ApplyCollisionLinearVelocityDelta(
            (Vector3d.Right - Vector3d.Forward) * (reverseMotion ? -Fixed64.One : Fixed64.One));
        Fixed64 orderSign = sourceIsA ? Fixed64.One : -Fixed64.One;
        var manifold = new ContactManifold();
        var deep = new ManifoldContact(1, Vector3d.Zero, Vector3d.Zero,
            Fixed64.Two, Vector3d.Forward * orderSign);
        var shallow = new ManifoldContact(2, Vector3d.Zero, Vector3d.Zero,
            Fixed64.One, Vector3d.Right * orderSign);
        manifold.AddContact(new ContactGroupKey(0, 0, region: 1), deep);
        manifold.AddContact(new ContactGroupKey(0, 0, region: 2), shallow);

        source.TrySelectRotationalClosingContact(target.Collider, manifold,
            Fixed64.Half, out ManifoldContact selected).Should().BeTrue();

        selected.ContactId.Should().Be(reverseMotion ? deep.ContactId : shallow.ContactId);
        manifold.PrimaryContact.ContactId.Should().Be(deep.ContactId);
        manifold[0].Normal.Should().Be(deep.Normal);
        manifold[1].Normal.Should().Be(shallow.Normal);
    }

    [Fact]
    public void RotationalClosingWitness_TinyClosingSample_ShouldNotHideClosingGroup()
    {
        using PhysicsScenarioBuilder scenario = CreateCcdScenario();
        SolidBody source = scenario.CreateSphere(Vector3d.Zero).Body;
        LSSphereCollider target = scenario.CreateStaticSphere(Vector3d.Zero);
        source.ApplyCollisionLinearVelocityDelta(
            Vector3d.Right * Fixed64.Epsilon + Vector3d.Forward);
        var manifold = new ContactManifold();
        manifold.AddContact(new ContactGroupKey(0, 0, region: 1),
            new ManifoldContact(1, Vector3d.Zero, Vector3d.Zero, Fixed64.Two, Vector3d.Right));
        manifold.AddContact(new ContactGroupKey(0, 0, region: 2),
            new ManifoldContact(2, Vector3d.Zero, Vector3d.Zero, Fixed64.Zero, Vector3d.Forward));

        source.TrySelectRotationalClosingContact(target, manifold,
            Fixed64.Half, out ManifoldContact selected).Should().BeTrue();
        selected.ContactId.Should().Be(2);
        manifold.BeginUpdate(0);
        manifold.AddContact(Vector3d.Zero, Vector3d.Zero, Fixed64.One, Vector3d.Right);
        source.TrySelectRotationalClosingContact(target, manifold,
            Fixed64.Half, out selected).Should().BeFalse();
        selected.Should().Be(default(ManifoldContact));
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void RotationalClosingWitness_AngularPointSpeed_ShouldRemainExactWhenUnrepresentable(
        bool reverseMotion)
    {
        using PhysicsScenarioBuilder scenario = CreateCcdScenario();
        SolidBody source = scenario.CreateSphere(Vector3d.Zero).Body;
        LSSphereCollider target = scenario.CreateStaticSphere(Vector3d.Zero);
        source.ApplyCollisionAngularVelocityDelta(reverseMotion ? Vector3d.Forward : -Vector3d.Forward);
        var manifold = new ContactManifold();
        var farAnchor = new ContactAnchor(Vector3d.Up * Fixed64.MaxValue,
            Vector3d.Up * Fixed64.MaxValue);
        manifold.AddContact(new ContactGroupKey(0, 0), new ManifoldContact(1,
            ContactAnchor.FromWorldPoint(Vector3d.Zero),
            ContactAnchor.FromWorldPoint(Vector3d.Zero), Fixed64.Two, Vector3d.Right));
        manifold.AddContact(new ContactGroupKey(0, 0), new ManifoldContact(2,
            farAnchor, ContactAnchor.FromWorldPoint(Vector3d.Zero), Fixed64.One, Vector3d.Right));

        source.TrySelectRotationalClosingContact(target, manifold, Fixed64.Half,
            out ManifoldContact selected).Should().Be(!reverseMotion);
        selected.ContactId.Should().Be(reverseMotion ? 0UL : 2UL);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void RotationalClosingWitness_ShouldUsePreparedKinematicSourceAndTargetMotion(
        bool sourceIsKinematic)
    {
        using PhysicsScenarioBuilder scenario = CreateCcdScenario();
        SolidBody source = scenario.CreateSphere(Vector3d.Zero,
            isKinematic: sourceIsKinematic).Body;
        SolidBody target = scenario.CreateSphere(Vector3d.Zero, isKinematic: true).Body;
        source.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        target.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        if (sourceIsKinematic)
            source.Agent.Transform.LocalPosition = Vector3d.Right;
        else
            source.ApplyCollisionLinearVelocityDelta(Vector3d.Right);
        target.Agent.Transform.LocalPosition = Vector3d.Right * Fixed64.Two;
        scenario.Context.AdvanceLateSimulateToken();
        scenario.Context.Physics.PrepareContinuousCollisionFrame();
        var manifold = new ContactManifold();
        manifold.AddContact(Vector3d.Zero, Vector3d.Zero, Fixed64.One, Vector3d.Right);

        // Source-only velocity would classify this contact as closing. The
        // faster authored target is separating at the same sampled frame time.
        source.TrySelectRotationalClosingContact(target.Collider, manifold,
            Fixed64.Half, out _).Should().BeFalse();
        manifold.SetContact(Vector3d.Zero, Vector3d.Zero, Fixed64.One, Vector3d.Left);
        source.TrySelectRotationalClosingContact(target.Collider, manifold,
            Fixed64.Half, out ManifoldContact selected).Should().BeTrue();
        selected.Normal.Should().Be(Vector3d.Left);
    }

    [Fact]
    public void RotationalDynamicResponse_WithUnrepresentableParallelLeverComponent_ShouldPreserveResponse3D()
    {
        var first = RunUnrepresentableParallelLeverRotationalResponse3D();
        var second = RunUnrepresentableParallelLeverRotationalResponse3D();

        first.Applied.Should().BeTrue();
        first.SourceLinearVelocity.X.Should().BeLessThan((Fixed64)4);
        first.SourceAngularVelocity.Z.Should().BeGreaterThan(Fixed64.Zero);
        first.TargetLinearVelocity.Should().Be(Vector3d.Zero);
        first.TargetAngularVelocity.Should().Be(Vector3d.Zero);
        first.Should().Be(second);
    }

    [Fact]
    public void RotationalDynamicResponse_WithUnrepresentablePointSpeed_ShouldApplyFinalAngularDelta3D()
    {
        using PhysicsScenarioBuilder scenario = CreateCcdScenario();
        Fixed64 minimum = Fixed64.MinIncrement;
        var sourceCollider = new UnsupportedTestCollider3D
        {
            InertiaTensor = new Fixed3x3(
                minimum, Fixed64.Zero, Fixed64.Zero,
                Fixed64.Zero, minimum, Fixed64.Zero,
                Fixed64.Zero, Fixed64.Zero, minimum)
        };
        ScenarioBody<UnsupportedTestCollider3D> source =
            scenario.CreateBody(
                sourceCollider,
                -Vector3d.Right * Fixed64.Two,
                FixedQuaternion.Identity);
        ScenarioBody<LSSphereCollider> target =
            scenario.CreateSphere(Vector3d.Zero);
        source.Body.FreezeAxes = BodyFreezeAxes3D.Position;
        target.Body.FreezeAxes = BodyFreezeAxes3D.All;
        source.Collider.Material = PhysicsMaterial.Frictionless;
        target.Collider.Material = PhysicsMaterial.Frictionless;
        scenario.Context.AdvanceLateSimulateToken();
        scenario.Context.Physics.PrepareContinuousCollisionFrame();
        source.Body.ApplyCollisionAngularVelocityDelta(-Vector3d.Forward);
        var contact = new ManifoldContact(
            contactId: 1,
            anchorA: new ContactAnchor(
                Vector3d.Up * Fixed64.MaxValue,
                Vector3d.Up * Fixed64.MaxValue),
            anchorB: ContactAnchor.FromWorldPoint(target.Body.WorldCenterOfMass),
            depth: Fixed64.Zero,
            normal: Vector3d.Right);
        bool applied =
            source.Body.TryApplyRotationalContinuousCollisionResponse(
                target.Body,
                contact,
                Fixed64.Zero,
                source.Body.Position3d,
                Vector3d.Zero,
                source.Body.Rotation,
                scenario.Context.DeltaTime * Fixed64.Half,
                scenario.Context.DeltaTime * Fixed64.Half,
                sourceIsKinematic: false);

        applied.Should().BeTrue();
        source.Body.AngularVelocity.Should().Be(Vector3d.Zero);
        target.Body.LinearVelocity.Should().Be(Vector3d.Zero);
        target.Body.AngularVelocity.Should().Be(Vector3d.Zero);
    }

    private static (
        bool Applied,
        Vector3d SourceLinearVelocity,
        Vector3d SourceAngularVelocity,
        Vector3d TargetLinearVelocity,
        Vector3d TargetAngularVelocity,
        ChronicleHash ReplayHash)
        RunUnrepresentableParallelLeverRotationalResponse3D()
    {
        using PhysicsScenarioBuilder scenario = CreateCcdScenario();
        ScenarioBody<LSSphereCollider> source =
            scenario.CreateSphere(-Vector3d.Right * (Fixed64)2);
        ScenarioBody<LSSphereCollider> target =
            scenario.CreateSphere(Vector3d.Zero);
        target.Body.FreezeAxes = BodyFreezeAxes3D.All;
        scenario.Context.AdvanceLateSimulateToken();
        scenario.Context.Physics.PrepareContinuousCollisionFrame();
        source.Body.ApplyCollisionLinearVelocityDelta(
            Vector3d.Right * (Fixed64)4);
        var contact = new ManifoldContact(
            contactId: 1,
            anchorA: new ContactAnchor(
                new Vector3d(
                    Fixed64.MaxValue,
                    Fixed64.One,
                    Fixed64.Zero),
                Vector3d.Right * Fixed64.MinIncrement),
            anchorB: ContactAnchor.FromWorldPoint(Vector3d.Up),
            depth: Fixed64.Zero,
            normal: Vector3d.Right);

        bool applied = source.Body.TryApplyRotationalContinuousCollisionResponse(
            target.Body,
            contact,
            Fixed64.Half,
            source.Body.Position3d,
            Vector3d.Right * Fixed64.Two,
            source.Body.Rotation,
            Fixed64.Zero,
            scenario.Context.DeltaTime,
            sourceIsKinematic: false);

        return (
            applied,
            source.Body.LinearVelocity,
            source.Body.AngularVelocity,
            target.Body.LinearVelocity,
            target.Body.AngularVelocity,
            scenario.Context.ComputeReplayHash(
                GravitasReplayHashMode.AuthoritativeWithSolverCaches));
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void RotationalDynamicResponse_NonClosingPair_ShouldRejectWithoutMutation3D(
        bool tangent)
    {
        using PhysicsScenarioBuilder scenario = CreateCcdScenario();
        scenario.Context.Environment.Gravity = Fixed64.Zero;
        scenario.Context.Environment.DampingFactor = Fixed64.Zero;
        ScenarioBody<LSSphereCollider> source = scenario.CreateSphere(
            -Vector3d.Right * (Fixed64)2);
        ScenarioBody<LSSphereCollider> target = scenario.CreateSphere(Vector3d.Zero);
        source.Body.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        target.Body.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        source.Body.ApplyCollisionLinearVelocityDelta(
            tangent ? Vector3d.Up : Vector3d.Left);
        scenario.Context.AdvanceLateSimulateToken();
        scenario.Context.Physics.PrepareContinuousCollisionFrame();
        var before = scenario.Context.ComputeReplayHash(
            GravitasReplayHashMode.AuthoritativeWithSolverCaches);
        var contact = new ManifoldContact(
            contactId: 1,
            pointA: -Vector3d.Right,
            pointB: -Vector3d.Right,
            depth: Fixed64.Zero,
            normal: Vector3d.Right);

        source.Body.TryApplyRotationalContinuousCollisionResponse(
                target.Body,
                contact,
                Fixed64.Half,
                source.Body.Position3d,
                Vector3d.Zero,
                source.Body.Rotation,
                Fixed64.Zero,
                scenario.Context.DeltaTime,
                sourceIsKinematic: false)
            .Should()
            .BeFalse();

        scenario.Context.ComputeReplayHash(
                GravitasReplayHashMode.AuthoritativeWithSolverCaches)
            .Should()
            .Be(before);
    }

    [Fact]
    public void KinematicRotationalResponse_WithZeroMassFrozenRotationTarget_ShouldRejectZeroEffectiveMass3D()
    {
        using PhysicsScenarioBuilder scenario = CreateCcdScenario();
        ScenarioBody<LSCuboidCollider> blade = CreateKinematicRotationalCcdBlade(scenario);
        ScenarioBody<LSSphereCollider> target = CreateDynamicRotationalTarget3D(scenario);
        blade.Body.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        target.Body.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        target.Body.Mass = Fixed64.Zero;
        target.Body.FreezeAxes = BodyFreezeAxes3D.Rotation;
        target.Body.Sleep();

        blade.Body.Agent.Transform.LocalRotation = RotationalMovingPairQuarterTurn3D;
        scenario.Context.LateSimulate();

        blade.Body.Rotation.Should().NotBe(RotationalMovingPairQuarterTurn3D);
        target.Body.LinearVelocity.Should().Be(Vector3d.Zero);
        target.Body.AngularVelocity.Should().Be(Vector3d.Zero);
    }

    [Fact]
    public void DynamicRotationalResponse_WhenSourceTrajectoryIsFull_ShouldRejectPairAtomically3D()
    {
        using PhysicsScenarioBuilder scenario = CreateCcdScenario();
        scenario.Context.Settings.ContinuousCollisionMaxToiIterations = 1;
        ScenarioBody<LSCuboidCollider> blade = scenario.CreateBody(
            new LSCuboidCollider
            {
                Size = new Vector3d((Fixed64)6, Fixed64.One, Fixed64.FromFraction(1, 5))
            },
            Vector3d.Zero,
            FixedQuaternion.Identity);
        ScenarioBody<LSSphereCollider> target = CreateDynamicRotationalTarget3D(scenario);
        blade.Body.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        target.Body.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        blade.Body.ApplyCollisionAngularVelocityDelta(
            Vector3d.Up * FixedMath.DegToRad((Fixed64)90));
        scenario.Context.AdvanceLateSimulateToken();
        scenario.Context.Physics.PrepareContinuousCollisionFrame();
        blade.Body.ApplyContinuousCollisionHandoff(
                blade.Body.Position3d,
                blade.Body.Rotation,
                blade.Body.LinearVelocity,
                blade.Body.AngularVelocity,
                scenario.Context.DeltaTime * Fixed64.FromFraction(3, 4))
            .Should()
            .BeTrue();
        Vector3d targetPosition = target.Body.Position3d;

        blade.Body.TryConsumeContinuousCollisionHandoff(
                updateSleepState: false,
                updateColliderState: false)
            .Should()
            .BeTrue();

        blade.Body.LastContinuousCollisionToiIterationLimitReached.Should().BeTrue();
        target.Body.Position3d.Should().Be(targetPosition);
        target.Body.LinearVelocity.Should().Be(Vector3d.Zero);
        target.Body.AngularVelocity.Should().Be(Vector3d.Zero);
    }

    [Fact]
    public void KinematicRotationalResponse_WhenTargetTrajectoryIsFull_ShouldLeaveTargetAtomic3D()
    {
        using PhysicsScenarioBuilder scenario = CreateCcdScenario();
        scenario.Context.Settings.ContinuousCollisionMaxToiIterations = 1;
        ScenarioBody<LSCuboidCollider> blade = CreateKinematicRotationalCcdBlade(scenario);
        ScenarioBody<LSSphereCollider> target = CreateDynamicRotationalTarget3D(scenario);
        blade.Body.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        target.Body.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        blade.Body.Agent.Transform.LocalRotation = RotationalMovingPairQuarterTurn3D;
        scenario.Context.AdvanceLateSimulateToken();
        scenario.Context.Physics.PrepareContinuousCollisionFrame();
        target.Body.ApplyContinuousCollisionHandoff(
                target.Body.Position3d,
                target.Body.Rotation,
                Vector3d.Zero,
                Vector3d.Zero,
                scenario.Context.DeltaTime * Fixed64.FromFraction(3, 4))
            .Should()
            .BeTrue();
        Vector3d targetPosition = target.Body.Position3d;

        blade.Body.LateSimulate(updateSleepState: false, updateColliderState: false);

        blade.Body.Rotation.Should().NotBe(RotationalMovingPairQuarterTurn3D);
        target.Body.Position3d.Should().Be(targetPosition);
        target.Body.LinearVelocity.Should().Be(Vector3d.Zero);
        target.Body.AngularVelocity.Should().Be(Vector3d.Zero);
    }

    [Fact]
    public void DynamicRotationalResponse_AtIterationLimit_ShouldStopAfterFirstMovingTarget()
    {
        using PhysicsScenarioBuilder scenario = CreateCcdScenario();
        scenario.Context.Settings.ContinuousCollisionMaxToiIterations = 1;
        ScenarioBody<LSCuboidCollider> blade = scenario.CreateBody(
            new LSCuboidCollider
            {
                Size = new Vector3d((Fixed64)6, Fixed64.One, Fixed64.FromFraction(1, 5))
            },
            Vector3d.Zero,
            FixedQuaternion.Identity);
        ScenarioBody<LSSphereCollider> first = CreateDynamicRotationalTarget3D(
            scenario,
            PhysicsScenarioBuilder.Yaw(30));
        ScenarioBody<LSSphereCollider> second = CreateDynamicRotationalTarget3D(
            scenario,
            PhysicsScenarioBuilder.Yaw(60));
        blade.Body.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        first.Body.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        second.Body.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        first.Body.Sleep();
        second.Body.Sleep();

        blade.Body.ApplyCollisionAngularVelocityDelta(
            Vector3d.Up * FixedMath.DegToRad((Fixed64)90));
        scenario.Context.LateSimulate();

        blade.Body.LastContinuousCollisionToiIterationCount.Should().Be(1);
        blade.Body.LastContinuousCollisionToiIterationLimitReached.Should().BeTrue();
        first.Body.IsSleeping.Should().BeFalse();
        second.Body.IsSleeping.Should().BeTrue();
    }

    [Fact]
    public void KinematicRotationalResponse_PositionFrozenSphereTarget_ShouldRejectZeroMobilityImpulse()
    {
        using PhysicsScenarioBuilder scenario = CreateCcdScenario();
        ScenarioBody<LSCuboidCollider> blade = CreateKinematicRotationalCcdBlade(scenario);
        ScenarioBody<LSSphereCollider> target = CreateDynamicRotationalTarget3D(scenario);
        blade.Body.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        target.Body.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        target.Body.FreezeAxes = BodyFreezeAxes3D.Position;
        target.Body.Sleep();

        blade.Body.Agent.Transform.LocalRotation = RotationalMovingPairQuarterTurn3D;
        scenario.Context.LateSimulate();

        blade.Body.Rotation.Should().NotBe(RotationalMovingPairQuarterTurn3D);
        target.Body.LinearVelocity.Should().Be(Vector3d.Zero);
        target.Body.AngularVelocity.Should().Be(Vector3d.Zero);
    }

    [Fact]
    public void KinematicRotationalResponse_UnrepresentableWorldCenterOfMass_ShouldUseRelativeLeverArm()
    {
        using PhysicsScenarioBuilder scenario = CreateCcdScenario();
        ScenarioBody<LSCuboidCollider> blade = CreateKinematicRotationalCcdBlade(scenario);
        ScenarioBody<LSSphereCollider> target = CreateDynamicRotationalTarget3D(scenario);
        blade.Body.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        target.Body.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        blade.Body.LocalCenterOfMassOffset = Vector3d.Up * Fixed64.MaxValue;
        blade.Body.ResetPosition(Vector3d.Up, FixedQuaternion.Identity);
        target.Body.ResetPosition(target.Body.Position3d + Vector3d.Up, FixedQuaternion.Identity);
        target.Body.Sleep();

        blade.Body.Agent.Transform.LocalRotation = RotationalMovingPairQuarterTurn3D;
        scenario.Context.LateSimulate();

        target.Body.IsSleeping.Should().BeFalse();
        (target.Body.LinearVelocity.MagnitudeSquared
            + target.Body.AngularVelocity.MagnitudeSquared)
            .Should()
            .BeGreaterThan(Fixed64.Zero);
    }

    [Fact]
    public void RotationalCandidateFallback_UnrepresentableRadius_ShouldFindOnlyRotatingBodies()
    {
        using PhysicsScenarioBuilder scenario = CreateCcdScenario();
        ScenarioBody<LSSphereCollider> source = scenario.CreateSphere(
            Vector3d.Zero,
            isKinematic: true);

        source.Body.HasNearbyRotationalContinuousCollisionTarget(
                Vector3d.Zero,
                Vector3d.Zero,
                Fixed64.MaxValue)
            .Should()
            .BeFalse();

        _ = scenario.CreateStaticSphere(Vector3d.Right);
        source.Body.HasNearbyRotationalContinuousCollisionTarget(
                Vector3d.Zero,
                Vector3d.Zero,
                Fixed64.MaxValue)
            .Should()
            .BeFalse();

        ScenarioBody<LSCuboidCollider> rotatingTarget = scenario.CreateCuboid(
            Vector3d.Right * (Fixed64)4);
        rotatingTarget.Body.ApplyCollisionAngularVelocityDelta(Vector3d.Up);

        source.Body.HasNearbyRotationalContinuousCollisionTarget(
                Vector3d.Zero,
                Vector3d.Zero,
                Fixed64.MaxValue)
            .Should()
            .BeTrue();
        source.Body.GatherRotationalContinuousCollisionCandidates(
                Vector3d.Zero,
                Vector3d.Zero,
                Vector3d.Zero,
                Fixed64.MaxValue)
            .Should()
            .Be(2);
    }

    [Fact]
    public void SphereSeparationGap_ShouldSupportSphereAndCuboidPairsOnly()
    {
        using PhysicsScenarioBuilder scenario = CreateCcdScenario();
        LSSphereCollider source = scenario.CreateStaticSphere(Vector3d.Zero);
        LSSphereCollider sphere = scenario.CreateStaticSphere(Vector3d.Right * (Fixed64)4);
        var cuboid = new LSCuboidCollider();
        scenario.InitializeStaticCollider(cuboid, Vector3d.Right * (Fixed64)4);
        var capsule = new LSCapsuleCollider();
        scenario.InitializeStaticCollider(capsule, Vector3d.Right * (Fixed64)4);

        SolidBody.TryGetSphereSeparationGap(source, sphere, out Fixed64 sphereGap)
            .Should()
            .BeTrue();
        sphereGap.Should().BeGreaterThan(Fixed64.Zero);
        SolidBody.TryGetSphereSeparationGap(source, cuboid, out Fixed64 cuboidGap)
            .Should()
            .BeTrue();
        cuboidGap.Should().BeGreaterThan(Fixed64.Zero);
        SolidBody.TryGetSphereSeparationGap(source, capsule, out _)
            .Should()
            .BeFalse();
    }

    [Fact]
    public void SphereCuboidSeparationGap_AtScalarFace_ShouldUseRelativeSurfaceAnchor()
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        context.World.TryAddGrid(
            new GridConfiguration(
                new Vector3d(
                    Fixed64.MaxValue - (Fixed64)4,
                    (Fixed64)(-4),
                    (Fixed64)(-4)),
                new Vector3d(
                    Fixed64.MaxValue,
                    (Fixed64)4,
                    (Fixed64)4)),
            out _).Should().BeTrue();
        Vector3d center = new(
            Fixed64.MaxValue - Fixed64.FromFraction(1, 8),
            Fixed64.Zero,
            Fixed64.Zero);
        var sphere = new LSSphereCollider();
        sphere.InitializeWithNoBody(new TestMatterAgent(
            context,
            new FixedTransform(
                center,
                FixedQuaternion.Identity,
                Vector3d.One)));
        var cuboid = new LSCuboidCollider();
        cuboid.InitializeWithNoBody(new TestMatterAgent(
            context,
            new FixedTransform(
                center,
                FixedQuaternion.Identity,
                Vector3d.One)));

        bool certified = true;
        Action query = () => certified =
            SolidBody.TryGetSphereSeparationGap(
                sphere,
                cuboid,
                out _);

        query.Should().NotThrow();
        certified.Should().BeFalse();
    }

    [Fact]
    public void SphereSeparationGap_UnrepresentableCenterDelta_ShouldRemainUncertified()
    {
        using PhysicsScenarioBuilder scenario = CreateCcdScenario();
        LSSphereCollider source = scenario.CreateStaticSphere(
            new Vector3d(Fixed64.MaxValue, Fixed64.Zero, Fixed64.Zero));
        LSSphereCollider target = scenario.CreateStaticSphere(
            new Vector3d(Fixed64.MinValue, Fixed64.Zero, Fixed64.Zero));

        SolidBody.TryGetSphereSeparationGap(source, target, out _)
            .Should()
            .BeFalse();
    }

    [Fact]
    public void SphereCuboidSeparationGap_UnrepresentableAnchorDelta_ShouldRemainUncertified()
    {
        using PhysicsScenarioBuilder scenario = CreateCcdScenario();
        LSSphereCollider source = scenario.CreateStaticSphere(
            new Vector3d(Fixed64.MaxValue, Fixed64.Zero, Fixed64.Zero));
        var target = new LSCuboidCollider();
        scenario.InitializeStaticCollider(
            target,
            new Vector3d(Fixed64.MinValue, Fixed64.Zero, Fixed64.Zero));

        SolidBody.TryGetSphereSeparationGap(source, target, out _)
            .Should()
            .BeFalse();
    }

    [Fact]
    public void SphereSeparationGap_UnrepresentableDistance_ShouldRemainUncertified()
    {
        using PhysicsScenarioBuilder scenario = CreateCcdScenario();
        LSSphereCollider source = scenario.CreateStaticSphere(Vector3d.Zero);
        LSSphereCollider target = scenario.CreateStaticSphere(
            new Vector3d(Fixed64.MaxValue, Fixed64.MaxValue, Fixed64.Zero));

        SolidBody.TryGetSphereSeparationGap(source, target, out _)
            .Should()
            .BeFalse();
    }

    [Fact]
    public void SphereSeparationGap_UnrepresentableCharacteristicScale_ShouldRemainUncertified()
    {
        using PhysicsScenarioBuilder scenario = CreateCcdScenario();
        LSSphereCollider source = scenario.CreateStaticSphere(Vector3d.Zero);
        LSSphereCollider target = scenario.CreateStaticSphere(
            new Vector3d(Fixed64.MaxValue, Fixed64.Zero, Fixed64.Zero));

        SolidBody.TryGetSphereSeparationGap(source, target, out _)
            .Should()
            .BeFalse();
    }

    [Fact]
    public void SphereCuboidSeparationGap_UnrepresentableProxyScale_ShouldRemainUncertified()
    {
        using PhysicsScenarioBuilder scenario = CreateCcdScenario();
        LSSphereCollider source = scenario.CreateStaticSphere(Vector3d.Zero);
        var target = new LSCuboidCollider();
        scenario.InitializeStaticCollider(
            target,
            new Vector3d((Fixed64)1_500_000_000, Fixed64.Zero, Fixed64.Zero));
        target.Size = new Vector3d(
            Fixed64.One,
            (Fixed64)2_000_000_000,
            (Fixed64)2_000_000_000);
        target.RebuildRuntimeShapeOnly(refreshMassProperties: false);

        SolidBody.TryGetSphereSeparationGap(source, target, out _)
            .Should()
            .BeFalse();
    }

    [Fact]
    public void SphereSeparationGap_UnrepresentableCombinedRadius_ShouldRemainUncertified()
    {
        using PhysicsScenarioBuilder scenario = CreateCcdScenario();
        LSSphereCollider source = scenario.CreateStaticSphere(Vector3d.Zero);
        LSSphereCollider target = scenario.CreateStaticSphere(Vector3d.Zero);
        source.Radius = Fixed64.MaxValue;
        target.Radius = Fixed64.MaxValue;

        SolidBody.TryGetSphereSeparationGap(source, target, out _)
            .Should()
            .BeFalse();
    }
}
