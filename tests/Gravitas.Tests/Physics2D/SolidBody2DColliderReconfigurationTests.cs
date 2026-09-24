using FixedMathSharp;
using FluentAssertions;
using Gravitas.Colliders;
using Gravitas.Constraints;
using Gravitas.Materials;
using Gravitas.Support;
using Gravitas.Tests.Support;
using GridForge.Configuration;
using System;
using Xunit;

namespace Gravitas.Tests.Physics2D;

public sealed partial class SolidBody2DColliderReconfigurationTests
{
    [Theory]
    [InlineData(-1, ColliderReconfigurationStatus.Applied)]
    [InlineData(0, ColliderReconfigurationStatus.Applied)]
    [InlineData(1, ColliderReconfigurationStatus.Blocked)]
    public void RotatedCapsuleAgainstCompound_ShouldUseAuthoredScalarAxes(
        long radiusOffset, ColliderReconfigurationStatus expected)
    {
        using GravitasWorldContext context = CreateContext(withGrid: false);
        Fixed64 sourceRotation = Fixed64.Pi / (Fixed64)6;
        Fixed64 targetRotation = Fixed64.Pi / (Fixed64)7;
        Vector2d sourceAxis = new(-FixedMath.Sin(sourceRotation), FixedMath.Cos(sourceRotation));
        Vector2d targetAxis = new(-FixedMath.Sin(targetRotation), FixedMath.Cos(targetRotation));
        Vector2d position = new Vector2d(-3, 4) + sourceAxis;
        SolidBody2D body = CreateBody(context, new LSCapsuleCollider2D((Fixed64)2, (Fixed64)6), position);
        body.SetRotation(sourceRotation);
        var compound = new LSCompoundCollider2D(new CompoundColliderPart2D(
            ColliderShapeDefinition2D.Capsule((Fixed64)2, (Fixed64)6), -targetAxis, targetRotation));
        CreateObstacle(context, compound, Vector2d.Zero);
        Fixed64 radius = (Fixed64)3 + Fixed64.FromRaw(radiusOffset);
        var before = context.ComputeReplayHash();

        // Exact authored endpoints are (-3,4) and (0,0), distance5. Both
        // capsule axes point away from those nearest endpoints. Renormalizing
        // the cached world axes would silently change this boundary geometry.
        var result = body.TryReconfigureCollider(
            ColliderShapeDefinition2D.Capsule(radius, Fixed64.Two + radius * Fixed64.Two),
            Vector2d.Zero, position, out var blocker, out var failure);

        result.Should().Be(expected);
        failure.Should().BeNull();
        if (expected == ColliderReconfigurationStatus.Blocked)
        {
            blocker.Should().BeSameAs(compound);
            context.ComputeReplayHash().Should().Be(before);
        }
        else
            blocker.Should().BeNull();
    }

    [Theory]
    [InlineData(0, false)]
    [InlineData(1, false)]
    [InlineData(2, false)]
    [InlineData(3, false)]
    [InlineData(0, true)]
    [InlineData(1, true)]
    [InlineData(2, true)]
    [InlineData(3, true)]
    public void StrictClearance_ShouldPreserveTangencyAndSubRawPenetrationForEveryTarget(
        int targetFamily, bool compound)
    {
        foreach (bool capsuleSource in new[] { false, true })
        foreach (int offset in new[] { -1, 0, 1 })
        {
            using GravitasWorldContext context = CreateContext(withGrid: false);
            bool radialTarget = targetFamily < 2;
            Fixed64 radius = radialTarget ? (Fixed64)2 : (Fixed64)5;
            Fixed64 axisLength = capsuleSource ? (Fixed64)2 : Fixed64.Zero;
            Fixed64 raw = Fixed64.FromRaw(offset);
            Vector2d position = new((Fixed64)3 + raw, (Fixed64)4 - raw + axisLength * Fixed64.Half);
            LSCollider2D source = capsuleSource
                ? new LSCapsuleCollider2D(radius - Fixed64.One, axisLength + (radius - Fixed64.One) * Fixed64.Two)
                : new LSCircleCollider2D(radius - Fixed64.One);
            SolidBody2D body = CreateBody(context, source, position);
            Vector2d[] vertices = { new(-2, -2), new(0, -2), new(0, 0), new(-2, 0) };
            var shape = targetFamily switch
            {
                0 => ColliderShapeDefinition2D.Circle((Fixed64)3),
                1 => ColliderShapeDefinition2D.Capsule((Fixed64)3, (Fixed64)8),
                2 => ColliderShapeDefinition2D.AABBox(new Vector2d(2, 2)),
                _ => ColliderShapeDefinition2D.ConvexPolygon(vertices)
            };
            Vector2d targetPosition = targetFamily == 1 ? new Vector2d(0, -1)
                : targetFamily == 2 ? new Vector2d(-1, -1) : Vector2d.Zero;
            LSCollider2D obstacle = compound
                ? new LSCompoundCollider2D(new CompoundColliderPart2D(shape, targetPosition))
                : shape.CreateRuntimeCollider();
            CreateObstacle(context, obstacle, compound ? Vector2d.Zero : targetPosition);
            var replacement = capsuleSource
                ? ColliderShapeDefinition2D.Capsule(radius, axisLength + radius * Fixed64.Two)
                : ColliderShapeDefinition2D.Circle(radius);
            var before = context.ComputeReplayHash();
            int publications = 0;

            var result = body.TryReconfigureCollider(replacement, Vector2d.Zero, position,
                out var blocker, out var failure, () => publications++);

            // Both radial and corner fixtures reduce to (3+e)^2+(4-e)^2 vs 25.
            bool penetrates = offset > 0;
            result.Should().Be(penetrates ? ColliderReconfigurationStatus.Blocked : ColliderReconfigurationStatus.Applied);
            failure.Should().BeNull();
            publications.Should().Be(penetrates ? 0 : 1);
            if (penetrates)
            {
                blocker.Should().BeSameAs(obstacle);
                context.ComputeReplayHash().Should().Be(before);
            }
            else
                blocker.Should().BeNull();
        }
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void SubRawCornerPenetration_ShouldBlockDespiteRoundedZeroContactDepth(bool capsule)
    {
        using GravitasWorldContext context = CreateContext(withGrid: false);
        Fixed64 raw = Fixed64.FromRaw(1);
        Vector2d position = new((Fixed64)3 + raw, (Fixed64)4 - raw);
        LSCollider2D source = capsule
            ? new LSCapsuleCollider2D((Fixed64)4, (Fixed64)8)
            : new LSCircleCollider2D((Fixed64)4);
        SolidBody2D body = CreateBody(context, source, position);
        LSCollider2D obstacle = CreateObstacle(context, new LSAABBoxCollider2D(new Vector2d(2, 2)), new Vector2d(-1, -1));
        var definition = capsule
            ? ColliderShapeDefinition2D.Capsule((Fixed64)5, (Fixed64)10)
            : ColliderShapeDefinition2D.Circle((Fixed64)5);
        var hash = context.ComputeReplayHash();

        var result = body.TryReconfigureCollider(definition, Vector2d.Zero, position, out var blocker, out var failure);

        // (3 + e)^2 + (4 - e)^2 = 25 - 2e + 2e^2 < 25 for one-raw e.
        // Its positive penetration is less than half a raw depth unit.
        result.Should().Be(ColliderReconfigurationStatus.Blocked);
        blocker.Should().BeSameAs(obstacle);
        failure.Should().BeNull();
        context.ComputeReplayHash().Should().Be(hash);
    }

    [Fact]
    public void Reconfiguration_ShouldPublishCircleGeometryAndHostPositionTogether()
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        context.Settings.RuntimeMode = PhysicsRuntimeMode.TwoD;
        var transform = new FixedTransform(new Vector3d(0, 7, 0), FixedQuaternion.Identity, Vector3d.One);
        var collider = new LSCircleCollider2D(Fixed64.Half);
        var body = new SolidBody2D(new TestMatterAgent(context, transform), collider) { Mass = Fixed64.One };
        body.Initialize(Vector2d.Zero);
        int publications = 0;
        Action publisher = () =>
        {
            publications++;
            collider.ScaledRadius.Should().Be(Fixed64.One);
            body.Position.Should().Be(new Vector2d(3, 4));
            transform.WorldPosition.Should().Be(new Vector3d(3, 7, 4));
        };
        ColliderReconfigurationStatus result = body.TryReconfigureCollider(
            ColliderShapeDefinition2D.Circle(Fixed64.One), Vector2d.Zero,
            new Vector2d(3, 4), out LSCollider2D? blocker, out Exception? failure, publisher);

        result.Should().Be(ColliderReconfigurationStatus.Applied);
        blocker.Should().BeNull();
        failure.Should().BeNull();
        publications.Should().Be(1);
        collider.Id.Should().Be(0);
        body.LocalCenterOfMassOffset.Should().Be(Vector2d.Zero);
        body.MomentOfInertia.Should().Be(Fixed64.Half);
    }

    [Fact]
    public void Unchanged_ShouldPublishOnceWithoutChangingReplayOrCommittedVersion()
    {
        using GravitasWorldContext context = CreateContext();
        SolidBody2D body = CreateBody(context, new LSCapsuleCollider2D(Fixed64.Half, (Fixed64)2));
        var hash = context.ComputeReplayHash();
        uint version = body.Collider.RuntimeShapeVersion;
        var expectedFailure = new InvalidOperationException("publisher");
        int publications = 0;

        var result = body.TryReconfigureCollider(ColliderShapeDefinition2D.Capsule(Fixed64.Half, (Fixed64)2),
            Vector2d.Zero, Vector2d.Zero, out var blocker, out var failure, () =>
            {
                publications++;
                throw expectedFailure;
            });

        result.Should().Be(ColliderReconfigurationStatus.Unchanged);
        blocker.Should().BeNull();
        failure.Should().BeSameAs(expectedFailure);
        publications.Should().Be(1);
        context.ComputeReplayHash().Should().Be(hash);
        body.Collider.RuntimeShapeVersion.Should().Be(version);
    }

    [Theory]
    [InlineData(0, ColliderReconfigurationStatus.Applied)]
    [InlineData(1, ColliderReconfigurationStatus.Blocked)]
    [InlineData(-1, ColliderReconfigurationStatus.Applied)]
    public void CapsuleGrowth_ShouldDistinguishTangencyFromOneRawPenetration(int penetrationRaw,
        ColliderReconfigurationStatus expected)
    {
        using GravitasWorldContext context = CreateContext();
        var capsule = new LSCapsuleCollider2D(Fixed64.Half, Fixed64.One);
        SolidBody2D body = CreateBody(context, capsule);
        LSCollider2D floor = CreateObstacle(context, new LSAABBoxCollider2D(new Vector2d(8, 1)),
            new Vector2d(Fixed64.Zero, -Fixed64.Half));
        var hash = context.ComputeReplayHash();
        int publications = 0;

        var result = body.TryReconfigureCollider(ColliderShapeDefinition2D.Capsule(Fixed64.Half, (Fixed64)2),
            Vector2d.Zero, new Vector2d(Fixed64.Zero, Fixed64.One - Fixed64.FromRaw(penetrationRaw)),
            out var blocker, out var failure, () => publications++);

        result.Should().Be(expected);
        failure.Should().BeNull();
        if (expected == ColliderReconfigurationStatus.Blocked)
        {
            blocker.Should().BeSameAs(floor);
            publications.Should().Be(0);
            context.ComputeReplayHash().Should().Be(hash);
            capsule.Height.Should().Be(Fixed64.One);
            body.Agent.Transform.WorldPosition.Should().Be(new Vector3d(0, 7, 0));
        }
        else
        {
            blocker.Should().BeNull();
            publications.Should().Be(1);
            capsule.Height.Should().Be((Fixed64)2);
        }
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void BlockedGrowth_ShouldSeeWholeCapsuleMiddleEvenWithoutGridMembership(bool compound)
    {
        using GravitasWorldContext context = CreateContext(withGrid: false);
        SolidBody2D body = CreateBody(context, new LSCapsuleCollider2D(Fixed64.Half, (Fixed64)6));
        LSCollider2D obstacle = compound
            ? new LSCompoundCollider2D(CompoundColliderPart2D.Circle(Fixed64.Half, Vector2d.Zero))
            : new LSPolygonCollider2D(new[] { new Vector2d(-1, -1), new Vector2d(1, -1),
                new Vector2d(1, 1), new Vector2d(-1, 1) });
        CreateObstacle(context, obstacle, new Vector2d(2, 0));
        obstacle.IsPartitioned.Should().BeFalse();

        var result = body.TryReconfigureCollider(ColliderShapeDefinition2D.Capsule((Fixed64)2, (Fixed64)6),
            Vector2d.Zero, Vector2d.Zero, out var blocker, out _);

        result.Should().Be(ColliderReconfigurationStatus.Blocked);
        blocker.Should().BeSameAs(obstacle);
    }

    [Theory]
    [InlineData(BodyMotionType.Dynamic)]
    [InlineData(BodyMotionType.Kinematic)]
    [InlineData(BodyMotionType.Static)]
    public void BlockerSelection_ShouldIncludeEveryBodyRoleAndUseStableRegistrationOrder(BodyMotionType role)
    {
        using GravitasWorldContext context = CreateContext();
        SolidBody2D body = CreateBody(context, new LSCircleCollider2D(Fixed64.Half));
        SolidBody2D first = CreateBody(context, new LSCircleCollider2D(Fixed64.Half), new Vector2d(2, 0), role);
        CreateBody(context, new LSCircleCollider2D(Fixed64.Half), new Vector2d(-2, 0), role);

        var result = body.TryReconfigureCollider(ColliderShapeDefinition2D.Circle((Fixed64)2), Vector2d.Zero,
            Vector2d.Zero, out var blocker, out _);

        result.Should().Be(ColliderReconfigurationStatus.Blocked);
        blocker.Should().BeSameAs(first.Collider);
    }

    [Theory]
    [InlineData(0)]
    [InlineData(1)]
    [InlineData(2)]
    [InlineData(3)]
    public void Growth_ShouldHonorTriggersSymmetricMasksAndHierarchy(int filter)
    {
        using GravitasWorldContext context = CreateContext();
        SolidBody2D body = CreateBody(context, new LSCircleCollider2D(Fixed64.Half));
        LSCollider2D obstacle = CreateObstacle(context, new LSCircleCollider2D(Fixed64.Half), new Vector2d(2, 0));
        body.Collider.Layer = new PhysicsLayer(2);
        obstacle.Layer = new PhysicsLayer(3);
        if (filter == 0) obstacle.IsTrigger = true;
        if (filter == 1) body.Collider.IgnoredCollisionLayers = PhysicsLayerMask.FromLayer(3);
        if (filter == 2) obstacle.IgnoredCollisionLayers = PhysicsLayerMask.FromLayer(2);
        if (filter == 3) obstacle.SetParent(body.Collider);

        var result = body.TryReconfigureCollider(ColliderShapeDefinition2D.Circle((Fixed64)2), Vector2d.Zero,
            Vector2d.Zero, out var blocker, out _);

        result.Should().Be(ColliderReconfigurationStatus.Applied);
        blocker.Should().BeNull();
        if (filter == 3) obstacle.Parent2D.Should().BeSameAs(body.Collider);
    }

    [Fact]
    public void CapsuleReplacement_ShouldRetainIdentityRotationMotionAndMaterialAndRefreshQueries()
    {
        using GravitasWorldContext context = CreateContext();
        var capsule = new LSCapsuleCollider2D(Fixed64.Half, (Fixed64)2) { Material = PhysicsMaterial.Bouncy };
        SolidBody2D body = CreateBody(context, capsule);
        body.SetRotation(Fixed64.HalfPi);
        body.AddLinearImpulse(Vector2d.Right);
        body.AddAngularImpulse(Fixed64.One);
        var velocity = body.LinearVelocity;
        var angularVelocity = body.AngularVelocity;
        var hostRotation = body.Agent.Transform.WorldRotation;
        int id = capsule.Id;
        uint version = capsule.RuntimeShapeVersion;
        var parent = CreateObstacle(context, new LSCircleCollider2D(Fixed64.Half), new Vector2d(-10, 0));
        capsule.SetParent(parent);

        var result = body.TryReconfigureCollider(ColliderShapeDefinition2D.Capsule(Fixed64.One, (Fixed64)2, PhysicsMaterial.Frictionless),
            Vector2d.Right, new Vector2d(5, 4), out _, out _);

        result.Should().Be(ColliderReconfigurationStatus.Applied);
        capsule.Id.Should().Be(id);
        capsule.Parent2D.Should().BeSameAs(parent);
        capsule.Material.Should().Be(PhysicsMaterial.Bouncy);
        capsule.AxisLength.Should().Be(Fixed64.Zero);
        capsule.Radius.Should().Be(Fixed64.One);
        capsule.Height.Should().Be((Fixed64)2);
        capsule.RuntimeShapeVersion.Should().Be(version + 1);
        body.LinearVelocity.Should().Be(velocity);
        body.AngularVelocity.Should().Be(angularVelocity);
        body.Rotation.Should().Be(Fixed64.HalfPi);
        body.Agent.Transform.WorldRotation.Should().Be(hostRotation);
        body.Agent.Transform.WorldPosition.Y.Should().Be((Fixed64)7);
        body.LocalCenterOfMassOffset.Should().Be(Vector2d.Right);
        context.Query2D.OverlapCircle(new Vector2d(5, 5), Fixed64.Half, out var hit).Should().BeTrue();
        hit.Collider.Should().BeSameAs(capsule);
        context.Query2D.OverlapCircle(Vector2d.Zero, Fixed64.Half, out _).Should().BeFalse();
        capsule.Simulate();
        capsule.RuntimeShapeVersion.Should().Be(version + 1, "publication must not leave a stale dirty snapshot");
    }

    [Fact]
    public void AuthoredButUncommittedShape_ShouldApplyAndRefreshExactState()
    {
        using GravitasWorldContext context = CreateContext();
        var capsule = new LSCapsuleCollider2D(Fixed64.Half, (Fixed64)2);
        SolidBody2D body = CreateBody(context, capsule);
        uint version = capsule.RuntimeShapeVersion;
        capsule.Height = (Fixed64)3;

        var result = body.TryReconfigureCollider(ColliderShapeDefinition2D.Capsule(Fixed64.Half, (Fixed64)3),
            Vector2d.Zero, Vector2d.Zero, out _, out _);

        result.Should().Be(ColliderReconfigurationStatus.Applied);
        capsule.AxisLength.Should().Be((Fixed64)2);
        capsule.RuntimeShapeVersion.Should().Be(version + 1);
    }

    [Theory]
    [InlineData(PhysicsRuntimeMode.ThreeD)]
    [InlineData(PhysicsRuntimeMode.Both)]
    [InlineData(PhysicsRuntimeMode.Mixed)]
    public void NonPureTwoD_ShouldRejectWithoutMutation(PhysicsRuntimeMode mode)
    {
        using GravitasWorldContext context = CreateContext();
        SolidBody2D body = CreateBody(context, new LSCircleCollider2D(Fixed64.Half));
        context.Settings.RuntimeMode = mode;
        var hash = context.ComputeReplayHash();

        Action operation = () => body.TryReconfigureCollider(ColliderShapeDefinition2D.Circle(Fixed64.One),
            Vector2d.Zero, Vector2d.Zero, out _, out _);

        operation.Should().Throw<InvalidOperationException>();
        context.ComputeReplayHash().Should().Be(hash);
    }

    [Theory]
    [InlineData(0)]
    [InlineData(1)]
    [InlineData(2)]
    [InlineData(3)]
    public void InvalidDefinitionOrPose_ShouldRejectBeforeChangingLiveState(int invalid)
    {
        using GravitasWorldContext context = CreateContext();
        SolidBody2D body = CreateBody(context, new LSCircleCollider2D(Fixed64.Half));
        var definition = invalid == 0 ? default : invalid == 1 ? ColliderShapeDefinition2D.Capsule(Fixed64.One, (Fixed64)2)
            : invalid == 2 ? ColliderShapeDefinition2D.AABBox(Vector2d.One) : ColliderShapeDefinition2D.Circle(Fixed64.One);
        var position = invalid == 3 ? new Vector2d(Fixed64.MaxValue, Fixed64.Zero) : Vector2d.Zero;
        var hash = context.ComputeReplayHash();

        Action operation = () => body.TryReconfigureCollider(definition, Vector2d.Right, position, out _, out _);

        operation.Should().Throw<ArgumentException>();
        context.ComputeReplayHash().Should().Be(hash);
        body.Agent.Transform.WorldPosition.Should().Be(new Vector3d(0, 7, 0));
    }

    [Fact]
    public void UnrepresentableParentPose_ShouldRejectWithoutChangingState()
    {
        using GravitasWorldContext context = CreateContext();
        SolidBody2D body = CreateBody(context, new LSCircleCollider2D(Fixed64.Half));
        body.Agent.Transform.SetParentKeepingLocal(new FixedTransform(Vector3d.Zero, FixedQuaternion.Identity,
            new Vector3d(Fixed64.Half, Fixed64.One, Fixed64.One)));
        var hash = context.ComputeReplayHash();

        Action operation = () => body.TryReconfigureCollider(ColliderShapeDefinition2D.Circle(Fixed64.One), Vector2d.Zero,
            new Vector2d(Fixed64.MaxValue, Fixed64.Zero), out _, out _);

        operation.Should().Throw<InvalidOperationException>();
        context.ComputeReplayHash().Should().Be(hash);
    }

    [Theory]
    [InlineData(0)]
    [InlineData(1)]
    [InlineData(2)]
    public void UnsafeLifetime_ShouldRejectBeforeChangingGeometry(int state)
    {
        using GravitasWorldContext context = CreateContext();
        var circle = new LSCircleCollider2D(Fixed64.Half);
        SolidBody2D body = CreateBody(context, circle);
        if (state == 0) body.Deactivate();
        if (state == 1) context.Reset();
        if (state == 2) context.Simulate();

        Action operation = () => body.TryReconfigureCollider(ColliderShapeDefinition2D.Circle(Fixed64.One),
            Vector2d.Zero, Vector2d.Zero, out _, out _);

        operation.Should().Throw<InvalidOperationException>();
        circle.Radius.Should().Be(Fixed64.Half);
    }

    [Fact]
    public void AcceptedGrowth_ShouldRetainLinkedJointButClearSolverAndTrajectoryCaches()
    {
        using GravitasWorldContext context = CreateContext();
        SolidBody2D body = CreateBody(context, new LSCircleCollider2D(Fixed64.Half));
        SolidBody2D linked = CreateBody(context, new LSCircleCollider2D(Fixed64.Half), new Vector2d(1, 0));
        Joint2D joint = context.Constraints2D.RegisterJoint(new JointDefinition2D(body, linked,
            JointFrame2D.Identity, JointFrame2D.Identity, JointType2D.Pin, JointLimit2D.Unrestricted,
            JointMotor2D.Disabled, JointCollisionPolicy.SuppressLinked));
        joint.SetCachedImpulse(0, Fixed64.One);
        body.EnsureContinuousCollisionFramePrepared(context.LateSimulateToken);
        body.ContinuousCollisionTrajectoryCount.Should().BeGreaterThan(0);
        body.Sleep();

        var result = body.TryReconfigureCollider(ColliderShapeDefinition2D.Circle(Fixed64.One), Vector2d.Zero,
            Vector2d.Zero, out var blocker, out _);

        result.Should().Be(ColliderReconfigurationStatus.Applied);
        blocker.Should().BeNull("the overlapping linked body is physically excluded");
        joint.GetCachedImpulse(0).Should().Be(Fixed64.Zero);
        context.Constraints2D.ShouldExcludeLinkedCollision(body.Collider, linked.Collider).Should().BeTrue();
        body.ContinuousCollisionTrajectoryCount.Should().Be(0);
        body.IsSleeping.Should().BeFalse();
    }

    [Theory]
    [InlineData(BodyMotionType.Static, 0)]
    [InlineData(BodyMotionType.Static, 1)]
    [InlineData(BodyMotionType.Kinematic, 1)]
    [InlineData(BodyMotionType.Dynamic, 1)]
    public void AcceptedReplacement_ShouldPreserveRoleAndExplicitMassReference(BodyMotionType role, int mass)
    {
        using GravitasWorldContext context = CreateContext();
        SolidBody2D body = CreateBody(context, new LSCircleCollider2D(Fixed64.Half), role: role);
        body.Mass = (Fixed64)mass;
        body.LocalCenterOfMassOffset = Vector2d.Right;
        body.Agent.Transform.SetParentKeepingLocal(new FixedTransform(new Vector3d(1, 0, 0),
            FixedQuaternion.Identity, Vector3d.One));

        var result = body.TryReconfigureCollider(ColliderShapeDefinition2D.Circle(Fixed64.One),
            new Vector2d(2, 0), new Vector2d(4, 0), out _, out var failure);

        result.Should().Be(ColliderReconfigurationStatus.Applied);
        failure.Should().BeNull();
        body.MotionType.Should().Be(role);
        body.Position.Should().Be(new Vector2d(4, 0));
        body.Agent.Transform.WorldPosition.Should().Be(new Vector3d(4, 7, 0));
        body.LocalCenterOfMassOffset.Should().Be(Vector2d.Right);
        body.MomentOfInertia.Should().Be(mass == 0 ? Fixed64.Zero : Fixed64.FromFraction(3, 2));
    }

    private static GravitasWorldContext CreateContext(bool withGrid = true)
    {
        var context = GravitasWorldContext.CreateOwned();
        context.Settings.RuntimeMode = PhysicsRuntimeMode.TwoD;
        if (withGrid)
            context.World.TryAddGrid(new GridConfiguration(new Vector3d(-16, 0, -16), new Vector3d(16, 0, 16)), out _)
                .Should().BeTrue();
        return context;
    }

    private static SolidBody2D CreateBody(GravitasWorldContext context, LSCollider2D collider,
        Vector2d position = default, BodyMotionType role = BodyMotionType.Dynamic)
    {
        var transform = new FixedTransform(new Vector3d(position.X, (Fixed64)7, position.Y), FixedQuaternion.Identity, Vector3d.One);
        var body = new SolidBody2D(new TestMatterAgent(context, transform), collider) { Mass = Fixed64.One, GravityScale = Fixed64.Zero };
        body.Initialize(position, motionType: role);
        return body;
    }

    private static LSCollider2D CreateObstacle(GravitasWorldContext context, LSCollider2D collider, Vector2d position)
    {
        var transform = new FixedTransform(new Vector3d(position.X, (Fixed64)19, position.Y), FixedQuaternion.Identity, Vector3d.One);
        collider.InitializeWithNoBody(new TestMatterAgent(context, transform));
        return collider;
    }
}
