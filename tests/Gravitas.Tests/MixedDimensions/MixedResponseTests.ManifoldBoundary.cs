using FixedMathSharp;
using FluentAssertions;
using Gravitas.Colliders;
using Gravitas.CollisionHandling;
using Gravitas.Materials;
using Gravitas.Tests.Support;
using Xunit;

namespace Gravitas.Tests.MixedDimensions;

/// <content>Checks retained response rows at the physical velocity and mass domain boundaries.</content>
public sealed partial class MixedResponseTests
{
    [Fact]
    public void MixedRoughCube_WithUnrepresentableSpatialSpinIncrease_ShouldRetainOnlyTheNormalSolve()
    {
        using GravitasWorldContext context = CreateMixedContext(frameRate: 64);
        context.Environment.MaxSpeed = Fixed64.MaxValue;
        context.Settings.DiscreteSolverIterations = 1;
        var rough = new PhysicsMaterial(Fixed64.One, Fixed64.One, Fixed64.Zero);
        LSMeshCollider mesh = MeshTestFixtures.CreateConvexCube(MeshColliderMode.Concave);
        mesh.Material = rough;
        var rotor = new SolidBody(new TestMatterAgent(context,
            new FixedTransform(Vector3d.Zero, FixedQuaternion.Identity, Vector3d.One)), mesh) { Mass = Fixed64.One };
        rotor.Initialize(Vector3d.Zero, FixedQuaternion.Identity);
        rotor.FreezeAxes = BodyFreezeAxes3D.RotationX | BodyFreezeAxes3D.RotationZ;
        rotor.ApplyCollisionLinearVelocityDelta(Vector3d.Right);
        rotor.ApplyCollisionAngularVelocityDelta(Vector3d.Up * Fixed64.MinValue);
        rotor.AngularVelocity.Should().Be(Vector3d.Up * Fixed64.MinValue);
        Vector2d position = Vector2d.Right * Fixed64.FromFraction(9, 10);
        var circle = new LSCircleCollider2D(Fixed64.Half)
        { Material = rough, MixedHalfThicknessOverride = Fixed64.Quarter };
        var belt = new SolidBody2D(new TestMatterAgent(context,
            new FixedTransform(position.ToVector3d(Fixed64.Zero), FixedQuaternion.Identity, Vector3d.One)), circle)
        { Mass = Fixed64.One };
        belt.Initialize(position, motionType: BodyMotionType.Kinematic);
        CollisionPairMixed pair = BuildMixedManifoldPair(mesh, circle);
        pair.Manifold.Count.Should().Be(2);
        foreach (ManifoldContact row in pair.Manifold)
        {
            row.Normal.Should().Be(Vector3d.Right);
            row.AnchorA.TryGetWorldPoint(out Vector3d point).Should().BeTrue();
            point.X.Should().Be(Fixed64.Half);
            point.Z.Should().Be(Fixed64.Zero);
        }
        belt.Agent.Transform.LocalPosition = position.ToVector3d(Fixed64.Zero)
            + Vector3d.Forward * (Fixed64.MaxValue * context.DeltaTime);
        context.AdvanceLateSimulateToken();
        belt.EnsureContinuousCollisionFramePrepared(context.LateSimulateToken);

        CollisionResponseMixed.ResolveManifold(pair);

        // At X=1/2, Z=0, normal transfer has no Y torque. The faster belt
        // would increase +Z surface speed and reinforce minimum spatial yaw;
        // rejecting that angular sum must keep the accepted normal stop.
        rotor.LinearVelocity.Should().Be(Vector3d.Zero);
        rotor.AngularVelocity.Should().Be(Vector3d.Up * Fixed64.MinValue);
        belt.LinearVelocity.Should().Be(Vector2d.Zero);
        pair.SolveImpulses.Count.Should().Be(2);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void MixedRoughMesh_WithUnrepresentableSpinIncrease_ShouldRetainOnlyTheNormalSolve(bool heavy)
    {
        using GravitasWorldContext context = CreateMixedContext(frameRate: 64);
        context.Environment.MaxSpeed = Fixed64.MaxValue;
        context.Settings.DiscreteSolverIterations = 1;
        var rough = new PhysicsMaterial(Fixed64.One, Fixed64.One, Fixed64.Zero);
        Fixed64 x = Fixed64.FromFraction(2, 5);
        var mesh = new LSMeshCollider(new[]
        {
            new Vector3d(x, (Fixed64)(-2), (Fixed64)(-2)), new Vector3d(x, (Fixed64)2, (Fixed64)(-2)),
            new Vector3d(x, (Fixed64)(-2), (Fixed64)2), new Vector3d(x, (Fixed64)2, (Fixed64)2)
        }, new[] { 0, 2, 1, 1, 2, 3 }, MeshColliderMode.Concave, MeshInertiaPolicy.SurfaceApproximation) { Material = rough };
        var mover = new SolidBody(new TestMatterAgent(context,
            new FixedTransform(Vector3d.Zero, FixedQuaternion.Identity, Vector3d.One)), mesh) { Mass = Fixed64.One };
        mover.Initialize(Vector3d.Zero, FixedQuaternion.Identity, BodyMotionType.Kinematic);
        SolidBody2D circle = CreateCircularNormalBody(context, zeroCore: false, Vector2d.Zero);
        circle.Collider.Material = rough;
        if (heavy) circle.Mass = Fixed64.MaxValue;
        circle.ApplyCollisionAngularVelocityDelta(Fixed64.MaxValue);
        CollisionPairMixed pair = BuildMixedManifoldPair(mesh, circle.Collider);
        pair.Manifold.Count.Should().Be(2);
        Fixed64 normalSpeed = heavy ? Fixed64.Two : Fixed64.One;
        mover.Agent.Transform.LocalPosition = Vector3d.Left * (normalSpeed * context.DeltaTime)
            + Vector3d.Forward * (Fixed64.MaxValue * context.DeltaTime);
        context.AdvanceLateSimulateToken();
        mover.EnsureContinuousCollisionFramePrepared(context.LateSimulateToken);

        CollisionResponseMixed.ResolveManifold(pair);

        // Circular normal forces have no yaw lever and can transfer the normal
        // speed independently. The rough surface would further increase MaxValue
        // yaw, so friction must reject its complete delta without undoing normal.
        // The heavy case also retains a normal/tangent budget wider than Fixed64.
        circle.LinearVelocity.Should().Be(Vector2d.Left * normalSpeed);
        circle.AngularVelocity.Should().Be(Fixed64.MaxValue);
        if (heavy) pair.ExactSolveImpulses.Should().Contain(state => state != null && state.HasNormal);
    }

    [Fact]
    public void MixedNormalRows_WithUnrepresentableClosingSpeed_ShouldShareTheFiniteVelocityChange()
    {
        using GravitasWorldContext context = CreateMixedContext();
        context.Environment.MaxSpeed = Fixed64.MaxValue;
        ScenarioBody<LSSphereCollider> source = CreateSphere3D(context, Vector3d.Zero, preventAngularForces: true);
        var collider = new LSCompoundCollider2D(
            CompoundColliderPart2D.Circle(Fixed64.Half, Vector2d.Right * Fixed64.FromFraction(3, 4), PhysicsMaterial.Frictionless),
            CompoundColliderPart2D.Circle(Fixed64.Half, Vector2d.Forward * Fixed64.FromFraction(3, 4), PhysicsMaterial.Frictionless));
        var target = new SolidBody2D(new TestMatterAgent(context,
            new FixedTransform(Vector3d.Zero, FixedQuaternion.Identity, Vector3d.One)), collider) { Mass = Fixed64.One };
        target.Initialize(Vector2d.Zero);
        target.FreezeAxes = BodyFreezeAxes2D.Rotation;
        source.Collider.Material = PhysicsMaterial.Frictionless;
        source.Body.ApplyCollisionLinearVelocityDelta(Vector3d.Right * Fixed64.MaxValue);
        target.ApplyCollisionLinearVelocityDelta(Vector2d.Left * Fixed64.MaxValue);
        CollisionPairMixed pair = BuildMixedManifoldPair(source.Collider, collider);
        pair.Manifold.GroupCount.Should().Be(2);

        CollisionResponseMixed.ResolveManifold(pair);

        // The closing speed is 2*MaxValue, but equal masses each change by
        // MaxValue and stop. Physics must succeed when a diagnostic scalar cannot narrow.
        source.Body.LinearVelocity.Should().Be(Vector3d.Zero);
        target.LinearVelocity.Should().Be(Vector2d.Zero);
        source.Body.AngularVelocity.Should().Be(Vector3d.Zero);
        target.AngularVelocity.Should().Be(Fixed64.Zero);
    }

    [Fact]
    public void MixedManifold_SeparatedPair_ShouldLeaveThePreviewAndBodiesUnchanged()
    {
        using GravitasWorldContext context = CreateMixedContext();
        ScenarioBody<LSSphereCollider> first = CreateSphere3D(context, Vector3d.Left * (Fixed64)3);
        SolidBody2D second = CreateCircle2D(context, Vector2d.Zero);
        var pair = new CollisionPairMixed(first.Collider, second.Collider);
        CollisionDetectionMixed.TryBuildManifold(first.Collider, second.Collider, pair.Manifold).Should().BeFalse();
        var impact = new ContactResponseSnapshot(first.Body.Position3d, Vector3d.Zero,
            first.Body.GetCenterOfMassAnchor(), ContactAnchor.FromWorldPoint(Vector3d.Zero),
            Vector3d.Right, Vector3d.Zero, Vector3d.Zero, Vector3d.Zero);
        ContactWarmStartCache cache = default;
        ExactContactImpulseState?[]? exact = null;

        CollisionResponseMixed.TrySolveManifoldVelocity(first.Collider, second.Collider, pair.Manifold,
            impact, ref cache, ref exact, out ContactResponseSnapshot solved).Should().BeFalse();
        CollisionResponseMixed.ResolveManifold(pair, 0, 1, correctPosition: true);

        solved.LinearA.Should().Be(Vector3d.Right);
        solved.LinearB.Should().Be(Vector3d.Zero);
        first.Body.Position3d.Should().Be(Vector3d.Left * (Fixed64)3);
        second.Position.Should().Be(Vector2d.Zero);
        first.Body.LinearVelocity.Should().Be(Vector3d.Zero);
        second.LinearVelocity.Should().Be(Vector2d.Zero);
    }

    [Fact]
    public void BodylessCircularNormalAnchor_ShouldRetainItsActualSurfaceFrame()
    {
        using GravitasWorldContext context = CreateMixedContext();
        LSCollider2D circle = CreateBodylessCircle2D(context, Vector2d.Zero);
        ContactAnchor surface = MixedEmbedded2DGeometry.GetSupportAnchor(circle, Vector3d.Right);

        ContactAnchor normal = ExactContactLever2D.GetDetectedNormalAnchor(circle, 0,
            ContactAnchor.FromWorldPoint(Vector3d.Zero), surface);

        normal.Should().Be(surface);
    }

    [Fact]
    public void CompoundNormalAnchor_WithoutLeafProvenance_ShouldRetainItsActualSurfaceFrame()
    {
        using GravitasWorldContext context = CreateMixedContext();
        var collider = new LSCompoundCollider2D(
            CompoundColliderPart2D.Circle(Fixed64.Quarter, Vector2d.Right * Fixed64.Half),
            CompoundColliderPart2D.Circle(Fixed64.Quarter, Vector2d.Left * Fixed64.Half));
        var body = new SolidBody2D(new TestMatterAgent(context,
            new FixedTransform(Vector3d.Zero, FixedQuaternion.Identity, Vector3d.One)), collider)
        { Mass = Fixed64.One };
        body.Initialize(Vector2d.Zero);
        ContactAnchor surface = MixedEmbedded2DGeometry.GetSupportAnchor(collider.GetPartCollider(0), Vector3d.Right);

        // Namespace zero identifies the owner, not a certified circular leaf.
        // Never infer a leaf's structural normal lever from missing provenance.
        ContactAnchor normal = ExactContactLever2D.GetDetectedNormalAnchor(collider, 0,
            ContactAnchor.FromWorldPoint(Vector3d.Zero), surface);

        normal.Should().Be(surface);
        normal.TryGetWorldPoint(out Vector3d point).Should().BeTrue();
        point.Should().Be(Vector3d.Right * Fixed64.FromFraction(3, 4));
    }

    [Fact]
    public void MixedCompactNormal_WithBodylessSpatialParticipant_ShouldOnlyStopPlanarClosingMotion()
    {
        using GravitasWorldContext context = CreateMixedContext();
        SolidBody2D body = CreateCircle2D(context, Vector2d.Zero);

        ContactNormalImpulseMixed.TryCalculateVelocityDeltas(null, Vector3d.Zero, Vector3d.Zero,
            Vector3d.Zero, body, Vector2d.Left, Fixed64.Zero, Vector2d.Left * Fixed64.Half,
            Vector3d.Right, Fixed64.Zero, Fixed64.Zero, out ContactNormalVelocityDeltaResultMixed result)
            .Should().BeTrue();

        result.LinearVelocityDelta3D.Should().Be(Vector3d.Zero);
        result.AngularVelocityDelta3D.Should().Be(Vector3d.Zero);
        result.LinearVelocityDelta2D.Should().Be(Vector2d.Right);
        result.AngularVelocityDelta2D.Should().Be(Fixed64.Zero);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void MixedNormalPreview_WhenKinematicPointMotionExceedsTheLinearDomain_ShouldReject(bool planarRecipient)
    {
        using GravitasWorldContext context = CreateMixedContext();
        Vector3d positionA = planarRecipient ? Vector3d.Zero : new Vector3d(Fixed64.FromFraction(3, 4), Fixed64.Zero, Fixed64.Half);
        Vector2d positionB = planarRecipient ? new Vector2d(Fixed64.FromFraction(3, 4), Fixed64.Half) : Vector2d.Zero;
        LSCollider shapeA = planarRecipient ? new LSCuboidCollider { Size = Vector3d.One } : new LSSphereCollider();
        LSCollider2D shapeB = planarRecipient ? new LSCircleCollider2D(Fixed64.Half) : new LSAABBoxCollider2D(Vector2d.One);
        var bodyA = new SolidBody(new TestMatterAgent(context,
            new FixedTransform(positionA, FixedQuaternion.Identity, Vector3d.One)), shapeA) { Mass = Fixed64.One };
        bodyA.Initialize(positionA, FixedQuaternion.Identity,
            planarRecipient ? BodyMotionType.Kinematic : BodyMotionType.Dynamic);
        var bodyB = new SolidBody2D(new TestMatterAgent(context,
            new FixedTransform(positionB.ToVector3d(Fixed64.Zero), FixedQuaternion.Identity, Vector3d.One)), shapeB) { Mass = Fixed64.One };
        bodyB.Initialize(positionB, motionType: planarRecipient ? BodyMotionType.Dynamic : BodyMotionType.Kinematic);
        shapeA.Material = shapeB.Material = PhysicsMaterial.Frictionless;
        bodyA.FreezeAxes = planarRecipient ? BodyFreezeAxes3D.None : BodyFreezeAxes3D.Rotation;
        bodyB.FreezeAxes = planarRecipient ? BodyFreezeAxes2D.Rotation : BodyFreezeAxes2D.None;
        var manifold = new ContactManifold();
        CollisionDetectionMixed.TryBuildManifold(shapeA, shapeB, manifold).Should().BeTrue();
        var impact = new ContactResponseSnapshot(positionA, positionB.ToVector3d(Fixed64.Zero),
            bodyA.GetCenterOfMassAnchor(), new ContactAnchor(positionB.ToVector3d(Fixed64.Zero), Vector3d.Zero),
            Vector3d.Right * Fixed64.MaxValue, planarRecipient ? Vector3d.Up * Fixed64.MaxValue : Vector3d.Zero,
            Vector3d.Right * Fixed64.MaxValue, planarRecipient ? Vector3d.Zero : Vector3d.Up * Fixed64.MaxValue,
            planarRecipient ? Fixed64.Zero : Fixed64.MinValue);
        ContactWarmStartCache cache = default;
        ExactContactImpulseState?[]? exact = null;

        CollisionResponseMixed.TrySolveManifoldVelocity(shapeA, shapeB, manifold, impact,
            ref cache, ref exact, out _).Should().BeFalse();

        // The kinematic box face at Z=1/2 has linear plus rotational speed
        // exceeding MaxValue. Its impulse delta fits, but adding it to the
        // recipient's already maximum velocity would exceed the final domain.
        bodyA.LinearVelocity.Should().Be(Vector3d.Zero);
        bodyB.LinearVelocity.Should().Be(Vector2d.Zero);
    }

    [Fact]
    public void MixedNormalPreview_WhenThePlanarAngularSumExceedsTheDomain_ShouldReject()
    {
        using GravitasWorldContext context = CreateMixedContext();
        Vector3d wallPosition = new(Fixed64.Half, Fixed64.Zero, Fixed64.FromFraction(2, 5));
        var wall = new LSCuboidCollider { Size = new Vector3d(Fixed64.Half, Fixed64.Two, Fixed64.FromFraction(1, 5)) };
        var bodyA = new SolidBody(new TestMatterAgent(context,
            new FixedTransform(wallPosition, FixedQuaternion.Identity, Vector3d.One)), wall) { Mass = Fixed64.One };
        bodyA.Initialize(wallPosition, FixedQuaternion.Identity, BodyMotionType.Kinematic);
        var box = new LSAABBoxCollider2D(Vector2d.One);
        var bodyB = new SolidBody2D(new TestMatterAgent(context,
            new FixedTransform(Vector3d.Zero, FixedQuaternion.Identity, Vector3d.One)), box) { Mass = Fixed64.One };
        bodyB.Initialize(Vector2d.Zero);
        wall.Material = box.Material = PhysicsMaterial.Frictionless;
        var manifold = new ContactManifold();
        CollisionDetectionMixed.TryBuildManifold(wall, box, manifold).Should().BeTrue();
        var impact = new ContactResponseSnapshot(wallPosition, Vector3d.Zero,
            bodyA.GetCenterOfMassAnchor(), ContactAnchor.FromWorldPoint(Vector3d.Zero),
            Vector3d.Backward * Fixed64.MaxValue, Vector3d.Zero, Vector3d.Zero,
            Vector3d.Up * Fixed64.MaxValue, Fixed64.MinValue);
        ContactWarmStartCache cache = default;
        ExactContactImpulseState?[]? exact = null;

        CollisionResponseMixed.TrySolveManifoldVelocity(wall, box, manifold, impact,
            ref cache, ref exact, out _).Should().BeFalse();

        // All contact levers have positive X. The incoming wall moving -Z
        // reinforces an already minimum planar yaw; publication must reject it.
        bodyB.AngularVelocity.Should().Be(Fixed64.Zero);
        bodyB.LinearVelocity.Should().Be(Vector2d.Zero);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void MixedCompactNormal_WithUnrepresentableAngularEffectiveMass_ShouldReject(bool planar)
    {
        using GravitasWorldContext context = CreateMixedContext();
        SolidBody? body3D = null;
        SolidBody2D? body2D = null;
        if (planar)
        {
            body2D = new SolidBody2D(new TestMatterAgent(context,
                new FixedTransform(Vector3d.Zero, FixedQuaternion.Identity, Vector3d.One)),
                new LSAABBoxCollider2D(new Vector2d((Fixed64)4, Fixed64.One))) { Mass = Fixed64.FromRaw(4) };
            body2D.Initialize(Vector2d.Zero);
        }
        else
        {
            body3D = new SolidBody(new TestMatterAgent(context,
                new FixedTransform(Vector3d.Zero, FixedQuaternion.Identity, Vector3d.One)),
                new LSCuboidCollider { Size = new Vector3d((Fixed64)4, Fixed64.One, Fixed64.One) }) { Mass = Fixed64.FromRaw(4) };
            body3D.Initialize(Vector3d.Zero, FixedQuaternion.Identity);
        }
        // The normal is the genuine +Z face direction at its X=2 corner.
        // A four-raw mass gives I=6 raw and an angular effective mass
        // 4/I greater than MaxValue. Compact arithmetic must reject it cleanly.
        ContactNormalImpulseMixed.TryCalculateVelocityDeltas(body3D,
            planar ? Vector3d.Zero : Vector3d.Forward, Vector3d.Zero,
            new Vector3d(Fixed64.Two, Fixed64.Zero, Fixed64.Half), body2D,
            planar ? Vector2d.Backward : Vector2d.Zero, Fixed64.Zero,
            new Vector2d(Fixed64.Two, Fixed64.Half), Vector3d.Forward,
            Fixed64.Zero, Fixed64.Zero, out _).Should().BeFalse();
    }

    [Fact]
    public void MixedCompactNormal_AtARealBoxCorner_ShouldRetainSpatialAngularResponse()
    {
        using GravitasWorldContext context = CreateMixedContext();
        var body = new SolidBody(new TestMatterAgent(context,
            new FixedTransform(Vector3d.Zero, FixedQuaternion.Identity, Vector3d.One)),
            new LSCuboidCollider { Size = Vector3d.One }) { Mass = Fixed64.One };
        body.Initialize(Vector3d.Zero, FixedQuaternion.Identity);

        ContactNormalImpulseMixed.TryCalculateVelocityDeltas(body, Vector3d.Forward,
            Vector3d.Zero, new Vector3d(Fixed64.Half, Fixed64.Zero, Fixed64.Half),
            null, Vector2d.Zero, Fixed64.Zero, Vector2d.Zero, Vector3d.Forward,
            Fixed64.Zero, Fixed64.Zero, out ContactNormalVelocityDeltaResultMixed result).Should().BeTrue();

        // I_y=1/6, lever=1/2, so J=1/(1+6/4)=2/5 and Δω_y=6J/2=6/5.
        (result.LinearVelocityDelta3D.Z + Fixed64.FromFraction(2, 5)).Abs()
            .Should().BeLessThanOrEqualTo(Fixed64.FromRaw(4));
        (result.AngularVelocityDelta3D.Y - Fixed64.FromFraction(6, 5)).Abs()
            .Should().BeLessThanOrEqualTo(Fixed64.FromRaw(8));
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void MixedCompoundTriggerManifold_ShouldKeepEveryBodyVelocityUntouched(bool spatialTrigger)
    {
        using GravitasWorldContext context = CreateMixedContext();
        var sphere = new LSSphereCollider { IsTrigger = spatialTrigger };
        var agent3D = new TestMatterAgent(context,
            new FixedTransform(Vector3d.Zero, FixedQuaternion.Identity, Vector3d.One));
        SolidBody? body3D = null;
        if (spatialTrigger) sphere.InitializeWithNoBody(agent3D);
        else
        {
            body3D = new SolidBody(agent3D, sphere) { Mass = Fixed64.One };
            body3D.Initialize(Vector3d.Zero, FixedQuaternion.Identity);
            body3D.ApplyCollisionLinearVelocityDelta(Vector3d.Right + Vector3d.Forward);
        }
        var trigger = new LSCompoundCollider2D(
            CompoundColliderPart2D.AABBox(new Vector2d(Fixed64.FromFraction(1, 5), (Fixed64)4),
                Vector2d.Right * Fixed64.FromFraction(2, 5)),
            CompoundColliderPart2D.AABBox(new Vector2d((Fixed64)4, Fixed64.FromFraction(1, 5)),
                Vector2d.Forward * Fixed64.FromFraction(2, 5))) { IsTrigger = !spatialTrigger };
        var agent2D = new TestMatterAgent(context,
            new FixedTransform(Vector3d.Zero, FixedQuaternion.Identity, Vector3d.One));
        SolidBody2D? body2D = null;
        if (!spatialTrigger) trigger.InitializeWithNoBody(agent2D);
        else
        {
            body2D = new SolidBody2D(agent2D, trigger) { Mass = Fixed64.One };
            body2D.Initialize(Vector2d.Zero);
            body2D.ApplyCollisionLinearVelocityDelta(Vector2d.Right + Vector2d.Forward);
        }
        CollisionPairMixed pair = BuildMixedManifoldPair(sphere, trigger);
        pair.Manifold.GroupCount.Should().Be(2);

        CollisionResponseMixed.ResolveManifold(pair);

        if (body3D != null)
        {
            body3D.Position3d.Should().Be(Vector3d.Zero);
            body3D.LinearVelocity.Should().Be(Vector3d.Right + Vector3d.Forward);
            body3D.AngularVelocity.Should().Be(Vector3d.Zero);
        }
        else
        {
            body2D!.Position.Should().Be(Vector2d.Zero);
            body2D.LinearVelocity.Should().Be(Vector2d.Right + Vector2d.Forward);
            body2D.AngularVelocity.Should().Be(Fixed64.Zero);
        }
        pair.SolveImpulses.Count.Should().Be(0);
    }

    [Fact]
    public void MixedIndependentHeavyRows_ShouldGrowThenReuseTheExactImpulseBank()
    {
        using GravitasWorldContext context = CreateMixedContext();
        ScenarioBody<LSSphereCollider> source = CreateSphere3D(context, Vector3d.Zero, preventAngularForces: true);
        var collider = new LSCompoundCollider2D(
            CompoundColliderPart2D.AABBox(new Vector2d(Fixed64.FromFraction(1, 5), Fixed64.FromFraction(1, 5)),
                Vector2d.Right * Fixed64.FromFraction(2, 5), PhysicsMaterial.Frictionless),
            CompoundColliderPart2D.AABBox(new Vector2d(Fixed64.FromFraction(1, 5), Fixed64.FromFraction(1, 5)),
                Vector2d.Forward * Fixed64.FromFraction(2, 5), PhysicsMaterial.Frictionless));
        var target = new SolidBody2D(new TestMatterAgent(context,
            new FixedTransform(Vector3d.Zero, FixedQuaternion.Identity, Vector3d.One)), collider) { Mass = Fixed64.One };
        target.Initialize(Vector2d.Zero);
        target.FreezeAxes = BodyFreezeAxes2D.Rotation;
        source.Body.Mass = target.Mass = Fixed64.MaxValue;
        source.Collider.Material = PhysicsMaterial.Frictionless;
        CollisionPairMixed pair = BuildMixedManifoldPair(source.Collider, collider);
        pair.Manifold.GroupCount.Should().Be(2);
        source.Body.ApplyCollisionLinearVelocityDelta((Vector3d.Right + Vector3d.Forward) * (Fixed64)4);

        CollisionResponseMixed.ResolveManifold(pair);

        // Each orthogonal row carries λ=2m, wider than Fixed64; both final
        // velocity components fit. Distinct groups require distinct exact states.
        source.Body.LinearVelocity.Should().Be((Vector3d.Right + Vector3d.Forward) * Fixed64.Two);
        target.LinearVelocity.Should().Be((Vector2d.Right + Vector2d.Forward) * Fixed64.Two);
        ExactContactImpulseState?[] states = pair.ExactSolveImpulses!;
        states.Length.Should().BeGreaterThan(ContactManifold.MaxContactsPerGroup);
        states[0]!.HasNormal.Should().BeTrue();
        states[ContactManifold.MaxContactsPerGroup]!.HasNormal.Should().BeTrue();
        source.Body.ApplyCollisionLinearVelocityDelta((Vector3d.Right + Vector3d.Forward) * Fixed64.Two);
        target.ApplyCollisionLinearVelocityDelta((Vector2d.Left + Vector2d.Backward) * Fixed64.Two);
        pair.MarkResting(1, pair.Manifold);

        CollisionResponseMixed.ResolveManifold(pair);

        pair.ExactSolveImpulses.Should().BeSameAs(states);
        source.Body.LinearVelocity.Should().Be((Vector3d.Right + Vector3d.Forward) * Fixed64.Two);
        target.LinearVelocity.Should().Be((Vector2d.Right + Vector2d.Forward) * Fixed64.Two);

        source.Body.FreezeAxes = BodyFreezeAxes3D.All;
        target.FreezeAxes = BodyFreezeAxes2D.All;
        var stationary = new ContactResponseSnapshot(source.Body.Position3d, target.Agent.Transform.WorldPosition,
            source.Body.GetCenterOfMassAnchor(), new ContactAnchor(target.Position.ToVector3d(Fixed64.Zero),
                FixedQuaternion.Identity, target.LocalCenterOfMassOffset.ToVector3d(Fixed64.Zero)),
            Vector3d.Zero, Vector3d.Zero, Vector3d.Zero, Vector3d.Zero);
        ContactWarmStartCache cache = default;
        ExactContactImpulseState?[]? retained = states;

        CollisionResponseMixed.TrySolveManifoldVelocity(source.Collider, collider, pair.Manifold,
            stationary, ref cache, ref retained, out ContactResponseSnapshot solved).Should().BeFalse();

        // A subsequent fully locked step has no effective response. Previously
        // allocated exact owners are retained but must carry no impulse into it.
        retained.Should().BeSameAs(states);
        states.Should().OnlyContain(state => state == null || !state.HasNormal && !state.HasTangents);
        solved.LinearA.Should().Be(Vector3d.Zero);
        solved.LinearB.Should().Be(Vector3d.Zero);

        ManifoldContact lockedRow = pair.Manifold.GetGroup(0)[0];
        ExactContactImpulseState? cleared = states[0];
        var first = ExactContactLever3D.CreateResponseOperand(source.Body, Vector3d.Zero, Vector3d.Zero,
            lockedRow.AnchorA.GetLeverFrom(stationary.CenterA), -lockedRow.Normal);
        var second = ExactContactLever2D.CreateResponseOperand(target, Vector2d.Zero, Fixed64.Zero,
            lockedRow.AnchorB.GetLeverFrom(stationary.CenterB), lockedRow.Normal);

        // The retained owner exists but was cleared at the new step boundary.
        // Its zero-effective row must report an empty, representable accumulator.
        ExactContactResponseKernel.TryGetAccumulatedNormalResponse(first, second,
            lockedRow.Normal, Fixed64.Zero, Fixed64.Zero, Fixed64.Zero, Fixed64.One, Fixed64.One,
            out ExactNormalResponse3D empty, stationary, ref cleared).Should().BeTrue();

        cleared.Should().BeSameAs(states[0]);
        empty.HasAppliedImpulse.Should().BeFalse();
        empty.TryGetAccumulatedImpulse(out Fixed64 remaining).Should().BeTrue();
        remaining.Should().Be(Fixed64.Zero);
    }

    [Fact]
    public void DetectedCircularNormal_WithWideCompoundLocalCenter_ShouldUseTheFiniteFrozenCenter()
    {
        using GravitasWorldContext context = CreateMixedContext();
        var collider = new LSCompoundCollider2D(
            CompoundColliderPart2D.Circle(Fixed64.Half, Vector2d.Right * (Fixed64)1_000_000_000, PhysicsMaterial.Frictionless),
            CompoundColliderPart2D.Circle(Fixed64.Half, Vector2d.Left * (Fixed64)1_000_000_000, PhysicsMaterial.Frictionless))
        {
            LocalOffset = Vector2d.Right * (Fixed64)1_500_000_000,
            MixedHalfThicknessOverride = Fixed64.Half
        };
        Vector2d position = Vector2d.Left * (Fixed64)1_500_000_000;
        var body = new SolidBody2D(new TestMatterAgent(context,
            new FixedTransform(position.ToVector3d(Fixed64.Zero), FixedQuaternion.Identity, Vector3d.One)), collider)
        {
            // The enormous authored lever remains physical: minimum positive mass
            // keeps its second moment in the scalar domain rather than saturating it.
            Mass = Fixed64.MinIncrement
        };
        body.Initialize(position);
        body.UseManualGrounding();
        Fixed64 x = (Fixed64)1_000_000_000 + Fixed64.Quarter;
        var mesh = new LSMeshCollider(new[]
        {
            new Vector3d(x, (Fixed64)(-2), (Fixed64)(-2)), new Vector3d(x, (Fixed64)2, (Fixed64)(-2)),
            new Vector3d(x, (Fixed64)(-2), (Fixed64)2), new Vector3d(x, (Fixed64)2, (Fixed64)2)
        }, new[] { 0, 2, 1, 1, 2, 3 }, MeshColliderMode.Concave, MeshInertiaPolicy.SurfaceApproximation);
        mesh.InitializeWithNoBody(new TestMatterAgent(context,
            new FixedTransform(Vector3d.Zero, FixedQuaternion.Identity, Vector3d.One)));
        var manifold = new ContactManifold();
        CollisionDetectionMixed.TryBuildManifold(mesh, collider, manifold).Should().BeTrue();
        collider.GetPartCollider(0).TransformRelativeMassPropertyPointExact(Vector2d.Zero)
            .TryGetPoint(out _).Should().BeFalse();
        var center = new ContactAnchor(position.ToVector3d(Fixed64.Zero), FixedQuaternion.Identity,
            body.LocalCenterOfMassOffset.ToVector3d(Fixed64.Zero));

        foreach (ManifoldContact row in manifold)
        {
            row.FeatureNamespaceB.Should().Be(-1);
            ContactAnchor normalPoint = ExactContactLever2D.GetDetectedNormalAnchor(collider,
                row.FeatureNamespaceB, center, row.AnchorB);
            normalPoint.TryGetWorldPoint(out Vector3d worldPoint).Should().BeTrue();
            worldPoint.Should().Be(Vector3d.Right * (Fixed64)1_000_000_000);
        }
        // This leaf lies on the body's X axis and meets a vertical X wall.
        // Its genuine center-to-COM lever is parallel to the normal, so yaw
        // contributes no normal speed, even though its local center cannot narrow.
        MixedContact.TrySelectClosingContact(manifold, ContactAnchor.FromWorldPoint(Vector3d.Zero),
            Vector3d.Zero, Vector3d.Zero, center, Vector2d.Zero, Fixed64.One, out _, collider)
            .Should().BeFalse();
    }

    [Fact]
    public void MixedManifold_WithElasticReboundOutsideTheDeltaDomain_ShouldRejectWithoutVelocityMutation()
    {
        using GravitasWorldContext context = CreateMixedContext();
        context.Environment.MaxSpeed = Fixed64.MaxValue;
        var elastic = new PhysicsMaterial(Fixed64.Zero, Fixed64.Zero, Fixed64.One);
        context.Settings.RestitutionVelocityThreshold = Fixed64.Zero;
        ScenarioBody<LSSphereCollider> source = CreateSphere3D(context, Vector3d.Zero, preventAngularForces: true);
        var walls = new LSCompoundCollider2D(
            CompoundColliderPart2D.AABBox(new Vector2d(Fixed64.FromFraction(1, 5), (Fixed64)4),
                Vector2d.Right * Fixed64.FromFraction(2, 5), elastic),
            CompoundColliderPart2D.AABBox(new Vector2d((Fixed64)4, Fixed64.FromFraction(1, 5)),
                Vector2d.Forward * Fixed64.FromFraction(2, 5), elastic));
        walls.InitializeWithNoBody(new TestMatterAgent(context,
            new FixedTransform(Vector3d.Zero, FixedQuaternion.Identity, Vector3d.One)));
        source.Collider.Material = elastic;
        source.Body.ApplyCollisionLinearVelocityDelta(Vector3d.Right * Fixed64.MaxValue);
        source.Body.LinearVelocity.Should().Be(Vector3d.Right * Fixed64.MaxValue);
        CollisionPairMixed pair = BuildMixedManifoldPair(source.Collider, walls);
        pair.Manifold.GroupCount.Should().Be(2);
        var impact = new ContactResponseSnapshot(Vector3d.Zero, Vector3d.Zero,
            source.Body.GetCenterOfMassAnchor(), ContactAnchor.FromWorldPoint(Vector3d.Zero),
            source.Body.LinearVelocity, Vector3d.Zero, Vector3d.Zero, Vector3d.Zero);
        ContactWarmStartCache cache = default;
        ExactContactImpulseState?[]? exact = null;

        CollisionResponseMixed.TrySolveManifoldVelocity(source.Collider, walls, pair.Manifold,
            impact, ref cache, ref exact, out _).Should().BeFalse();
        CollisionResponseMixed.ResolveManifold(pair);

        // Reflecting +MaxValue requires a -2*MaxValue delta. The velocity
        // response is rejected explicitly; the independent tangent wall stays inert.
        source.Body.LinearVelocity.Should().Be(Vector3d.Right * Fixed64.MaxValue);
        source.Body.AngularVelocity.Should().Be(Vector3d.Zero);
    }

    [Fact]
    public void MixedManifold_WithAngularReboundBeyondTheFinalDomain_ShouldRejectTheCompletePreview()
    {
        using GravitasWorldContext context = CreateMixedContext();
        var box = new LSCuboidCollider { Size = Vector3d.One, Material = PhysicsMaterial.Frictionless };
        var source = new SolidBody(new TestMatterAgent(context,
            new FixedTransform(Vector3d.Zero, FixedQuaternion.Identity, Vector3d.One)), box) { Mass = Fixed64.One };
        source.Initialize(Vector3d.Zero, FixedQuaternion.Identity);
        var wall = new LSAABBoxCollider2D(new Vector2d(Fixed64.Half, Fixed64.FromFraction(1, 5)))
        { Material = PhysicsMaterial.Frictionless };
        // Its finite X span [1/4, 3/4] touches only the source's positive-X
        // face levers: every inward impulse reinforces the incoming negative yaw.
        Vector2d wallPosition = new(Fixed64.Half, Fixed64.FromFraction(2, 5));
        var target = new SolidBody2D(new TestMatterAgent(context,
            new FixedTransform(wallPosition.ToVector3d(Fixed64.Zero), FixedQuaternion.Identity, Vector3d.One)), wall)
        { Mass = Fixed64.One };
        target.Initialize(wallPosition, motionType: BodyMotionType.Kinematic);
        var manifold = new ContactManifold();
        CollisionDetectionMixed.TryBuildManifold(box, wall, manifold).Should().BeTrue();
        var impact = new ContactResponseSnapshot(Vector3d.Zero, wallPosition.ToVector3d(Fixed64.Zero),
            source.GetCenterOfMassAnchor(), new ContactAnchor(wallPosition.ToVector3d(Fixed64.Zero), Vector3d.Zero),
            Vector3d.Zero, Vector3d.Up * Fixed64.MinValue,
            Vector3d.Forward * Fixed64.MaxValue, Vector3d.Zero);
        ContactWarmStartCache cache = default;
        ExactContactImpulseState?[]? exact = null;

        CollisionResponseMixed.TrySolveManifoldVelocity(box, wall, manifold, impact,
            ref cache, ref exact, out _).Should().BeFalse();

        // The fast-moving wall can increase an already minimum negative yaw.
        // Although each impulse delta fits, the final angular sum does not.
        source.LinearVelocity.Should().Be(Vector3d.Zero);
        source.AngularVelocity.Should().Be(Vector3d.Zero);
        target.LinearVelocity.Should().Be(Vector2d.Zero);
    }
}
