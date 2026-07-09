using FixedMathSharp;
using FluentAssertions;
using Gravitas.Colliders;
using Gravitas.CollisionHandling;
using Gravitas.Queries;
using Gravitas.Tests.Support;
using Xunit;

namespace Gravitas.Tests.CollisionHandlingTests;

public sealed class ContinuousCollisionPolicyTests
{
    [Fact]
    public void AllowsDynamic3DTarget_ShouldRejectEachInvalidEligibilityGate()
    {
        ContinuousCollisionTargetPolicy.AllowsDynamic3DTarget(
                isSelf: false,
                active: true,
                positionFullyFrozen: false,
                kinematic: false,
                trigger: false,
                sibling: false,
                layerCollisionDisabled: false,
                physicalPairAllowed: true)
            .Should().BeTrue();

        ContinuousCollisionTargetPolicy.AllowsDynamic3DTarget(true, true, false, false, false, false, false, true).Should().BeFalse();
        ContinuousCollisionTargetPolicy.AllowsDynamic3DTarget(false, false, false, false, false, false, false, true).Should().BeFalse();
        ContinuousCollisionTargetPolicy.AllowsDynamic3DTarget(false, true, true, false, false, false, false, true).Should().BeFalse();
        ContinuousCollisionTargetPolicy.AllowsDynamic3DTarget(false, true, false, true, false, false, false, true).Should().BeFalse();
        ContinuousCollisionTargetPolicy.AllowsDynamic3DTarget(false, true, false, false, true, false, false, true).Should().BeFalse();
        ContinuousCollisionTargetPolicy.AllowsDynamic3DTarget(false, true, false, false, false, true, false, true).Should().BeFalse();
        ContinuousCollisionTargetPolicy.AllowsDynamic3DTarget(false, true, false, false, false, false, true, true).Should().BeFalse();
        ContinuousCollisionTargetPolicy.AllowsDynamic3DTarget(false, true, false, false, false, false, false, false).Should().BeFalse();
    }

    [Fact]
    public void AllowsDynamic2DTarget_ShouldRejectEachInvalidEligibilityGate()
    {
        ContinuousCollisionTargetPolicy.AllowsDynamic2DTarget(
                isSelf: false,
                active: true,
                positionFullyFrozen: false,
                kinematic: false,
                trigger: false,
                physicalPairRequired: true)
            .Should().BeTrue();

        ContinuousCollisionTargetPolicy.AllowsDynamic2DTarget(true, true, false, false, false, true).Should().BeFalse();
        ContinuousCollisionTargetPolicy.AllowsDynamic2DTarget(false, false, false, false, false, true).Should().BeFalse();
        ContinuousCollisionTargetPolicy.AllowsDynamic2DTarget(false, true, true, false, false, true).Should().BeFalse();
        ContinuousCollisionTargetPolicy.AllowsDynamic2DTarget(false, true, false, true, false, true).Should().BeFalse();
        ContinuousCollisionTargetPolicy.AllowsDynamic2DTarget(false, true, false, false, true, true).Should().BeFalse();
        ContinuousCollisionTargetPolicy.AllowsDynamic2DTarget(false, true, false, false, false, false).Should().BeFalse();
    }

    [Fact]
    public void AllowsMixedDynamicTarget_ShouldRejectEachInvalidEligibilityGate()
    {
        ContinuousCollisionTargetPolicy.AllowsMixedDynamicTarget(
                active: true,
                positionFullyFrozen: false,
                kinematic: false,
                trigger: false,
                mixedPairRequired: true)
            .Should().BeTrue();

        ContinuousCollisionTargetPolicy.AllowsMixedDynamicTarget(false, false, false, false, true).Should().BeFalse();
        ContinuousCollisionTargetPolicy.AllowsMixedDynamicTarget(true, true, false, false, true).Should().BeFalse();
        ContinuousCollisionTargetPolicy.AllowsMixedDynamicTarget(true, false, true, false, true).Should().BeFalse();
        ContinuousCollisionTargetPolicy.AllowsMixedDynamicTarget(true, false, false, true, true).Should().BeFalse();
        ContinuousCollisionTargetPolicy.AllowsMixedDynamicTarget(true, false, false, false, false).Should().BeFalse();
    }

    [Fact]
    public void AllowsStaticOrKinematic3DTarget_ShouldRequireValidFilteredStaticOrKinematicCollider()
    {
        ContinuousCollisionTargetPolicy.AllowsStaticOrKinematic3DTarget(
                hasCollider: true,
                isSelf: false,
                ignored: false,
                trigger: false,
                sibling: false,
                layerCollisionDisabled: false,
                physicalPairAllowed: true,
                isStatic: true,
                bodyKinematic: false)
            .Should().BeTrue();

        ContinuousCollisionTargetPolicy.AllowsStaticOrKinematic3DTarget(true, false, false, false, false, false, true, false, true)
            .Should().BeTrue();

        ContinuousCollisionTargetPolicy.AllowsStaticOrKinematic3DTarget(false, false, false, false, false, false, true, true, false).Should().BeFalse();
        ContinuousCollisionTargetPolicy.AllowsStaticOrKinematic3DTarget(true, true, false, false, false, false, true, true, false).Should().BeFalse();
        ContinuousCollisionTargetPolicy.AllowsStaticOrKinematic3DTarget(true, false, true, false, false, false, true, true, false).Should().BeFalse();
        ContinuousCollisionTargetPolicy.AllowsStaticOrKinematic3DTarget(true, false, false, true, false, false, true, true, false).Should().BeFalse();
        ContinuousCollisionTargetPolicy.AllowsStaticOrKinematic3DTarget(true, false, false, false, true, false, true, true, false).Should().BeFalse();
        ContinuousCollisionTargetPolicy.AllowsStaticOrKinematic3DTarget(true, false, false, false, false, true, true, true, false).Should().BeFalse();
        ContinuousCollisionTargetPolicy.AllowsStaticOrKinematic3DTarget(true, false, false, false, false, false, false, true, false).Should().BeFalse();
        ContinuousCollisionTargetPolicy.AllowsStaticOrKinematic3DTarget(true, false, false, false, false, false, true, false, false).Should().BeFalse();
    }

    [Fact]
    public void AllowsStaticOrKinematic2DTarget_ShouldRequireValidFilteredStaticOrKinematicCollider()
    {
        ContinuousCollisionTargetPolicy.AllowsStaticOrKinematic2DTarget(
                hasCollider: true,
                isSelf: false,
                ignored: false,
                trigger: false,
                physicalPairRequired: true,
                isStatic: true,
                bodyKinematic: false)
            .Should().BeTrue();

        ContinuousCollisionTargetPolicy.AllowsStaticOrKinematic2DTarget(true, false, false, false, true, false, true)
            .Should().BeTrue();

        ContinuousCollisionTargetPolicy.AllowsStaticOrKinematic2DTarget(false, false, false, false, true, true, false).Should().BeFalse();
        ContinuousCollisionTargetPolicy.AllowsStaticOrKinematic2DTarget(true, true, false, false, true, true, false).Should().BeFalse();
        ContinuousCollisionTargetPolicy.AllowsStaticOrKinematic2DTarget(true, false, true, false, true, true, false).Should().BeFalse();
        ContinuousCollisionTargetPolicy.AllowsStaticOrKinematic2DTarget(true, false, false, true, true, true, false).Should().BeFalse();
        ContinuousCollisionTargetPolicy.AllowsStaticOrKinematic2DTarget(true, false, false, false, false, true, false).Should().BeFalse();
        ContinuousCollisionTargetPolicy.AllowsStaticOrKinematic2DTarget(true, false, false, false, true, false, false).Should().BeFalse();
    }

    [Fact]
    public void AllowsMixedStaticOrKinematicTarget_ShouldRequireValidFilteredStaticOrKinematicCollider()
    {
        ContinuousCollisionTargetPolicy.AllowsMixedStaticOrKinematicTarget(
                hasCollider: true,
                ignored: false,
                trigger: false,
                mixedPairRequired: true,
                isStatic: true,
                bodyKinematic: false)
            .Should().BeTrue();

        ContinuousCollisionTargetPolicy.AllowsMixedStaticOrKinematicTarget(true, false, false, true, false, true)
            .Should().BeTrue();

        ContinuousCollisionTargetPolicy.AllowsMixedStaticOrKinematicTarget(false, false, false, true, true, false).Should().BeFalse();
        ContinuousCollisionTargetPolicy.AllowsMixedStaticOrKinematicTarget(true, true, false, true, true, false).Should().BeFalse();
        ContinuousCollisionTargetPolicy.AllowsMixedStaticOrKinematicTarget(true, false, true, true, true, false).Should().BeFalse();
        ContinuousCollisionTargetPolicy.AllowsMixedStaticOrKinematicTarget(true, false, false, false, true, false).Should().BeFalse();
        ContinuousCollisionTargetPolicy.AllowsMixedStaticOrKinematicTarget(true, false, false, true, false, false).Should().BeFalse();
    }

    [Fact]
    public void TryResolveSourceNormal3D_ShouldPreferExplicitNormalThenDisplacementFallback()
    {
        ContinuousCollisionImpulsePolicy.TryResolveSourceNormal(
                new Vector3d((Fixed64)2, Fixed64.Zero, Fixed64.Zero),
                Vector3d.Forward,
                out Vector3d explicitNormal)
            .Should().BeTrue();
        explicitNormal.Should().Be(Vector3d.Right);

        ContinuousCollisionImpulsePolicy.TryResolveSourceNormal(
                Vector3d.Zero,
                Vector3d.Forward * (Fixed64)2,
                out Vector3d displacementFallback)
            .Should().BeTrue();
        displacementFallback.Should().Be(-Vector3d.Forward);

        ContinuousCollisionImpulsePolicy.TryResolveSourceNormal(
                Vector3d.Zero,
                Vector3d.Zero,
                out Vector3d zeroNormal)
            .Should().BeFalse();
        zeroNormal.Should().Be(Vector3d.Zero);
    }

    [Fact]
    public void TryResolveSourceNormal2D_ShouldPreferExplicitNormalThenDisplacementFallback()
    {
        ContinuousCollisionImpulsePolicy.TryResolveSourceNormal(
                new Vector2d((Fixed64)2, Fixed64.Zero),
                Vector2d.Forward,
                out Vector2d explicitNormal)
            .Should().BeTrue();
        explicitNormal.Should().Be(Vector2d.Right);

        ContinuousCollisionImpulsePolicy.TryResolveSourceNormal(
                Vector2d.Zero,
                Vector2d.Forward * (Fixed64)2,
                out Vector2d displacementFallback)
            .Should().BeTrue();
        displacementFallback.Should().Be(-Vector2d.Forward);

        ContinuousCollisionImpulsePolicy.TryResolveSourceNormal(
                Vector2d.Zero,
                Vector2d.Zero,
                out Vector2d zeroNormal)
            .Should().BeFalse();
        zeroNormal.Should().Be(Vector2d.Zero);
    }

    [Fact]
    public void TryResolveImpactNormal_ShouldRejectZeroNormals()
    {
        ContinuousCollisionImpulsePolicy.TryResolveImpactNormal(
                new Vector3d(Fixed64.Zero, (Fixed64)3, Fixed64.Zero),
                out Vector3d normal3D)
            .Should().BeTrue();
        normal3D.Should().Be(Vector3d.Up);

        ContinuousCollisionImpulsePolicy.TryResolveImpactNormal(Vector3d.Zero, out normal3D)
            .Should().BeFalse();
        normal3D.Should().Be(Vector3d.Zero);

        ContinuousCollisionImpulsePolicy.TryResolveImpactNormal(
                new Vector2d(Fixed64.Zero, (Fixed64)3),
                out Vector2d normal2D)
            .Should().BeTrue();
        normal2D.Should().Be(Vector2d.Forward);

        ContinuousCollisionImpulsePolicy.TryResolveImpactNormal(Vector2d.Zero, out normal2D)
            .Should().BeFalse();
        normal2D.Should().Be(Vector2d.Zero);
    }

    [Fact]
    public void ResolveSweptSpherePoint_ShouldUseTargetSurfaceExceptCenterOverlapFallback()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSSphereCollider> target = scenario.CreateSphere(Vector3d.Zero, immovable: true);

        ContinuousCollisionContactPolicy.ResolveSweptSpherePoint(
                target.Collider,
                Vector3d.Right * (Fixed64)2,
                Vector3d.Right)
            .Should().Be(Vector3d.Right * target.Collider.ScaledRadius);

        ContinuousCollisionContactPolicy.ResolveSweptSpherePoint(
                target.Collider,
                target.Collider.Center,
                Vector3d.Right)
            .Should().Be(-Vector3d.Right * target.Collider.ScaledRadius);
    }

    [Fact]
    public void ResolveSweptSphereNormal_ShouldUseCurvedShapeDeltaBeforeTargetNormal()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSCuboidCollider> target = scenario.CreateCuboid(Vector3d.Zero, immovable: true);

        Vector3d normal = ContinuousCollisionContactPolicy.ResolveSweptSphereNormal(
            target.Collider,
            Vector3d.Right * Fixed64.Half,
            Vector3d.Right * (Fixed64)2,
            Vector3d.Right);

        normal.Should().Be(Vector3d.Right);
    }

    [Fact]
    public void ResolveSweptSphereNormal_ShouldFlipMeshNormalFacingSweepDirection()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ScenarioBody<LSMeshCollider> mesh = scenario.CreateBody(
            MeshTestFixtures.CreateVerticalQuad(
                Fixed64.Zero,
                -Fixed64.One,
                Fixed64.One,
                inertiaPolicy: MeshInertiaPolicy.SurfaceApproximation),
            Vector3d.Zero,
            FixedQuaternion.Identity,
            immovable: true);

        Vector3d normal = ContinuousCollisionContactPolicy.ResolveSweptSphereNormal(
            mesh.Collider,
            new Vector3d(Fixed64.Zero, Fixed64.One, Fixed64.Zero),
            new Vector3d((Fixed64)(-1), Fixed64.One, Fixed64.Zero),
            Vector3d.Right);

        normal.Should().Be(-Vector3d.Right);
    }

    [Fact]
    public void ResolveSweptSphereNormal_ShouldFallbackToDeltaThenDirection()
    {
        var target = new ZeroNormalCollider3D();

        ContinuousCollisionContactPolicy.ResolveSweptSphereNormal(
                target,
                Vector3d.Zero,
                Vector3d.Right,
                Vector3d.Right)
            .Should().Be(Vector3d.Right);

        ContinuousCollisionContactPolicy.ResolveSweptSphereNormal(
                target,
                Vector3d.Zero,
                Vector3d.Zero,
                Vector3d.Right)
            .Should().Be(-Vector3d.Right);

        ContinuousCollisionContactPolicy.ResolveSweptSphereNormal(
                target,
                Vector3d.Zero,
                Vector3d.Zero,
                Vector3d.Zero)
            .Should().Be(Vector3d.Zero);
    }

    [Fact]
    public void ResolveConvexSweepHitNormal_ShouldUseTargetNormalThenFallbacksAndOrientAgainstSweep()
    {
        var upward = new UnsupportedTestCollider3D();
        ConvexSweepHitPolicy.ResolveHitNormal(
                upward,
                Vector3d.Zero,
                Vector3d.Right,
                Vector3d.Forward,
                Vector3d.Up)
            .Should().Be(-Vector3d.Up);

        var zero = new ZeroNormalCollider3D();
        ConvexSweepHitPolicy.ResolveHitNormal(
                zero,
                Vector3d.Zero,
                Vector3d.Right * (Fixed64)2,
                Vector3d.Forward,
                Vector3d.Up)
            .Should().Be(Vector3d.Right);

        ConvexSweepHitPolicy.ResolveHitNormal(
                zero,
                Vector3d.Zero,
                Vector3d.Zero,
                Vector3d.Forward * (Fixed64)2,
                Vector3d.Up)
            .Should().Be(Vector3d.Forward);

        ConvexSweepHitPolicy.ResolveHitNormal(
                zero,
                Vector3d.Zero,
                Vector3d.Zero,
                Vector3d.Zero,
                Vector3d.Up)
            .Should().Be(-Vector3d.Up);

        ConvexSweepHitPolicy.ResolveHitNormal(
                zero,
                Vector3d.Zero,
                Vector3d.Zero,
                Vector3d.Zero,
                Vector3d.Zero)
            .Should().Be(Vector3d.Zero);
    }

    private sealed class ZeroNormalCollider3D : LSCollider
    {
        public override ColliderType Shape => (ColliderType)byte.MaxValue;

        public override int Priority => 0;

        protected override void BuildShape()
        {
            Area = Fixed64.One;
            SetBoundsMinMax(Center - Vector3d.One, Center + Vector3d.One);
        }

        public override Fixed3x3 CalculateInertiaTensor(Fixed64 mass, Vector3d localCenterOfMassOffset) =>
            Fixed3x3.Zero;

        public override Vector3d ClosestPointOnSurface(Vector3d other) => Center;

        public override Vector3d GetNormalAtPoint(Vector3d point) => Vector3d.Zero;

        public override bool ColliderOverlapsRay(RaycastSegmentWorker worker, ref SwiftCollections.SwiftList<Vector3d> outputIntersectionPoints) =>
            false;
    }
}
