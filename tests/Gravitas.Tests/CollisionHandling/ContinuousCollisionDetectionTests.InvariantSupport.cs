using System;
using FixedMathSharp;
using FluentAssertions;
using Gravitas.Colliders;
using Gravitas.CollisionHandling;
using Gravitas.Queries;
using Gravitas.Support;
using Gravitas.Tests.Support;
using Xunit;

namespace Gravitas.Tests.CollisionHandlingTests;

public sealed partial class ContinuousCollisionDetectionTests
{
    [Theory]
    [InlineData(ColliderType.AABox, 0)]
    [InlineData(ColliderType.AABox, 1)]
    [InlineData(ColliderType.AABox, -1)]
    [InlineData(ColliderType.Sphere, 0)]
    [InlineData(ColliderType.Sphere, 1)]
    [InlineData(ColliderType.Sphere, -1)]
    [InlineData(ColliderType.Capsule, 0)]
    [InlineData(ColliderType.Capsule, 1)]
    [InlineData(ColliderType.Capsule, -1)]
    [InlineData(ColliderType.Cylinder, 0)]
    [InlineData(ColliderType.Cylinder, 1)]
    [InlineData(ColliderType.Cylinder, -1)]
    [InlineData(ColliderType.Cone, 0)]
    [InlineData(ColliderType.Cone, 1)]
    [InlineData(ColliderType.Cone, -1)]
    public void RotationalArbiter_CenteredSphereOnPrimitive_ShouldRejectOnlyClosingMotion(
        ColliderType supportType,
        int normalDirection)
    {
        using PhysicsScenarioBuilder scenario = CreateCcdScenario();
        LSCollider support = CreateInvariantSupportPrimitive(supportType);
        Vector3d supportCenter = supportType == ColliderType.Capsule
            ? Vector3d.Down * (Fixed64)2
            : Vector3d.Down;
        // The cone's flat base faces upward; this exact half-turn introduces
        // no trigonometric rounding into the authored support plane y = 0.
        FixedQuaternion supportRotation = supportType == ColliderType.Cone
            ? new FixedQuaternion(Fixed64.One, Fixed64.Zero, Fixed64.Zero, Fixed64.Zero)
            : FixedQuaternion.Identity;
        scenario.CreateBody(support, supportCenter, supportRotation, immovable: true);
        AddSeparatedInvariantSupportBlade(scenario);
        Vector3d start = Vector3d.Up * Fixed64.FromFraction(1, 8);
        ScenarioBody<LSSphereCollider> source = scenario.CreateBody(
            new LSSphereCollider { Radius = Fixed64.FromFraction(1, 8) },
            start, FixedQuaternion.Identity, isKinematic: true);
        source.Body.UseManualGrounding();
        source.Body.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        Vector3d displacement = normalDirection == 0
            ? Vector3d.Right * Fixed64.FromFraction(1, 1024)
            : Vector3d.Up * Fixed64.FromFraction(normalDirection, 64);
        Vector3d requested = start + displacement;
        FixedQuaternion requestedRotation = PhysicsScenarioBuilder.Yaw(5);
        source.Body.Agent.Transform.LocalPosition = requested;
        source.Body.Agent.Transform.LocalRotation = requestedRotation;

        scenario.Context.LateSimulate();

        if (normalDirection < 0)
        {
            // Contact exists from t = 0. The depth-limited search accepts the
            // first leaf's midpoint witness: 1 / 2^(MaxDepth + 1) of the frame.
            // For this 1/64-unit request that permits at most 1/524288 unit.
            Fixed64 contactTimeResolution = Fixed64.FromFraction(
                1, 1 << (ContinuousCollisionMath.RotationalIntervalMaxDepth + 1));
            Fixed64 maximumPenetration = displacement.Y.Abs() * contactTimeResolution;
            source.Body.Position3d.Y.Should().BeInRange(start.Y - maximumPenetration, start.Y,
                "closing against {0} may consume only the initial contact interval", supportType);
            source.Body.Position3d.Y.Should().BeGreaterThan(requested.Y,
                "the support must reject the requested penetration, not discard the contact");
            source.Body.Position3d.X.Should().Be(start.X);
            source.Body.Position3d.Z.Should().Be(start.Z);
            source.Body.LastContinuousCollisionToiIterationCount.Should().BeGreaterThan(0);
        }
        else
        {
            source.Body.Position3d.Should().Be(requested,
                "tangent or separating movement must not stick to {0}", supportType);
            source.Body.Rotation.Should().Be(requestedRotation);
            source.Body.LastContinuousCollisionToiIterationCount.Should().Be(0);
        }
    }

    [Fact]
    public void RotationalArbiter_CompoundSupport_ShouldNotHideItsLaterBlockingChild()
    {
        using PhysicsScenarioBuilder scenario = CreateCcdScenario();
        var compound = new LSCompoundCollider(
            CompoundColliderPart.Cuboid(new Vector3d(4, 2, 4), Vector3d.Down),
            CompoundColliderPart.Cuboid(new Vector3d(Fixed64.Quarter, (Fixed64)2, (Fixed64)2),
                Vector3d.Right * Fixed64.Half));
        scenario.InitializeStaticCollider(compound, Vector3d.Zero);
        AddSeparatedInvariantSupportBlade(scenario);
        Vector3d start = Vector3d.Up * Fixed64.FromFraction(1, 8);
        ScenarioBody<LSSphereCollider> source = scenario.CreateBody(
            new LSSphereCollider { Radius = Fixed64.FromFraction(1, 8) },
            start, FixedQuaternion.Identity, isKinematic: true);
        source.Body.UseManualGrounding();
        source.Body.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        scenario.Context.Query3D.SweepSphere(start, Fixed64.FromFraction(1, 8),
            Vector3d.Right, Fixed64.One, out Physics3DHit firstHit,
            PhysicsLayerMask.All, source.Collider).Should().BeTrue();
        firstHit.Collider.Should().BeSameAs(compound);
        Vector3d.Dot(firstHit.Normal, Vector3d.Right).Should().Be(Fixed64.Zero,
            "the first child is tangent support, not the later blocking wall");
        source.Body.Agent.Transform.LocalPosition = start + Vector3d.Right;
        source.Body.Agent.Transform.LocalRotation = PhysicsScenarioBuilder.Yaw(5);

        scenario.Context.LateSimulate();

        // The second child's near face is 3/8, so the center may not pass 1/4.
        // A conservative stop on the first child is permitted; tunnelling is not.
        source.Body.Position3d.X.Should().BeInRange(Fixed64.Zero, Fixed64.Quarter);
        source.Body.Position3d.Y.Should().Be(start.Y);
        source.Body.LastContinuousCollisionToiIterationCount.Should().BeGreaterThan(0);
    }

    private static LSCollider CreateInvariantSupportPrimitive(ColliderType shape) => shape switch
    {
        ColliderType.AABox => new LSCuboidCollider { Size = new Vector3d(4, 2, 4) },
        ColliderType.Sphere => new LSSphereCollider { Radius = Fixed64.One },
        ColliderType.Capsule => new LSCapsuleCollider { Radius = Fixed64.One, Size = new Vector3d(2, 4, 2) },
        ColliderType.Cylinder => new LSCylinderCollider { Radius = (Fixed64)2, Size = new Vector3d(4, 2, 4) },
        ColliderType.Cone => new LSConeCollider { Radius = (Fixed64)2, Size = new Vector3d(4, 2, 4) },
        _ => throw new ArgumentOutOfRangeException(nameof(shape))
    };

    private static void AddSeparatedInvariantSupportBlade(PhysicsScenarioBuilder scenario)
    {
        ScenarioBody<LSCuboidCollider> blade = scenario.CreateBody(
            new LSCuboidCollider { Size = new Vector3d((Fixed64)6, Fixed64.One, Fixed64.FromFraction(1, 5)) },
            Vector3d.Up, PhysicsScenarioBuilder.Yaw(-5), isKinematic: true);
        blade.Body.UseManualGrounding();
        blade.Body.ContinuousCollisionMode = ContinuousCollisionMode.Discrete;
        blade.Body.Agent.Transform.LocalRotation = PhysicsScenarioBuilder.Yaw(5);
        // Its broad radius admits the rotational arbiter, but its bottom stays
        // at 1/2, above every requested sphere pose in this fixture.
    }
}
