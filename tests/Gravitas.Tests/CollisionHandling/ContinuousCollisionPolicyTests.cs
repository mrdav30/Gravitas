using FixedMathSharp;
using FluentAssertions;
using Gravitas.Colliders;
using Gravitas.CollisionHandling;
using Gravitas.Materials;
using Gravitas.Queries;
using Gravitas.Support;
using Gravitas.Tests.Support;
using System;
using Xunit;

namespace Gravitas.Tests.CollisionHandlingTests;

public sealed class ContinuousCollisionPolicyTests
{
    [Theory]
    [InlineData(false, false, false)]
    [InlineData(false, false, true)]
    [InlineData(false, true, false)]
    [InlineData(false, true, true)]
    [InlineData(true, false, false)]
    [InlineData(true, false, true)]
    [InlineData(true, true, false)]
    [InlineData(true, true, true)]
    public void ContainedFiniteAxisSweep_ShouldKeepOutwardNormalIndependentOfTravel(
        bool cone, bool compound, bool rotated)
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        // This exact unit quaternion permutes axes without changing the shape
        // through rounded trigonometry. Test a translated target as well.
        FixedQuaternion rotation = rotated
            ? new(Fixed64.Half, Fixed64.Half, Fixed64.Half, Fixed64.Half)
            : FixedQuaternion.Identity;
        Vector3d origin = new(3, -2, 1);
        LSCollider target = CreateFiniteAxisTarget(cone, compound);
        scenario.CreateBody(target, origin, rotation, immovable: true);
        Vector3d localCenter = new(Fixed64.FromFraction(1, 10),
            cone ? -Fixed64.FromFraction(1, 10) : Fixed64.Zero, Fixed64.Zero);
        Vector3d center = origin + rotation.Rotate(localCenter);
        // The cylinder side points Right; the cone side 2*x+y=1/2 has
        // outward gradient (2,1,0), even for a center inside the solid.
        Vector3d expected = rotation.Rotate(cone ? new Vector3d(2, 1, 0).Normalized : Vector3d.Right);
        foreach (Vector3d direction in new[] { expected, -expected })
        {
            scenario.Context.Query3D.SweepSphere(center, Fixed64.FromFraction(1, 32),
                direction, Fixed64.One, out Physics3DHit hit, PhysicsLayerMask.FromLayer(0)).Should().BeTrue();
            hit.Collider.Should().BeSameAs(target);
            hit.Distance.Should().Be(Fixed64.Zero);
            Vector3d.Dot(hit.Normal, expected).Should().BeGreaterThan(Fixed64.FromFraction(99999, 100000));
            Vector3d.Dot(hit.Point - center, expected).Should().BeGreaterThan(Fixed64.Zero);
        }

        for (int i = 0; i < 32; i++)
            scenario.Context.Query3D.SweepSphere(center, Fixed64.FromFraction(1, 32),
                expected, Fixed64.One, out _, PhysicsLayerMask.FromLayer(0));
        long before = GC.GetAllocatedBytesForCurrentThread();
        int hits = 0;
        for (int i = 0; i < 128; i++)
            if (scenario.Context.Query3D.SweepSphere(center, Fixed64.FromFraction(1, 32),
                    expected, Fixed64.One, out _, PhysicsLayerMask.FromLayer(0))) hits++;
        long allocated = GC.GetAllocatedBytesForCurrentThread() - before;
        hits.Should().Be(128);
        allocated.Should().Be(0);
    }

    [Theory]
    [InlineData(false, false, false)]
    [InlineData(false, false, true)]
    [InlineData(false, true, false)]
    [InlineData(false, true, true)]
    [InlineData(true, false, false)]
    [InlineData(true, false, true)]
    [InlineData(true, true, false)]
    [InlineData(true, true, true)]
    public void ContainedFiniteAxisCcd_FullLoopShouldAllowEscapeAndBlockDeeperMotion(
        bool cone, bool compound, bool escaping)
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        scenario.Context.SetFrameRate(1);
        scenario.Context.Environment.Gravity = Fixed64.Zero;
        scenario.Context.Environment.AirDensity = Fixed64.Zero;
        scenario.Context.Environment.DampingFactor = Fixed64.Zero;
        LSCollider target = CreateFiniteAxisTarget(cone, compound);
        target.Material = PhysicsMaterial.Frictionless;
        scenario.CreateBody(target, Vector3d.Zero, FixedQuaternion.Identity, immovable: true);
        Vector3d start = new(Fixed64.FromFraction(1, 10),
            cone ? -Fixed64.FromFraction(1, 10) : Fixed64.Zero, Fixed64.Zero);
        SolidBody source = scenario.CreateBody(new LSSphereCollider { Radius = Fixed64.FromFraction(1, 32) },
            start, FixedQuaternion.Identity, preventAngularForces: true).Body;
        source.Collider.Material = PhysicsMaterial.Frictionless;
        source.UseManualGrounding();
        source.ContinuousCollisionMode = ContinuousCollisionMode.Continuous;
        Vector3d outward = cone ? new Vector3d(2, 1, 0).Normalized : Vector3d.Right;
        Vector3d velocity = escaping ? Vector3d.Right * 4 : -outward;
        source.AddLinearImpulse(velocity);
        scenario.Context.Simulate();
        scenario.Context.LateSimulate();

        if (escaping)
        {
            source.Position3d.Should().Be(start + velocity);
            source.LinearVelocity.Should().Be(velocity);
            source.LastContinuousCollisionToiIterationCount.Should().Be(0);
        }
        else
        {
            source.LastContinuousCollisionToiIterationCount.Should().BeGreaterThan(0);
            Vector3d.Dot(source.LinearVelocity, outward).Should().BeGreaterThanOrEqualTo(-Fixed64.Epsilon);
            Vector3d.Dot(source.Position3d - start, outward).Should().BeGreaterThanOrEqualTo(-Fixed64.Epsilon);
        }
    }

    [Fact]
    public void MeshSphereContact_WithUnrepresentableCenterOffset_ShouldRetainGeometricSide()
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        LSMeshCollider mesh = MeshTestFixtures.CreateConvexQuadFloor(MeshColliderMode.Concave);
        Vector3d surface = new(Fixed64.Zero, Fixed64.MinValue + (Fixed64)4, Fixed64.Zero);
        mesh.InitializeWithNoBody(new TestMatterAgent(
            context, new FixedTransform(surface, FixedQuaternion.Identity, Vector3d.One)));
        Vector3d center = new(Fixed64.Zero, Fixed64.MaxValue - (Fixed64)4, Fixed64.Zero);

        ContinuousCollisionContactPolicy.TryResolveSweptSphereContact(
            mesh, center, Vector3d.Up, out ContactAnchor anchor, out Vector3d normal)
            .Should().BeTrue();

        anchor.TryGetWorldPoint(out Vector3d point).Should().BeTrue();
        point.Should().Be(surface);
        normal.Should().Be(Vector3d.Up);
    }

    [Fact]
    public void ContinuousCollisionContactPolicy_ShouldResolveSweptSphereAnchorsAndNormals()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        LSSphereCollider sphere = scenario.CreateStaticSphere(Vector3d.Zero);
        LSCuboidCollider cuboid = scenario.CreateCuboid(Vector3d.Zero).Collider;

        ContinuousCollisionContactPolicy.TryResolveSweptSphereContact(
                sphere,
                sphere.Center,
                Vector3d.Right,
                out ContactAnchor coincidentAnchor,
                out Vector3d coincidentNormal)
            .Should().BeTrue();
        coincidentAnchor.TryGetWorldPoint(out Vector3d coincidentPoint).Should().BeTrue();
        coincidentPoint.Should().Be(sphere.Center - Vector3d.Right * sphere.ScaledRadius);
        coincidentNormal.Should().Be(Vector3d.Left);

        ContinuousCollisionContactPolicy.TryResolveSweptSphereContact(
                sphere,
                Vector3d.Right * (Fixed64)2,
                -Vector3d.Right,
                out ContactAnchor separatedAnchor,
                out Vector3d separatedNormal)
            .Should().BeTrue();
        separatedAnchor.TryGetWorldPoint(out Vector3d separatedPoint).Should().BeTrue();
        separatedPoint.Should().Be(Vector3d.Right * sphere.ScaledRadius);
        separatedNormal.Should().Be(Vector3d.Right);

        ContinuousCollisionContactPolicy.TryResolveSweptSphereContact(
                cuboid,
                Vector3d.Right,
                -Vector3d.Right,
                out _,
                out Vector3d cuboidNormal)
            .Should().BeTrue();
        cuboidNormal.Should().Be(Vector3d.Right);

        ContinuousCollisionContactPolicy.TryResolveSweptSphereContact(
                sphere,
                sphere.Center,
                -Vector3d.Up,
                out _,
                out Vector3d upwardFallback)
            .Should().BeTrue();
        upwardFallback.Should().Be(Vector3d.Up);

        ContinuousCollisionContactPolicy.TryResolveSweptSphereContact(
                sphere,
                sphere.Center,
                Vector3d.Zero,
                out _,
                out Vector3d zeroMotionFallback)
            .Should().BeTrue();
        zeroMotionFallback.Should().Be(Vector3d.Right);
    }

    [Fact]
    public void ContinuousCollisionContactPolicy_ShouldRejectUnrepresentableFiniteAxisWitnesses()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        LSCylinderCollider cylinder =
            scenario.CreateCylinder(Vector3d.Zero).Collider;
        LSConeCollider cone = scenario.CreateBody(
            new LSConeCollider(),
            Vector3d.Zero,
            FixedQuaternion.Identity).Collider;
        Vector3d farCenter = new(
            Fixed64.MinValue + Fixed64.One,
            Fixed64.Zero,
            Fixed64.Zero);
        Vector3d impact = new(
            Fixed64.MaxValue - Fixed64.One,
            Fixed64.Zero,
            Fixed64.Zero);
        cylinder.LocalOffset = farCenter;
        cone.LocalOffset = farCenter;
        cylinder.RebuildRuntimeShapeOnly().Should().BeTrue();
        cone.RebuildRuntimeShapeOnly().Should().BeTrue();

        ContinuousCollisionContactPolicy.TryResolveSweptSphereContact(
            cylinder,
            impact,
            Vector3d.Right,
            out ContactAnchor cylinderAnchor,
            out Vector3d cylinderNormal).Should().BeFalse();
        ContinuousCollisionContactPolicy.TryResolveSweptSphereContact(
            cone,
            impact,
            Vector3d.Right,
            out ContactAnchor coneAnchor,
            out Vector3d coneNormal).Should().BeFalse();

        cylinderAnchor.Should().Be(default(ContactAnchor));
        cylinderNormal.Should().Be(Vector3d.Zero);
        coneAnchor.Should().Be(default(ContactAnchor));
        coneNormal.Should().Be(Vector3d.Zero);
    }

    [Fact]
    public void CuboidContactPolicy_ShouldFallBackToExactFaceNormalWhenAnchorOffsetIsUnrepresentable()
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        var cuboid = new LSCuboidCollider { Size = Vector3d.One };
        Vector3d center = new(
            Fixed64.MaxValue - Fixed64.One,
            Fixed64.Zero,
            Fixed64.Zero);
        cuboid.InitializeWithNoBody(new TestMatterAgent(
            context,
            new FixedTransform(
                center,
                FixedQuaternion.Identity,
                Vector3d.One)));
        Vector3d impact = new(
            Fixed64.MinValue + Fixed64.One,
            Fixed64.Zero,
            Fixed64.Zero);

        ContinuousCollisionContactPolicy.TryResolveSweptSphereContact(
                cuboid,
                impact,
                Vector3d.Right,
                out ContactAnchor anchor,
                out Vector3d normal)
            .Should().BeTrue();

        anchor.TryGetWorldPoint(out _).Should().BeTrue();
        normal.Should().Be(Vector3d.Left);
    }

    [Fact]
    public void ContinuousCollisionContactPolicy_ShouldStabilizeCustomColliderNormals()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        var custom = new UnsupportedTestCollider3D
        {
            ClosestPointOverride = Vector3d.Left,
            NormalOverride = Vector3d.Zero,
        };
        scenario.InitializeStaticCollider(custom, Vector3d.Zero);

        custom.NormalOverride = Vector3d.Up * (Fixed64)2;
        ContinuousCollisionContactPolicy.TryResolveSweptSphereContact(
            custom, Vector3d.Right, Vector3d.Up, out _, out Vector3d customNormal)
            .Should().BeTrue();
        customNormal.Should().Be(Vector3d.Up);
        custom.NormalOverride = Vector3d.Zero;

        ContinuousCollisionContactPolicy.TryResolveSweptSphereContact(
            custom,
            Vector3d.Right,
            Vector3d.Up,
            out _,
            out Vector3d geometricNormal).Should().BeTrue();
        geometricNormal.Should().Be(Vector3d.Right);

        custom.ClosestPointOverride = Vector3d.Zero;
        ContinuousCollisionContactPolicy.TryResolveSweptSphereContact(
            custom,
            Vector3d.Zero,
            Vector3d.Up,
            out _,
            out Vector3d motionNormal).Should().BeTrue();
        motionNormal.Should().Be(Vector3d.Down);
        ContinuousCollisionContactPolicy.TryResolveSweptSphereContact(
            custom,
            Vector3d.Zero,
            Vector3d.Zero,
            out _,
            out Vector3d zeroNormal).Should().BeTrue();
        zeroNormal.Should().Be(Vector3d.Zero);

        custom.LocalOffset = new Vector3d(
            Fixed64.MinValue + Fixed64.One,
            Fixed64.Zero,
            Fixed64.Zero);
        custom.ClosestPointOverride = new Vector3d(
            Fixed64.MaxValue - Fixed64.One,
            Fixed64.Zero,
            Fixed64.Zero);
        custom.RebuildRuntimeShapeOnly().Should().BeTrue();
        ContinuousCollisionContactPolicy.TryResolveSweptSphereContact(
            custom,
            custom.ClosestPointOverride.Value,
            Vector3d.Right,
            out ContactAnchor unavailable,
            out Vector3d unavailableNormal).Should().BeFalse();
        unavailable.Should().Be(default(ContactAnchor));
        unavailableNormal.Should().Be(Vector3d.Zero);
    }

    [Fact]
    public void ContinuousCollisionImpulsePolicy_ShouldResolveNormalsFromHitOrSourceMotion()
    {
        ContinuousCollisionImpulsePolicy.TryResolveSourceNormal(
            Vector3d.Right * (Fixed64)2,
            Vector3d.Up,
            out Vector3d normal3D).Should().BeTrue();
        normal3D.Should().Be(Vector3d.Right);

        ContinuousCollisionImpulsePolicy.TryResolveSourceNormal(
            Vector3d.Zero,
            Vector3d.Right * (Fixed64)2,
            out normal3D).Should().BeTrue();
        normal3D.Should().Be(-Vector3d.Right);

        ContinuousCollisionImpulsePolicy.TryResolveSourceNormal(
            Vector3d.Zero,
            Vector3d.Zero,
            out normal3D).Should().BeFalse();
        normal3D.Should().Be(Vector3d.Zero);

        ContinuousCollisionImpulsePolicy.TryResolveImpactNormal(
            Vector3d.Up * (Fixed64)2,
            out normal3D).Should().BeTrue();
        normal3D.Should().Be(Vector3d.Up);

        ContinuousCollisionImpulsePolicy.TryResolveImpactNormal(
            Vector3d.Zero,
            out normal3D).Should().BeFalse();

        ContinuousCollisionImpulsePolicy.TryResolveSourceNormal(
            Vector2d.Right * (Fixed64)2,
            Vector2d.Forward,
            out Vector2d normal2D).Should().BeTrue();
        normal2D.Should().Be(Vector2d.Right);

        ContinuousCollisionImpulsePolicy.TryResolveSourceNormal(
            Vector2d.Zero,
            Vector2d.Right * (Fixed64)2,
            out normal2D).Should().BeTrue();
        normal2D.Should().Be(-Vector2d.Right);

        ContinuousCollisionImpulsePolicy.TryResolveSourceNormal(
            Vector2d.Zero,
            Vector2d.Zero,
            out normal2D).Should().BeFalse();

        ContinuousCollisionImpulsePolicy.TryResolveImpactNormal(
            Vector2d.Forward * (Fixed64)2,
            out normal2D).Should().BeTrue();
        normal2D.Should().Be(Vector2d.Forward);

        ContinuousCollisionImpulsePolicy.TryResolveImpactNormal(
            Vector2d.Zero,
            out normal2D).Should().BeFalse();
    }

    [Fact]
    public void TryResolveVelocityDelta_ShouldAcceptExactFiniteAndZeroMobility()
    {
        Fixed64 normalComponent = Fixed64.FromFraction(1, 65536);

        ContinuousCollisionImpulsePolicy.TryResolveVelocityDelta(
            new Vector2d(Fixed64.Zero, normalComponent),
            Fixed64.One,
            Fixed64.One,
            Fixed64.MinIncrement,
            out Vector2d delta2D).Should().BeTrue();
        ContinuousCollisionImpulsePolicy.TryResolveVelocityDelta(
            new Vector3d(Fixed64.Zero, normalComponent, Fixed64.Zero),
            Fixed64.One,
            Fixed64.One,
            Fixed64.MinIncrement,
            out Vector3d delta3D).Should().BeTrue();
        ContinuousCollisionImpulsePolicy.TryResolveVelocityDelta(
            Vector3d.One,
            Fixed64.MaxValue,
            Fixed64.Zero,
            Fixed64.Zero,
            out Vector3d zeroDelta).Should().BeTrue();
        ContinuousCollisionImpulsePolicy.TryResolveVelocityDelta(
            Vector2d.One,
            Fixed64.MaxValue,
            Fixed64.Zero,
            Fixed64.Zero,
            out Vector2d zeroDelta2D).Should().BeTrue();

        delta2D.Should().Be(new Vector2d(Fixed64.Zero, (Fixed64)65536));
        delta3D.Should().Be(new Vector3d(Fixed64.Zero, (Fixed64)65536, Fixed64.Zero));
        zeroDelta.Should().Be(Vector3d.Zero);
        zeroDelta2D.Should().Be(Vector2d.Zero);
    }

    [Fact]
    public void TryResolveVelocityDelta_ShouldRejectOnlyFinalOverflowAndRemainComponentAtomic()
    {
        ContinuousCollisionImpulsePolicy.TryResolveVelocityDelta(
            Vector2d.Right,
            Fixed64.Two,
            Fixed64.One,
            Fixed64.One,
            out Vector2d ordinaryDelta).Should().BeTrue();
        ContinuousCollisionImpulsePolicy.TryResolveVelocityDelta(
            new Vector2d(Fixed64.MinIncrement, Fixed64.One),
            Fixed64.Two,
            Fixed64.MaxValue,
            Fixed64.One,
            out Vector2d overflow2D).Should().BeFalse();
        ContinuousCollisionImpulsePolicy.TryResolveVelocityDelta(
            new Vector3d(Fixed64.MinIncrement, Fixed64.Zero, Fixed64.One),
            Fixed64.Two,
            Fixed64.MaxValue,
            Fixed64.One,
            out Vector3d overflow3D).Should().BeFalse();
        ContinuousCollisionImpulsePolicy.TryResolveVelocityDelta(
            Vector2d.One,
            Fixed64.Two,
            Fixed64.MaxValue,
            Fixed64.One,
            out _).Should().BeFalse();
        ContinuousCollisionImpulsePolicy.TryResolveVelocityDelta(
            new Vector3d(Fixed64.MinIncrement, Fixed64.One, Fixed64.Zero),
            Fixed64.Two,
            Fixed64.MaxValue,
            Fixed64.One,
            out _).Should().BeFalse();
        ContinuousCollisionImpulsePolicy.TryResolveVelocityDelta(
            Vector3d.One,
            Fixed64.Two,
            Fixed64.MaxValue,
            Fixed64.One,
            out _).Should().BeFalse();

        ordinaryDelta.Should().Be(Vector2d.Right * Fixed64.Two);
        overflow2D.Should().Be(default);
        overflow3D.Should().Be(default);
    }

    private static LSCollider CreateFiniteAxisTarget(bool cone, bool compound) => compound
        ? new LSCompoundCollider(cone
            ? CompoundColliderPart.Cone(Fixed64.Half, Fixed64.One, Vector3d.Zero)
            : CompoundColliderPart.Cylinder(Fixed64.Half, Fixed64.One, Vector3d.Zero))
        : cone ? new LSConeCollider() : new LSCylinderCollider();
}
