using FixedMathSharp;
using FluentAssertions;
using Gravitas.Colliders;
using Gravitas.Queries;
using Gravitas.Support;
using Gravitas.Tests.Support;
using SwiftCollections;
using System;
using Xunit;

namespace Gravitas.Tests.Queries;

public sealed class GravitasQuery3DServiceConeTests
{
    private static readonly PhysicsLayerMask IncludeLayerZero = PhysicsLayerMask.FromLayer(0);

    [Fact]
    public void OverlapCone_ShouldHitPrimitiveTargetsAndOrderAllHitsByAxialDistance()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        LSSphereCollider sphere = scenario.CreateSphere(new Vector3d((Fixed64)2, Fixed64.Zero, Fixed64.Zero)).Collider;
        LSCuboidCollider cuboid = scenario.CreateCuboid(new Vector3d((Fixed64)4, Fixed64.Half, Fixed64.Zero)).Collider;
        LSCapsuleCollider capsule = scenario.CreateCapsule(new Vector3d((Fixed64)6, Fixed64.One, Fixed64.Zero)).Collider;
        LSCylinderCollider cylinder = scenario.CreateCylinder(new Vector3d((Fixed64)8, Fixed64.One, Fixed64.Zero)).Collider;
        LSConeCollider cone = scenario.CreateBody(
            new LSConeCollider { Radius = Fixed64.Half, Size = new Vector3d(Fixed64.One, (Fixed64)2, Fixed64.One) },
            new Vector3d((Fixed64)10, Fixed64.One, Fixed64.Zero),
            FixedQuaternion.Identity).Collider;
        var hits = new SwiftList<Physics3DHit>();

        bool closest = scenario.Context.Query3D.OverlapCone(
            Vector3d.Zero,
            Vector3d.Right,
            (Fixed64)12,
            (Fixed64)3,
            out Physics3DHit closestHit,
            IncludeLayerZero);
        int count = scenario.Context.Query3D.OverlapConeAll(
            Vector3d.Zero,
            Vector3d.Right,
            (Fixed64)12,
            (Fixed64)3,
            IncludeLayerZero,
            hits);

        closest.Should().BeTrue();
        closestHit.Collider.Should().BeSameAs(sphere);
        count.Should().Be(5);
        hits[0].Collider.Should().BeSameAs(sphere);
        hits[1].Collider.Should().BeSameAs(cuboid);
        hits[2].Collider.Should().BeSameAs(capsule);
        hits[3].Collider.Should().BeSameAs(cylinder);
        hits[4].Collider.Should().BeSameAs(cone);
        hits.Should().OnlyContain(hit => hit.Distance >= Fixed64.Zero);
    }

    [Fact]
    public void OverlapCone_ShouldSupportMeshCompoundFilteringAndValidation()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        LSMeshCollider mesh = scenario.CreateBody(
            MeshTestFixtures.CreateConvexCube(inertiaPolicy: MeshInertiaPolicy.SurfaceApproximation),
            new Vector3d((Fixed64)2, Fixed64.Zero, Fixed64.Zero),
            FixedQuaternion.Identity).Collider;
        LSCompoundCollider compound = scenario.CreateBody(
            new LSCompoundCollider(CompoundColliderPart.Cone(Fixed64.Half, (Fixed64)2, Vector3d.Zero)),
            new Vector3d((Fixed64)4, Fixed64.Zero, Fixed64.Zero),
            FixedQuaternion.Identity).Collider;
        LSMeshCollider concaveMesh = scenario.CreateBody(
            new LSMeshCollider(
                new[]
                {
                    new Vector3d(Fixed64.One, Fixed64.Zero, Fixed64.One),
                    new Vector3d((Fixed64)6, Fixed64.Zero, Fixed64.One),
                    new Vector3d((Fixed64)6, (Fixed64)2, Fixed64.One)
                },
                new[] { 0, 1, 2 },
                MeshColliderMode.Concave,
                MeshInertiaPolicy.SurfaceApproximation),
            Vector3d.Zero,
            FixedQuaternion.Identity).Collider;
        LSSphereCollider trigger = scenario.CreateStaticSphere(new Vector3d((Fixed64)6, Fixed64.Zero, Fixed64.Zero));
        trigger.IsTrigger = true;
        LSSphereCollider masked = scenario.CreateSphere(new Vector3d((Fixed64)8, Fixed64.Zero, Fixed64.Zero)).Collider;
        masked.Layer = new PhysicsLayer(2);
        var hits = new SwiftList<Physics3DHit>();

        int count = scenario.Context.Query3D.OverlapConeAll(
            Vector3d.Zero,
            Vector3d.Right,
            (Fixed64)10,
            (Fixed64)2,
            IncludeLayerZero,
            hits);
        Action zeroDirection = () => scenario.Context.Query3D.OverlapCone(
            Vector3d.Zero,
            Vector3d.Zero,
            Fixed64.One,
            Fixed64.Half,
            out _,
            IncludeLayerZero);
        Action invalidLength = () => scenario.Context.Query3D.OverlapCone(
            Vector3d.Zero,
            Vector3d.Right,
            Fixed64.Zero,
            Fixed64.Half,
            out _,
            IncludeLayerZero);

        count.Should().Be(4);
        hits.Should().Contain(hit => ReferenceEquals(hit.Collider, mesh));
        hits.Should().Contain(hit => ReferenceEquals(hit.Collider, compound));
        hits.Should().Contain(hit => ReferenceEquals(hit.Collider, concaveMesh));
        hits.Should().Contain(hit => ReferenceEquals(hit.Collider, trigger));
        hits.Should().NotContain(hit => ReferenceEquals(hit.Collider, masked));
        zeroDirection.Should().Throw<ArgumentException>().WithParameterName("direction");
        invalidLength.Should().Throw<ArgumentException>().WithParameterName("length");
    }

    [Fact]
    public void OverlapConeAll_ShouldNotAllocateAfterWarmup()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        _ = scenario.CreateSphere(new Vector3d((Fixed64)2, Fixed64.Zero, Fixed64.Zero));
        _ = scenario.CreateCuboid(new Vector3d((Fixed64)4, Fixed64.Half, Fixed64.Zero));
        var hits = new SwiftList<Physics3DHit>(8);

        long allocatedBytes = AllocationTestHelper.MeasureSteadyState(() =>
            scenario.Context.Query3D.OverlapConeAll(
                Vector3d.Zero,
                Vector3d.Right,
                (Fixed64)6,
                (Fixed64)2,
                IncludeLayerZero,
                hits));

        allocatedBytes.Should().Be(0);
    }
}
