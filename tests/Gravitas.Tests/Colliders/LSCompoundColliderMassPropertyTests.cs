using FixedMathSharp;
using FluentAssertions;
using Gravitas.Colliders;
using Gravitas.Queries;
using Gravitas.Support;
using Gravitas.Tests.Support;
using Xunit;

namespace Gravitas.Tests.Colliders;

public sealed class LSCompoundColliderMassPropertyTests
{
    [Fact]
    public void MassProperties_ShouldWeightSolidPartsByVolume()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        var compound = new LSCompoundCollider(
            CompoundColliderPart.Sphere(Fixed64.One, Vector3d.Zero),
            CompoundColliderPart.Sphere((Fixed64)2, new Vector3d((Fixed64)9, Fixed64.Zero, Fixed64.Zero)));
        scenario.CreateBody(compound, Vector3d.Zero, FixedQuaternion.Identity, mass: (Fixed64)9);
        Vector3d expectedCenter = new((Fixed64)8, Fixed64.Zero, Fixed64.Zero);
        Fixed3x3 expectedTensor = new(
            Fixed64.FromFraction(66, 5), Fixed64.Zero, Fixed64.Zero,
            Fixed64.Zero, Fixed64.FromFraction(426, 5), Fixed64.Zero,
            Fixed64.Zero, Fixed64.Zero, Fixed64.FromFraction(426, 5));

        compound.CalculateLocalCenterOfMassOffset().Should().Be(expectedCenter);
        Fixed3x3 tensor = compound.CalculateInertiaTensor((Fixed64)9, expectedCenter);
        AssertNear(tensor.M11, expectedTensor.M11);
        AssertNear(tensor.M22, expectedTensor.M22);
        AssertNear(tensor.M33, expectedTensor.M33);
        tensor.M12.Should().Be(Fixed64.Zero);
        tensor.M13.Should().Be(Fixed64.Zero);
        tensor.M23.Should().Be(Fixed64.Zero);
    }

    [Fact]
    public void MassProperties_ShouldRotatePartOffsetsAndAnisotropicTensorsIntoOwnerSpace()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        FixedQuaternion partRotation = FixedQuaternion.FromEulerAnglesInDegrees(
            Fixed64.Zero,
            Fixed64.Zero,
            (Fixed64)90);
        var compound = new LSCompoundCollider(
            new CompoundColliderPart(
                ColliderShapeDefinition.Cuboid(new Vector3d((Fixed64)2, (Fixed64)4, (Fixed64)6)),
                new Vector3d((Fixed64)2, Fixed64.Zero, Fixed64.Zero),
                partRotation));
        compound.LocalOffset = new Vector3d((Fixed64)3, Fixed64.Zero, Fixed64.Zero);
        scenario.CreateBody(compound, new Vector3d((Fixed64)10, Fixed64.Zero, Fixed64.Zero), FixedQuaternion.Identity);
        Vector3d expectedCenter = new((Fixed64)3, (Fixed64)2, Fixed64.Zero);
        Fixed3x3 expectedTensor = new(
            (Fixed64)40, Fixed64.Zero, Fixed64.Zero,
            Fixed64.Zero, (Fixed64)52, Fixed64.Zero,
            Fixed64.Zero, Fixed64.Zero, (Fixed64)20);

        compound.Center.Should().Be(new Vector3d((Fixed64)13, Fixed64.Zero, Fixed64.Zero));
        compound.GetPartCollider(0).Center.Should().Be(new Vector3d((Fixed64)13, (Fixed64)2, Fixed64.Zero));
        compound.CalculateLocalCenterOfMassOffset().Should().Be(expectedCenter);
        compound.CalculateInertiaTensor((Fixed64)12, expectedCenter).Should().Be(expectedTensor);
        compound.ScaledSize.Should().Be(Vector3d.One);
    }

    [Fact]
    public void MassProperties_WhenAllVolumesQuantizeToZero_ShouldConserveMassInAuthoredOrder()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        Fixed64 tinyRadius = Fixed64.FromRaw(1);
        var compound = new LSCompoundCollider(
            CompoundColliderPart.Sphere(tinyRadius, new Vector3d(-Fixed64.One, Fixed64.Zero, Fixed64.Zero)),
            CompoundColliderPart.Sphere(tinyRadius, Vector3d.Zero),
            CompoundColliderPart.Sphere(tinyRadius, new Vector3d((Fixed64)4, Fixed64.Zero, Fixed64.Zero)));
        scenario.CreateBody(compound, Vector3d.Zero, FixedQuaternion.Identity);
        Fixed64 mass = Fixed64.One;
        Vector3d center = compound.CalculateLocalCenterOfMassOffset();
        Fixed64 firstMass = mass / (Fixed64)3;
        Fixed64 lastMass = mass - firstMass - firstMass;
        Fixed64 expectedTransverse = firstMass * (center.X + Fixed64.One) * (center.X + Fixed64.One)
            + firstMass * center.X * center.X
            + lastMass * ((Fixed64)4 - center.X) * ((Fixed64)4 - center.X);
        Fixed3x3 expected = new(
            Fixed64.Zero, Fixed64.Zero, Fixed64.Zero,
            Fixed64.Zero, expectedTransverse, Fixed64.Zero,
            Fixed64.Zero, Fixed64.Zero, expectedTransverse);

        center.Should().Be(new Vector3d(Fixed64.One, Fixed64.Zero, Fixed64.Zero));
        compound.CalculateInertiaTensor(mass, center).Should().Be(expected);
    }

    [Fact]
    public void MassProperties_ShouldAssignResidualToLastPositiveWeightBeforeTrailingZeroParts()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        var compound = new LSCompoundCollider(
            CompoundColliderPart.Sphere(Fixed64.One, Vector3d.Zero),
            CompoundColliderPart.Sphere(
                Fixed64.FromRaw(1),
                new Vector3d((Fixed64)1_000_000, Fixed64.Zero, Fixed64.Zero)));
        scenario.CreateBody(compound, Vector3d.Zero, FixedQuaternion.Identity);

        compound.CalculateLocalCenterOfMassOffset().Should().Be(Vector3d.Zero);
        compound.CalculateInertiaTensor(Fixed64.One, Vector3d.Zero).Should().Be(new Fixed3x3(
            Fixed64.FromFraction(2, 5), Fixed64.Zero, Fixed64.Zero,
            Fixed64.Zero, Fixed64.FromFraction(2, 5), Fixed64.Zero,
            Fixed64.Zero, Fixed64.Zero, Fixed64.FromFraction(2, 5)));
    }

    [Fact]
    public void OffCenterPart_ShouldKeepConservativeRadiusAndPublicQueryVisibility()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        var compound = new LSCompoundCollider(
            CompoundColliderPart.Sphere(Fixed64.Half, new Vector3d((Fixed64)10, Fixed64.Zero, Fixed64.Zero)));
        scenario.CreateBody(compound, Vector3d.Zero, FixedQuaternion.Identity);

        Vector3d farthestBoundsCorner = new(
            Fixed64.FromFraction(21, 2),
            Fixed64.Half,
            Fixed64.Half);
        compound.ScaledRadius.Should().Be(farthestBoundsCorner.Magnitude);
        bool found = scenario.Context.Query3D.OverlapCircle(
            new Vector3d((Fixed64)10, Fixed64.Zero, Fixed64.Zero),
            Fixed64.Half,
            out Physics3DHit hit,
            PhysicsLayerMask.FromLayer(0));

        found.Should().BeTrue();
        hit.Collider.Should().BeSameAs(compound);
    }

    [Fact]
    public void GeometryReducers_ShouldPreserveFirstAuthoredTieAndAggregateCapsuleProjection()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        var tied = new LSCompoundCollider(
            CompoundColliderPart.Sphere(Fixed64.Half, new Vector3d((Fixed64)(-4), Fixed64.Zero, Fixed64.Zero)),
            CompoundColliderPart.Sphere(Fixed64.Half, new Vector3d((Fixed64)2, Fixed64.Zero, Fixed64.Zero)));
        scenario.CreateBody(tied, Vector3d.Zero, FixedQuaternion.Identity);
        Vector3d tiePoint = new(-Fixed64.One, Fixed64.Zero, Fixed64.Zero);

        tied.ClosestPointOnSurface(tiePoint)
            .Should().Be(new Vector3d(-Fixed64.FromFraction(7, 2), Fixed64.Zero, Fixed64.Zero));
        tied.GetNormalAtPoint(tiePoint).Should().Be(Vector3d.Right);

        var projected = new LSCompoundCollider(
            CompoundColliderPart.Capsule(Fixed64.One, (Fixed64)4, Vector3d.Zero),
            CompoundColliderPart.Sphere(Fixed64.One, new Vector3d((Fixed64)7, Fixed64.Zero, Fixed64.Zero)));
        scenario.CreateBody(projected, Vector3d.Zero, FixedQuaternion.Identity);

        projected.CalculateLocalCenterOfMassOffset()
            .Should().Be(new Vector3d((Fixed64)2, Fixed64.Zero, Fixed64.Zero));
        projected.GetFrontalArea(Vector3d.Up).Should().Be((Fixed64)2 * Fixed64.Pi);
        projected.GetFrontalArea(Vector3d.Right).Should().Be((Fixed64)2 * Fixed64.Pi + (Fixed64)4);
        projected.GetFrontalArea(Vector3d.Zero).Should().Be(projected.Area);
    }

    [Fact]
    public void MassPropertyWeights_ShouldUseSolidVolumeAndExplicitShellPolicy()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ColliderShapeDefinition closedCube = MeshTestFixtures.CreateConvexCubeDefinition();
        ColliderShapeDefinition shellCube = MeshTestFixtures.CreateConvexCubeDefinition(
            MeshInertiaPolicy.SurfaceApproximation);
        var compound = new LSCompoundCollider(
            CompoundColliderPart.Sphere(Fixed64.One, Vector3d.Zero),
            CompoundColliderPart.Capsule(Fixed64.One, (Fixed64)4, Vector3d.Zero),
            CompoundColliderPart.Cuboid(new Vector3d((Fixed64)2, (Fixed64)3, (Fixed64)4), Vector3d.Zero),
            CompoundColliderPart.Cylinder(Fixed64.One, (Fixed64)3, Vector3d.Zero),
            CompoundColliderPart.Cone(Fixed64.One, (Fixed64)3, Vector3d.Zero),
            new CompoundColliderPart(closedCube),
            new CompoundColliderPart(shellCube));
        scenario.CreateBody(compound, Vector3d.Zero, FixedQuaternion.Identity);
        var sphere = (LSSphereCollider)compound.GetPartCollider(0);
        var capsule = (LSCapsuleCollider)compound.GetPartCollider(1);
        var box = (LSCuboidCollider)compound.GetPartCollider(2);
        var cylinder = (LSCylinderCollider)compound.GetPartCollider(3);
        var cone = (LSConeCollider)compound.GetPartCollider(4);
        var closedMesh = (LSMeshCollider)compound.GetPartCollider(5);
        var shellMesh = (LSMeshCollider)compound.GetPartCollider(6);

        sphere.CalculateMassPropertyWeight().Should().Be(
            Fixed64.FromFraction(4, 3) * Fixed64.Pi * sphere.ScaledRadiusSqr * sphere.ScaledRadius);
        capsule.CalculateMassPropertyWeight().Should().Be(
            Fixed64.Pi * capsule.ScaledRadiusSqr * capsule.CylinderHeight
            + Fixed64.FromFraction(4, 3) * Fixed64.Pi * capsule.ScaledRadiusSqr * capsule.ScaledRadius);
        box.CalculateMassPropertyWeight().Should().Be((Fixed64)24);
        cylinder.CalculateMassPropertyWeight().Should().Be(
            Fixed64.Pi * cylinder.ScaledRadiusSqr * cylinder.Height);
        cone.CalculateMassPropertyWeight().Should().Be(cone.Volume);
        closedMesh.Mesh.TryGetClosedVolumeMassProperties(out MeshMassProperties closedProperties, out _)
            .Should().BeTrue();
        closedMesh.CalculateMassPropertyWeight().Should().Be(closedProperties.Volume);
        shellMesh.CalculateMassPropertyWeight().Should().Be(shellMesh.Mesh.TotalArea);
        shellMesh.CalculateMassPropertyWeight().Should().NotBe(closedMesh.CalculateMassPropertyWeight());
    }

    [Fact]
    public void OpenRequireClosedMeshPart_ShouldExposeZeroMassWeightWithoutForcingAngularInertia()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        Vector3d[] vertices =
        {
            new(-Fixed64.One, Fixed64.Zero, -Fixed64.One),
            new(Fixed64.One, Fixed64.Zero, -Fixed64.One),
            new(-Fixed64.One, Fixed64.Zero, Fixed64.One),
            new(Fixed64.One, Fixed64.Zero, Fixed64.One)
        };
        int[] triangles = { 0, 2, 1, 1, 2, 3 };
        var compound = new LSCompoundCollider(
            CompoundColliderPart.ConvexMesh(
                vertices,
                triangles,
                Vector3d.Zero,
                MeshInertiaPolicy.RequireClosedVolume));
        scenario.CreateBody(
            compound,
            Vector3d.Zero,
            FixedQuaternion.Identity,
            preventAngularForces: true);
        var mesh = (LSMeshCollider)compound.GetPartCollider(0);

        mesh.CalculateMassPropertyWeight().Should().Be(Fixed64.Zero);
        compound.CalculateMassPropertyWeight().Should().Be(Fixed64.Zero);
        compound.CalculateLocalCenterOfMassOffset().Should().Be(Vector3d.Zero);
    }

    [Fact]
    public void QuantizedConePart_ShouldUseDeterministicDegenerateSideProjection()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        Vector3d tinyScale = new(
            Fixed64.FromRaw(1),
            Fixed64.FromRaw(1),
            Fixed64.FromRaw(1));
        var compound = new LSCompoundCollider(
            new CompoundColliderPart(
                ColliderShapeDefinition.Cone(Fixed64.One, Fixed64.One),
                Vector3d.Zero,
                FixedQuaternion.Identity,
                tinyScale));
        scenario.CreateBody(compound, Vector3d.Zero, FixedQuaternion.Identity);
        var cone = (LSConeCollider)compound.GetPartCollider(0);

        cone.CalculateMassPropertyWeight().Should().Be(Fixed64.Zero);
        cone.ClosestPointOnSurface(Vector3d.Right)
            .Should().Be(new Vector3d(Fixed64.FromRaw(1), Fixed64.Zero, Fixed64.Zero));
    }

    [Fact]
    public void ClosedMeshPartCenterOfMass_ShouldTransformEntireLocalPointIntoOwnerSpace()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        Vector3d[] vertices =
        {
            Vector3d.Zero,
            Vector3d.Right,
            Vector3d.Up,
            Vector3d.Forward
        };
        int[] triangles =
        {
            1, 2, 3,
            0, 2, 1,
            0, 1, 3,
            0, 3, 2
        };
        FixedQuaternion partRotation = FixedQuaternion.FromEulerAnglesInDegrees(
            Fixed64.Zero,
            Fixed64.Zero,
            (Fixed64)90);
        var compound = new LSCompoundCollider(
            CompoundColliderPart.ConvexMesh(
                vertices,
                triangles,
                new Vector3d((Fixed64)2, Fixed64.Zero, Fixed64.Zero),
                partRotation,
                Vector3d.One));
        compound.LocalOffset = Vector3d.Right;
        scenario.CreateBody(compound, Vector3d.Zero, FixedQuaternion.Identity);
        var mesh = (LSMeshCollider)compound.GetPartCollider(0);
        Vector3d localMeshCenter = new(
            Fixed64.FromFraction(7, 4),
            -Fixed64.FromFraction(1, 4),
            -Fixed64.FromFraction(1, 4));
        Vector3d expectedCenter = Vector3d.Right + partRotation * localMeshCenter;

        AssertVectorNear(mesh.CalculateLocalCenterOfMassOffset(), expectedCenter);
        AssertVectorNear(compound.CalculateLocalCenterOfMassOffset(), expectedCenter);
        Fixed3x3 tensor = compound.CalculateInertiaTensor(Fixed64.One, expectedCenter);
        tensor.M12.Should().Be(tensor.M21);
        tensor.M13.Should().Be(tensor.M31);
        tensor.M23.Should().Be(tensor.M32);
    }

    private static void AssertNear(Fixed64 actual, Fixed64 expected) =>
        FixedMath.Abs(actual - expected)
            .Should().BeLessThanOrEqualTo(Fixed64.Epsilon * (Fixed64)512);

    private static void AssertVectorNear(Vector3d actual, Vector3d expected)
    {
        AssertNear(actual.X, expected.X);
        AssertNear(actual.Y, expected.Y);
        AssertNear(actual.Z, expected.Z);
    }
}
