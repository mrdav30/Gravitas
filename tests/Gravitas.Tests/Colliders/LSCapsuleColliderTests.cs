using FixedMathSharp;
using FixedMathSharp.Geometry;
using FluentAssertions;
using Gravitas.Colliders;
using Gravitas.Tests.Support;
using System;
using Xunit;

namespace Gravitas.Tests.Colliders;

public sealed class LSCapsuleColliderTests
{
    [Fact]
    public void ReassigningCurrentDimensions_ShouldNotRebuildRuntimeGeometry()
    {
        using PhysicsScenarioBuilder scenario =
            PhysicsScenarioBuilder.Create();
        LSCapsuleCollider capsule = scenario.CreateCapsule(Vector3d.Zero)
            .Collider;
        uint runtimeVersion = capsule.RuntimeShapeVersion;

        capsule.Radius = capsule.Radius;
        capsule.Size = capsule.Size;
        capsule.Simulate();

        capsule.RuntimeShapeVersion.Should().Be(runtimeVersion);
    }

    [Fact]
    public void GetFrontalArea_ShouldUseCapsuleProjectionInWorldSpace()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        var capsule = scenario.CreateBody(
            new LSCapsuleCollider
            {
                Radius = Fixed64.One,
                Size = new Vector3d((Fixed64)2, (Fixed64)4, (Fixed64)2)
            },
            Vector3d.Zero,
            FixedQuaternion.Identity).Collider;
        var rotated = scenario.CreateBody(
            new LSCapsuleCollider
            {
                Radius = Fixed64.One,
                Size = new Vector3d((Fixed64)2, (Fixed64)4, (Fixed64)2)
            },
            new Vector3d((Fixed64)8, Fixed64.Zero, Fixed64.Zero),
            FixedQuaternion.FromEulerAnglesInDegrees(Fixed64.Zero, Fixed64.Zero, (Fixed64)90)).Collider;

        Fixed64 capArea = Fixed64.Pi * capsule.ScaledRadiusSqr;
        Fixed64 sideProfile = (Fixed64)2 * capsule.ScaledRadius * capsule.AxisLength;
        Vector3d diagonal = new Vector3d(Fixed64.One, Fixed64.One, Fixed64.Zero).Normalized;
        Fixed64 diagonalAxial = Vector3d.Dot(diagonal, capsule.WorldAxis).Abs();
        Fixed64 diagonalRadial = FixedMath.Sqrt(Fixed64.One - diagonalAxial * diagonalAxial);
        Vector3d fixedPointOverNormalizedAxis = new(
            Fixed64.FromRaw(1),
            Fixed64.FromRaw(100_000),
            Fixed64.Zero);

        capsule.GetFrontalArea(Vector3d.Zero).Should().Be(capsule.Area);
        capsule.GetFrontalArea(capsule.WorldAxis).Should().Be(capArea);
        capsule.GetFrontalArea(fixedPointOverNormalizedAxis).Should().Be(capArea);
        capsule.GetFrontalArea(Vector3d.Right).Should().Be(capArea + sideProfile);
        capsule.GetFrontalArea(diagonal).Should().Be(capArea + sideProfile * diagonalRadial);
        AssertNear(rotated.GetFrontalArea(Vector3d.Right), capArea);
        AssertNear(rotated.GetFrontalArea(Vector3d.Up), capArea + sideProfile);
    }

    [Fact]
    public void CalculateInertiaTensor_WithHeightEqualDiameter_ShouldMatchSphereLimit()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        var capsule = scenario.CreateBody(
            new LSCapsuleCollider
            {
                Radius = Fixed64.One,
                Size = new Vector3d((Fixed64)2, (Fixed64)2, (Fixed64)2)
            },
            Vector3d.Zero,
            FixedQuaternion.Identity).Collider;
        var sphere = scenario.CreateBody(
            new LSSphereCollider { Radius = Fixed64.One },
            new Vector3d((Fixed64)4, Fixed64.Zero, Fixed64.Zero),
            FixedQuaternion.Identity).Collider;
        Fixed64 mass = (Fixed64)5;
        Vector3d reference = new(Fixed64.One, (Fixed64)2, (Fixed64)3);

        capsule.AxisLength.Should().Be(Fixed64.Zero);
        capsule.CalculateInertiaTensor(mass, reference)
            .Should()
            .Be(sphere.CalculateInertiaTensor(mass, reference));
    }

    [Fact]
    public void CalculateInertiaTensor_ShouldIncludeHemisphereCentroidShift()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        var capsule = scenario.CreateBody(
            new LSCapsuleCollider
            {
                Radius = Fixed64.One,
                Size = new Vector3d((Fixed64)2, (Fixed64)4, (Fixed64)2)
            },
            Vector3d.Zero,
            FixedQuaternion.Identity).Collider;
        Fixed64 mass = (Fixed64)10;
        Fixed64 radiusSqr = capsule.ScaledRadiusSqr;
        Fixed64 height = capsule.AxisLength;
        Fixed64 cylinderVolume = Fixed64.Pi * radiusSqr * height;
        Fixed64 capVolume = Fixed64.FromFraction(4, 3) * Fixed64.Pi * radiusSqr * capsule.ScaledRadius;
        Fixed64 cylinderMass = mass * (cylinderVolume / (cylinderVolume + capVolume));
        Fixed64 capMass = mass - cylinderMass;
        Fixed64 d = height * Fixed64.Half;
        Fixed64 expectedTransverse =
            Fixed64.FromFraction(1, 12) * cylinderMass * ((Fixed64)3 * radiusSqr + height * height)
            + capMass * (Fixed64.FromFraction(2, 5) * radiusSqr
                + d * d
                + Fixed64.FromFraction(3, 4) * d * capsule.ScaledRadius);
        Fixed64 expectedAxial =
            Fixed64.Half * cylinderMass * radiusSqr
            + Fixed64.FromFraction(2, 5) * capMass * radiusSqr;

        Fixed3x3 tensor = capsule.CalculateInertiaTensor(mass, Vector3d.Zero);

        tensor.M11.Should().Be(expectedTransverse);
        tensor.M22.Should().Be(expectedAxial);
        tensor.M33.Should().Be(expectedTransverse);
        tensor.M12.Should().Be(Fixed64.Zero);
        tensor.M13.Should().Be(Fixed64.Zero);
        tensor.M23.Should().Be(Fixed64.Zero);
    }

    [Fact]
    public void CalculateInertiaTensor_WithQuantizedZeroVolume_ShouldUseThinRodLimit()
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        var compound = new LSCompoundCollider(
            CompoundColliderPart.Capsule(
                Fixed64.One,
                (Fixed64)4,
                Vector3d.Zero,
                FixedQuaternion.Identity,
                new Vector3d(
                    Fixed64.FromRaw(1),
                    Fixed64.One,
                    Fixed64.FromRaw(1))));
        compound.InitializeWithNoBody(new TestMatterAgent(context));
        var capsule = (LSCapsuleCollider)compound.GetPartCollider(0);
        Fixed64 mass = (Fixed64)12;

        Fixed3x3 tensor =
            capsule.CalculateInertiaTensor(mass, Vector3d.Zero);
        Fixed64 expectedTransverse =
            Fixed64.FromFraction(1, 12)
            * mass
            * capsule.AxisLength
            * capsule.AxisLength;

        capsule.ScaledRadius.Should().Be(Fixed64.FromRaw(1));
        capsule.AxisLength.Should().BeGreaterThan(Fixed64.Epsilon);
        tensor.M11.Should().Be(expectedTransverse);
        tensor.M22.Should().Be(Fixed64.Zero);
        tensor.M33.Should().Be(expectedTransverse);
    }

    [Fact]
    public void Initialize_WithQuantizedZeroFinalRadius_ShouldRejectWithoutPublishing()
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        Vector3d tinyScale = new(Fixed64.FromRaw(1), Fixed64.FromRaw(1), Fixed64.FromRaw(1));
        var compound = new LSCompoundCollider(
            CompoundColliderPart.Capsule(
                Fixed64.Half,
                (Fixed64)1_000_000_000,
                Vector3d.Zero,
                FixedQuaternion.Identity,
                tinyScale));
        var capsule = (LSCapsuleCollider)compound.GetPartCollider(0);
        var body = new SolidBody(new TestMatterAgent(context), compound);
        Fixed64 originalRadius = capsule.ScaledRadius;
        Fixed64 originalAxisLength = capsule.AxisLength;
        var originalBounds = capsule.Bounds;
        uint originalShapeVersion = capsule.RuntimeShapeVersion;

        Action initialize = () => body.Initialize(Vector3d.Zero, FixedQuaternion.Identity);

        initialize.Should().Throw<ArgumentException>().WithParameterName("dimension");
        body.Active.Should().BeFalse();
        body.DynamicId.Should().Be(-1);
        compound.Id.Should().Be(-1);
        capsule.ScaledRadius.Should().Be(originalRadius);
        capsule.AxisLength.Should().Be(originalAxisLength);
        capsule.Bounds.Should().Be(originalBounds);
        capsule.RuntimeShapeVersion.Should().Be(originalShapeVersion);
    }

    [Fact]
    public void Initialize_WhenScaledHeightIsShorterThanDiameter_ShouldRejectWithoutRegistration()
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        var transform = new FixedTransform(
            Vector3d.Zero,
            FixedQuaternion.Identity,
            new Vector3d(
                Fixed64.Two,
                Fixed64.One,
                Fixed64.One));
        var capsule = new LSCapsuleCollider
        {
            Radius = Fixed64.One,
            Size = new Vector3d(
                Fixed64.Two,
                Fixed64.Two,
                Fixed64.Two)
        };
        var body = new SolidBody(
            new TestMatterAgent(context, transform),
            capsule);

        Action initialize = () =>
            body.Initialize(Vector3d.Zero, FixedQuaternion.Identity);

        initialize.Should()
            .Throw<ArgumentException>()
            .WithParameterName("snapshot");
        body.Active.Should().BeFalse();
        capsule.Id.Should().Be(-1);
        capsule.HasHostBinding.Should().BeFalse();
        context.Physics.ColliderCount.Should().Be(0);
    }

    [Fact]
    public void GetNormalAtPoint_WithSubMagnitudeCapOffsets_ShouldUseRotatedAxialFallbacks()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        FixedQuaternion rotation = FixedQuaternion.FromEulerAnglesInDegrees(
            Fixed64.Zero,
            Fixed64.Zero,
            (Fixed64)90);
        var capsule = scenario.CreateBody(
            new LSCapsuleCollider
            {
                Radius = Fixed64.One,
                Size = new Vector3d((Fixed64)2, (Fixed64)4, (Fixed64)2)
            },
            Vector3d.Zero,
            rotation).Collider;
        FixedSegment.TryGetCenteredAxisEndpoint(
            capsule.Center,
            capsule.WorldAxis,
            capsule.AxisLength,
            positive: true,
            out Vector3d topCenter).Should().BeTrue();
        FixedSegment.TryGetCenteredAxisEndpoint(
            capsule.Center,
            capsule.WorldAxis,
            capsule.AxisLength,
            positive: false,
            out Vector3d bottomCenter).Should().BeTrue();
        Vector3d topPoint = topCenter + capsule.WorldAxis * Fixed64.FromRaw(1);
        Vector3d bottomPoint = bottomCenter - capsule.WorldAxis * Fixed64.FromRaw(1);

        capsule.GetNormalAtPoint(topPoint).Should().Be(rotation * Vector3d.Up);
        capsule.GetNormalAtPoint(bottomPoint).Should().Be(rotation * -Vector3d.Up);
    }

    private static void AssertNear(Fixed64 actual, Fixed64 expected) =>
        FixedMath.Abs(actual - expected).Should().BeLessThanOrEqualTo(Fixed64.Epsilon * (Fixed64)64);
}
