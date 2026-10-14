using System;
using System.Linq;
using FixedMathSharp;
using FluentAssertions;
using Gravitas.Colliders;
using Gravitas.Support;
using Gravitas.Tests.Support;
using Xunit;

namespace Gravitas.Tests.Colliders;

public sealed class LSCompoundColliderCacheTests
{
    [Fact]
    public void CompoundTensor_ShouldFollowMassReferenceOffsetScaleAndCommittedGeometry()
    {
        var part = CompoundColliderPart.Cuboid(Vector3d.One * (Fixed64)6, Vector3d.Right);
        var compound = new LSCompoundCollider(part,
            CompoundColliderPart.Cuboid(Vector3d.One * (Fixed64)6, -Vector3d.Right));
        Fixed3x3 centered = Tensor(12, 14, 14);
        compound.CalculateInertiaTensor(Fixed64.Two, Vector3d.Zero).Should().Be(centered);
        compound.CalculateCenterOfMassInertiaTensor(Fixed64.Two).Should().Be(centered);
        compound.CalculateInertiaTensor((Fixed64)4, Vector3d.Zero).Should().Be(Tensor(24, 28, 28));
        compound.CalculateInertiaTensor(Fixed64.Zero, Vector3d.Zero).Should().Be(Fixed3x3.Zero);
        new LSSphereCollider().CalculateInertiaTensor(-Fixed64.One, Vector3d.Zero).Should().Be(Fixed3x3.Zero);

        compound.CalculateLocalCenterOfMassOffset().Should().Be(Vector3d.Zero);
        compound.LocalOffset = new Vector3d((Fixed64)3, -Fixed64.One, Fixed64.Zero);
        compound.CalculateLocalCenterOfMassOffset().Should().Be(compound.LocalOffset);
        compound.CalculateInertiaTensor(Fixed64.Two, Vector3d.Zero).Should().Be(Tensor(14, 32, 34, 6));
        compound.CalculateInertiaTensor(Fixed64.Two, compound.LocalOffset).Should().Be(centered);

        using var context = GravitasWorldContext.CreateOwned();
        var transform = new FixedTransform(Vector3d.Zero, FixedQuaternion.Identity, Vector3d.One * Fixed64.Two);
        compound.InitializeWithNoBody(new TestMatterAgent(context, transform));
        Vector3d center = new((Fixed64)6, (Fixed64)(-2), Fixed64.Zero);
        compound.CalculateLocalCenterOfMassOffset().Should().Be(center);
        compound.CalculateInertiaTensor(Fixed64.Two, Vector3d.Zero).Should().Be(Tensor(56, 128, 136, 24));
        compound.CalculateInertiaTensor(Fixed64.Two, center).Should().Be(Tensor(48, 56, 56));

        transform.LocalPosition = new Vector3d((Fixed64)100, (Fixed64)200, (Fixed64)300);
        compound.Simulate();
        compound.CalculateInertiaTensor(Fixed64.Two, center).Should().Be(Tensor(48, 56, 56));
        transform.LocalScale = Vector3d.One * (Fixed64)3;
        compound.Simulate();
        compound.CalculateLocalCenterOfMassOffset().Should().Be(compound.LocalOffset * (Fixed64)3);
        compound.CalculateInertiaTensor(Fixed64.Two, compound.CalculateLocalCenterOfMassOffset())
            .Should().Be(Tensor(108, 126, 126));
    }

    [Theory]
    [InlineData(false, false)]
    [InlineData(false, true)]
    [InlineData(true, false)]
    public void MeshCompoundTensor_ShouldRetainCommittedIntegralsAcrossRejectedScaleAndRefreshAfterRecovery(
        bool surface, bool reverseWinding)
    {
        ColliderShapeDefinition source = MeshTestFixtures.CreateConvexCubeDefinition();
        Vector3d[] vertices = Enumerable.Range(0, source.MeshVertexCount).Select(source.GetMeshVertex).ToArray();
        int[] triangles = Enumerable.Range(0, source.MeshTriangleIndexCount).Select(source.GetMeshTriangleIndex).ToArray();
        if (reverseWinding)
            for (int i = 0; i < triangles.Length; i += 3)
                (triangles[i + 1], triangles[i + 2]) = (triangles[i + 2], triangles[i + 1]);
        MeshInertiaPolicy policy = surface ? MeshInertiaPolicy.SurfaceApproximation : MeshInertiaPolicy.RequireClosedVolume;
        var compound = new LSCompoundCollider(CompoundColliderPart.ConvexMesh(vertices, triangles, Vector3d.Zero, policy));
        using var context = GravitasWorldContext.CreateOwned();
        var transform = new FixedTransform(Vector3d.Zero, FixedQuaternion.Identity, Vector3d.One);
        compound.InitializeWithNoBody(new TestMatterAgent(context, transform));
        Fixed64 mass = (Fixed64)(surface ? 18 : 6);
        // Unit cube volume has I/m=1/6; a uniform six-face shell has I/m=5/18.
        int diagonal = surface ? 5 : 1;
        Fixed3x3 expected = Tensor(diagonal, diagonal, diagonal);
        compound.CalculateInertiaTensor(mass, Vector3d.Zero).Should().Be(expected);
        transform.LocalScale = new Vector3d(-Fixed64.One, Fixed64.One, Fixed64.One);
        Action rejected = compound.Simulate;
        rejected.Should().Throw<ArgumentException>();
        compound.CalculateInertiaTensor(mass, Vector3d.Zero).Should().Be(expected);
        transform.LocalScale = Vector3d.One * Fixed64.Two;
        compound.Simulate();
        expected = Tensor(4 * diagonal, 4 * diagonal, 4 * diagonal);
        compound.CalculateInertiaTensor(mass, Vector3d.Zero).Should().Be(expected);
        transform.LocalPosition = Vector3d.One;
        compound.Simulate();
        compound.CalculateInertiaTensor(mass, Vector3d.Zero).Should().Be(expected);
    }

    [Fact]
    public void StagedMeshIntegral_ShouldNotReplaceCommittedGeometryBeforePublication()
    {
        ColliderShapeDefinition source = MeshTestFixtures.CreateConvexCubeDefinition();
        var compound = new LSCompoundCollider(new CompoundColliderPart(source));
        var part = (LSMeshCollider)compound.GetPartCollider(0);
        part.Mesh.PrepareTransformation(Vector3d.Zero, FixedQuaternion.Identity, Vector3d.One * Fixed64.Two,
            Vector3d.One, MeshInertiaPolicy.RequireClosedVolume);
        part.CalculatePreparedMassPropertyWeight().TryGetMeasure(out Fixed64 stagedVolume).Should().BeTrue();
        stagedVolume.Should().Be((Fixed64)8);
        part.Mesh.OwnerScale.Should().Be(Vector3d.One);

        // The unpopulated current cache must not inherit the discarded scale-two
        // integrals. A subsequent pose candidate reuses the validated unit cube.
        compound.CalculateInertiaTensor((Fixed64)6, Vector3d.Zero).Should().Be(Tensor(1, 1, 1));
        part.Mesh.PrepareTransformation(Vector3d.One, FixedQuaternion.Identity, Vector3d.One,
            Vector3d.One, MeshInertiaPolicy.RequireClosedVolume);
        part.CalculatePreparedMassPropertyWeight().TryGetMeasure(out stagedVolume).Should().BeTrue();
        stagedVolume.Should().Be(Fixed64.One);
    }

    [Fact]
    public void CompoundTensor_ShouldUseSphereLimitForCollapsedCapsuleAndRejectCollapsedSphereDimensions()
    {
        var capsule = new LSCompoundCollider(CompoundColliderPart.Capsule(Fixed64.One, Fixed64.Two, Vector3d.Zero));
        Fixed3x3 expected = new LSSphereCollider { Radius = Fixed64.One }
            .CalculateInertiaTensor(Fixed64.One, Vector3d.Zero);
        capsule.CalculateInertiaTensor(Fixed64.One, Vector3d.Zero).Should().Be(expected);
        var zero = new LSCompoundCollider(CompoundColliderPart.Sphere(Fixed64.FromRaw(1),
            Vector3d.Zero, FixedQuaternion.Identity, Vector3d.One * Fixed64.Half));
        Action calculate = () => zero.CalculateInertiaTensor(Fixed64.One, Vector3d.Zero);
        calculate.Should().Throw<ArgumentException>().WithMessage("*greater than zero*");
    }

    [Theory]
    [InlineData(24, 618475290600)]
    [InlineData(1500000000, long.MaxValue)]
    public void RoundedRadiusCapsule_ShouldRetainRodLimitAndFinalSaturation(int height, long expectedRaw)
    {
        var compound = new LSCompoundCollider2D(CompoundColliderPart2D.Capsule(Fixed64.FromRaw(1),
            (Fixed64)height, Vector2d.Zero, Fixed64.Zero, Vector2d.One * Fixed64.Half));
        var rod = (LSCapsuleCollider2D)compound.GetPartCollider(0);
        Fixed64 mass = (Fixed64)12;
        rod.CalculateMomentOfInertia(mass, Vector2d.Zero).Should().Be(Fixed64.FromRaw(expectedRaw));
    }

    private static Fixed3x3 Tensor(int x, int y, int z, int xy = 0) => new(
        (Fixed64)x, (Fixed64)xy, Fixed64.Zero,
        (Fixed64)xy, (Fixed64)y, Fixed64.Zero,
        Fixed64.Zero, Fixed64.Zero, (Fixed64)z);
}
