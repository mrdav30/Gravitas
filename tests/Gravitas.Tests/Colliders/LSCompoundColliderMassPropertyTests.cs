using FixedMathSharp;
using FixedMathSharp.Geometry;
using FluentAssertions;
using Gravitas.Colliders;
using Gravitas.Queries;
using Gravitas.Support;
using Gravitas.Tests.Support;
using System;
using System.Numerics;
using Xunit;

namespace Gravitas.Tests.Colliders;

public sealed class LSCompoundColliderMassPropertyTests
{
    [Theory]
    [InlineData(3, 0)]
    [InlineData(6, 1)]
    public void CubeInertia_ShouldRoundTheCompleteDiagonalOnceAcrossRepresentations(long massRaw, long diagonalRaw)
    {
        var box = new LSCuboidCollider { Size = Vector3d.One };
        CompoundColliderPart part = CompoundColliderPart.Cuboid(Vector3d.One, Vector3d.Zero);
        var single = new LSCompoundCollider(part);
        var coincident = new LSCompoundCollider(part, part);
        Fixed64 mass = Fixed64.FromRaw(massRaw);
        // A unit cube has all three unit-mass diagonal entries 1/6.
        Fixed3x3 expected = RawTensor(diagonalRaw, diagonalRaw, diagonalRaw, 0, 0, 0);

        box.CalculateInertiaTensor(mass, Vector3d.Zero).Should().Be(expected);
        single.CalculateInertiaTensor(mass, Vector3d.Zero).Should().Be(expected);
        coincident.CalculateInertiaTensor(mass, Vector3d.Zero).Should().Be(expected);
    }

    [Fact]
    public void CubeInertia_ShouldRetainLargeUnitGeometryUntilTinyMassIsApplied()
    {
        Vector3d size = Vector3d.One * Fixed64.MaxValue;
        var box = new LSCuboidCollider { Size = size };
        var compound = new LSCompoundCollider(CompoundColliderPart.Cuboid(size, Vector3d.Zero));
        // Half extents round to 2^62 raw; Iraw = 2*hRaw^2/(3*Q^2).
        Fixed3x3 expected = RawTensor(768614336404564651, 768614336404564651, 768614336404564651, 0, 0, 0);

        box.CalculateInertiaTensor(Fixed64.FromRaw(1), Vector3d.Zero).Should().Be(expected);
        compound.CalculateInertiaTensor(Fixed64.FromRaw(1), Vector3d.Zero).Should().Be(expected);
    }

    [Fact]
    public void MicroCubeMeshCompound_ShouldRetainUnroundedVolumeIntegrals()
    {
        ColliderShapeDefinition source = MeshTestFixtures.CreateConvexCubeDefinition();
        Fixed64 side = Fixed64.FromFraction(1, 128);
        var vertices = new Vector3d[source.MeshVertexCount];
        var triangles = new int[source.MeshTriangleIndexCount];
        for (int i = 0; i < vertices.Length; i++) vertices[i] = source.GetMeshVertex(i) * side;
        for (int i = 0; i < triangles.Length; i++) triangles[i] = source.GetMeshTriangleIndex(i);
        var compound = new LSCompoundCollider(new CompoundColliderPart(ColliderShapeDefinition.ConvexMesh(vertices, triangles)));
        var box = new LSCuboidCollider { Size = Vector3d.One * side };
        using var context = GravitasWorldContext.CreateOwned();
        compound.InitializeWithNoBody(new TestMatterAgent(context));
        // I/m = side^2/6; scalar unit-inertia rounding must not be amplified by mass.
        Fixed3x3 expected = RawTensor(43690666666667, 43690666666667, 43690666666667, 0, 0, 0);
        Fixed64 mass = (Fixed64)1_000_000_000;

        compound.CalculateInertiaTensor(mass, Vector3d.Zero).Should().Be(expected);
        box.CalculateInertiaTensor(mass, Vector3d.Zero).Should().Be(expected);
    }

    [Fact]
    public void MicroTriangleShellCompound_ShouldRetainUnroundedCenteredSurfaceIntegrals()
    {
        Fixed64 side = Fixed64.FromFraction(1, 128);
        Vector3d[] vertices = { Vector3d.Zero, Vector3d.Right * side, Vector3d.Up * side };
        // Mesh preparation centers the authored bounds; restore the authored
        // frame so the independent triangle integral uses its original vertices.
        Vector3d offset = new(side * Fixed64.Half, side * Fixed64.Half, Fixed64.Zero);
        var compound = new LSCompoundCollider(CompoundColliderPart.ConvexMesh(
            vertices, new[] { 0, 1, 2 }, offset, MeshInertiaPolicy.SurfaceApproximation));
        using var context = GravitasWorldContext.CreateOwned();
        compound.InitializeWithNoBody(new TestMatterAgent(context));
        Fixed64 mass = (Fixed64)1_000_000_000;
        BigInteger q = BigInteger.One << 32;
        BigInteger s = side.m_rawValue;
        BigInteger c = 11184811;
        BigInteger variance = s * s - 4 * c * s + 6 * c * c;
        Fixed64 diagonal = RoundRational(mass.m_rawValue * variance, 6 * q * q);
        Fixed64 polar = RoundRational(2 * mass.m_rawValue * variance, 6 * q * q);
        Fixed64 product = RoundRational(mass.m_rawValue * (-s * s + 8 * c * s - 12 * c * c), 12 * q * q);
        Vector3d center = new(Fixed64.FromRaw((long)c), Fixed64.FromRaw((long)c), Fixed64.Zero);

        compound.CalculateLocalCenterOfMassOffset().Should().Be(center);
        compound.CalculateInertiaTensor(mass, center).Should().Be(new Fixed3x3(
            diagonal, product, Fixed64.Zero,
            product, diagonal, Fixed64.Zero,
            Fixed64.Zero, Fixed64.Zero, polar));
    }

    [Theory]
    [InlineData(1, false, true)]
    [InlineData(1, true, true)]
    [InlineData(3, false, false)]
    [InlineData(3, true, false)]
    [InlineData(3, false, true)]
    [InlineData(3, true, true)]
    [InlineData(4294967296, false, true)]
    [InlineData(4294967296, true, true)]
    public void UnequalSphereCompound_ShouldNormalizeEveryTensorEntryOnce(long massRaw, bool reversed, bool obliqueOffset)
    {
        Vector3d secondOffset = obliqueOffset
            ? new Vector3d((Fixed64)2048, (Fixed64)1024, (Fixed64)(-512))
            : Vector3d.Right * (Fixed64)2048;
        CompoundColliderPart first = CompoundColliderPart.Sphere((Fixed64)512, Vector3d.Zero);
        CompoundColliderPart second = CompoundColliderPart.Sphere((Fixed64)1024, secondOffset);
        var compound = reversed ? new LSCompoundCollider(second, first) : new LSCompoundCollider(first, second);
        Fixed3x3 expected = SphereCompoundOracle(massRaw, secondOffset, out Vector3d expectedCenter);
        using var context = GravitasWorldContext.CreateOwned();
        compound.InitializeWithNoBody(new TestMatterAgent(context));

        compound.CalculateLocalCenterOfMassOffset().Should().Be(expectedCenter);
        compound.CalculateInertiaTensor(Fixed64.FromRaw(massRaw), expectedCenter).Should().Be(expected);
    }

    [Fact]
    public void CoincidentSphereParts_ShouldMatchSingleSphereWithoutRoundingChildContributions()
    {
        Fixed64 radius = Fixed64.FromFraction(3, 4);
        var sphere = new LSSphereCollider { Radius = radius };
        var compound = new LSCompoundCollider(
            CompoundColliderPart.Sphere(radius, Vector3d.Zero),
            CompoundColliderPart.Sphere(radius, Vector3d.Zero));
        Fixed64 mass = Fixed64.FromRaw(3);
        Fixed3x3 expected = new(Fixed64.FromRaw(1), Fixed64.Zero, Fixed64.Zero,
            Fixed64.Zero, Fixed64.FromRaw(1), Fixed64.Zero,
            Fixed64.Zero, Fixed64.Zero, Fixed64.FromRaw(1));

        sphere.CalculateInertiaTensor(mass, Vector3d.Zero).Should().Be(expected);
        compound.CalculateInertiaTensor(mass, Vector3d.Zero).Should().Be(expected);
    }

    [Fact]
    public void SphereCompound_ShouldRetainUnrepresentableUnitInertiaUntilTinyParentMassIsApplied()
    {
        var compound = new LSCompoundCollider(
            CompoundColliderPart.Sphere(Fixed64.MaxValue, Vector3d.Zero),
            CompoundColliderPart.Sphere(Fixed64.MaxValue, Vector3d.Zero));
        Fixed64 expected = Fixed64.FromRaw(1844674406941458432);

        compound.CalculateInertiaTensor(Fixed64.FromRaw(1), Vector3d.Zero).Should().Be(new Fixed3x3(
            expected, Fixed64.Zero, Fixed64.Zero,
            Fixed64.Zero, expected, Fixed64.Zero,
            Fixed64.Zero, Fixed64.Zero, expected));
    }

    [Theory]
    [InlineData(false, false)]
    [InlineData(false, true)]
    [InlineData(true, false)]
    [InlineData(true, true)]
    public void CubeAndMeshCompound_ShouldRetainVolumeOrShellWeightsThroughTensorReduction(bool shell, bool reversed)
    {
        CompoundColliderPart first = CompoundColliderPart.Cuboid(Vector3d.One, Vector3d.Zero);
        CompoundColliderPart second = new(MeshTestFixtures.CreateConvexCubeDefinition(
            shell ? MeshInertiaPolicy.SurfaceApproximation : MeshInertiaPolicy.RequireClosedVolume),
            new Vector3d((Fixed64)3, Fixed64.One, (Fixed64)2));
        var compound = reversed ? new LSCompoundCollider(second, first) : new LSCompoundCollider(first, second);
        using var context = GravitasWorldContext.CreateOwned();
        compound.InitializeWithNoBody(new TestMatterAgent(context));
        // Unit cubes have volume 1; the shell has area 6 and unit-mass diagonal 5/18.
        Fixed3x3 expected = shell
            ? RawTensor(3, 6, 4, -1, -2, -1)
            : RawTensor(4, 10, 8, -2, -4, -2);

        compound.CalculateInertiaTensor(Fixed64.FromRaw(3), compound.CalculateLocalCenterOfMassOffset())
            .Should().Be(expected);
    }

    [Theory]
    [InlineData(1, 4, 2)]
    [InlineData(3, 12, 5)]
    [InlineData(4294967296, 16464041301, 7158278827)]
    public void RotatedBoxCompound_ShouldCombineUnroundedAnisotropicContributions(long massRaw, long xyRaw, long zRaw)
    {
        ColliderShapeDefinition shape = ColliderShapeDefinition.Cuboid(new Vector3d((Fixed64)2, (Fixed64)4, (Fixed64)6));
        FixedQuaternion quarterTurn = new FixedQuaternion(Fixed64.Zero, Fixed64.Zero, Fixed64.One, Fixed64.One).Normalized;
        var compound = new LSCompoundCollider(new CompoundColliderPart(shape),
            new CompoundColliderPart(shape, Vector3d.Zero, quarterTurn));
        using var context = GravitasWorldContext.CreateOwned();
        compound.InitializeWithNoBody(new TestMatterAgent(context));
        // Averaging the original and quarter-turned box yields unit diagonals 23/6, 23/6, 5/3.
        compound.CalculateInertiaTensor(Fixed64.FromRaw(massRaw), Vector3d.Zero)
            .Should().Be(RawTensor(xyRaw, xyRaw, zRaw, 0, 0, 0));
    }

    [Theory]
    [InlineData(0, 1443109012, 1443109012)]
    [InlineData(1, 7217632882, 1803886265)]
    [InlineData(2, 3383226276, 1082331759)]
    [InlineData(3, 5648436632, 1681068476)]
    public void FractionalCurvedInertia_ShouldRetainNonbinaryProducts(int family, long transverseRaw, long axialRaw)
    {
        Fixed64 radius = Fixed64.FromFraction(3, 5);
        Fixed64 height = Fixed64.FromFraction(11, 4);
        LSCollider collider = family switch
        {
            0 => new LSSphereCollider(ColliderShapeDefinition.Sphere(radius)),
            1 => new LSCylinderCollider(ColliderShapeDefinition.Cylinder(radius, height)),
            2 => new LSConeCollider(ColliderShapeDefinition.Cone(radius, height)),
            _ => new LSCapsuleCollider(ColliderShapeDefinition.Capsule(radius, height))
        };

        Fixed3x3 tensor = collider.CalculateInertiaTensor(Fixed64.FromFraction(7, 3), collider.CalculateLocalCenterOfMassOffset());
        tensor.M11.Should().Be(Fixed64.FromRaw(transverseRaw));
        tensor.M22.Should().Be(Fixed64.FromRaw(axialRaw));
        tensor.M33.Should().Be(Fixed64.FromRaw(transverseRaw));
    }

    [Fact]
    public void TinyCompoundMass_ShouldRetainMassRatiosAndParallelAxisTerms()
    {
        var compound = new LSCompoundCollider(
            CompoundColliderPart.Sphere((Fixed64)1024, Vector3d.Right),
            CompoundColliderPart.Sphere((Fixed64)1024, -Vector3d.Right));

        Fixed3x3 tensor = compound.CalculateInertiaTensor(Fixed64.FromRaw(3), Vector3d.Zero);

        tensor.Should().Be(new Fixed3x3(
            Fixed64.FromRaw(1258291), Fixed64.Zero, Fixed64.Zero,
            Fixed64.Zero, Fixed64.FromRaw(1258294), Fixed64.Zero,
            Fixed64.Zero, Fixed64.Zero, Fixed64.FromRaw(1258294)));
    }

    [Theory]
    [InlineData(1, 1844674406941458432)]
    [InlineData(4294967296, long.MaxValue)]
    public void SphereInertia_ShouldSaturateOnlyTheFinalMoment(long massRaw, long expectedRaw)
    {
        var sphere = new LSSphereCollider { Radius = Fixed64.MaxValue };
        sphere.CalculateInertiaTensor(Fixed64.FromRaw(massRaw), Vector3d.Zero).M11
            .Should().Be(Fixed64.FromRaw(expectedRaw));
    }

    [Theory]
    [InlineData(0)]
    [InlineData(1)]
    [InlineData(2)]
    [InlineData(3)]
    [InlineData(4)]
    public void UnequalDetachedParts_ShouldUseScaledVolumesBeforeInitialization(int family)
    {
        ColliderShapeDefinition shape = family switch
        {
            0 => ColliderShapeDefinition.Sphere(Fixed64.One),
            1 => ColliderShapeDefinition.Cylinder(Fixed64.One, Fixed64.Two),
            2 => ColliderShapeDefinition.Capsule(Fixed64.One, (Fixed64)4),
            3 => ColliderShapeDefinition.Cone(Fixed64.One, (Fixed64)4),
            _ => ColliderShapeDefinition.Cuboid(Vector3d.One * Fixed64.Two)
        };
        var compound = new LSCompoundCollider(
            new CompoundColliderPart(shape, Vector3d.Zero),
            new CompoundColliderPart(shape, Vector3d.Right * (Fixed64)9,
                FixedQuaternion.Identity, Vector3d.One * Fixed64.Two));

        // Doubling every dimension gives the second solid eight times the mass.
        Vector3d expected = new((Fixed64)8,
            family == 3 ? Fixed64.FromFraction(-17, 9) : Fixed64.Zero,
            Fixed64.Zero);
        Vector3d detached = compound.CalculateLocalCenterOfMassOffset();
        detached.Should().Be(expected);
        using var context = GravitasWorldContext.CreateOwned();
        compound.InitializeWithNoBody(new TestMatterAgent(context));

        compound.CalculateLocalCenterOfMassOffset().Should().Be(detached);
    }

    [Theory]
    [InlineData(0)]
    [InlineData(1)]
    [InlineData(2)]
    [InlineData(3)]
    [InlineData(4)]
    public void EquivalentDetachedParts_ShouldRetainMassPropertiesAfterInitialization(int family)
    {
        Vector3d scale = new(Fixed64.One, (Fixed64)4, Fixed64.One);
        Vector3d offset = Vector3d.Right * (Fixed64)10;
        CompoundColliderPart first;
        CompoundColliderPart second;
        switch (family)
        {
            case 0:
                scale.Y = (Fixed64)2;
                first = CompoundColliderPart.Sphere(Fixed64.One, Vector3d.Zero, FixedQuaternion.Identity, scale);
                second = CompoundColliderPart.Sphere((Fixed64)2, offset);
                break;
            case 1:
                first = CompoundColliderPart.Cylinder(Fixed64.One, Fixed64.Two, Vector3d.Zero, FixedQuaternion.Identity, scale);
                second = CompoundColliderPart.Cylinder(Fixed64.One, (Fixed64)8, offset);
                break;
            case 2:
                first = CompoundColliderPart.Capsule(Fixed64.One, (Fixed64)4, Vector3d.Zero, FixedQuaternion.Identity, scale);
                second = CompoundColliderPart.Capsule(Fixed64.One, (Fixed64)16, offset);
                break;
            case 3:
                first = CompoundColliderPart.Cone(Fixed64.One, (Fixed64)4, Vector3d.Zero, FixedQuaternion.Identity, scale);
                second = CompoundColliderPart.Cone(Fixed64.One, (Fixed64)16, offset);
                break;
            default:
                first = CompoundColliderPart.Cuboid(Vector3d.One * Fixed64.Two, Vector3d.Zero, FixedQuaternion.Identity, scale);
                second = CompoundColliderPart.Cuboid(new Vector3d(Fixed64.Two, (Fixed64)8, Fixed64.Two), offset);
                break;
        }

        var compound = new LSCompoundCollider(first, second);
        Vector3d expectedCenter = new((Fixed64)5, family == 3 ? (Fixed64)(-4) : Fixed64.Zero, Fixed64.Zero);
        compound.CalculateLocalCenterOfMassOffset().Should().Be(expectedCenter);
        Fixed3x3 detachedTensor = compound.CalculateInertiaTensor(Fixed64.Two, expectedCenter);
        using var context = GravitasWorldContext.CreateOwned();
        compound.InitializeWithNoBody(new TestMatterAgent(context));

        compound.CalculateLocalCenterOfMassOffset().Should().Be(expectedCenter);
        compound.CalculateInertiaTensor(Fixed64.Two, expectedCenter).Should().Be(detachedTensor);
    }

    [Theory]
    [InlineData(0, 1, 419430, 419430)]
    [InlineData(0, 3, 1258291, 1258291)]
    [InlineData(0, 4294967296, 1801439850528768, 1801439850528768)]
    [InlineData(1, 1, 419430, 419430)]
    [InlineData(1, 3, 1258291, 1258291)]
    [InlineData(2, 1, 611669, 524288)]
    [InlineData(2, 3, 1835008, 1572864)]
    [InlineData(3, 1, 314573, 314573)]
    [InlineData(3, 3, 943718, 943718)]
    [InlineData(4, 1, 763963, 464369)]
    [InlineData(4, 3, 2291888, 1393108)]
    public void CurvedInertia_ShouldApplyMassBeforeFinalRounding(int family, long massRaw, long transverseRaw, long axialRaw)
    {
        Fixed64 radius = (Fixed64)1024;
        LSCollider collider = family switch
        {
            0 => new LSSphereCollider(ColliderShapeDefinition.Sphere(radius)),
            1 => new LSCapsuleCollider(ColliderShapeDefinition.Capsule(radius, (Fixed64)2048)),
            2 => new LSCylinderCollider(ColliderShapeDefinition.Cylinder(radius, (Fixed64)2048)),
            3 => new LSConeCollider(ColliderShapeDefinition.Cone(radius, (Fixed64)2048)),
            _ => new LSCapsuleCollider(ColliderShapeDefinition.Capsule(radius, (Fixed64)3072))
        };
        using var context = GravitasWorldContext.CreateOwned();
        collider.InitializeWithNoBody(new TestMatterAgent(context));

        // Independently rounded raw products retain each documented fixed coefficient.
        Fixed3x3 tensor = collider.CalculateInertiaTensor(Fixed64.FromRaw(massRaw), collider.CalculateLocalCenterOfMassOffset());
        tensor.Should().Be(new Fixed3x3(
            Fixed64.FromRaw(transverseRaw), Fixed64.Zero, Fixed64.Zero,
            Fixed64.Zero, Fixed64.FromRaw(axialRaw), Fixed64.Zero,
            Fixed64.Zero, Fixed64.Zero, Fixed64.FromRaw(transverseRaw)));
    }

    [Theory]
    [InlineData(0)]
    [InlineData(-1)]
    public void CalculateInertiaTensor_ForNonpositiveMass_ShouldReturnZeroBeforeGeometryEvaluation(int mass)
    {
        var collider = new LSCompoundCollider(
            CompoundColliderPart.Sphere(Fixed64.Half, Vector3d.Right * Fixed64.Two))
        {
            LocalOffset = Vector3d.Right * Fixed64.MaxValue
        };
        collider.CalculateInertiaTensor((Fixed64)mass, Vector3d.Right).Should().Be(Fixed3x3.Zero);
    }

    [Fact]
    public void DetachedCompound_ShouldAggregateAnUnrepresentableChildCenter()
    {
        var compound = new LSCompoundCollider(
            CompoundColliderPart.Sphere(Fixed64.Half, Vector3d.Right),
            CompoundColliderPart.Sphere(Fixed64.Half, -Vector3d.Right))
        {
            LocalOffset = new Vector3d(
                Fixed64.MaxValue,
                Fixed64.Zero,
                Fixed64.Zero)
        };
        LSCollider unrepresentablePart = compound.GetPartCollider(0);

        Action calculatePart =
            () => unrepresentablePart.CalculateLocalCenterOfMassOffset();
        Action calculatePartTensor =
            () => unrepresentablePart.CalculateInertiaTensor(
                Fixed64.One,
                Vector3d.Zero);
        Vector3d center = compound.CalculateLocalCenterOfMassOffset();
        Fixed3x3 tensor = compound.CalculateInertiaTensor(
            Fixed64.One,
            center);

        calculatePart.Should().Throw<InvalidOperationException>()
            .WithMessage("*outside*coordinate*domain*");
        calculatePartTensor.Should().Throw<InvalidOperationException>()
            .WithMessage("*outside the Fixed64 scalar domain*");
        center.Should().Be(new Vector3d(
            Fixed64.MaxValue,
            Fixed64.Zero,
            Fixed64.Zero));
        tensor.M11.Should().BeGreaterThan(Fixed64.Zero);
        tensor.M22.Should().BeGreaterThan(tensor.M11);
        tensor.M33.Should().Be(tensor.M22);
    }

    [Fact]
    public void DetachedCompound_ShouldRejectAnUnrepresentableAggregateCenter()
    {
        var compound = new LSCompoundCollider(
            CompoundColliderPart.Sphere(Fixed64.Half, Vector3d.Right),
            CompoundColliderPart.Sphere(
                Fixed64.Half,
                new Vector3d((Fixed64)3, Fixed64.Zero, Fixed64.Zero)))
        {
            LocalOffset = new Vector3d(
                Fixed64.MaxValue,
                Fixed64.Zero,
                Fixed64.Zero)
        };

        Action calculate = () =>
            compound.CalculateLocalCenterOfMassOffset();

        calculate.Should().Throw<InvalidOperationException>()
            .WithMessage("*compound collider's center of mass*");
    }

    [Fact]
    public void InitializedCompound_ShouldAggregateAnUnrepresentableBodyLocalChildCenter()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        var compound = new LSCompoundCollider(
            CompoundColliderPart.Sphere(Fixed64.Half, Vector3d.Right),
            CompoundColliderPart.Sphere(Fixed64.Half, -Vector3d.Right))
        {
            LocalOffset = new Vector3d(
                Fixed64.MaxValue,
                Fixed64.Zero,
                Fixed64.Zero)
        };

        scenario.CreateBody(
            compound,
            new Vector3d(
                Fixed64.MinValue,
                Fixed64.Zero,
                Fixed64.Zero),
            FixedQuaternion.Identity,
            mass: Fixed64.One);

        compound.CalculateLocalCenterOfMassOffset().Should().Be(
            new Vector3d(
                Fixed64.MaxValue,
                Fixed64.Zero,
                Fixed64.Zero));
    }

    [Fact]
    public void Compound_ShouldPreserveWideVolumeRatios()
    {
        Fixed64 extent = (Fixed64)1_500_000;
        var compound = new LSCompoundCollider(
            CompoundColliderPart.Cuboid(
                new Vector3d(extent, extent, extent),
                Vector3d.Zero),
            CompoundColliderPart.Cuboid(
                new Vector3d(extent, extent, extent * Fixed64.Two),
                new Vector3d((Fixed64)3, Fixed64.Zero, Fixed64.Zero)));

        compound.CalculateLocalCenterOfMassOffset().Should().Be(
            new Vector3d(Fixed64.Two, Fixed64.Zero, Fixed64.Zero));
    }

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
    public void MassProperties_ShouldKeepPartTranslationInOwnerFrameAndRotateAnisotropicTensor()
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
        Vector3d expectedCenter = new((Fixed64)5, Fixed64.Zero, Fixed64.Zero);
        Vector3d detachedCenter =
            compound.CalculateLocalCenterOfMassOffset();
        scenario.CreateBody(compound, new Vector3d((Fixed64)10, Fixed64.Zero, Fixed64.Zero), FixedQuaternion.Identity);
        Fixed3x3 expectedTensor = new(
            (Fixed64)40, Fixed64.Zero, Fixed64.Zero,
            Fixed64.Zero, (Fixed64)52, Fixed64.Zero,
            Fixed64.Zero, Fixed64.Zero, (Fixed64)20);

        detachedCenter.Should().Be(expectedCenter);
        compound.Center.Should().Be(new Vector3d((Fixed64)13, Fixed64.Zero, Fixed64.Zero));
        compound.GetPartCollider(0).Center.Should().Be(new Vector3d((Fixed64)15, Fixed64.Zero, Fixed64.Zero));
        compound.CalculateLocalCenterOfMassOffset().Should().Be(expectedCenter);
        compound.CalculateInertiaTensor((Fixed64)12, expectedCenter).Should().Be(expectedTensor);
    }

    [Fact]
    public void MassProperties_WhenScalarVolumesRoundToZero_ShouldRetainRationalWeightsThroughInertia()
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
        // Equal masses at offsets -2, -1, 3 give mean squared distance 14/3.
        Fixed64 expectedTransverse = Fixed64.FromFraction(14, 3);
        Fixed3x3 expected = new(
            Fixed64.Zero, Fixed64.Zero, Fixed64.Zero,
            Fixed64.Zero, expectedTransverse, Fixed64.Zero,
            Fixed64.Zero, Fixed64.Zero, expectedTransverse);

        center.Should().Be(new Vector3d(Fixed64.One, Fixed64.Zero, Fixed64.Zero));
        compound.CalculateInertiaTensor(mass, center).Should().Be(expected);
    }

    [Fact]
    public void MassProperties_ShouldRetainNegligiblePositiveWeightsWithoutBiasingFinalTensor()
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
    public void MassProperties_ShouldRetainUnequalBoxWeightsThroughFinalTensor()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        var compound = new LSCompoundCollider(
            CompoundColliderPart.Cuboid(
                new Vector3d((Fixed64)3, Fixed64.One, Fixed64.One),
                Vector3d.Zero),
            CompoundColliderPart.Cuboid(
                new Vector3d((Fixed64)3, Fixed64.One, Fixed64.One),
                Vector3d.Zero),
            CompoundColliderPart.Cuboid(
                new Vector3d((Fixed64)3, Fixed64.One, Fixed64.One),
                Vector3d.Zero),
            CompoundColliderPart.Cuboid(Vector3d.One, Vector3d.Zero));
        scenario.CreateBody(
            compound,
            Vector3d.Zero,
            FixedQuaternion.Identity,
            preventAngularForces: true);
        // Weights 3:3:3:1 give transverse raw inertia 2*(9*5/6+1/6)/10 = 23/15.
        Fixed3x3 expected = RawTensor(0, 2, 2, 0, 0, 0);

        compound.CalculateInertiaTensor(
                Fixed64.FromRaw(2),
                Vector3d.Zero)
            .Should()
            .Be(expected);
    }

    [Fact]
    public void OffCenterPart_ShouldKeepConservativeRadiusAndPublicQueryVisibility()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        var compound = new LSCompoundCollider(
            CompoundColliderPart.Sphere(Fixed64.Half, new Vector3d((Fixed64)10, Fixed64.Zero, Fixed64.Zero)));
        scenario.CreateBody(compound, Vector3d.Zero, FixedQuaternion.Identity);

        compound.ScaledRadius.Should().Be(Fixed64.FromFraction(21, 2));
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

        GetMeasure(sphere.CalculateMassPropertyWeight()).Should().Be(
            Fixed64.FromFraction(4, 3) * Fixed64.Pi * sphere.ScaledRadiusSqr * sphere.ScaledRadius);
        GetMeasure(capsule.CalculateMassPropertyWeight()).Should().Be(
            Fixed64.Pi * capsule.ScaledRadiusSqr * capsule.AxisLength
            + Fixed64.FromFraction(4, 3) * Fixed64.Pi * capsule.ScaledRadiusSqr * capsule.ScaledRadius);
        GetMeasure(box.CalculateMassPropertyWeight()).Should().Be((Fixed64)24);
        GetMeasure(cylinder.CalculateMassPropertyWeight()).Should().Be(
            Fixed64.Pi * cylinder.ScaledRadiusSqr * cylinder.Height);
        AssertNear(
            GetMeasure(cone.CalculateMassPropertyWeight()),
            cone.Volume);
        closedMesh.Mesh.TryGetClosedVolumeMassProperties(out MeshMassProperties closedProperties, out _)
            .Should().BeTrue();
        GetMeasure(closedMesh.CalculateMassPropertyWeight()).Should().Be(closedProperties.Volume);
        GetMeasure(shellMesh.CalculateMassPropertyWeight()).Should().Be(shellMesh.Mesh.TotalArea);
        GetMeasure(shellMesh.CalculateMassPropertyWeight())
            .Should().NotBe(GetMeasure(closedMesh.CalculateMassPropertyWeight()));
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

        GetMeasure(mesh.CalculateMassPropertyWeight()).Should().Be(Fixed64.Zero);
        GetMeasure(compound.CalculateMassPropertyWeight()).Should().Be(Fixed64.Zero);
        compound.CalculateLocalCenterOfMassOffset().Should().Be(Vector3d.Zero);
        FluentActions.Invoking(
                () => compound.CalculateInertiaTensor(
                    Fixed64.One,
                    Vector3d.Zero))
            .Should().Throw<InvalidOperationException>()
            .WithMessage("*valid mass properties*");
    }

    [Fact]
    public void MinimumRepresentableConePart_ShouldUseDeterministicDegenerateSideProjection()
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

        cone.WorldAxis.Should().Be(Vector3d.Up);
        cone.Height.Should().Be(Fixed64.FromRaw(1));
        cone.ScaledRadius.Should().Be(Fixed64.FromRaw(1));
        GetMeasure(cone.CalculateMassPropertyWeight()).Should().Be(Fixed64.Zero);
        cone.ClosestPointOnSurface(Vector3d.Right)
            .Should().Be(new Vector3d(Fixed64.FromRaw(1), Fixed64.Zero, Fixed64.Zero));
    }

    [Fact]
    public void ClosedMeshPart_WithOwnerAndPartScaleRotation_ShouldTransformCenterAndReferenceTensor()
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
        Vector3d partScale = new((Fixed64)2, (Fixed64)3, (Fixed64)4);
        Vector3d ownerScale = new((Fixed64)3, (Fixed64)2, Fixed64.One);
        Vector3d partOffset = new((Fixed64)2, -Fixed64.One, Fixed64.Half);
        var compound = new LSCompoundCollider(
            CompoundColliderPart.ConvexMesh(
                vertices,
                triangles,
                partOffset,
                partRotation,
                partScale));
        compound.LocalOffset = new Vector3d(Fixed64.One, (Fixed64)2, (Fixed64)3);
        var transform = new FixedTransform(Vector3d.Zero, FixedQuaternion.Identity, ownerScale);
        var body = new SolidBody(new TestMatterAgent(scenario.Context, transform), compound)
        {
            Mass = Fixed64.One
        };
        body.Initialize(Vector3d.Zero, FixedQuaternion.Identity);
        var mesh = (LSMeshCollider)compound.GetPartCollider(0);
        Vector3d rawCenter = new(Fixed64.Quarter, Fixed64.Quarter, Fixed64.Quarter);
        Vector3d effectiveScale = Vector3d.Multiply(ownerScale, partScale);
        Vector3d scaledPartOffset = Vector3d.Multiply(partOffset, ownerScale);
        Vector3d centeredMeshCenter = Vector3d.Multiply(
            rawCenter - mesh.Mesh.LocalBounds.Center,
            effectiveScale);
        Vector3d expectedCenter = Vector3d.Multiply(compound.LocalOffset, ownerScale)
            + scaledPartOffset
            + partRotation * centeredMeshCenter;

        AssertVectorNear(mesh.CalculateLocalCenterOfMassOffset(), expectedCenter);
        AssertVectorNear(compound.CalculateLocalCenterOfMassOffset(), expectedCenter);
        mesh.Mesh.TryGetClosedVolumeMassProperties(out MeshMassProperties properties, out _).Should().BeTrue();
        Fixed3x3 meshCenterTensor = properties.CalculateInertiaTensor(Fixed64.One, properties.CenterOfMass);
        Fixed3x3 ownerCenterTensor = InertiaTensorMath.RotateToFrame(meshCenterTensor, partRotation);
        Vector3d ownerReference = expectedCenter + new Vector3d(Fixed64.One, (Fixed64)2, (Fixed64)3);
        Fixed3x3 expectedTensor = InertiaTensorMath.AddParallelAxisTensor(
            ownerCenterTensor,
            Fixed64.One,
            ownerReference - expectedCenter);
        Fixed3x3 tensor = compound.CalculateInertiaTensor(Fixed64.One, ownerReference);

        AssertMatrixNear(tensor, expectedTensor);
    }

    [Fact]
    public void Compound_ShouldRejectAnUnrepresentablePartParallelAxisTensor()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        var compound = new LSCompoundCollider(
            CompoundColliderPart.Sphere(
                Fixed64.One,
                new Vector3d((Fixed64)(-50000), Fixed64.Zero, Fixed64.Zero)),
            CompoundColliderPart.Sphere(
                Fixed64.One,
                new Vector3d((Fixed64)50000, Fixed64.Zero, Fixed64.Zero)));
        scenario.CreateBody(
            compound,
            Vector3d.Zero,
            FixedQuaternion.Identity,
            preventAngularForces: true);

        FluentActions.Invoking(
                () => compound.CalculateInertiaTensor(
                    Fixed64.Two,
                    Vector3d.Zero))
            .Should().Throw<InvalidOperationException>()
            .WithMessage("*outside the Fixed64 scalar domain*");
    }

    [Fact]
    public void Compound_ShouldRejectAnUnrepresentableFinalTensorSum()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        var compound = new LSCompoundCollider(
            CompoundColliderPart.Sphere((Fixed64)2, Vector3d.Zero),
            CompoundColliderPart.Sphere((Fixed64)2, Vector3d.Zero));
        scenario.CreateBody(
            compound,
            Vector3d.Zero,
            FixedQuaternion.Identity);

        FluentActions.Invoking(
                () => compound.CalculateInertiaTensor(
                    Fixed64.MaxValue,
                    Vector3d.Zero))
            .Should().Throw<InvalidOperationException>()
            .WithMessage("*outside the Fixed64 scalar domain*");
    }

    private static Fixed3x3 SphereCompoundOracle(long massRaw, Vector3d secondOffset, out Vector3d center)
    {
        BigInteger q = BigInteger.One << 32;
        BigInteger[] radii = { 512 * q, 1024 * q };
        BigInteger[] weights = { BigInteger.Pow(radii[0], 3), BigInteger.Pow(radii[1], 3) };
        BigInteger totalWeight = weights[0] + weights[1];
        BigInteger[] offset = { secondOffset.X.m_rawValue, secondOffset.Y.m_rawValue, secondOffset.Z.m_rawValue };
        Fixed64[] coordinates = new Fixed64[3];
        for (int axis = 0; axis < 3; axis++)
            coordinates[axis] = RoundRational(weights[1] * offset[axis], totalWeight);
        center = new Vector3d(coordinates[0], coordinates[1], coordinates[2]);
        Fixed64[] entries = new Fixed64[9];
        for (int row = 0; row < 3; row++)
        for (int column = 0; column < 3; column++)
        {
            BigInteger integral = BigInteger.Zero;
            for (int part = 0; part < 2; part++)
            {
                BigInteger[] shift = new BigInteger[3];
                for (int axis = 0; axis < 3; axis++)
                    shift[axis] = (part == 0 ? BigInteger.Zero : offset[axis]) - coordinates[axis].m_rawValue;
                BigInteger geometry = row == column ? 1717986918 * radii[part] * radii[part]
                    : -q * shift[row] * shift[column];
                if (row == column)
                    for (int axis = 0; axis < 3; axis++)
                        if (axis != row) geometry += q * shift[axis] * shift[axis];
                integral += weights[part] * geometry;
            }
            entries[row * 3 + column] = RoundRational(massRaw * integral, totalWeight * q * q * q);
        }
        return new Fixed3x3(entries[0], entries[1], entries[2], entries[3], entries[4], entries[5], entries[6], entries[7], entries[8]);
    }

    private static Fixed3x3 RawTensor(long xx, long yy, long zz, long xy, long xz, long yz) =>
        new(Fixed64.FromRaw(xx), Fixed64.FromRaw(xy), Fixed64.FromRaw(xz),
            Fixed64.FromRaw(xy), Fixed64.FromRaw(yy), Fixed64.FromRaw(yz),
            Fixed64.FromRaw(xz), Fixed64.FromRaw(yz), Fixed64.FromRaw(zz));

    private static Fixed64 RoundRational(BigInteger numerator, BigInteger denominator)
    {
        BigInteger rounded = BigInteger.DivRem(BigInteger.Abs(numerator), denominator, out BigInteger remainder);
        int half = (2 * remainder).CompareTo(denominator);
        if (half > 0 || half == 0 && !rounded.IsEven) rounded++;
        return Fixed64.FromRaw((long)(numerator.Sign < 0 ? -rounded : rounded));
    }

    private static void AssertNear(Fixed64 actual, Fixed64 expected) =>
        FixedMath.Abs(actual - expected)
            .Should().BeLessThanOrEqualTo(Fixed64.Epsilon * (Fixed64)512);

    private static Fixed64 GetMeasure(ExactMassWeight weight)
    {
        weight.TryGetMeasure(out Fixed64 measure).Should().BeTrue();
        return measure;
    }

    private static void AssertVectorNear(Vector3d actual, Vector3d expected)
    {
        AssertNear(actual.X, expected.X);
        AssertNear(actual.Y, expected.Y);
        AssertNear(actual.Z, expected.Z);
    }

    private static void AssertMatrixNear(Fixed3x3 actual, Fixed3x3 expected)
    {
        AssertNear(actual.M11, expected.M11);
        AssertNear(actual.M12, expected.M12);
        AssertNear(actual.M13, expected.M13);
        AssertNear(actual.M21, expected.M21);
        AssertNear(actual.M22, expected.M22);
        AssertNear(actual.M23, expected.M23);
        AssertNear(actual.M31, expected.M31);
        AssertNear(actual.M32, expected.M32);
        AssertNear(actual.M33, expected.M33);
    }
}
