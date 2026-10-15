using System;
using System.Linq;
using System.Numerics;
using FixedMathSharp;
using FluentAssertions;
using Gravitas.Colliders;
using Gravitas.Tests.Support;
using Xunit;

namespace Gravitas.Tests.Colliders;

public sealed class MeshRuntimeMassPropertyTests
{
    [Theory]
    [InlineData(false, 1, 3)]
    [InlineData(false, 1, 6)]
    [InlineData(false, 128, 4294967296000000000)]
    [InlineData(true, 1, 3)]
    [InlineData(true, 1, 6)]
    [InlineData(true, 128, 4294967296000000000)]
    public void RuntimeMesh_ShouldApplyMassBeforeProjectingUnitTensor(bool surface, int divisor, long massRaw)
    {
        MeshInertiaPolicy policy = Policy(surface);
        Vector3d sides = Vector3d.One / (Fixed64)divisor;
        LSMeshCollider collider = Box(sides, policy);
        Fixed64 mass = Fixed64.FromRaw(massRaw);
        Fixed3x3 expected = BoxTensor(sides, massRaw, surface, Vector3d.Zero, Vector3d.Zero);
        collider.Mesh.CalculateInertiaTensor(mass, policy).Should().Be(expected);
        collider.CalculateInertiaTensor(mass, Vector3d.Zero).Should().Be(expected);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void RuntimeSimplex_ShouldRetainFirstMomentsForEachRequestedReference(bool surface)
    {
        Fixed64 a = Fixed64.FromRaw(Fixed64.One.m_rawValue + 3);
        Fixed64 b = Fixed64.FromRaw(Fixed64.One.m_rawValue * 2 + 1);
        Fixed64 c = surface ? Fixed64.Zero : Fixed64.FromRaw(Fixed64.One.m_rawValue * 3 + 5);
        Vector3d[] vertices = surface
            ? new[] { Vector3d.Zero, Vector3d.Right * a, Vector3d.Up * b }
            : new[] { Vector3d.Zero, Vector3d.Right * a, Vector3d.Up * b, Vector3d.Forward * c };
        int[] triangles = surface ? new[] { 0, 1, 2 } : new[] { 1, 2, 3, 0, 2, 1, 0, 1, 3, 0, 3, 2 };
        MeshInertiaPolicy policy = Policy(surface);
        var collider = new LSMeshCollider(vertices, triangles, MeshColliderMode.Convex, policy);
        foreach (long massRaw in new[] { 3L, 6L, 123456789012L, 4294967296000000000L })
        foreach (Vector3d reference in new[] { Vector3d.Zero, new Vector3d(Fixed64.Half, -Fixed64.Quarter, Fixed64.One) })
        {
            Fixed64 mass = Fixed64.FromRaw(massRaw);
            collider.Mesh.CalculateInertiaTensor(mass, policy, reference)
                .Should().Be(SimplexTensor(a, b, c, massRaw, surface, reference));
            // The collider reference is body-local; the mesh reference is in its centered geometry frame.
            Vector3d bodyReference = collider.LocalOffset + reference;
            collider.CalculateInertiaTensor(mass, bodyReference)
                .Should().Be(SimplexTensor(a, b, c, massRaw, surface, reference));
        }
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void DetachedMeshPart_ShouldUseAuthoredScaleBeforeAndAfterInitialization(bool surface)
    {
        MeshInertiaPolicy policy = Policy(surface);
        ColliderShapeDefinition definition = MeshTestFixtures.CreateConvexCubeDefinition(policy);
        Vector3d partScale = new((Fixed64)2, (Fixed64)3, (Fixed64)4);
        Vector3d partOffset = new((Fixed64)3, Fixed64.One, (Fixed64)2);
        var compound = new LSCompoundCollider(new CompoundColliderPart(definition, partOffset,
            new FixedQuaternion(Fixed64.Half, Fixed64.Half, Fixed64.Half, Fixed64.Half), partScale));
        compound.LocalOffset = new Vector3d(Fixed64.One, Fixed64.Two, Fixed64.Zero);
        Vector3d center = compound.LocalOffset + partOffset;
        compound.CalculateLocalCenterOfMassOffset().Should().Be(center);
        compound.CalculateInertiaTensor((Fixed64)6, Vector3d.Zero).Should()
            .Be(BoxTensor(new Vector3d(partScale.Z, partScale.X, partScale.Y), ((Fixed64)6).m_rawValue,
                surface, center, Vector3d.Zero));
        compound.GetPartCollider(0).CalculateInertiaTensor((Fixed64)6, Vector3d.Zero).Should()
            .Be(BoxTensor(new Vector3d(partScale.Z, partScale.X, partScale.Y), ((Fixed64)6).m_rawValue,
                surface, center, Vector3d.Zero));

        using var context = GravitasWorldContext.CreateOwned();
        Vector3d ownerScale = new((Fixed64)3, (Fixed64)4, Fixed64.One);
        var transform = new FixedTransform(Vector3d.Zero, FixedQuaternion.Identity, ownerScale);
        compound.InitializeWithNoBody(new TestMatterAgent(context, transform));
        Vector3d effective = Vector3d.Multiply(partScale, ownerScale);
        center = Vector3d.Multiply(compound.LocalOffset + partOffset, ownerScale);
        compound.CalculateLocalCenterOfMassOffset().Should().Be(center);
        compound.CalculateInertiaTensor((Fixed64)6, Vector3d.Zero).Should()
            .Be(BoxTensor(new Vector3d(effective.Z, effective.X, effective.Y), ((Fixed64)6).m_rawValue,
                surface, center, Vector3d.Zero));
        compound.GetPartCollider(0).CalculateInertiaTensor((Fixed64)6, Vector3d.Zero).Should()
            .Be(BoxTensor(new Vector3d(effective.Z, effective.X, effective.Y), ((Fixed64)6).m_rawValue,
                surface, center, Vector3d.Zero));
    }

    [Fact]
    public void RuntimeMesh_ShouldRetainSemanticOffsetsBeyondTheScalarDifferenceDomain()
    {
        LSMeshCollider collider = Box(Vector3d.One, MeshInertiaPolicy.RequireClosedVolume);
        collider.LocalOffset = new Vector3d((Fixed64)(-1500000000), Fixed64.Zero, Fixed64.Zero);
        Vector3d reference = new((Fixed64)1500000000, Fixed64.Zero, Fixed64.Zero);
        collider.CalculateInertiaTensor(Fixed64.FromRaw(1), reference).Should()
            .Be(BoxTensor(Vector3d.One, 1, false, collider.LocalOffset, reference));
    }

    [Fact]
    public void RuntimeMesh_ShouldRefreshOnlyPublishedGeometryAndRetainBothPolicyResults()
    {
        ColliderShapeDefinition definition = MeshTestFixtures.CreateConvexCubeDefinition();
        var mesh = new PhysicsMesh(definition.GetMeshVerticesForRuntime(), definition.GetMeshTrianglesForRuntime(),
            Vector3d.Zero, FixedQuaternion.Identity);
        foreach (Vector3d scale in new[] { Vector3d.One, Vector3d.One * Fixed64.Two, new Vector3d((Fixed64)2, (Fixed64)3, (Fixed64)4) })
        {
            mesh.UpdateTransform(Vector3d.Zero, FixedQuaternion.Identity, scale);
            foreach (bool surface in new[] { false, true, false, true })
            {
                MeshInertiaPolicy policy = Policy(surface);
                foreach (long massRaw in new[] { 3L, 6L, ((Fixed64)6).m_rawValue })
                    mesh.CalculateInertiaTensor(Fixed64.FromRaw(massRaw), policy, Vector3d.One)
                        .Should().Be(BoxTensor(scale, massRaw, surface, Vector3d.Zero, Vector3d.One));
            }
            mesh.PrepareTransformation(Vector3d.Zero, FixedQuaternion.Identity, Vector3d.One * (Fixed64)5,
                Vector3d.One, MeshInertiaPolicy.RequireClosedVolume);
            mesh.CalculateInertiaTensor((Fixed64)6).Should()
                .Be(BoxTensor(scale, ((Fixed64)6).m_rawValue, false, Vector3d.Zero, Vector3d.Zero));
            Action rejected = () => mesh.UpdateTransform(Vector3d.Zero, FixedQuaternion.Identity,
                new Vector3d(-Fixed64.One, Fixed64.One, Fixed64.One));
            rejected.Should().Throw<ArgumentException>();
            mesh.CalculateInertiaTensor((Fixed64)6).Should()
                .Be(BoxTensor(scale, ((Fixed64)6).m_rawValue, false, Vector3d.Zero, Vector3d.Zero));
        }
    }

    [Fact]
    public void PublicProjection_ShouldRetainItsDataContractWhileRuntimeUsesCompleteIntegrals()
    {
        PhysicsMesh mesh = Box(Vector3d.One, MeshInertiaPolicy.RequireClosedVolume).Mesh;
        mesh.TryGetClosedVolumeMassProperties(out MeshMassProperties properties, out _).Should().BeTrue();
        properties.UnitMassInertiaTensor.M11.Should().Be(Fixed64.FromRaw(715827883));
        properties.CalculateInertiaTensor(Fixed64.FromRaw(3), Vector3d.Zero).M11.Should().Be(Fixed64.FromRaw(1));
        mesh.CalculateInertiaTensor(Fixed64.FromRaw(3)).M11.Should().Be(Fixed64.Zero);
        mesh.SurfaceMassProperties.CalculateInertiaTensor(Fixed64.FromRaw(27), Vector3d.Zero).M11
            .Should().Be(Fixed64.FromRaw(7));
        mesh.CalculateInertiaTensor(Fixed64.FromRaw(27), MeshInertiaPolicy.SurfaceApproximation).M11
            .Should().Be(Fixed64.FromRaw(8));
    }

    [Fact]
    public void MeshAndCollider_ShouldRetainTheirDistinctValidationAndNonpositiveMassContracts()
    {
        LSMeshCollider collider = Box(Vector3d.One, MeshInertiaPolicy.RequireClosedVolume);
        collider.CalculateInertiaTensor(-Fixed64.One, Vector3d.One).Should().Be(Fixed3x3.Zero);
        collider.Mesh.CalculateInertiaTensor(Fixed64.Zero).Should().Be(Fixed3x3.Zero);
        collider.Mesh.CalculateInertiaTensor(-Fixed64.One).Should()
            .Be(BoxTensor(Vector3d.One, -Fixed64.One.m_rawValue, false, Vector3d.Zero, Vector3d.Zero));
        Action invalidPolicy = () => collider.Mesh.CalculateInertiaTensor(Fixed64.Zero, (MeshInertiaPolicy)(-1));
        invalidPolicy.Should().Throw<ArgumentOutOfRangeException>();
        PhysicsMesh open = MeshTestFixtures.CreateConvexQuadFloor().Mesh;
        Action invalidVolume = () => open.CalculateInertiaTensor(Fixed64.Zero);
        invalidVolume.Should().Throw<InvalidOperationException>();
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void RuntimeMesh_ShouldSaturateOnlyTheFinalSignedMoments(bool surface)
    {
        PhysicsMesh mesh = Box(Vector3d.One, Policy(surface)).Mesh;
        foreach (Vector3d reference in new[] {
            new Vector3d(Fixed64.MaxValue, Fixed64.MinValue, Fixed64.Zero),
            new Vector3d(Fixed64.MaxValue, Fixed64.MaxValue, Fixed64.Zero) })
        foreach (long massRaw in new[] { 1L, 2L, -1L, long.MinValue })
            mesh.CalculateInertiaTensor(Fixed64.FromRaw(massRaw), Policy(surface), reference).Should()
                .Be(BoxTensor(Vector3d.One, massRaw, surface, Vector3d.Zero, reference));
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void DetachedScaleFailure_ShouldAllowACompensatingHostScale(bool surface)
    {
        ColliderShapeDefinition definition = MeshTestFixtures.CreateConvexCubeDefinition(Policy(surface));
        Fixed64 large = (Fixed64)1048576;
        var compound = new LSCompoundCollider(new CompoundColliderPart(definition, Vector3d.Zero,
            FixedQuaternion.Identity, Vector3d.One * large));
        Action detached = () => compound.CalculateInertiaTensor(Fixed64.One, Vector3d.Zero);
        detached.Should().Throw<ArgumentException>();
        using var context = GravitasWorldContext.CreateOwned();
        compound.InitializeWithNoBody(new TestMatterAgent(context,
            new FixedTransform(Vector3d.Zero, FixedQuaternion.Identity, Vector3d.One / large)));
        compound.CalculateInertiaTensor((Fixed64)6, Vector3d.Zero).Should()
            .Be(BoxTensor(Vector3d.One, ((Fixed64)6).m_rawValue, surface, Vector3d.Zero, Vector3d.Zero));
    }

    [Fact]
    public void RuntimeMesh_ShouldRetainFractionalSemanticOrigin()
    {
        LSMeshCollider collider = Box(Vector3d.One, MeshInertiaPolicy.RequireClosedVolume);
        collider.LocalOffset = new Vector3d(Fixed64.FromRaw(1), Fixed64.Zero, Fixed64.Zero);
        using var context = GravitasWorldContext.CreateOwned();
        collider.InitializeWithNoBody(new TestMatterAgent(context,
            new FixedTransform(Vector3d.Zero, FixedQuaternion.Identity, Vector3d.One * Fixed64.Half)));
        Vector3d reference = new(Fixed64.One, Fixed64.One, Fixed64.Zero);
        collider.CalculateInertiaTensor(Fixed64.One, reference).Should()
            .Be(BoxTensor(Vector3d.One * Fixed64.Half, Fixed64.One.m_rawValue, false,
                collider.LocalOffset, reference, centerDenominator: 2));
    }

    [Fact]
    public void DetachedUnsupportedSurface_ShouldNotDisappearBesideAValidPart()
    {
        var compound = new LSCompoundCollider(
            new CompoundColliderPart(MeshTestFixtures.CreateConvexCubeDefinition(MeshInertiaPolicy.SurfaceApproximation),
                Vector3d.Zero, FixedQuaternion.Identity, Vector3d.One * (Fixed64)1048576),
            new CompoundColliderPart(ColliderShapeDefinition.Sphere(Fixed64.One), Vector3d.Zero));
        Action calculate = () => compound.CalculateInertiaTensor(Fixed64.FromRaw(1), Vector3d.Zero);
        calculate.Should().Throw<ArgumentException>();
    }

    [Fact]
    public void AlternatingRuntimeInputsAndPolicies_ShouldRemainAllocationFree()
    {
        PhysicsMesh mesh = Box(Vector3d.One, MeshInertiaPolicy.RequireClosedVolume).Mesh;
        long sum = 0;
        for (int i = 0; i < 64; i++)
            sum += mesh.CalculateInertiaTensor((Fixed64)(i % 3 + 1), Policy((i & 1) != 0), Vector3d.One * (Fixed64)(i % 4)).M11.m_rawValue;
        long before = GC.GetAllocatedBytesForCurrentThread();
        for (int i = 0; i < 1024; i++)
            sum += mesh.CalculateInertiaTensor((Fixed64)(i % 3 + 1), Policy((i & 1) != 0), Vector3d.One * (Fixed64)(i % 4)).M11.m_rawValue;
        long allocated = GC.GetAllocatedBytesForCurrentThread() - before;
        allocated.Should().Be(0);
        sum.Should().BePositive();
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void RuntimeMassMutation_ShouldPublishTheFinalNormalizedMeshResponse(bool surface)
    {
        Vector3d sides = Vector3d.One / (Fixed64)128;
        using var context = GravitasWorldContext.CreateOwned();
        var body = new SolidBody(new TestMatterAgent(context), Box(sides, Policy(surface))) { Mass = (Fixed64)6 };
        body.Initialize(Vector3d.Zero, FixedQuaternion.Identity);
        foreach (long massRaw in new[] { ((Fixed64)1000000000).m_rawValue, ((Fixed64)999999999).m_rawValue })
        {
            body.Mass = Fixed64.FromRaw(massRaw);
            Fixed3x3 inertia = BoxTensor(sides, massRaw, surface, Vector3d.Zero, Vector3d.Zero);
            Fixed64 inverse = Rounded(BigInteger.One << 64, inertia.M11.m_rawValue);
            body.InverseInertiaTensor.Should().Be(new Fixed3x3(
                inverse, Fixed64.Zero, Fixed64.Zero, Fixed64.Zero, inverse, Fixed64.Zero,
                Fixed64.Zero, Fixed64.Zero, inverse));
        }
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void PreparedZeroWeightFallback_ShouldPreserveItsHostCandidate(bool readDetached)
    {
        var open = ColliderShapeDefinition.ConvexMesh(
            new[] { Vector3d.Zero, Vector3d.Right, Vector3d.Up }, new[] { 0, 1, 2 });
        var compound = new LSCompoundCollider(
            new CompoundColliderPart(open, Vector3d.Right),
            new CompoundColliderPart(open, Vector3d.Left * (Fixed64)3, FixedQuaternion.Identity, Vector3d.One * Fixed64.Two));
        if (readDetached) compound.CalculateLocalCenterOfMassOffset().Should().Be(Vector3d.Left);
        using var context = GravitasWorldContext.CreateOwned();
        compound.InitializeWithNoBody(new TestMatterAgent(context,
            new FixedTransform(Vector3d.Zero, FixedQuaternion.Identity, Vector3d.One * Fixed64.Two)));
        compound.CalculateLocalCenterOfMassOffset().Should().Be(Vector3d.Left * Fixed64.Two);
        ((LSMeshCollider)compound.GetPartCollider(1)).Mesh.OwnerScale.Should().Be(Vector3d.One * Fixed64.Two);
        Action inertia = () => compound.CalculateInertiaTensor(Fixed64.One, Vector3d.Zero);
        inertia.Should().Throw<InvalidOperationException>();
    }

    [Fact]
    public void RuntimeMesh_ShouldRefreshItsSemanticOriginAndRetainAnUnrepresentableOrigin()
    {
        LSMeshCollider collider = Box(Vector3d.One, MeshInertiaPolicy.RequireClosedVolume);
        collider.CalculateCenterOfMassInertiaTensor((Fixed64)6).Should()
            .Be(BoxTensor(Vector3d.One, ((Fixed64)6).m_rawValue, false, Vector3d.Zero, Vector3d.Zero));
        using var context = GravitasWorldContext.CreateOwned();
        collider.InitializeWithNoBody(new TestMatterAgent(context,
            new FixedTransform(Vector3d.Zero, FixedQuaternion.Identity, Vector3d.One * Fixed64.Two)));
        collider.CalculateInertiaTensor((Fixed64)6, Vector3d.Zero).Should()
            .Be(BoxTensor(Vector3d.One * Fixed64.Two, ((Fixed64)6).m_rawValue, false, Vector3d.Zero, Vector3d.Zero));
        Vector3d limit = Vector3d.Right * Fixed64.MaxValue;
        collider.LocalOffset = limit;
        // The semantic center is twice the scalar limit; its displacement
        // from the requested scalar limit is exactly one limit.
        collider.CalculateInertiaTensor(Fixed64.FromRaw(1), limit).Should()
            .Be(BoxTensor(Vector3d.One * Fixed64.Two, 1, false, limit, Vector3d.Zero));

        LSMeshCollider open = MeshTestFixtures.CreateConvexQuadFloor(inertiaPolicy: MeshInertiaPolicy.RequireClosedVolume);
        open.LocalOffset = new Vector3d((Fixed64)(-1500000000), Fixed64.Zero, Fixed64.Zero);
        Action invalid = () => open.CalculateInertiaTensor(Fixed64.One,
            new Vector3d((Fixed64)1500000000, Fixed64.Zero, Fixed64.Zero));
        invalid.Should().Throw<InvalidOperationException>();
    }

    private static MeshInertiaPolicy Policy(bool surface) => surface
        ? MeshInertiaPolicy.SurfaceApproximation : MeshInertiaPolicy.RequireClosedVolume;

    private static LSMeshCollider Box(Vector3d sides, MeshInertiaPolicy policy)
    {
        ColliderShapeDefinition source = MeshTestFixtures.CreateConvexCubeDefinition(policy);
        Vector3d[] vertices = Enumerable.Range(0, source.MeshVertexCount)
            .Select(i => Vector3d.Multiply(source.GetMeshVertex(i), sides)).ToArray();
        int[] triangles = Enumerable.Range(0, source.MeshTriangleIndexCount).Select(source.GetMeshTriangleIndex).ToArray();
        return new LSMeshCollider(vertices, triangles, MeshColliderMode.Convex, policy);
    }

    private static Fixed3x3 BoxTensor(Vector3d sides, long massRaw, bool surface, Vector3d center, Vector3d reference,
        int centerDenominator = 1)
    {
        BigInteger[] s = Raw(sides), d = Raw(center).Zip(Raw(reference), (x, y) => x - y * centerDenominator).ToArray();
        BigInteger[] area = { s[1] * s[2], s[0] * s[2], s[0] * s[1] };
        BigInteger total = surface ? area.Aggregate(BigInteger.Zero, (sum, value) => sum + value) : BigInteger.One;
        BigInteger denominator = 12 * total * (BigInteger.One << 64) * centerDenominator * centerDenominator;
        Fixed64[] entries = new Fixed64[9];
        for (int row = 0; row < 3; row++)
        for (int col = 0; col < 3; col++)
        {
            BigInteger numerator = -12 * total * d[row] * d[col];
            if (row == col)
            {
                int j = (row + 1) % 3, k = (row + 2) % 3;
                BigInteger j2 = s[j] * s[j], k2 = s[k] * s[k];
                numerator = surface ? area[row] * (j2 + k2) + area[j] * (3 * j2 + k2)
                    + area[k] * (j2 + 3 * k2) : j2 + k2;
                numerator *= centerDenominator * centerDenominator;
                numerator += 12 * total * (d[j] * d[j] + d[k] * d[k]);
            }
            entries[row * 3 + col] = Rounded(massRaw * numerator, denominator);
        }
        return Matrix(entries);
    }

    private static Fixed3x3 SimplexTensor(Fixed64 a, Fixed64 b, Fixed64 c, long massRaw, bool surface, Vector3d reference)
    {
        BigInteger[] lengths = { a.m_rawValue, b.m_rawValue, c.m_rawValue };
        BigInteger[] r = Raw(reference);
        for (int i = 0; i < 3; i++)
            r[i] += Rounded(lengths[i], 2).m_rawValue;
        // Uniform d-simplex: E[x_i]=a_i/(d+1), E[x_i²]=2a_i²/((d+1)(d+2)), E[x_ix_j]=a_ia_j/((d+1)(d+2)).
        int factor = surface ? 4 : 5, divisor = surface ? 12 : 20;
        BigInteger[] second = Enumerable.Range(0, 3)
            .Select(i => 2 * lengths[i] * lengths[i] - 2 * factor * lengths[i] * r[i] + divisor * r[i] * r[i]).ToArray();
        Fixed64[] entries = new Fixed64[9];
        for (int row = 0; row < 3; row++)
        for (int col = 0; col < 3; col++)
        {
            BigInteger n = row == col ? second[(row + 1) % 3] + second[(row + 2) % 3]
                : -lengths[row] * lengths[col] + factor * (lengths[row] * r[col] + lengths[col] * r[row])
                    - divisor * r[row] * r[col];
            entries[row * 3 + col] = Rounded(massRaw * n, divisor * (BigInteger.One << 64));
        }
        return Matrix(entries);
    }

    private static BigInteger[] Raw(Vector3d vector) => new BigInteger[]
        { vector.X.m_rawValue, vector.Y.m_rawValue, vector.Z.m_rawValue };

    private static Fixed3x3 Matrix(Fixed64[] v) => new(v[0], v[1], v[2], v[3], v[4], v[5], v[6], v[7], v[8]);

    private static Fixed64 Rounded(BigInteger numerator, BigInteger denominator)
    {
        BigInteger value = BigInteger.DivRem(BigInteger.Abs(numerator), denominator, out BigInteger remainder);
        if (remainder * 2 > denominator || (remainder * 2 == denominator && !value.IsEven)) value++;
        if (numerator.Sign < 0) value = -value;
        return Fixed64.FromRaw((long)BigInteger.Min(long.MaxValue, BigInteger.Max(long.MinValue, value)));
    }
}
