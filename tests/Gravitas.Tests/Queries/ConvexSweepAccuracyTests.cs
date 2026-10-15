using FixedMathSharp;
using FixedMathSharp.Geometry;
using FluentAssertions;
using Gravitas.Colliders;
using Gravitas.Queries;
using Gravitas.Tests.Support;
using Gravitas.Support;
using GridForge.Configuration;
using Xunit;

namespace Gravitas.Tests.Queries;

public sealed class ConvexSweepAccuracyTests
{
    [Theory]
    [InlineData(ColliderType.Capsule, 1)]
    [InlineData(ColliderType.Capsule, 200)]
    [InlineData(ColliderType.Capsule, 500)]
    [InlineData(ColliderType.Cylinder, 1)]
    [InlineData(ColliderType.Cylinder, 200)]
    [InlineData(ColliderType.Cylinder, 500)]
    [InlineData(ColliderType.Cone, 1)]
    [InlineData(ColliderType.Cone, 200)]
    [InlineData(ColliderType.Cone, 500)]
    public void PointSweep_WithHorizontalThreeFourFiveSection_ShouldPreserveEntryDistance(
        ColliderType shape,
        int scale)
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        Fixed64 radius = (Fixed64)(5 * scale);
        LSCollider target = CreateTarget(context, shape, radius, Vector3d.Zero);
        Fixed64 chordX = GetChordX(target);

        var worker = new ConvexSweepQueryWorker();
        worker.PrepareSphereSource(new Vector3d(chordX, (Fixed64)(3 * scale), -radius),
            Fixed64.Zero, new Vector3d(Fixed64.Zero, Fixed64.Zero, radius * Fixed64.Two));

        worker.TrySweepPreparedSource(target, out Physics3DHit hit).Should().BeTrue();
        AssertEntry(hit, scale);
        if (shape != ColliderType.Cone)
        {
            Vector3d expectedNormal = new(Fixed64.Zero,
                Fixed64.FromFraction(3, 5), -Fixed64.FromFraction(4, 5));
            Vector3d.Dot(hit.Normal, expectedNormal).Should()
                .BeGreaterThan(Fixed64.FromFraction(9999, 10000));
        }
    }

    [Theory]
    [InlineData(ColliderType.Capsule, 0)]
    [InlineData(ColliderType.Capsule, -1)]
    [InlineData(ColliderType.Capsule, 1)]
    [InlineData(ColliderType.Cylinder, 0)]
    [InlineData(ColliderType.Cylinder, -1)]
    [InlineData(ColliderType.Cylinder, 1)]
    [InlineData(ColliderType.Cone, 0)]
    [InlineData(ColliderType.Cone, -1)]
    [InlineData(ColliderType.Cone, 1)]
    public void SphereSweep_WithTranslatedTarget_ShouldPreserveEntryFromBothDirections(
        ColliderType shape,
        int scalarFace)
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        int scale = scalarFace == 0 ? 500 : 200;
        Fixed64 outerRadius = (Fixed64)(5 * scale);
        Fixed64 sourceRadius = (Fixed64)scale * Fixed64.Quarter;
        Vector3d center = scalarFace switch
        {
            -1 => new Vector3d(Fixed64.MinValue + Fixed64.Two, Fixed64.Zero,
                Fixed64.MinValue + outerRadius + Fixed64.One),
            1 => new Vector3d(Fixed64.MaxValue - Fixed64.Two, Fixed64.Zero,
                Fixed64.MaxValue - outerRadius - Fixed64.One),
            _ => new Vector3d((Fixed64)17, (Fixed64)(-11), (Fixed64)23)
        };
        LSCollider target = CreateTarget(context, shape, outerRadius - sourceRadius, center);
        Fixed64 chordX = GetChordX(target);
        var start = new Vector3d(chordX, center.Y + (Fixed64)(3 * scale), center.Z - outerRadius);
        var end = new Vector3d(chordX, start.Y, center.Z + outerRadius);
        var worker = new ConvexSweepQueryWorker();

        // The sphere dilation restores a 5*scale radial section: both directions
        // therefore retain the same 3-4-5 entry, including the cone's base rim.
        worker.PrepareSphereSource(start, sourceRadius, end - start);
        worker.TrySweepPreparedSource(target, out Physics3DHit forward).Should().BeTrue();
        AssertEntry(forward, scale);
        worker.PrepareSphereSource(end, sourceRadius, start - end);
        worker.TrySweepPreparedSource(target, out Physics3DHit reverse).Should().BeTrue();
        AssertEntry(reverse, scale);
        forward.Normal.Z.Should().BeLessThan(Fixed64.Zero);
        reverse.Normal.Z.Should().BeGreaterThan(Fixed64.Zero);
    }

    [Fact]
    public void PublicSphereSweep_WithHorizontalCylinder_ShouldPreserveAnalyticEntry()
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        context.World.TryAddGrid(new GridConfiguration(
            Vector3d.One * (Fixed64)(-8), Vector3d.One * (Fixed64)8), out _).Should().BeTrue();
        LSCollider target = CreateTarget(context, ColliderType.Cylinder,
            (Fixed64)5 - Fixed64.Quarter, Vector3d.Zero);

        context.Query3D.SweepSphere(new Vector3d(Fixed64.Zero, (Fixed64)3, (Fixed64)(-5)),
            Fixed64.Quarter, Vector3d.Forward, (Fixed64)10, out Physics3DHit hit,
            PhysicsLayerMask.FromLayer(0)).Should().BeTrue();

        hit.Collider.Should().BeSameAs(target);
        AssertEntry(hit, 1);
    }

    [Fact]
    public void PointSweep_WithTinyBox_ShouldRetainTolerantInitialContact()
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        Fixed64 size = Fixed64.FromRaw(100000);
        var target = new LSCuboidCollider { Size = Vector3d.One * size };
        target.InitializeWithNoBody(new TestMatterAgent(context,
            new FixedTransform(Vector3d.Zero, FixedQuaternion.Identity, Vector3d.One)));
        var worker = new ConvexSweepQueryWorker();
        worker.PrepareSphereSource(Vector3d.Left * size, Fixed64.Zero, Vector3d.Right);

        worker.TrySweepPreparedSource(target, out Physics3DHit hit).Should().BeTrue();
        hit.Distance.Should().Be(Fixed64.Zero);
        hit.Normal.X.Should().BeLessThan(Fixed64.Zero);
    }

    public static TheoryData<Vector3d[], bool> TetrahedronCases => new()
    {
        { new[] { Vector3d.Right, Vector3d.Up, Vector3d.Forward, -Vector3d.One }, true },
        { new[] { Vector3d.Right * Fixed64.MinIncrement, Vector3d.Up * Fixed64.MinIncrement,
            Vector3d.Forward * Fixed64.MinIncrement, -Vector3d.One * Fixed64.MinIncrement }, true },
        { new[] { Vector3d.Right * Fixed64.MaxValue, Vector3d.Up * Fixed64.MaxValue,
            Vector3d.Forward * Fixed64.MaxValue, -Vector3d.One * Fixed64.MaxValue }, true },
        { new[] { Vector3d.Zero, Vector3d.Up, Vector3d.Forward, Vector3d.Right }, true },
        { new[] { Vector3d.Right, -Vector3d.Right, Vector3d.Up, Vector3d.Forward }, true },
        { new[] { Vector3d.Right, Vector3d.Up, -Vector3d.Right - Vector3d.Up, Vector3d.Forward }, true },
        { new[] { Vector3d.Right, Vector3d.Right * Fixed64.Two, Vector3d.Up, Vector3d.Forward }, false },
        { new[] { Vector3d.Right, Vector3d.Up, -Vector3d.Right, -Vector3d.Up }, false },
        { new[] { Vector3d.One, Vector3d.One, Vector3d.One, Vector3d.One }, false },
        // The captured outside simplex formerly passed four rounded face-product tests.
        { new[] { FromRaw(-1073741824, 11229125, -14972502),
            FromRaw(1073741824, 23061000, -6097502),
            FromRaw(1073741824, -178052125, -156944378),
            FromRaw(-1073741824, -82247750, -85081878) }, false }
    };

    [Theory]
    [MemberData(nameof(TetrahedronCases))]
    public void TetrahedronContainment_ShouldPreserveBoundaryAndDegeneracyAcrossWindings(
        Vector3d[] vertices,
        bool expected)
    {
        for (int a = 0; a < 4; a++)
        for (int b = 0; b < 4; b++)
        for (int c = 0; c < 4; c++)
        {
            if (a == b || a == c || b == c)
                continue;
            int d = 6 - a - b - c;
            ConvexSweepQueryWorker.IsOriginInsideTetrahedron(
                vertices[a], vertices[b], vertices[c], vertices[d]).Should().Be(expected);
        }
    }

    private static Vector3d FromRaw(long x, long y, long z) =>
        new(Fixed64.FromRaw(x), Fixed64.FromRaw(y), Fixed64.FromRaw(z));

    private static LSCollider CreateTarget(
        GravitasWorldContext context,
        ColliderType shape,
        Fixed64 radius,
        Vector3d center)
    {
        LSCollider target = shape switch
        {
            ColliderType.Capsule => new LSCapsuleCollider(),
            ColliderType.Cylinder => new LSCylinderCollider(),
            _ => new LSConeCollider()
        };
        target.Radius = radius;
        target.Size = new Vector3d(radius * Fixed64.Two,
            shape == ColliderType.Capsule ? radius * Fixed64.Two + Fixed64.Two : Fixed64.Two,
            radius * Fixed64.Two);
        var rotation = new FixedQuaternion(Fixed64.Zero, Fixed64.Zero,
            Fixed64.One, Fixed64.One).Normalized;
        (rotation * Vector3d.Up).Normalized.Should().Be(-Vector3d.Right);
        target.InitializeWithNoBody(new TestMatterAgent(context,
            new FixedTransform(center, rotation, Vector3d.One)));
        return target;
    }

    private static Fixed64 GetChordX(LSCollider target)
    {
        if (target is not LSConeCollider cone)
            return target.Center.X;
        FixedSegment.TryGetCenteredAxisEndpoint(cone.Center, cone.WorldAxis,
            cone.Height, false, out Vector3d baseCenter).Should().BeTrue();
        return baseCenter.X;
    }

    private static void AssertEntry(Physics3DHit hit, int scale)
    {
        // The 3-4-5 normal closes at 4/5, converting world separation tolerance
        // into along-chord error; allow one distance-rounding tolerance as well.
        Fixed64 entryTolerance = ConvexSweepQueryWorker.ContactTolerance * Fixed64.FromFraction(5, 4)
            + Fixed64.FromFraction(1, 1_048_576);
        (hit.Distance - (Fixed64)scale).Abs().Should().BeLessThanOrEqualTo(entryTolerance);
    }
}
