using FixedMathSharp;
using FixedMathSharp.Geometry;
using FluentAssertions;
using Gravitas.Colliders;
using Gravitas.CollisionHandling;
using Gravitas.Queries;
using Gravitas.Support;
using Gravitas.Tests.Support;
using SwiftCollections;
using Xunit;

namespace Gravitas.Tests.Queries;

public sealed class PreparedConvexMeshInitialContactTests
{
    public static TheoryData<ColliderType, bool, bool> SourceCases
    {
        get
        {
            var cases = new TheoryData<ColliderType, bool, bool>();
            foreach (ColliderType type in new[]
                { ColliderType.Sphere, ColliderType.Capsule, ColliderType.OBBox,
                    ColliderType.Cylinder, ColliderType.Cone, ColliderType.Mesh })
            {
                cases.Add(type, false, false);
                cases.Add(type, true, false);
                cases.Add(type, true, true);
            }
            return cases;
        }
    }

    [Theory]
    [MemberData(nameof(SourceCases))]
    public void PreparedSourceInitialContact_ShouldRetainExitNormalAndTargetSurfaceWitness(
        ColliderType sourceType, bool closedHull, bool contained)
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        LSMeshCollider target = CreateTarget(closedHull, contained);
        scenario.InitializeStaticCollider(target, Vector3d.Zero);
        LSCollider source = sourceType switch
        {
            ColliderType.Sphere => new LSSphereCollider(),
            ColliderType.Capsule => new LSCapsuleCollider { Size = new Vector3d(1, 2, 1) },
            ColliderType.OBBox => new LSCuboidCollider(),
            ColliderType.Cylinder => new LSCylinderCollider(),
            ColliderType.Cone => new LSConeCollider(),
            _ => MeshTestFixtures.CreateConvexCube()
        };
        Fixed64 sourceHalfHeight = sourceType == ColliderType.Capsule ? Fixed64.One : Fixed64.Half;
        Fixed64 targetHalfExtent = contained ? (Fixed64)4 : closedHull ? Fixed64.Half : Fixed64.Zero;
        Fixed64 height = contained ? Fixed64.One : targetHalfExtent + sourceHalfHeight - Fixed64.Quarter;
        scenario.InitializeStaticCollider(source, Vector3d.Up * height);
        var worker = new ConvexSweepQueryWorker();

        foreach (Vector3d travel in new[] { Vector3d.Up, Vector3d.Down, Vector3d.Right })
        {
            if (source is LSMeshCollider mesh)
                worker.PrepareConvexMeshSource(mesh, travel);
            else
                worker.PreparePrimitiveSource(source, travel);
            worker.TrySweepPreparedSource(target, out Physics3DHit hit).Should().BeTrue();
            AssertGeometricHit(hit, target, closedHull, targetHalfExtent);
        }
    }

    [Theory]
    [InlineData(false, false, false)]
    [InlineData(false, true, false)]
    [InlineData(false, true, true)]
    [InlineData(true, false, false)]
    [InlineData(true, true, false)]
    [InlineData(true, true, true)]
    public void SyntheticSourceInitialContact_ShouldRetainExitNormalAndTargetSurfaceWitness(
        bool circleSlab, bool closedHull, bool contained)
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        LSMeshCollider target = CreateTarget(closedHull, contained);
        scenario.InitializeStaticCollider(target, Vector3d.Zero);
        Fixed64 halfHeight = circleSlab ? Fixed64.Quarter : Fixed64.Half;
        Fixed64 targetHalfExtent = contained ? (Fixed64)4 : closedHull ? Fixed64.Half : Fixed64.Zero;
        Vector3d center = Vector3d.Up * (contained
            ? Fixed64.One
            : targetHalfExtent + halfHeight - Fixed64.FromFraction(1, 8));
        var worker = new ConvexSweepQueryWorker();

        foreach (Vector3d travel in new[] { Vector3d.Up, Vector3d.Down, Vector3d.Right })
        {
            if (circleSlab)
                worker.PrepareCircleSlabSource(center, Fixed64.Half, halfHeight, travel);
            else
                worker.PrepareSphereSource(center, Fixed64.Half, travel);
            worker.TrySweepPreparedSource(target, out Physics3DHit hit).Should().BeTrue();
            AssertGeometricHit(hit, target, closedHull, targetHalfExtent);
        }
    }

    [Theory]
    [InlineData(0)]
    [InlineData(1)]
    [InlineData(2)]
    public void InitialMeshQueries_ShouldRemainAllocationFreeAfterWarmup(int targetKind)
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        LSMeshCollider mesh = CreateTarget(targetKind != 0, false);
        LSCollider target = targetKind == 2
            ? new LSCompoundCollider(new CompoundColliderPart(MeshTestFixtures.CreateDefinition(mesh), Vector3d.Zero))
            : mesh;
        scenario.InitializeStaticCollider(target, Vector3d.Zero);
        var source = new LSCuboidCollider();
        scenario.InitializeStaticCollider(source, Vector3d.Up * (targetKind == 0 ? Fixed64.Quarter : Fixed64.FromFraction(3, 4)));
        var hits = new SwiftList<Physics3DHit>(1);
        Physics3DHit closest = default;
        PhysicsLayerMask layers = PhysicsLayerMask.FromLayer(0);

        AllocationTestHelper.MeasureSteadyState(() =>
        {
            scenario.Context.Query3D.SweepCuboid(source, Vector3d.Up, layers, out closest);
            scenario.Context.Query3D.SweepCuboidAll(source, Vector3d.Up, layers, hits);
        }).Should().Be(0);

        closest.Collider.Should().BeSameAs(target);
        closest.Distance.Should().Be(Fixed64.Zero);
        closest.Normal.Should().Be(Vector3d.Up);
        hits.Count.Should().Be(1);
        hits[0].Should().Be(closest);
    }

    private static LSMeshCollider CreateTarget(bool closedHull, bool contained)
    {
        if (!closedHull)
            return new LSMeshCollider(
                new[] { new Vector3d(-4, 0, -4), new Vector3d(0, 0, 4), new Vector3d(4, 0, -4) },
                new[] { 0, 1, 2 }, MeshColliderMode.Concave, MeshInertiaPolicy.SurfaceApproximation);
        LSMeshCollider cube = MeshTestFixtures.CreateConvexCube();
        if (!contained)
            return cube;
        Vector3d[] vertices = cube.Mesh.LocalVertices.ToArray();
        for (int i = 0; i < vertices.Length; i++)
            vertices[i] *= (Fixed64)8;
        return new LSMeshCollider(vertices, cube.Mesh.Triangles.ToArray());
    }

    [Theory]
    [InlineData(false, false)]
    [InlineData(false, true)]
    [InlineData(true, false)]
    [InlineData(true, true)]
    public void InitialMeshContact_ShouldKeepUnmaterializableWitnessAndGeometricNormal(bool positive, bool cone)
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        Fixed64 offset = positive ? (Fixed64)5 : (Fixed64)(-5);
        Vector3d origin = new(Fixed64.Zero,
            positive ? Fixed64.MaxValue - (Fixed64)4 : Fixed64.MinValue + (Fixed64)4, Fixed64.Zero);
        var target = new LSMeshCollider(
            new[] { new Vector3d((Fixed64)(-4), offset, (Fixed64)(-4)),
                new Vector3d(Fixed64.Zero, offset, (Fixed64)4),
                new Vector3d((Fixed64)4, offset, (Fixed64)(-4)),
                new Vector3d((Fixed64)(-4), -offset, (Fixed64)(-4)),
                new Vector3d(Fixed64.Zero, -offset, (Fixed64)4),
                new Vector3d((Fixed64)4, -offset, (Fixed64)(-4)) },
            new[] { 0, 1, 2, 3, 4, 5 }, MeshColliderMode.Concave, MeshInertiaPolicy.SurfaceApproximation);
        scenario.InitializeStaticCollider(target, origin);
        LSCollider source = cone
            ? new LSConeCollider { Size = new Vector3d(16, 16, 16) }
            : new LSCuboidCollider { Size = new Vector3d(16, 16, 16) };
        scenario.InitializeStaticCollider(source, origin);
        Vector3d exit = positive ? Vector3d.Down : Vector3d.Up;
        var worker = new ConvexSweepQueryWorker();
        worker.PreparePrimitiveSource(source, exit);
        worker.TrySweepPreparedSource(target, out Physics3DHit hit).Should().BeTrue();
        hit.Distance.Should().Be(Fixed64.Zero);
        hit.Normal.Should().Be(exit);
        hit.Anchor.IsValid.Should().BeTrue();
        hit.TryGetPoint(out _).Should().BeFalse();
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void UncertifiedInitialGjkIntersection_ShouldNotInventMeshContact(bool coincidentWitnesses)
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        var target = new LSMeshCollider(new[]
            { new Vector3d(-2, 0, -2), new Vector3d(2, 0, -2), new Vector3d(2, 0, 2) },
            new[] { 0, 1, 2 }, MeshColliderMode.Concave, MeshInertiaPolicy.SurfaceApproximation);
        scenario.InitializeStaticCollider(target, Vector3d.Zero);
        var source = new LSCuboidCollider();
        scenario.InitializeStaticCollider(source,
            new Vector3d(Fixed64.FromFraction(5, 2) + Fixed64.FromRaw(1), Fixed64.Zero, Fixed64.Zero));
        target.Mesh.GetLocalTriangleVertices(0, out Vector3d a, out Vector3d b, out Vector3d c);
        var sourceShape = new ConvexShape(source, Vector3d.Zero);
        var targetShape = new ConvexShape(target, 0, a, b, c);
        targetShape.TryGetInitialMeshContactNormal(sourceShape, out _, out _).Should().BeFalse();
        var targetWitness = new FixedPointAnchor(target.Mesh.Origin, target.Mesh.Rotation,
            coincidentWitnesses ? new Vector3d(2, 0, 0) : a);
        var sourceWitness = coincidentWitnesses ? targetWitness
            : sourceShape.GetSupportAnchor(Vector3d.One);
        var result = ConvexSweepQueryWorker.GjkResult.CreateIntersection(sourceWitness, targetWitness);
        var anchor = new ContactAnchor(targetWitness);
        Vector3d normal = Vector3d.Up;
        // GJK retains the preceding simplex's witnesses when its bounded
        // search reports intersection. A coincident or nonseparating pair is
        // insufficient when complete geometry rejects the initial contact.
        // The public one-raw gap regression separately verifies that a valid
        // separating witness still reports the query's tolerance contact.
        ConvexSweepQueryWorker.TryResolveInitialMeshContact(sourceShape, targetShape,
            result, ref anchor, ref normal).Should().BeFalse();
    }
    [Fact]
    public void ThinCylinderOutsideMeshVertex_ShouldRejectUncertifiedInitialIntersection()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        Fixed64 radius = Fixed64.FromRaw((5L << 29) - 1);
        Vector3d nearest = new(Fixed64.FromFraction(3, 8), Fixed64.Zero,
            Fixed64.Half - Fixed64.MinIncrement);
        Vector3d perpendicular = new(-Fixed64.Half, Fixed64.Zero, Fixed64.FromFraction(3, 8));
        var target = new LSMeshCollider(new[] { nearest,
            nearest * Fixed64.Two + perpendicular,
            nearest * Fixed64.Two - perpendicular },
            new[] { 0, 1, 2 }, MeshColliderMode.Concave, MeshInertiaPolicy.SurfaceApproximation);
        scenario.InitializeStaticCollider(target, Vector3d.Zero);
        var source = new LSCylinderCollider { Radius = radius,
            Size = new Vector3d(Fixed64.One, Fixed64.MinIncrement, Fixed64.One) };
        scenario.InitializeStaticCollider(source, Vector3d.Zero);

        // Radius is about 0.625 units. Both triangle edges leave the nearest
        // vertex outward, and its exact squared radial gap is strictly positive.
        // GJK's rounded radial/axial witnesses nevertheless coincide. Unlike a
        // certified tolerance contact, this cannot establish a geometric normal.
        long x = nearest.X.m_rawValue;
        long z = nearest.Z.m_rawValue;
        long r = radius.m_rawValue;
        checked(x * x + z * z - r * r).Should().Be(1L << 30);
        scenario.Context.Query3D.SweepCylinder(source, Vector3d.Right,
            PhysicsLayerMask.FromLayer(0), out _).Should().BeFalse();
    }

    private static void AssertGeometricHit(Physics3DHit hit, LSMeshCollider target,
        bool closedHull, Fixed64 targetHalfExtent)
    {
        hit.Collider.Should().BeSameAs(target);
        hit.Distance.Should().Be(Fixed64.Zero);
        (hit.Normal - Vector3d.Up).Magnitude.Should().BeLessThanOrEqualTo(Fixed64.Epsilon);
        hit.TryGetPoint(out Vector3d point).Should().BeTrue();
        if (closedHull)
        {
            point.X.Abs().Should().BeLessThanOrEqualTo(targetHalfExtent + Fixed64.Epsilon);
            point.Z.Abs().Should().BeLessThanOrEqualTo(targetHalfExtent + Fixed64.Epsilon);
            point.Y.Should().Be(targetHalfExtent);
        }
        else
        {
            point.Y.Abs().Should().BeLessThanOrEqualTo(Fixed64.Epsilon);
            // These three halfspaces describe the selected triangle, rather
            // than merely its broad-phase box.
            point.Z.Should().BeGreaterThanOrEqualTo((Fixed64)(-4) - Fixed64.Epsilon);
            (point.X * (Fixed64)2 + point.Z).Should().BeLessThanOrEqualTo((Fixed64)4 + Fixed64.Epsilon);
            (-point.X * (Fixed64)2 + point.Z).Should().BeLessThanOrEqualTo((Fixed64)4 + Fixed64.Epsilon);
        }
    }
}
