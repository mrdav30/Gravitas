using FixedMathSharp;
using FluentAssertions;
using Gravitas.Colliders;
using Gravitas.Queries;
using Gravitas.Support;
using Gravitas.Tests.Support;
using SwiftCollections;
using Xunit;

namespace Gravitas.Tests.Queries;

public sealed class MeshSphereSweepContactTests
{
    private static readonly PhysicsLayerMask LayerZero = PhysicsLayerMask.FromLayer(0);

    [Theory]
    [InlineData(MeshColliderMode.Convex, false, -1)]
    [InlineData(MeshColliderMode.Convex, false, 1)]
    [InlineData(MeshColliderMode.Convex, true, -1)]
    [InlineData(MeshColliderMode.Convex, true, 1)]
    [InlineData(MeshColliderMode.Concave, false, -1)]
    [InlineData(MeshColliderMode.Concave, false, 1)]
    [InlineData(MeshColliderMode.Concave, true, -1)]
    [InlineData(MeshColliderMode.Concave, true, 1)]
    public void InitialMeshFaceOverlap_ShouldPreserveSourceSideForEitherWindingAndTravel(
        MeshColliderMode mode, bool reverseWinding, int side)
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        LSMeshCollider mesh = AddQuad(scenario, mode, reverseWinding, Vector3d.Zero);
        Vector3d normal = Vector3d.Up * (Fixed64)side;
        Vector3d start = normal * Fixed64.Quarter;

        foreach (Vector3d direction in new[] { normal, -normal, Vector3d.Right })
            AssertInitialHit(scenario.Context, mesh, start, direction, Fixed64.Half, Vector3d.Zero, normal);
    }

    [Theory]
    [InlineData(false, -1)]
    [InlineData(false, 0)]
    [InlineData(false, 1)]
    [InlineData(true, -1)]
    [InlineData(true, 0)]
    [InlineData(true, 1)]
    public void InitialMeshFaceOverlap_OnlyExactCoincidenceShouldUseAuthoredFace(
        bool reverseWinding, int rawGap)
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        LSMeshCollider mesh = AddQuad(scenario, MeshColliderMode.Concave, reverseWinding, Vector3d.Zero);
        Vector3d expected = rawGap == 0
            ? (reverseWinding ? Vector3d.Down : Vector3d.Up)
            : Vector3d.Up * (Fixed64)rawGap;
        Vector3d start = new(Fixed64.Zero, Fixed64.FromRaw(rawGap), Fixed64.Zero);

        foreach (Vector3d direction in new[] { Vector3d.Up, Vector3d.Down, Vector3d.Right })
            AssertInitialHit(scenario.Context, mesh, start, direction, Fixed64.Half, Vector3d.Zero, expected);
    }

    [Theory]
    [InlineData(MeshColliderMode.Convex, false)]
    [InlineData(MeshColliderMode.Convex, true)]
    [InlineData(MeshColliderMode.Concave, false)]
    [InlineData(MeshColliderMode.Concave, true)]
    public void InitialMeshEdgeAndVertexOverlap_ShouldReportRadialFeatureNormals(
        MeshColliderMode mode, bool reverseWinding)
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        LSMeshCollider mesh = AddQuad(scenario, mode, reverseWinding, Vector3d.Zero);
        Vector3d edgePoint = Vector3d.Right * (Fixed64)2;
        Vector3d edgeOffset = new(Fixed64.FromFraction(3, 10), Fixed64.FromFraction(4, 10), Fixed64.Zero);
        AssertInitialHit(scenario.Context, mesh, edgePoint + edgeOffset, Vector3d.Right,
            Fixed64.One, edgePoint, edgeOffset.Normalized);

        Vector3d vertex = new(2, 0, 2);
        Vector3d vertexOffset = new(Fixed64.Quarter, Fixed64.Zero, Fixed64.Quarter);
        AssertInitialHit(scenario.Context, mesh, vertex + vertexOffset, Vector3d.Forward,
            Fixed64.Half, vertex, vertexOffset.Normalized);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void ClosedConvexMeshSurfaceOverlap_ShouldRetainInteriorAndExteriorSourceSides(bool inside)
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        LSMeshCollider mesh = MeshTestFixtures.CreateConvexCube();
        scenario.InitializeStaticCollider(mesh, Vector3d.Zero);
        Vector3d start = Vector3d.Up * (inside ? Fixed64.Quarter : Fixed64.FromFraction(3, 4));
        Vector3d expected = inside ? Vector3d.Down : Vector3d.Up;

        // Sphere/mesh sweeps query the two-sided triangle surface, including a
        // center on the interior side. This does not invent solid-hull admission.
        AssertInitialHit(scenario.Context, mesh, start, Vector3d.Up, Fixed64.Half,
            Vector3d.Up * Fixed64.Half, expected);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void CompoundMeshSurfaceTie_ShouldRetainFirstAuthoredPartWitness(bool ceilingFirst)
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        ColliderShapeDefinition quad = MeshTestFixtures.CreateDefinition(MeshTestFixtures.CreateConvexQuadFloor());
        CompoundColliderPart floor = new(quad, Vector3d.Down * Fixed64.Quarter);
        CompoundColliderPart ceiling = new(quad, Vector3d.Up * Fixed64.Quarter);
        var compound = new LSCompoundCollider(ceilingFirst ? ceiling : floor, ceilingFirst ? floor : ceiling);
        scenario.InitializeStaticCollider(compound, Vector3d.Zero);
        Vector3d start = Vector3d.Zero;

        AssertInitialHit(scenario.Context, compound, start, Vector3d.Right, Fixed64.Half,
            ceilingFirst ? Vector3d.Up * Fixed64.Quarter : Vector3d.Down * Fixed64.Quarter,
            ceilingFirst ? Vector3d.Down : Vector3d.Up);
    }

    [Fact]
    public void EqualDistanceMeshTargets_ShouldOrderClosestAndAllHitsByColliderId()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        LSMeshCollider first = AddQuad(scenario, MeshColliderMode.Concave, false, Vector3d.Zero);
        LSMeshCollider second = AddQuad(scenario, MeshColliderMode.Concave, true, Vector3d.Zero);
        var hits = new SwiftList<Physics3DHit>();
        Vector3d start = Vector3d.Up * Fixed64.Quarter;

        scenario.Context.Query3D.SweepSphere(start, Fixed64.Half, Vector3d.Up, Fixed64.One,
            out Physics3DHit closest, LayerZero).Should().BeTrue();
        scenario.Context.Query3D.SweepSphereAll(start, start + Vector3d.Up, Fixed64.Half, LayerZero, hits)
            .Should().Be(2);

        closest.Collider.Should().BeSameAs(first);
        hits[0].Collider.Should().BeSameAs(first);
        hits[1].Collider.Should().BeSameAs(second);
        hits[0].Should().Be(closest);
        hits[0].Normal.Should().Be(Vector3d.Up);
        hits[1].Normal.Should().Be(Vector3d.Up);
    }

    [Theory]
    [InlineData(MeshColliderMode.Convex, false)]
    [InlineData(MeshColliderMode.Convex, true)]
    [InlineData(MeshColliderMode.Concave, false)]
    [InlineData(MeshColliderMode.Concave, true)]
    public void SweptGroundProbe_ShouldRejectOverheadMeshAndSelectFartherFloor(
        MeshColliderMode mode, bool reverseWinding)
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        SolidBody body = scenario.CreateSphere(Vector3d.Zero).Body;
        body.GroundProbeMode = GroundProbeMode.SweptSphere;
        body.GroundProbeRadius = Fixed64.Half;
        body.GroundOriginOffset = Fixed64.Half;
        body.GroundedDistanceRay = body.GroundDownDistanceOnAir = (Fixed64)2;
        _ = AddQuad(scenario, mode, reverseWinding, Vector3d.Up * Fixed64.FromFraction(3, 4), bodyBacked: true);

        body.CheckGround();
        body.IsGrounded.Should().BeFalse();
        body.GroundNormal.Should().Be(Vector3d.Zero);

        LSMeshCollider floor = AddQuad(scenario, mode, reverseWinding, Vector3d.Down * Fixed64.FromFraction(3, 4));

        body.CheckGround();

        body.IsGrounded.Should().BeTrue();
        body.HitPlatform.Should().BeSameAs(floor.Transform);
        body.HitPoint.Should().Be(Vector3d.Down * Fixed64.FromFraction(3, 4));
        body.GroundNormal.Should().Be(Vector3d.Up);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void MeshSphereSweeps_ShouldRemainAllocationFreeAfterWarmup(bool compoundTarget)
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        LSCollider target = compoundTarget
            ? new LSCompoundCollider(new CompoundColliderPart(
                MeshTestFixtures.CreateDefinition(MeshTestFixtures.CreateConvexQuadFloor()), Vector3d.Zero))
            : MeshTestFixtures.CreateConvexQuadFloor(MeshColliderMode.Concave);
        scenario.InitializeStaticCollider(target, Vector3d.Zero);
        Vector3d start = Vector3d.Up * Fixed64.Quarter;
        var hits = new SwiftList<Physics3DHit>();
        Physics3DHit closest = default;

        AllocationTestHelper.MeasureSteadyState(() =>
        {
            scenario.Context.Query3D.SweepSphere(start, Fixed64.Half, Vector3d.Up, Fixed64.One,
                out closest, LayerZero);
            scenario.Context.Query3D.SweepSphereAll(start, start + Vector3d.Up, Fixed64.Half, LayerZero, hits);
        }).Should().Be(0);

        closest.Collider.Should().BeSameAs(target);
        closest.Normal.Should().Be(Vector3d.Up);
        hits.Count.Should().Be(1);
        hits[0].Should().Be(closest);
    }

    private static LSMeshCollider AddQuad(PhysicsScenarioBuilder scenario, MeshColliderMode mode,
        bool reverseWinding, Vector3d position, bool bodyBacked = false)
    {
        LSMeshCollider mesh = MeshTestFixtures.CreateConvexQuadFloor(mode);
        var rotation = reverseWinding
            ? new FixedQuaternion(Fixed64.One, Fixed64.Zero, Fixed64.Zero, Fixed64.Zero)
            : FixedQuaternion.Identity;
        var agent = new TestMatterAgent(scenario.Context, new FixedTransform(position, rotation, Vector3d.One));
        if (bodyBacked)
            new SolidBody(agent, mesh).Initialize(position, rotation, BodyMotionType.Static);
        else
            mesh.InitializeWithNoBody(agent);
        return mesh;
    }

    private static void AssertInitialHit(GravitasWorldContext context, LSCollider target,
        Vector3d start, Vector3d direction, Fixed64 radius, Vector3d point, Vector3d normal)
    {
        var hits = new SwiftList<Physics3DHit>();
        context.Query3D.SweepSphere(start, radius, direction, Fixed64.One,
            out Physics3DHit closest, LayerZero).Should().BeTrue();
        context.Query3D.SweepSphereAll(start, start + direction, radius, LayerZero, hits).Should().Be(1);

        closest.Collider.Should().BeSameAs(target);
        closest.Distance.Should().Be(Fixed64.Zero);
        closest.Point.Should().Be(point);
        (closest.Normal - normal).Magnitude.Should().BeLessThanOrEqualTo(Fixed64.Epsilon);
        hits[0].Should().Be(closest);
    }
}
