using FixedMathSharp;
using FluentAssertions;
using Gravitas.Colliders;
using Gravitas.Materials;
using Gravitas.Queries;
using Gravitas.Support;
using Gravitas.Tests.Support;
using SwiftCollections;
using Xunit;

namespace Gravitas.Tests.CollisionHandlingTests;

public sealed class TerrainChunkMigrationTests
{
    private readonly ITestOutputHelper _output;

    public TerrainChunkMigrationTests(ITestOutputHelper output) => _output = output;

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void FrictionlessBodyWithinChunkInterior_ShouldMatchContinuousFloor(bool capsule)
    {
        using PhysicsScenarioBuilder continuous = PhysicsScenarioBuilder.Create();
        using PhysicsScenarioBuilder chunked = PhysicsScenarioBuilder.Create();
        SolidBody control = CreateScene(continuous, false, capsule);
        SolidBody migrated = CreateScene(chunked, true, capsule);
        Fixed64 maxHeightDifference = Fixed64.Zero;
        int peakControlPairs = 0, peakChunkPairs = 0;
        for (int frame = 0; frame < 16; frame++)
        {
            continuous.Context.Simulate();
            continuous.Context.LateSimulate();
            chunked.Context.Simulate();
            chunked.Context.LateSimulate();

            control.LinearVelocity.X.Should().Be(Fixed64.One);
            migrated.LinearVelocity.X.Should().Be(control.LinearVelocity.X);
            migrated.Position3d.X.Should().Be(control.Position3d.X);
            migrated.Position3d.Z.Should().Be(Fixed64.Zero);
            migrated.AngularVelocity.Should().Be(Vector3d.Zero);
            migrated.Position3d.Y.Should().BeGreaterThan(Fixed64.Zero);
            maxHeightDifference = FixedMath.Max(maxHeightDifference,
                (migrated.Position3d.Y - control.Position3d.Y).Abs());
            peakControlPairs = System.Math.Max(peakControlPairs,
                control.Collider.CollisionPairCount + control.Collider.CollisionPairHolderCount);
            peakChunkPairs = System.Math.Max(peakChunkPairs,
                migrated.Collider.CollisionPairCount + migrated.Collider.CollisionPairHolderCount);
        }
        maxHeightDifference.Should().Be(Fixed64.Zero);
        migrated.Position3d.X.Should().Be((Fixed64)(-1));
        peakControlPairs.Should().Be(1);
        peakChunkPairs.Should().Be(2, "nearby chunks retain distinct broad-phase pair identities even before the seam");
        _output.WriteLine($"capsule={capsule}; peak pairs={peakControlPairs}/{peakChunkPairs}; max height difference={maxHeightDifference}");
    }

    [Fact]
    public void SphereSweepAtChunkBoundary_ShouldRetainPhysicalHitAndDistinctColliderResults()
    {
        using PhysicsScenarioBuilder continuous = PhysicsScenarioBuilder.Create();
        using PhysicsScenarioBuilder chunked = PhysicsScenarioBuilder.Create();
        AddFloor(continuous, false);
        AddFloor(chunked, true);
        var controlHits = new SwiftList<Physics3DHit>();
        var chunkHits = new SwiftList<Physics3DHit>();
        Vector3d start = new(0, 2, 0), end = new(0, -2, 0);
        var mask = PhysicsLayerMask.FromLayer(0);

        continuous.Context.Query3D.SweepSphereAll(start, end, Fixed64.Half, mask, controlHits).Should().Be(1);
        chunked.Context.Query3D.SweepSphereAll(start, end, Fixed64.Half, mask, chunkHits).Should().Be(2);

        for (int i = 0; i < chunkHits.Count; i++)
        {
            chunkHits[i].Distance.Should().Be(controlHits[0].Distance);
            chunkHits[i].Normal.Should().Be(controlHits[0].Normal);
            chunkHits[i].Point.Should().Be(controlHits[0].Point);
        }
        controlHits[0].Distance.Should().Be(Fixed64.One + Fixed64.Half);
        controlHits[0].Point.Should().Be(Vector3d.Zero);
        controlHits[0].Normal.Should().Be(Vector3d.Up);
        chunkHits[0].Collider.Should().NotBeSameAs(chunkHits[1].Collider);
    }

    private static SolidBody CreateScene(PhysicsScenarioBuilder scene, bool chunks, bool capsule)
    {
        scene.Context.SetFrameRate(16);
        scene.Context.Environment.Gravity = Fixed64.One;
        scene.Context.Environment.AirDensity = Fixed64.Zero;
        scene.Context.Environment.DampingFactor = Fixed64.Zero;
        scene.Context.Environment.MinSpeed = Fixed64.Zero;
        AddFloor(scene, chunks, staticBodies: true);
        LSCollider collider = capsule
            ? new LSCapsuleCollider { Radius = Fixed64.Half, Size = new(1, 2, 1) }
            : new LSSphereCollider { Radius = Fixed64.Half };
        collider.Material = PhysicsMaterial.Frictionless;
        SolidBody body = scene.CreateBody(collider, new Vector3d((Fixed64)(-2), capsule ? Fixed64.One : Fixed64.Half, Fixed64.Zero),
            FixedQuaternion.Identity, preventAngularForces: true).Body;
        body.UseManualGrounding();
        body.SleepEnabled = false;
        body.AddLinearImpulse(Vector3d.Right);
        return body;
    }

    private static void AddFloor(PhysicsScenarioBuilder scene, bool chunks, bool staticBodies = false)
    {
        Vector3d[] vertices = { new(-8, 0, -4), new(0, 0, -4), new(8, 0, -4),
            new(-8, 0, 4), new(0, 0, 4), new(8, 0, 4) };
        if (!chunks)
        {
            var mesh = new LSMeshCollider(vertices, new[] { 0, 3, 1, 1, 3, 4, 1, 4, 2, 2, 4, 5 },
                MeshColliderMode.Concave, MeshInertiaPolicy.SurfaceApproximation)
                { Material = PhysicsMaterial.Frictionless };
            InitializeFloor(scene, mesh, staticBodies);
            return;
        }
        for (int i = 0; i < 2; i++)
        {
            var mesh = new LSMeshCollider(new[] { vertices[i], vertices[i + 1], vertices[i + 3], vertices[i + 4] },
                new[] { 0, 2, 1, 1, 2, 3 }, MeshColliderMode.Concave, MeshInertiaPolicy.SurfaceApproximation)
                { Material = PhysicsMaterial.Frictionless };
            InitializeFloor(scene, mesh, staticBodies);
        }
    }

    private static void InitializeFloor(PhysicsScenarioBuilder scene, LSMeshCollider mesh, bool staticBodies)
    {
        if (staticBodies)
            scene.CreateBody(mesh, Vector3d.Zero, FixedQuaternion.Identity, immovable: true);
        else
            scene.InitializeStaticCollider(mesh, Vector3d.Zero);
    }
}
