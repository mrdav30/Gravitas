using FixedMathSharp;
using FluentAssertions;
using Gravitas.Colliders;
using Gravitas.Queries;
using Gravitas.Tests.Support;
using GridForge.Configuration;
using GridForge.Grids.Storage;
using GridForge.Grids.Topology;
using GridForge.Spatial;
using SwiftCollections;
using Xunit;

namespace Gravitas.Tests.MixedDimensions;

public sealed partial class MixedQueryCcdTests
{
    [Fact]
    public void SweepCircleAgainst3DAll_WithExtremeSparseGridSpan_ShouldReturnOrderedHits()
    {
        using GravitasWorldContext context = CreateExtremeSparseGridContext();
        Fixed64 extent = (Fixed64)100_000;
        var nearTarget = CreateBodyless3D(
            context,
            new LSSphereCollider { Radius = Fixed64.One },
            Vector3d.Zero);
        var farTarget = CreateBodyless3D(
            context,
            new LSSphereCollider { Radius = Fixed64.One },
            new Vector3d((Fixed64)10, Fixed64.Zero, Fixed64.Zero));
        var hits = new SwiftList<PhysicsMixedHit>(2);

        int hitCount = context.QueryMixed.SweepCircleAgainst3DAll(
            new Vector2d((Fixed64)(-200_000), Fixed64.Zero),
            new Vector2d((Fixed64)200_000, Fixed64.Zero),
            extent,
            Fixed64.Zero,
            Fixed64.One,
            IncludeLayerZero,
            hits);

        hitCount.Should().Be(2);
        context.QueryMixed.LastQueryCandidateCount.Should().Be(2);
        hits[0].Collider3D.Should().BeSameAs(nearTarget);
        hits[0].Distance.Should().Be((Fixed64)99_999);
        hits[1].Collider3D.Should().BeSameAs(farTarget);
        hits[1].Distance.Should().Be((Fixed64)100_009);
    }

    [Fact]
    public void SweepCircleAgainst3DAll_WithExtremeSparseGridSpan_ShouldNotAllocateAfterWarmup()
    {
        using GravitasWorldContext context = CreateExtremeSparseGridContext();
        _ = CreateBodyless3D(
            context,
            new LSSphereCollider { Radius = Fixed64.One },
            Vector3d.Zero);
        Fixed64 extent = (Fixed64)100_000;
        var hits = new SwiftList<PhysicsMixedHit>(1);

        long allocatedBytes = AllocationTestHelper.MeasureSteadyState(
            () => context.QueryMixed.SweepCircleAgainst3DAll(
                new Vector2d((Fixed64)(-200_000), Fixed64.Zero),
                new Vector2d((Fixed64)200_000, Fixed64.Zero),
                extent,
                Fixed64.Zero,
                Fixed64.One,
                IncludeLayerZero,
                hits),
            warmupIterations: 16,
            stabilizationIterations: 4,
            measurementIterations: 16);

        allocatedBytes.Should().Be(0);
    }

    private static GravitasWorldContext CreateExtremeSparseGridContext()
    {
        GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        context.ApplySettings(new PhysicsSettings(4, null));
        context.Settings.RuntimeMode = PhysicsRuntimeMode.Mixed;

        Fixed64 extent = (Fixed64)100_000;
        GridConfiguration configuration = new(
            new Vector3d(-extent, -extent, -extent),
            new Vector3d(extent, extent, extent),
            topologyMetrics: GridTopologyMetrics.Rectangular(extent),
            storageKind: GridStorageKind.Sparse);
        context.World.TryAddGrid(
                configuration,
                new[] { new VoxelIndex(1, 1, 1) },
                out _)
            .Should()
            .BeTrue();
        return context;
    }
}
