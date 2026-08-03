using FixedMathSharp;
using FluentAssertions;
using Gravitas.Colliders;
using Gravitas.Queries;
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
    public void SweepCircleAgainst3DAll_WithExtremeSparseGridSpan_ShouldReturnExpectedHit()
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
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

        var target = CreateBodyless3D(
            context,
            new LSSphereCollider { Radius = Fixed64.One },
            Vector3d.Zero);
        var hits = new SwiftList<PhysicsMixedHit>();

        int hitCount = context.QueryMixed.SweepCircleAgainst3DAll(
            new Vector2d((Fixed64)(-200_000), Fixed64.Zero),
            new Vector2d((Fixed64)200_000, Fixed64.Zero),
            extent,
            Fixed64.Zero,
            Fixed64.One,
            IncludeLayerZero,
            hits);

        hitCount.Should().Be(1);
        hits[0].Collider3D.Should().BeSameAs(target);
        hits[0].Distance.Should().Be((Fixed64)99_999);
    }
}
