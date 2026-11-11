using System;
using FixedMathSharp;
using FluentAssertions;
using Gravitas.Colliders;
using GridForge.Spatial;
using SwiftCollections;
using Xunit;

namespace Gravitas.Tests.Colliders;

public sealed class ColliderRuntimeStateTests
{
    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void PairState_FirstContact_ShouldPrepareBothRoles(bool holderFirst)
    {
        var state = new ColliderPairState<object>();
        object pair = new();
        state.CollisionPairs.Should().BeNull();
        state.CollisionPairHolders.Should().BeNull();
        if (holderFirst) state.TryAddCollisionPairHolder(7);
        else state.TryAddCollisionPair(7, pair);

        long before = GC.GetAllocatedBytesForCurrentThread();
        if (holderFirst) state.TryAddCollisionPair(8, pair);
        else state.TryAddCollisionPairHolder(8);
        long allocated = GC.GetAllocatedBytesForCurrentThread() - before;

        allocated.Should().Be(0);
        state.CollisionPairCount.Should().Be(1);
        state.CollisionPairHolderCount.Should().Be(1);
        state.TryGetCollisionPair(holderFirst ? 8 : 7, out object? found).Should().BeTrue();
        found.Should().BeSameAs(pair);
    }

    [Fact]
    public void PairState_ShouldHandleEmptyAndAllocatedCollectionsDeterministically()
    {
        var state = new ColliderPairState<object>();
        object pair = new();

        state.TryGetCollisionPair(7, out object? missing).Should().BeFalse();
        missing.Should().BeNull();
        state.TryRemoveCollisionPair(7, out missing).Should().BeFalse();
        missing.Should().BeNull();
        state.TryRemoveCollisionPairHolder(7).Should().BeFalse();

        state.TryAddCollisionPair(7, pair).Should().BeTrue();
        state.TryGetCollisionPair(7, out object? found).Should().BeTrue();
        found.Should().BeSameAs(pair);
        state.TryRemoveCollisionPair(8, out missing).Should().BeFalse();
        state.TryRemoveCollisionPair(7, out found).Should().BeTrue();
        found.Should().BeSameAs(pair);

        state.TryAddCollisionPairHolder(7).Should().BeTrue();
        state.TryAddCollisionPairHolder(7).Should().BeFalse();
        state.TryRemoveCollisionPairHolder(7).Should().BeTrue();
        state.ClearCollisionPairs();
        state.ClearCollisionPairHolders();
    }

    [Fact]
    public void PartitionState2D_ShouldMatchStoredBoundsAndClearCoordinatesSafely()
    {
        var state = new ColliderPartitionState2D();
        Vector2d min = -Vector2d.One;
        Vector2d max = Vector2d.One;

        state.MatchesGridBounds(min, max, partitionKind: 2).Should().BeFalse();
        state.ClearCoordinates();

        state.SetPreviousGridBounds(min, max, partitionKind: 2);
        state.MarkPartitioned();

        state.MatchesGridBounds(min, max, partitionKind: 2).Should().BeTrue();
        state.MatchesGridBounds(min, max, partitionKind: 3).Should().BeFalse();
        state.Coordinates = new SwiftList<WorldVoxelIndex>
        {
            new(1, 0, 2, new VoxelIndex(1, 0, 1))
        };
        state.ClearCoordinates();

        state.Coordinates.Count.Should().Be(0);
    }
}
