using GridForge.Grids;
using GridForge.Spatial;
using SwiftCollections;
using System;
using System.Reflection;
using Xunit;

namespace Gravitas.Tests.Partitions;

public sealed class RetainedPartitionLifecycleTests
{
    [Fact]
    public void OccupiedColdRentals_ShouldNotRescanExistingRetainedPartitions()
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        var owner = new object();
        var retained = new SwiftList<ProbePartition>();
        var empty = new SwiftList<ProbePartition>();
        var pool = new SwiftStack<ProbePartition>();
        const int count = 32;
        for (int i = 0; i < count; i++)
        {
            Assert.False(RetainedPartitionLifecycle.TryRetireEmptyForReuse(
                empty, pool, context.World, owner,
                _ => Assert.Fail("An occupied partition cannot be released.")));
            var partition = new ProbePartition(owner);
            RetainedPartitionLifecycle.Track(retained, empty, owner, partition, nameof(ProbePartition));
        }

        int emptyChecks = 0;
        for (int i = 0; i < retained.Count; i++)
            emptyChecks += retained[i].EmptyChecks;
        // Registration may inspect each new payload, but not every old one on
        // each rental. This fails on the former n*(n-1)/2 occupied-scene scan.
        Assert.True(emptyChecks <= 2 * count,
            $"Expected linear eligibility checks, observed {emptyChecks} for {count} occupied rentals.");
    }

    [Fact]
    public void OccupiedPartitionsBecomingEmpty_ShouldNotAllocateEligibilityStorage()
    {
        var owner = new object();
        var retained = new SwiftList<ProbePartition>();
        var empty = new SwiftList<ProbePartition>();
        for (int i = 0; i < 32; i++)
            RetainedPartitionLifecycle.Track(retained, empty, owner, new ProbePartition(owner), nameof(ProbePartition));

        long before = GC.GetAllocatedBytesForCurrentThread();
        for (int i = 0; i < retained.Count; i++)
        {
            retained[i].Empty = true;
            RetainedPartitionLifecycle.RefreshEmptyEligibility(empty, owner, retained[i]);
        }
        long allocated = GC.GetAllocatedBytesForCurrentThread() - before;

        Assert.Equal(0, allocated);
        Assert.Equal(retained.Count, empty.Count);
        for (int i = 0; i < empty.Count; i++)
            Assert.Equal(i, empty[i].EmptyIndex);
    }

    [Fact]
    public void EligibilityChanges_ShouldKeepDenseIndicesWithoutDuplicateEntries()
    {
        var owner = new object();
        var retained = new SwiftList<ProbePartition>();
        var empty = new SwiftList<ProbePartition>();
        var first = new ProbePartition(owner) { Empty = true };
        var second = new ProbePartition(owner) { Empty = true };
        var third = new ProbePartition(owner) { Empty = true };
        foreach (ProbePartition partition in new[] { first, second, third })
            RetainedPartitionLifecycle.Track(retained, empty, owner, partition, nameof(ProbePartition));

        RetainedPartitionLifecycle.RefreshEmptyEligibility(empty, owner, second);
        Assert.Equal(3, empty.Count);
        second.Empty = false;
        RetainedPartitionLifecycle.RefreshEmptyEligibility(empty, owner, second);
        Assert.Equal(-1, second.EmptyIndex);
        Assert.Same(third, empty[1]);
        Assert.Equal(1, third.EmptyIndex);
        Assert.Equal(3, retained.Count);

        second.Empty = true;
        second.Allocated = true;
        RetainedPartitionLifecycle.RefreshEmptyEligibility(empty, owner, second);
        Assert.Equal(2, empty.Count);
        second.Allocated = false;
        RetainedPartitionLifecycle.RefreshEmptyEligibility(empty, owner, second);
        Assert.Equal(2, second.EmptyIndex);
        RetainedPartitionLifecycle.RefreshEmptyEligibility(empty, new object(), second);
        Assert.Equal(-1, second.EmptyIndex);
        Assert.Equal(2, empty.Count);

        int cursor = 0;
        RetainedPartitionLifecycle.Untrack(retained, empty, owner, first, ref cursor);
        Assert.Same(third, empty[0]);
        Assert.Equal(0, third.EmptyIndex);
        Assert.Equal(-1, first.EmptyIndex);
        Assert.Single(empty);
    }

    [Theory]
    [InlineData(0)]
    [InlineData(1)]
    [InlineData(2)]
    [InlineData(3)]
    public void Reuse_ShouldDiscardStaleEligibilityBeforeDetaching(int mutation)
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        var owner = new object();
        var retained = new SwiftList<ProbePartition>();
        var empty = new SwiftList<ProbePartition>();
        var pool = new SwiftStack<ProbePartition>();
        var partition = new ProbePartition(owner) { Empty = true };
        RetainedPartitionLifecycle.Track(retained, empty, owner, partition, nameof(ProbePartition));
        switch (mutation)
        {
            case 0: partition.ClearRetainedIndex(); break;
            case 1: partition.OwningObject = new object(); break;
            case 2: partition.Empty = false; break;
            case 3: partition.Allocated = true; break;
        }

        Assert.False(RetainedPartitionLifecycle.TryRetireEmptyForReuse(
            empty, pool, context.World, owner, _ => Assert.Fail("A stale candidate must remain attached.")));
        Assert.Empty(empty);
        Assert.Equal(-1, partition.EmptyIndex);
        Assert.Single(retained);
    }

    [Fact]
    public void StaleCollidingEmptyIndex_ShouldNotRemoveAnotherPartition()
    {
        var owner = new object();
        var retained = new SwiftList<ProbePartition>();
        var empty = new SwiftList<ProbePartition>();
        var tracked = new ProbePartition(owner) { Empty = true };
        RetainedPartitionLifecycle.Track(retained, empty, owner, tracked, nameof(ProbePartition));
        var stale = new ProbePartition(owner) { EmptyIndex = tracked.EmptyIndex };
        RetainedPartitionLifecycle.RefreshEmptyEligibility(empty, owner, stale);
        Assert.Equal(-1, stale.EmptyIndex);
        Assert.Same(tracked, empty[0]);

        // Even a stale in-range slot must be checked by reference before deciding
        // an eligible payload is already present in the dense list.
        stale.Empty = true;
        stale.SetRetainedIndex(1);
        stale.EmptyIndex = tracked.EmptyIndex;
        RetainedPartitionLifecycle.RefreshEmptyEligibility(empty, owner, stale);
        Assert.Equal(2, empty.Count);
        Assert.Equal(1, stale.EmptyIndex);
        Assert.Equal(0, tracked.EmptyIndex);
    }

    [Theory]
    [InlineData(0)]
    [InlineData(1)]
    [InlineData(2)]
    public void OwnedPartitions_ShouldMaintainEligibilityAcrossResetReoccupationAndPooling(int dimension)
    {
        using GravitasWorldContext context = GravitasWorldContext.CreateOwned();
        switch (dimension)
        {
            case 0:
                PhysicsPartition partition3D = context.Collisions.RentPartition();
                VerifyOwnedEligibility(partition3D, context.Collisions, () => partition3D.AddStaticObject(17),
                    () => partition3D.RemoveStaticObject(17), partition3D.ResetRetainedMembership,
                    () => context.Collisions.ReleasePartition(partition3D), context.Collisions.RentPartition);
                break;
            case 1:
                PhysicsPartition2D partition2D = context.Collisions2D.RentPartition();
                VerifyOwnedEligibility(partition2D, context.Collisions2D, () => partition2D.AddStaticObject(17),
                    () => partition2D.RemoveStaticObject(17), partition2D.ResetRetainedMembership,
                    () => context.Collisions2D.ReleasePartition(partition2D), context.Collisions2D.RentPartition);
                break;
            case 2:
                PhysicsMixedPartition mixedPartition = context.MixedCollisions.RentPartition();
                VerifyOwnedEligibility(mixedPartition, context.MixedCollisions, () => mixedPartition.AddStatic3DObject(17),
                    () => mixedPartition.RemoveStatic3DObject(17), mixedPartition.ResetRetainedMembership,
                    () => context.MixedCollisions.ReleasePartition(mixedPartition), context.MixedCollisions.RentPartition);
                break;
        }
    }

    private static void VerifyOwnedEligibility<TPartition, TOwner>(
        TPartition partition, TOwner owner, Action occupy, Action empty, Action reset,
        Action release, Func<TPartition> rent)
        where TPartition : class, IRetainedPhysicsPartition<TOwner>
        where TOwner : class
    {
        owner.GetType().GetMethod("TrackRetainedPartition", BindingFlags.Instance | BindingFlags.NonPublic)!
            .Invoke(owner, new object[] { partition });
        Assert.Equal(0, partition.EmptyIndex);
        reset();
        Assert.Equal(0, partition.EmptyIndex);
        occupy();
        Assert.Equal(-1, partition.EmptyIndex);
        empty();
        Assert.Equal(0, partition.EmptyIndex);
        occupy();
        reset();
        Assert.Equal(0, partition.EmptyIndex);
        release();
        Assert.Equal(-1, partition.EmptyIndex);
        Assert.Equal(-1, partition.RetainedIndex);
        Assert.Same(partition, rent());
        Assert.Equal(-1, partition.EmptyIndex);
    }

    private sealed class ProbePartition : IRetainedPhysicsPartition<object>
    {
        internal ProbePartition(object owner) => OwningObject = owner;

        internal object OwningObject { get; set; }

        internal bool Empty { get; set; }

        internal bool Allocated { get; set; }

        internal int EmptyChecks { get; private set; }

        public int RetainedIndex { get; private set; } = -1;

        public int EmptyIndex { get; set; } = -1;

        public bool IsEmpty { get { EmptyChecks++; return Empty; } }

        public bool IsAllocated => Allocated;

        public long EmptySinceFrame => -1;

        public WorldVoxelIndex WorldIndex { get; set; }

        public bool IsOwnedBy(object owner) => ReferenceEquals(OwningObject, owner);

        public void SetRetainedIndex(int index) => RetainedIndex = index;

        public void ClearRetainedIndex() => RetainedIndex = -1;

        public void SetParentIndex(WorldVoxelIndex index) => WorldIndex = index;

        public void OnAddToVoxel(Voxel voxel) { }

        public void OnRemoveFromVoxel(Voxel voxel) { }
    }
}
