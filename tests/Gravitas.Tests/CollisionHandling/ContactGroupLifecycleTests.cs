using Chronicler.Hashing;
using FixedMathSharp;
using FluentAssertions;
using Gravitas.CollisionHandling;
using Gravitas.Tests.Support;
using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using Xunit;

namespace Gravitas.Tests.CollisionHandlingTests;

public sealed class ContactGroupLifecycleTests
{
    [Fact]
    public void NarrowPhaseRegeneration_WithoutResponse_ShouldRetireVanishedWarmStarts()
    {
        using var scenario = PhysicsScenarioBuilder.Create();
        var first = scenario.CreateSphere(Vector3d.Zero);
        var second = scenario.CreateSphere(Vector3d.Right * Fixed64.Half);
        CollisionPair pair = scenario.CreatePair(first.Collider, second.Collider);
        CollisionDetection.DoCollisionCheck(pair).Should().BeTrue();
        ulong id = pair.Manifold[0].ContactId;
        pair.StoreWarmStartImpulse(id, pair.Manifold[0].Normal, Fixed64.One, Fixed64.Zero);
        first.Body.FreezeAxes = second.Body.FreezeAxes = BodyFreezeAxes3D.All;
        second.Body.SetPosition(Vector3d.Right * (Fixed64)4);
        second.Collider.RebuildRuntimeShapeOnly();

        CollisionDetection.DoCollisionCheck(pair).Should().BeFalse();

        pair.TryGetWarmStartImpulse(id, out _).Should().BeFalse();
        second.Body.SetPosition(Vector3d.Right * Fixed64.Half);
        second.Collider.RebuildRuntimeShapeOnly();
        CollisionDetection.DoCollisionCheck(pair).Should().BeTrue();
        pair.TryGetWarmStartImpulse(id, out _).Should().BeFalse();
    }

    [Fact]
    public void GroupReduction_ShouldPreferClampedDepthBeforeContactIdentity()
    {
        var manifold = new ContactManifold();
        for (ulong id = 1; id <= 5; id++)
            manifold.AddContact(default, new ManifoldContact(id, Vector3d.Zero, Vector3d.Zero,
                Fixed64.MaxValue, Vector3d.Up, depthIsClamped: id == 2 || id >= 4));

        manifold.Select(contact => contact.ContactId).Should().Equal(1UL, 2UL, 4UL, 5UL);
    }

    [Fact]
    public void IndependentGroups_ShouldKeepFourSamplesEachInCanonicalProvenanceOrder()
    {
        ContactGroupKey[] keys = CanonicalKeys();
        var forward = new ContactManifold();
        var reversed = new ContactManifold();
        for (int step = 0; step < keys.Length; step++)
        {
            AddSamples(forward, keys[step], reverse: false);
            AddSamples(reversed, keys[keys.Length - 1 - step], reverse: true);
        }

        forward.GroupCount.Should().Be(9);
        forward.Count.Should().Be(36);
        forward.ToArray().Should().Equal(reversed.ToArray());
        for (int group = 0; group < keys.Length; group++)
        {
            forward.GetGroup(group).Key.CompareTo(keys[group]).Should().Be(0);
            forward.GetGroupStartIndex(group).Should().Be(group * 4);
            forward.GetGroupContactCount(group).Should().Be(4);
            forward.Skip(group * 4).Take(4).Select(contact => contact.ContactId)
                .Should().Equal(2UL, 3UL, 4UL, 5UL);
        }
    }

    [Fact]
    public void DeepSamples_ShouldNeverEvictAnIndependentShallowSupport()
    {
        var manifold = new ContactManifold();
        ContactGroupKey shallow = new(0, 0, 1);
        ContactGroupKey deep = new(0, 0, 2);
        ManifoldContact support = Sample(42, Fixed64.FromFraction(1, 100));
        manifold.AddContact(shallow, support);
        for (ulong id = 1; id <= 8; id++)
            manifold.AddContact(deep, Sample(id, (Fixed64)(int)id));

        manifold.Count.Should().Be(5);
        manifold[0].Should().Be(support);
        manifold.GetGroupContactCount(0).Should().Be(1);
        manifold.GetGroupStartIndex(1).Should().Be(1);
        manifold.Skip(1).Select(contact => contact.ContactId).Should().Equal(5UL, 6UL, 7UL, 8UL);
        manifold.PrimaryContact.ContactId.Should().Be(8);
        // A duplicate feature is replaced only by a deeper observation within its own group.
        manifold.AddContact(shallow, Sample(42, Fixed64.One));
        manifold.AddContact(shallow, Sample(42, Fixed64.Half));
        manifold.Count.Should().Be(5);
        manifold[0].Depth.Should().Be(Fixed64.One);
    }

    [Fact]
    public void GroupEnumeration_ShouldCopyAndResetAcrossDifferentGroupSizes()
    {
        var manifold = new ContactManifold();
        manifold.AddContact(new ContactGroupKey(1, 0), Sample(10, Fixed64.One));
        manifold.AddContact(new ContactGroupKey(2, 0), Sample(30, Fixed64.One));
        manifold.AddContact(new ContactGroupKey(2, 0), Sample(20, Fixed64.One));
        manifold.BeginUpdate(11);
        manifold.LastUpdatedFrame.Should().Be(11);
        manifold.Count.Should().Be(0);
        manifold.GroupCount.Should().Be(0);
        for (int group = 3; group >= 1; group--)
            manifold.AddContact(new ContactGroupKey(group, 0), Sample((ulong)group, Fixed64.One));

        ContactManifold.Enumerator original = manifold.GetEnumerator();
        Assert.Throws<IndexOutOfRangeException>(() => original.Current);
        original.MoveNext().Should().BeTrue();
        ContactManifold.Enumerator copy = original;
        original.MoveNext().Should().BeTrue();
        original.Current.ContactId.Should().Be(2);
        copy.Current.ContactId.Should().Be(1);
        copy.MoveNext().Should().BeTrue();
        copy.Current.Should().Be(original.Current);
        copy.Reset();
        copy.MoveNext().Should().BeTrue();
        copy.Current.ContactId.Should().Be(1);
        copy.Dispose();

        using IEnumerator<ManifoldContact> generic = ((IEnumerable<ManifoldContact>)manifold).GetEnumerator();
        IEnumerator nongeneric = ((IEnumerable)manifold).GetEnumerator();
        for (int index = 0; index < manifold.Count; index++)
        {
            generic.MoveNext().Should().BeTrue();
            nongeneric.MoveNext().Should().BeTrue();
            generic.Current.Should().Be(manifold[index]);
            nongeneric.Current.Should().Be(manifold[index]);
        }
        generic.MoveNext().Should().BeFalse();
        nongeneric.MoveNext().Should().BeFalse();
        nongeneric.Reset();
        nongeneric.MoveNext().Should().BeTrue();
        nongeneric.Current.Should().Be(manifold[0]);

        manifold.Reset();
        manifold.LastUpdatedFrame.Should().Be(-1);
        manifold.HasContact.Should().BeFalse();
        manifold.GroupCount.Should().Be(0);
        manifold.GetEnumerator().MoveNext().Should().BeFalse();
        manifold.SetContact(Vector3d.Zero, Vector3d.Right, Fixed64.One, Vector3d.Right);
        manifold.Count.Should().Be(1);
        manifold.GroupCount.Should().Be(1);
    }

    [Theory]
    [InlineData(-1)]
    [InlineData(2)]
    public void GroupInspection_ShouldRejectIndicesOutsideActiveGroups(int index)
    {
        var manifold = new ContactManifold();
        manifold.AddContact(new ContactGroupKey(1, 0), Sample(1, Fixed64.One));
        manifold.AddContact(new ContactGroupKey(2, 0), Sample(2, Fixed64.One));

        Assert.Throws<IndexOutOfRangeException>(() => manifold.GetGroupStartIndex(index));
        Assert.Throws<IndexOutOfRangeException>(() => manifold.GetGroupContactCount(index));
        Assert.Throws<IndexOutOfRangeException>(() => manifold[index]);
        Assert.Throws<ArgumentOutOfRangeException>(() => manifold.ReserveGroups(-1));
    }

    [Fact]
    public void WarmStartGroups_ShouldIsolateEqualFeaturesAndRemoveOnlyTheirOwnImpulses()
    {
        var cache = new ContactWarmStartCache();
        for (int group = 8; group >= 0; group--)
            cache.Set(new ContactGroupKey(group, 0), 42, Vector3d.Up,
                (Fixed64)(group + 1), Fixed64.Half, Fixed64.Quarter);

        cache.Count.Should().Be(9);
        cache.GroupCount.Should().Be(9);
        for (int group = 0; group < 9; group++)
        {
            ContactGroupKey key = new(group, 0);
            cache.TryGet(key, 42, out ContactWarmStartImpulse impulse).Should().BeTrue();
            impulse.NormalImpulse.Should().Be((Fixed64)(group + 1));
            impulse.TangentImpulse.Should().Be(Fixed64.Half);
            impulse.SecondaryTangentImpulse.Should().Be(Fixed64.Quarter);
            cache.GetGroup(group).GetContactId(0).Should().Be(42);
            cache.GetGroup(group).GetImpulseUnchecked(0).Should().Be(impulse);
        }

        cache.Set(new ContactGroupKey(4, 0), 42, Vector3d.Right,
            Fixed64.Two, -Fixed64.Half, -Fixed64.Quarter);
        cache.TryGet(new ContactGroupKey(4, 0), 42, out ContactWarmStartImpulse changed).Should().BeTrue();
        changed.Normal.Should().Be(Vector3d.Right);
        changed.NormalImpulse.Should().Be(Fixed64.Two);
        changed.TangentImpulse.Should().Be(-Fixed64.Half);
        changed.SecondaryTangentImpulse.Should().Be(-Fixed64.Quarter);
        cache.Remove(new ContactGroupKey(4, 0), 99).Should().BeFalse();
        cache.Remove(new ContactGroupKey(99, 0), 42).Should().BeFalse();
        cache.TryGet(new ContactGroupKey(99, 0), 42, out _).Should().BeFalse();
        foreach (int group in new[] { 0, 8, 4 })
        {
            cache.Remove(new ContactGroupKey(group, 0), 42).Should().BeTrue();
            cache.TryGet(new ContactGroupKey(group, 0), 42, out _).Should().BeFalse();
        }
        cache.Count.Should().Be(6);
        cache.GroupCount.Should().Be(6);
        cache.TryGet(42, out _).Should().BeFalse();
    }

    [Theory]
    [InlineData(5)]
    [InlineData(50)]
    public void FullStaleWarmStartGroup_ShouldInsertNewFeatureInBoundedCanonicalOrder(int newFeature)
    {
        var cache = new ContactWarmStartCache();
        foreach (ulong id in new[] { 40UL, 20UL, 10UL, 30UL })
            cache.Set(id, Vector3d.Up, (Fixed64)(int)id, Fixed64.Half, Fixed64.Quarter);

        cache.Set((ulong)newFeature, Vector3d.Right,
            (Fixed64)newFeature, -Fixed64.Half, -Fixed64.Quarter);

        cache.GroupCount.Should().Be(1);
        cache.Count.Should().Be(4);
        ContactWarmStartGroup group = cache.GetGroup(0);
        ulong[] expected = newFeature == 5
            ? new[] { 5UL, 10UL, 20UL, 30UL }
            : new[] { 10UL, 20UL, 30UL, 50UL };
        for (int index = 0; index < expected.Length; index++)
        {
            ulong id = expected[index];
            group.GetContactId(index).Should().Be(id);
            cache.TryGet(id, out ContactWarmStartImpulse impulse).Should().BeTrue();
            impulse.NormalImpulse.Should().Be((Fixed64)(int)id);
            impulse.Normal.Should().Be(id == (ulong)newFeature ? Vector3d.Right : Vector3d.Up);
            impulse.TangentImpulse.Should().Be(id == (ulong)newFeature ? -Fixed64.Half : Fixed64.Half);
            impulse.SecondaryTangentImpulse.Should().Be(id == (ulong)newFeature ? -Fixed64.Quarter : Fixed64.Quarter);
        }
        cache.TryGet(40, out _).Should().BeFalse();
    }

    [Fact]
    public void Regeneration_ShouldPruneAbsentGroupsAndFeaturesWithoutReassigningImpulses()
    {
        var manifold = new ContactManifold();
        var cache = new ContactWarmStartCache();
        for (int group = 0; group < 9; group++)
            for (ulong id = 1; id <= 4; id++)
                cache.Set(new ContactGroupKey(group, 0), id, Vector3d.Up,
                    (Fixed64)(group * 10 + (int)id), Fixed64.Zero, Fixed64.Zero);

        // Group four remains geometrically present, but all of its previous features disappear.
        manifold.AddContact(new ContactGroupKey(4, 0), Sample(99, Fixed64.One));
        foreach (int group in new[] { 7, 3, 0 })
            foreach (ulong id in new[] { 4UL, 2UL })
                manifold.AddContact(new ContactGroupKey(group, 0), Sample(id, Fixed64.One));
        cache.Retain(manifold);

        cache.GroupCount.Should().Be(3);
        cache.Count.Should().Be(6);
        foreach (int group in new[] { 0, 3, 7 })
        {
            ContactGroupKey key = new(group, 0);
            foreach (ulong id in new[] { 2UL, 4UL })
            {
                cache.TryGet(key, id, out ContactWarmStartImpulse impulse).Should().BeTrue();
                impulse.NormalImpulse.Should().Be((Fixed64)(group * 10 + (int)id));
            }
            cache.TryGet(key, 1, out _).Should().BeFalse();
            cache.TryGet(key, 3, out _).Should().BeFalse();
        }
        cache.TryGet(new ContactGroupKey(4, 0), 1, out _).Should().BeFalse();
        manifold.Reset();
        cache.Retain(manifold);
        cache.Count.Should().Be(0);
        cache.GroupCount.Should().Be(0);
    }

    [Theory]
    [InlineData(0)]
    [InlineData(1)]
    [InlineData(2)]
    public void PairLifecycle_ShouldInvalidateEveryGroupedWarmStart(int lifecycle)
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        var first = scenario.CreateCuboid(Vector3d.Zero);
        var second = scenario.CreateCuboid(Vector3d.Right);
        CollisionPair pair = scenario.CreatePair(first.Collider, second.Collider);
        for (int group = 0; group < 9; group++)
        {
            ContactGroupKey key = new(group, 0);
            pair.Manifold.AddContact(key, Sample(42, Fixed64.One));
            pair.StoreWarmStartImpulse(key, 42, Vector3d.Up, Fixed64.One, Fixed64.Half, Fixed64.Quarter);
        }

        if (lifecycle == 0) pair.Reset();
        else if (lifecycle == 1) pair.Deactivate();
        else pair.ClearWarmStart();

        for (int group = 0; group < 9; group++)
            pair.TryGetWarmStartImpulse(new ContactGroupKey(group, 0), 42, out _).Should().BeFalse();
        if (lifecycle != 2)
        {
            pair.Manifold.Count.Should().Be(0);
            pair.Manifold.GroupCount.Should().Be(0);
        }
    }

    [Fact]
    public void ReplayHash_ShouldBeIndependentOfGroupAndFeatureInsertionOrder()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        var first = scenario.CreateCuboid(Vector3d.Zero);
        var second = scenario.CreateCuboid(Vector3d.Right);
        CollisionPair pair = scenario.CreatePair(first.Collider, second.Collider);
        ContactGroupKey[] keys = CanonicalKeys();
        PopulateReplayPair(pair, keys, reverse: false);
        ChronicleHash baseline = HashPair(pair);
        pair.Manifold.Reset();
        pair.ClearWarmStart();
        PopulateReplayPair(pair, keys, reverse: true);

        HashPair(pair).Should().Be(baseline,
            "canonical replay state must not depend on narrow-phase discovery or cache insertion order");
        ContactGroupKey firstKey = keys[0];
        pair.StoreWarmStartImpulse(firstKey, 1, Vector3d.Up, Fixed64.Two, Fixed64.Half, Fixed64.Quarter);
        HashPair(pair).Should().NotBe(baseline, "every retained group impulse affects future response");
    }

    [Fact]
    public void WarmedManifoldAndCacheRegeneration_ShouldAllocateNothingAcrossNineGroups()
    {
        var manifold = new ContactManifold();
        var cache = new ContactWarmStartCache();
        ContactGroupKey[] keys = CanonicalKeys();
        ManifoldContact[] contacts = Enumerable.Range(1, 4).Select(id => Sample((ulong)id, Fixed64.One)).ToArray();
        manifold.ReserveGroups(9);
        manifold.ReserveGroups(1);
        Regenerate(ref cache, manifold, keys, contacts, 32);

        long before = GC.GetAllocatedBytesForCurrentThread();
        ulong checksum = Regenerate(ref cache, manifold, keys, contacts, 128);
        long allocated = GC.GetAllocatedBytesForCurrentThread() - before;

        checksum.Should().Be(128UL * 9 * (1 + 2 + 3 + 4));
        allocated.Should().Be(0);
        cache.Count.Should().Be(36);
        manifold.Count.Should().Be(36);
    }

    private static ContactGroupKey[] CanonicalKeys() => new[]
    {
        new ContactGroupKey(-1, 2, 3, 4, 5),
        new ContactGroupKey(1, 1, 3, 4, 5),
        new ContactGroupKey(1, 2, 2, 4, 5),
        new ContactGroupKey(1, 2, 3, 3, 5),
        new ContactGroupKey(1, 2, 3, 4, 4),
        new ContactGroupKey(1, 2, 3, 4, 5),
        new ContactGroupKey(1, 2, 3, 4, 6),
        new ContactGroupKey(1, 3, 3, 4, 5),
        new ContactGroupKey(2, 2, 3, 4, 5)
    };

    private static ManifoldContact Sample(ulong id, Fixed64 depth) =>
        new(id, new Vector3d((int)id, 0, 0), new Vector3d((int)id, 0, 1), depth, Vector3d.Up);

    private static void AddSamples(ContactManifold manifold, ContactGroupKey key, bool reverse)
    {
        for (int step = 0; step < 5; step++)
        {
            int id = reverse ? 5 - step : step + 1;
            manifold.AddContact(key, Sample((ulong)id, (Fixed64)id));
        }
    }

    private static void PopulateReplayPair(CollisionPair pair, ContactGroupKey[] keys, bool reverse)
    {
        pair.Manifold.BeginUpdate(7);
        for (int step = 0; step < keys.Length; step++)
        {
            ContactGroupKey key = keys[reverse ? keys.Length - 1 - step : step];
            for (int point = 0; point < 4; point++)
            {
                ulong id = (ulong)(reverse ? 4 - point : point + 1);
                pair.Manifold.AddContact(key, Sample(id, (Fixed64)(int)id));
                pair.StoreWarmStartImpulse(key, id, Vector3d.Up,
                    (Fixed64)(int)id, Fixed64.Half, Fixed64.Quarter);
            }
        }
    }

    private static ChronicleHash HashPair(CollisionPair pair)
    {
        var writer = new ChronicleHashWriter();
        pair.ContributeReplayHash(ref writer, GravitasReplayHashMode.Authoritative);
        return writer.ToHash();
    }

    private static ulong Regenerate(ref ContactWarmStartCache cache, ContactManifold manifold,
        ContactGroupKey[] keys, ManifoldContact[] contacts, int repetitions)
    {
        ulong checksum = 0;
        for (int repetition = 0; repetition < repetitions; repetition++)
        {
            manifold.BeginUpdate(repetition);
            cache.Clear();
            for (int group = keys.Length - 1; group >= 0; group--)
                foreach (ManifoldContact contact in contacts)
                {
                    manifold.AddContact(keys[group], contact);
                    cache.Set(keys[group], contact.ContactId, Vector3d.Up,
                        Fixed64.One, Fixed64.Zero, Fixed64.Zero);
                }
            cache.Retain(manifold);
            foreach (ManifoldContact contact in manifold)
                checksum += contact.ContactId;
        }
        return checksum;
    }
}
