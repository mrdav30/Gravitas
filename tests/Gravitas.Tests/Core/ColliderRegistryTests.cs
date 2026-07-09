using FluentAssertions;
using Gravitas.Colliders;
using Xunit;

namespace Gravitas.Tests.Core;

public sealed class ColliderRegistryTests
{
    [Fact]
    public void Remove_ShouldKeepLiveServiceIndicesDense()
    {
        var registry = new ColliderRegistry<TestRegistryCollider>(capacity: 4);
        var first = new TestRegistryCollider();
        var second = new TestRegistryCollider();
        var third = new TestRegistryCollider();
        registry.Register(first);
        registry.Register(second);
        registry.Register(third);

        registry.Remove(second).Should().BeTrue();

        registry.Count.Should().Be(2);
        third.ServiceIndex.Should().Be(1);
        registry.TryGetByServiceIndex(0, out TestRegistryCollider? resolvedFirst).Should().BeTrue();
        registry.TryGetByServiceIndex(1, out TestRegistryCollider? resolvedThird).Should().BeTrue();
        resolvedFirst.Should().BeSameAs(first);
        resolvedThird.Should().BeSameAs(third);
        second.Id.Should().Be(-1);
        second.ServiceIndex.Should().Be(-1);
        second.ReplayOrder.Should().Be(-1);
    }

    [Fact]
    public void Remove_ShouldRejectStaleOrMismatchedColliderIdentity()
    {
        var registry = new ColliderRegistry<TestRegistryCollider>(capacity: 4);
        var registered = new TestRegistryCollider();
        var other = new TestRegistryCollider();
        int id = registry.Register(registered);
        other.SetRegistryState(id, serviceIndex: 0, replayOrder: 0);

        registry.Remove(other).Should().BeFalse();
        registry.Remove(new TestRegistryCollider()).Should().BeFalse();

        registry.Count.Should().Be(1);
        registry.TryGetById(id, out TestRegistryCollider? resolved).Should().BeTrue();
        resolved.Should().BeSameAs(registered);
        other.Id.Should().Be(id);
    }

    [Fact]
    public void TryGetByServiceIndex_ShouldRejectOutOfRangeIndices()
    {
        var registry = new ColliderRegistry<TestRegistryCollider>(capacity: 4);
        var collider = new TestRegistryCollider();
        registry.Register(collider);

        registry.TryGetByServiceIndex(-1, out TestRegistryCollider? negative).Should().BeFalse();
        registry.TryGetByServiceIndex(registry.Count, out TestRegistryCollider? end).Should().BeFalse();
        registry.TryGetByServiceIndex(0, out TestRegistryCollider? resolved).Should().BeTrue();

        negative.Should().BeNull();
        end.Should().BeNull();
        resolved.Should().BeSameAs(collider);
    }

    [Fact]
    public void PrepareReplayColliders_ShouldAssignStableReplayOrdinalsAfterLiveReorder()
    {
        var registry = new ColliderRegistry<TestRegistryCollider>(capacity: 4);
        var first = new TestRegistryCollider();
        var second = new TestRegistryCollider();
        var third = new TestRegistryCollider();
        registry.Register(first);
        registry.Register(second);
        registry.Register(third);
        registry.Remove(second);

        var replayColliders = registry.PrepareReplayColliders();

        replayColliders.Count.Should().Be(2);
        replayColliders[0].Should().BeSameAs(first);
        replayColliders[1].Should().BeSameAs(third);
        first.ReplayOrdinal.Should().Be(0);
        third.ReplayOrdinal.Should().Be(1);
    }

    private sealed class TestRegistryCollider : IPhysicsColliderRegistryItem
    {
        public int Id { get; private set; } = -1;

        public int ServiceIndex { get; private set; } = -1;

        public int ReplayOrder { get; private set; } = -1;

        public int ReplayOrdinal { get; private set; } = -1;

        public void SetRegistryState(int id, int serviceIndex, int replayOrder)
        {
            Id = id;
            ServiceIndex = serviceIndex;
            ReplayOrder = replayOrder;
            ReplayOrdinal = -1;
        }

        public void SetRegistryServiceIndex(int serviceIndex) => ServiceIndex = serviceIndex;

        public void SetRegistryReplayOrdinal(int replayOrdinal) => ReplayOrdinal = replayOrdinal;

        public void ClearRegistryState()
        {
            Id = -1;
            ServiceIndex = -1;
            ReplayOrder = -1;
            ReplayOrdinal = -1;
        }
    }
}
