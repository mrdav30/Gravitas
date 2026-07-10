using FluentAssertions;
using SwiftCollections;
using Xunit;

namespace Gravitas.Tests.Core;

public sealed class IslandGraphUtilityTests
{
    [Fact]
    public void IslandGraphUtility_ShouldDeduplicateUnionCompressResolveAndWakeDeterministically()
    {
        var nodes = new SwiftList<TestIslandNode>
        {
            new(4, new TestIslandBody(isAwake: false)),
            new(2, new TestIslandBody(isAwake: true)),
            new(4, new TestIslandBody(isAwake: true)),
            new(6, new TestIslandBody(isAwake: false))
        };
        var comparer = new IslandNodeKeyComparer<TestIslandNode>();

        IslandGraphUtility.SortAndDeduplicate(nodes, comparer);

        nodes.Count.Should().Be(3);
        nodes[0].BodyKey.Should().Be(2);
        nodes[0].ParentIndex.Should().Be(0);
        nodes[1].BodyKey.Should().Be(4);
        nodes[1].ParentIndex.Should().Be(1);
        nodes[2].BodyKey.Should().Be(6);
        nodes[2].ParentIndex.Should().Be(2);

        int node2 = IslandGraphUtility.Find(nodes, 2);
        int node4 = IslandGraphUtility.Find(nodes, 4);
        int node6 = IslandGraphUtility.Find(nodes, 6);
        IslandGraphUtility.Find(nodes, 8).Should().Be(-1);

        IslandGraphUtility.Union(nodes, node4, node6);
        IslandGraphUtility.Union(nodes, node2, node6);
        IslandGraphUtility.CompressRoots(nodes);

        nodes[0].RootKey.Should().Be(2);
        nodes[1].RootKey.Should().Be(2);
        nodes[2].RootKey.Should().Be(2);
        IslandGraphUtility.ResolveConstraintRootKey(nodes, node4, node6).Should().Be(2);
        IslandGraphUtility.ResolveConstraintRootKey(nodes, -1, node6).Should().Be(2);
        IslandGraphUtility.ResolveConstraintRootKey(nodes, -1, -1).Should().Be(-1);

        IslandGraphUtility.WakeBodies(nodes, rootKey: 99).Should().BeFalse();
        IslandGraphUtility.WakeBodies(nodes, rootKey: 2).Should().BeTrue();
        nodes[0].Body.WakeCount.Should().Be(1);
        nodes[1].Body.WakeCount.Should().Be(1);
        nodes[2].Body.WakeCount.Should().Be(1);
    }

    private sealed class TestIslandBody
    {
        public TestIslandBody(bool isAwake)
        {
            IsAwake = isAwake;
        }

        public bool IsAwake { get; private set; }

        public int WakeCount { get; private set; }

        public void Wake()
        {
            IsAwake = true;
            WakeCount++;
        }
    }

    private struct TestIslandNode : IIslandNodeState
    {
        public TestIslandNode(int bodyKey, TestIslandBody body)
        {
            BodyKey = bodyKey;
            Body = body;
            ParentIndex = -1;
            RootKey = bodyKey;
        }

        public int BodyKey { get; }

        public TestIslandBody Body { get; }

        public int ParentIndex { get; set; }

        public int RootKey { get; set; }

        public bool IsAwakeForCollision => Body.IsAwake;

        public void WakeFromCollision() => Body.Wake();
    }
}
