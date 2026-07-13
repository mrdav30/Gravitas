using FluentAssertions;
using Gravitas.Colliders;
using Gravitas.Tests.Support;
using System;
using System.Collections.Generic;
using Xunit;

namespace Gravitas.Tests.Colliders;

public sealed class ColliderHierarchyStateTests
{
    [Fact]
    public void HierarchyKey_ShouldRoundTripPackedIdentityAndRejectInvalidIds()
    {
        ColliderHierarchyKey none = ColliderHierarchyKey.None;
        ColliderHierarchyKey twoD = ColliderHierarchyKey.Create2D(12);
        ColliderHierarchyKey threeD = ColliderHierarchyKey.Create3D(12);

        none.IsValid.Should().BeFalse();
        none.Packed.Should().Be(0UL);
        ColliderHierarchyKey.FromPacked(0UL).Should().Be(none);
        ColliderHierarchyKey.FromPacked(twoD.Packed).Should().Be(twoD);
        twoD.Is2D.Should().BeTrue();
        twoD.Is3D.Should().BeFalse();
        threeD.Is3D.Should().BeTrue();
        threeD.Is2D.Should().BeFalse();
        twoD.Equals((object?)null).Should().BeFalse();
        twoD.Equals("2d").Should().BeFalse();
        (twoD == ColliderHierarchyKey.Create2D(12)).Should().BeTrue();
        (twoD != threeD).Should().BeTrue();

        Action createNegative2D = () => ColliderHierarchyKey.Create2D(-1);
        Action createNegative3D = () => ColliderHierarchyKey.Create3D(-1);
        createNegative2D.Should().Throw<ArgumentOutOfRangeException>().WithParameterName("id");
        createNegative3D.Should().Throw<ArgumentOutOfRangeException>().WithParameterName("id");
    }

    [Fact]
    public void Initialize_ShouldResetChildrenAndConfiguredParentState()
    {
        var state = new ColliderHierarchyState();
        ColliderHierarchyKey child = ColliderHierarchyKey.Create3D(2);
        state.Initialize(isParent: true);
        state.AddChild(child).Should().BeTrue();

        state.Initialize(isParent: false);

        state.IsParent.Should().BeFalse();
        state.IsChild.Should().BeTrue();
        state.ChildCount.Should().Be(0);
        state.ParentKey.Should().Be(ColliderHierarchyKey.None);
        state.Parent.Should().BeNull();
        state.TopParent.Should().BeNull();

        state.Initialize(isParent: true);

        state.IsParent.Should().BeTrue();
        state.IsChild.Should().BeFalse();
    }

    [Fact]
    public void EmptyChildCleanup_ShouldRemainIdempotent()
    {
        var state = new ColliderHierarchyState();
        ColliderHierarchyKey child = ColliderHierarchyKey.Create3D(2);

        state.ClearChildren();

        state.RemoveChild(child).Should().BeFalse();
        state.IsParent.Should().BeFalse();
        state.ChildCount.Should().Be(0);
    }

    [Fact]
    public void AddAndRemoveChild_ShouldRejectInvalidDuplicatesAndMissingKeys()
    {
        var state = new ColliderHierarchyState();
        ColliderHierarchyKey first = ColliderHierarchyKey.Create3D(1);
        ColliderHierarchyKey second = ColliderHierarchyKey.Create2D(2);
        state.Initialize(isParent: false);

        Action addInvalid = () => state.AddChild(ColliderHierarchyKey.None);

        addInvalid.Should().Throw<ArgumentException>().WithParameterName("key");
        state.AddChild(first).Should().BeTrue();
        state.AddChild(first).Should().BeFalse();
        state.AddChild(second).Should().BeTrue();
        state.RemoveChild(ColliderHierarchyKey.None).Should().BeFalse();
        state.RemoveChild(ColliderHierarchyKey.Create3D(99)).Should().BeFalse();
        state.IsParent.Should().BeTrue();

        state.RemoveChild(first).Should().BeTrue();

        state.IsParent.Should().BeTrue();

        state.RemoveChild(second).Should().BeTrue();

        state.IsParent.Should().BeFalse();
        state.ChildCount.Should().Be(0);
    }

    [Fact]
    public void SetParent_ShouldRejectInvalidKeysContextsAndCycles()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        using PhysicsScenarioBuilder foreignScenario = PhysicsScenarioBuilder.Create();
        var owner = new TestNode(ColliderHierarchyKey.Create3D(1), scenario.Context);
        var invalidOwner = new TestNode(ColliderHierarchyKey.None, scenario.Context);
        var parent = new TestNode(ColliderHierarchyKey.Create3D(2), scenario.Context);
        var invalidParent = new TestNode(ColliderHierarchyKey.None, scenario.Context);
        var foreignParent = new TestNode(ColliderHierarchyKey.Create3D(3), foreignScenario.Context);
        owner.Register(owner, parent);
        parent.Register(owner, parent);

        Action nullParent = () => owner.State.SetParent(owner, null!);
        Action selfParent = () => owner.State.SetParent(owner, owner);
        Action invalidOwnerKey = () => invalidOwner.State.SetParent(invalidOwner, parent);
        Action invalidParentKey = () => owner.State.SetParent(owner, invalidParent);
        Action crossContext = () => owner.State.SetParent(owner, foreignParent);

        nullParent.Should().Throw<ArgumentNullException>().WithParameterName("parent");
        selfParent.Should().Throw<ArgumentException>().WithParameterName("parent");
        invalidOwnerKey.Should().Throw<ArgumentException>().WithParameterName("owner");
        invalidParentKey.Should().Throw<ArgumentException>().WithParameterName("parent");
        crossContext.Should().Throw<ArgumentException>().WithParameterName("parent");

        owner.State.SetParent(owner, parent);
        Action cycle = () => parent.State.SetParent(parent, owner);

        cycle.Should().Throw<ArgumentException>().WithParameterName("parent");
    }

    [Fact]
    public void SetAndClearParent_ShouldTrackTopParentAndMissingLookupCleanup()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        var top = new TestNode(ColliderHierarchyKey.Create3D(1), scenario.Context);
        var firstMiddle = new TestNode(ColliderHierarchyKey.Create3D(2), scenario.Context);
        var secondMiddle = new TestNode(ColliderHierarchyKey.Create3D(3), scenario.Context);
        var child = new TestNode(ColliderHierarchyKey.Create2D(4), scenario.Context);
        RegisterAll(top, firstMiddle, secondMiddle, child);
        firstMiddle.State.SetParent(firstMiddle, top);
        secondMiddle.State.SetParent(secondMiddle, top);
        child.State.SetParent(child, firstMiddle);

        child.State.SetParent(child, secondMiddle);

        child.State.Parent.Should().BeSameAs(secondMiddle);
        child.State.TopParent.Should().BeSameAs(top);
        top.State.ChildCount.Should().Be(3);

        child.RemoveFromLookup(top.HierarchyKey);
        child.State.ClearParent(child);

        child.State.ParentKey.Should().Be(ColliderHierarchyKey.None);
        child.State.Parent.Should().BeNull();
        child.State.TopParent.Should().BeNull();
        child.State.IsChild.Should().BeTrue();
        top.State.ChildCount.Should().Be(2);
    }

    [Fact]
    public void ReparentWithMissingPreviousTopParentLookup_ShouldAdoptNewTopParent()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        var previousTop = new TestNode(ColliderHierarchyKey.Create3D(1), scenario.Context);
        var newTop = new TestNode(ColliderHierarchyKey.Create2D(2), scenario.Context);
        var child = new TestNode(ColliderHierarchyKey.Create3D(3), scenario.Context);
        RegisterAll(previousTop, newTop, child);
        child.State.SetParent(child, previousTop);
        child.RemoveFromLookup(previousTop.HierarchyKey);

        child.State.SetParent(child, newTop);

        child.State.Parent.Should().BeSameAs(newTop);
        child.State.TopParent.Should().BeSameAs(newTop);
        child.State.ParentKey.Should().Be(newTop.HierarchyKey);
        previousTop.State.ChildCount.Should().Be(0);
        previousTop.State.IsParent.Should().BeFalse();
        newTop.State.ChildCount.Should().Be(1);
    }

    [Fact]
    public void ExcludesCollisionWith_ShouldCoverInvalidSelfParentChildAndSiblingCases()
    {
        using PhysicsScenarioBuilder scenario = PhysicsScenarioBuilder.Create();
        var parent = new TestNode(ColliderHierarchyKey.Create3D(1), scenario.Context);
        var child = new TestNode(ColliderHierarchyKey.Create3D(2), scenario.Context);
        var sibling = new TestNode(ColliderHierarchyKey.Create2D(3), scenario.Context);
        var unrelated = new TestNode(ColliderHierarchyKey.Create2D(4), scenario.Context);
        RegisterAll(parent, child, sibling, unrelated);
        child.State.SetParent(child, parent);
        sibling.State.SetParent(sibling, parent);

        child.State.ExcludesCollisionWith(
            sibling.State,
            ColliderHierarchyKey.None,
            sibling.HierarchyKey).Should().BeFalse();
        child.State.ExcludesCollisionWith(
            sibling.State,
            child.HierarchyKey,
            ColliderHierarchyKey.None).Should().BeFalse();
        child.State.ExcludesCollisionWith(
            child.State,
            child.HierarchyKey,
            child.HierarchyKey).Should().BeTrue();
        child.State.ExcludesCollisionWith(
            parent.State,
            child.HierarchyKey,
            parent.HierarchyKey).Should().BeTrue();
        parent.State.ExcludesCollisionWith(
            child.State,
            parent.HierarchyKey,
            child.HierarchyKey).Should().BeTrue();
        child.State.ExcludesCollisionWith(
            sibling.State,
            child.HierarchyKey,
            sibling.HierarchyKey).Should().BeTrue();
        child.State.ExcludesCollisionWith(
            unrelated.State,
            child.HierarchyKey,
            unrelated.HierarchyKey).Should().BeFalse();
    }

    private static void RegisterAll(params TestNode[] nodes)
    {
        foreach (TestNode node in nodes)
        {
            node.Register(nodes);
        }
    }

    private sealed class TestNode : IColliderHierarchyNode
    {
        private readonly Dictionary<ulong, IColliderHierarchyNode> _nodes = new();

        public TestNode(ColliderHierarchyKey key, GravitasWorldContext context)
        {
            HierarchyKey = key;
            Context = context;
            State.Initialize(isParent: false);
        }

        public ColliderHierarchyState State;

        public ColliderHierarchyKey HierarchyKey { get; }

        public GravitasWorldContext Context { get; }

        public IColliderHierarchyNode? HierarchyParent => State.Parent;

        public void AddChild(ColliderHierarchyKey key) => State.AddChild(key);

        public void RemoveChild(ColliderHierarchyKey key) => State.RemoveChild(key);

        public void ClearParentReference() => State.ClearParentReference();

        public bool TryGetHierarchyColliderByKey(ColliderHierarchyKey key, out IColliderHierarchyNode? collider) =>
            _nodes.TryGetValue(key.Packed, out collider);

        public void Register(params TestNode[] nodes)
        {
            foreach (TestNode node in nodes)
            {
                if (node.HierarchyKey.IsValid)
                    _nodes[node.HierarchyKey.Packed] = node;
            }
        }

        public void RemoveFromLookup(ColliderHierarchyKey key) => _nodes.Remove(key.Packed);
    }
}
