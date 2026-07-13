//=======================================================================
// ColliderHierarchyState.cs
//=======================================================================
// MIT License, Copyright (c) 2026–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using SwiftCollections;
using System.Runtime.CompilerServices;

namespace Gravitas.Colliders;

internal struct ColliderHierarchyState
{
    private bool _configuredAsParent;
    private SwiftHashSet<ulong>? _children;

    public bool IsChild { get; private set; }

    public bool IsParent { get; private set; }

    public ColliderHierarchyKey ParentKey { get; private set; }

    public ColliderHierarchyKey TopParentKey => TopParent?.HierarchyKey ?? ColliderHierarchyKey.None;

    public IColliderHierarchyNode? Parent { get; private set; }

    public IColliderHierarchyNode? TopParent { get; private set; }

    public int ChildCount => _children?.Count ?? 0;

    public SwiftHashSet<ulong>? Children => _children;

    public void Initialize(bool isParent)
    {
        _configuredAsParent = isParent;
        IsParent = isParent;
        IsChild = !isParent;
        ParentKey = ColliderHierarchyKey.None;
        Parent = null;
        TopParent = null;
        _children?.Clear();
    }

    public void SetParent(IColliderHierarchyNode owner, IColliderHierarchyNode parent)
    {
        SwiftThrowHelper.ThrowIfNull(parent, nameof(parent));
        SwiftThrowHelper.ThrowIfArgument(
            ReferenceEquals(owner, parent),
            nameof(parent),
            "Collider cannot be parented to itself.");
        SwiftThrowHelper.ThrowIfArgument(
            !owner.HierarchyKey.IsValid,
            nameof(owner),
            "Owner collider must have a valid hierarchy key before setting a parent.");
        SwiftThrowHelper.ThrowIfArgument(
            !parent.HierarchyKey.IsValid,
            nameof(parent),
            "Parent collider must have a valid hierarchy key before being assigned.");

        GravitasWorldContext ownerContext = owner.Context;
        SwiftThrowHelper.ThrowIfArgument(
            !ReferenceEquals(ownerContext, parent.Context),
            nameof(parent),
            "Parent collider must belong to the same GravitasWorldContext.");

        IColliderHierarchyNode topParent = FindTopParent(owner, parent);
        ColliderHierarchyKey topParentKey = topParent.HierarchyKey;
        bool topParentChanged = ParentKey != topParentKey;
        if (ParentKey.IsValid && topParentChanged)
            TopParent!.RemoveChild(owner.HierarchyKey);

        Parent = parent;
        TopParent = topParent;
        ParentKey = topParentKey;
        IsChild = true;
        if (topParentChanged)
            topParent.AddChild(owner.HierarchyKey);
    }

    public void ClearParent(IColliderHierarchyNode owner)
    {
        if (ParentKey.IsValid)
            TopParent!.RemoveChild(owner.HierarchyKey);

        Parent = null;
        TopParent = null;
        ParentKey = ColliderHierarchyKey.None;
        IsChild = !_configuredAsParent;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public void ClearParentReference()
    {
        Parent = null;
        TopParent = null;
        ParentKey = ColliderHierarchyKey.None;
        IsChild = !_configuredAsParent;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public void ClearChildren()
    {
        _children?.Clear();
        IsParent = _configuredAsParent;
    }

    public bool AddChild(ColliderHierarchyKey key)
    {
        SwiftThrowHelper.ThrowIfArgument(!key.IsValid, nameof(key), "Child collider key must be valid.");
        _children ??= new();
        if (!_children.Add(key.Packed))
            return false;

        IsParent = true;
        return true;
    }

    public bool RemoveChild(ColliderHierarchyKey key)
    {
        if (!key.IsValid || _children?.Remove(key.Packed) != true)
            return false;

        if (_children.Count == 0)
            IsParent = _configuredAsParent;

        return true;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public bool ExcludesCollisionWith(in ColliderHierarchyState other, ColliderHierarchyKey ownerKey, ColliderHierarchyKey otherKey)
    {
        if (!ownerKey.IsValid || !otherKey.IsValid)
            return false;

        if (ownerKey == otherKey)
            return true;

        if (ParentKey == otherKey || other.ParentKey == ownerKey)
            return true;

        return ParentKey.IsValid && ParentKey == other.ParentKey;
    }

    private static IColliderHierarchyNode FindTopParent(IColliderHierarchyNode owner, IColliderHierarchyNode parent)
    {
        IColliderHierarchyNode current = parent;
        while (current.HierarchyParent != null)
        {
            current = current.HierarchyParent;
            SwiftThrowHelper.ThrowIfArgument(
                ReferenceEquals(owner, current),
                nameof(parent),
                "Collider hierarchy cannot contain cycles.");
        }

        return current;
    }
}
