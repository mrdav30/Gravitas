using SwiftCollections;
using System.Runtime.CompilerServices;

namespace Gravitas.Colliders;

internal struct ColliderHierarchyState
{
    private bool _configuredAsParent;
    private SwiftHashSet<int>? _children;

    public bool IsChild { get; private set; }

    public bool IsParent { get; private set; }

    public int ParentId { get; private set; }

    public LSCollider? Parent { get; private set; }

    public int ChildCount => _children?.Count ?? 0;

    public SwiftHashSet<int>? Children => _children;

    public void Initialize(bool isParent)
    {
        _configuredAsParent = isParent;
        IsParent = isParent;
        IsChild = !isParent;
        ParentId = -1;
        Parent = null;
        _children?.Clear();
    }

    public void SetParent(LSCollider owner, LSCollider parent)
    {
        SwiftThrowHelper.ThrowIfNull(parent, nameof(parent));
        SwiftThrowHelper.ThrowIfArgument(
            ReferenceEquals(owner, parent),
            nameof(parent),
            "Collider cannot be parented to itself.");

        GravitasWorldContext ownerContext = owner.Context;
        SwiftThrowHelper.ThrowIfArgument(
            !parent.TryGetBoundContext(out GravitasWorldContext? parentContext)
                || !ReferenceEquals(ownerContext, parentContext),
            nameof(parent),
            "Parent collider must belong to the same GravitasWorldContext.");

        LSCollider topParent = FindTopParent(owner, parent);
        bool topParentChanged = ParentId != topParent.Id;
        if (ParentId != -1
            && topParentChanged
            && ownerContext.Physics.TryGetColliderById(ParentId, out LSCollider? previousParent))
        {
            previousParent!.RemoveChild(owner.Id);
        }

        Parent = parent;
        ParentId = topParent.Id;
        IsChild = true;
        if (topParentChanged)
            topParent.AddChild(owner.Id);
    }

    public void ClearParent(LSCollider owner)
    {
        if (ParentId != -1
            && owner.TryGetBoundContext(out GravitasWorldContext? context)
            && context!.Physics.TryGetColliderById(ParentId, out LSCollider? previousParent))
        {
            previousParent!.RemoveChild(owner.Id);
        }

        Parent = null;
        ParentId = -1;
        IsChild = !_configuredAsParent;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public void ClearParentReference()
    {
        Parent = null;
        ParentId = -1;
        IsChild = !_configuredAsParent;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public void ClearChildren()
    {
        _children?.Clear();
        IsParent = _configuredAsParent;
    }

    public bool AddChild(int id)
    {
        _children ??= new();
        if (!_children.Add(id))
            return false;

        IsParent = true;
        return true;
    }

    public bool RemoveChild(int id)
    {
        if (_children?.Remove(id) != true)
            return false;

        if (_children.Count == 0)
            IsParent = _configuredAsParent;

        return true;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public bool ExcludesCollisionWith(in ColliderHierarchyState other, int ownerId, int otherId)
    {
        if (ownerId == otherId)
            return true;

        if (ParentId == otherId || other.ParentId == ownerId)
            return true;

        return ParentId != -1 && ParentId == other.ParentId;
    }

    private static LSCollider FindTopParent(LSCollider owner, LSCollider parent)
    {
        LSCollider current = parent;
        while (current.Parent != null)
        {
            current = current.Parent;
            SwiftThrowHelper.ThrowIfArgument(
                ReferenceEquals(owner, current),
                nameof(parent),
                "Collider hierarchy cannot contain cycles.");
        }

        return current;
    }
}
