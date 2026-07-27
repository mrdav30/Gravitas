//=======================================================================
// LSCollider2D.Hierarchy.cs
//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using SwiftCollections;

namespace Gravitas.Colliders;

public abstract partial class LSCollider2D
{
    private void ClearChildParentReferences()
    {
        SwiftHashSet<ulong>? children = _hierarchyState.Children;
        if (children == null || _context == null)
            return;

        foreach (ulong childPackedKey in children)
        {
            ColliderHierarchyKey childKey = ColliderHierarchyKey.FromPacked(childPackedKey);
            if (((IColliderHierarchyNode)this).TryGetHierarchyColliderByKey(
                    childKey,
                    out IColliderHierarchyNode? child))
            {
                child!.ClearParentReference();
            }
        }

        _hierarchyState.ClearChildren();
    }

    void IColliderHierarchyNode.AddChild(ColliderHierarchyKey key)
    {
        ThrowIfCompoundPartLifecycle(nameof(IColliderHierarchyNode.AddChild));
        _hierarchyState.AddChild(key);
    }

    void IColliderHierarchyNode.RemoveChild(ColliderHierarchyKey key)
    {
        ThrowIfCompoundPartLifecycle(nameof(IColliderHierarchyNode.RemoveChild));
        _hierarchyState.RemoveChild(key);
    }

    void IColliderHierarchyNode.ClearParentReference() =>
        _hierarchyState.ClearParentReference();

    bool IColliderHierarchyNode.TryGetHierarchyColliderByKey(
        ColliderHierarchyKey key,
        out IColliderHierarchyNode? collider)
    {
        collider = null;
        if (!key.IsValid || _context == null)
            return false;

        if (key.Is2D
            && _context.Physics2D.TryGetColliderById(
                key.Id,
                out LSCollider2D? collider2D))
        {
            collider = collider2D;
            return true;
        }

        if (key.Is3D
            && _context.Physics.TryGetColliderById(
                key.Id,
                out LSCollider? collider3D))
        {
            collider = collider3D;
            return true;
        }

        return false;
    }
}
