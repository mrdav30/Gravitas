//=======================================================================
// IColliderHierarchyNode.cs
//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

namespace Gravitas.Colliders;

/// <summary>
/// Provides the shared operations for linking and resolving colliders within a context hierarchy.
/// </summary>
internal interface IColliderHierarchyNode
{
    ColliderHierarchyKey HierarchyKey { get; }

    GravitasWorldContext Context { get; }

    IColliderHierarchyNode? HierarchyParent { get; }

    void AddChild(ColliderHierarchyKey key);

    void RemoveChild(ColliderHierarchyKey key);

    void ClearParentReference();

    bool TryGetHierarchyColliderByKey(ColliderHierarchyKey key, out IColliderHierarchyNode? collider);
}
