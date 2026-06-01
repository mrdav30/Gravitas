namespace Gravitas.Colliders;

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
