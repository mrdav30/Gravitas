//=======================================================================
// LSCollider.RuntimeLifecycle.cs
//=======================================================================
// MIT License, Copyright (c) 2026–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp;
using Gravitas.CollisionHandling;
using SwiftCollections;
using System;
using System.Runtime.CompilerServices;

namespace Gravitas.Colliders;

public abstract partial class LSCollider
{
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public void SetPreviousGridBounds() =>
        _partitionState.SetPreviousGridBounds(BoundsMin, BoundsMax, Context.Collisions.ResolvePartitionKind(this));

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal bool TryGetCollisionPair(int otherId, out CollisionPair? collisionPair) =>
        _pairState.TryGetCollisionPair(otherId, out collisionPair);

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal bool TryAddCollisionPair(int otherId, CollisionPair collisionPair) =>
        _pairState.TryAddCollisionPair(otherId, collisionPair);

    internal bool TryRemoveCollisionPair(int otherId) =>
        _pairState.TryRemoveCollisionPair(otherId, out _);

    internal bool TryAddCollisionPairHolder(int otherId) => _pairState.TryAddCollisionPairHolder(otherId);

    internal bool TryRemoveCollisionPairHolder(int otherId) => _pairState.TryRemoveCollisionPairHolder(otherId);

    internal void ClearCollisionPairState()
    {
        _pairState.ClearCollisionPairs();
        _pairState.ClearCollisionPairHolders();
    }

    internal void ClearRuntimeRelationships()
    {
        ClearChildParentReferences();
        ClearParent();
    }

    public void Deactivate()
    {
        ThrowIfCompoundPartLifecycle(nameof(Deactivate));
        if (_body != null)
        {
            _body.Deactivate();
            return;
        }

        DeactivateRuntimeRegistration();
    }

    internal void DeactivateRuntimeRegistration()
    {
        _deactivationInProgress = true;
        try
        {
            if (_id >= 0)
                Context.Physics.DessimilateCollider(this);

            _active = false;
            ClearBindingState();
        }
        finally
        {
            _deactivationInProgress = false;
        }
    }

    private void ClearBindingState()
    {
        _body = null;
        _agent = null;
        _context = null;
    }

    private void ClearChildParentReferences()
    {
        SwiftHashSet<ulong>? children = _hierarchyState.Children;
        if (children == null)
            return;

        foreach (ulong childPackedKey in children)
        {
            ColliderHierarchyKey childKey = ColliderHierarchyKey.FromPacked(childPackedKey);
            if (((IColliderHierarchyNode)this).TryGetHierarchyColliderByKey(childKey, out IColliderHierarchyNode? child))
                child!.ClearParentReference();
        }

        _hierarchyState.ClearChildren();
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal void BindContext(GravitasWorldContext context)
    {
        SwiftThrowHelper.ThrowIfNull(context, nameof(context));
        SwiftThrowHelper.ThrowIfArgument(
            _context != null && !ReferenceEquals(_context, context),
            nameof(context),
            "Collider is already bound to a different GravitasWorldContext.");
        _context = context;
    }

    internal void BindCompoundPart(
        LSCompoundCollider owner,
        FixedQuaternion localRotation,
        Vector3d localScale,
        GravitasWorldContext context)
    {
        SwiftThrowHelper.ThrowIfNull(owner, nameof(owner));
        SwiftThrowHelper.ThrowIfArgument(
            HasHostBinding,
            nameof(owner),
            "Compound collider parts cannot be initialized as standalone colliders.");
        _compoundOwner = owner;
        _compoundLocalRotation = localRotation;
        _compoundLocalScale = localScale;
        BindContext(context);
        RebuildRuntimeShapeState();
    }

    internal void ReserveCompoundPart(LSCompoundCollider owner)
    {
        SwiftThrowHelper.ThrowIfNull(owner, nameof(owner));
        SwiftThrowHelper.ThrowIfArgument(
            HasHostBinding,
            nameof(owner),
            "Compound collider parts cannot be initialized as standalone colliders.");
        SwiftThrowHelper.ThrowIfArgument(
            _compoundOwner != null && !ReferenceEquals(_compoundOwner, owner),
            nameof(owner),
            "Compound collider part is already owned by another compound collider.");

        _compoundOwner = owner;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private void ThrowIfCompoundPartLifecycle(string operation)
    {
        SwiftThrowHelper.ThrowIfTrue(
            _compoundOwner != null,
            operation,
            "Compound collider parts are geometry owned by LSCompoundCollider and cannot run standalone lifecycle operations.");
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal bool TryGetBoundContext(out GravitasWorldContext? context)
    {
        context = _context;
        return context != null;
    }
}
