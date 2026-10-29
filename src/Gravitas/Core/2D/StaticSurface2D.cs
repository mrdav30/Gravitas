//=======================================================================
// StaticSurface2D.cs
//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp;
using FixedMathSharp.Geometry;
using Gravitas.Colliders;
using SwiftCollections;
using System;

namespace Gravitas;

/// <summary>Declares continuity between registered static AABB or convex polygon colliders in one context.</summary>
/// <remarks>
/// Exact shared boundary spans participate in one filled planar surface while members
/// retain their collider identities, materials, queries and overlap events.
/// Mixed contacts also preserve each member's finite embedded height and thickness.
/// Create declarations through <see cref="GravitasPhysics2DService.CreateStaticSurface"/>.
/// Context reset or disposal ends the declaration's lifetime.
/// </remarks>
/// <content>Owns typed member lifetime, transactional replacement and contact invalidation.</content>
public sealed partial class StaticSurface2D : IDisposable, Chronicler.IRecordable
{
    private readonly GravitasWorldContext _context;
    private LSCollider2D[] _members = Array.Empty<LSCollider2D>();
    private FixedBoundBox[] _memberBounds = Array.Empty<FixedBoundBox>();
    private readonly SwiftHashSet<int> _changedSuppliers = new();
    private bool _disposed;
    internal int ReplayMembership => _members.Length == 0 ? 0 : _members[0].ReplayOrdinal + 1;

    internal StaticSurface2D(GravitasWorldContext context, ReadOnlySpan<LSCollider2D> members)
    {
        _context = context;
        _mixedAdmit = AdmitMixedAxis;
        _mixedSphereAdmit = AdmitMixedSpherePoint;
        _mixedCapsuleAdmit = AdmitMixedPrismSupportPoint;
        _mixedSphereBoundaryAdmit = AdmitMixedSphereBoundaryPoint;
        _mixedAdmitRigid = AdmitMixedRigidAxis;
        ReplaceMembers(members);
    }

    /// <summary>Atomically replaces the registered members of this declaration.</summary>
    /// <param name="members">Distinct active AABB or convex polygon colliders with bodyless or explicit-static ownership in this context. May be empty.</param>
    /// <remarks>
    /// Call between complete fixed steps. Compound roots and leaves are ineligible.
    /// Rejection preserves the previous membership; disconnected pieces retain their boundaries.
    /// Deactivated members must be explicitly added again after reinitialization.
    /// </remarks>
    /// <exception cref="ArgumentException">A member is ineligible or belongs to another declaration.</exception>
    /// <exception cref="ObjectDisposedException">This declaration or its context has ended.</exception>
    /// <exception cref="InvalidOperationException">A fixed step is in progress.</exception>
    public void ReplaceMembers(ReadOnlySpan<LSCollider2D> members)
    {
        _context.ThrowIfDisposed();
        _context.ThrowIfFixedStepMutationNotAllowed();
        if (_disposed) throw new ObjectDisposedException(nameof(StaticSurface2D));
        LSCollider2D[] next = members.ToArray();
        var unique = new SwiftHashSet<LSCollider2D>();
        foreach (LSCollider2D member in next)
        {
            // Registry identity excludes stale lifetimes even when an ID has been reused.
            if (member is not (LSAABBoxCollider2D or LSPolygonCollider2D)
                || !unique.Add(member) || member.CompoundOwner != null
                || !_context.Physics2D.TryGetColliderById(member.Id, out var registered) || registered != member
                || !member.IsStatic || !member.IsActive || member.IsDeactivationInProgress
                || member.StaticSurface != null && member.StaticSurface != this)
                throw new ArgumentException("Members must be distinct registered static AABB or convex polygon colliders in this context.", nameof(members));
        }
        bool unchanged = next.Length == _members.Length;
        foreach (LSCollider2D member in _members) unchanged &= unique.Contains(member);
        if (unchanged) return;
        Array.Sort(next, static (first, second) => ((IPhysicsColliderRegistryItem)first).ReplayOrder
            .CompareTo(((IPhysicsColliderRegistryItem)second).ReplayOrder));
        // Publish associations only after every candidate has passed admission.
        var bounds = new FixedBoundBox[next.Length];
        for (int index = 0; index < next.Length; index++) bounds[index] = next[index].MixedBounds3D;
        LSCollider2D[] previousMembers = _members;
        foreach (LSCollider2D previous in previousMembers)
            if (!unique.Contains(previous)) Invalidate(previous);
        foreach (LSCollider2D previous in previousMembers) previous.StaticSurface = null;
        _members = next; _memberBounds = bounds;
        foreach (LSCollider2D member in _members) member.StaticSurface = this;
        foreach (LSCollider2D member in _members)
            if (Array.IndexOf(previousMembers, member) < 0) Invalidate(member);
    }

    internal void Invalidate(LSCollider2D? changed = null, bool includeNeighbors = true)
    {
        InvalidateGeometry();
        _changedSuppliers.Clear();
        FixedBoundBox before = default, after = default;
        if (changed != null)
        {
            int index = Array.IndexOf(_members, changed);
            before = _memberBounds[index]; after = changed.MixedBounds3D;
            _memberBounds[index] = after;
        }
        for (int index = 0; index < _members.Length; index++)
            if (changed == null || _members[index] == changed || includeNeighbors
                && (Touches(_memberBounds[index], before) || Touches(_memberBounds[index], after)))
            {
                _changedSuppliers.Add(_members[index].Id + 1);
                _members[index].MarkContactDependencyChanged(notifySurface: false);
            }
        foreach (LSCollider2D member in _members)
            _context.Physics2D.ClearWarmStartCachesForCollider(member, _changedSuppliers);
        _context.MixedCollisions.InvalidateSurfaceContacts(null, this, _changedSuppliers);
        _changedSuppliers.Clear();
    }

    private bool Touches(in FixedBoundBox first, in FixedBoundBox second) =>
        _context.Settings.RuntimeMode == PhysicsRuntimeMode.Mixed ? first.Intersects(second)
        : FixedBoundArea.FromMinMax(new(first.Min.X, first.Min.Z), new(first.Max.X, first.Max.Z)).Intersects(
            FixedBoundArea.FromMinMax(new(second.Min.X, second.Min.Z), new(second.Max.X, second.Max.Z)));

    partial void InvalidateGeometry();

    internal void Remove(LSCollider2D member)
    {
        int index = Array.IndexOf(_members, member);
        if (index < 0) return;
        Invalidate(member);
        member.StaticSurface = null;
        Array.Copy(_members, index + 1, _members, index, _members.Length - index - 1);
        Array.Copy(_memberBounds, index + 1, _memberBounds, index, _memberBounds.Length - index - 1);
        Array.Resize(ref _members, _members.Length - 1);
        Array.Resize(ref _memberBounds, _members.Length);
    }

    /// <summary>Ends this declaration and restores independent member response.</summary>
    /// <remarks>Call between complete fixed steps. Members remain registered. Repeated disposal is harmless.</remarks>
    /// <exception cref="InvalidOperationException">A fixed step is in progress.</exception>
    public void Dispose()
    {
        if (_disposed) return;
        _context.ThrowIfFixedStepMutationNotAllowed();
        Release();
        _context.Physics2D.RemoveStaticSurface(this);
    }

    internal void Release()
    {
        Invalidate();
        foreach (LSCollider2D member in _members) member.StaticSurface = null;
        _members = Array.Empty<LSCollider2D>();
        _memberBounds = Array.Empty<FixedBoundBox>();
        _disposed = true;
    }
}
