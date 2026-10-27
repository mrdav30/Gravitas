//=======================================================================
// StaticSurface2D.cs
//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using Gravitas.Colliders;
using SwiftCollections;
using System;

namespace Gravitas;

/// <summary>Internal filled-planar continuity declaration pending query, CCD and restore parity.</summary>
/// <content>Owns typed member lifetime, transactional replacement and contact invalidation.</content>
internal sealed partial class StaticSurface2D : IDisposable
{
    private readonly GravitasWorldContext _context;
    private LSCollider2D[] _members = Array.Empty<LSCollider2D>();
    private bool _disposed;

    internal StaticSurface2D(GravitasWorldContext context, ReadOnlySpan<LSCollider2D> members)
    {
        _context = context;
        _mixedAdmit = AdmitMixedAxis;
        _mixedSphereAdmit = AdmitMixedSpherePoint;
        _mixedSphereBoundaryAdmit = AdmitMixedSphereBoundaryPoint;
        _mixedAdmitRigid = AdmitMixedRigidAxis;
        ReplaceMembers(members);
    }

    internal void ReplaceMembers(ReadOnlySpan<LSCollider2D> members)
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
        // Publish associations only after every candidate has passed admission.
        Invalidate();
        foreach (LSCollider2D previous in _members) previous.StaticSurface = null;
        _members = next;
        foreach (LSCollider2D member in _members) member.StaticSurface = this;
        Invalidate();
    }

    internal void Invalidate()
    {
        InvalidateGeometry();
        foreach (LSCollider2D member in _members)
        {
            member.MarkContactDependencyChanged(notifySurface: false);
            // A genuine overlap carrier may retain a different member's support row.
            _context.Physics2D.ClearWarmStartCachesForCollider(member);
        }
    }

    partial void InvalidateGeometry();

    internal void Remove(LSCollider2D member)
    {
        int index = Array.IndexOf(_members, member);
        if (index < 0) return;
        Invalidate();
        member.StaticSurface = null;
        Array.Copy(_members, index + 1, _members, index, _members.Length - index - 1);
        Array.Resize(ref _members, _members.Length - 1);
    }

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
        _disposed = true;
    }
}
