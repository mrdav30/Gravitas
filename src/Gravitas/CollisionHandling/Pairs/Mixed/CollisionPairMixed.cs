//=======================================================================
// CollisionPairMixed.cs
//=======================================================================
// MIT License, Copyright (c) 2026–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using Gravitas.Colliders;
using System.Runtime.CompilerServices;

namespace Gravitas.CollisionHandling;

/// <summary>
/// Owns one stable mixed 3D/2D collision pair identity and contact lifecycle.
/// </summary>
internal sealed class CollisionPairMixed
{
    private bool _isColliding;
    private bool _isTriggerPair;

    public CollisionPairMixed(LSCollider collider3D, LSCollider2D collider2D)
    {
        Collider3D = collider3D;
        Collider2D = collider2D;
        Initialize(collider3D, collider2D);
    }

    public LSCollider Collider3D { get; private set; }

    public LSCollider2D Collider2D { get; private set; }

    public int Collider3DId { get; private set; }

    public int Collider2DId { get; private set; }

    public ulong Key { get; private set; }

    public GravitasWorldContext Context => Collider3D.Context;

    public int LastFrame { get; private set; } = -1;

    public bool IsColliding => _isColliding;

    public bool IsTriggerPair => _isTriggerPair;

    public MixedContact Contact { get; private set; }

    public void Initialize(LSCollider collider3D, LSCollider2D collider2D)
    {
        SwiftThrowHelper.ThrowIfNull(collider3D, nameof(collider3D));
        SwiftThrowHelper.ThrowIfNull(collider2D, nameof(collider2D));

        Collider3D = collider3D;
        Collider2D = collider2D;
        Collider3DId = collider3D.Id;
        Collider2DId = collider2D.Id;
        Key = MixedColliderKey.CreateKey(Collider3DId, Collider2DId);
        LastFrame = -1;
        _isColliding = false;
        _isTriggerPair = collider3D.IsTrigger || collider2D.IsTrigger;
        Contact = default;
    }

    public void MarkColliding(int frame, MixedContact contact)
    {
        bool changed = !_isColliding;
        _isColliding = true;
        _isTriggerPair = Collider3D.IsTrigger || Collider2D.IsTrigger;
        Contact = contact;
        LastFrame = frame;

        Context.Diagnostics.EmitMixedContact(this, contact, true);

        Collider3D.NotifyMixedContact(Collider2D, true, changed, _isTriggerPair);
        Collider2D.NotifyMixedContact(Collider3D, true, changed, _isTriggerPair);
    }

    public void MarkResting(int frame)
    {
        LastFrame = frame;
    }

    public void MarkResting(int frame, MixedContact contact)
    {
        Contact = contact;
        LastFrame = frame;
    }

    public void MarkSeparated()
    {
        if (!_isColliding)
            return;

        _isColliding = false;
        Contact = default;
        Collider3D.NotifyMixedContact(Collider2D, false, true, _isTriggerPair);
        Collider2D.NotifyMixedContact(Collider3D, false, true, _isTriggerPair);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal void WakeSleepingBodiesForCollision()
    {
        SolidBody? body3D = Collider3D.Body;
        SolidBody2D? body2D = Collider2D.Body;
        if (body3D == null || body2D == null)
            return;

        bool body3DAwake = body3D.IsAwakeForCollision;
        bool body2DAwake = body2D.IsAwakeForCollision;
        if (body3D.IsSleeping && body2DAwake)
            body3D.Wake();
        if (body2D.IsSleeping && body3DAwake)
            body2D.Wake();
    }
}
