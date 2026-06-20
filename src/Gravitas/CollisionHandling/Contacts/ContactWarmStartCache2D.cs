using FixedMathSharp;
using System.Runtime.CompilerServices;

namespace Gravitas.CollisionHandling;

/// <summary>
/// Fixed-size 2D warm-start cache keyed by stable manifold contact identity.
/// </summary>
internal struct ContactWarmStartCache2D
{
    private ulong _contactId0;
    private ulong _contactId1;
    private ContactWarmStartImpulse _impulse0;
    private ContactWarmStartImpulse _impulse1;

    public int Count { get; private set; }

    public void Clear()
    {
        Count = 0;
        _contactId0 = 0UL;
        _contactId1 = 0UL;
        _impulse0 = default;
        _impulse1 = default;
    }

    public void Set(ulong contactId, Fixed64 normalImpulse, Fixed64 tangentImpulse)
    {
        ContactWarmStartImpulse impulse = new(normalImpulse, tangentImpulse);
        for (int i = 0; i < Count; i++)
        {
            if (GetContactId(i) != contactId)
                continue;

            SetImpulseUnchecked(i, impulse);
            return;
        }

        if (Count < ContactManifold2D.MaxContactCount)
        {
            SetContactIdUnchecked(Count, contactId);
            SetImpulseUnchecked(Count, impulse);
            Count++;
            return;
        }

        SetContactIdUnchecked(ContactManifold2D.MaxContactCount - 1, contactId);
        SetImpulseUnchecked(ContactManifold2D.MaxContactCount - 1, impulse);
    }

    public bool TryGet(ulong contactId, out ContactWarmStartImpulse impulse)
    {
        for (int i = 0; i < Count; i++)
        {
            if (GetContactId(i) != contactId)
                continue;

            impulse = GetImpulseUnchecked(i);
            return true;
        }

        impulse = default;
        return false;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private ulong GetContactId(int index) => index == 0 ? _contactId0 : _contactId1;

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private ContactWarmStartImpulse GetImpulseUnchecked(int index) => index == 0 ? _impulse0 : _impulse1;

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private void SetContactIdUnchecked(int index, ulong contactId)
    {
        if (index == 0)
        {
            _contactId0 = contactId;
            return;
        }

        _contactId1 = contactId;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private void SetImpulseUnchecked(int index, ContactWarmStartImpulse impulse)
    {
        if (index == 0)
        {
            _impulse0 = impulse;
            return;
        }

        _impulse1 = impulse;
    }
}
