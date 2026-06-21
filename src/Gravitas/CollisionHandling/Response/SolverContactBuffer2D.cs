//=======================================================================
// SolverContactBuffer2D.cs
//=======================================================================
// MIT License, Copyright (c) 2026–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp;
using System.Runtime.CompilerServices;

namespace Gravitas.CollisionHandling;

/// <summary>
/// Fixed-capacity pure 2D solver contact buffer.
/// </summary>
internal struct SolverContactBuffer2D
{
    private SolverContact2D _contact0;
    private SolverContact2D _contact1;
    private Fixed64 _normalImpulse0;
    private Fixed64 _normalImpulse1;
    private Fixed64 _tangentImpulse0;
    private Fixed64 _tangentImpulse1;
    private Fixed64 _normalVelocity0;
    private Fixed64 _normalVelocity1;

    public int Count { get; private set; }

    public void Add(SolverContact2D contact)
    {
        if (Count >= ContactManifold2D.MaxContactCount)
            return;

        if (Count == 0)
            _contact0 = contact;
        else
            _contact1 = contact;

        Count++;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public SolverContact2D GetContact(int index) => index == 0 ? _contact0 : _contact1;

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public Fixed64 GetNormalImpulse(int index) => index == 0 ? _normalImpulse0 : _normalImpulse1;

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public Fixed64 GetTangentImpulse(int index) => index == 0 ? _tangentImpulse0 : _tangentImpulse1;

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public Fixed64 GetNormalVelocity(int index) => index == 0 ? _normalVelocity0 : _normalVelocity1;

    public void SetNormalImpulse(int index, Fixed64 impulse, Fixed64 normalVelocity)
    {
        if (index == 0)
        {
            _normalImpulse0 = impulse;
            _normalVelocity0 = normalVelocity;
            return;
        }

        _normalImpulse1 = impulse;
        _normalVelocity1 = normalVelocity;
    }

    public void SetTangentImpulse(int index, Fixed64 impulse)
    {
        if (index == 0)
        {
            _tangentImpulse0 = impulse;
            return;
        }

        _tangentImpulse1 = impulse;
    }
}
