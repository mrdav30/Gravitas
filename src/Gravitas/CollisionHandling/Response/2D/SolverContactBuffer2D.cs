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
    private ContactNormalImpulseResult2D _normalResult0;
    private ContactNormalImpulseResult2D _normalResult1;
    private Fixed64 _tangentImpulse0;
    private Fixed64 _tangentImpulse1;

    public int Count { get; private set; }

    public void Add(SolverContact2D contact)
    {
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
    public ContactNormalImpulseResult2D GetNormalResult(int index) =>
        index == 0 ? _normalResult0 : _normalResult1;

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public Fixed64 GetTangentImpulse(int index) => index == 0 ? _tangentImpulse0 : _tangentImpulse1;

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public void SetNormalImpulse(
        int index,
        Fixed64 impulse,
        ContactNormalImpulseResult2D normalResult)
    {
        if (index == 0)
        {
            _normalImpulse0 = impulse;
            _normalResult0 = normalResult;
            return;
        }

        _normalImpulse1 = impulse;
        _normalResult1 = normalResult;
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
