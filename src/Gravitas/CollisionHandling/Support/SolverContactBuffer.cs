using System.Runtime.CompilerServices;
using FixedMathSharp;

namespace Gravitas.CollisionHandling;

/// <summary>
/// A buffer for storing contact points and their associated impulses and velocities during collision resolution.
/// </summary>
internal struct SolverContactBuffer
{
    private SolverContact _contact0;
    private SolverContact _contact1;
    private SolverContact _contact2;
    private SolverContact _contact3;
    private Fixed64 _normalImpulse0;
    private Fixed64 _normalImpulse1;
    private Fixed64 _normalImpulse2;
    private Fixed64 _normalImpulse3;
    private Fixed64 _normalVelocity0;
    private Fixed64 _normalVelocity1;
    private Fixed64 _normalVelocity2;
    private Fixed64 _normalVelocity3;

    public int Count { get; private set; }

    public void Add(SolverContact contact)
    {
        if (Count >= ContactManifold.MaxContactCount)
            return;

        switch (Count)
        {
            case 0:
                _contact0 = contact;
                break;
            case 1:
                _contact1 = contact;
                break;
            case 2:
                _contact2 = contact;
                break;
            default:
                _contact3 = contact;
                break;
        }

        Count++;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public SolverContact GetContact(int index) =>
        index switch
        {
            0 => _contact0,
            1 => _contact1,
            2 => _contact2,
            _ => _contact3
        };

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public Fixed64 GetNormalImpulse(int index) =>
        index switch
        {
            0 => _normalImpulse0,
            1 => _normalImpulse1,
            2 => _normalImpulse2,
            _ => _normalImpulse3
        };

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public Fixed64 GetNormalVelocity(int index) =>
        index switch
        {
            0 => _normalVelocity0,
            1 => _normalVelocity1,
            2 => _normalVelocity2,
            _ => _normalVelocity3
        };

    public void SetNormalImpulse(int index, Fixed64 impulse, Fixed64 normalVelocity)
    {
        switch (index)
        {
            case 0:
                _normalImpulse0 = impulse;
                _normalVelocity0 = normalVelocity;
                break;
            case 1:
                _normalImpulse1 = impulse;
                _normalVelocity1 = normalVelocity;
                break;
            case 2:
                _normalImpulse2 = impulse;
                _normalVelocity2 = normalVelocity;
                break;
            default:
                _normalImpulse3 = impulse;
                _normalVelocity3 = normalVelocity;
                break;
        }
    }
}
