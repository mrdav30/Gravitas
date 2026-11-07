//=======================================================================
// ContinuousCollisionMath.IntervalSearchStatus.cs
//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

namespace Gravitas.CollisionHandling;

/// <content>Defines the nested IntervalSearchStatus type.</content>
internal static partial class ContinuousCollisionMath
{
    public enum IntervalSearchStatus : byte
    {
        CertifiedNoHit,
        ExactHit,
        Unresolved,
    }
}
