//=======================================================================
// CollisionPairLifetimeToken.cs
//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using Gravitas.Colliders;
using Gravitas.CollisionHandling;

namespace Gravitas;

/// <summary>
/// Identifies a spatial collision pair lifetime and checks that both collider lifetimes still match.
/// </summary>
internal readonly struct CollisionPairLifetimeToken
{
    internal CollisionPairLifetimeToken(CollisionPair pair)
    {
        Pair = pair;
        LifetimeVersion = pair.LifetimeVersion;
        ColliderA = new ColliderLifetimeToken(pair.ColliderA);
        ColliderB = new ColliderLifetimeToken(pair.ColliderB);
    }

    internal CollisionPair Pair { get; }

    internal long LifetimeVersion { get; }

    private ColliderLifetimeToken ColliderA { get; }

    private ColliderLifetimeToken ColliderB { get; }

    internal bool IsCurrentLifetime => Pair.LifetimeVersion == LifetimeVersion
        && ColliderA.IsCurrentLifetime
        && ColliderB.IsCurrentLifetime;
}
