//=======================================================================
// MixedPairLifetimeToken.cs
//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using Gravitas.Colliders;
using Gravitas.CollisionHandling;

namespace Gravitas;

internal readonly struct MixedPairLifetimeToken
{
    internal MixedPairLifetimeToken(CollisionPairMixed pair)
    {
        Pair = pair;
        LifetimeVersion = pair.LifetimeVersion;
        Collider3D = new ColliderLifetimeToken(pair.Collider3D);
        Collider2D = new ColliderLifetimeToken2D(pair.Collider2D);
    }

    internal CollisionPairMixed Pair { get; }

    internal long LifetimeVersion { get; }

    private ColliderLifetimeToken Collider3D { get; }

    private ColliderLifetimeToken2D Collider2D { get; }

    internal bool IsCurrentLifetime => Pair.LifetimeVersion == LifetimeVersion
        && Collider3D.IsCurrentLifetime
        && Collider2D.IsCurrentLifetime;
}
