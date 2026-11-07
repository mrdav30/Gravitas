//=======================================================================
// ColliderLifetimeToken2D.cs
//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

namespace Gravitas.Colliders;

internal readonly struct ColliderLifetimeToken2D
{
    internal ColliderLifetimeToken2D(LSCollider2D collider)
    {
        Collider = collider;
        LifetimeVersion = collider.LifetimeVersion;
    }

    internal LSCollider2D Collider { get; }

    internal long LifetimeVersion { get; }

    internal bool IsActive => Collider.IsActive && IsCurrentLifetime;

    internal bool IsCurrentLifetime => Collider.LifetimeVersion == LifetimeVersion;
}
