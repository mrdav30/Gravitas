//=======================================================================
// ColliderLifetimeToken.cs
//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

namespace Gravitas.Colliders;

/// <summary>
/// Captures a 3D collider registration lifetime so retained work can reject stale bindings.
/// </summary>
internal readonly struct ColliderLifetimeToken
{
    internal ColliderLifetimeToken(LSCollider collider)
    {
        Collider = collider;
        LifetimeVersion = collider.LifetimeVersion;
    }

    internal LSCollider Collider { get; }

    internal long LifetimeVersion { get; }

    internal bool IsActive => Collider.IsActive
        && !Collider.IsDeactivationInProgress
        && IsCurrentLifetime;

    internal bool IsCurrentLifetime => Collider.LifetimeVersion == LifetimeVersion;
}
