//=======================================================================
// ColliderTriggerEventPolicy.cs
//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using System.Runtime.CompilerServices;

namespace Gravitas.Colliders;

internal static class ColliderTriggerEventPolicy
{
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal static bool ShouldRaise(LSCollider self, LSCollider other) =>
        self.IsTrigger != other.IsTrigger
        && (self.IsTrigger ? other.Body != null : self.Body != null);

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal static bool ShouldRaise(LSCollider2D self, LSCollider2D other) =>
        self.IsTrigger != other.IsTrigger
        && (self.IsTrigger ? other.Body != null : self.Body != null);

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal static bool ShouldRaise(LSCollider collider3D, LSCollider2D collider2D) =>
        collider3D.IsTrigger != collider2D.IsTrigger
        && (collider3D.IsTrigger ? collider2D.Body != null : collider3D.Body != null);
}
