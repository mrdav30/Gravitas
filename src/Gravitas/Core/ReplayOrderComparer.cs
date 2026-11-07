//=======================================================================
// ColliderRegistry.ReplayOrderComparer.cs
//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using System.Collections.Generic;
using System.Runtime.CompilerServices;
using Gravitas.Colliders;

namespace Gravitas;

/// <summary>
/// Compares registered colliders by their recorded registration order.
/// </summary>
internal readonly struct ReplayOrderComparer<TCollider> : IComparer<TCollider>
    where TCollider : class, IPhysicsColliderRegistryItem
{
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public int Compare(TCollider? x, TCollider? y) =>
        x!.ReplayOrder.CompareTo(y!.ReplayOrder);
}
