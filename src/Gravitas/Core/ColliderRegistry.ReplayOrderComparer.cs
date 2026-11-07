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

/// <content>Defines the nested ReplayOrderComparer type.</content>
internal sealed partial class ColliderRegistry<TCollider>
    where TCollider : class, IPhysicsColliderRegistryItem
{
    internal readonly struct ReplayOrderComparer : IComparer<TCollider>
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public int Compare(TCollider? x, TCollider? y) =>
            x!.ReplayOrder.CompareTo(y!.ReplayOrder);
    }
}
