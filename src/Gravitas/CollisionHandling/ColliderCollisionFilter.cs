//=======================================================================
// ColliderCollisionFilter.cs
//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using Gravitas.Colliders;
using System.Runtime.CompilerServices;

namespace Gravitas.CollisionHandling;

internal static class ColliderCollisionFilter
{
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal static bool AllowsPhysicalPair(LSCollider first, LSCollider second) =>
        !first.IgnoresCollisionLayer(second.Layer)
        && !second.IgnoresCollisionLayer(first.Layer);

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal static bool AllowsPhysicalPair(LSCollider2D first, LSCollider2D second) =>
        !first.IgnoresCollisionLayer(second.Layer)
        && !second.IgnoresCollisionLayer(first.Layer);

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal static bool AllowsPhysicalPair(LSCollider collider3D, LSCollider2D collider2D) =>
        !collider3D.IgnoresCollisionLayer(collider2D.Layer)
        && !collider2D.IgnoresCollisionLayer(collider3D.Layer);
}
