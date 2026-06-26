//=======================================================================
// MixedColliderKey.cs
//=======================================================================
// MIT License, Copyright (c) 2026–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using System.Runtime.CompilerServices;

namespace Gravitas;

/// <summary>
/// Stable mixed-dimension broad-phase identity for one 3D collider and one 2D collider.
/// </summary>
internal readonly struct MixedColliderKey
{
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public MixedColliderKey(int collider3DId, int collider2DId)
    {
        SwiftThrowHelper.ThrowIfNegative(collider3DId, nameof(collider3DId));
        SwiftThrowHelper.ThrowIfNegative(collider2DId, nameof(collider2DId));
        Collider3DId = collider3DId;
        Collider2DId = collider2DId;
        Key = CreateKey(collider3DId, collider2DId);
    }

    public int Collider3DId { get; }

    public int Collider2DId { get; }

    public ulong Key { get; }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static ulong CreateKey(int collider3DId, int collider2DId) =>
        ((ulong)(uint)collider3DId << 32) | (uint)collider2DId;
}
