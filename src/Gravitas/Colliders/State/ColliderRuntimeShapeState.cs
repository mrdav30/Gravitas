//=======================================================================
// ColliderRuntimeShapeState.cs
//=======================================================================
// MIT License, Copyright (c) 2026–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using System;
using System.Runtime.CompilerServices;

namespace Gravitas.Colliders;

internal class ColliderRuntimeShapeState<TSnapshot>
    where TSnapshot : IEquatable<TSnapshot>
{
    private TSnapshot _snapshot = default!;
    private bool _hasSnapshot;
    private bool _dirty = true;

    public uint RuntimeVersion { get; private set; }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public void MarkDirty() => _dirty = true;

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public bool ShouldRebuild(in TSnapshot snapshot) =>
        _dirty || !_hasSnapshot || !_snapshot.Equals(snapshot);

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public void Commit(in TSnapshot snapshot)
    {
        _snapshot = snapshot;
        _hasSnapshot = true;
        _dirty = false;
        RuntimeVersion++;
    }
}
