using System.Runtime.CompilerServices;

namespace Gravitas.Colliders;

internal sealed class ColliderRuntimeShapeState
{
    private ColliderShapeSnapshot _snapshot;
    private bool _hasSnapshot;
    private bool _dirty = true;

    public uint RuntimeVersion { get; private set; }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public void MarkDirty() => _dirty = true;

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public bool ShouldRebuild(in ColliderShapeSnapshot snapshot) =>
        _dirty || !_hasSnapshot || _snapshot != snapshot;

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public void Commit(in ColliderShapeSnapshot snapshot)
    {
        _snapshot = snapshot;
        _hasSnapshot = true;
        _dirty = false;
        RuntimeVersion++;
    }
}
