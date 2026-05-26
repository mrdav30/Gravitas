using FixedMathSharp;
using GridForge.Spatial;
using SwiftCollections;
using System.Runtime.CompilerServices;

namespace Gravitas.Colliders;

internal struct ColliderPartitionState
{
    public bool IsPartitioned { get; private set; }

    public bool PartitionChanged { get; set; }

    public uint BroadPhaseVersion { get; private set; }

    public Vector3d LastGridBoundsMin { get; private set; }

    public Vector3d LastGridBoundsMax { get; private set; }

    public SwiftList<WorldVoxelIndex>? Coordinates;

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public void MarkBroadPhaseChanged() => BroadPhaseVersion++;

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public void MarkPartitioned()
    {
        IsPartitioned = true;
        PartitionChanged = true;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public void MarkUnpartitioned() => IsPartitioned = false;

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public void SetPreviousGridBounds(Vector3d min, Vector3d max)
    {
        LastGridBoundsMin = min;
        LastGridBoundsMax = max;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public void ClearCoordinates() => Coordinates?.Clear();
}
