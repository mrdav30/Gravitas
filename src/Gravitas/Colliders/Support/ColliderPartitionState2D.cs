using FixedMathSharp;
using GridForge.Spatial;
using SwiftCollections;
using System.Runtime.CompilerServices;

namespace Gravitas.Colliders;

internal struct ColliderPartitionState2D
{
    public bool IsPartitioned { get; private set; }

    public uint BroadPhaseVersion { get; private set; }

    public Vector2d LastGridBoundsMin { get; private set; }

    public Vector2d LastGridBoundsMax { get; private set; }

    public int LastPartitionKind { get; private set; }

    public SwiftList<WorldVoxelIndex>? Coordinates;

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public void MarkBroadPhaseChanged() => BroadPhaseVersion++;

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public void MarkPartitioned()
    {
        IsPartitioned = true;
        MarkBroadPhaseChanged();
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public void MarkUnpartitioned() => IsPartitioned = false;

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public bool MatchesGridBounds(Vector2d min, Vector2d max, int partitionKind) =>
        IsPartitioned
        && LastGridBoundsMin == min
        && LastGridBoundsMax == max
        && LastPartitionKind == partitionKind;

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public void SetPreviousGridBounds(Vector2d min, Vector2d max, int partitionKind = 0)
    {
        LastGridBoundsMin = min;
        LastGridBoundsMax = max;
        LastPartitionKind = partitionKind;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public void ClearCoordinates() => Coordinates?.Clear();
}
