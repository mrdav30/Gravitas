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
    public bool MatchesGridBounds(Vector2d min, Vector2d max) =>
        IsPartitioned && LastGridBoundsMin == min && LastGridBoundsMax == max;

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public void SetPreviousGridBounds(Vector2d min, Vector2d max)
    {
        LastGridBoundsMin = min;
        LastGridBoundsMax = max;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public void ClearCoordinates() => Coordinates?.Clear();
}
