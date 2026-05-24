using FixedMathSharp;
using System;
using System.Runtime.CompilerServices;

namespace Gravitas.Colliders;

internal readonly struct ColliderShapeSnapshot : IEquatable<ColliderShapeSnapshot>
{
    public ColliderShapeSnapshot(
        Vector3d center,
        FixedQuaternion rotation,
        Vector3d localScale,
        Vector3d localOffset,
        Vector3d size,
        Fixed64 radius)
    {
        Center = center;
        Rotation = rotation;
        LocalScale = localScale;
        LocalOffset = localOffset;
        Size = size;
        Radius = radius;
    }

    public Vector3d Center { get; }

    public FixedQuaternion Rotation { get; }

    public Vector3d LocalScale { get; }

    public Vector3d LocalOffset { get; }

    public Vector3d Size { get; }

    public Fixed64 Radius { get; }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public bool Equals(ColliderShapeSnapshot other) =>
        Center == other.Center
        && Rotation == other.Rotation
        && LocalScale == other.LocalScale
        && LocalOffset == other.LocalOffset
        && Size == other.Size
        && Radius == other.Radius;

    public override bool Equals(object? obj) =>
        obj is ColliderShapeSnapshot other && Equals(other);

    public override int GetHashCode() =>
        HashCode.Combine(Center, Rotation, LocalScale, LocalOffset, Size, Radius);

    public static bool operator ==(ColliderShapeSnapshot left, ColliderShapeSnapshot right) => left.Equals(right);

    public static bool operator !=(ColliderShapeSnapshot left, ColliderShapeSnapshot right) => !left.Equals(right);
}
