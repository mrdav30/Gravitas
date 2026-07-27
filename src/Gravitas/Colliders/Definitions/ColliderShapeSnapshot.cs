//=======================================================================
// ColliderShapeSnapshot.cs
//=======================================================================
// MIT License, Copyright (c) 2026–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp;
using System;
using System.Runtime.CompilerServices;

namespace Gravitas.Colliders;

internal readonly struct ColliderShapeSnapshot : IEquatable<ColliderShapeSnapshot>
{
    public ColliderShapeSnapshot(
        Vector3d center,
        FixedQuaternion rotation,
        Vector3d ownerScale,
        Vector3d partScale,
        Vector3d localOffset,
        Vector3d size,
        Fixed64 radius)
    {
        Center = center;
        Rotation = rotation;
        OwnerScale = ownerScale;
        PartScale = partScale;
        LocalOffset = localOffset;
        Size = size;
        Radius = radius;
    }

    public Vector3d Center { get; }

    public FixedQuaternion Rotation { get; }

    public Vector3d OwnerScale { get; }

    public Vector3d PartScale { get; }

    public Vector3d LocalOffset { get; }

    public Vector3d Size { get; }

    public Fixed64 Radius { get; }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public bool Equals(ColliderShapeSnapshot other) =>
        Center == other.Center
        && Rotation == other.Rotation
        && OwnerScale == other.OwnerScale
        && PartScale == other.PartScale
        && LocalOffset == other.LocalOffset
        && Size == other.Size
        && Radius == other.Radius;

    public override bool Equals(object? obj) =>
        obj is ColliderShapeSnapshot other && Equals(other);

    public override int GetHashCode()
    {
        unchecked
        {
            int hash = 17;
            hash = Mix(hash, Center);
            hash = (hash * 31) + Rotation.X.GetHashCode();
            hash = (hash * 31) + Rotation.Y.GetHashCode();
            hash = (hash * 31) + Rotation.Z.GetHashCode();
            hash = (hash * 31) + Rotation.W.GetHashCode();
            hash = Mix(hash, OwnerScale);
            hash = Mix(hash, PartScale);
            hash = Mix(hash, LocalOffset);
            hash = Mix(hash, Size);
            hash = (hash * 31) + Radius.GetHashCode();
            return hash;
        }
    }

    private static int Mix(int hash, Vector3d value)
    {
        hash = (hash * 31) + value.X.GetHashCode();
        hash = (hash * 31) + value.Y.GetHashCode();
        return (hash * 31) + value.Z.GetHashCode();
    }

    public static bool operator ==(ColliderShapeSnapshot left, ColliderShapeSnapshot right) => left.Equals(right);

    public static bool operator !=(ColliderShapeSnapshot left, ColliderShapeSnapshot right) => !left.Equals(right);
}
