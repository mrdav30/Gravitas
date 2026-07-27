//=======================================================================
// ColliderShapeSnapshot2D.cs
//=======================================================================
// MIT License, Copyright (c) 2026–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp;
using System;
using System.Runtime.CompilerServices;

namespace Gravitas.Colliders;

internal readonly struct ColliderShapeSnapshot2D : IEquatable<ColliderShapeSnapshot2D>
{
    public ColliderShapeSnapshot2D(
        Vector2d center,
        Fixed64 rotation,
        Vector2d ownerScale,
        Vector2d partScale,
        Vector2d localOffset,
        uint shapeVersion,
        Fixed64 mixedSlabCenterY,
        Fixed64 mixedHalfThickness)
    {
        Center = center;
        Rotation = rotation;
        OwnerScale = ownerScale;
        PartScale = partScale;
        LocalOffset = localOffset;
        ShapeVersion = shapeVersion;
        MixedSlabCenterY = mixedSlabCenterY;
        MixedHalfThickness = mixedHalfThickness;
    }

    public Vector2d Center { get; }

    public Fixed64 Rotation { get; }

    public Vector2d OwnerScale { get; }

    public Vector2d PartScale { get; }

    public Vector2d LocalOffset { get; }

    public uint ShapeVersion { get; }

    public Fixed64 MixedSlabCenterY { get; }

    public Fixed64 MixedHalfThickness { get; }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public bool Equals(ColliderShapeSnapshot2D other) =>
        Center == other.Center
        && Rotation == other.Rotation
        && OwnerScale == other.OwnerScale
        && PartScale == other.PartScale
        && LocalOffset == other.LocalOffset
        && ShapeVersion == other.ShapeVersion
        && MixedSlabCenterY == other.MixedSlabCenterY
        && MixedHalfThickness == other.MixedHalfThickness;

    public override bool Equals(object? obj) =>
        obj is ColliderShapeSnapshot2D other && Equals(other);

    public override int GetHashCode()
    {
        unchecked
        {
            int hash = 17;
            hash = hash * 31 + Center.X.GetHashCode();
            hash = hash * 31 + Center.Y.GetHashCode();
            hash = hash * 31 + Rotation.GetHashCode();
            hash = hash * 31 + OwnerScale.X.GetHashCode();
            hash = hash * 31 + OwnerScale.Y.GetHashCode();
            hash = hash * 31 + PartScale.X.GetHashCode();
            hash = hash * 31 + PartScale.Y.GetHashCode();
            hash = hash * 31 + LocalOffset.X.GetHashCode();
            hash = hash * 31 + LocalOffset.Y.GetHashCode();
            hash = hash * 31 + ShapeVersion.GetHashCode();
            hash = hash * 31 + MixedSlabCenterY.GetHashCode();
            hash = hash * 31 + MixedHalfThickness.GetHashCode();
            return hash;
        }
    }
}
