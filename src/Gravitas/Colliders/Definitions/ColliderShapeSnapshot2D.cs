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
        Vector2d localScale,
        Vector2d localOffset,
        uint shapeVersion,
        Fixed64 mixedSlabCenterY,
        Fixed64 mixedHalfThickness)
    {
        Center = center;
        Rotation = rotation;
        LocalScale = localScale;
        LocalOffset = localOffset;
        ShapeVersion = shapeVersion;
        MixedSlabCenterY = mixedSlabCenterY;
        MixedHalfThickness = mixedHalfThickness;
    }

    public Vector2d Center { get; }

    public Fixed64 Rotation { get; }

    public Vector2d LocalScale { get; }

    public Vector2d LocalOffset { get; }

    public uint ShapeVersion { get; }

    public Fixed64 MixedSlabCenterY { get; }

    public Fixed64 MixedHalfThickness { get; }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public bool Equals(ColliderShapeSnapshot2D other) =>
        Center == other.Center
        && Rotation == other.Rotation
        && LocalScale == other.LocalScale
        && LocalOffset == other.LocalOffset
        && ShapeVersion == other.ShapeVersion
        && MixedSlabCenterY == other.MixedSlabCenterY
        && MixedHalfThickness == other.MixedHalfThickness;

    public override bool Equals(object? obj) =>
        obj is ColliderShapeSnapshot2D other && Equals(other);

    public override int GetHashCode() =>
        HashCode.Combine(Center, Rotation, LocalScale, LocalOffset, ShapeVersion, MixedSlabCenterY, MixedHalfThickness);
}
