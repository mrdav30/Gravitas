//=======================================================================
// ContactLever2D.cs
//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp;

namespace Gravitas.CollisionHandling;

internal readonly struct ContactLever2D
{
    private ContactLever2D(Vector2d vector, bool isExact)
    {
        Vector = vector;
        IsExact = isExact;
    }

    internal bool IsExact { get; }

    internal Vector2d Vector { get; }

    internal static ContactLever2D Zero =>
        new(Vector2d.Zero, isExact: false);

    internal static ContactLever2D Create(
        in ContactAnchor2D point,
        in ContactAnchor2D origin)
    {
        if (point.TryGetOffsetFrom(origin, out Vector2d vector))
            return new ContactLever2D(vector, isExact: false);

        return new ContactLever2D(default, isExact: true);
    }
}
