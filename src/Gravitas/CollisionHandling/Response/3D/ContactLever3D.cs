//=======================================================================
// ContactLever3D.cs
//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp;

namespace Gravitas.CollisionHandling;

/// <summary>
/// Carries a 3D contact lever or signals that the solver must use exact anchor arithmetic.
/// </summary>
internal readonly struct ContactLever3D
{
    private ContactLever3D(Vector3d vector, bool isExact)
    {
        Vector = vector;
        IsExact = isExact;
    }

    internal bool IsExact { get; }

    internal Vector3d Vector { get; }

    internal static ContactLever3D Create(
        in ContactAnchor point,
        in ContactAnchor origin)
    {
        if (point.TryGetOffsetFrom(origin, out Vector3d vector))
            return new ContactLever3D(vector, isExact: false);

        return new ContactLever3D(default, isExact: true);
    }
}
