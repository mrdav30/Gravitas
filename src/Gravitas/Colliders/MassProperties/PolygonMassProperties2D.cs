//=======================================================================
// PolygonMassProperties2D.cs
//=======================================================================
// MIT License, Copyright (c) 2024-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using System;

using FixedMathSharp;
using FixedMathSharp.Geometry;

namespace Gravitas.Colliders;

/// <summary>
/// Owns exact mass-property interpretation for convex polygon area.
/// </summary>
internal static class PolygonMassProperties2D
{
    internal static bool TryGetWeightAndCentroid(
        ReadOnlySpan<Vector2d> vertices,
        out ExactMassWeight weight,
        out Vector2d centroid)
    {
        bool result =
            WideConvex2dRelations.TryGetSignedDoubleAreaAndCentroid(
                vertices,
                out Signed320 signedDoubleArea,
                out centroid);
        weight = result
            ? ExactMassProperties.CreateAreaWeight(signedDoubleArea)
            : ExactMassWeight.Zero;
        return result;
    }
}
