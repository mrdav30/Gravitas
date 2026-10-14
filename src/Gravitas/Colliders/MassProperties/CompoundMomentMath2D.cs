//=======================================================================
// CompoundMomentMath2D.cs
//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp;

namespace Gravitas.Colliders;

/// <summary>
/// Retains area-weighted intrinsic moments in Q224 with the common factor sixty,
/// so compound mass is applied once after geometry and parallel-axis reduction.
/// </summary>
internal static class CompoundMomentMath2D
{
    internal static Signed576 GetWeightedMoment(LSCollider2D part, ExactMassWeight weight,
        Fixed64 rotation, Signed320 dx, Signed320 dy, out Signed576 firstMomentContribution)
    {
        firstMomentContribution = default;
        if (part is LSCircleCollider2D circle)
        {
            Fixed64 radius = circle.ScaledRadius;
            return WideArithmetic.MultiplySigned576(
                WideArithmetic.MultiplySigned320(weight.Numerator,
                    ExactMassProperties.InertiaGeometryTerm(Fixed64.Half, radius, radius)), 60);
        }
        if (part is LSAABBoxCollider2D box)
        {
            Vector2d half = box.ScaledHalfExtents;
            Signed192 squared = WideArithmetic.AddSigned192(
                Fixed64.GetExactRawProduct(half.X, half.X),
                Fixed64.GetExactRawProduct(half.Y, half.Y));
            return WideArithmetic.MultiplySigned576(
                WideArithmetic.MultiplySigned320(weight.Numerator, squared), Signed192.One, Signed192.Signed(20));
        }
        if (part is LSPolygonCollider2D polygon)
        {
            PolygonMassProperties2D properties = polygon.GetMassProperties();
            firstMomentContribution = properties.CalculateWeightedFirstMomentContribution(rotation, dx, dy);
            return properties.CalculateWeightedMoment();
        }

        // Authored compound definitions admit only circle, box, polygon and capsule parts.
        var capsule = (LSCapsuleCollider2D)part;
        capsule.GetMassPropertyGeometry(out Fixed64 radiusValue, out Fixed64 length);
        if (length <= Fixed64.Epsilon)
        {
            return WideArithmetic.MultiplySigned576(
                WideArithmetic.MultiplySigned320(weight.Numerator,
                    ExactMassProperties.InertiaGeometryTerm(Fixed64.Half, radiusValue, radiusValue)), 60);
        }
        if (radiusValue == Fixed64.Zero)
        {
            // A collapsed detached stadium has zero geometric area. The compound
            // supplies nominal weight, while its intrinsic moment retains the rod limit.
            return WideArithmetic.MultiplySigned576(
                WideArithmetic.MultiplySigned320(weight.Numerator,
                    ExactMassProperties.InertiaGeometryTerm(Fixed64.One, length, length)), 5);
        }

        ExactMassWeight rectangleWeight = ExactMassWeight.FromProduct(Fixed64.Two, radiusValue, length);
        ExactMassWeight capWeight = ExactMassWeight.FromProduct(Fixed64.Pi, radiusValue, radiusValue);
        LSCapsuleCollider2D.GetCenteredMomentGeometry(radiusValue, length,
            out Signed192 rectangleGeometry, out Signed192 capGeometry);
        return WideArithmetic.MultiplySigned576(WideArithmetic.AddSigned576(
            WideArithmetic.MultiplySigned320(rectangleWeight.Numerator, rectangleGeometry),
            WideArithmetic.MultiplySigned320(capWeight.Numerator, capGeometry)), 5);
    }
}
