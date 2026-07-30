//=======================================================================
// ExactLever3D.Arithmetic.cs
//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp;
using FixedMathSharp.Geometry;

namespace Gravitas.CollisionHandling;

/// <content>
/// Contains exact point-velocity, quadratic-form, and transformed-cross
/// arithmetic for rigid-body contact levers.
/// </content>
internal readonly partial struct ExactLever3D
{
    internal static void GetRelativePointVelocityRatio(
        Vector3d firstLinearVelocity,
        Vector3d firstAngularVelocity,
        in ExactLever3D firstLever,
        Vector3d secondLinearVelocity,
        Vector3d secondAngularVelocity,
        in ExactLever3D secondLever,
        Vector3d projectionAxis,
        out Signed832 numerator,
        out Signed832 denominator)
    {
        // Normalized anchors bound each lever denominator below the Signed320 width.
        Signed320 firstDenominator =
            Signed320.NarrowValue(firstLever.Denominator);
        Signed320 secondDenominator =
            Signed320.NarrowValue(secondLever.Denominator);
        Signed704 firstAngular =
            GetCrossProductProjectionNumerator(
                firstLever,
                projectionAxis,
                firstAngularVelocity);
        Signed704 secondAngular =
            GetCrossProductProjectionNumerator(
                secondLever,
                projectionAxis,
                secondAngularVelocity);
        Signed192 linear = WideGeometry.GetDifferenceDotProduct3D(
            secondLinearVelocity.X,
            firstLinearVelocity.X,
            secondLinearVelocity.Y,
            firstLinearVelocity.Y,
            secondLinearVelocity.Z,
            firstLinearVelocity.Z,
            projectionAxis.X,
            Fixed64.Zero,
            projectionAxis.Y,
            Fixed64.Zero,
            projectionAxis.Z,
            Fixed64.Zero);
        Signed576 commonLeverDenominator =
            WideArithmetic.MultiplySigned320(
                firstDenominator,
                secondDenominator);
        Signed576 linearAtCommonDenominator =
            WideArithmetic.MultiplySigned576(
                commonLeverDenominator,
                linear,
                Signed192.One);
        numerator = WideArithmetic.AddSigned832(
            Signed832.ExtendValue(linearAtCommonDenominator),
            WideArithmetic.MultiplySigned704ToSigned832(
                secondAngular,
                firstDenominator));
        numerator = WideArithmetic.SubtractSigned832(
            numerator,
            WideArithmetic.MultiplySigned704ToSigned832(
                firstAngular,
                secondDenominator));
        Signed576 narrowDenominator = WideArithmetic.MultiplySigned576(
            commonLeverDenominator,
            Signed192.One,
            Signed192.One);
        denominator = Signed832.ExtendValue(narrowDenominator);
    }

    internal static void GetCrossProductQuadraticFormRatio(
        in ExactLever3D lever,
        Vector3d crossVector,
        Fixed3x3 transform,
        out Signed832 numerator,
        out Signed832 denominator)
    {
        GetCrossProductNumerators(
            lever.XNumerator,
            lever.YNumerator,
            lever.ZNumerator,
            crossVector,
            out Signed704 crossX,
            out Signed704 crossY,
            out Signed704 crossZ);

        numerator = GetQuadraticTerm(
            crossX, crossX, transform.M11);
        numerator = WideArithmetic.AddSigned832(
            numerator,
            GetQuadraticTerm(crossX, crossY, transform.M21));
        numerator = WideArithmetic.AddSigned832(
            numerator,
            GetQuadraticTerm(crossX, crossZ, transform.M31));
        numerator = WideArithmetic.AddSigned832(
            numerator,
            GetQuadraticTerm(crossY, crossX, transform.M12));
        numerator = WideArithmetic.AddSigned832(
            numerator,
            GetQuadraticTerm(crossY, crossY, transform.M22));
        numerator = WideArithmetic.AddSigned832(
            numerator,
            GetQuadraticTerm(crossY, crossZ, transform.M32));
        numerator = WideArithmetic.AddSigned832(
            numerator,
            GetQuadraticTerm(crossZ, crossX, transform.M13));
        numerator = WideArithmetic.AddSigned832(
            numerator,
            GetQuadraticTerm(crossZ, crossY, transform.M23));
        numerator = WideArithmetic.AddSigned832(
            numerator,
            GetQuadraticTerm(crossZ, crossZ, transform.M33));

        denominator = WideArithmetic.MultiplySigned576ToSigned832(
            lever.Denominator,
            lever.Denominator);
        // A raw quadratic result carries four Q32.32 denominator factors.
        denominator = WideArithmetic.MultiplySigned832(
            denominator,
            Signed192.One);
        denominator = WideArithmetic.MultiplySigned832(
            denominator,
            Signed192.One);
        denominator = WideArithmetic.MultiplySigned832(
            denominator,
            Signed192.One);
        denominator = WideArithmetic.MultiplySigned832(
            denominator,
            Signed192.One);
    }

    internal static bool TryGetTransformedScaledCrossProduct(
        in ExactLever3D lever,
        Vector3d crossVector,
        Fixed3x3 transform,
        Fixed64 firstMultiplier,
        Fixed64 secondMultiplier,
        Fixed64 divisor,
        out Vector3d result)
    {
        GetTransformedCrossProduct(
            lever,
            crossVector,
            transform,
            out Signed832 transformedX,
            out Signed832 transformedY,
            out Signed832 transformedZ);
        Signed832 denominator = GetTransformedCrossProductDenominator(
            lever,
            Signed192.Raw(divisor));
        return TryGetScaledVector(
            transformedX,
            transformedY,
            transformedZ,
            firstMultiplier,
            secondMultiplier,
            denominator,
            out result);
    }

    internal static bool TryGetTransformedWeightedCrossProduct(
        in ExactLever3D lever,
        Vector3d first,
        Fixed64 firstScale,
        Vector3d second,
        Fixed64 secondScale,
        Vector3d third,
        Fixed64 thirdScale,
        Fixed3x3 transform,
        out Vector3d result)
    {
        GetTransformedCrossProduct(
            lever,
            first,
            transform,
            out Signed832 firstX,
            out Signed832 firstY,
            out Signed832 firstZ);
        GetTransformedCrossProduct(
            lever,
            second,
            transform,
            out Signed832 secondX,
            out Signed832 secondY,
            out Signed832 secondZ);
        GetTransformedCrossProduct(
            lever,
            third,
            transform,
            out Signed832 thirdX,
            out Signed832 thirdY,
            out Signed832 thirdZ);
        Signed832 denominator = GetTransformedCrossProductDenominator(
            lever,
            Signed192.One);
        bool representable = TryGetWeightedComponent(
                firstX,
                firstScale,
                secondX,
                secondScale,
                thirdX,
                thirdScale,
                denominator,
                out Fixed64 resultX)
            & TryGetWeightedComponent(
                firstY,
                firstScale,
                secondY,
                secondScale,
                thirdY,
                thirdScale,
                denominator,
                out Fixed64 resultY)
            & TryGetWeightedComponent(
                firstZ,
                firstScale,
                secondZ,
                secondScale,
                thirdZ,
                thirdScale,
                denominator,
                out Fixed64 resultZ);
        result = representable
            ? new Vector3d(resultX, resultY, resultZ)
            : default;
        return representable;
    }

    internal static void GetTransformedCrossProduct(
        in ExactLever3D lever,
        Vector3d crossVector,
        Fixed3x3 transform,
        out Signed832 x,
        out Signed832 y,
        out Signed832 z)
    {
        GetCrossProductNumerators(
            lever.XNumerator,
            lever.YNumerator,
            lever.ZNumerator,
            crossVector,
            out Signed704 crossX,
            out Signed704 crossY,
            out Signed704 crossZ);
        x = TransformCrossProduct(
            crossX,
            crossY,
            crossZ,
            transform.M11,
            transform.M21,
            transform.M31);
        y = TransformCrossProduct(
            crossX,
            crossY,
            crossZ,
            transform.M12,
            transform.M22,
            transform.M32);
        z = TransformCrossProduct(
            crossX,
            crossY,
            crossZ,
            transform.M13,
            transform.M23,
            transform.M33);
    }

    private static bool TryGetScaledVector(
        Signed832 transformedX,
        Signed832 transformedY,
        Signed832 transformedZ,
        Fixed64 firstMultiplier,
        Fixed64 secondMultiplier,
        Signed832 denominator,
        out Vector3d result)
    {
        bool representable = TryGetScaledComponent(
                transformedX,
                firstMultiplier,
                secondMultiplier,
                denominator,
                out Fixed64 resultX)
            & TryGetScaledComponent(
                transformedY,
                firstMultiplier,
                secondMultiplier,
                denominator,
                out Fixed64 resultY)
            & TryGetScaledComponent(
                transformedZ,
                firstMultiplier,
                secondMultiplier,
                denominator,
                out Fixed64 resultZ);
        result = representable
            ? new Vector3d(resultX, resultY, resultZ)
            : default;
        return representable;
    }

    private static void GetRawCrossProduct(
        Vector3d first,
        Vector3d second,
        out Signed320 x,
        out Signed320 y,
        out Signed320 z)
    {
        x = WideArithmetic.SubtractSigned320(
            WideArithmetic.MultiplySigned192(
                Signed192.Raw(first.Y),
                Signed192.Raw(second.Z)),
            WideArithmetic.MultiplySigned192(
                Signed192.Raw(first.Z),
                Signed192.Raw(second.Y)));
        y = WideArithmetic.SubtractSigned320(
            WideArithmetic.MultiplySigned192(
                Signed192.Raw(first.Z),
                Signed192.Raw(second.X)),
            WideArithmetic.MultiplySigned192(
                Signed192.Raw(first.X),
                Signed192.Raw(second.Z)));
        z = WideArithmetic.SubtractSigned320(
            WideArithmetic.MultiplySigned192(
                Signed192.Raw(first.X),
                Signed192.Raw(second.Y)),
            WideArithmetic.MultiplySigned192(
                Signed192.Raw(first.Y),
                Signed192.Raw(second.X)));
    }

    private static Signed704 GetCrossProductProjectionNumerator(
        in ExactLever3D lever,
        Vector3d crossVector,
        Vector3d projectionVector)
    {
        GetRawCrossProduct(
            crossVector,
            projectionVector,
            out Signed320 coefficientX,
            out Signed320 coefficientY,
            out Signed320 coefficientZ);
        return WideArithmetic.AddSigned704(
            WideArithmetic.AddSigned704(
                WideArithmetic.MultiplySigned576ToSigned704(
                    lever.XNumerator,
                    coefficientX),
                WideArithmetic.MultiplySigned576ToSigned704(
                    lever.YNumerator,
                    coefficientY)),
            WideArithmetic.MultiplySigned576ToSigned704(
                lever.ZNumerator,
                coefficientZ));
    }

    private static void GetCrossProductNumerators(
        Signed576 x,
        Signed576 y,
        Signed576 z,
        Vector3d vector,
        out Signed704 crossX,
        out Signed704 crossY,
        out Signed704 crossZ)
    {
        Signed320 vectorX =
            Signed320.ExtendValue(Signed192.Raw(vector.X));
        Signed320 vectorY =
            Signed320.ExtendValue(Signed192.Raw(vector.Y));
        Signed320 vectorZ =
            Signed320.ExtendValue(Signed192.Raw(vector.Z));
        crossX = WideArithmetic.SubtractSigned704(
            WideArithmetic.MultiplySigned576ToSigned704(y, vectorZ),
            WideArithmetic.MultiplySigned576ToSigned704(z, vectorY));
        crossY = WideArithmetic.SubtractSigned704(
            WideArithmetic.MultiplySigned576ToSigned704(z, vectorX),
            WideArithmetic.MultiplySigned576ToSigned704(x, vectorZ));
        crossZ = WideArithmetic.SubtractSigned704(
            WideArithmetic.MultiplySigned576ToSigned704(x, vectorY),
            WideArithmetic.MultiplySigned576ToSigned704(y, vectorX));
    }

    private static Signed832 TransformCrossProduct(
        Signed704 x,
        Signed704 y,
        Signed704 z,
        Fixed64 coefficientX,
        Fixed64 coefficientY,
        Fixed64 coefficientZ) =>
        WideArithmetic.AddSigned832(
            WideArithmetic.AddSigned832(
                WideArithmetic.MultiplySigned704ToSigned832(
                    x,
                    Signed192.Raw(coefficientX)),
                WideArithmetic.MultiplySigned704ToSigned832(
                    y,
                    Signed192.Raw(coefficientY))),
            WideArithmetic.MultiplySigned704ToSigned832(
                z,
                Signed192.Raw(coefficientZ)));

    private static Signed832 GetTransformedCrossProductDenominator(
        in ExactLever3D lever,
        Signed192 divisor)
    {
        Signed320 fixedScaleSquared = WideArithmetic.MultiplySigned192(
            Signed192.One,
            Signed192.One);
        Signed704 transformedDenominator =
            WideArithmetic.MultiplySigned576ToSigned704(
                lever.Denominator,
                fixedScaleSquared);
        Signed320 scaleAndDivisor = WideArithmetic.MultiplySigned192(
            Signed192.One,
            divisor);
        return WideArithmetic.MultiplySigned704ToSigned832(
            transformedDenominator,
            scaleAndDivisor);
    }

    private static bool TryGetScaledComponent(
        Signed832 value,
        Fixed64 firstMultiplier,
        Fixed64 secondMultiplier,
        Signed832 denominator,
        out Fixed64 result) =>
        Fixed64.TryGetSignedRawRatio(
            WideArithmetic.MultiplySigned832(
                WideArithmetic.MultiplySigned832(
                    value,
                    Signed192.Raw(firstMultiplier)),
                Signed192.Raw(secondMultiplier)),
            denominator,
            0,
            out result);

    private static bool TryGetWeightedComponent(
        Signed832 first,
        Fixed64 firstScale,
        Signed832 second,
        Fixed64 secondScale,
        Signed832 third,
        Fixed64 thirdScale,
        Signed832 denominator,
        out Fixed64 result)
    {
        Signed832 numerator = WideArithmetic.AddSigned832(
            WideArithmetic.AddSigned832(
                WideArithmetic.MultiplySigned832(
                    first,
                    Signed192.Raw(firstScale)),
                WideArithmetic.MultiplySigned832(
                    second,
                    Signed192.Raw(secondScale))),
            WideArithmetic.MultiplySigned832(
                third,
                Signed192.Raw(thirdScale)));
        return Fixed64.TryGetSignedRawRatio(
            WideArithmetic.MultiplySigned832(
                numerator,
                Signed192.One),
            denominator,
            0,
            out result);
    }

    private static Signed832 GetQuadraticTerm(
        Signed704 left,
        Signed704 right,
        Fixed64 coefficient)
    {
        if (left.IsZero | right.IsZero | coefficient == Fixed64.Zero)
            return default;

        return WideArithmetic.MultiplySigned704ToSigned832(
            left,
            right,
            Signed192.Raw(coefficient));
    }
}
