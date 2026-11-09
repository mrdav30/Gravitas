//=======================================================================
// ExactLever3D.cs
//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp;
using FixedMathSharp.Geometry;

namespace Gravitas.CollisionHandling;

/// <summary>
/// Represents a rigid-body contact lever without narrowing its point-anchor
/// displacement to the Q32.32 scalar domain.
/// </summary>
internal readonly partial struct ExactLever3D
{
    internal static readonly ExactLever3D Zero =
        new(default, default, default, Signed576.ExtendValue(Signed192.Raw(Fixed64.MinIncrement)));

    private ExactLever3D(
        Signed576 xNumerator,
        Signed576 yNumerator,
        Signed576 zNumerator,
        Signed576 denominator)
    {
        XNumerator = xNumerator;
        YNumerator = yNumerator;
        ZNumerator = zNumerator;
        Denominator = denominator;
    }

    internal Signed576 XNumerator { get; }

    internal Signed576 YNumerator { get; }

    internal Signed576 ZNumerator { get; }

    internal Signed576 Denominator { get; }

    internal static ExactLever3D Create(
        in FixedPointAnchor point,
        in FixedPointAnchor center)
    {
        // Shared frame and additive terms cancel before exact rotation.
        if (point.Origin == center.Origin
            && point.Rotation == center.Rotation
            && point.LocalDisplacement == center.LocalDisplacement
            && point.LocalTranslation == center.LocalTranslation
            && point.ExactLocalTerm.Equals(center.ExactLocalTerm))
        {
            var basis = new WideRationalBasis3d(point.Rotation);
            WideAxis3 offset = WideRigidProjection.TransformLocalAxis(basis,
                WideArithmetic.SubtractSigned192(Signed192.Raw(point.LocalPoint.X), Signed192.Raw(center.LocalPoint.X)),
                WideArithmetic.SubtractSigned192(Signed192.Raw(point.LocalPoint.Y), Signed192.Raw(center.LocalPoint.Y)),
                WideArithmetic.SubtractSigned192(Signed192.Raw(point.LocalPoint.Z), Signed192.Raw(center.LocalPoint.Z)));
            return new ExactLever3D(Signed576.ExtendValue(offset.X), Signed576.ExtendValue(offset.Y),
                Signed576.ExtendValue(offset.Z), Signed576.ExtendValue(basis.Denominator));
        }

        WidePointAnchor3d.GetExactRelativeOffsetRatio(
            point.Origin,
            point.Rotation,
            point.LocalPoint,
            point.LocalDisplacement,
            point.LocalTranslation,
            point.ExactLocalTerm,
            center.Origin,
            center.Rotation,
            center.LocalPoint,
            center.LocalDisplacement,
            center.LocalTranslation,
            center.ExactLocalTerm,
            out Signed576 x,
            out Signed576 y,
            out Signed576 z,
            out Signed576 denominator);
        return new ExactLever3D(x, y, z, denominator);
    }

    internal static ExactLever3D CreateXZ(
        in FixedPointAnchor2d point,
        in FixedPointAnchor2d center)
    {
        WidePointAnchor2d.GetExactRelativeOffsetRatio(
            point.Origin,
            point.LocalPoint,
            point.LocalDisplacement,
            point.ExactLocalTerm,
            point.Rotation,
            center.Origin,
            center.LocalPoint,
            center.LocalDisplacement,
            center.ExactLocalTerm,
            center.Rotation,
            out Signed320 x,
            out Signed320 z,
            out Signed320 denominator);
        return new ExactLever3D(
            Signed576.ExtendValue(x),
            default,
            Signed576.ExtendValue(z),
            Signed576.ExtendValue(denominator));
    }
}
