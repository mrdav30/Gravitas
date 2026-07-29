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
internal readonly struct ExactLever3D
{
    private readonly WideLever3dValue _value;

    private ExactLever3D(in WideLever3dValue value)
    {
        _value = value;
    }

    internal WideLever3dValue Value => _value;

    internal Signed576 XNumerator => _value.XNumerator;

    internal Signed576 YNumerator => _value.YNumerator;

    internal Signed576 ZNumerator => _value.ZNumerator;

    internal Signed576 Denominator => _value.Denominator;

    internal static ExactLever3D Create(
        in FixedPointAnchor point,
        in FixedPointAnchor center) =>
        new(WideLever3d.GetValue(point, center));

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
            new WideLever3dValue(
                Signed576.ExtendValue(x),
                default,
                Signed576.ExtendValue(z),
                Signed576.ExtendValue(denominator)));
    }
}
