//=======================================================================
// JointFrame2D.cs
//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp;

namespace Gravitas.Constraints;

/// <summary>
/// Immutable local anchor frame used by a pure 2D joint.
/// </summary>
public readonly struct JointFrame2D
{
    /// <summary>
    /// Creates a local planar joint frame.
    /// </summary>
    public JointFrame2D(Vector2d anchor, Fixed64 angle)
    {
        Anchor = anchor;
        Angle = angle;
    }

    /// <summary>
    /// Gets the body-local anchor point in the X/Z simulation plane.
    /// </summary>
    public Vector2d Anchor { get; }

    /// <summary>
    /// Gets the body-local scalar frame angle in radians.
    /// </summary>
    public Fixed64 Angle { get; }

    /// <summary>
    /// Gets the identity local frame.
    /// </summary>
    public static JointFrame2D Identity => default;
}
