//=======================================================================
// CollisionDetection.Support.cs
//=======================================================================
// MIT License, Copyright (c) 2026–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp;

namespace Gravitas.CollisionHandling;

internal readonly struct AxisPenetration
{
    public AxisPenetration(Vector3d axis, Fixed64 depth)
    {
        Axis = axis;
        Depth = depth;
        HasValue = true;
    }

    public Vector3d Axis { get; }

    public Fixed64 Depth { get; }

    public bool HasValue { get; }
}
