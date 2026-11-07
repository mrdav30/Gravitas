//=======================================================================
// DynamicCcdPlanarBounds.cs
//=======================================================================
// MIT License, Copyright (c) 2026-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp;

namespace Gravitas.CollisionHandling;

internal readonly struct DynamicCcdPlanarBounds
{
    public DynamicCcdPlanarBounds(Fixed64 minX, Fixed64 minZ, Fixed64 maxX, Fixed64 maxZ)
    {
        MinX = minX;
        MinZ = minZ;
        MaxX = maxX;
        MaxZ = maxZ;
    }

    public Fixed64 MinX { get; }
    public Fixed64 MinZ { get; }
    public Fixed64 MaxX { get; }
    public Fixed64 MaxZ { get; }
}
