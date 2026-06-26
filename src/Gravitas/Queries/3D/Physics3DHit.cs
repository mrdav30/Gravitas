//=======================================================================
// Physics3DHit.cs
//=======================================================================
// MIT License, Copyright (c) 2026–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp;
using Gravitas.Colliders;

namespace Gravitas.Queries;

public readonly struct Physics3DHit
{
    public readonly LSCollider? Collider;

    public readonly SolidBody? Body;

    public readonly Vector3d Point;

    public readonly Vector3d Normal;

    public readonly Fixed64 Distance;

    public readonly Vector3d Direction;

    public Physics3DHit(LSCollider? collider, Vector3d point, Vector3d normal, Fixed64 distance, Vector3d direction)
    {
        Collider = collider;
        Body = collider?.Body;
        Point = point;
        Normal = normal;
        Distance = distance;
        Direction = direction;
    }
}
