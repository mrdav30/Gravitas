//=======================================================================
// Physics2DHit.cs
//=======================================================================
// MIT License, Copyright (c) 2026–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp;
using Gravitas.Colliders;

namespace Gravitas.Queries;

/// <summary>
/// Result from a pure 2D query.
/// </summary>
public readonly struct Physics2DHit
{
    public Physics2DHit(LSCollider2D collider, Vector2d point, Vector2d normal, Fixed64 distance)
    {
        Collider = collider;
        Body = collider.Body;
        Point = point;
        Normal = normal;
        Distance = distance;
    }

    public LSCollider2D Collider { get; }

    public SolidBody2D? Body { get; }

    public Vector2d Point { get; }

    public Vector2d Normal { get; }

    public Fixed64 Distance { get; }
}
