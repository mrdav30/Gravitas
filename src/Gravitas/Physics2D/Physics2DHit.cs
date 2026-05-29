using FixedMathSharp;
using Gravitas.Colliders;

namespace Gravitas;

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

    public StiffBody2D? Body { get; }

    public Vector2d Point { get; }

    public Vector2d Normal { get; }

    public Fixed64 Distance { get; }
}
