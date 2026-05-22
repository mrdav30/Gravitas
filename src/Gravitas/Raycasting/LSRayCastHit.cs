using FixedMathSharp;
using Gravitas.Colliders;

namespace Gravitas.Raycasting;

public readonly struct LSRaycastHit
{
    public readonly LSCollider? Collider;

    public readonly StiffBody? Body;

    public readonly Vector3d Point;

    public readonly Vector3d Normal;

    public readonly Fixed64 Distance;

    public readonly Vector3d Direction;

    public LSRaycastHit(LSCollider? collider, Vector3d point, Vector3d normal, Fixed64 distance, Vector3d direction)
    {
        Collider = collider;
        Body = collider?.Body;
        Point = point;
        Normal = normal;
        Distance = distance;
        Direction = direction;
    }
}
