using FixedMathSharp;
using Gravitas.Colliders;

namespace Gravitas.CollisionHandling;

/// <summary>
/// Represents a body involved in a collision response, containing necessary information for collision resolution.
/// </summary>
internal readonly struct ResponseBody
{
    private ResponseBody(StiffBody body, Fixed64 inverseMass, Fixed3x3 inverseInertiaTensor)
    {
        Body = body;
        InverseMass = inverseMass;
        InverseInertiaTensor = inverseInertiaTensor;
    }

    public StiffBody Body { get; }

    public Fixed64 InverseMass { get; }

    public Fixed3x3 InverseInertiaTensor { get; }

    public bool CanMove => InverseMass > Fixed64.Zero;

    public bool CanRotate => CanMove && !Body.AngularForcesHalted && !Body.IsKinematic;

    public static ResponseBody Create(LSCollider collider)
    {
        StiffBody body = collider.Body!;
        bool movable = !body.Immovable && !body.IsKinematic;
        Fixed64 inverseMass = movable ? body.InverseMass : Fixed64.Zero;
        Fixed3x3 inverseInertiaTensor = movable && !body.AngularForcesHalted
            ? body.InverseInteriaTensor
            : Fixed3x3.Zero;

        return new ResponseBody(body, inverseMass, inverseInertiaTensor);
    }
}