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

    public bool CanRotate => InverseInertiaTensor != Fixed3x3.Zero;

    public static ResponseBody Create(LSCollider collider)
    {
        StiffBody body = collider.Body!;
        Fixed64 inverseMass = body.EffectiveInverseMass;
        Fixed3x3 inverseInertiaTensor = body.EffectiveInverseInertiaTensor;

        return new ResponseBody(body, inverseMass, inverseInertiaTensor);
    }
}
