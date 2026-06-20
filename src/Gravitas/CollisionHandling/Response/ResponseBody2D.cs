using FixedMathSharp;
using Gravitas.Colliders;

namespace Gravitas.CollisionHandling;

/// <summary>
/// Scalar pure 2D response body data for one collider participant.
/// </summary>
internal readonly struct ResponseBody2D
{
    private ResponseBody2D(StiffBody2D? body, Fixed64 inverseMass, Fixed64 inverseMoment)
    {
        Body = body;
        InverseMass = inverseMass;
        InverseMoment = inverseMoment;
    }

    public StiffBody2D? Body { get; }

    public Fixed64 InverseMass { get; }

    public Fixed64 InverseMoment { get; }

    public bool CanTranslate => Body != null && InverseMass > Fixed64.Zero;

    public bool CanRotate => Body != null && InverseMoment > Fixed64.Zero;

    public static ResponseBody2D Create(LSCollider2D collider)
    {
        StiffBody2D? body = collider.Body;
        if (body == null)
            return default;

        return new ResponseBody2D(
            body,
            body.EffectiveInverseMass,
            body.EffectiveInverseMomentOfInertia);
    }
}
